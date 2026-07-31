/**
 * SPI Bus Sniffer Pico (spi_sniffer_pio)
 * Passive SPI Bus sniffer utilizing the RP2040/RP2350 PIO blocks.
 * (C) Juan Schiavoni 2021-2026
 *
 * Utilizes 3 state machines working concurrently via internal loopback 
 * signaling. Core 0 performs high-speed non-blocking extraction of the 
 * PIO FIFO into a 40K RAM FIFO. Core 1 decodes and streams the output 
 * via USB CDC using an optimized 256-byte deinterleaving lookup table.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "spi_sniffer.pio.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "hardware/irq.h"
#include "tusb.h"

#include "ram_fifo.h"

#undef PRINT_VAL
#undef PRINT_TIME_T
#undef PRINT_HEX_INDEX

// ==============================================================================
// GLOBAL HARDWARE CONFIGURATION PROFILE (Centralized Control Macros)
// ==============================================================================
#define TARGET_BUS_PIRATE_5          0  // 1: Bus Pirate 5 Hardware, 0: Standard Raspberry Pi Pico

#if TARGET_BUS_PIRATE_5
    #define SPI_CS_PIN               8  // Physical CS            BP5->(IO0) 
    #define SPI_CLK_PIN              9  // Physical CLK           BP5->(IO1)
    #define SPI_MOSI_PIN             10 // Physical MOSI          BP5->(IO2)
    #define SPI_MISO_PIN             11 // Physical MISO          BP5->(IO3)
    #define SPI_EV0_PIN              12 // Virtual loopback 0     BP5->(I04) (Must immediately follow MISO)
    #define SPI_EV1_PIN              13 // Virtual loopback 1     BP5->(IO5)
    #define SPI_TAP_ENABLE_PIN       14 // Enable capture         BP5->(IO6)
    #define BP5_BUFDIR0_PIN          0 
    #define BP5_BUFDIR1_PIN          1
    #define BP5_BUFDIR2_PIN          2
    #define BP5_BUFDIR3_PIN          3
    #define BP5_BUFDIR4_PIN          4
    #define BP5_BUFDIR5_PIN          5
    #define BP5_BUFDIR6_PIN          6
    #define BP5_BUFDIR7_PIN          7
    #undef LED_PIN
#else
    #define SPI_CS_PIN               0  // Custom layout for regular Pico
    #define SPI_CLK_PIN              1  
    #define SPI_MOSI_PIN             2  
    #define SPI_MISO_PIN             3  
    #define SPI_EV0_PIN              4  
    #define SPI_EV1_PIN              5  
    #define SPI_TAP_ENABLE_PIN       6  // Enable capture when HIGH
    #define LED_PIN                  25
#endif

// The bridge is powered whenever the tap is, so there is nothing to gate on.
// Disabled so that GP6 needs no wire: the gate defaults to enabled through an
// internal pull-up, and a floating pin beside a switching bus can glitch it.
#define SPI_TAP_ENABLE_PIN_CONFIG    0  // 1: Enable dynamic hardware gating via monitoring pin, 0: Disabled

// --- Signals this project adds on top of upstream's four -------------------
// GP4 and GP5 carry the EV0/EV1 loopback and GP6 is the disabled gate, so the
// first pin free of upstream's block is GP7.
#define TAP_DIO0_PIN                 7  // Radio DIO0, an input. Logic8 CH4.
#define TAP_RESET_PIN                8  // Bridge RST, driven open-drain.

// How long the bridge's reset is held low. The bridge re-runs radio init at
// tick ~1.73 s and beacons within 5 ms of it (docs/research/beacon.md), so
// the pulse only has to be long enough to be seen.
#define TAP_RESET_HOLD_MS            100

// The PIO pushes 0b00 for data, 0b01 for a CS fall and 0b11 for a CS rise.
// 0b10 is the one code it never produces, so core 0 uses it to mark a DIO0
// edge and core 1 dispatches all four the same way.
#define EV_DIO0                      0x02
#define EV_DIO0_RISING_BIT           (1u << 0)

// Sniffer Output Format Settings
// 0: Traditional Verbose Mode (S[88-00][00-44]P\r\n) -> ~18 bytes per frame
// 1: Optimized Compact Mode (S88000044P\n) -> ~11 bytes per frame
#define SNIFFER_COMPACT_MODE         1  // 0: Verbose Mode, 1: Compact Parallel Hex Stream

// --- Temporary instrumentation ---------------------------------------------
// Counters that diagnose where the tap loses records. Set to 0 for a capture
// build. The gate exists because the probes are not free: the two gap probes
// read time_us_64() on every pass of each core's loop, and the ring level
// probe adds a call for every word. A capture build must not pay that, and
// the measurement it disturbs is its own.
//
// The permanent loss markers are NOT behind this gate. A capture build still
// writes '!' for a truncated transaction and 'X' for a lost record, because
// the host has to tell an idle bus from a lost one either way.
//
// This replaces upstream's SNIFFER_TELEMETRY, which wrote prose into the
// middle of the wire format and left two of its four counters dead.
#define TAP_TELEMETRY                1
#define TAP_TELEMETRY_PERIOD_US      10000000ull

#define RAM_FIFO_SIZE                40000

bool ram_fifo_overflow = false;

// Non-zero while the bridge's reset is held low. Core 0 releases it.
static volatile uint64_t reset_deadline_us = 0;

// DIO0 edges that arrived part-way through a transaction, held until its
// line is closed. Eight is far more than one transaction can collect: a
// transaction runs about 5 us and DIO0 moves a few times a second.
#define MAX_PENDING_DIO0 8
static struct { uint64_t stamp; bool rising; } pending_dio0[MAX_PENDING_DIO0];
static uint32_t pending_dio0_count = 0;
static bool in_transaction = false;

// --- Permanent loss markers ------------------------------------------------
// A dropped record has to reach the host, or a gap reads as an idle bus and a
// discarded transaction becomes a false null. docs/plans/raw-frame-buffer.md
// sets the same rule for the receiver.
//
// 'txn_lost_data' covers the case the host cannot otherwise see: a dropped
// data word closes its transaction normally, so a 26-byte beacon that lost
// four bytes frames as a valid 22-byte transaction. Core 1 writes '!' before
// the closing 'P' and the host discards that transaction.
//
// 'records_lost' covers a dropped START or STOP, which leaves no line to mark,
// and a buffer that buff_send() discarded with no host attached. Core 1 writes
// an 'X' line at the next transaction boundary.
static volatile bool txn_lost_data = false;
static volatile bool records_lost = false;

#if TAP_TELEMETRY
static uint32_t tlm_ring_max = 0;
static uint32_t tlm_ring_ovf = 0;
static uint32_t tlm_pio_stall = 0;
static uint32_t tlm_usb_stall = 0;
static uint32_t tlm_cdc_disc = 0;
static uint32_t tlm_dio0_drop = 0;
static uint32_t tlm_c0_gap_us = 0;
static uint32_t tlm_c1_gap_us = 0;
static uint64_t tlm_next_report_us = 0;
static volatile bool tlm_request = false;

#define TLM_INC(counter)      do { (counter)++; } while (0)
#define TLM_MAX(counter, val) do {                     \
        uint32_t tlm_v_ = (val);                       \
        if (tlm_v_ > (counter)) { (counter) = tlm_v_; } \
    } while (0)
#else
// The argument is never expanded, so a gated build neither declares the
// counter nor evaluates the expression that would have fed it.
#define TLM_INC(counter)      do { } while (0)
#define TLM_MAX(counter, val) do { } while (0)
#endif

#define ASCII_BUFF_SIZE 1024
static char ascii_buff[ASCII_BUFF_SIZE];
static uint32_t ascii_index = 0;

static uint8_t mosi_miso_lut[256];

static PIO pio_sniffer = pio0; 
static uint sm_main;
static uint sm_data;
static uint sm_start; 

/**
 * @brief Pre-calculates the deinterleaving values into a 256-byte static LUT.
 *
 * Maps an 8-bit interleaved sequence [M3 O3 M2 O2 M1 O1 M0 O0]
 * directly into separate 4-bit nibbles: MISO in high nibble, MOSI in low nibble.
 */
void init_decoding_lut(void) {
    for (int i = 0; i < 256; i++) {
        uint8_t mosi_nibble = 0;
        uint8_t miso_nibble = 0;
        
        mosi_nibble |= (i & 0x01) ? 1 : 0;
        mosi_nibble |= (i & 0x04) ? 2 : 0;
        mosi_nibble |= (i & 0x10) ? 4 : 0;
        mosi_nibble |= (i & 0x40) ? 8 : 0;
        
        miso_nibble |= (i & 0x02) ? 1 : 0;
        miso_nibble |= (i & 0x08) ? 2 : 0;
        miso_nibble |= (i & 0x20) ? 4 : 0;
        miso_nibble |= (i & 0x80) ? 8 : 0;
        
        mosi_miso_lut[i] = (miso_nibble << 4) | mosi_nibble;
    }
}

static inline char nibble_to_hex(uint8_t nibble) {
    nibble &= 0x0F;
    if (nibble > 9) {
        nibble += 'A' - '0' - 10;
    }
    return (nibble + '0');
}

/**
 * @brief Transmits the accumulated text buffer over USB CDC using Bulk transfers.
 * Handles partial writes safely via memory shifts to maintain protocol stream integrity.
 */
void buff_send(void) {
    if (ascii_index > 0) {
        if (tud_cdc_connected()) {
            uint32_t written = tud_cdc_write(ascii_buff, ascii_index);
            tud_cdc_write_flush();
            
            if (written < ascii_index) {
                TLM_INC(tlm_usb_stall);
                uint32_t remaining = ascii_index - written;
                memmove(ascii_buff, &ascii_buff[written], remaining);
                ascii_index = remaining;
            } else {
                ascii_index = 0;
            }
        } else {
            // No host, so the buffer goes nowhere. What it held is lost, and
            // it was almost certainly a part line. Mark it, so the host reads
            // an 'X' as soon as it attaches again.
            TLM_INC(tlm_cdc_disc);
            records_lost = true;
            ascii_index = 0;
        }
    }
}

/**
 * @brief High-speed non-blocking character insertion into the local text buffer.
 * Triggers an emergency data flush and applies backpressure if allocation limits are reached.
 */
static inline void buff_putchar(char c) {
    if (ascii_index >= ASCII_BUFF_SIZE - 1) {
        buff_send();
        
        while (ascii_index >= ASCII_BUFF_SIZE - 1) {
            tud_task();
            buff_send();
        }
    } 
    ascii_buff[ascii_index++] = c;
}

#if TAP_TELEMETRY
/**
 * @brief Appends an unpadded decimal uint32 to the stream.
 *
 * The division is a software routine on this part, so this runs only when a
 * counter record goes out. No loop that touches the bus calls it.
 */
static void buff_put_u32_dec(uint32_t val) {
    char tmp[10];
    int len = 0;
    do {
        tmp[len++] = '0' + (val % 10);
        val /= 10;
    } while (val);
    while (len--) {
        buff_putchar(tmp[len]);
    }
}
#endif

/**
 * @brief High-performance helper to append an 8-digit hexadecimal uint32 to the stream.
 */
static inline void buff_put_u32_hex8(uint32_t val) {
    for (int i = 7; i >= 0; i--) {
        buff_putchar(nibble_to_hex(val >> (i * 4)));
    }
    buff_putchar(' ');
}

/**
 * @brief Appends a 64-bit value as 16 hexadecimal characters.
 *
 * Fixed width so the host can slice a record without a delimiter: a
 * transaction is 'S', sixteen characters, four per SPI byte, sixteen more,
 * then 'P'.
 */
static inline void buff_put_u64_hex16(uint64_t val) {
    for (int i = 15; i >= 0; i--) {
        buff_putchar(nibble_to_hex((uint8_t)(val >> (i * 4))));
    }
}

/**
 * @brief Reads the two timestamp words that core 0 pushed after an event.
 *
 * Core 0 writes the triple only when the ring has room for all three, so the
 * words are always present. The wait covers the microscopic window where core
 * 1 catches up mid-push.
 */
static inline uint64_t read_stamp(void) {
    while (ram_fifo_is_empty()) {
        tight_loop_contents();
    }
    uint64_t hi = ram_fifo_get();
    while (ram_fifo_is_empty()) {
        tight_loop_contents();
    }
    return (hi << 32) | ram_fifo_get();
}

/**
 * @brief Writes one DIO0 edge record on its own line.
 */
static inline void emit_dio0(bool rising, uint64_t stamp) {
    buff_putchar('D');
    buff_putchar(rising ? 'R' : 'F');
    buff_put_u64_hex16(stamp);
    buff_putchar('\r');
    buff_putchar('\n');
}

/**
 * @brief Helper function to write a null-terminated string into the optimized buffer.
 */
static inline void buff_putstring(const char *s) {
    while (*s) {
        buff_putchar(*s++);
    }
}

/**
 * @brief Writes the lost-record marker, if anything went missing.
 *
 * Call this only between transactions. An 'X' inside an 'S...P' line would
 * corrupt the line it interrupts.
 */
static inline void emit_lost_marker(void) {
    if (!records_lost) {
        return;
    }
    records_lost = false;
    buff_putchar('X');
    buff_put_u64_hex16(time_us_64());
    buff_putchar('\r');
    buff_putchar('\n');
}

#if TAP_TELEMETRY
/**
 * @brief Writes the counter record.
 *
 * One line, and the host parses it by name. The counters never reset, so the
 * host takes differences between two records.
 */
static void emit_counters(void) {
    buff_putchar('C');
    buff_put_u64_hex16(time_us_64());
    buff_putstring(" ringmax=");  buff_put_u32_dec(tlm_ring_max);
    buff_putstring(" ringovf=");  buff_put_u32_dec(tlm_ring_ovf);
    buff_putstring(" piostall="); buff_put_u32_dec(tlm_pio_stall);
    buff_putstring(" usbstall="); buff_put_u32_dec(tlm_usb_stall);
    buff_putstring(" cdcdisc=");  buff_put_u32_dec(tlm_cdc_disc);
    buff_putstring(" c0gap=");    buff_put_u32_dec(tlm_c0_gap_us);
    buff_putstring(" c1gap=");    buff_put_u32_dec(tlm_c1_gap_us);
    buff_putstring(" dio0drop="); buff_put_u32_dec(tlm_dio0_drop);
    buff_putchar('\r');
    buff_putchar('\n');
    buff_send();
}

/**
 * @brief Writes the counter record when it is due, or when the host asked.
 *
 * Call this only between transactions, for the same reason as the marker.
 */
static inline void telemetry_service(void) {
    uint64_t now = time_us_64();
    if (!tlm_request && now < tlm_next_report_us) {
        return;
    }
    tlm_request = false;
    tlm_next_report_us = now + TAP_TELEMETRY_PERIOD_US;
    emit_counters();
}
#endif

/**
 * @brief Pushes an event word and its 64-bit timestamp as one unit.
 *
 * Core 1 reads the two timestamp words by position, so a partial push would
 * put it permanently out of step. The room check makes the triple atomic:
 * either all three go in, or none does and the record is dropped whole.
 */
static inline void push_stamped(uint32_t word, uint64_t stamp) {
    if (ram_fifo_get_level() + 3 > RAM_FIFO_SIZE) {
        TLM_INC(tlm_ring_ovf);
        // A dropped START or STOP leaves no line for core 1 to mark, so the
        // loss goes out as its own 'X' record instead.
        records_lost = true;
        ram_fifo_overflow = true;
        return;
    }
    ram_fifo_set(word);
    ram_fifo_set((uint32_t)(stamp >> 32));
    ram_fifo_set((uint32_t)stamp);
}

/**
 * @brief Releases the bridge's reset once the hold has elapsed.
 *
 * Timed against the clock rather than held with a sleep, so core 0 keeps
 * draining the PIO FIFO throughout. The point of the reset is to capture the
 * bridge coming back, which means not going blind while it goes down.
 */
static inline void service_reset(void) {
    if (!reset_deadline_us) {
        return;
    }
    if (time_us_64() >= reset_deadline_us) {
        // Back to an input, which is the released state: the bridge's own
        // pull-up takes the line high and the Pico stops driving it.
        gpio_set_dir(TAP_RESET_PIN, GPIO_IN);
        reset_deadline_us = 0;
    }
}

/**
 * @brief Asserts the bridge's reset, open-drain.
 */
static void assert_reset(void) {
    gpio_put(TAP_RESET_PIN, 0);
    gpio_set_dir(TAP_RESET_PIN, GPIO_OUT);
    reset_deadline_us = time_us_64() + (TAP_RESET_HOLD_MS * 1000ull);
}

/**
 * @brief Reads host commands from USB CDC.
 *
 * The Pi has no picotool and no Pico toolchain, so without this the board
 * needs a physical BOOTSEL press for every reflash. 'B' reboots into the
 * mass-storage bootloader, which makes a reflash a file copy over SSH.
 */
static inline void poll_host_commands(void) {
    if (!tud_cdc_available()) {
        return;
    }

    uint8_t cmd;
    if (tud_cdc_read(&cmd, 1) != 1) {
        return;
    }

    if (cmd == 'B') {
        reset_usb_boot(0, 0);   // Does not return.
    } else if (cmd == 'R') {
        assert_reset();
    }
#if TAP_TELEMETRY
    else if (cmd == 'T') {
        // Core 1 writes the record at the next transaction boundary. Writing
        // it here would cut into whatever line is open.
        tlm_request = true;
    }
#endif
}

/**
 * @brief Core 1 Processing Loop.
 * Extracts captures directly from the RAM FIFO, detects control/data phases,
 * performs LUT-accelerated decoding, and packages data into the streaming text buffer.
 */
void core1_print() {
    uint32_t val;
    uint32_t last_data_time = time_us_32();
    bool buffer_dirty = false;
#ifdef PRINT_HEX_INDEX
    uint32_t capture_index = 0;
#endif
    
    init_decoding_lut();

    // Core 1 services USB, so it owns USBCTRL_IRQ. Both cores share one
    // vector table, so the handler tusb_init() installed serves core 1
    // unchanged.
    irq_set_enabled(USBCTRL_IRQ, true);

#if TAP_TELEMETRY
    uint64_t c1_last_us = time_us_64();
#endif

    while (true) {
#if TAP_TELEMETRY
        // The period of this loop. Its longest value is how long core 1 stopped
        // consuming, which is the stall this instrumentation exists to find.
        uint64_t c1_now = time_us_64();
        TLM_MAX(tlm_c1_gap_us, (uint32_t)(c1_now - c1_last_us));
        c1_last_us = c1_now;
#endif
        tud_task();
        poll_host_commands();

        if (ram_fifo_is_empty()) {
            if (buffer_dirty && (time_us_32() - last_data_time > 200)) {
                buff_send();
                buffer_dirty = false;
            }
            // An idle bus is the safest boundary there is: no line is open,
            // so the marker and the counter record cannot cut into one.
            if (!in_transaction) {
                emit_lost_marker();
#if TAP_TELEMETRY
                telemetry_service();
#endif
            }
            continue;
        }

        val = ram_fifo_get();
        last_data_time = time_us_32();
        buffer_dirty = true;

#ifdef LED_PIN
        gpio_put(LED_PIN, false);
#endif
        uint32_t ev_code = ((val >> 18) & 0x03);

        if (ev_code == EV_START) {
            buff_putchar('S');
            buff_put_u64_hex16(read_stamp());
            in_transaction = true;
        } else if (ev_code == EV_DIO0) {
            // A DIO0 edge lands whenever the radio raises the pin, which is
            // often part-way through a transaction. Writing it out there
            // would split the transaction's line across the 'D' record and
            // corrupt both. Hold it and emit it at the next boundary. The
            // stamp was taken on core 0 at the edge, so deferring the write
            // costs nothing but ordering.
            uint64_t stamp = read_stamp();
            bool rising = (val & EV_DIO0_RISING_BIT) != 0;
            if (in_transaction) {
                if (pending_dio0_count < MAX_PENDING_DIO0) {
                    pending_dio0[pending_dio0_count].stamp = stamp;
                    pending_dio0[pending_dio0_count].rising = rising;
                    pending_dio0_count++;
                } else {
                    TLM_INC(tlm_dio0_drop);
                    records_lost = true;
                }
            } else {
                emit_dio0(rising, stamp);
            }
        } else if (ev_code == EV_STOP) {
            buff_put_u64_hex16(read_stamp());
            // A transaction that lost a data word still closes cleanly, so
            // without this marker a 26-byte beacon that lost four bytes reads
            // as a valid 22-byte transaction. The host discards a line that
            // carries '!' rather than trusting its length.
            if (txn_lost_data) {
                txn_lost_data = false;
                buff_putchar('!');
            }
            buff_putchar('P');
            buff_putchar('\r');
            buff_putchar('\n');
            in_transaction = false;
            for (uint32_t i = 0; i < pending_dio0_count; i++) {
                emit_dio0(pending_dio0[i].rising, pending_dio0[i].stamp);
            }
            pending_dio0_count = 0;

            // The line is closed, so this is a safe place to write both.
            emit_lost_marker();
#if TAP_TELEMETRY
            telemetry_service();
#endif
        } else if (ev_code == EV_DATA) {
            uint16_t raw_word = val & 0xFFFF;
            uint8_t low_half  = raw_word & 0xFF;
            uint8_t high_half = (raw_word >> 8) & 0xFF;
            
            uint8_t dec_low  = mosi_miso_lut[low_half];
            uint8_t dec_high = mosi_miso_lut[high_half];
            
            uint8_t mosi = (dec_low & 0x0F) | ((dec_high & 0x0F) << 4);
            uint8_t miso = ((dec_low & 0xF0) >> 4) | (dec_high & 0xF0);
#if SNIFFER_COMPACT_MODE
            buff_putchar(nibble_to_hex(mosi >> 4));
            buff_putchar(nibble_to_hex(mosi));
            buff_putchar(nibble_to_hex(miso >> 4));
            buff_putchar(nibble_to_hex(miso));
#else
            buff_putchar('[');
            buff_putchar(nibble_to_hex(mosi >> 4));
            buff_putchar(nibble_to_hex(mosi));
            buff_putchar('-');
            buff_putchar(nibble_to_hex(miso >> 4));
            buff_putchar(nibble_to_hex(miso));
            buff_putchar(']');
#endif
        } else {
            buff_putstring("\r\n[RAW_U:0x");
            buff_put_u32_hex8(val);
            buff_putstring("]\r\n");
        }

#ifdef LED_PIN
        if (!ram_fifo_overflow) {
            gpio_put(LED_PIN, true);
        }
#endif
    }
}

/**
 * @brief Hardware GPIO IRQ callback to monitor target power state pin.
 */
void spi_tap_enable_callback(uint gpio, uint32_t events) {
    if (gpio == SPI_TAP_ENABLE_PIN) { 
        if (events & GPIO_IRQ_EDGE_FALL) {
            pio_sm_set_enabled(pio_sniffer, sm_main, false);
        } 
        else if (events & GPIO_IRQ_EDGE_RISE) {
            // Target reader woke up: Clear residual RX noise and enable PIO execution
            pio_sm_clear_fifos(pio_sniffer, sm_main);
            pio_sm_set_enabled(pio_sniffer, sm_main, true);
        }
    }
}

/**
 * @brief Initializes the target tracking input pin and configures edge-triggered interrupts.
 */
void setup_spi_tap_enable_pin() {
    gpio_init(SPI_TAP_ENABLE_PIN);
    gpio_set_dir(SPI_TAP_ENABLE_PIN, GPIO_IN);
    gpio_pull_up(SPI_TAP_ENABLE_PIN); 

    gpio_set_irq_enabled_with_callback(
        SPI_TAP_ENABLE_PIN, 
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
        true, 
        &spi_tap_enable_callback
    );
}

/**
 * @brief Configures global hardware pins and Bus Pirate v5 logic buffers.
 */
void init_pins(void) {
    uint spi_pins[] = {
        SPI_CS_PIN, 
        SPI_CLK_PIN, 
        SPI_MOSI_PIN, 
        SPI_MISO_PIN, 
        SPI_EV0_PIN, 
        SPI_EV1_PIN, 
    };

    // Configure all tap pins as clean digital inputs
    for (size_t i = 0; i < sizeof(spi_pins) / sizeof(spi_pins[0]); i++) {
        gpio_init(spi_pins[i]);
        gpio_set_dir(spi_pins[i], GPIO_IN);
        gpio_disable_pulls(spi_pins[i]); 
    }

    // CRITICAL: Internal Pull-Up on CS to prevent false PRG1 start triggers from idle noise
    gpio_pull_up(SPI_CS_PIN);

    // DIO0 is a passive input like the four above.
    gpio_init(TAP_DIO0_PIN);
    gpio_set_dir(TAP_DIO0_PIN, GPIO_IN);
    gpio_disable_pulls(TAP_DIO0_PIN);

    // The bridge's reset is driven open-drain: an input while released, and
    // an output only while it is held low. The Pico therefore never fights
    // the bridge's pull-up and never sources current into the bridge, which
    // also means an unpowered Pico cannot hold the bridge in reset.
    gpio_init(TAP_RESET_PIN);
    gpio_set_dir(TAP_RESET_PIN, GPIO_IN);
    gpio_disable_pulls(TAP_RESET_PIN);
    gpio_put(TAP_RESET_PIN, 0);

#if TARGET_BUS_PIRATE_5    
    uint bp5_pins[] = {
        BP5_BUFDIR0_PIN, 
        BP5_BUFDIR1_PIN, 
        BP5_BUFDIR2_PIN, 
        BP5_BUFDIR3_PIN, 
        BP5_BUFDIR4_PIN, 
        BP5_BUFDIR5_PIN, 
        BP5_BUFDIR6_PIN,
        BP5_BUFDIR7_PIN,
    };
    
    // Configure Bus Pirate level shifter buffer direction pins as outputs driven low
    for (size_t i = 0; i < sizeof(bp5_pins) / sizeof(bp5_pins[0]); i++) {
        gpio_init(bp5_pins[i]);
        gpio_set_dir(bp5_pins[i], GPIO_OUT);
        gpio_put(bp5_pins[i], 0); 
    }
#endif

#ifdef LED_PIN
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
#endif
}

int main() {
    float div = 1.0f;

    init_pins();
    
    stdio_init_all();

    // Core 0 runs the PIO drain loop and does not service USB.
    // stdio_init_all() enables USBCTRL_IRQ on the core that calls it, and
    // core 1 enables it for itself.
    irq_set_enabled(USBCTRL_IRQ, false);

#if SPI_TAP_ENABLE_PIN_CONFIG
    setup_spi_tap_enable_pin();
#endif

    // Synchronously spin up execution state machines with clean configurations
    sm_main = pio_claim_unused_sm(pio_sniffer, true);
    uint offset_main = pio_add_program(pio_sniffer, &spi_main_program);
    spi_main_program_init(pio_sniffer, sm_main, offset_main, div, SPI_MOSI_PIN, SPI_EV0_PIN);

    sm_data = pio_claim_unused_sm(pio_sniffer, true);
    uint offset_data = pio_add_program(pio_sniffer, &spi_data_program);
    // Corrected call signature: match spi_data_program_init in spi_sniffer.pio
    spi_data_program_init(pio_sniffer, sm_data, offset_data, div, SPI_CLK_PIN, SPI_CS_PIN);

    sm_start = pio_claim_unused_sm(pio_sniffer, true);
    uint offset_start = pio_add_program(pio_sniffer, &spi_start_program);
    spi_start_program_init(pio_sniffer, sm_start, offset_start, div, SPI_CS_PIN, SPI_EV0_PIN);

    if (!ram_fifo_init(RAM_FIFO_SIZE)) {
        while (true);
    }

    pio_sm_set_enabled(pio_sniffer, sm_main, true);
    pio_sm_set_enabled(pio_sniffer, sm_start, true);
    pio_sm_set_enabled(pio_sniffer, sm_data, true);

    multicore_launch_core1(core1_print);

#ifdef LED_PIN    
    gpio_put(LED_PIN, true);
#endif

    bool dio0_last = gpio_get(TAP_DIO0_PIN);

    // The PIO sets this bit when its receive FIFO overran, which means the
    // state machine discarded bytes. Core 0 owns that FIFO, so core 0 reads
    // the flag. Upstream read it once for every word consumed, on core 1,
    // which put a register read on the decode path and cleared a core 0
    // condition from the wrong core.
    const uint32_t stall_mask = 1u << (PIO_FDEBUG_RXSTALL_LSB + sm_main);
    bool drained = false;

#if TAP_TELEMETRY
    uint64_t c0_last_us = time_us_64();
#endif

    while (true) {
#if TAP_TELEMETRY
        // The period of this loop. Its longest value is how long core 0 stopped
        // draining, which is what a PIO receive overrun needs in order to happen.
        uint64_t c0_now = time_us_64();
        TLM_MAX(tlm_c0_gap_us, (uint32_t)(c0_now - c0_last_us));
        c0_last_us = c0_now;
#endif
        while (pio_sm_get_rx_fifo_level(pio_sniffer, sm_main) > 0) {
            drained = true;
            uint32_t capture_val = pio_sm_get(pio_sniffer, sm_main);

            // The stamp has to be taken here, where the word leaves the PIO
            // FIFO. Upstream stamps on core 1, after a 40,000-word ring that
            // core 1 drains at whatever rate the USB stack allows, so its
            // stamp measures when core 1 got to the word rather than when CS
            // fell. This loop does nothing else, so the delay from the edge
            // is a microsecond or two against a budget of about 60 us.
            uint32_t ev_code = (capture_val >> 18) & 0x03;
            if (ev_code == EV_START || ev_code == EV_STOP) {
                push_stamped(capture_val, time_us_64());
            } else if (!ram_fifo_set(capture_val)) {
                TLM_INC(tlm_ring_ovf);
                // The transaction in progress lost a byte. Core 1 marks its
                // line with '!' so the host does not read a short transaction
                // as a complete one.
                txn_lost_data = true;
                ram_fifo_overflow = true;
            }

            TLM_MAX(tlm_ring_max, ram_fifo_get_level());
        }

        // Check the overrun flag only after a drain pass moved something. An
        // overrun cannot happen while the FIFO is idle, and this keeps the
        // register read off the spin that runs when the bus is quiet.
        if (drained) {
            drained = false;
            if (pio_sniffer->fdebug & stall_mask) {
                pio_sniffer->fdebug = stall_mask;   // Write one to clear.
                TLM_INC(tlm_pio_stall);
                // The PIO discarded bytes, so a transaction lost content or a
                // boundary went missing. The host has to know either way.
                records_lost = true;
            }
        }

        // DIO0 is polled rather than given its own state machine. This loop
        // spins on a FIFO level read, so it comes back around well inside a
        // DIO0 pulse; a state machine would be more robust but adds a second
        // mechanism for a signal that moves a few times per second.
        bool dio0_now = gpio_get(TAP_DIO0_PIN);
        if (dio0_now != dio0_last) {
            dio0_last = dio0_now;
            push_stamped((EV_DIO0 << 18) | (dio0_now ? EV_DIO0_RISING_BIT : 0),
                         time_us_64());
        }

        service_reset();
    }
}
