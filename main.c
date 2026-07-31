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
#include "hardware/irq.h"
#include "tusb.h"

#include "ram_fifo.h"

#undef PRINT_VAL
#undef PRINT_TIME_T
#undef PRINT_HEX_INDEX

// ==============================================================================
// GLOBAL HARDWARE CONFIGURATION PROFILE (Centralized Control Macros)
// ==============================================================================
#define TARGET_BUS_PIRATE_5          1  // 1: Bus Pirate 5 Hardware, 0: Standard Raspberry Pi Pico

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

#define SPI_TAP_ENABLE_PIN_CONFIG    1  // 1: Enable dynamic hardware gating via monitoring pin, 0: Disabled

// Sniffer Output Format Settings
// 0: Traditional Verbose Mode (S[88-00][00-44]P\r\n) -> ~18 bytes per frame
// 1: Optimized Compact Mode (S88000044P\n) -> ~11 bytes per frame
#define SNIFFER_COMPACT_MODE         1  // 0: Verbose Mode, 1: Compact Parallel Hex Stream

// Monitor telemetry to diagnose whether USB communication has sufficient 
// speed to transmit frames without collapsing the 40K RAM buffer.
// 1: Activated for diagnostics, 0: Deactivated for clean production
// In bash : tio -b 115200 /dev/ttyACM0 -L --log-file sniff.txt
// Search: grep -A 5 "Sniffer Telemetry Report" sniff.txt
// output: --- Sniffer Telemetry Report ---
//         RAM FIFO High-Water Mark: 0000000056 / 40000
//         RAM FIFO Total Overflows: 0000000000 
//         USB CDC Engine Stalls:    0000000000 
//         -------------------------------
#define SNIFFER_TELEMETRY            0  
#define SNIFFER_TELEMETRY_FRAMES     200

#define RAM_FIFO_SIZE                40000

bool ram_fifo_overflow = false;

static uint32_t fifo_max_level = 0;
static uint32_t fifo_overflow_counter = 0;
static uint32_t usb_stall_counter = 0;
static uint32_t transaction_counter = 0;
static uint32_t pio_rx_stall_counter = 0;

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
                uint32_t remaining = ascii_index - written;
                memmove(ascii_buff, &ascii_buff[written], remaining);
                ascii_index = remaining;
            } else {
                ascii_index = 0;
            }
        } else {
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

/**
 * @brief High-performance helper to append a 10-digit decimal uint32 to the stream.
 */
static inline void buff_put_u32_dec10(uint32_t val) {
    char tmp[10];
    for (int i = 9; i >= 0; i--) {
        tmp[i] = '0' + (val % 10);
        val /= 10;
    }
    for (int i = 0; i < 10; i++) {
        buff_putchar(tmp[i]);
    }
    buff_putchar(' ');
}

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
 * @brief Helper function to write a null-terminated string into the optimized buffer.
 */
static inline void buff_putstring(const char *s) {
    while (*s) {
        buff_putchar(*s++);
    }
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

    while (true) {
        tud_task();

        if (ram_fifo_is_empty()) {
            if (buffer_dirty && (time_us_32() - last_data_time > 200)) {
                buff_send();
                buffer_dirty = false;
            }
            continue;
        }

        val = ram_fifo_get();
        last_data_time = time_us_32();
        buffer_dirty = true;

#ifdef LED_PIN
        gpio_put(LED_PIN, false);
#endif          
        // 1. Verify if RX FIFO stall occurred on the main state machine
        uint32_t stall_mask = 1u << (PIO_FDEBUG_RXSTALL_LSB + sm_main);
    
        if (pio_sniffer->fdebug & stall_mask) {
            pio_rx_stall_counter++;
        
            // Clear flag by writing '1' (W1C) for the next observation window
            pio_sniffer->fdebug = stall_mask; 
        }
        
        uint32_t ev_code = ((val >> 18) & 0x03);

        if (ev_code == EV_START) {
#if defined(PRINT_TIME_T)
            buff_put_u32_dec10(time_us_32());
#elif defined(PRINT_HEX_INDEX)
            buff_put_u32_hex8(capture_index++);
#endif
            buff_putchar('S');
        } else if (ev_code == EV_STOP) {
            buff_putchar('P');
            buff_putchar('\r');
            buff_putchar('\n');

#if SNIFFER_TELEMETRY
            transaction_counter++;
            if (transaction_counter >= SNIFFER_TELEMETRY_FRAMES) {
                transaction_counter = 0;
                
                buff_putstring("\r\n--- Sniffer Telemetry Report ---\r\n");
                buff_putstring("RAM FIFO High-Water Mark: ");
                buff_put_u32_dec10(fifo_max_level);
                buff_putstring("/ 40000\r\n");
                buff_putstring("RAM FIFO Total Overflows: ");
                buff_put_u32_dec10(fifo_overflow_counter);
                buff_putstring("\r\n");
                buff_putstring("PIO RX Hardware Stalls:   ");
                buff_put_u32_dec10(pio_rx_stall_counter);
                buff_putstring("\r\n");
                buff_putstring("USB CDC Engine Stalls:    ");
                buff_put_u32_dec10(usb_stall_counter);
                buff_putstring("\r\n");
                buff_putstring("--------------------------------\r\n");
                
                buff_send(); 
                fifo_max_level = 0;
            }
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

    while (true) {
        while (pio_sm_get_rx_fifo_level(pio_sniffer, sm_main) > 0) {
            uint32_t capture_val = pio_sm_get(pio_sniffer, sm_main);
            
            if (!ram_fifo_set(capture_val)) {
                fifo_overflow_counter++; 
                ram_fifo_overflow = true;
            }
            
            uint32_t current_level = ram_fifo_get_level();
            if (current_level > fifo_max_level) {
                fifo_max_level = current_level;
            }
        }
    }
}
