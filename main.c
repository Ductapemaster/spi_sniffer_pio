/**
 * =============================================================================
 * SPI Bus Sniffer Pico (spi_sniffer_pio)
 * Passive SPI Bus sniffer utilizing the RP2040/RP2350 PIO blocks.
 * (C) Juan Schiavoni 2021-2026
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "spi_sniffer.pio.h"
#include "pico/multicore.h"
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
    #define MY_SPI_CS_PIN            8  // Physical CS pin on Bus Pirate 5
    #define MY_SPI_CLK_PIN           9  // Physical CLK pin on Bus Pirate 5
    #define MY_SPI_MOSI_PIN         10  // Physical MOSI pin (Starts consecutive hardware block)
    #define MY_SPI_MISO_PIN         11  // Physical MISO pin
    #define MY_SPI_EV0_PIN          12  // Virtual loopback pin 0 (Must immediately follow MISO)
    #define MY_SPI_EV1_PIN          13  // Virtual loopback pin 1
    #define BOARD_INIT()            init_bus_pirate_v5_buffers()
#else
    #define MY_SPI_CS_PIN            0  // Custom layout for regular Pico
    #define MY_SPI_CLK_PIN           1  
    #define MY_SPI_MOSI_PIN          2  
    #define MY_SPI_MISO_PIN          3  
    #define MY_SPI_EV0_PIN           4  
    #define MY_SPI_EV1_PIN           5  
    #define BOARD_INIT()            ((void)0) 
#endif

#define SPI_TAP_ENABLE_PIN_CONFIG    1  // 1: Enable dynamic hardware gating via monitoring pin, 0: Disabled
#define SPI_TAP_ENABLE_PIN          14  // Generic configuration pin tracking target operational state

// ==============================================================================
// SYSTEM SETTINGS & TELEMETRY
// ==============================================================================
#define SNIFFER_COMPACT_MODE         1  // 0: Verbose Mode, 1: Compact Parallel Hex Stream
#define SNIFFER_TELEMETRY            0  // 1: Activated diagnostics report, 0: Deactivated clean production

const uint led_pin = 25;
bool ram_fifo_overflow = false;

static uint32_t fifo_max_level = 0;
static uint32_t fifo_overflow_counter = 0;
static uint32_t usb_stall_counter = 0;
static uint32_t transaction_counter = 0;

#define ASCII_BUFF_SIZE 1024
static char ascii_buff[ASCII_BUFF_SIZE];
static uint32_t ascii_index = 0;

static uint8_t mosi_miso_lut[256];

static PIO pio_sniffer = pio0; 
static uint sm_main;

/**
 * @brief Pre-calculates the deinterleaving values into a 256-byte static LUT.
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

static inline void buff_put_u32_hex8(uint32_t val) {
    for (int i = 7; i >= 0; i--) {
        buff_putchar(nibble_to_hex(val >> (i * 4)));
    }
    buff_putchar(' ');
}

static inline void buff_putstring(const char *s) {
    while (*s) {
        buff_putchar(*s++);
    }
}

/**
 * @brief Core 1 Processing Loop.
 */
void core1_print() {
    uint32_t val;
    uint32_t last_data_time = time_us_32();
    bool buffer_dirty = false;
#ifdef PRINT_HEX_INDEX
    uint32_t capture_index = 0;
#endif
    
    init_decoding_lut();

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

        gpio_put(led_pin, false);
        
        uint32_t ev_code = (val & 0x000C0000) ? ((val >> 18) & 0x03) : EV_DATA;

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
            if (transaction_counter >= 20000) {
                transaction_counter = 0;
                
                buff_putstring("\r\n--- Sniffer Telemetry Report ---\r\n");
                buff_putstring("RAM FIFO High-Water Mark: ");
                buff_put_u32_dec10(fifo_max_level);
                buff_putstring("/ 40000\r\n");
                buff_putstring("RAM FIFO Total Overflows: ");
                buff_put_u32_dec10(fifo_overflow_counter);
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
            buff_putchar('U');
        }

        if (!ram_fifo_overflow) {
            gpio_put(led_pin, true);
        }
    }
}

/**
 * @brief Hardware GPIO IRQ callback to monitor generic target power state pin.
 */
void spi_tap_enable_callback(uint gpio, uint32_t events) {
    if (gpio == SPI_TAP_ENABLE_PIN) { 
        if (events & GPIO_IRQ_EDGE_FALL) {
            pio_sm_set_enabled(pio_sniffer, sm_main, false);
        } 
        else if (events & GPIO_IRQ_EDGE_RISE) {
            pio_sm_clear_fifos(pio_sniffer, sm_main);
            pio_sm_set_enabled(pio_sniffer, sm_main, true);
        }
    }
}

/**
 * @brief Initializes the target tracking input pin and configures its edge-triggered interrupts.
 */
void setup_spi_tap_enable_pin() {
    gpio_init(SPI_TAP_ENABLE_PIN);
    gpio_set_dir(SPI_TAP_ENABLE_PIN, GPIO_IN);
    
    gpio_set_irq_enabled_with_callback(
        SPI_TAP_ENABLE_PIN, 
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 
        true, 
        &spi_tap_enable_callback
    );
}

/**
 * @brief Controls Bus Pirate v5 logic buffers to map level-shifted safe IO pathways.
 */
void init_bus_pirate_v5_buffers(void) {
    for (int i = 0; i <= 7; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUT);
        gpio_put(i, 0); 
    }

    for (int i = 8; i <= 15; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i); 
    }
}

int main() {
    float div = 1.0f;

    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    
    BOARD_INIT();
    
    stdio_init_all();

#if SPI_TAP_ENABLE_PIN_CONFIG
    setup_spi_tap_enable_pin();
#endif

    // Synchronously spin up the execution state machines passing the clean configurations
    sm_main = pio_claim_unused_sm(pio_sniffer, true);
    uint offset_main = pio_add_program(pio_sniffer, &spi_main_program);
    spi_main_program_init(pio_sniffer, sm_main, offset_main, div, MY_SPI_MOSI_PIN, MY_SPI_EV0_PIN);

    uint sm_data = pio_claim_unused_sm(pio_sniffer, true);
    uint offset_data = pio_add_program(pio_sniffer, &spi_data_program);
    spi_data_program_init(pio_sniffer, sm_data, offset_data, div, MY_SPI_CLK_PIN, MY_SPI_CS_PIN, MY_SPI_EV0_PIN);

    uint sm_start = pio_claim_unused_sm(pio_sniffer, true);
    uint offset_start = pio_add_program(pio_sniffer, &spi_start_program);
    spi_start_program_init(pio_sniffer, sm_start, offset_start, div, MY_SPI_CS_PIN, MY_SPI_EV0_PIN);

    uint sm_stop = pio_claim_unused_sm(pio_sniffer, true);
    uint offset_stop = pio_add_program(pio_sniffer, &spi_stop_program);
    spi_stop_program_init(pio_sniffer, sm_stop, offset_stop, div, MY_SPI_CS_PIN, MY_SPI_EV0_PIN);

    if (!ram_fifo_init(40000)) {
        while (true);
    }

    pio_sm_set_enabled(pio_sniffer, sm_main, true);
    pio_sm_set_enabled(pio_sniffer, sm_start, true);
    pio_sm_set_enabled(pio_sniffer, sm_stop, true);
    pio_sm_set_enabled(pio_sniffer, sm_data, true);

    multicore_launch_core1(core1_print);
    
    gpio_put(led_pin, true);

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
