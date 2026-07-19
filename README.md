# SPI Sniffer PIO

A high-performance, passive SPI sniffer designed for the RP2040 and RP2350 microcontrollers. This project utilizes the hardware PIO (Programmable I/O) state machines to capture SPI traffic synchronously and streams the data over high-speed USB CDC to a host-side decoder.

## Features

- **Vendor-Agnostic & Portable:** Decoupled from hardcoded GPIO configurations. Supports standard Raspberry Pi Pico hardware and the Bus Pirate 5 using a `BOARD_INIT` macro pattern.
- **Dynamic Pin Mapping:** PIO state machines utilize relative pin mapping configured completely via macros at the top of `main.c`.
- **Hardware-Assisted Capture:** Low-overhead data acquisition using raw PIO processing combined with localized RAM buffering.
- **Automated Workflow:** Flat directory structure with a root-level Makefile wrapper for seamless compilation.
- **Host Decoder Included:** Python-based scripts to process, parse, and analyze captured SPI transactions in real time.

## Directory Structure

```text
.
├── CMakeLists.txt     # Build configuration optimized for Pico SDK
├── Makefile           # Root-level build automation wrapper
├── .gitignore         # Build artifact exclusions
├── README.md          # Project documentation
├── main.c             # Unified configuration and core system logic
├── ram_fifo.c         # Local ring-buffer implementation
├── ram_fifo.h
├── spi_sniffer.pio    # Refactored PIO script with relative pin mapping
└── tools/
    └── rfid_decoder.py # Host-side Python validation and decoding script
