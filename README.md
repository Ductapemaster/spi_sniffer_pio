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
```

## PIO-Based SPI Sniffer Working Principle

The firmware leverages the RP2040/RP2350 Programmable I/O (PIO) blocks to achieve low-latency, hardware-level capture of SPI frames without CPU intervention during transmission. The architecture is inspired by a proven internal IRQ/WAIT synchronization mechanism originally designed for I2C sniffing.

To fully decode the SPI transaction lifecycle, the system utilizes three specialized PIO state machines (SM) running concurrent programs coordinated via internal interrupts (`IRQ 5`) and auxiliary event signaling pins.

### Architecture Diagram

```mermaid
graph TD
    subgraph Hardware Inputs
        G0[GPIO 0: CS]
        G1[GPIO 1: CLK]
        G2[GPIO 2: MOSI]
        G3[GPIO 3: MISO]
        G6[GPIO 6: EN *Optional]
    end

    subgraph PIO State Machines
        PRG1[PRG1: CS-FALL<br/>SM0 / Detects Transaction Start]
        PRG2[PRG2: CS-RISE<br/>SM1 / Detects Transaction Stop]
        PRG3[PRG3: MAIN DATA SAMPLER<br/>SM2 / Shifter & Error Detector]
    end

    subgraph Host Outputs / Event Codes
        G4[GPIO 4: EV1]
        G5[GPIO 5: EV0]
    end

    G0 --> PRG1
    G0 --> PRG2
    G1 & G2 & G3 --> PRG3
    G6 -.->|Enable Control| PRG3

    PRG1 -->|Triggers IRQ 5 & Sets EV=01| PRG3
    PRG2 -->|Triggers IRQ 5 & Sets EV=11| PRG3
    PRG3 -->|Pushes Data & Sets EV=00| G4 & G5
```

## Detailed Component Breakdown
### 1. PRG1: CS-FALL (Transaction Initialization)
* Role: Monitors the Chip Select (CS) line for a high-to-low transition.

* Operation: As soon as a falling edge is detected on the CS pin, it marks the beginning of an SPI frame. It sets the external event configuration code to START (0x01) via the auxiliary pins (EV1, EV0) and fires the internal hardware interrupt IRQ 5. This immediately signals and synchronizes the main data sampler program.

### 2. PRG2: CS-RISE (Transaction Termination)
- Role: Monitors the Chip Select (CS) line for a low-to-high transition.

* Operation: When the master de-asserts the CS line, this program detects the rising edge, immediately flags the event pins with the STOP (0x11) code, and triggers IRQ 5. This acts as an asynchronous abort/cleanup signal for the main data loop.

### 3. PRG3: Main Data Sampler & Processing Loop
* Role: Handles synchronization with the SPI serial clock (CLK), shifts raw data from both data lines, and ensures protocol integrity.

* Operation:

    * Data Shifting: Upon receiving the initialization trigger from IRQ 5 (via PRG1), it begins sampling both MISO and MOSI lines on the configured edge of the incoming CLK signal.

    * FIFO Management: It relies on the PIO's hardware FIFO AUTO PUSH mechanism to efficiently stream full parsed data blocks down to the RX FIFO buffer where the CPU or DMA can collect them. During regular transmissions, it signals the DATA (0x00) state on the event pins.

    * Bit Missing Detection: If a rising edge interrupt from PRG2 occurs before a full byte structure is shifted in, PRG3 catches the condition, flags a framing error ("Bit Missing"), flushes its internal shift registers, and prepares for the next transaction.

## Hardware Configuration & Pinout

| Signal Name | Pin Assignment | Direction | Description |
| :---: | :--- | :---: | :--- |
| CS |  GPIO 0 |  Input |  SPI Chip Select (Active Low) | 
|  CLK |  GPIO 1 |  Input |  SPI Serial Clock |  
|  MOSI |  GPIO 2 |  Input |  Master Output Slave Input data line |  
|  MISO |  GPIO 3 |  Input |  Master Input Slave Output data line | 
| EV1 | GPIO 4 | Output | Event Code Bit 1 | 
| EV0 | GPIO 5 | Output | Event Code Bit 0 | 
| EN | GPIO 6 | Input | Hardware Capture Enable (Optional) |

## Event Code Truth Table
The downstream host-side decoder uses the status of the auxiliary event pins (EV1, EV0) to accurately reconstruct the packet lifecycle stream:

| EV1 | EV0 | Protocol State Event | Technical Condition |
| :---: | :--- | :---: | :--- |
| 0 | 0 | DATA | Normal operational data byte transfer via MISO/MOSI lines. |
| 0 | 1 | START | CS Falling Edge detected. The sniffer engine resets its buffers. |
| 1 | 1 | STOP | CS Rising Edge detected. Frame closed; incomplete bytes flagged as missing bits. |

## Getting Started

### Quick Start (Precompiled Binaries)

If you want to use the sniffer immediately without installing compiling tools, you can use the pre-built binaries included in this repository:

* **Locate the Binaries**: Navigate to the `bin/` directory in the root of this project.
* **Select Your Architecture**:
    * Download `spi_sniffer_pio_rp2040.uf2` if you are using the original Raspberry Pi Pico (RP2040).
    * Download `spi_sniffer_pio_rp2350.uf2` if you are deploying to the Raspberry Pi Pico 2 (RP2350).
* **Flash the Board**: Connect your Pico board to your computer via USB while holding down the `BOOTSEL` button, then drag and drop the corresponding `.uf2` file into the mounted mass storage volume.

---

### Development Environment Setup

To compile the firmware from source, your environment must have the ARM GNU Toolchain and the Raspberry Pi Pico SDK (version 2.0.0 or higher is required to compile for the RP2350 platform).

* **Toolchain Installation**: For an automated, streamlined setup of the SDK and compiler tools on Linux or Raspberry Pi OS environments, you can follow the official automated setup resources available at the [Raspberry Pi Pico Setup Repository](https://github.com/raspberrypi/pico-setup).
* **Environment Configuration**: Make sure that the `PICO_SDK_PATH` environment variable is correctly exported and points to your local installation directory of the Pico SDK.

---

### Building from Source

This project includes a root-level `Makefile` wrapper to automate the standard `cmake` and `make` sequence, eliminating the need for manual build directory navigation.

1. Clone the repository and enter the project directory:
```bash
git clone https://github.com/jjsch-dev/spi_sniffer_pio.git
cd spi_sniffer_pio
```
2. Compile for your specific target hardware:

* For Raspberry Pi Pico (RP2040):

```bash
make pico1
```
This command initializes the build folder, targets the standard Pico board profile, runs compilation, and automatically places the output inside bin/spi_sniffer_pio_rp2040.uf2.

* For Raspberry Pi Pico 2 (RP2350):

```bash
make pico2
```
This command configures the build setup utilizing the pico2 board profile to compile for the new architecture, placing the final binary inside bin/spi_sniffer_pio_rp2350.uf2.

2. Clean the environment:
To wipe out the generated build artifacts and clear the precompiled binary folder, run:
```bash
make clean
```

## Hardware & System Configuration

The project features a centralized global configuration profile using C preprocessor macros. This abstraction layer enables seamless transitioning between hardware platforms, pin configurations, and host-side telemetry formatting before compiling the firmware.

### 1. Target Hardware Selection Profile

The firmware adapts its underlying pin mapping and system initialization code by toggling the main hardware profile macro:

* **Bus Pirate 5 Mode (`TARGET_BUS_PIRATE_5 1`)**:
    * **Target Architecture**: Optimizes the system topology for the Bus Pirate 5 hardware interface.
    * **Pin Mapping**: Automatically binds the SPI tap inputs and event lines to physical GPIOs 8 through 13.
    * **Hardware Abstraction**: Configures `BOARD_INIT()` to invoke `init_bus_pirate_v5_buffers()`, ensuring that on-board bidirectional voltage shifters and logic buffers are safely initialized before execution.
* **Standard Pico Mode (`TARGET_BUS_PIRATE_5 0`)**:
    * **Target Architecture**: Provisions a clean layout for standalone Raspberry Pi Pico or Pico 2 boards.
    * **Pin Mapping**: Maps all connections to a flat, sequential layout using GPIOs 0 through 5.
    * **Hardware Abstraction**: Binds `BOARD_INIT()` to an optimized, compiler-friendly no-op expression (`((void)0)`), eliminating call overhead.

### 2. Dynamic Hardware Gating

You can dynamically isolate or trigger data acquisition using an external tracking pin:

* **`SPI_TAP_ENABLE_PIN_CONFIG`**:
    * Set to `1` to activate dynamic hardware gating. The PIO data sampling machine evaluates the state of the designated validation pin, restricting data storage exclusively to windows when the target system is operationally active.
    * Set to `0` to disable gating. The PIO engine captures data continuously, bypassing pin level evaluation.
* **`SPI_TAP_ENABLE_PIN`**:
    * Sets the specific GPIO number (defaults to GPIO 14) dedicated to tracking the status or activation signal of the target board.

### 3. System Settings & Telemetry Formatting

These definitions balance data output structure against available serialization throughput:

* **`SNIFFER_COMPACT_MODE`**:
    * **Value `0` (Verbose Mode)**: Generates human-readable console outputs, tracking text labels alongside individual line events. Ideal for direct terminal monitoring.
    * **Value `1` (Compact Parallel Hex Stream)**: Outputs compressed, highly structured parallel hexadecimal frames. This minimizes serial bus congestion and is optimized for processing by host-side Python decoders.
* **`SNIFFER_TELEMETRY`**:
    * **Value `1` (Diagnostics Active)**: Interleaves low-level runtime metadata within the execution pipeline to analyze state machine performance and processing loops.
    * **Value `0` (Clean Production)**: Deactivates non-essential diagnostic output, preserving 100% of the transmission bandwidth for decoded SPI traffic.
