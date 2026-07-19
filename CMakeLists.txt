# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(spi_sniffer_pio C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source files
add_executable(${PROJECT_NAME} 
    main.c
    ram_fifo.c
)

# Create C header file with the name <pio program>.pio.h
pico_generate_pio_header(${PROJECT_NAME}  
    ${CMAKE_CURRENT_LIST_DIR}/spi_sniffer.pio
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions) and hardware drivers
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_pio
    pico_multicore
)

# Set preprocessor flags for optimized USB CDC performance
target_compile_definitions(${PROJECT_NAME} PUBLIC 
    PICO_STDOUT_MUTEX=0 
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
)

# Enable USB CDC output, disable UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
