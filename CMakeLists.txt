# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")
set_property(CACHE PICO_BOARD PROPERTY STRINGS pico2 pico2_w)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(FlightCodePI C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(FlightCodePI
        config_protocol.c
        FlightCodePI.c
        esc_controller.c
        flight_log.c
        flight_settings.c
        imu.c
        mpu6050.c
        mpu6500.c
        rate_controller.c
        sbus_receiver.c
)

pico_generate_pio_header(FlightCodePI
        ${CMAKE_CURRENT_LIST_DIR}/sbus_rx.pio
)

pico_generate_pio_header(FlightCodePI
        ${CMAKE_CURRENT_LIST_DIR}/dshot.pio
)

pico_set_program_name(FlightCodePI "FlightCodePI")
pico_set_program_version(FlightCodePI "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(FlightCodePI 0)
pico_enable_stdio_usb(FlightCodePI 1)

# Add the standard library to the build
target_link_libraries(FlightCodePI
        pico_stdlib
        hardware_clocks
        hardware_flash
        hardware_i2c
        hardware_pio
        hardware_spi
        hardware_watchdog
)

# Pico 2 W routes its onboard LED through the CYW43 device. The "none"
# architecture initializes that device without enabling a Wi-Fi connection.
if (PICO_CYW43_SUPPORTED)
    target_link_libraries(FlightCodePI pico_cyw43_arch_none)
endif()

# Add the standard include files to the build
target_include_directories(FlightCodePI PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(FlightCodePI 
        
        )

pico_add_extra_outputs(FlightCodePI)

