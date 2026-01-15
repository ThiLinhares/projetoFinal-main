# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
cmake_policy(SET CMP0057 NEW)

# --- CONFIGURAÇÃO DO FREERTOS ---
set(FREERTOS_KERNEL_PATH "C:/FreeRTOSv202406.04-LTS/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel")
set(FREERTOS_PORT_PATH "${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040")

include(pico_sdk_import.cmake)

project(gas_fogo C CXX ASM)

pico_sdk_init()

# --- Criação da Biblioteca FreeRTOS ---
add_library(FreeRTOS STATIC
    "${FREERTOS_KERNEL_PATH}/tasks.c"
    "${FREERTOS_KERNEL_PATH}/queue.c"
    "${FREERTOS_KERNEL_PATH}/list.c"
    "${FREERTOS_KERNEL_PATH}/timers.c"
    "${FREERTOS_KERNEL_PATH}/event_groups.c"
    "${FREERTOS_KERNEL_PATH}/stream_buffer.c"
    "${FREERTOS_KERNEL_PATH}/portable/MemMang/heap_4.c"
    "${FREERTOS_PORT_PATH}/port.c"
)

target_include_directories(FreeRTOS PUBLIC
    "."
    "${FREERTOS_KERNEL_PATH}/include"
    "${FREERTOS_PORT_PATH}/include"
)

target_link_libraries(FreeRTOS PUBLIC 
    pico_stdlib
    pico_multicore
    hardware_exception
    hardware_irq
    hardware_claim
    hardware_sync
)

# --- Executável Principal ---
add_executable(gas_fogo
    gas_fogo.c
    bme280_driver.c
)

add_subdirectory(lib/u8g2pico)

pico_enable_stdio_usb(gas_fogo 1)
pico_enable_stdio_uart(gas_fogo 0)

# ADICIONADO: hardware_pwm para controle de brilho
target_link_libraries(gas_fogo
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_pwm  # <--- ADICIONADO AQUI
    u8g2pico
    FreeRTOS
)

target_include_directories(gas_fogo PRIVATE
    "${FREERTOS_KERNEL_PATH}/include"
    "${FREERTOS_PORT_PATH}/include"
    "lib/u8g2pico/lib/u8g2/csrc"
)

pico_add_extra_outputs(gas_fogo)