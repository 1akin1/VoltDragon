cmake_minimum_required(VERSION 3.20)

project(VoltDragon
    VERSION 0.1.0
    DESCRIPTION "Simulated avionics for a power-line inspection UAV"
    LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Cortex-M4F (STM32F407) target flags shared by both nodes
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

add_library(target_options INTERFACE)
target_compile_options(target_options INTERFACE
    ${MCU_FLAGS}
    -Wall -Wextra -Werror -Wshadow -Wconversion
    -ffunction-sections -fdata-sections
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-O2 -g>)
target_link_options(target_options INTERFACE
    ${MCU_FLAGS}
    -Wl,--gc-sections
    --specs=nano.specs --specs=nosys.specs)

add_subdirectory(firmware/common)
add_subdirectory(firmware/node_a)
add_subdirectory(firmware/node_b)
