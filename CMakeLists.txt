cmake_minimum_required(VERSION 3.20)

project(VoltDragon
    VERSION 0.1.0
    DESCRIPTION "Simulated avionics for a power-line inspection UAV"
    LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Host unit tests replace the firmware build (they need the native compiler, not the cross toolchain).
option(VOLTDRAGON_HOST_TESTS "Build host unit tests instead of the firmware" OFF)
if(VOLTDRAGON_HOST_TESTS)
    enable_testing()
    add_subdirectory(tests/unit)
    return()
endif()

# Cortex-M4F (STM32F407) target flags shared by both nodes
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

# CPU, optimisation and linker flags: used by all target code, including third-party libraries.
add_library(target_arch INTERFACE)
target_compile_options(target_arch INTERFACE
    ${MCU_FLAGS}
    -ffunction-sections -fdata-sections
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-O2 -g>)
target_link_options(target_arch INTERFACE
    ${MCU_FLAGS}
    -Wl,--gc-sections
    --specs=nano.specs --specs=nosys.specs)

# Project code additionally builds warning-free under strict warnings.
add_library(target_options INTERFACE)
target_link_libraries(target_options INTERFACE target_arch)
target_compile_options(target_options INTERFACE
    -Wall -Wextra -Werror -Wshadow -Wconversion)

add_subdirectory(firmware/common)
add_subdirectory(firmware/node_a)
add_subdirectory(firmware/node_b)
