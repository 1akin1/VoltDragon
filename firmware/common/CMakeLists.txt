# Startup code, drivers and services shared by every node.
# An OBJECT library makes sure the vector table and fault handlers are always
# linked, even though no code references them by name.
add_library(voltdragon_common OBJECT
    src/ap_msg.c
    src/board.c
    src/can.c
    src/can_msg.c
    src/cmd_dispatch.c
    src/cmd_protocol.c
    src/crc32.c
    src/crc8.c
    src/fault.c
    src/flight_mode.c
    src/i2c.c
    src/iwdg.c
    src/log.c
    src/nmea.c
    src/node_boot.c
    src/reset_info.c
    src/spi.c
    src/startup_stm32f4.c
    src/systick.c
    src/tlm_msg.c
    src/uart.c)

target_include_directories(voltdragon_common PUBLIC include)
target_link_libraries(voltdragon_common PUBLIC target_options)

set(VOLTDRAGON_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32f407vg.ld)
target_link_options(voltdragon_common INTERFACE
    -T${VOLTDRAGON_LINKER_SCRIPT}
    -nostartfiles)
set_property(TARGET voltdragon_common APPEND PROPERTY INTERFACE_LINK_DEPENDS ${VOLTDRAGON_LINKER_SCRIPT})
