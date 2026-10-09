add_executable(node_a
    src/can_tx.c
    src/flashlog.c
    src/gps.c
    src/imu.c
    src/lsm9ds1.c
    src/main.c
    src/mt25q.c
    src/nav.c)
set_target_properties(node_a PROPERTIES SUFFIX ".elf")
# m: single-precision maths (sqrtf, atan2f) for the heading computation.
target_link_libraries(node_a PRIVATE voltdragon_common m)
target_link_options(node_a PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:node_a>/node_a.map)

add_custom_command(TARGET node_a POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:node_a> $<TARGET_FILE_DIR:node_a>/node_a.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:node_a>
    VERBATIM)
