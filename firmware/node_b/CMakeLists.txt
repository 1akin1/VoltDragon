add_executable(node_b
    src/can_rx.c
    src/commands.c
    src/main.c)
set_target_properties(node_b PROPERTIES SUFFIX ".elf")
target_link_libraries(node_b PRIVATE voltdragon_common)
target_link_options(node_b PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:node_b>/node_b.map)

add_custom_command(TARGET node_b POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:node_b> $<TARGET_FILE_DIR:node_b>/node_b.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:node_b>
    VERBATIM)
