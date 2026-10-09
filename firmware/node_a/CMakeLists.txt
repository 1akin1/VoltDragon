# FreeRTOS kernel: tasks, lists and queues (mutexes) with the Cortex-M4F port.
# Static allocation only, so no heap implementation is linked. Third-party code
# is built with the CPU flags but not the project's strict warning set.
set(FREERTOS_DIR ${PROJECT_SOURCE_DIR}/third_party/freertos)

add_library(freertos STATIC
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c)
# SYSTEM: warnings from the kernel headers are not reported in project code either.
target_include_directories(freertos SYSTEM PUBLIC
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rtos)
target_link_libraries(freertos PRIVATE target_arch)

add_executable(node_a
    src/ap_link.c
    src/can_tx.c
    src/flashlog.c
    src/gps.c
    src/imu.c
    src/lsm9ds1.c
    src/main.c
    src/mt25q.c
    src/nav.c
    src/route.c
    src/rtos/rtos_port.c
    src/safety.c
    src/tasks.c)
set_target_properties(node_a PROPERTIES SUFFIX ".elf")
target_include_directories(node_a PRIVATE src src/rtos)
# m: single-precision maths (sqrtf, atan2f) for the heading computation.
target_link_libraries(node_a PRIVATE voltdragon_common freertos m)
target_link_options(node_a PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:node_a>/node_a.map)

add_custom_command(TARGET node_a POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:node_a> $<TARGET_FILE_DIR:node_a>/node_a.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:node_a>
    VERBATIM)
