# Host unit tests for firmware modules that have no hardware dependencies.
# Built with the native compiler: cmake --preset host-tests && ctest --preset host-tests

set(FIRMWARE_COMMON ${PROJECT_SOURCE_DIR}/firmware/common)

add_library(host_options INTERFACE)
target_compile_options(host_options INTERFACE
    -Wall -Wextra -Werror -Wshadow -Wconversion -g -O0)
target_include_directories(host_options INTERFACE
    ${FIRMWARE_COMMON}/include
    ${CMAKE_CURRENT_SOURCE_DIR})

function(add_unit_test name)
    add_executable(${name} ${name}.c ${ARGN})
    target_link_libraries(${name} PRIVATE host_options)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_unit_test(test_cmd_protocol
    ${FIRMWARE_COMMON}/src/cmd_protocol.c)
add_unit_test(test_can_msg
    ${FIRMWARE_COMMON}/src/can_msg.c
    ${FIRMWARE_COMMON}/src/crc8.c
    ${FIRMWARE_COMMON}/src/crc32.c)
add_unit_test(test_tlm_msg
    ${FIRMWARE_COMMON}/src/tlm_msg.c
    ${FIRMWARE_COMMON}/src/crc32.c)
add_unit_test(test_nmea
    ${FIRMWARE_COMMON}/src/nmea.c
    ${FIRMWARE_COMMON}/src/cmd_protocol.c)
add_unit_test(test_flight_mode
    ${FIRMWARE_COMMON}/src/flight_mode.c)
add_unit_test(test_ap_msg
    ${FIRMWARE_COMMON}/src/ap_msg.c
    ${FIRMWARE_COMMON}/src/flight_mode.c
    ${FIRMWARE_COMMON}/src/cmd_protocol.c)
add_unit_test(test_route
    ${PROJECT_SOURCE_DIR}/firmware/node_a/src/route.c)
target_include_directories(test_route PRIVATE ${PROJECT_SOURCE_DIR}/firmware/node_a/src)
target_link_libraries(test_route PRIVATE m)
add_unit_test(test_cmd_dispatch
    ${FIRMWARE_COMMON}/src/cmd_dispatch.c
    ${FIRMWARE_COMMON}/src/cmd_protocol.c)
add_unit_test(test_vib_features
    ${PROJECT_SOURCE_DIR}/firmware/node_a/src/vib_features.c
    ${PROJECT_SOURCE_DIR}/firmware/node_a/src/vib_alarm.c)
target_include_directories(test_vib_features PRIVATE ${PROJECT_SOURCE_DIR}/firmware/node_a/src)
target_link_libraries(test_vib_features PRIVATE m)

# The INT8 classifier on TensorFlow Lite Micro, built natively from the same vendored sources.
set(TFLM_DIR ${PROJECT_SOURCE_DIR}/third_party/tflite-micro)
file(STRINGS ${TFLM_DIR}/sources.txt TFLM_SOURCES)
list(TRANSFORM TFLM_SOURCES PREPEND ${TFLM_DIR}/)
add_library(tflm_host STATIC ${TFLM_SOURCES})
target_include_directories(tflm_host SYSTEM PUBLIC
    ${TFLM_DIR}
    ${TFLM_DIR}/third_party/flatbuffers/include)
target_compile_definitions(tflm_host PUBLIC TF_LITE_STATIC_MEMORY)
target_compile_options(tflm_host PRIVATE -O1 -fno-exceptions -fno-rtti)

set(NODE_A_SRC ${PROJECT_SOURCE_DIR}/firmware/node_a/src)
add_executable(test_vib_model test_vib_model.cc
    ${NODE_A_SRC}/vib_model.cc
    ${NODE_A_SRC}/vib_model_data.c
    ${NODE_A_SRC}/vib_features.c
    ${NODE_A_SRC}/tflm_port.cc)
target_include_directories(test_vib_model PRIVATE ${NODE_A_SRC})
# 64-bit pointers make the interpreter's bookkeeping in the arena larger than on Node A.
target_compile_definitions(test_vib_model PRIVATE VIB_MODEL_ARENA_BYTES=8192U)
target_compile_options(test_vib_model PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>)
target_link_libraries(test_vib_model PRIVATE host_options tflm_host m)
add_test(NAME test_vib_model COMMAND test_vib_model)
