# lwIP: only the bare-metal IPv4/UDP/ARP/ICMP core is built (configured in src/net/lwipopts.h).
# Third-party code is built with the CPU flags but not the project's strict warning set.
set(LWIP_DIR ${PROJECT_SOURCE_DIR}/third_party/lwip)

add_library(lwip STATIC
    ${LWIP_DIR}/src/core/def.c
    ${LWIP_DIR}/src/core/inet_chksum.c
    ${LWIP_DIR}/src/core/init.c
    ${LWIP_DIR}/src/core/ip.c
    ${LWIP_DIR}/src/core/mem.c
    ${LWIP_DIR}/src/core/memp.c
    ${LWIP_DIR}/src/core/netif.c
    ${LWIP_DIR}/src/core/pbuf.c
    ${LWIP_DIR}/src/core/timeouts.c
    ${LWIP_DIR}/src/core/udp.c
    ${LWIP_DIR}/src/core/ipv4/etharp.c
    ${LWIP_DIR}/src/core/ipv4/icmp.c
    ${LWIP_DIR}/src/core/ipv4/ip4.c
    ${LWIP_DIR}/src/core/ipv4/ip4_addr.c
    ${LWIP_DIR}/src/netif/ethernet.c)
# SYSTEM: warnings from lwIP headers are not reported in project code either.
target_include_directories(lwip SYSTEM PUBLIC
    ${LWIP_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/net)
target_link_libraries(lwip PRIVATE target_arch)

add_executable(node_b
    src/can_rx.c
    src/commands.c
    src/main.c
    src/net/eth.c
    src/net/ethernetif.c
    src/net/lwip_port.c
    src/net/net.c
    src/telemetry.c)
set_target_properties(node_b PROPERTIES SUFFIX ".elf")
target_include_directories(node_b PRIVATE src src/net)
target_link_libraries(node_b PRIVATE voltdragon_common lwip)
target_link_options(node_b PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:node_b>/node_b.map)

add_custom_command(TARGET node_b POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:node_b> $<TARGET_FILE_DIR:node_b>/node_b.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:node_b>
    VERBATIM)
