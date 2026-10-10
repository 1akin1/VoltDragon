*** Comments ***
Phase 2 milestone: IMU data travels from Node A to Node B over CAN, then on to
the ground station as UDP telemetry (HLR-012, HLR-013). Node B also accepts
operator commands over UDP (HLR-014).

The frames leaving Node B are checked byte by byte with Renode's network
interface tester. The ground station is simulated by renode/ground_station.py,
which injects ARP and UDP frames into Node B's Ethernet MAC.

Frame offsets: Ethernet header 0..13, IPv4 header 14..33, UDP header 34..41,
UDP payload from 42. Packet layout: docs/telemetry.md.

Run from the repository root (inside WSL):
    renode-test tests/robot/system_telemetry.robot


*** Settings ***
Resource            ${RENODEKEYWORDS}

Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown


*** Variables ***
${ROOT}             ${CURDIR}/../..
${ELF_A}            ${ROOT}/build/debug/firmware/node_a/node_a.elf
${ELF_B}            ${ROOT}/build/debug/firmware/node_b/node_b.elf
# Broadcast from Node B (02:00:00:56:44:02) to port 5600: Ethernet, IPv4 (UDP, 108 bytes,
# 192.168.10.2 -> 192.168.10.255) and UDP (96 bytes), then "VDTM", version 4, any flags, length 88.
${TELEMETRY_FRAME}  ffffffffffff020000564402080045000074____0000ff11____c0a80a02c0a80aff____15e00060____5644544d04__5800


*** Keywords ***
Create System
    Execute Command    $bin_a=@${ELF_A}
    Execute Command    $bin_b=@${ELF_B}
    Execute Command    include @${ROOT}/renode/system.resc
    Execute Command    include @${ROOT}/renode/ground_station.py
    Execute Command    include @${ROOT}/renode/plant_feed.py
    # The network interface tester only sees time pass while the emulation runs, so the
    # UART testers must not pause it after a match: the emulation runs throughout the test.
    ${a}=    Create Terminal Tester    sysbus.usart2    machine=node_a
    ${b}=    Create Terminal Tester    sysbus.usart2    machine=node_b
    ${cmd}=    Create Terminal Tester    sysbus.usart3    machine=node_b
    Set Test Variable    ${A}    ${a}
    Set Test Variable    ${B}    ${b}
    Set Test Variable    ${CMD}    ${cmd}
    Create Network Interface Tester    sysbus.ethernet    machine=node_b
    Start Emulation
    Wait For Line On Uart    net: link up, 100 Mbit/s full duplex    testerId=${B}

Wait For Node B
    [Arguments]    ${text}    ${timeout}=2
    Wait For Line On Uart    ${text}    testerId=${B}    timeout=${timeout}

Wait For Frame
    [Arguments]    ${hex}    ${index}=0    ${timeout}=2
    Wait For Outgoing Packet With Bytes At Index    ${hex}    ${index}    100    ${timeout}

Send Udp Command
    [Documentation]    Sends "$<body>*<CK>" to Node B's command port from the simulated ground station.
    [Arguments]    ${body}
    ${ck}=    Evaluate    '%02X' % functools.reduce(lambda a, c: a ^ ord(c), $body, 0)    modules=functools
    Execute Command    mach set "node_b"
    Execute Command    gs_arp
    Execute Command    gs_udp 5601 "$${body}*${ck}"
    Execute Command    mach clear

Udp Reply Should Be
    [Documentation]    Waits for Node B's reply from port 5601 to the ground station's port 5700.
    [Arguments]    ${body}
    ${ck}=    Evaluate    '%02X' % functools.reduce(lambda a, c: a ^ ord(c), $body, 0)    modules=functools
    ${payload}=    Evaluate    ('$' + $body + '*' + $ck).encode().hex()
    # UDP header at frame offset 34: port 5601 -> 5700, any length and checksum, then the reply.
    Wait For Frame    15e11644________${payload}    34


*** Test Cases ***
Should Broadcast Telemetry At 10 Hz
    Create System
    Wait For Frame    ${TELEMETRY_FRAME}
    Wait For Node B    heartbeat 2    timeout=3
    Wait For Node B    tlm: 10 packets/s at 10 Hz

Should Carry Node A Imu Data To The Ground Station
    Create System
    Execute Command    mach set "node_a"
    Execute Command    sysbus.i2c1.imu AccelerationX -0.5
    Execute Command    sysbus.i2c1.imu AccelerationZ 1
    Execute Command    mach clear
    # Payload offset 24 (frame offset 66): accel X, Y, Z = -500, 0, 999 mg as little-endian int16.
    Wait For Frame    0cfe0000e703    66    timeout=3

Should Flag Fresh Node A Data
    Create System
    Wait For Node B    heartbeat 1
    # Flags (frame offset 47): Node A data fresh, IMU valid, recorder OK, vibration monitor
    # active. No GPS here, and the magnetometer model reads zero, so GPS fix and
    # magnetometer OK are clear.
    Wait For Frame    5644544d0487    42

Should Carry Node A Safety State
    Create System
    Wait For Node B    heartbeat 1
    # Payload offset 74 (frame offset 116): distance unknown (no GPS), battery unknown
    # (no autopilot), mode MISSION, no safety flags set.
    Wait For Frame    ffffff0000    116
    # Payload offset 82 (frame offset 124): no vibration alarm.
    Wait For Frame    00    124

Should Carry The Vibration Alarm To The Ground Station
    Create System
    Wait For Node B    heartbeat 1
    # A damaged propeller's signature on Node A's IMU (renode/plant_feed.py).
    Execute Command    mach set "node_a"
    Execute Command    vibration_feed "0.25" "4.0" "20" "6"
    Execute Command    mach clear
    Wait For Node B    can: Node A VIBRATION FAULT: imbalance    timeout=3
    # Payload offset 78 (frame offset 120): safety flags with the vibration alarm, request
    # id, ground link age (none yet: saturated), then the alarm: imbalance (HLR-009).
    Wait For Frame    40__ffff01    120

Should Change The Telemetry Rate On Command
    Create System
    Write Line To Uart    $1,TLM_RATE,50*3C    testerId=${CMD}    waitForEcho=False
    Wait For Line On Uart    $1,ACK,TLM_RATE,50*    testerId=${CMD}
    Wait For Node B    tlm: 50 packets/s at 50 Hz    timeout=3

Should Answer Commands Over Udp
    Create System
    Send Udp Command    1,PING
    Wait For Node B    cmd udp: $1,PING*0D -> ACK
    Udp Reply Should Be    1,ACK,PING

Should Reject A Corrupted Udp Command
    Create System
    Execute Command    mach set "node_b"
    Execute Command    gs_arp
    Execute Command    gs_udp 5601 "$2,PING*00"
    Execute Command    mach clear
    Wait For Node B    cmd udp: $2,PING*00 -> CHECKSUM
    Udp Reply Should Be    0,NAK,-,CHECKSUM

Should Keep Running Without Ethernet
    Execute Command    $bin_b=@${ELF_B}
    Execute Command    $platform_b=@${ROOT}/renode/stm32f407.repl
    Execute Command    include @${ROOT}/renode/node_b.resc
    ${b}=    Create Terminal Tester    sysbus.usart2
    Start Emulation
    Wait For Line On Uart    net: Ethernet did not start    testerId=${b}
    Wait For Line On Uart    heartbeat 2    testerId=${b}
