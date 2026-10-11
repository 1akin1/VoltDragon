*** Comments ***
Phase 6: Node B's ARINC 429 output (HLR-015).

Node B sends altitude (label 203), true heading (314) and the distance to the
nearest conductor (251) every 50 ms to an ARINC 429 transmitter on SPI2. The
transmitter is a Renode model (renode/Arinc429Transmitter.cs) with a bus monitor
that decodes every word on its own: these tests check its log, so the firmware's
encoder is checked against an independent decoder.

Words, labels and the transmitter's command set: docs/arinc429.md.

Run from the repository root (inside WSL):
    renode-test tests/robot/system_arinc.robot


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
# 150 m along route line_a, 925 m above sea level, about 10.9 m from the nearest
# conductor (as in system_safety.robot); RMC: 10 knots on a course of 90 degrees.
${GGA}              GPGGA,100001.00,3954.03247,N,03248.09725,E,1,09,0.9,925.0,M,36.0,M,,
${RMC}              GPRMC,100001.00,A,3954.03247,N,03248.09725,E,10.00,90.0,091026,,,A


*** Keywords ***
Create System
    Execute Command    $bin_a=@${ELF_A}
    Execute Command    $bin_b=@${ELF_B}
    Execute Command    include @${ROOT}/renode/system.resc
    Create Log Tester    2
    ${a}=    Create Terminal Tester    sysbus.usart2    machine=node_a    defaultPauseEmulation=True
    ${gps}=    Create Terminal Tester    sysbus.uart4    machine=node_a    defaultPauseEmulation=True
    ${ap}=    Create Terminal Tester    sysbus.uart5    machine=node_a    defaultPauseEmulation=True
    ${b}=    Create Terminal Tester    sysbus.usart2    machine=node_b    defaultPauseEmulation=True
    Set Test Variable    ${A}    ${a}
    Set Test Variable    ${GPS}    ${gps}
    Set Test Variable    ${AP}    ${ap}
    Set Test Variable    ${B}    ${b}

Create Node B Alone
    [Arguments]    ${platform}=${ROOT}/renode/node_b.repl
    Execute Command    $bin_b=@${ELF_B}
    Execute Command    $platform_b=@${platform}
    Execute Command    include @${ROOT}/renode/node_b.resc
    Create Log Tester    2
    ${b}=    Create Terminal Tester    sysbus.usart2    defaultPauseEmulation=True
    Set Test Variable    ${B}    ${b}

Wait For Node B
    [Arguments]    ${text}    ${timeout}=3    ${regex}=false
    Wait For Line On Uart    ${text}    testerId=${B}    timeout=${timeout}    treatAsRegex=${regex}

Bus Should Carry
    [Documentation]    Waits for the bus monitor to decode a word matching the regular expression.
    [Arguments]    ${pattern}    ${timeout}=2
    Wait For Log Entry    arinc429: ${pattern}    timeout=${timeout}    treatAsRegex=true

Nmea
    [Arguments]    ${body}
    ${sentence}=    Evaluate    "$%s*%02X" % ($body, functools.reduce(lambda a, c: a ^ ord(c), $body, 0))    modules=functools
    RETURN    ${sentence}

Fly With Gps
    [Documentation]    Repeats the GPS fix and course at 5 Hz, as a receiver would, for about two
    ...    seconds (paced by Node A's 10 Hz orders to the autopilot).
    ${gga}=    Nmea    ${GGA}
    ${rmc}=    Nmea    ${RMC}
    FOR    ${i}    IN RANGE    10
        Write Line To Uart    ${gga}    testerId=${GPS}    waitForEcho=False
        Write Line To Uart    ${rmc}    testerId=${GPS}    waitForEcho=False
        Wait For Line On Uart    $VDCMD,    testerId=${AP}
        Wait For Line On Uart    $VDCMD,    testerId=${AP}
    END


*** Test Cases ***
Should Find The Transmitter At Start
    Create System
    Wait For Node B    arinc: transmitter ready; labels 203, 314 and 251 every 50 ms
    Bus Should Carry    reset

Should Send Three Words At 20 Hz
    Create System
    Wait For Node B    heartbeat 2
    Wait For Node B    arinc: 60 words/s, SPI errors 0

Should Carry Altitude Heading And Line Distance
    [Documentation]    With a GPS fix, Node A's heading (the GPS course: the magnetometer model
    ...    reads no field, so Node A does not trust it) and its distance to the line.
    Create System
    Fly With Gps
    Wait For Node B    arinc: 60 words/s, SPI errors 0; altitude NO, heading NO, line distance NO
    # 925 m = 3034.8 ft; 90 degrees; 10.9 m is 87 eighths of a metre (10.875 m).
    Bus Should Carry    label 203 sdi 0 ssm NO data 0x[0-9A-F]{5} parity ok, altitude 3035 ft
    Bus Should Carry    label 314 sdi 0 ssm NO data 0x20000 parity ok, true heading 90\\.00 deg
    Bus Should Carry    label 251 sdi 0 ssm NO data 0x[0-9A-F]{5} parity ok, line distance 1[01]\\.\\d\\d m

Should Report No Computed Data Without A Fix
    [Documentation]    Node A runs but has no GPS: no position, no heading source, no distance.
    Create System
    Wait For Node B    arinc: 60 words/s, SPI errors 0; altitude NCD, heading NCD, line distance NCD    timeout=4
    Bus Should Carry    label 203 sdi 0 ssm NCD data 0x00000 parity ok
    Bus Should Carry    label 314 sdi 0 ssm NCD data 0x00000 parity ok
    Bus Should Carry    label 251 sdi 0 ssm NCD data 0x00000 parity ok

Should Report A Failure Warning Without Node A
    Create Node B Alone
    Wait For Node B    arinc: 60 words/s, SPI errors 0; altitude FW, heading FW, line distance FW    timeout=4
    Bus Should Carry    label 314 sdi 0 ssm FW data 0x00000 parity ok

Should Keep Running Without A Transmitter
    Create Node B Alone    ${ROOT}/renode/stm32f407.repl
    Wait For Node B    arinc: no transmitter on SPI2 (id 0x00), running without ARINC 429 output
    Wait For Node B    heartbeat 2
