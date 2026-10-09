*** Comments ***
Phase 2: Node A sends sensor and status messages to Node B over CAN at 50 Hz
(HLR-011). Every frame carries a sequence counter and a CRC; Node B rejects
corrupted frames, counts lost ones and filters out foreign identifiers
(HLR-013). Both nodes run in one emulation, connected through a CAN hub.

Message layout: docs/can-messages.md.

Run from the repository root (inside WSL):
    renode-test tests/robot/system_can.robot


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
${CLEAN_LINK}       rejected 0, lost 0, unknown id 0, overruns 0


*** Keywords ***
Create System
    [Arguments]    ${platform_a}=${ROOT}/renode/node_a.repl
    Execute Command    $bin_a=@${ELF_A}
    Execute Command    $bin_b=@${ELF_B}
    Execute Command    $platform_a=@${platform_a}
    Execute Command    include @${ROOT}/renode/system.resc
    ${a}=    Create Terminal Tester    sysbus.usart2    machine=node_a    defaultPauseEmulation=True
    ${b}=    Create Terminal Tester    sysbus.usart2    machine=node_b    defaultPauseEmulation=True
    ${cmd}=    Create Terminal Tester    sysbus.usart3    machine=node_b    defaultPauseEmulation=True
    Set Test Variable    ${A}    ${a}
    Set Test Variable    ${B}    ${b}
    Set Test Variable    ${CMD}    ${cmd}

Feed Node A Imu
    Execute Command    mach set "node_a"
    Execute Command    sysbus.i2c1.imu AccelerationX -0.5
    Execute Command    sysbus.i2c1.imu AccelerationZ 1
    Execute Command    sysbus.i2c1.imu AngularRateY 10
    Execute Command    sysbus.i2c1.mag MagneticX 0.25
    Execute Command    mach clear

Press On Node A
    [Arguments]    ${key}
    Write Char On Uart    ${key}    testerId=${A}

Wait For Node B
    [Arguments]    ${text}    ${timeout}=3
    Wait For Line On Uart    ${text}    testerId=${B}    timeout=${timeout}


*** Test Cases ***
Should Deliver Node A Data To Node B At 50 Hz
    Create System
    Feed Node A Imu
    # Four messages at 50 Hz each; the first second includes start-up, so check the second one.
    Wait For Node B    heartbeat 2
    Wait For Node B    can: rx 200/s, ${CLEAN_LINK}
    # Values as Node A measured them (Renode IMU model gain: see node_a_imu.robot).
    # The age depends on where the report falls in the 5 ms CAN rotation; freshness is
    # checked by system_telemetry.robot (Should Flag Fresh Node A Data).
    Wait For Node B    ms, resets 0, imu ok, recorder ok, acc -500 0 999 mg, gyro 0 10500 0 mdps, mag 287 0 0 mG

Should Reject A Corrupted Frame
    Create System
    Wait For Node B    heartbeat 1
    Press On Node A    c
    Wait For Line On Uart    injected fault 1    testerId=${A}
    # The rejected frame is also missing from its sequence, so it counts as lost as well.
    Wait For Node B    rejected 1, lost 1, unknown id 0, overruns 0

Should Count A Lost Frame
    Create System
    Wait For Node B    heartbeat 1
    Press On Node A    g
    Wait For Line On Uart    injected fault 2    testerId=${A}
    Wait For Node B    rejected 0, lost 1, unknown id 0, overruns 0

Should Filter Out Foreign Identifiers In Hardware
    Create System
    # Inject in steady state: the first second also counts a start-up frame.
    Wait For Line On Uart    can: tx 200/s    testerId=${A}
    Press On Node A    u
    Wait For Line On Uart    injected fault 3    testerId=${A}
    # Node A sent one extra frame, but Node B's filter never let it through.
    Wait For Line On Uart    can: tx 201/s    testerId=${A}
    Wait For Node B    can: rx 200/s, ${CLEAN_LINK}

Should Resynchronise When Node A Restarts
    Create System
    Wait For Node B    heartbeat 2
    Press On Node A    r
    Wait For Line On Uart    reset cause: SOFTWARE, reset count: 1    testerId=${A}
    Wait For Node B    can: Node A restarted (reset count 1)
    # The restarted sequence counters are not counted as lost frames.
    Wait For Node B    can: rx 200/s, ${CLEAN_LINK}
    Wait For Node B    resets 1, imu ok

Should Report Link Statistics On The Command Interface
    Create System
    Wait For Node B    heartbeat 1
    Write Line To Uart    $1,CAN*51    testerId=${CMD}    waitForEcho=False
    # valid, rejected, lost, unknown_id, age_ms
    Wait For Line On Uart    \\$1,ACK,CAN,\\d+,0,0,0,\\d\\*[0-9A-F]{2}    testerId=${CMD}    treatAsRegex=true

Should Flag Missing Imu Data From Node A
    Create System    platform_a=${ROOT}/renode/stm32f407.repl
    Wait For Line On Uart    imu: init failed    testerId=${A}
    # Only STATUS flows (50 frames/s), and it reports the IMU as invalid and no recorder.
    Wait For Node B    can: rx 50/s, ${CLEAN_LINK}
    Wait For Node B    imu invalid, recorder off

Should Report No Data Without Node A
    Execute Command    $bin_b=@${ELF_B}
    Execute Command    include @${ROOT}/renode/node_b.resc
    ${b}=    Create Terminal Tester    sysbus.usart2    defaultPauseEmulation=True
    Wait For Line On Uart    can: no data from Node A    testerId=${b}
