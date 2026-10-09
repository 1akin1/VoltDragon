*** Comments ***
Phase 4: Node A's safety logic, driven open loop (HLR-001 to HLR-006).

Both nodes run in one emulation. The tests stand in for the rest of the system:
GPS sentences go into Node A's UART4, autopilot status ($VDAPS) into its UART5,
where Node A's orders to the autopilot ($VDCMD) are also checked, and operator
commands (MODE, and PING as the ground station heartbeat) into Node B's USART3.
Closed-loop scenarios, with the plant flying the vehicle, are in
tests/integration/test_safety.py.

GPS positions: 150 m along route line_a, at the height of the conductors, at
the stated distance from the nearest one; generated with the plant's route and
NMEA models (sim/plant).

Run from the repository root (inside WSL):
    renode-test tests/robot/system_safety.robot


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
${GGA_20M}          $GPGGA,100001.00,3954.02826,N,03248.10041,E,1,09,0.9,925.0,M,36.0,M,,*6B
${GGA_11M}          $GPGGA,100001.00,3954.03247,N,03248.09725,E,1,09,0.9,925.0,M,36.0,M,,*6A
${GGA_13M}          $GPGGA,100001.00,3954.03153,N,03248.09795,E,1,09,0.9,925.0,M,36.0,M,,*67
${GGA_16M}          $GPGGA,100001.00,3954.03013,N,03248.09901,E,1,09,0.9,925.0,M,36.0,M,,*61
${CMD_MISSION}      $VDCMD,MISSION,0,0*38
${CMD_AVOID}        $VDCMD,MISSION,1,150*3D
${CMD_HOLD}         $VDCMD,HOLD,0,0*7B
${CMD_RTH}          $VDCMD,RTH,0,0*3A
${CMD_LAND}         $VDCMD,LAND,0,0*73


*** Keywords ***
Create System
    Execute Command    $bin_a=@${ELF_A}
    Execute Command    $bin_b=@${ELF_B}
    Execute Command    $platform_a=@${ROOT}/renode/node_a.repl
    Execute Command    include @${ROOT}/renode/system.resc
    ${a}=    Create Terminal Tester    sysbus.usart2    machine=node_a    defaultPauseEmulation=True
    ${gps}=    Create Terminal Tester    sysbus.uart4    machine=node_a    defaultPauseEmulation=True
    ${ap}=    Create Terminal Tester    sysbus.uart5    machine=node_a    defaultPauseEmulation=True
    ${b}=    Create Terminal Tester    sysbus.usart2    machine=node_b    defaultPauseEmulation=True
    ${cmd}=    Create Terminal Tester    sysbus.usart3    machine=node_b    defaultPauseEmulation=True
    Set Test Variable    ${A}    ${a}
    Set Test Variable    ${GPS}    ${gps}
    Set Test Variable    ${AP}    ${ap}
    Set Test Variable    ${B}    ${b}
    Set Test Variable    ${CMD}    ${cmd}
    Wait For Node A    heartbeat 1

Wait For Node A
    [Arguments]    ${text}    ${timeout}=3    ${regex}=false
    Wait For Line On Uart    ${text}    testerId=${A}    timeout=${timeout}    treatAsRegex=${regex}

Node A Should Not Log
    [Arguments]    ${text}    ${timeout}=1
    Should Not Be On Uart    ${text}    testerId=${A}    timeout=${timeout}

Send Gps
    [Arguments]    ${sentence}
    Write Line To Uart    ${sentence}    testerId=${GPS}    waitForEcho=False

Fly At
    [Documentation]    Repeats a GPS position at 5 Hz, as a receiver would, for about a second
    ...    and a half (paced by Node A's 10 Hz orders to the autopilot), so the fix is
    ...    fresh when Node A next reports.
    [Arguments]    ${sentence}
    FOR    ${i}    IN RANGE    8
        Send Gps    ${sentence}
        Wait For Line On Uart    $VDCMD,    testerId=${AP}
        Wait For Line On Uart    $VDCMD,    testerId=${AP}
    END

Send Autopilot Status
    [Arguments]    ${sentence}
    Write Line To Uart    ${sentence}    testerId=${AP}    waitForEcho=False

Autopilot Should Be Ordered
    [Arguments]    ${sentence}    ${timeout}=1
    Wait For Line On Uart    ${sentence}    testerId=${AP}    timeout=${timeout}

Send Command
    [Arguments]    ${frame}    ${reply}
    Write Line To Uart    ${frame}    testerId=${CMD}    waitForEcho=False
    Wait For Line On Uart    ${reply}    testerId=${CMD}

Send Heartbeat
    [Documentation]    A PING from the ground station, as its display sends once a second.
    [Arguments]    ${seq}
    ${body}=    Set Variable    ${seq},PING
    ${checksum}=    Evaluate    functools.reduce(lambda a, c: a ^ ord(c), $body, 0)    modules=functools
    ${frame}=    Evaluate    "$%s*%02X" % ($body, $checksum)
    ${reply}=    Evaluate    "$%s,ACK,PING*%02X" % ($seq, functools.reduce(lambda a, c: a ^ ord(c), "%s,ACK,PING" % $seq, 0))    modules=functools
    Send Command    ${frame}    ${reply}


*** Test Cases ***
Should Report The Safety State At Start
    Create System
    Wait For Node A    safety: mode MISSION, distance unknown/0 dm, battery 255 %, autopilot silent, ground link ok, rejected requests 0
    # The autopilot gets its orders (10 Hz) even before it reports.
    Autopilot Should Be Ordered    ${CMD_MISSION}

Should Warn And Keep Away From The Line
    [Documentation]    HLR-001, HLR-002: warning below 12 m in the control cycle that sees the
    ...    position, avoidance until clear of 15 m.
    Create System
    Fly At    ${GGA_20M}
    Wait For Node A    safety: mode MISSION, distance (199|200) dm    regex=true
    Node A Should Not Log    PROXIMITY_WARNING    timeout=0.1
    Autopilot Should Be Ordered    ${CMD_MISSION}

    Send Gps    ${GGA_11M}
    # The fix age shows the warning came within a control cycle (20 ms) of the position.
    Wait For Node A    safety: PROXIMITY_WARNING, 109 dm from a conductor \\(fix age 1?\\d ms\\), moving away    regex=true
    Autopilot Should Be Ordered    ${CMD_AVOID}

    # 13 m: beyond the warning distance but not yet clear, so the vehicle keeps avoiding.
    Fly At    ${GGA_13M}
    Wait For Node A    safety: mode MISSION, distance 1(29|30) dm    regex=true
    Node A Should Not Log    proximity warning cleared    timeout=0.1
    Autopilot Should Be Ordered    ${CMD_AVOID}

    Send Gps    ${GGA_16M}
    Wait For Node A    safety: proximity warning cleared at 1(59|60) dm    regex=true
    Autopilot Should Be Ordered    ${CMD_MISSION}

Should Keep The Mission While Heartbeats Arrive And Return Home When They Stop
    [Documentation]    HLR-003: RETURN_TO_HOME 3 s after the last ground station contact.
    Create System
    FOR    ${n}    IN RANGE    2    8
        Wait For Node A    heartbeat ${n}
        Send Heartbeat    ${n}
    END
    # Six seconds of heartbeats, one a second: the link holds.
    Wait For Node A    safety: mode MISSION, distance unknown/0 dm, battery 255 %, autopilot silent, ground link ok
    Node A Should Not Log    ground link lost    timeout=0.1

    # Then silence: the link is lost just over 3 s after the last PING, and the vehicle heads home.
    Wait For Node A    safety: ground link lost for 30[0-2]\\d ms    timeout=5    regex=true
    Wait For Node A    safety: mode MISSION -> RETURN_TO_HOME (ground link lost)
    Autopilot Should Be Ordered    ${CMD_RTH}
    Wait For Line On Uart    can: A mode RETURN_TO_HOME    testerId=${B}

Should Not Lose The Link Before A Ground Station Is Heard
    Create System
    Wait For Node A    heartbeat 5    timeout=6
    Node A Should Not Log    ground link lost    timeout=0.1
    Wait For Node A    safety: mode MISSION

Should Need An Override To Leave Return To Home
    [Documentation]    HLR-005, HLR-006: once the vehicle returns home, only an explicit operator
    ...    override resumes the mission.
    Create System
    Send Heartbeat    1
    Wait For Node A    safety: mode MISSION -> RETURN_TO_HOME (ground link lost)    timeout=5
    # The link comes back, but the vehicle keeps returning home.
    Send Heartbeat    2
    Wait For Node A    safety: ground link back
    Send Command    $3,MODE,MISSION*7C    $3,NAK,MODE,REFUSED*0E
    Wait For Line On Uart    cmd: mode RETURN_TO_HOME -> MISSION refused: NEEDS_OVERRIDE    testerId=${B}
    Node A Should Not Log    -> MISSION    timeout=0.5

    Send Command    $4,MODE,MISSION,OVERRIDE*43    $4,ACK,MODE,MISSION*1E
    Wait For Node A    safety: mode RETURN_TO_HOME -> MISSION (operator, override)
    Autopilot Should Be Ordered    ${CMD_MISSION}

Should Hold And Resume On Operator Request
    [Documentation]    HLR-005
    Create System
    Send Command    $1,MODE,HOLD*3D    $1,ACK,MODE,HOLD*58
    Wait For Node A    safety: mode MISSION -> HOLD (operator)
    Autopilot Should Be Ordered    ${CMD_HOLD}
    Send Command    $2,MODE,MISSION*7D    $2,ACK,MODE,MISSION*18
    Wait For Node A    safety: mode HOLD -> MISSION (operator)
    Autopilot Should Be Ordered    ${CMD_MISSION}

Should Reject Malformed Mode Requests
    Create System
    Send Command    $1,MODE,FLY*61    $1,NAK,MODE,ARGS*5D
    Send Command    $2,MODE,HOLD,PLEASE*1C    $2,NAK,MODE,ARGS*5E
    Send Command    $3,MODE*1C    $3,NAK,MODE,ARGS*5F
    Node A Should Not Log    safety: mode MISSION ->    timeout=0.5

Should Return Home On Low Battery And Land On Critical Battery
    [Documentation]    HLR-004
    Create System
    Send Autopilot Status    $VDAPS,55,MISSION*1C
    Wait For Node A    safety: mode MISSION, distance unknown/0 dm, battery 55 %, autopilot ok
    Send Autopilot Status    $VDAPS,18,MISSION*15
    Wait For Node A    safety: battery low, 18 %
    Wait For Node A    safety: mode MISSION -> RETURN_TO_HOME (battery low)
    Autopilot Should Be Ordered    ${CMD_RTH}
    Send Autopilot Status    $VDAPS,7,RTH*29
    Wait For Node A    safety: battery critical, 7 %
    Wait For Node A    safety: mode RETURN_TO_HOME -> LAND (battery critical)
    Autopilot Should Be Ordered    ${CMD_LAND}
    # Node B sees the same state over CAN: battery low and critical, autopilot OK.
    Wait For Line On Uart    can: A mode LAND, distance 65535 dm, battery 7 %, flags 0x38    testerId=${B}

Should Treat Land As Final
    Create System
    Send Command    $1,MODE,LAND*35    $1,ACK,MODE,LAND*50
    Wait For Node A    safety: mode MISSION -> LAND (operator)
    Send Command    $2,MODE,MISSION,OVERRIDE*45    $2,NAK,MODE,REFUSED*0F
    Send Command    $3,MODE,RTH*7E    $3,NAK,MODE,REFUSED*0E
    # Automatic triggers do not leave LAND either.
    Send Autopilot Status    $VDAPS,18,LAND*5E
    Wait For Node A    safety: battery low, 18 %
    Node A Should Not Log    safety: mode LAND ->    timeout=1
    Autopilot Should Be Ordered    ${CMD_LAND}
