*** Comments ***
Phase 2: Node B's operator command interface on USART3 (HLR-014). Every
command is acknowledged or rejected with a reason; corrupted frames are
rejected; a retransmitted command is answered again without re-executing.

Protocol: docs/command-interface.md. Parser edge cases are covered by the host
unit tests in tests/unit; these tests check the firmware end to end.

Run from the repository root (inside WSL):
    renode-test tests/robot/node_b_commands.robot


*** Settings ***
Resource            ${RENODEKEYWORDS}

Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown


*** Variables ***
${ROOT}             ${CURDIR}/../..
${ELF}              ${ROOT}/build/debug/firmware/node_b/node_b.elf


*** Keywords ***
Create Node B
    Execute Command    $bin_b=@${ELF}
    Execute Command    include @${ROOT}/renode/node_b.resc
    ${console}=    Create Terminal Tester    sysbus.usart2    defaultPauseEmulation=True
    ${cmd}=    Create Terminal Tester    sysbus.usart3    defaultPauseEmulation=True
    Set Test Variable    ${CONSOLE}    ${console}
    Set Test Variable    ${CMD}    ${cmd}
    Wait For Line On Uart    cmd: listening on USART3    testerId=${CONSOLE}

Framed
    [Documentation]    Returns "$<body>*<CK>" with the XOR checksum of the body.
    [Arguments]    ${body}
    ${ck}=    Evaluate    '%02X' % functools.reduce(lambda a, c: a ^ ord(c), $body, 0)    modules=functools
    RETURN    $${body}*${ck}

Send Raw
    [Arguments]    ${line}
    Write Line To Uart    ${line}    testerId=${CMD}    waitForEcho=False

Send Command
    [Arguments]    ${body}
    ${line}=    Framed    ${body}
    Send Raw    ${line}

Expect Response
    [Documentation]    Waits for the exact response line, including its checksum.
    [Arguments]    ${body}
    ${line}=    Framed    ${body}
    Wait For Line On Uart    ${line}    testerId=${CMD}

Command Should Return
    [Arguments]    ${request}    ${response}
    Send Command    ${request}
    Expect Response    ${response}


*** Test Cases ***
Should Boot With The Command Interface
    Create Node B
    Wait For Line On Uart    I heartbeat 1    testerId=${CONSOLE}

Should Answer Ping And Version
    Create Node B
    Command Should Return    1,PING    1,ACK,PING
    Command Should Return    2,VERSION    2,ACK,VERSION,0.1.0

Should Report Status Counters
    Create Node B
    Command Should Return    1,PING    1,ACK,PING
    Command Should Return    2,FLY    2,NAK,FLY,UNKNOWN
    Send Command    3,STATUS
    # uptime_ms, reset_count, accepted, rejected (the STATUS request itself is not yet counted)
    Wait For Line On Uart    \\$3,ACK,STATUS,\\d+,0,1,1\\*[0-9A-F]{2}    testerId=${CMD}    treatAsRegex=true

Should Set And Query The Telemetry Rate
    Create Node B
    Command Should Return    1,TLM_RATE    1,ACK,TLM_RATE,10
    Command Should Return    2,TLM_RATE,25    2,ACK,TLM_RATE,25
    Command Should Return    3,TLM_RATE    3,ACK,TLM_RATE,25

Should Reject Invalid Arguments
    Create Node B
    Command Should Return    1,TLM_RATE,51    1,NAK,TLM_RATE,ARGS
    Command Should Return    2,TLM_RATE,9    2,NAK,TLM_RATE,ARGS
    Command Should Return    3,TLM_RATE,fast    3,NAK,TLM_RATE,ARGS
    Command Should Return    4,PING,now    4,NAK,PING,ARGS
    # The rejected values did not change the setting.
    Command Should Return    5,TLM_RATE    5,ACK,TLM_RATE,10

Should Reject Unknown Commands
    Create Node B
    Command Should Return    1,FLY    1,NAK,FLY,UNKNOWN
    Command Should Return    2,ping    2,NAK,ping,UNKNOWN

Should Reject Corrupted Frames
    Create Node B
    Send Raw    $1,PING*00
    Expect Response    0,NAK,-,CHECKSUM
    Send Raw    hello
    Expect Response    0,NAK,-,FRAME
    Send Command    0,PING
    Expect Response    0,NAK,-,SEQ
    ${long}=    Evaluate    '$1,' + 'X' * 100
    Send Raw    ${long}
    Expect Response    0,NAK,-,LENGTH
    # The interface still works afterwards.
    Command Should Return    2,PING    2,ACK,PING

Should Resynchronise After Line Noise
    Create Node B
    ${line}=    Framed    1,PING
    Send Raw    @!~noise${line}
    Expect Response    1,ACK,PING

Should Replay The Response To A Retransmitted Command
    Create Node B
    Command Should Return    1,TLM_RATE,20    1,ACK,TLM_RATE,20
    Command Should Return    1,TLM_RATE,20    1,ACK,TLM_RATE,20
    Wait For Line On Uart    cmd: $1,TLM_RATE,20*    testerId=${CONSOLE}
    Wait For Line On Uart    -> ACK (replayed)    testerId=${CONSOLE}
    # The replay was not counted as a second command.
    Send Command    2,STATUS
    Wait For Line On Uart    \\$2,ACK,STATUS,\\d+,0,1,0\\*    testerId=${CMD}    treatAsRegex=true

Should Answer Back To Back Commands In Order
    Create Node B
    ${a}=    Framed    1,PING
    ${b}=    Framed    2,VERSION
    ${c}=    Framed    3,TLM_RATE,50
    # All three lines arrive before the firmware answers the first one.
    Send Raw    ${a}\r${b}\r${c}
    Expect Response    1,ACK,PING
    Expect Response    2,ACK,VERSION,0.1.0
    Expect Response    3,ACK,TLM_RATE,50
