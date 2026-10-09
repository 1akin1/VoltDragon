"""Reference telemetry packet shared by the Python tests.

The same bytes are in docs/telemetry.md and tests/unit/test_tlm_msg.c, so the
firmware encoder and both decoders are checked against one definition.
"""

REFERENCE = bytes.fromhex(
    "5644544d033f58000700000040e20100c0d40100010003000cfefa00e7031a04"
    "fcd600001f018dff35fe0901e803000001000000020000009246c8172ef88c13"
    "22247c1708023c009600b7004c002003a4010000305d4ac4"
)
