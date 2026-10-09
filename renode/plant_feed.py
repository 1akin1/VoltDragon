# Co-simulation helpers: feed the Python plant model's data into the nodes.
#
# Load with:  include @renode/plant_feed.py
#
#   plant_feed "<samples>" "<gps hex>" "<mag>" "<autopilot hex>"     (Node A selected)
#
# <samples>:       ';'-separated accelerometer and gyroscope samples, each
#                  'ax,ay,az,gx,gy,gz' in g and deg/s. They are queued in the LSM9DS1
#                  model, which hands out one sample per firmware read, so a whole
#                  co-simulation step is fed at once (ahead of time).
# <gps hex>:       bytes for UART4 (NMEA sentences) as hex, or '-' for none.
# <mag>:           'mx,my,mz' in gauss, set as the magnetometer's current value. The
#                  magnetometer model ignores queued samples, so it is updated once per step.
# <autopilot hex>: bytes from the autopilot for UART5 ($VDAPS) as hex, or '-' for none.
#
#   uart_write "<uart>" "<hex>"        (any machine selected)
#
# Writes bytes into a UART of the selected machine, e.g. the ground station's
# heartbeat into Node B's command interface: uart_write "sysbus.usart3" "24312c...".
#
# Renode turns functions named mc_<name> into monitor commands; this runs in
# Renode's IronPython 2.7.

import System
from System import Byte, Decimal

_INVARIANT = System.Globalization.CultureInfo.InvariantCulture


def _decimal(text):
    return Decimal.Parse(text, _INVARIANT)


def _write_hex(uart, text):
    text = str(text)
    if text == "-":
        return
    for i in range(0, len(text), 2):
        uart.WriteChar(Byte(int(text[i:i + 2], 16)))


def mc_plant_feed(samples, gps_hex, mag_values, autopilot_hex="-"):
    machine = monitor.Machine  # noqa: F821
    imu = machine["sysbus.i2c1.imu"]
    for sample in str(samples).split(";"):
        if not sample:
            continue
        v = [_decimal(x) for x in sample.split(",")]
        imu.FeedAccelerationSample(v[0], v[1], v[2], 1)
        imu.FeedAngularRateSample(v[3], v[4], v[5], 1)

    m = [_decimal(x) for x in str(mag_values).split(",")]
    mag = machine["sysbus.i2c1.mag"]
    mag.MagneticX = m[0]
    mag.MagneticY = m[1]
    mag.MagneticZ = m[2]

    _write_hex(machine["sysbus.uart4"], gps_hex)
    _write_hex(machine["sysbus.uart5"], autopilot_hex)


def mc_uart_write(uart_name, data_hex):
    _write_hex(monitor.Machine[str(uart_name)], data_hex)  # noqa: F821
