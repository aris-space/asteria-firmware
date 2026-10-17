# GNSS receiver config, used by the test & validation scripts, and to generate the
# config the sensor board writes to the receiver's RAM at startup.
#
#   uv run config.py

from pyubx2 import SET_LAYER_RAM, TXN_NONE, UBXMessage

BAUD_RATE = 921600
BAUD_RATE_CONFIG = [("CFG_UART1_BAUDRATE", BAUD_RATE)]
CONFIG = [
    ("CFG_UART1INPROT_UBX", 1),  # UBX in
    ("CFG_UART1OUTPROT_UBX", 1),  # UBX out
    ("CFG_UART1OUTPROT_NMEA", 0),  # no NMEA
    ("CFG_MSGOUT_UBX_NAV_PVT_UART1", 1),  # NAV-PVT every solution
    ("CFG_MSGOUT_UBX_NAV_STATUS_UART1", 1),  # NAV-STATUS every solution
    ("CFG_RATE_MEAS", 100),  # 100 ms, so 10 Hz
    ("CFG_RATE_NAV", 1),  # one solution per measurement
    ("CFG_NAVSPG_DYNMODEL", 8),  # airborne <4g
]


# UBX-CFG-VALSET frame for the receiver's RAM, as a Rust byte array
def rust_format_const(name, items):
    frame = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, items).serialize()
    text = ""
    for key, value in items:
        text += "// %s=%s\n" % (key, value)
    text += "const %s: [u8; %d] = [\n" % (name, len(frame))
    for i in range(0, len(frame), 16):
        row = ", ".join("0x%02x" % byte for byte in frame[i:i + 16])
        text += "    " + row + ",\n"
    text += "];"
    return text


if __name__ == "__main__":
    print(rust_format_const("BAUD_RATE_VALSET", BAUD_RATE_CONFIG))
    print(rust_format_const("CONFIG_VALSET", CONFIG))
