# /// script
# requires-python = ">=3.12"
# dependencies = ["pyubx2"]
# ///
"""Prints the UBX-CFG-VALSET frames the GNSS readout writes to the receivers'
RAM, as the Rust constants in `app/src/tasks/readout/gnss.rs`.

    uv run scripts/gnss_config.py
"""

from pyubx2 import SET_LAYER_RAM, TXN_NONE, UBXMessage

BAUD_RATE = [("CFG_UART1_BAUDRATE", 921_600)]
CONFIG = [
    ("CFG_UART1INPROT_UBX", 1),
    ("CFG_UART1OUTPROT_UBX", 1),
    ("CFG_UART1OUTPROT_NMEA", 0),
    ("CFG_MSGOUT_UBX_NAV_PVT_UART1", 1),
    ("CFG_MSGOUT_UBX_NAV_STATUS_UART1", 1),
    ("CFG_RATE_MEAS", 50),
    ("CFG_RATE_NAV", 1),
    ("CFG_NAVSPG_DYNMODEL", 8),
]


def rust_const(name: str, items: list[tuple[str, int]]) -> str:
    frame = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, items).serialize()
    keys = "".join(f"// {key}={value}\n" for key, value in items)
    rows = (
        "    " + ", ".join(f"0x{byte:02x}" for byte in frame[i : i + 16]) + ","
        for i in range(0, len(frame), 16)
    )
    return f"{keys}const {name}: [u8; {len(frame)}] = [\n" + "\n".join(rows) + "\n];"


print(rust_const("BAUD_RATE_VALSET", BAUD_RATE))
print(rust_const("CONFIG_VALSET", CONFIG))
