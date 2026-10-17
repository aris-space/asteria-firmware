# Writes config.py to the receiver's flash and checks it.
#
#   uv run flash_config.py /dev/tty.usbmodem1101

import sys
import time

from pyubx2 import POLL_LAYER_FLASH, SET_LAYER_BBR, SET_LAYER_FLASH, SET_LAYER_RAM, TXN_NONE
from pyubx2 import ERR_IGNORE, POLL, UBXMessage, UBXReader
from serial import serial_for_url

from config import BAUD_RATE, BAUD_RATE_CONFIG, CONFIG

# baud rates to try, ours first, then the ones a receiver might still be on
BAUD_RATES = [BAUD_RATE, 9600, 38400, 115200, 230400, 460800]
LAYERS = SET_LAYER_RAM | SET_LAYER_BBR | SET_LAYER_FLASH


# drop whatever is waiting so the next read only sees the reply
def send(ser, msg):
    ser.reset_input_buffer()
    ser.write(msg.serialize())
    ser.flush()


# returns the first message with this name, or None after 1 s
def wait_for(ser, name):
    reader = UBXReader(ser, quitonerror=ERR_IGNORE)
    end = time.time() + 1
    while time.time() < end:
        raw, msg = reader.read()
        if msg is not None and msg.identity == name:
            return msg
    return None


# finds the baud rate the receiver answers on and prints its version
def connect(port):
    for baud in BAUD_RATES:
        ser = serial_for_url(port, baud, timeout=0.1)
        send(ser, UBXMessage("MON", "MON-VER", POLL))
        ver = wait_for(ser, "MON-VER")
        if ver is not None:
            print("Receiver at", baud, "baud")
            print(" ", ver.swVersion.decode().strip("\x00"))
            for i in range(1, 20):
                name = "extension_%02d" % i
                if hasattr(ver, name):
                    print(" ", getattr(ver, name).decode().strip("\x00"))
            return ser
        ser.close()
    sys.exit("No receiver on " + port)


# reads the config back from flash and compares it to config.py
def check_flash(ser):
    items = BAUD_RATE_CONFIG + CONFIG
    keys = [key for key, value in items]
    send(ser, UBXMessage.config_poll(POLL_LAYER_FLASH, 0, keys))
    stored = wait_for(ser, "CFG-VALGET")
    if stored is None:
        print("No reply to CFG-VALGET")
        return False

    ok = True
    for key, value in items:
        actual = getattr(stored, key, None)
        if actual == value:
            print("  %-34s %8s  ok" % (key, actual))
        else:
            print("  %-34s %8s  MISMATCH, want %s" % (key, actual, value))
            ok = False
    return ok


if __name__ == "__main__":
    ser = connect(sys.argv[1])

    # write to RAM, BBR and flash in one go
    send(ser, UBXMessage.config_set(LAYERS, TXN_NONE, CONFIG))
    if wait_for(ser, "ACK-ACK") is None:
        sys.exit("Configuration not acknowledged")

    # sent last, the ACK gets lost on UART1 when the baud rate changes
    send(ser, UBXMessage.config_set(LAYERS, TXN_NONE, BAUD_RATE_CONFIG))
    time.sleep(0.1)
    ser.baudrate = BAUD_RATE

    print("Flash:")
    if not check_flash(ser):
        sys.exit("FAILED")
    print("Done")
