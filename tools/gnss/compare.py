# Tests one module and antenna, appends the result to gnss_results.csv.
#
#   uv run compare.py /dev/tty.usbmodem1101
#
# 1. asks for the module and antenna ids
# 2. cold start the receiver
# 3. checks the config in flash
# 4. waits for a 3D fix, up to 5 min, and records the TTFF
# 5. measures for 2 min: satellites, C/N0 and accuracy
# 6. prints the averages and appends them to gnss_results.csv

import os
import sys
import time
from datetime import datetime
from statistics import mean, median

from pyubx2 import POLL, SET, UBXMessage

from flash_config import check_flash, connect, send, wait_for

FIX_TIMEOUT = 300
TEST_TIME = 120
RESULTS_FILE = "gnss_results.csv"
# satellites above this barely change with the time of day, so their C/N0 is
# comparable between runs
HIGH_ELEVATION = 30

# cold start, GNSS only
COLD_START = UBXMessage(
    "CFG", "CFG-RST", SET,
    eph=1, alm=1, health=1, klob=1, pos=1, clkd=1, osc=1, utc=1, rtc=1, aop=1,
    resetMode=2,
)


# position, fix type and accuracy
def read_pvt(ser):
    send(ser, UBXMessage("NAV", "NAV-PVT", POLL))
    return wait_for(ser, "NAV-PVT")


# signal strength of every tracked satellite, strongest first, and of the ones
# above HIGH_ELEVATION only
def read_cnos(ser):
    send(ser, UBXMessage("NAV", "NAV-SAT", POLL))
    sat = wait_for(ser, "NAV-SAT")
    cnos = []
    high_cnos = []
    if sat is None:
        return cnos, high_cnos
    for i in range(1, sat.numSvs + 1):
        cno = getattr(sat, "cno_%02d" % i)
        elev = getattr(sat, "elev_%02d" % i)
        if cno > 0:
            cnos.append(cno)
            if elev >= HIGH_ELEVATION:
                high_cnos.append(cno)
    cnos.sort(reverse=True)
    return cnos, high_cnos


def ask_id(question):
    while True:
        answer = input(question).strip()
        if answer.isdigit():
            return answer
        print("  needs to be a number")


port = sys.argv[1]
module = ask_id("Module id: ")
antenna = ask_id("Antenna id: ")
ser = connect(port)

print("1. Cold start")
send(ser, COLD_START)
# TTFF counts from here
start = time.time()
time.sleep(1)

print("2. Checking configuration in flash")
if not check_flash(ser):
    sys.exit("FAILED, run flash_config.py first")

print("3. Waiting for a 3D fix")
ttff = None
while ttff is None:
    elapsed = time.time() - start
    if elapsed > FIX_TIMEOUT:
        sys.exit("FAILED, no fix after %d s" % FIX_TIMEOUT)
    pvt = read_pvt(ser)
    if pvt is not None:
        print("  %4.0f s  fix type %d  %d sats" % (elapsed, pvt.fixType, pvt.numSV))
        # 3 = 3D, 4 = 3D + dead reckoning
        if pvt.fixType == 3 or pvt.fixType == 4:
            ttff = elapsed
    time.sleep(1)

print("4. Measuring for %d s" % TEST_TIME)
sats = []
top4 = []
all_cno = []
high_cno = []
h_acc = []
v_acc = []
test_start = time.time()
while time.time() - test_start < TEST_TIME:
    pvt = read_pvt(ser)
    cnos, high_cnos = read_cnos(ser)
    if pvt is not None and len(high_cnos) > 0:
        sats.append(pvt.numSV)
        # average of the 4 strongest, which are likely to experience less multipath
        top4.append(mean(cnos[:4]))
        all_cno.append(mean(cnos))
        high_cno.append(mean(high_cnos))
        # mm to m
        h_acc.append(pvt.hAcc / 1000)
        v_acc.append(pvt.vAcc / 1000)
        print("  %2d sats  high %4.1f dBHz  top4 %4.1f dBHz  hAcc %.2f m  vAcc %.2f m"
              % (pvt.numSV, high_cno[-1], top4[-1], h_acc[-1], v_acc[-1]))
    time.sleep(1)

if len(sats) == 0:
    sys.exit("FAILED, no data from the receiver")

now = datetime.now().isoformat(timespec="seconds")
header = "time,module,antenna,ttff_s,sats,high_cno,top4_cno,mean_cno,h_acc_m,v_acc_m\n"
line = "%s,%s,%s,%.0f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f\n" % (
    now, module, antenna, ttff, mean(sats), mean(high_cno), mean(top4), mean(all_cno),
    median(h_acc), median(v_acc))

print()
print("  TTFF      %.0f s" % ttff)
print("  sats      %.1f" % mean(sats))
print("  high C/N0 %.1f dBHz" % mean(high_cno))
print("  top4 C/N0 %.1f dBHz" % mean(top4))
print("  mean C/N0 %.1f dBHz" % mean(all_cno))
print("  hAcc      %.2f m" % median(h_acc))
print("  vAcc      %.2f m" % median(v_acc))

# one line per run, header only for a new file
new_file = not os.path.exists(RESULTS_FILE)
with open(RESULTS_FILE, "a") as f:
    if new_file:
        f.write(header)
    f.write(line)
print("Appended to", RESULTS_FILE)
