# GNSS tools

Make sure you have the asteria-firmware repository or the scripts from there. `tools/gnss/` contains the tools needed for flashing, validating and comparing the GNSS modules and antennas.

We will use the scripts from `tools/gnss/` with [uv](https://docs.astral.sh/uv/), but you can alternatively use `python` from an environment with `pyserial` and `pyubx2` installed.

### To flash and validate the modules

* connect an antenna

* connect the GNSS module to your computer via USB-C

* check which port the module appears under, the exact command varies based on your host platform. The following example assumes `/dev/tty.usbmodem1101`

* flash the configuration and validate it by running
    * `uv run flash_config.py /dev/tty.usbmodem1101`
    * you should see something like this (the version lines depend on the module and its firmware):
      ```
      Receiver at 921600 baud
        EXT CORE 1.00 (61b2dd)
        ROM BASE 0x118B2060
        FWVER=HPG 1.32
        PROTVER=27.31
        MOD=ZED-F9P
        GPS;GLO;GAL;BDS
        SBAS;QZSS
      Flash:
        CFG_UART1_BAUDRATE                   921600  ok
        CFG_UART1INPROT_UBX                       1  ok
        CFG_UART1OUTPROT_UBX                      1  ok
        CFG_UART1OUTPROT_NMEA                     0  ok
        CFG_MSGOUT_UBX_NAV_PVT_UART1              1  ok
        CFG_MSGOUT_UBX_NAV_STATUS_UART1           1  ok
        CFG_RATE_MEAS                           100  ok
        CFG_RATE_NAV                              1  ok
        CFG_NAVSPG_DYNMODEL                       8  ok
      Done
      ```
    * if a line says `MISMATCH` or it ends with `FAILED`, run it again. If it still fails, **the module is bad**!

### To test the antennas

* put the antenna outside or at a window with a clear view of the sky, and leave it alone while the test runs

* make sure the environment stays the same across all antennas you would like to test.

* run the test
    * `uv run compare.py /dev/tty.usbmodem1101`
    * enter the module id and the antenna id when asked
    * the script does a cold start of the module, checks the config again, waits for a 3D fix (up to 5 min), then measures for 2 min
    * you should see something like this, with one line per second while waiting and measuring. The result gets added to `gnss_results.csv`:
      ```
      Module id: 3
      Antenna id: 7
      Receiver at 921600 baud
        EXT CORE 1.00 (61b2dd)
        ...
      1. Cold start
      2. Checking configuration in flash
        CFG_UART1_BAUDRATE                   921600  ok
        CFG_UART1INPROT_UBX                       1  ok
        CFG_UART1OUTPROT_UBX                      1  ok
        CFG_UART1OUTPROT_NMEA                     0  ok
        CFG_MSGOUT_UBX_NAV_PVT_UART1              1  ok
        CFG_MSGOUT_UBX_NAV_STATUS_UART1           1  ok
        CFG_RATE_MEAS                           100  ok
        CFG_RATE_NAV                              1  ok
        CFG_NAVSPG_DYNMODEL                       8  ok
      3. Waiting for a 3D fix
           1 s  fix type 0  0 sats
           2 s  fix type 0  2 sats
        ...
          27 s  fix type 0  3 sats
          28 s  fix type 3  9 sats
      4. Measuring for 120 s
        12 sats  high 41.8 dBHz  top4 44.5 dBHz  hAcc 3.20 m  vAcc 5.10 m
        14 sats  high 42.1 dBHz  top4 44.8 dBHz  hAcc 2.80 m  vAcc 4.60 m
        ...
        22 sats  high 42.4 dBHz  top4 45.2 dBHz  hAcc 1.00 m  vAcc 1.50 m

        TTFF      28 s
        sats      21.0
        high C/N0 42.2 dBHz
        top4 C/N0 45.0 dBHz
        mean C/N0 32.0 dBHz
        hAcc      1.05 m
        vAcc      1.52 m
      Appended to gnss_results.csv
      ```
    * if it ends with `FAILED, no fix after 300 s`, check the antenna connection and the view of the sky first

### Comparing the antennas

* use the same module for every antenna, and test them all in the same environment.
* run `compare.py` for every antenna
* compare `high_cno` in `gnss_results.csv`, the average C/N0 of the satellites above 30°. A bad antenna or cable will show up a few dB-Hz below the others

### What to look at

* `high_cno`: 3 dB-Hz or more below the others is a bad antenna
* `ttff_s`: no fix or a lot slower than the others is a bad module
