# DigisparkRNG

Hardware random number generator on a Digispark (ATtiny85): watchdog timer
jitter and ADC noise, conditioned with the Ascon permutation. The design notes
are in the header comment of `clockdrift_rng/clockdrift_rng.ino`.

Every output bit is backed by credited entropy: the sketch estimates how much
each sample is worth, and squeezes 64 bits out only once 128 have been
credited. That makes it an *entropy source* rather than a random number
generator -- about 285 bytes/s, which is the rate the sources actually
produce, not a limit imposed by USB.

A 64 MB capture passed PractRand (`RNG_stdin8`, 2^26 bytes) with no anomalies
in 95 test results. Note what that does and does not say: any competent
conditioner makes any input look uniform -- hash a counter and every test
suite ever written passes -- so the statistical run tests the *plumbing*. The
claim about the entropy rests on the SP 800-90B assessment of the raw samples
below.

## Layout

- `clockdrift_rng/`: the sketch. `~/Arduino/clockdrift_rng` is a symlink to
  it, so the Arduino IDE finds it and the source lives here.
  - `clockdrift_rng.ino`, `ascon_permute.S`: firmware
  - `rawdump.py`: convert raw dumps for SP 800-90B, compute health test cutoffs
  - `analyze.py`: statistics on hex output (`x` mode), a check for bugs only
- `tools/`
  - `build.sh [SKETCH_DIR]`: compile for the Digispark at 16.5 MHz into `build/`
  - `flash.sh [HEX]`: Ctrl-C x3 into the bootloader, flash, wait for the port
  - `stump.py`: capture a mode, jump to the bootloader, reboot, wait for the port
  - `rngread.c`: read the random bytes over libusb (see *Two transports*)
  - `60-digispark-rng.rules`: udev rule so `rngread` needs no root
  - `assess.sh [ADC_MINUTES] [INTERVAL_MINUTES]`: raw `d` and `r` captures, the
    SP 800-90B assessment and health test cutoffs, all in `captures/<date-time>/`
  - `build-sp800-90b.sh`: rebuild `ea_non_iid` and its libraries in `third_party/`

Not in the repository, because they belong to other projects: `arduino-cli`
(copied from the Arduino IDE AppImage), `arduino-lint`, `ea_non_iid` (built by
`build-sp800-90b.sh`), `third_party/`, `build/` and `captures/`.

Needs [DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast) in
`~/Arduino/libraries`.

## Typical use

    tools/build.sh && tools/flash.sh
    python3 tools/stump.py capture x 60 out.txt             # conditioned output
    nohup tools/assess.sh 6 0 > captures/assess.log 2>&1 &  # ADC only, ~8 minutes

Board: micronucleus 2.6 bootloader (6650 bytes free). If no sketch is running,
`flash.sh` waits 60 s for the board to be plugged in.

## Modes

Send the character over the serial port, or ask for it with `rngread --mode`:

| mode | output |
|---|---|
| `x` | conditioned bytes as hex, 78 digits a line (default) |
| `X` | conditioned bytes, binary, no messages |
| `r` | raw watchdog intervals, low 16 bits of CPU cycles, binary |
| `d` | raw ADC readings, signed bytes, binary |

`r` and `d` are the un-conditioned sources, for the assessment below. They
will fail PractRand, and should: a source carrying about one bit of entropy
per byte is not meant to look uniform, only to be unpredictable. Those are
different properties and only the second one matters here.

## Two transports

`RNG_VENDOR` chooses how the bytes leave the chip:

- **0 (default)**: the USB serial port, read with any terminal or with
  `stump.py`.
- **1**: vendor control requests on endpoint 0, read with `tools/rngread.c`
  and libusb. The serial port stays enumerated and completely unused, so
  another function of the same sketch can have it -- this chip's USB driver
  has no endpoint to spare for a second one. Needs `USB_CFG_VENDOR_HOOK` set
  to 1 in DigiCDCFast's `src/usbconfig.h`.

        EXTRA_FLAGS="-DRNG_VENDOR=1 -DUSB_CFG_VENDOR_HOOK=1" tools/build.sh
        cc -O2 -o tools/rngread tools/rngread.c -lusb-1.0
        tools/rngread --info
        tools/rngread | pv > /dev/null

  `rngread` needs permission on the USB device: install
  `tools/60-digispark-rng.rules` in `/etc/udev/rules.d/`, or run it as root.
  It learns how long a burst takes and sleeps through most of it rather than
  polling hard, because each request steals cycles from the timing the
  entropy comes from.

  In vendor builds the diagnostics that `x` mode prints as `#` comments become
  requests instead (`rngread --info`): protocol, mode, the Ascon self-test,
  and whether each source still passes its health tests.

Both transports measure the same 285 bytes/s, so the choice costs nothing.

## Assessing the entropy

`tools/assess.sh` captures raw samples and runs NIST SP 800-90B's non-IID
estimator over them. What the sketch credits is deliberately below what the
estimator returns:

| source | estimated | credited |
|---|---|---|
| watchdog intervals | 6.44 bits/sample | 1 |
| ADC readings | 1.09 bits/sample | 1 |

The intervals are cut hardest because part of their jitter is USB polling
latency, which the host controls rather than physics.

SP 800-90B's continuous health tests (repetition count, adaptive proportion)
run on every sample with cutoffs for alpha = 2^-40; a source that fails stops
being credited, and `rngread --info` says so.

**Not yet done:** the restart test (SP 800-90B section 3.1.4) -- the first
samples after each of many power-ups, checked for column-wise predictability.
It is the one test that catches a source which looks fine within a run but
starts from the same place every boot, and within-run testing cannot see that
at all.
