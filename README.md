# DigisparkRNG

Hardware random number generator on a Digispark (ATtiny85): watchdog timer
jitter and ADC noise, conditioned with the Ascon permutation. The design notes
are in the header comment of `clockdrift_rng/clockdrift_rng.ino`.

Every output bit is backed by credited entropy: the sketch estimates how much
each sample is worth, and squeezes 64 bits out only once 128 have been
credited. That makes it an *entropy source* rather than a random number
generator -- about 285 bytes/s, which is the rate the sources actually
produce, not a limit imposed by USB.

A 64 MB capture of `x` output passed PractRand (`RNG_stdin8`, 2^26 bytes) with
no anomalies in 95 test results, over 65 hours at the entropy rate. Note what that does and does not say: any competent
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
`~/Arduino/libraries` -- **1.4.0 or newer** for the libusb transport, which
uses a hook that release added; the serial build works with any version. The
Digistump AVR core provides the board.

## Typical use

    tools/build.sh && tools/flash.sh
    python3 tools/stump.py capture x 60 out.txt             # conditioned output
    nohup tools/assess.sh 6 0 > captures/assess.log 2>&1 &  # ADC only, ~8 minutes

Board: micronucleus 2.6 bootloader (6650 bytes free). If no sketch is running,
`flash.sh` waits 60 s for the board to be plugged in.

## Build options

Pass them through `EXTRA_FLAGS`, as in
`EXTRA_FLAGS="-DRNG_VENDOR=1" tools/build.sh`.

| option | | |
|---|---|---|
| `RNG_VENDOR` | 0 | 1: vendor control transfers instead of the serial port (see *Two transports*) |
| `STREAM_MODE` | 1 | 0: no `S` mode, 130 bytes less flash |
| `RNG_CDC_ECHO` | `RNG_VENDOR` | 1: echo the serial port back, to prove it still works while the bytes go out over libusb |
| `ADC_BATCH` | 16 | ADC readings between USB services. Tuned; the transmitter's timing was measured with it |
| `STACK_CHECK` | 0 | 1: report never-used RAM after each `x` line |

Sizes on the Digispark, of the 6650 bytes micronucleus leaves and 512 of RAM:

| build | flash | RAM |
|---|---|---|
| default | 6062 | 382 |
| `STREAM_MODE=0` | 5932 | 381 |
| `RNG_VENDOR=1` | 5924 | 387 |
| both | 5762 | 386 |

## Modes

Send the character over the serial port, or ask for it with `rngread --mode`:

| mode | output |
|---|---|
| `x` | conditioned bytes as hex, 78 digits a line (default) |
| `X` | conditioned bytes, binary, no messages |
| `S` | the same conditioner free-running, 5650 bytes/s |
| `r` | raw watchdog intervals, low 16 bits of CPU cycles, binary |
| `d` | raw ADC readings, signed bytes, binary |

`S` is a different thing from `X` and the difference is the whole point of
this project, so it is worth stating plainly. In `X` every bit is backed by
credited entropy, and the output is unpredictable even against an adversary
with unlimited computation. `S` keeps squeezing whether or not entropy has
been credited for it, so its output is unpredictable *because Ascon is* -- a
DRBG (an RBG2 construction, in SP 800-90C's terms) rather than an entropy
source. That is the ordinary architecture of every OS random generator and
perfectly sound; it is simply a different claim.

Two things keep it honest. Nothing is emitted until the capacity has been
seeded four times over, so a stream never comes from a state that was never
filled; and entropy keeps being absorbed, so a compromised state heals. The
backtracking ratchet still runs, but only where fresh entropy paid for it, so
a captured state exposes at most one credit's worth of earlier output rather
than the whole stream.

The failure mode you must know about: **if a source dies, `S` keeps streaming
perfect-looking bytes from a stale seed, forever and silently.** In `X` the
rate visibly collapses. So a consumer of `S` should check the source flags and
`seeded` (`rngread --info`) rather than trusting the stream on its own.
Compile it out with `STREAM_MODE=0` if you would rather it not exist; it costs
130 bytes of flash.

Measured: **5650 bytes/s**, twenty times `X`'s 284, against a theoretical
8000 that low-speed USB allows with 8-byte packets. The permutation is not
what stops it -- raw `d` mode, with no Ascon at all, reaches 3800 by the same
path. What is left is the shape of the sending: the sketch fills a 39-byte
burst, sends it in five USB frames and then goes quiet, so the pipe idles
between bursts. Keeping it full would want the output topped up continuously
rather than in bursts, which is a larger change than the remaining 2 kB/s is
worth. `S` also samples the ADC in twos rather than sixteens, since a
conversion is about 100 us and sampling in sixteens left the pipe idle half
the time (3900 -> 5650 bytes/s); `X`'s sampling cadence is untouched.

`r` and `d` are the un-conditioned sources, for the assessment below. They
will fail PractRand, and should: a source carrying about one bit of entropy
per byte is not meant to look uniform, only to be unpredictable. Those are
different properties and only the second one matters here.

### Read the binary modes with the tty in raw mode

A tty starts cooked, with echo on, and a serial port's echo goes back *to the
device*. In `X`, `S`, `r` and `d` that means the host echoes random bytes back
at the sketch, and about one byte in 256 is a mode character -- so the mode
changes by itself, and a stream turns into a trickle of something else. The
default `x` is immune, hex digits being no command, which is why this went
unnoticed for so long.

    stty -F /dev/ttyACM0 raw -echo

`tools/rngread.c` is unaffected (it never opens the tty) and so is anything
that sets raw mode itself. A `cat` or a plain `open()` is not. (Found by the
rngtacho session, on a device that had quietly switched itself to `r`.)

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
  whether each source still passes its health tests, bursts dropped because
  nobody read them, and whether `S` mode has been seeded.

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

## License and credits

GPL version 2, in `LICENSE`. The firmware links V-USB through
[DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast), and V-USB is
distributed by [Objective Development](https://www.obdev.at/vusb/) under the
GPL version 2 or version 3 (with a commercial license offered separately), so
the combined work is covered by the GPL.

`clockdrift_rng/ascon_permute.S` is not mine: it is the AVR assembler
permutation from [ascon/ascon-c](https://github.com/ascon/ascon-c)
(`crypto_hash/asconxof128/avr_lowsize`, commit 446347f) by L. Cardoso and
J. Großschädl of the University of Luxembourg, under CC0-1.0, with only the
function name changed. Its header says so and should stay.

Ascon is the NIST lightweight cryptography standard (SP 800-232). The entropy
assessment uses NIST's SP 800-90B `ea_non_iid`, built by
`tools/build-sp800-90b.sh`. `tools/rngread.c` links libusb (LGPL-2.1+).

There is no circuit diagram because there is no circuit: the entropy comes
from the watchdog oscillator's drift against the CPU clock and from the ADC's
own amplifier noise, with nothing attached to the board.
