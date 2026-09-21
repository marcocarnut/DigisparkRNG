# Running DigisparkRNG in Wokwi

Live project: **https://wokwi.com/projects/475707136206905345** — open it to run
the simulator in a browser, no hardware needed. `diagram.json` here is the same
wiring, kept in the repo so this stays reproducible if the link ever moves.

## What it simulates

Wokwi has no USB, so the simulator runs the **UART build** (`RNG_UART=1`): the
random bytes come out on **PB2** as 8N1 serial, and the status LED is on **PB1**.
The default baud is 115200, which is what Wokwi's serial monitor uses, so the
`s` hex stream shows up in the monitor on start, and the LED blinks once a
second (the blink is driven off the watchdog, which the simulator does run).

## A pleasant surprise: the floating pin

For the generator to produce anything at all, the ADC source needs noise on its
input -- the floating analog pin PB5. avr8js's ADC has no randomness of its own,
so this was not a given; but Wokwi's front-end models a floating analog pin
realistically enough that the source carries entropy, passes its health tests,
and the stream seeds and runs. A happy thing to find in a simulator.

It also gives a way to *demonstrate a source failure*: wire PB5 to GND in the
diagram. The ADC then reads a constant, fails its SP 800-90B health tests within
a few dozen samples, and the LED switches to the fast "a source has died" blink
-- the same thing it would do on real hardware if that source stopped.

**But do not mistake the simulation for a validated RNG.** It *looks* like it
works and produces random-looking bytes, and it exercises the firmware end to
end -- which is what it is for. But the quality claims in the main README (the
PractRand run, the SP 800-90B assessment) were made against the **real device's**
output, not the simulator's. The simulator's entropy is Wokwi's model of a
floating pin, not real analog physics, and its output has not been put through
those tests. Treat it as a functional demo, not as evidence about the randomness.

## Wiring (in `diagram.json`)

| ATtiny85 | serial monitor |
|---|---|
| PB2 (TX) | RX |
| PB0 (RX) | TX |
| GND | GND |

and the LED: PB1 → 1 kΩ resistor → LED anode, LED cathode → GND.

## What does NOT work in the simulator

Receiving does not. The UART receiver uses the ATtiny85's USI clocked by a
Timer0 compare match, and avr8js (Wokwi's AVR engine) does not wire the USI to
Timer0 — so a byte typed into the monitor never reaches the sketch, and you
**cannot switch modes over the serial line under Wokwi**. Transmit and the LED
are unaffected. To try a mode other than `s` in the simulator, set it at build
time, e.g. `#define DEFAULT_MODE 'x'` at the top of the sketch, or pass
`-DDEFAULT_MODE="'d'"` to the build. On real hardware the USI receiver works and
mode switching over the UART is fine (between bursts).

## Building it yourself in a fresh Wokwi project

If you don't use the link above: create an ATtiny85 project, paste
`clockdrift_rng/clockdrift_rng.ino` and `clockdrift_rng/ascon_permute.S`, add
`#define RNG_UART 1` at the very top of the sketch, and paste this
`diagram.json`. No libraries are needed — the UART build carries no USB stack.
