/*
  clockdrift_rng -- Digispark (ATtiny85) hardware random number generator

  Board:  Digispark, Clock "16.5 MHz - For V-USB"   Library: DigiCDCFast (~/Arduino/libraries)
  Needs micronucleus 2.x (6650 bytes available).

  Entropy sources
  ---------------
  Watchdog jitter: the WDT runs off its own ~128 kHz RC oscillator while the
  CPU runs off the internal RC oscillator + PLL (16.5 MHz). The WDT interrupt
  fires every ~16 ms and each firing is timestamped with CPU-cycle resolution
  (60.6 ns). A sample is the interval between two firings.

  ADC noise: ADC0-ADC0 (both inputs on PB5, the reset pin) at 20x gain,
  bipolar, so a reading is the amplifier's offset plus its noise. The USB pins
  are avoided: they'd saturate the input and the sample-and-hold charge would
  disturb USB. A sample is one reading, clamped to a signed byte.

  Conditioning ('x' mode)
  -----------------------
  Samples are absorbed into an Ascon permutation state (sponge, 64-bit rate,
  256-bit capacity). Each sample adds a conservative entropy credit per source,
  taken from SP 800-90B estimates of that source. Once 128 bits have been
  credited, 64 bits are squeezed out and the state is ratcheted ("forget":
  zero the rate and permute, twice) so earlier outputs can't be recovered from
  a later state. Credits then restart from zero.
  SP 800-90B repetition count and adaptive proportion tests run on every
  sample; a source that fails stops being credited.

  Output is hex ('x', 78 digits a line) or binary ('X'); 'S' streams.
  USB activity disturbs both sources, so
  all modes collect a burst with no output, send it, then discard the samples
  taken while sending.

  Timestamping details
  --------------------
  millis/micros use Timer1 (prescaler 64, 3.88 us per tick). Timer0 is a
  free-running counter with no prescaler; its value mod 64 recovers the Timer1
  prescaler phase, giving single-cycle resolution without another interrupt.
  The core's millis ISR is ISR_NOBLOCK and increments its overflow counter
  last, so a capture that preempts it is 16384 cycles (one Timer1 overflow)
  early. Such a timestamp is detected by the short-then-long interval pair it
  causes and corrected, which delays processing by one sample.
  Because of Timer0, analogWrite() on pins 0 and 1 must not be used.

  Sending Ctrl-C three times in a row (within 2 s) reboots into the bootloader.

  Transport: the random bytes leave over USB (DigiCDCFast) by default, or over
  a plain 8N1 UART on PB2 (TX) / PB0 (RX) when built with RNG_UART -- no USB at
  all, so it runs on any board and under a simulator, and reads on a terminal
  or feeds another microcontroller's UART. See the RNG_UART block below.
*/

// Transport, chosen before the includes because RNG_UART removes V-USB.
#ifndef RNG_UART
#define RNG_UART 0     // 1: a bit-banged UART on PB2/PB0 instead of USB
#endif

// Status LED on PB1 (the onboard LED on most Digisparks; some clones wire it
// to PB0 -- define LED_PIN then). Solid while it has not yet validated the
// sources and so emits nothing; a brief 10% blink once a second while
// generating normally with both sources passing; a fast 50% blink, five times
// a second, if a source has failed its health tests.
#ifndef LED_STATUS
#define LED_STATUS 1
#endif
#ifndef LED_PIN
#define LED_PIN PB1
#endif

#if RNG_UART
#include <util/delay.h>
#else
#include <DigiCDCFast.h>
#endif
#include <avr/wdt.h>
#include <avr/eeprom.h>

// Remember the last mode across power cycles in EEPROM byte 0, so a device
// keeps whatever it was set to. Erased EEPROM (0xFF) means "never set" and
// becomes the default. 0 disables it: the mode is always DEFAULT_MODE.
#ifndef MODE_EEPROM
#define MODE_EEPROM 1
#endif

// Transport for the random bytes.
//   0: a line (or a binary burst) on the USB serial port, read with any
//      terminal or with tools/stump.py.
//   1: vendor control requests on endpoint 0, read with tools/rngread.c and
//      libusb. The serial port stays enumerated and untouched, so another
//      function of the same sketch can have it -- which is the point: this
//      chip's USB driver has no endpoints to spare for a second one.
// Needs USB_CFG_VENDOR_HOOK 1 in DigiCDCFast's usbconfig.h.
#ifndef RNG_VENDOR
#define RNG_VENDOR 0
#endif
// 1: echo back whatever arrives on the serial port, which in RNG_VENDOR
// builds is otherwise unused. It is there to prove the port still works while
// the random bytes go out over libusb -- stand-in for whatever real second
// function the port is meant to carry.
#ifndef RNG_CDC_ECHO
#define RNG_CDC_ECHO RNG_VENDOR
#endif

#if RNG_VENDOR && !USB_CFG_VENDOR_HOOK
#error "RNG_VENDOR needs USB_CFG_VENDOR_HOOK in DigiCDCFast's usbconfig.h"
#endif
#if RNG_UART && RNG_VENDOR
#error "RNG_UART and RNG_VENDOR are two transports; pick one"
#endif

#if RNG_VENDOR
// Vendor requests, all addressed to the device so that libusb needs no
// interface claimed and the kernel's serial driver keeps the port.
#define RNG_RQ_READ   1  // in:  the burst that is ready, or nothing yet
#define RNG_RQ_MODE   2  // out: wValue is the mode character
#define RNG_RQ_INFO   3  // in:  one diagnostic, chosen by wValue, one byte
#define RNG_RQ_BOOT   4  // out: into the bootloader
// One diagnostic per request rather than a packed word: nothing to agree on
// between the two sides beyond the index, and room to add more.
#define RNG_INFO_PROTOCOL   0  // this protocol's version
#define RNG_INFO_MODE       1  // the mode in force
#define RNG_INFO_ASCON      2  // 1 if the Ascon self-test passed
#define RNG_INFO_INTERVALS  3  // 1 if the interval source still passes its health tests
#define RNG_INFO_ADC        4  // 1 if the ADC source does
#define RNG_INFO_OVERRUNS   5  // captures dropped because a burst was not read, saturating
#define RNG_INFO_SEEDED     6  // 'S' mode: 1 once the capacity has been seeded
#define RNG_PROTOCOL        1

static volatile uint8_t vendorLen;    // bytes of burst waiting to be read
static volatile bool vendorTaken;     // and the host has just taken them
static volatile bool bootWanted;      // asked for from inside the interrupt
static uint8_t overruns;              // captures dropped waiting to be read
#endif

// Output mode, switchable at runtime by sending the character over serial.
// Lowercase is hex text (readable on a terminal); uppercase is binary.
//  'x' / 'X': conditioned random bytes, hex / binary.
//  's' / 'S': conditioned, free-running at the chip's rate, hex / binary.
//             's' is the default -- readable hex a terminal shows on plug-in.
//  'r' / 'R': raw watchdog intervals, low 16 bits of CPU cycles, hex / binary.
//  'd' / 'D': raw ADC readings, signed bytes, hex / binary.
// Binary bursts (X, S, R, D) are framed as 0xA5, mode, byte count, data; the
// assessment tools read the uppercase R and D. Hex is for eyes, not tools.
// The mode is remembered in EEPROM across power cycles (see MODE_EEPROM).
// The mode at power-up; override it to fix a build to one output without a
// terminal (a simulator, or a standalone device).
#ifndef STREAM_MODE
#define STREAM_MODE       1    // full definition and rationale below
#endif
#ifndef DEFAULT_MODE
#if RNG_VENDOR
#define DEFAULT_MODE      'X'  // the host formats; libusb reads bursts
#elif STREAM_MODE
#define DEFAULT_MODE      's'  // hex stream: what a terminal shows first
#else
#define DEFAULT_MODE      'x'  // hex conditioned, when there is no stream
#endif
#endif

// Entropy credit per sample, in 1/64 bit; 0 disables crediting a source.
// SP 800-90B non-IID assessment (ea_non_iid) on 'R'/'D' dumps:
//   intervals (low 8 bits):  6.44 bits/sample from 66747 samples
//   ADC readings (0..11):    1.09 bits/sample from 1232751 samples as 4-bit
//                            symbols (1.11 as 8-bit; 1.14 from 286299 earlier)
// Intervals get only 1 bit: part of their jitter is USB polling latency,
// which the host controls. The ADC gets 1 bit, ~10% under its estimate at
// room temperature on one board.
#define INTERVAL_CREDIT   64
#define ADC_CREDIT        64
// SP 800-90B health test cutoffs for the assessed entropy, alpha = 2^-40
// (rawdump.py cutoffs H): the ADC is sampled thousands of times per second,
// so 2^-20 would give false alarms within minutes.
#define INTERVAL_RCT_CUTOFF  8
#define INTERVAL_APT_CUTOFF  31
#define ADC_RCT_CUTOFF       38
#define ADC_APT_CUTOFF       321

#define LINE_LENGTH       78
#define BURST_BYTES       (LINE_LENGTH / 2)
#define RING_SIZE         4   // must be a power of 2
#define TIMER1_OVF_CYCLES 16384L
#define APT_WINDOW        512
#define CREDIT_PER_OUTPUT (128 * 64)  // credited 1/64 bits per 64 output bits

// 'S' mode: keep squeezing whether or not entropy has been credited for it,
// so the rate is the chip's rather than the sources'. What comes out is then
// unpredictable because Ascon is, not because every bit is backed by measured
// entropy -- a DRBG (SP 800-90C calls this an RBG2 construction) rather than
// an entropy source. Two things keep that honest:
//   - nothing is emitted until the capacity has been seeded SEED_FILLS times
//     over, so a stream is never served from a state that has not been filled;
//   - entropy keeps being absorbed, so a compromised state heals.
// The health tests still run, and 'S' will happily keep streaming from a
// stale seed if a source dies -- which is exactly the failure that becomes
// invisible here and is visible in 'X'. The host can see it: ask for the
// source flags.
#ifndef STREAM_MODE
#define STREAM_MODE       1
#endif
#define SEED_FILLS        4   // x 128 credited bits into a 256-bit capacity
#ifndef ADC_BATCH
#define ADC_BATCH         16          // readings between USB services
#endif
// 'S' services USB far more often: a conversion is about 100 us and the host
// takes 8 bytes a frame, so sampling in sixteens leaves the pipe idle half
// the time. 16 -> 4 measured 3900 -> 5200 bytes/s, and 2 -> 5660; 1 gains nothing.
#define ADC_BATCH_STREAM  2
#define STACK_CHECK       0   // 1: report never-used RAM after each 'x' line
// 1: control build that absorbs nothing. Credits and output timing are
// unchanged, but the output must then repeat identically after every reboot,
// showing that the unpredictability comes from the samples, not from Ascon.
#define NO_ENTROPY_CONTROL 0

// The Digispark core runs millis() on Timer1 and exposes its overflow count
// here; the interval timestamp reads it. A weak definition lets the sketch
// also link against a core that does not (a simulator, or a plain ATtiny
// core), where nothing drives it and the interval timestamps are degenerate.
// That is harmless where it happens: such an environment has no jitter for
// them to carry, and the health tests see the constant and refuse to credit
// it. On the real Digispark the core's strong definition wins.
extern "C" { volatile unsigned long millis_timer_overflow_count __attribute__((weak)); }
extern "C" void ascon_permute(uint8_t *state, uint8_t rounds);  // ascon_permute.S

struct Health {       // SP 800-90B health test state for one source
  uint8_t last, repeats;       // repetition count test
  uint8_t aptSample;           // adaptive proportion test
  uint16_t aptCount, aptIndex;
  bool failed;
};

static char mode = DEFAULT_MODE;
static bool producing;    // has validated entropy and squeezed at least once

// A legal mode letter (STREAM_MODE gates 's'/'S'). Used to reject a corrupt or
// erased EEPROM byte on boot.
static bool validMode(char c)
{
  if (c == 'x' || c == 'X' || c == 'r' || c == 'R' || c == 'd' || c == 'D')
    return true;
#if STREAM_MODE
  if (c == 's' || c == 'S')
    return true;
#endif
  return false;
}

#if MODE_EEPROM
static char persistedMode;   // last value known to be in EEPROM byte 0
// Boot: adopt the stored mode; an erased (0xFF) or corrupt byte becomes the
// default and is written back. loop() then writes any later change once.
static void restoreMode()
{
  char c = (char)eeprom_read_byte((const uint8_t *)0);
  if (!validMode(c))
    c = DEFAULT_MODE;
  mode = c;
  eeprom_update_byte((uint8_t *)0, (uint8_t)c);  // writes only if it differs
  persistedMode = c;
}
// Called from loop(), so the ~3 ms EEPROM write never lands in an interrupt
// (a mode set over libusb happens in the USB ISR). eeprom_update writes only
// when the byte differs, so it also stays off the wear budget when idle.
static void persistMode()
{
  if (mode != persistedMode) {
    eeprom_update_byte((uint8_t *)0, (uint8_t)mode);
    persistedMode = mode;
  }
}
#endif
// The captures, one array per field rather than an array of 6-byte structs:
// indexing those needs a multiply, which the compiler does with a library
// call, and a call inside an interrupt makes it save every call-clobbered
// register. (From the rngtacho session, which measured 58 bytes on its build.)
static uint16_t ringOvf[RING_SIZE];  // Timer1 overflow count, low 16 bits
static uint8_t  ringT1[RING_SIZE];   // TCNT1 (ticks every 64 CPU cycles)
static uint8_t  ringT0[RING_SIZE];   // TCNT0 (ticks every CPU cycle)
static volatile uint8_t ringHead, ringTail;
static volatile bool overrun;

#if LED_STATUS
// The LED's clock: the watchdog fires ~62.5 times a second (16 ms period) and
// is the one timebase that runs everywhere. millis() is not: setup() reprograms
// Timer0 for the entropy timestamp and clears its overflow interrupt, so a core
// that runs millis() on Timer0 (the ATtiny cores Wokwi uses) never advances it.
// The Digispark core runs millis() on Timer1, so it survives on real hardware
// -- but the watchdog tick blinks the LED identically in both.
static volatile uint16_t ledTicks;
#define LED_WDT_HZ     62                  // watchdog interrupts per second
#define LED_SLOW       LED_WDT_HZ          // ~1 s period, the "producing" blink
#define LED_SLOW_ON    (LED_WDT_HZ / 10)   // ~10% duty
#define LED_FAST       (LED_WDT_HZ / 5)    // ~5 Hz period, the "failed" blink
#endif

ISR(WDT_vect)
{
  // Hardware reads first, in a fixed order, so the latency between them is
  // constant (it folds into the calibration offset below).
  uint8_t t0 = TCNT0;
  uint8_t t1 = TCNT1;
  // Only the low half of the count is ever used (see the timestamp arithmetic
  // below). Reading it alone is still consistent: the millis interrupt cannot
  // run inside this one. A uint16_t* cast would trip -Wstrict-aliasing.
  const volatile uint8_t *count = (const volatile uint8_t *)&millis_timer_overflow_count;
  uint16_t ovf = count[0] | count[1] << 8;
  if ((TIFR & _BV(TOV1)) && t1 < 255)  // pending overflow not yet counted
    ovf++;

  uint8_t next = (ringHead + 1) & (RING_SIZE - 1);
  if (next == ringTail) {
    overrun = true;
  } else {
    uint8_t h = ringHead;
    ringOvf[h] = ovf;
    ringT1[h] = t1;
    ringT0[h] = t0;
    ringHead = next;
  }
#if LED_STATUS
  ledTicks++;   // last, so it never delays the timestamp reads above
#endif
}

// Runs before init(): a watchdog reset leaves the watchdog armed, and
// SerialUSB.begin() takes longer than the shortest timeout.
void disableWatchdog() __attribute__((naked, used, section(".init3")));
void disableWatchdog()
{
  MCUSR = 0;
  wdt_disable();
}

// Quiet everything the sketch enabled and jump to the reset vector, which
// micronucleus points at itself. (A watchdog reset loops forever on
// micronucleus 1.06, which doesn't clear WDRF.)
static void enterBootloader()
{
  cli();
  WDTCR = _BV(WDCE) | _BV(WDE);
  WDTCR = 0;
  USICR = 0;      // stop the USI receiver (no-op on the USB build)
  TIMSK = 0;
  TCCR0B = 0;
  TCCR1 = 0;
  ((void (*)())0)();
}

// ----------------------------------------------------------------- transport
// The random bytes and the mode commands go through five calls: txBegin,
// txByte, txFlush, rxAvail, rxRead. Over USB they are DigiCDCFast; over the
// UART they are the bit-banged code below.
#if RNG_UART

#ifndef RNG_UART_BAUD
#define RNG_UART_BAUD 115200 // the send blocks a whole byte (below), so keep it
#endif                       // short: 115200 is ~87 us; V-USB has OSCCAL pinned
                             // to 16.5 MHz, so the internal RC holds it. Lower
                             // rates work too (9600, ...) but block proportionally longer.
#define UART_TX  PB2         // PB1 is the status LED, so TX is on PB2
#define UART_RX  PB0

// The transmitter bit-bangs with cycle-counted delays; UART_TX_OVERHEAD is the
// non-delay cycles of one send-loop pass (from the disassembly), and F_CPU/baud
// adapts to the build clock. Tune UART_TX_OVERHEAD if a receiver sees framing
// errors. The receiver instead uses the USI, clocked by Timer0 (below).
#define UART_BIT_CYCLES (F_CPU / RNG_UART_BAUD)
#ifndef UART_TX_OVERHEAD
#define UART_TX_OVERHEAD 10
#endif

// Received bytes wait here for loop() -- a keystroke or two, never a stream.
static volatile uint8_t uartRx[4];
static volatile uint8_t uartRxHead, uartRxTail;
static uint8_t rxBitTicks;   // Timer0 CTC period - 1 for one bit, set in txBegin
static uint8_t rxPrescaler;  // Timer0 clock-select bits for the bit rate

// A whole byte is bit-banged with interrupts off, so the watchdog can't stretch
// a bit into a framing error. It blocks the caller for the byte -- ~87 us at
// 115200, more at lower rates -- which is why an incoming start bit can be
// missed while a burst streams: mode switching over the UART is reliable only
// between bursts, or set DEFAULT_MODE at build time. The samples the watchdog
// would have taken meanwhile are discarded after the send (see sendBurst).
static void txByte(uint8_t c)
{
  uint8_t s = SREG;
  cli();
  uint16_t frame = ((uint16_t)c << 1) | 0x200;  // start bit (0), 8 data, stop (1)
  for (uint8_t i = 0; i < 10; i++) {
    // Branchless, so every bit takes the same cycles whatever its value -- a
    // data-dependent branch would jitter the bit width, which shows at 115200.
    PORTB = (PORTB & ~_BV(UART_TX)) | ((uint8_t)(frame & 1) << UART_TX);
    frame >>= 1;
    __builtin_avr_delay_cycles(UART_BIT_CYCLES - UART_TX_OVERHEAD);
  }
  SREG = s;                          // line left idle high on the stop bit
}

// Receive with the USI, so the ISRs stay short and interrupts stay on -- a
// bit-banged read would hold them off for a whole character. PCINT0 on PB0
// catches the start bit's falling edge (INT0 can't: its pin is PB2, the TX
// line), then the USI shifts the eight data bits in, clocked by Timer0 at the
// bit rate, and PCINT0 re-arms once they are in. USI wire mode 0 samples DI
// (PB0) only -- it never drives DO (PB1, the LED) or USCK (PB2, TX).
//
// Timer0 is the entropy fine-timestamp between bytes; the start bit borrows it
// (CTC at the bit rate) and USI_OVF hands it back free-running. The timestamp
// is a difference, so Timer0's phase shift cancels out -- only the one interval
// spanning the keystroke is disturbed, and the next one is clean again.
//
// NOTE: avr8js (Wokwi) does not clock the USI from a Timer0 compare match, so
// reception -- and switching mode over the UART -- does not work in the
// emulator. The transmit stream does; to try another mode there, set
// DEFAULT_MODE at build time. On real hardware the USI receiver works.
ISR(PCINT0_vect)
{
  if (PINB & _BV(UART_RX))          // a rising edge, or noise: not a start bit
    return;
  PCMSK = 0;                        // one byte at a time: no PCINT during the read
  TCCR0B = 0;
  TCCR0A = _BV(WGM01);              // CTC: Timer0 clears at OCR0A, one tick a bit
  OCR0A = rxBitTicks;
  TCNT0 = rxBitTicks / 2;           // first USI sample lands mid start bit
  TIFR = _BV(OCF0A);
  USISR = _BV(USIOIF) | 7;          // overflow after 9 shifts: start + 8 data bits
  USICR = _BV(USIOIE) | _BV(USICS0);  // mode 0 (DI only), clocked by Timer0 match
  TCCR0B = rxPrescaler;
}

ISR(USI_OVF_vect)                   // the eight data bits are in
{
  USICR = 0;                        // stop the USI
  TCCR0A = 0;                        // hand Timer0 back to the entropy timestamp:
  TCCR0B = _BV(CS00);                // normal mode, free-running at F_CPU
  uint8_t next = (uartRxHead + 1) & 3;
  if (next != uartRxTail) {         // drop it rather than overwrite an unread one
    uartRx[uartRxHead] = USIBR;     // bits arrived LSB-first; rxRead un-reverses
    uartRxHead = next;
  }
  GIFR = _BV(PCIF);                 // ignore any edge seen during the read
  PCMSK = _BV(UART_RX);             // re-arm the start-bit detector
}

static void txBegin()
{
  // One RX bit in Timer0 ticks, at clk/8 or clk/64 if that overflows the 8-bit
  // counter. Constant-folded: RNG_UART_BAUD and F_CPU are known at compile time.
  unsigned long ticks = (F_CPU / 8 + RNG_UART_BAUD / 2) / RNG_UART_BAUD;
  rxPrescaler = _BV(CS01);                 // clk/8
  if (ticks > 256) {
    ticks = (F_CPU / 64 + RNG_UART_BAUD / 2) / RNG_UART_BAUD;
    rxPrescaler = _BV(CS01) | _BV(CS00);   // clk/64
  }
  rxBitTicks = ticks - 1;

  PORTB |= _BV(UART_TX) | _BV(UART_RX);  // TX idle high; RX (USI DI) pull-up
  DDRB |= _BV(UART_TX);                   // TX output, RX stays input
  PCMSK = _BV(UART_RX);                   // pin-change only on RX (the start bit)
  GIFR = _BV(PCIF);
  GIMSK |= _BV(PCIE);
}

static bool rxAvail() { return uartRxHead != uartRxTail; }
static uint8_t rxRead()
{
  uint8_t s = uartRx[uartRxTail];
  uartRxTail = (uartRxTail + 1) & 3;
  uint8_t c = 0;                 // the USI shifted DI in LSB-first: bit 0 is bit 7
  for (uint8_t i = 0; i < 8; i++, s >>= 1)
    c = (c << 1) | (s & 1);
  return c;
}
static void txFlush() {}  // txByte returns only once the byte is on the wire

#else  // USB transport

static bool streaming();       // defined below; needed by txByte
static bool txDropped;         // a stream write hit a stalled reader this burst

static void txBegin()          { SerialUSB.begin(); }
// A stalled reader must never wedge the free-running stream. If it did, loop()
// would stop being re-entered -- the LED would freeze and the three Ctrl-Cs
// into the bootloader would never be read. So in a streaming mode we drop the
// rest of the burst on the first short write, exactly as the binary 'S' path
// does (one burst of random bytes is as good as another); 'x' and the raw
// modes still block, so their credited or finite output is never lost.
static void txByte(uint8_t c)
{
  if (streaming()) {
    if (!txDropped && !SerialUSB.write(c))
      txDropped = true;
  } else {
    while (!SerialUSB.write(c));
  }
}
static void txFlush()          { SerialUSB.flush(); }
static bool rxAvail()          { return SerialUSB.available(); }
static uint8_t rxRead()        { return SerialUSB.read(); }

#endif

static void put(uint8_t c)
{
  txByte(c);  // blocks until the byte is sent (USB: until the buffer has room)
}

static void putNibble(uint8_t n)
{
  put(n < 10 ? '0' + n : 'A' - 10 + n);
}

// Flash string as a '#' comment line; lowercase, so hex parsers skip it.
static void putMessage(const char *s)
{
#if RNG_VENDOR
  // The port belongs to whatever else this sketch carries; the diagnostics
  // these messages used to give are RNG_RQ_INFO requests instead.
  (void)s;
#else
  if (mode == 'X')
    return;  // would corrupt the binary stream
  put('#');
  put(' ');
  while (char c = pgm_read_byte(s++))
    put(c);
  put('\r');
  put('\n');
#endif
}

#if STACK_CHECK
extern uint8_t _end;  // end of .bss, where the heap would start

// Fill free RAM with a marker so the deepest stack use can be found later.
void paintStack() __attribute__((naked, used, section(".init3")));
void paintStack()
{
  for (uint8_t *p = &_end; p < (uint8_t *)SP - 4; p++)
    *p = 0xC5;
}

static void reportStack()
{
  uint16_t untouched = 0;
  for (uint8_t *p = &_end; *p == 0xC5; p++)
    untouched++;
  put('#');
  put(' ');
  for (int8_t shift = 12; shift >= 0; shift -= 4)
    putNibble((untouched >> shift) & 15);
  put('\r');
  put('\n');
}
#endif

static uint8_t burst[BURST_BYTES];
static uint8_t burstLen;       // bytes in burst[]

// The free-running stream, in either spelling ('s' hex, 'S' binary).
static bool streaming() { return mode == 's' || mode == 'S'; }

static bool conditioned()
{
  return streaming() || mode == 'x' || mode == 'X';
}

// The raw modes come in two spellings: uppercase binary (R, D) for the
// assessment tools, lowercase hex (r, d) for reading on a plain terminal.
// Each samples the same source; only the output format differs.
static bool rawIntervals() { return mode == 'r' || mode == 'R'; }
static bool rawAdc()       { return mode == 'd' || mode == 'D'; }
// Which output is hex text: conditioned 'x', and the two lowercase raw modes.
static bool outputHex()    { return mode == 'x' || mode == 'r' || mode == 'd' || mode == 's'; }

static uint8_t burstCapacity()  // in bytes: whole intervals in r/R
{
  return rawIntervals() ? BURST_BYTES / 2 * 2 : BURST_BYTES;
}

static void resetOutput()
{
  burstLen = 0;
}

// ---------------------------------------------------------------- conditioning

static uint8_t state[40];      // Ascon state: 5 little-endian 64-bit words
static uint8_t absorbPos;      // next rate byte to absorb into
static uint16_t credit;        // in 1/64 bit
static bool asconOk;
#if STREAM_MODE
static uint8_t seedFills;      // credited fills so far, up to SEED_FILLS
#endif

static Health intervalHealth, adcHealth;

// SP 800-90B section 4.4 continuous health tests on one sample.
static bool healthy(Health &h, uint8_t sample, uint8_t rctCutoff,
                    uint16_t aptCutoff, const char *failMessage)
{
  if (h.failed)
    return false;
  if (sample == h.last && h.repeats) {
    if (++h.repeats >= rctCutoff)
      h.failed = true;
  } else {
    h.last = sample;
    h.repeats = 1;
  }
  if (h.aptIndex == 0) {
    h.aptSample = sample;
    h.aptCount = 1;
  } else if (sample == h.aptSample && ++h.aptCount >= aptCutoff) {
    h.failed = true;
  }
  if (++h.aptIndex == APT_WINDOW)
    h.aptIndex = 0;
  if (h.failed)
    putMessage(failMessage);
  return !h.failed;
}

static void absorb(uint8_t b)
{
#if NO_ENTROPY_CONTROL
  return;
#endif
  state[absorbPos] ^= b;
  if (++absorbPos == 8) {
    ascon_permute(state, 12);
    absorbPos = 0;
  }
}

static void addCredit(uint8_t amount)
{
  if (credit < CREDIT_PER_OUTPUT)
    credit += amount;
}

// Squeeze 64 bits into the burst once enough entropy has been credited -- or,
// in 'S' mode once seeded, whether or not it has.
static void maybeSqueeze()
{
  if (!asconOk || burstLen == burstCapacity())
    return;
  bool credited = credit >= CREDIT_PER_OUTPUT;
#if STREAM_MODE
  if (!credited && (!streaming() || seedFills < SEED_FILLS))
    return;
#else
  if (!credited)
    return;
#endif
  producing = true;   // validated entropy is now coming out (drives the LED)
  state[39] ^= 0x80;  // domain separation from absorbing
  ascon_permute(state, 12);
  for (uint8_t i = 0; i < 8 && burstLen < burstCapacity(); i++)
    burst[burstLen++] = state[i];
  if (credited) {
    // Forget: 128 bits of the state zeroed, so earlier output cannot be
    // recovered from a later state. Only where fresh entropy paid for it --
    // in 'S' the free-running squeezes between two credited ones are
    // recoverable from a captured state, and that window is one credit's
    // worth of gathering, not the whole stream.
    for (uint8_t i = 0; i < 2; i++) {
      memset(state, 0, 8);
      ascon_permute(state, 12);
    }
    absorbPos = 0;
    credit = 0;
#if STREAM_MODE
    if (seedFills < SEED_FILLS)
      seedFills++;
#endif
  }
}

static void initAscon()
{
  // Ascon-XOF128 IV; P12 of it must give the published initial state.
  static const uint8_t iv[8] PROGMEM = {0x03, 0x00, 0xcc, 0x00, 0x00, 0x08, 0x00, 0x00};
  static const uint8_t expected[8] PROGMEM = {0xeb, 0x47, 0x94, 0x8d, 0x76, 0xce, 0x82, 0xda};
  for (uint8_t i = 0; i < 8; i++)  // loops rather than memcpy_P and memcmp_P,
    state[i] = pgm_read_byte(&iv[i]);  // which nothing else here pulls in
  ascon_permute(state, 12);
  asconOk = true;
  for (uint8_t i = 0; i < 8; i++)
    if (state[i] != pgm_read_byte(&expected[i]))
      asconOk = false;
}

// ------------------------------------------------------------------- sampling

static int8_t readAdc()
{
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC))
    ;
  int16_t v = ADC;
  v = (v & 0x200) ? v - 0x400 : v;  // 10-bit two's complement
  return v > 127 ? 127 : v < -128 ? -128 : v;
}

static void processInterval(uint16_t interval)  // the low 16 bits
{
  if (conditioned()) {
    absorb(interval >> 8);
    absorb(interval);
    if (INTERVAL_CREDIT &&
        healthy(intervalHealth, interval, INTERVAL_RCT_CUTOFF,
                INTERVAL_APT_CUTOFF, PSTR("interval health test failed")))
      addCredit(INTERVAL_CREDIT);
  } else if (rawIntervals()) {
    if (burstLen < burstCapacity()) {
      burst[burstLen++] = interval >> 8;
      burst[burstLen++] = interval;
    }
  }
}

static uint8_t stamps;  // valid entries in loop()'s stamp[], 3 = steady state

// Drop pending captures and start a fresh chain of intervals.
static void restartCapture()
{
  cli();
  ringTail = ringHead;
  overrun = false;
  sei();
  stamps = 0;
}

static void sendBurst()
{
  uint8_t bytes = burstLen;
#if RNG_VENDOR
  // Nothing is sent: the host takes the burst with RNG_RQ_READ. Capture stays
  // stopped until it does, so no sample is taken while the burst waits, and
  // the samples disturbed by the transfer itself are the ones discarded. The
  // framing the serial modes need is redundant here -- the transfer's own
  // length delimits the burst. (Capture restarts when the transfer begins
  // rather than when it ends, which V-USB does not report; burst[] is not
  // written again until a whole burst's worth of entropy has been credited,
  // hundreds of milliseconds against the transfer's few.)
  vendorLen = bytes;
#else
#if !RNG_UART
  txDropped = false;   // fresh drop budget for this burst (CDC stream only)
#endif
  if (outputHex()) {
    // 'x' (conditioned) and the raw hex modes 'R'/'D' all print two hex digits
    // a byte and end the line -- readable on any terminal.
    for (uint8_t i = 0; i < bytes; i++) {
      putNibble(burst[i] >> 4);
      putNibble(burst[i] & 15);
    }
    put('\r');
    put('\n');
#if STACK_CHECK
    reportStack();
#endif
  } else if (mode == 'R' || mode == 'D') {
    put(0xA5);
    put(mode);
    put(bytes);
    for (uint8_t i = 0; i < bytes; i++)
      put(burst[i]);
  } else if (mode == 'X'
#if STREAM_MODE
             || mode == 'S'
#endif
            ) {
#if RNG_UART
    for (uint8_t i = 0; i < bytes; i++)
      put(burst[i]);              // the UART blocks per byte; nothing to batch
#else
    // A block at a time, not a byte: write(uint8_t) waits for the host and
    // services USB on every call, which costs more than the byte is worth
    // when a whole burst is ready.
    //
    // Stop at a short write rather than retrying it. write() returns 0 once
    // the host has stopped draining, and looping on that never ends: loop()
    // is never re-entered, so the three Ctrl-Cs into the bootloader are never
    // read. At 284 bytes/s the host nearly always drains in time and it never
    // showed; at 5650 it wedges the first time a tool writes to the port
    // without reading it, which is exactly what tools/stump.py does to ask
    // for the bootloader. Dropping the rest of a burst costs nothing -- one
    // burst of random bytes is as good as another.
    for (uint8_t sent = 0; sent < bytes; ) {
      uint8_t n = SerialUSB.write(burst + sent, bytes - sent);
      if (!n)
        break;
      sent += n;
    }
#endif
  }
#if STREAM_MODE
  if (streaming()) {
    // No flush and no restart: discarding the samples disturbed by sending is
    // what keeps 'X' honest about its entropy, and 'S' does not make that
    // claim. Stopping for it would cost most of the rate the mode exists for.
    resetOutput();
    return;
  }
#endif
  txFlush();  // sample again only once the port is quiet
  resetOutput();
  restartCapture();
#endif
}

#if RNG_VENDOR
// Answers the vendor requests; runs inside the USB interrupt, so it only
// reads state and sets flags, and loop() does the work.
extern "C" uchar digiCdcVendorSetup(usbRequest_t *rq)
{
  static uint8_t info;
  switch (rq->bRequest) {
  case RNG_RQ_READ:
    if (!vendorLen)
      return 0;                  // nothing ready: the host waits and asks again
    usbMsgPtr = (usbMsgPtr_t)burst;
    vendorTaken = true;          // loop() starts the next capture
    return vendorLen;
  case RNG_RQ_MODE: {
    uint8_t m = rq->wValue.bytes[0];
    if (m == 'X' || m == 'r' || m == 'd' || m == 'R' || m == 'D'
#if STREAM_MODE
        || m == 'S' || m == 's'
#endif
       ) {
      mode = m;
      vendorTaken = true;        // restart under the new mode
    }
    return 0;
  }
  case RNG_RQ_INFO:
    switch (rq->wValue.bytes[0]) {
    case RNG_INFO_PROTOCOL:  info = RNG_PROTOCOL; break;
    case RNG_INFO_MODE:      info = mode; break;
    case RNG_INFO_ASCON:     info = asconOk; break;
    case RNG_INFO_INTERVALS: info = !intervalHealth.failed; break;
    case RNG_INFO_ADC:       info = !adcHealth.failed; break;
    case RNG_INFO_OVERRUNS:  info = overruns; break;
#if STREAM_MODE
    case RNG_INFO_SEEDED:    info = seedFills >= SEED_FILLS; break;
#endif
    default:                 return 0;
    }
    usbMsgPtr = (usbMsgPtr_t)&info;
    return 1;
  case RNG_RQ_BOOT:
    bootWanted = true;           // not from in here: loop() quiets the timers
    return 0;
  }
  return 0;
}
#endif

#if LED_STATUS
// Solid until the RNG is producing, a brief blink a second once it is, a fast
// 50% blink if a source failed. Single-bit PORTB writes are one sbi/cbi each,
// so they cannot race V-USB's own PORTB writes on the USB build.
static void serviceLed()
{
  uint16_t t;
  uint8_t s = SREG;
  cli();
  t = ledTicks;   // 16-bit read the WDT ISR could otherwise tear
  SREG = s;

  bool on;
  if (intervalHealth.failed || adcHealth.failed)
    on = (t % LED_FAST) < (LED_FAST / 2);   // ~5 Hz, 50%: a source failed
  else if (producing)
    on = (t % LED_SLOW) < LED_SLOW_ON;      // ~1 Hz, 10%: generating, healthy
  else
    on = true;                              // solid: not validated, no output
  if (on)
    PORTB |= _BV(LED_PIN);
  else
    PORTB &= ~_BV(LED_PIN);
}
#endif

void setup()
{
  txBegin();
#if MODE_EEPROM
  restoreMode();
#endif
#if LED_STATUS
  DDRB |= _BV(LED_PIN);
  PORTB |= _BV(LED_PIN);   // solid on until the sources validate
#endif

  initAscon();
  if (!asconOk)
    putMessage(PSTR("ascon self-test failed"));

  cli();
  // Timer0: normal mode, no prescaler, no interrupts.
  TCCR0A = 0;
  TCCR0B = _BV(CS00);
  TIMSK &= ~(_BV(OCIE0A) | _BV(OCIE0B) | _BV(TOIE0));

  // ADC: ADC0-ADC0 at 20x, bipolar, Vcc reference, 16.5 MHz / 128 = 129 kHz.
  ADMUX = 0x09;
  ADCSRB = _BV(BIN);
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  // Watchdog: interrupt-only mode (no reset), shortest period (16 ms).
  WDTCR = _BV(WDCE) | _BV(WDE);
  WDTCR = _BV(WDIE);
  sei();
}

void loop()
{
  // stamp[0]: last timestamp already used; stamp[1]: awaiting validation
  static __uint24 stamp[2];
  static bool haveFineOffset;
  static uint8_t fineOffset;

#if LED_STATUS
  serviceLed();
#endif
#if MODE_EEPROM
  persistMode();
#endif

  static uint8_t ctrlCs;
  static unsigned long firstCtrlC;
  if (ctrlCs && millis() - firstCtrlC >= 2000)
    ctrlCs = 0;

#if RNG_VENDOR
  SerialUSB.refresh();  // service USB even when nothing reads the port
  if (bootWanted)
    enterBootloader();
  // Ctrl-C three times still reaches the bootloader from a terminal, so
  // tools/flash.sh works the same in either build.
  if (SerialUSB.available()) {
    char c = SerialUSB.read();
    if (c != 3)
      ctrlCs = 0;
    else if (ctrlCs++ == 0)
      firstCtrlC = millis();
    if (ctrlCs == 3)
      enterBootloader();
#if RNG_CDC_ECHO
    SerialUSB.write(c);
#endif
  }
  if (vendorTaken) {
    vendorTaken = false;
    vendorLen = 0;
    resetOutput();
    restartCapture();
  }
  if (vendorLen)
    return;  // a burst is waiting to be read: take no samples meanwhile
#else
  if (rxAvail()) {  // CDC: also services USB. UART: a byte the PCINT read
    char c = rxRead();
    if (c != 3)
      ctrlCs = 0;
    else if (ctrlCs++ == 0)
      firstCtrlC = millis();
    if (ctrlCs == 3)
      enterBootloader();
    if (c == 'x' || c == 'X' || c == 'r' || c == 'd' || c == 'R' || c == 'D'
#if STREAM_MODE
        || c == 'S' || c == 's'
#endif
       ) {
      mode = c;
      if (mode == 'x') {
        put('\r');
        put('\n');
      }
      resetOutput();
      restartCapture();
    }
  }
#endif

  if (overrun) {
#if RNG_VENDOR
    if (overruns < 255)
      overruns++;
#endif
    restartCapture();
  }

  while (ringTail != ringHead) {
    uint8_t t = ringTail;  // the slot is not written by the ISR until popped
    __uint24 cOvf = ringOvf[t];
    uint8_t cT1 = ringT1[t], cT0 = ringT0[t];
    ringTail = (t + 1) & (RING_SIZE - 1);

    // Timer1 position in CPU cycles; its low byte is always a multiple of 64.
    __uint24 coarse = ((cOvf << 8) | cT1) << 6;
    // TCNT0 minus that is (Timer1 prescaler phase + constant) mod 256, where
    // the phase is 0..63. Centre the first sample at 128 so later ones land
    // in 65..191 and never wrap; the resulting constant offset cancels out
    // in intervals.
    uint8_t phase = cT0 - (uint8_t)coarse;
    if (!haveFineOffset) {
      fineOffset = phase - 128;
      haveFineOffset = true;
    }
    __uint24 timestamp = coarse + (uint8_t)(phase - fineOffset);

    if (stamps < 2) {
      stamp[stamps++] = timestamp;
      continue;  // stamp[0] is never checked, so its interval is not used
    }
    __int24 curvature = (__int24)((timestamp - stamp[1]) - (stamp[1] - stamp[0]));
    if (curvature > (__int24)TIMER1_OVF_CYCLES)
      stamp[1] += (__uint24)TIMER1_OVF_CYCLES;
    if (stamps == 3)
      processInterval(stamp[1] - stamp[0]);  // unsigned, wraps correctly
    else
      stamps = 3;
    stamp[0] = stamp[1];
    stamp[1] = timestamp;
  }

  if (rawAdc()) {
    while (burstLen < burstCapacity())  // ~4 ms, well within USB's limit
      burst[burstLen++] = readAdc();
  } else if (conditioned()) {
#if STREAM_MODE
    uint8_t batch = streaming() ? ADC_BATCH_STREAM : ADC_BATCH;
#else
    const uint8_t batch = ADC_BATCH;
#endif
    for (uint8_t i = 0; i < batch; i++) {
      uint8_t sample = readAdc();
      absorb(sample);
      if (ADC_CREDIT &&
          healthy(adcHealth, sample, ADC_RCT_CUTOFF, ADC_APT_CUTOFF,
                  PSTR("adc health test failed")))
        addCredit(ADC_CREDIT);
    }
#if STREAM_MODE
    // 'S' fills the burst here rather than taking one squeeze per batch of
    // samples: the batch is 16 ADC conversions, about 1.6 ms, and pacing the
    // output to that would hold the stream near the entropy rate, which is
    // the one thing this mode exists not to do.
    if (streaming())
      // At least once, so seeding can progress (seedFills advances only inside
      // maybeSqueeze, on a credited squeeze); then, once seeded, fill the rest
      // of the burst free-running. Booting straight into 'S' relies on this --
      // gating the whole thing on seedFills would never seed from cold.
      do
        maybeSqueeze();
      while (seedFills >= SEED_FILLS && burstLen < burstCapacity());
    else
#endif
      maybeSqueeze();
  }

  if (burstLen == burstCapacity())
    sendBurst();
}
