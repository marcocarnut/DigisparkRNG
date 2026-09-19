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

  Output is hex, 78 digits per line ('x'), or raw bytes ('X').
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
*/

#include <DigiCDCFast.h>
#include <avr/wdt.h>

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

// Output mode, switchable at runtime by sending the character over serial:
//  'x': conditioned random bytes (hex).
//  'X': conditioned random bytes (binary, no messages).
//  'r': raw consecutive intervals in CPU cycles, low 16 bits (binary).
//  'd': raw consecutive ADC readings as signed bytes (binary).
// Binary bursts are framed as 0xA5, mode character, byte count, data.
#if RNG_VENDOR
#define DEFAULT_MODE      'X'  // no hex over libusb: the host formats
#else
#define DEFAULT_MODE      'x'
#endif

// Entropy credit per sample, in 1/64 bit; 0 disables crediting a source.
// SP 800-90B non-IID assessment (ea_non_iid) on 'r'/'d' dumps:
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

extern "C" volatile unsigned long millis_timer_overflow_count;  // wiring.c
extern "C" void ascon_permute(uint8_t *state, uint8_t rounds);  // ascon_permute.S

struct Capture {
  uint32_t ovf;   // Timer1 overflow count
  uint8_t  t1;    // TCNT1 (ticks every 64 CPU cycles)
  uint8_t  t0;    // TCNT0 (ticks every CPU cycle)
};

struct Health {       // SP 800-90B health test state for one source
  uint8_t last, repeats;       // repetition count test
  uint8_t aptSample;           // adaptive proportion test
  uint16_t aptCount, aptIndex;
  bool failed;
};

static char mode = DEFAULT_MODE;
static Capture ring[RING_SIZE];
static volatile uint8_t ringHead, ringTail;
static volatile bool overrun;

ISR(WDT_vect)
{
  // Hardware reads first, in a fixed order, so the latency between them is
  // constant (it folds into the calibration offset below).
  uint8_t t0 = TCNT0;
  uint8_t t1 = TCNT1;
  uint32_t ovf = millis_timer_overflow_count;
  if ((TIFR & _BV(TOV1)) && t1 < 255)  // pending overflow not yet counted
    ovf++;

  uint8_t next = (ringHead + 1) & (RING_SIZE - 1);
  if (next == ringTail) {
    overrun = true;
  } else {
    ring[ringHead].ovf = ovf;
    ring[ringHead].t1 = t1;
    ring[ringHead].t0 = t0;
    ringHead = next;
  }
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
  TIMSK = 0;
  TCCR0B = 0;
  TCCR1 = 0;
  ((void (*)())0)();
}

static void put(uint8_t c)
{
  while (!SerialUSB.write(c))  // write() drops the char when the buffer is full
    ;
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
static uint16_t burstBits;

static bool conditioned()
{
#if STREAM_MODE
  if (mode == 'S')
    return true;
#endif
  return mode == 'x' || mode == 'X';
}

static uint16_t burstCapacity()  // in bits
{
  return mode == 'r' ? BURST_BYTES / 2 * 16 : BURST_BYTES * 8;
}

static void resetOutput()
{
  burstBits = 0;
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
  if (!asconOk || burstBits == burstCapacity())
    return;
  bool credited = credit >= CREDIT_PER_OUTPUT;
#if STREAM_MODE
  if (!credited && (mode != 'S' || seedFills < SEED_FILLS))
    return;
#else
  if (!credited)
    return;
#endif
  state[39] ^= 0x80;  // domain separation from absorbing
  ascon_permute(state, 12);
  for (uint8_t i = 0; i < 8 && burstBits < burstCapacity(); i++, burstBits += 8)
    burst[burstBits >> 3] = state[i];
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
  memcpy_P(state, iv, 8);
  ascon_permute(state, 12);
  asconOk = memcmp_P(state, expected, 8) == 0;
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

static void processInterval(uint32_t interval)
{
  if (conditioned()) {
    absorb(interval >> 8);
    absorb(interval);
    if (INTERVAL_CREDIT &&
        healthy(intervalHealth, interval, INTERVAL_RCT_CUTOFF,
                INTERVAL_APT_CUTOFF, PSTR("interval health test failed")))
      addCredit(INTERVAL_CREDIT);
  } else if (mode == 'r') {
    if (burstBits < burstCapacity()) {
      burst[burstBits >> 3] = interval >> 8;
      burst[(burstBits >> 3) + 1] = interval;
      burstBits += 16;
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
  uint8_t bytes = burstBits / 8;
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
  if (mode == 'r' || mode == 'd') {
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
    // A block at a time, not a byte: write(uint8_t) waits for the host and
    // services USB on every call, which costs more than the byte is worth
    // when a whole burst is ready.
    for (uint8_t sent = 0; sent < bytes; )
      sent += SerialUSB.write(burst + sent, bytes - sent);
  } else {
    for (uint8_t i = 0; i < bytes; i++) {
      putNibble(burst[i] >> 4);
      putNibble(burst[i] & 15);
    }
    put('\r');
    put('\n');
#if STACK_CHECK
    reportStack();
#endif
  }
#if STREAM_MODE
  if (mode == 'S') {
    // No flush and no restart: discarding the samples disturbed by sending is
    // what keeps 'X' honest about its entropy, and 'S' does not make that
    // claim. Stopping for it would cost most of the rate the mode exists for.
    resetOutput();
    return;
  }
#endif
  SerialUSB.flush();  // sample again only once USB is quiet
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
    if (m == 'X' || m == 'r' || m == 'd'
#if STREAM_MODE
        || m == 'S'
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

void setup()
{
  SerialUSB.begin();

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
  static uint32_t stamp[2];
  static bool haveFineOffset;
  static uint8_t fineOffset;

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
  if (SerialUSB.available()) {  // also services USB
    char c = SerialUSB.read();
    if (c != 3)
      ctrlCs = 0;
    else if (ctrlCs++ == 0)
      firstCtrlC = millis();
    if (ctrlCs == 3)
      enterBootloader();
    if (c == 'x' || c == 'X' || c == 'r' || c == 'd'
#if STREAM_MODE
        || c == 'S'
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
    Capture c = ring[ringTail];  // slot is not written by the ISR until popped
    ringTail = (ringTail + 1) & (RING_SIZE - 1);

    // Timer1 position in CPU cycles; its low byte is always a multiple of 64.
    uint32_t coarse = ((c.ovf << 8) | c.t1) << 6;
    // TCNT0 minus that is (Timer1 prescaler phase + constant) mod 256, where
    // the phase is 0..63. Centre the first sample at 128 so later ones land
    // in 65..191 and never wrap; the resulting constant offset cancels out
    // in intervals.
    uint8_t phase = c.t0 - (uint8_t)coarse;
    if (!haveFineOffset) {
      fineOffset = phase - 128;
      haveFineOffset = true;
    }
    uint32_t timestamp = coarse + (uint8_t)(phase - fineOffset);

    if (stamps < 2) {
      stamp[stamps++] = timestamp;
      continue;  // stamp[0] is never checked, so its interval is not used
    }
    int32_t curvature = (timestamp - stamp[1]) - (stamp[1] - stamp[0]);
    if (curvature > TIMER1_OVF_CYCLES)
      stamp[1] += TIMER1_OVF_CYCLES;
    if (stamps == 3)
      processInterval(stamp[1] - stamp[0]);  // unsigned, wraps correctly
    else
      stamps = 3;
    stamp[0] = stamp[1];
    stamp[1] = timestamp;
  }

  if (mode == 'd') {
    while (burstBits < burstCapacity()) {  // ~4 ms, well within USB's limit
      burst[burstBits >> 3] = readAdc();
      burstBits += 8;
    }
  } else if (conditioned()) {
#if STREAM_MODE
    uint8_t batch = mode == 'S' ? ADC_BATCH_STREAM : ADC_BATCH;
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
    if (mode == 'S')
      while (seedFills >= SEED_FILLS && burstBits < burstCapacity())
        maybeSqueeze();
    else
#endif
      maybeSqueeze();
  }

  if (burstBits == burstCapacity())
    sendBurst();
}
