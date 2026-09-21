# tools/

Most of these are described in the main [README](../README.md) -- building,
flashing, capturing, and the SP 800-90B assessment. One topic is worth its own
note, because the obvious thing to reach for is usually more than you need.

## Feeding the kernel's entropy pool

You can pipe the RNG into `/dev/random` so the system CRNG mixes in real
hardware entropy. **For normal use on a current kernel, a plain copy once a
minute is more than enough** -- no special tool, no root (`/dev/random` is
world-writable). Leave the board in a conditioned mode (`x` or `X`) and add to
your crontab:

    * * * * *  bash -c 'dd if=/dev/ttyACM0 bs=1 count=32 of=/dev/random 2>/dev/null'

That *mixes* 32 fresh bytes into the pool every minute, which contributes to the
CRNG state. It does not *credit* entropy -- that is a separate ioctl -- but on a
modern kernel the credit does nothing anyway.

### Why the credit no longer matters

Since Linux 5.18 (2022, the random-subsystem rewrite) the generator is always
seeded once early boot is past, `/dev/random` no longer blocks, and
`/proc/sys/kernel/random/entropy_avail` simply sits at 256. **Mixing** (a plain
write) still adds your bytes to the CRNG state and is worth doing; **crediting**
(`RNDADDENTROPY`) changes nothing you can observe. So the one-liner above is the
whole story for a normal desktop or server.

### rng2random.c -- the credited feed, for when it still matters

`rng2random.c` reads conditioned output and adds it with the `RNDADDENTROPY`
ioctl, which credits the entropy as well as mixing it (and so needs root --
`CAP_SYS_ADMIN`). That credit earns its keep only in the narrow cases the
one-liner does not cover:

- **older kernels** (pre-5.18, still common on embedded and LTS systems), where
  `/dev/random` genuinely blocked until `entropy_avail` was high enough;
- **early-boot seeding** on an entropy-starved headless board -- though that
  wants running from initramfs, not cron, which is already too late;
- **policy or compliance** that wants the hardware contribution explicitly
  counted rather than merely stirred in.

Build and use:

    cc -O2 -o tools/rng2random tools/rng2random.c
    sudo tools/rng2random /dev/ttyACM0 32        # add 32 credited bytes (256 bits)

Point it at a *conditioned* mode (`x` hex or `X` binary): there every output bit
is backed by credited entropy. It refuses the raw modes `R`/`D` (it recognises
their `0xA5, mode, count` framing) but cannot tell `x` from `s` or `X` from `S`,
so that part is on you. It detects hex vs binary from the data's shape, decoding
hex and taking binary as raw bytes, and credits 8 bits a byte by default
(override with a third argument).
