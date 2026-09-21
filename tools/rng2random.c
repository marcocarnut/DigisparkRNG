/* rng2random.c -- feed credited entropy from DigisparkRNG into /dev/random.

   Reads n bytes of conditioned output from the RNG's serial port and hands
   them to the kernel with the RNDADDENTROPY ioctl, which *credits* entropy.
   A plain copy into /dev/random (dd .. of=/dev/random) mixes the bytes into
   the pool but credits nothing -- the credit is a separate ioctl. Meant to be
   run from cron. Needs CAP_SYS_ADMIN, so run it as root.

   Usage:  rng2random <device> <nbytes> [credit_bits]
     device       the RNG serial port, e.g. /dev/ttyACM0 (or a USB-serial
                  adapter on the UART build; set its baud with stty first)
     nbytes       entropy bytes to gather, 16 or more
     credit_bits  bits of entropy to credit (default nbytes*8, i.e. full)

   Point it at a CONDITIONED mode: 'x' (hex) or 'X' (binary), where every
   output bit is backed by credited entropy. Do NOT use 's'/'S' (a DRBG
   stream) or 'r'/'d'/'R'/'D' (the raw sources) -- a full credit is not
   warranted there. The output's shape is detected automatically: hex text is
   decoded, binary is taken as raw bytes. The raw modes 'R'/'D' frame their
   output (0xA5, mode, count, data), which is recognised and refused; but 'x'
   vs 's', and 'X' vs 'S', look identical, so those the tool cannot police --
   that is on you.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/random.h>

#define SAMPLE 256   /* bytes sniffed to decide hex vs binary */

static int fd_dev;
static unsigned char rbuf[512];
static int rpos, rlen;

/* One byte from the device, or -1 on a read timeout / EOF. The sniff sample is
   replayed through here first (see main), so nothing read is wasted. */
static int get_byte(void)
{
    if (rpos >= rlen) {
        int n = read(fd_dev, rbuf, sizeof rbuf);
        if (n <= 0)
            return -1;
        rlen = n;
        rpos = 0;
    }
    return rbuf[rpos++];
}

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Decode n bytes from the hex text: two hex digits a byte, '#' comment lines
   and any stray non-hex character skipped. Returns 0, or -1 on timeout. */
static int collect_hex(unsigned char *dst, int n)
{
    int got = 0, hi = -1, comment = 0, bol = 1;
    while (got < n) {
        int c = get_byte();
        if (c < 0) return -1;
        if (c == '\n') { bol = 1; comment = 0; hi = -1; continue; }
        if (c == '\r') continue;
        if (bol) { bol = 0; if (c == '#') comment = 1; }
        if (comment) continue;
        int v = hexval(c);
        if (v < 0) continue;
        if (hi < 0) hi = v;
        else { dst[got++] = (hi << 4) | v; hi = -1; }
    }
    return 0;
}

/* Take n bytes straight off a binary stream. The conditioned binary mode 'X'
   emits raw bytes (as does the DRBG 'S'); only the raw modes 'R'/'D' frame
   their output, and those are turned away before we get here. Returns 0, or -1
   on timeout. */
static int collect_raw(unsigned char *dst, int n)
{
    for (int i = 0; i < n; i++) {
        int c = get_byte();
        if (c < 0) return -1;
        dst[i] = (unsigned char)c;
    }
    return 0;
}

/* Does the sample look like the framed raw output of 'R'/'D' (0xA5, mode,
   count, data)? Conditioned 'X' is raw bytes where 0xA5 is just noise, so this
   marker appears at most by chance; a real R/D stream carries it every frame. */
static int looks_framed(const unsigned char *s, int len)
{
    int hits = 0;
    for (int i = 0; i + 2 < len; i++)
        if (s[i] == 0xA5 && (s[i+1] == 'R' || s[i+1] == 'D')
            && s[i+2] >= 1 && s[i+2] <= 64)
            hits++;
    return hits >= 2;
}

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        fprintf(stderr,
            "usage: %s <device> <nbytes> [credit_bits]\n"
            "  read nbytes of conditioned RNG output ('x' or 'X' mode) and add it\n"
            "  to /dev/random with an entropy credit. Needs root (CAP_SYS_ADMIN).\n",
            argv[0]);
        return 2;
    }
    const char *dev = argv[1];
    int nbytes = atoi(argv[2]);
    if (nbytes < 16) {
        fprintf(stderr, "%s: nbytes must be 16 or more\n", argv[0]);
        return 2;
    }
    int credit = (argc == 4) ? atoi(argv[3]) : nbytes * 8;
    if (credit < 0) credit = 0;
    if (credit > nbytes * 8) credit = nbytes * 8;   /* never claim >8 bits a byte */

    fd_dev = open(dev, O_RDWR | O_NOCTTY);
    if (fd_dev < 0) {
        fprintf(stderr, "%s: open %s: %s\n", argv[0], dev, strerror(errno));
        return 1;
    }
    struct termios t;
    if (tcgetattr(fd_dev, &t) == 0) {
        cfmakeraw(&t);              /* 8N1, no echo -- echo would feed mode chars back */
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 20;         /* 2 s per-read timeout, a stall guard */
        tcsetattr(fd_dev, TCSANOW, &t);
    }

    /* Sniff a sample into rbuf (which is also the replay buffer, so nothing is
       copied or lost) to decide hex vs binary, then let get_byte() replay it. */
    int slen = 0, hexish = 0;
    while (slen < SAMPLE) {
        int n = read(fd_dev, rbuf + slen, SAMPLE - slen);
        if (n <= 0) break;
        slen += n;
    }
    if (slen == 0) {
        fprintf(stderr, "%s: no data from %s -- is the RNG streaming?\n", argv[0], dev);
        return 1;
    }
    for (int i = 0; i < slen; i++) {
        int c = rbuf[i];
        if (hexval(c) >= 0 || c == '\r' || c == '\n' || c == '#' || c == ' ')
            hexish++;
    }
    int is_hex = (hexish * 100 >= slen * 90);   /* random binary is ~9% hex-ish */
    if (!is_hex && looks_framed(rbuf, slen)) {
        fprintf(stderr, "%s: %s looks like a framed raw mode ('R'/'D'), not the "
                "conditioned 'X' -- switch it to 'X' or 'x'\n", argv[0], dev);
        return 1;
    }
    rlen = slen;
    rpos = 0;

    /* Gather straight into the pool-info buffer -- no bounce buffer, no copy. */
    struct rand_pool_info *info = malloc(sizeof *info + nbytes);
    info->entropy_count = credit;
    info->buf_size = nbytes;
    unsigned char *data = (unsigned char *)info->buf;

    int rc = is_hex ? collect_hex(data, nbytes) : collect_raw(data, nbytes);
    if (rc < 0) {
        fprintf(stderr, "%s: timed out gathering %d bytes from %s\n", argv[0], nbytes, dev);
        return 1;
    }

    int rnd = open("/dev/random", O_WRONLY);
    if (rnd < 0) {
        fprintf(stderr, "%s: open /dev/random: %s\n", argv[0], strerror(errno));
        return 1;
    }
    if (ioctl(rnd, RNDADDENTROPY, info) < 0) {
        fprintf(stderr, "%s: RNDADDENTROPY: %s%s\n", argv[0], strerror(errno),
                errno == EPERM ? " (needs root / CAP_SYS_ADMIN)" : "");
        return 1;
    }
    fprintf(stderr, "%s: added %d %s bytes to /dev/random, credited %d bits\n",
            argv[0], nbytes, is_hex ? "hex" : "binary", credit);
    return 0;
}
