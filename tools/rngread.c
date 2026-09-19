/*
 * rngread -- read clockdrift_rng built with RNG_VENDOR=1, write to stdout.
 *
 *   cc -O2 -o rngread rngread.c -lusb-1.0
 *
 *   ./rngread | pv > /dev/null            # see the rate
 *   ./rngread | RNG_stdin8 stdin8         # straight into PractRand
 *   ./rngread --mode d > adc.bin          # raw ADC samples for SP 800-90B
 *   ./rngread --info                      # what the firmware says about itself
 *
 * The requests are addressed to the device rather than to an interface, so
 * this works while the kernel's cdc_acm driver holds the serial port: nothing
 * here claims an interface. Reading needs permission on the USB device --
 * either run as root or drop a udev rule in, for example
 * /etc/udev/rules.d/60-digispark-rng.rules:
 *
 *   SUBSYSTEM=="usb", ATTR{idVendor}=="16d0", ATTR{idProduct}=="087e", MODE="0666"
 */

#include <errno.h>
#include <time.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#define VID 0x16d0
#define PID 0x087e

/* Must match the sketch. */
enum { RQ_READ = 1, RQ_MODE = 2, RQ_INFO = 3, RQ_BOOT = 4 };
enum { INFO_PROTOCOL, INFO_MODE, INFO_ASCON, INFO_INTERVALS, INFO_ADC,
       INFO_OVERRUNS, INFO_SEEDED };
#define PROTOCOL 1

#define IN  (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | \
             LIBUSB_ENDPOINT_IN)
#define OUT (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | \
             LIBUSB_ENDPOINT_OUT)

static const char *info_name[] = {
    "protocol", "mode", "ascon self-test", "interval source",
    "ADC source", "overruns", "seeded (S mode)",
};

static volatile sig_atomic_t stop;
static void on_signal(int sig) { (void)sig; stop = 1; }

static int get_info(libusb_device_handle *h, int which, unsigned char *out)
{
    return libusb_control_transfer(h, IN, RQ_INFO, which, 0, out, 1, 1000);
}

int main(int argc, char **argv)
{
    int mode = 0, show_info = 0, boot = 0;
    long limit = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--mode") && i + 1 < argc)
            mode = argv[++i][0];
        else if (!strcmp(argv[i], "--bytes") && i + 1 < argc)
            limit = strtol(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--info"))
            show_info = 1;
        else if (!strcmp(argv[i], "--bootloader"))
            boot = 1;
        else {
            fprintf(stderr,
                "usage: %s [--mode X|S|r|d] [--bytes N] [--info] [--bootloader]\n"
                "  X  conditioned random bytes, entropy-backed (default)\n"
                "  S  the same conditioner free-running, as fast as it goes:\n"
                "     unpredictable because Ascon is, not because every bit is\n"
                "     backed by measured entropy. Check 'seeded' and the source\n"
                "     flags with --info before trusting it.\n"
                "  r  raw watchdog intervals, two bytes each, little end first\n"
                "  d  raw ADC readings, one signed byte each\n", argv[0]);
            return 2;
        }
    }

    if (libusb_init(NULL) < 0) {
        fprintf(stderr, "rngread: libusb_init failed\n");
        return 1;
    }

    libusb_device_handle *h = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!h) {
        fprintf(stderr, "rngread: no device %04x:%04x, or no permission to "
                        "open it (see the comment at the top of this file)\n",
                VID, PID);
        libusb_exit(NULL);
        return 1;
    }

    unsigned char v = 0;
    int r = get_info(h, INFO_PROTOCOL, &v);
    if (r != 1) {
        fprintf(stderr, "rngread: the device did not answer an INFO request "
                        "(%s).\nIs it built with RNG_VENDOR=1?\n",
                libusb_error_name(r));
        goto fail;
    }
    if (v != PROTOCOL) {
        fprintf(stderr, "rngread: device speaks protocol %u, this program "
                        "speaks %u\n", v, PROTOCOL);
        goto fail;
    }

    if (boot) {
        libusb_control_transfer(h, OUT, RQ_BOOT, 0, 0, NULL, 0, 1000);
        fprintf(stderr, "rngread: asked it into the bootloader\n");
        goto done;
    }

    if (show_info) {
        for (size_t i = 0; i < sizeof info_name / sizeof *info_name; i++) {
            if (get_info(h, (int)i, &v) != 1) {
                fprintf(stderr, "%-16s unreadable\n", info_name[i]);
                continue;
            }
            if (i == INFO_MODE)
                printf("%-16s %c\n", info_name[i], v);
            else if (i == INFO_PROTOCOL || i == INFO_OVERRUNS)
                printf("%-16s %u\n", info_name[i], v);
            else
                printf("%-16s %s\n", info_name[i], v ? "ok" : "FAILED");
        }
        goto done;
    }

    if (mode) {
        r = libusb_control_transfer(h, OUT, RQ_MODE, mode, 0, NULL, 0, 1000);
        if (r < 0) {
            fprintf(stderr, "rngread: setting mode '%c' failed (%s)\n",
                    mode, libusb_error_name(r));
            goto fail;
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* The burst is at most 39 bytes; ask for more than the firmware can give
     * and take what comes. A short transfer is the whole of what was ready. */
    unsigned char buf[64];
    long total = 0;
    int quiet = 0;

    /* The device stops gathering entropy while a finished burst waits to be
     * collected, so every millisecond of poll latency is entropy not made.
     * Asking constantly is no answer either: each request steals cycles from
     * the very timing the entropy comes from. So learn how long a burst takes
     * and sleep through most of it, then look often for the short remainder.
     * Both numbers come down: fewer requests, and less waiting. */
    long period_us = 150000;        /* first guess, refined below */
    int waited = 0;                 /* empty polls since the last data */
    struct timespec last;
    clock_gettime(CLOCK_MONOTONIC, &last);

    while (!stop && (!limit || total < limit)) {
        r = libusb_control_transfer(h, IN, RQ_READ, 0, 0, buf, sizeof buf, 2000);
        if (r < 0) {
            if (r == LIBUSB_ERROR_NO_DEVICE) {
                fprintf(stderr, "rngread: the device went away\n");
                goto fail;
            }
            fprintf(stderr, "rngread: read failed (%s)\n",
                    libusb_error_name(r));
            goto fail;
        }
        if (r == 0) {
            waited = 1;
            usleep(4000);           /* the short remainder: look often */
            if (++quiet == 2500) {  /* ten seconds */
                fprintf(stderr, "rngread: no data for 10 s; check --info\n");
                quiet = 0;
            }
            continue;
        }
        quiet = 0;

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long delta_us = (now.tv_sec - last.tv_sec) * 1000000L
                      + (now.tv_nsec - last.tv_nsec) / 1000;
        last = now;
        if (!waited) {
            /* Data was already there when we woke: the nap was too long, and
             * the delta cannot say by how much because it contains the nap.
             * Cut it back until we start having to wait again. A stream mode
             * drives this to nothing, which is what it should do. */
            period_us -= period_us / 4;
        } else if (delta_us > 1000 && delta_us < 10000000L) {
            period_us += (delta_us - period_us) / 4;   /* settles in a few bursts */
        }
        waited = 0;
        if (limit && total + r > limit)
            r = (int)(limit - total);
        if (fwrite(buf, 1, (size_t)r, stdout) != (size_t)r) {
            if (errno == EPIPE)
                goto done;          /* the reader closed: not an error */
            perror("rngread: write");
            goto fail;
        }
        /* Flush every burst. stdout is block buffered down a pipe, and at a
         * few hundred bytes a second that leaves anything watching the stream
         * -- pv, or a program waiting on entropy -- with nothing for ten or
         * twenty seconds and then a lump. The write costs nothing next to the
         * wait for the next burst. */
        if (fflush(stdout) && errno == EPIPE)
            goto done;
        total += r;

        /* Sleep through the bulk of the next burst. Short of it deliberately:
         * overshooting wastes what the sleep was meant to save. */
        long nap = period_us * 17 / 20;
        if (nap > 2000 && !stop && (!limit || total < limit))
            usleep((useconds_t)nap);
    }

done:
    fflush(stdout);
    libusb_close(h);
    libusb_exit(NULL);
    return 0;
fail:
    libusb_close(h);
    libusb_exit(NULL);
    return 1;
}
