#!/usr/bin/env python3
"""Talk to the clockdrift_rng Digispark over its DigiCDC serial port.

usage:
  stump.py capture MODE SECONDS OUT   select MODE (x/X/r/d), save output to OUT
  stump.py bootloader                 Ctrl-C x3: jump to the bootloader
  stump.py setbaud RATE               set the port's bit rate (134: bridge -> bootloader)
  stump.py reboot                     bootloader, then back to the sketch
  stump.py wait [SECONDS]             wait for the serial port to appear
"""
import os
import select
import sys
import termios
import time
import tty

PORT = "/dev/ttyACM0"


def open_port():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    time.sleep(0.5)
    return fd


def capture(mode, seconds, out):
    fd = open_port()
    os.write(fd, mode.encode())
    n = 0
    end = time.time() + seconds
    with open(out, "wb", buffering=0) as f:  # unbuffered: survives a kill
        while time.time() < end:
            ready, _, _ = select.select([fd], [], [], 0.5)
            if ready:
                data = os.read(fd, 256)
                f.write(data)
                n += len(data)
    os.close(fd)
    print(f"{n} bytes in {seconds:.0f} s -> {out}")


def bootloader():
    fd = open_port()
    os.write(fd, b"\x03\x03\x03")
    time.sleep(0.2)
    try:
        os.close(fd)
    except OSError:  # the device may already be gone
        pass


def wait(limit):
    t0 = time.time()
    while not os.path.exists(PORT):
        if time.time() - t0 > limit:
            sys.exit(f"no {PORT} after {limit:.0f} s")
        time.sleep(0.5)
    time.sleep(1)
    print(f"{PORT} after {time.time() - t0:.1f} s")


def setbaud(rate):
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = getattr(termios, f"B{rate}")
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    time.sleep(0.3)
    try:
        os.close(fd)
    except OSError:
        pass


def reboot():
    bootloader()
    t0 = time.time()
    while os.path.exists(PORT) and time.time() - t0 < 10:
        time.sleep(0.2)
    wait(60)


def main():
    args = sys.argv[1:]
    if len(args) == 4 and args[0] == "capture":
        capture(args[1], float(args[2]), args[3])
    elif args == ["bootloader"]:
        bootloader()
    elif len(args) == 2 and args[0] == "setbaud":
        setbaud(int(args[1]))
    elif args == ["reboot"]:
        reboot()
    elif args and args[0] == "wait" and len(args) <= 2:
        wait(float(args[1]) if len(args) == 2 else 180)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
