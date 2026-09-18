#!/usr/bin/env python3
"""Tools for clockdrift_rng raw sample dumps ('r' and 'd' modes).

usage:
  rawdump.py symbols capture.bin r|d out.sym
      Write one byte per sample for SP 800-90B ea_non_iid (8 bits/symbol):
      'd' ADC readings as signed bytes, 'r' low 8 bits of each interval.
  rawdump.py cutoffs H [A]
      SP 800-90B health test cutoffs (alpha = 2^-A, default 40; APT window
      512) for a min-entropy of H bits per sample. The spec allows A from 20
      to 40; use 40 for sources sampled thousands of times per second.
"""
import math
import sys

APT_WINDOW = 512


def bursts(path, kind):
    """Frames are 0xA5, kind, n, n data bytes; accept a frame only when the
    next one starts right after it (or the file ends)."""
    d = open(path, "rb").read()
    i, out = 0, []
    while i + 3 <= len(d):
        if d[i] == 0xA5 and d[i + 1] == ord(kind) and i + 3 + d[i + 2] <= len(d):
            nxt = i + 3 + d[i + 2]
            if nxt == len(d) or d[nxt] == 0xA5:
                out.append(d[i + 3:nxt])
                i = nxt
                continue
        i += 1
    return out


def samples(path, kind):
    if kind == "d":
        return [b for burst in bursts(path, kind) for b in burst]
    return [int.from_bytes(burst[j:j + 2], "big")
            for burst in bursts(path, kind) for j in range(0, len(burst) - 1, 2)]


def rct_cutoff(h, alpha_log2):
    return 1 + math.ceil(alpha_log2 / h)


def apt_cutoff(h, alpha_log2):
    """1 + CRITBINOM(W, 2^-H, 1 - alpha): the smallest count c with
    P(X >= c) <= alpha. Summed from the top, since 1 - 2^-40 is too close
    to 1 for an accumulated CDF to be accurate."""
    p = 2.0 ** -h
    alpha = 2.0 ** -alpha_log2
    tail = 0.0
    for c in range(APT_WINDOW, 0, -1):
        tail += math.comb(APT_WINDOW, c) * p ** c * (1 - p) ** (APT_WINDOW - c)
        if tail > alpha:
            return c + 1
    return 1


def main():
    if len(sys.argv) == 5 and sys.argv[1] == "symbols":
        _, _, path, kind, out = sys.argv
        s = samples(path, kind)
        open(out, "wb").write(bytes(v & 0xFF for v in s))
        print(f"{len(s)} samples written to {out}")
    elif len(sys.argv) in (3, 4) and sys.argv[1] == "cutoffs":
        h = float(sys.argv[2])
        a = int(sys.argv[3]) if len(sys.argv) == 4 else 40
        print(f"H = {h}, alpha = 2^-{a}: RCT cutoff {rct_cutoff(h, a)},"
              f" APT cutoff {apt_cutoff(h, a)} (window {APT_WINDOW})")
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
