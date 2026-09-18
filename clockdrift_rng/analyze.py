#!/usr/bin/env python3
"""Quick statistics for clockdrift_rng hex output.

usage: analyze.py capture.txt [seconds]
"""
import math
import sys
from collections import Counter


def load_bits(path):
    bits = []
    for line in open(path, errors="replace"):
        line = line.strip()
        if line and all(c in "0123456789ABCDEF" for c in line):
            for c in line:
                v = int(c, 16)
                bits.extend((v >> s) & 1 for s in (3, 2, 1, 0))
    return bits


def autocorr(bits, lag):
    n = len(bits) - lag
    m = sum(bits) / len(bits)
    var = m * (1 - m)
    return sum((bits[i] - m) * (bits[i + lag] - m) for i in range(n)) / n / var


def report(name, bits, seconds=None):
    n = len(bits)
    if n < 16:
        print(f"{name}: too few bits ({n})")
        return
    ones = sum(bits)
    p = ones / n
    z = (ones - n / 2) / math.sqrt(n / 4)
    print(f"== {name}: {n} bits" + (f", {n / seconds:.1f} bits/s" if seconds else ""))
    print(f"  P(1) = {p:.4f}   (z = {z:+.2f})")
    pairs = Counter(tuple(bits[i:i + 2]) for i in range(n - 1))
    print("  overlapping pairs:",
          "  ".join(f"{a}{b}={pairs[(a, b)] / (n - 1):.3f}" for a in (0, 1) for b in (0, 1)))
    print("  autocorrelation lag 1..4:",
          " ".join(f"{autocorr(bits, k):+.3f}" for k in range(1, 5)),
          f"  (noise ~ +/-{2 / math.sqrt(n):.3f})")
    runs = 1 + sum(bits[i] != bits[i + 1] for i in range(n - 1))
    exp_runs = 2 * n * p * (1 - p) + 1
    print(f"  runs = {runs}, expected {exp_runs:.0f} for independent bits")
    h_min = -math.log2(max(p, 1 - p))
    print(f"  min-entropy per bit from bias alone: {h_min:.4f} (ignores correlation)")


def von_neumann(bits):
    return [a for a, b in zip(bits[0::2], bits[1::2]) if a != b]


def position_report(path, line_length=78, segments=6):
    """P(1) by position within full-length lines (buffered mode: one burst each)."""
    lines = [l.strip() for l in open(path, errors="replace")]
    lines = [l for l in lines if len(l) == line_length and all(c in "0123456789ABCDEF" for c in l)]
    if len(lines) < 10:
        return
    width = line_length * 4 // segments
    ps = []
    for s in range(segments):
        seg = [(int(l[i // 4], 16) >> (3 - i % 4)) & 1 for l in lines for i in range(s * width, (s + 1) * width)]
        ps.append(sum(seg) / len(seg))
    noise = 2 * math.sqrt(0.25 / (len(lines) * width))
    print(f"  P(1) by position in line, {segments} segments:",
          " ".join(f"{p:.3f}" for p in ps), f"  (noise ~ +/-{noise:.3f})")


def main():
    bits = load_bits(sys.argv[1])
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else None
    report("raw", bits, seconds)
    position_report(sys.argv[1])
    vn = von_neumann(bits)
    report("Von Neumann", vn, seconds)
    if len(vn) >= 8 * 256 * 5:  # >= 5 expected per byte value
        data = bytes(int("".join(map(str, vn[i:i + 8])), 2) for i in range(0, len(vn) - 7, 8))
        counts = Counter(data)
        exp = len(data) / 256
        chi2 = sum((counts[v] - exp) ** 2 / exp for v in range(256))
        print(f"  byte chi-square = {chi2:.1f} (255 dof, expect ~255 +/- 23)")


if __name__ == "__main__":
    main()
