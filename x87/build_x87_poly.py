#!/usr/bin/env python3
"""Generate the x87 transcendental programs (x87_poly_rom.svh).

FSIN, FCOS, FSINCOS, FPTAN, FPATAN, F2XM1, FYL2X and FYL2XP1 run as short
programs of the executor's own operations (multiply, add, subtract, divide,
round to integer, compare), sequenced by x87_control's polynomial engine. Each
operation rounds to the executor's 53-bit significand, round to nearest; the
last one rounds per the control word. The engine holds temporaries in a
16-entry scratch and reads constants from a 128-entry table. An operand can
be truncated to its top 13 significand bits (a product of it with a constant
of up to 40 bits is exact), and XTR splits a positive number into 2^k * m in
control logic (no executor operation).

  sin/cos: Cody-Waite reduction by pi/2 in four parts (n * part exact for
           |n| < 2^20), then the fdlibm kernels __kernel_sin/__kernel_cos;
           the quadrant (n mod 4) selects and signs the result.
  atan2:   t = min(|y|,|x|) / max(|y|,|x|), fdlibm's atan reduction
           (thresholds 7/16 and 11/16, c = 0, 1/2, 1), its polynomial, then
           pi/2 - r when |y| > |x|, pi - r when x < 0, and y's sign.
  2^x - 1: k = 0 for |x| < 1/2, else k = +-1 (a compare), f = x - k exact;
           2^f - 1 = h + T, h = trunc(f) * ln2_hi exact,
           T = f^2 * Q(f) + ((f - trunc(f)) * ln2_hi + f * ln2_lo), Q a
           degree-10 fit of (2^f - 1 - f ln2) / f^2 on [-1/2, 1/2]. k = 1 as fdlibm's
           expm1: 2 * ((h + 1/2) + T) for h < -1/4, else 1 + 2 * (h + T);
           k = -1: (h + T) / 2 - 1/2. |x| < 2^-60: x * ln2 (one rounding).
  y*log2(x): x = 2^k * m with sqrt(1/2) <= m < sqrt(2) (XTR), f = m - 1
           exact; fdlibm's log kernel (s = f / (2 + f), R(s^2) with Lg1..Lg7)
           and e_log2's split: hi = f - f^2/2 truncated to 13 bits, lo the
           rest, log2(m) = LH + LL with LH = hi * ivln2_hi exact. For k != 0,
           LH becomes k + LH rounded to 2^-25 (summed exactly through
           2^27 + 2^15 + k, which XTR builds) and the remainder joins LL.
           Then y * (LH + LL) = trunc(y) * LH (exact) + ((y - trunc(y)) * LH
           + y * LL), one final rounding.
  y*log2(1+x): FYL2XP1 with |x| < 1 - sqrt(2)/2 (Intel's domain) runs the
           log kernel on f = x (k = 0), never forming 1 + x; outside it
           (undefined on Intel) FYL2X of 1 + x. |x| < 2^-60:
           log2(1 + x) = x / ln2 in the same hi/lo form.

The script models every program exactly (53-bit round-to-nearest operations
with an unbounded exponent, as the executor computes within its range),
checks them against mpmath and fails if an error exceeds the limit, and can
write instruction-level tests whose expected bit patterns come from the model.
"""

from __future__ import annotations

import argparse
import math
import random
import struct
from pathlib import Path

import mpmath
from mpmath import libmp

mpmath.mp.prec = 200

# --------------------------------------------------------------------------
# Constants (all exactly representable doubles)
# --------------------------------------------------------------------------


def rn(x) -> float:
    """mpmath number rounded to the nearest double."""
    return float(mpmath.mpf(x))


def trunc_bits(x, bits: int) -> float:
    """x truncated to `bits` significant bits."""
    m, e = mpmath.frexp(mpmath.mpf(x))
    scaled = mpmath.floor(m * 2**bits)
    return float(mpmath.ldexp(scaled, e - bits))


PI = mpmath.pi
PIO2 = PI / 2

# pi/2 in four parts: three 33-bit heads (products with |n| < 2^20 exact)
# and a double tail.
P1 = trunc_bits(PIO2, 33)
P2 = trunc_bits(PIO2 - P1, 33)
P3 = trunc_bits(PIO2 - P1 - P2, 33)
P4 = rn(PIO2 - P1 - P2 - P3)


def hilo(x) -> tuple[float, float]:
    hi = rn(x)
    return hi, rn(mpmath.mpf(x) - hi)


def h2d(h: int) -> float:
    return struct.unpack(">d", h.to_bytes(8, "big"))[0]


# fdlibm k_sin.c / k_cos.c / s_atan.c coefficients.
S = [h2d(v) for v in (0xBFC5555555555549, 0x3F8111111110F8A6, 0xBF2A01A019C161D5,
                      0x3EC71DE357B1FE7D, 0xBE5AE5E68A2B9CEB, 0x3DE5D93A5ACFD57C)]
C = [h2d(v) for v in (0x3FA555555555554C, 0xBF56C16C16C15177, 0x3EFA01A019CB1590,
                      0xBE927E4F809C52AD, 0x3E21EE9EBDB4B1C4, 0xBDA8FAE9BE8838D4)]
AT = [h2d(v) for v in (0x3FD555555555550D, 0xBFC999999998EBC4, 0x3FC24924920083FF,
                       0xBFBC71C6FE231671, 0x3FB745CDC54C206E, 0xBFB3B0F2AF749A6D,
                       0x3FB10D66A0D03D51, 0xBFADDE2D52DEFD9A, 0x3FA97B4B24760DEB,
                       0xBFA2B4442C6A6C2F, 0x3F90AD3AE322DA11)]
# atan2's result is base + atan(c) or base - atan(c), plus or minus the
# kernel's atan(t'), with the quadrant folded into the hi/lo constants:
#   A: |y| <= |x|, x >= 0:  atan(c)        + atan(t')
#   B: |y| >  |x|, x >= 0:  pi/2 - atan(c) - atan(t')
#   C: |y| <= |x|, x <  0:  pi - atan(c)   - atan(t')
#   D: |y| >  |x|, x <  0:  pi/2 + atan(c) + atan(t')
ATAN_C = [mpmath.mpf(0), mpmath.atan(mpmath.mpf(1) / 2), PI / 4]
ATAN_TABLES = {
    "A": [hilo(a) for a in ATAN_C],
    "B": [hilo(PIO2 - a) for a in ATAN_C],
    "C": [hilo(PI - a) for a in ATAN_C],
    "D": [hilo(PIO2 + a) for a in ATAN_C],
}

LN2 = mpmath.log(2)
IVLN2 = 1 / LN2
# 2^f - 1: ln2 = LN2HI (40 bits: trunc(f) * LN2HI is exact) + LN2LO.
LN2HI = trunc_bits(LN2, 40)
LN2LO = rn(LN2 - LN2HI)
# E2..E12: mpmath.chebyfit of (2^f - 1 - f * (LN2HI + LN2LO)) / f^2 on
# [-1/2, 1/2], degree 10 (fit error 2^-60.4; 2^-56.9 relative to 2^f - 1
# with the coefficients rounded).
E = [h2d(v) for v in (0x3FCEBFBDFF82C58F, 0x3FAC6B08D704A0C2, 0x3F83B2AB6FBA4E78,
                      0x3F55D87FE78A5276, 0x3F2430912F86BF52, 0x3EEFFCBFC670DCD4,
                      0x3EB62C022401B97D, 0x3E7B524FAE627834, 0x3E3E4CF1CD08D4B8,
                      0x3DFE9D3FE3952179, 0x3DBC48F769AD2C46)]
# fdlibm e_log.c's Lg1..Lg7.
LG = [h2d(v) for v in (0x3FE5555555555593, 0x3FD999999997FA04, 0x3FD2492494229359,
                       0x3FCC71C51D8E78AF, 0x3FC7466496CB03DE, 0x3FC39A09D078C69F,
                       0x3FC2F112DF3E5244)]
# 1/ln2 = IVLN2HI (27 bits: a 13-bit hi times it is exact) + IVLN2LO.
IVLN2HI = trunc_bits(IVLN2, 27)
IVLN2LO = rn(IVLN2 - IVLN2HI)
# XTR's sum constant: N = 2^27 + 2^15 + k has ulp 2^-25.
KBASE = 2.0**27 + 2.0**15
SQRT2_SIG = int(mpmath.floor(mpmath.sqrt(2) * 2**52))   # m >= sqrt(2) halves

CONSTS: list[tuple[str, float]] = [
    ("TWO_OVER_PI", rn(2 / PI)),
    ("P1", P1), ("P2", P2), ("P3", P3), ("P4", P4),
    *[(f"S{i + 1}", v) for i, v in enumerate(S)],
    *[(f"C{i + 1}", v) for i, v in enumerate(C)],
    ("HALF", 0.5), ("ONE", 1.0),
    ("K7_16", 7 / 16), ("K11_16", 11 / 16),
    # Indexed by the atan reduction id (0, 1, 2): keep each group adjacent.
    ("CT0", 0.0), ("CT1", 0.5), ("CT2", 1.0),
    *[(f"HI{q}{i}", hl[0]) for q, tab in ATAN_TABLES.items() for i, hl in enumerate(tab)],
    *[(f"LO{q}{i}", hl[1]) for q, tab in ATAN_TABLES.items() for i, hl in enumerate(tab)],
    *[(f"AT{i}", v) for i, v in enumerate(AT)],
    # F2XM1, FYL2X, FYL2XP1
    ("TWO", 2.0), ("MQUARTER", -0.25),
    ("LN2", rn(LN2)), ("LN2HI", LN2HI), ("LN2LO", LN2LO),
    *[(f"E{i + 2}", v) for i, v in enumerate(E)],
    *[(f"LG{i + 1}", v) for i, v in enumerate(LG)],
    ("IVLN2", rn(IVLN2)), ("IVLN2HI", IVLN2HI), ("IVLN2LO", IVLN2LO),
    ("KBASE", KBASE), ("LOG1P_LIM", rn(1 - mpmath.sqrt(2) / 2)),
]
CONST_INDEX = {name: i for i, (name, _) in enumerate(CONSTS)}
CONST_BITS = 7
assert len(CONSTS) <= 1 << CONST_BITS

# --------------------------------------------------------------------------
# Program
# --------------------------------------------------------------------------

OPS = {"MUL": 0, "ADD": 1, "SUB": 2, "DIV": 3, "RND": 4, "CMP": 5, "AUX": 6, "XTR": 7}
PREDS = ["ALWAYS", "Q0", "Q1", "Q2", "Q3", "Q0S", "Q1S", "Q2S", "Q3S",
         "Q0C", "Q1C", "Q2C", "Q3C", "SWAP", "NOSWAP", "YNEG", "QA", "QB", "QC", "QD",
         "QA0", "QA1", "KNZ"]
PRED = {name: i for i, name in enumerate(PREDS)}
SETP = {"NONE": 0, "SWAP": 1, "ID1": 2, "ID2": 3}   # CMP: SWAP = a < b, IDn = a >= b
SLOTS = ["IN_A", "IN_B", "N", "R", "Z", "T1", "P", "Q", "S", "C", "H",
         "T", "U", "V", "W", "A2"]
SLOT = {name: i for i, name in enumerate(SLOTS)}
assert not set(SLOTS) & {name for name, _ in CONSTS}, "slot and constant names overlap"
TRUNC_BITS = 13                 # a_trunc/b_trunc keep sig[52:40]


class Op:
    def __init__(self, op, a=None, b=None, dest="T1", pred="ALWAYS", final=False,
                 setp="NONE", a_neg=False, a_abs=False, b_abs=False, a_id=False, b_id=False,
                 a_trunc=False, b_trunc=False):
        self.op, self.a, self.b, self.dest = op, a, b, dest
        self.pred, self.final, self.setp = pred, final, setp
        self.a_neg, self.a_abs, self.b_abs, self.a_id, self.b_id = a_neg, a_abs, b_abs, a_id, b_id
        self.a_trunc, self.b_trunc = a_trunc, b_trunc


def horner(acc: str, var: str, coeffs: list[str]) -> list[Op]:
    """acc = coeffs[0] + var*(coeffs[1] + ... + var*coeffs[-1])."""
    ops = [Op("MUL", var, coeffs[-1], acc)]
    for k in reversed(coeffs[:-1]):
        ops.append(Op("ADD", acc, k, acc))
        if k != coeffs[0]:
            ops.append(Op("MUL", acc, var, acc))
    return ops


SINCOS = [
    Op("MUL", "IN_A", "TWO_OVER_PI", "T1"),
    Op("RND", "T1", None, "N"),
    # A = x - n*P1 is exact; the tail n*(P2 + P3 + P4) is subtracted once,
    # and RLO = (A - R) - tail keeps what that rounding dropped (r = R + RLO).
    Op("MUL", "N", "P1", "T1"), Op("SUB", "IN_A", "T1", "U"),
    Op("MUL", "N", "P2", "T1"), Op("MUL", "N", "P3", "W"), Op("ADD", "T1", "W", "T1"),
    Op("MUL", "N", "P4", "W"), Op("ADD", "T1", "W", "T1"),
    Op("SUB", "U", "T1", "R"), Op("SUB", "U", "R", "W"), Op("SUB", "W", "T1", "V"),
    Op("MUL", "R", "R", "Z"),
    # sin(r) = R + (RLO + R*z*(S1 + z*(S2 + ... + z*S6)))
    *horner("P", "Z", ["S1", "S2", "S3", "S4", "S5", "S6"]),
    Op("MUL", "R", "Z", "T1"), Op("MUL", "T1", "P", "T1"), Op("ADD", "T1", "V", "T1"),
    Op("ADD", "R", "T1", "S"),
    # cos(r) = 1 - (z/2 - (z*z*(C1 + z*(C2 + ... + z*C6)) - R*RLO))
    *horner("Q", "Z", ["C1", "C2", "C3", "C4", "C5", "C6"]),
    Op("MUL", "Z", "Z", "T1"), Op("MUL", "T1", "Q", "T1"),
    Op("MUL", "R", "V", "W"), Op("SUB", "T1", "W", "T1"),
    Op("MUL", "Z", "HALF", "H"), Op("SUB", "H", "T1", "T1"), Op("SUB", "ONE", "T1", "C"),
    # cos(x) for FPTAN/FSINCOS, then sin(x) or cos(x) by quadrant.
    Op("AUX", "C", pred="Q0"), Op("AUX", "S", pred="Q1", a_neg=True),
    Op("AUX", "C", pred="Q2", a_neg=True), Op("AUX", "S", pred="Q3"),
    Op("MUL", "S", "ONE", pred="Q0S", final=True),
    Op("MUL", "C", "ONE", pred="Q1S", final=True),
    Op("MUL", "S", "ONE", pred="Q2S", final=True, a_neg=True),
    Op("MUL", "C", "ONE", pred="Q3S", final=True, a_neg=True),
    Op("MUL", "C", "ONE", pred="Q0C", final=True),
    Op("MUL", "S", "ONE", pred="Q1C", final=True, a_neg=True),
    Op("MUL", "C", "ONE", pred="Q2C", final=True, a_neg=True),
    Op("MUL", "S", "ONE", pred="Q3C", final=True),
]

# FPATAN: IN_A = Y (ST1), IN_B = X (ST0).
ATAN = [
    Op("CMP", "IN_B", "IN_A", None, setp="SWAP", a_abs=True, b_abs=True),
    Op("DIV", "IN_A", "IN_B", "T", pred="NOSWAP", a_abs=True, b_abs=True),
    Op("DIV", "IN_B", "IN_A", "T", pred="SWAP", a_abs=True, b_abs=True),
    Op("CMP", "T", "K7_16", None, setp="ID1"),
    Op("CMP", "T", "K11_16", None, setp="ID2"),
    Op("SUB", "T", "CT0", "U", b_id=True),
    Op("MUL", "T", "CT0", "V", b_id=True),
    Op("ADD", "V", "ONE", "V"),
    Op("DIV", "U", "V", "T"),
    Op("MUL", "T", "T", "Z"), Op("MUL", "Z", "Z", "W"),
    *horner("P", "W", ["AT0", "AT2", "AT4", "AT6", "AT8", "AT10"]),
    Op("MUL", "P", "Z", "P"),
    *horner("A2", "W", ["AT1", "AT3", "AT5", "AT7", "AT9"]),
    Op("MUL", "A2", "W", "A2"),
    Op("ADD", "P", "A2", "S"), Op("MUL", "T", "S", "S"),
    # Kernel added (A, D): R = HI - ((t'*s - LO) - t'); subtracted (B, C):
    # R = HI - ((t' - t'*s) - LO).
    *[o for q in "AD" for o in (
        Op("SUB", "S", f"LO{q}0", "S", pred=f"Q{q}", b_id=True),
        Op("SUB", "S", "T", "S", pred=f"Q{q}"),
        Op("SUB", f"HI{q}0", "S", "R", pred=f"Q{q}", a_id=True))],
    *[o for q in "BC" for o in (
        Op("SUB", "T", "S", "S", pred=f"Q{q}"),
        Op("SUB", "S", f"LO{q}0", "S", pred=f"Q{q}", b_id=True),
        Op("SUB", f"HI{q}0", "S", "R", pred=f"Q{q}", a_id=True))],
    Op("MUL", "R", "ONE", pred="YNEG", final=True, a_neg=True),
    Op("MUL", "R", "ONE", final=True),
]

# F2XM1: IN_A = IN_B = x, 2^-60 <= |x| < 1 (x87_executor classifies the rest);
# xneg = x < 0. f = x - k overwrites IN_A.
EXP = [
    Op("CMP", "IN_A", "HALF", None, setp="SWAP", a_abs=True),       # swap: k = 0
    Op("SUB", "IN_A", "ONE", "IN_A", pred="QA"),                     # k = 1
    Op("ADD", "IN_A", "ONE", "IN_A", pred="QC"),                     # k = -1
    Op("SUB", "IN_A", "IN_A", "V", b_trunc=True),                    # f - trunc(f)
    *horner("P", "IN_A", [f"E{i}" for i in range(2, 13)]),
    Op("MUL", "P", "IN_A", "P"),
    Op("MUL", "P", "IN_A", "P"),                                     # f^2 Q(f)
    Op("MUL", "V", "LN2HI", "V"),
    Op("MUL", "IN_A", "LN2LO", "W"),
    Op("ADD", "V", "W", "V"),
    Op("ADD", "P", "V", "T"),                                        # T
    Op("MUL", "IN_A", "LN2HI", "H", a_trunc=True),                   # h (exact)
    Op("ADD", "H", "T", pred="SWAP", final=True),
    Op("ADD", "H", "T", "W", pred="QC"),
    Op("MUL", "W", "HALF", "W", pred="QC"),
    Op("SUB", "W", "HALF", pred="QC", final=True),
    Op("CMP", "H", "MQUARTER", None, setp="ID1", pred="QA"),         # id: h >= -1/4
    Op("ADD", "H", "HALF", "W", pred="QA0"),
    Op("ADD", "W", "T", "W", pred="QA0"),
    Op("MUL", "W", "TWO", pred="QA0", final=True),
    Op("ADD", "H", "T", "W", pred="QA1"),
    Op("MUL", "W", "TWO", "W", pred="QA1"),
    Op("ADD", "W", "ONE", pred="QA1", final=True),
]
EXP_TINY = [Op("MUL", "IN_A", "LN2", final=True)]                   # |x| < 2^-60

# FYL2X / FYL2XP1: IN_A = y (ST1), IN_B = x (ST0). The kernel takes f in IN_B.
LOG = [
    # FYL2XP1: |x| < 1 - sqrt(2)/2 keeps f = x (k = 0); else FYL2X of 1 + x.
    Op("CMP", "IN_B", "LOG1P_LIM", None, setp="SWAP", a_abs=True),
    Op("ADD", "IN_B", "ONE", "IN_B", pred="NOSWAP"),
    # FYL2X: C = m, N = 2^27 + 2^15 + k; f = m - 1.
    Op("XTR", "IN_B", "N", "C", pred="NOSWAP"),
    Op("SUB", "C", "ONE", "IN_B", pred="NOSWAP"),
    # s = f/(2 + f), z = s^2, R = z*(Lg1 + z*(Lg2 + ... + z*Lg7))
    Op("ADD", "IN_B", "TWO", "T1"),
    Op("DIV", "IN_B", "T1", "S"),
    Op("MUL", "S", "S", "Z"),
    *horner("R", "Z", [f"LG{i}" for i in range(1, 8)]),
    Op("MUL", "R", "Z", "R"),
    # hfsq = f^2/2, r = s*(hfsq + R): log(1 + f) = f - hfsq + r
    Op("MUL", "IN_B", "HALF", "H"), Op("MUL", "H", "IN_B", "H"),
    Op("ADD", "H", "R", "T1"), Op("MUL", "S", "T1", "T1"),
    # hi = trunc(f - hfsq), lo = (f - hi) - hfsq + r
    Op("SUB", "IN_B", "H", "U"),
    Op("SUB", "IN_B", "U", "V", b_trunc=True),
    Op("SUB", "V", "H", "V"), Op("ADD", "V", "T1", "V"),
    # LH = hi*IVLN2HI (exact), LL = lo/ln2 + hi*IVLN2LO
    Op("MUL", "U", "IVLN2HI", "P", a_trunc=True),
    Op("MUL", "V", "IVLN2", "V"),
    Op("MUL", "U", "IVLN2LO", "W", a_trunc=True),
    Op("ADD", "V", "W", "Q"),
    # k != 0: LH = k + LH rounded to 2^-25, the rest into LL.
    Op("ADD", "N", "P", "T", pred="KNZ"),
    Op("SUB", "N", "T", "W", pred="KNZ"),
    Op("ADD", "W", "P", "W", pred="KNZ"),
    Op("ADD", "W", "Q", "Q", pred="KNZ"),
    Op("SUB", "T", "KBASE", "P", pred="KNZ"),
]
# y*(LH + LL) = trunc(y)*LH (exact) + ((y - trunc(y))*LH + y*LL)
Y_TAIL = [
    Op("SUB", "IN_A", "IN_A", "T", b_trunc=True),
    Op("MUL", "T", "P", "T"),
    Op("MUL", "IN_A", "Q", "W"),
    Op("ADD", "T", "W", "T"),
    Op("MUL", "IN_A", "P", "W", a_trunc=True),
    Op("ADD", "W", "T", final=True),
]
LOG += Y_TAIL
# FYL2XP1, |x| < 2^-60: log2(1 + x) = x/ln2: hi = trunc(x), lo = x - hi.
LOG1P_TINY = [
    Op("SUB", "IN_B", "IN_B", "V", b_trunc=True),
    Op("MUL", "IN_B", "IVLN2HI", "P", a_trunc=True),
    Op("MUL", "V", "IVLN2", "V"),
    Op("MUL", "IN_B", "IVLN2LO", "W", a_trunc=True),
    Op("ADD", "V", "W", "Q"),
    *Y_TAIL,
]

ENTRIES = {}
PROGRAM: list[Op] = []
for _name, _prog in (("SINCOS", SINCOS), ("ATAN", ATAN), ("EXP", EXP), ("EXP_TINY", EXP_TINY),
                     ("LOG1P", LOG), ("LOG1P_TINY", LOG1P_TINY)):
    ENTRIES[_name] = len(PROGRAM)
    PROGRAM += _prog
ENTRIES["LOG"] = ENTRIES["LOG1P"] + 2                # FYL2X: at XTR
assert PROGRAM[ENTRIES["LOG"]].op == "XTR"
PC_BITS = 8
assert len(PROGRAM) <= 1 << PC_BITS
ATAN_ENTRY = ENTRIES["ATAN"]


def operand(name: str | None) -> tuple[int, int]:
    """(is_const, index)."""
    if name is None:
        return 0, 0
    if name in SLOT:
        return 0, SLOT[name]
    return 1, CONST_INDEX[name]


FIELDS = [  # name, width, comment
    ("op", 3, "X87_POLY_OP_*"),
    ("final_op", 1, "last operation: rounds per the control word"),
    ("a_const", 1, "A from the constant table (else scratch)"),
    ("a_index", CONST_BITS, ""),
    ("a_id", 1, "add the atan reduction id to A's constant index"),
    ("a_neg", 1, ""),
    ("a_abs", 1, ""),
    ("a_trunc", 1, f"A's significand cut to its top {TRUNC_BITS} bits"),
    ("b_const", 1, ""),
    ("b_index", CONST_BITS, "XTR: the slot written with 2^27 + 2^15 + k"),
    ("b_id", 1, ""),
    ("b_abs", 1, ""),
    ("b_trunc", 1, ""),
    ("dest", 4, "scratch slot written with the result"),
    ("pred", 5, "run only when the predicate holds"),
    ("setp", 2, "CMP: 1 swap = A < B, 2/3 id += A >= B"),
]
WORD_BITS = sum(w for _, w, _ in FIELDS)


def encode(o: Op) -> int:
    a_c, a_i = operand(o.a)
    b_c, b_i = operand(o.b)
    values = {
        "op": OPS[o.op], "final_op": int(o.final), "a_const": a_c, "a_index": a_i,
        "a_id": int(o.a_id), "a_neg": int(o.a_neg), "a_abs": int(o.a_abs),
        "a_trunc": int(o.a_trunc), "b_const": b_c, "b_index": b_i, "b_id": int(o.b_id),
        "b_abs": int(o.b_abs), "b_trunc": int(o.b_trunc),
        "dest": SLOT[o.dest] if o.dest else 0, "pred": PRED[o.pred], "setp": SETP[o.setp],
    }
    word = 0
    for name, width, _ in FIELDS:
        assert 0 <= values[name] < (1 << width), (name, values[name])
        word = (word << width) | values[name]
    return word


# --------------------------------------------------------------------------
# Model: 53-bit round-to-nearest-even operations with an unbounded exponent
# (the executor's arithmetic within its range). Values are mpf; the model has
# no signed zero (no program result is zero).
# --------------------------------------------------------------------------

MPF = mpmath.mpf
_LIBOP = {"MUL": libmp.mpf_mul, "ADD": libmp.mpf_add, "SUB": libmp.mpf_sub, "DIV": libmp.mpf_div}


def r53(v) -> MPF:
    return MPF(libmp.mpf_pos(MPF(v)._mpf_, 53, "n"))


def op53(op: str, x: MPF, y: MPF) -> MPF:
    return MPF(_LIBOP[op](x._mpf_, y._mpf_, 53, "n"))


def op_exact(op: str, x: MPF, y: MPF) -> MPF:
    if op == "DIV":
        return MPF(libmp.mpf_div(x._mpf_, y._mpf_, 400, "n"))
    return MPF(_LIBOP[op](x._mpf_, y._mpf_, 0))


def sig_exp(v: MPF) -> tuple[int, int, int]:
    """(sign, 53-bit significand, unbiased exponent) of a nonzero value with <= 53 bits."""
    s, man, exp, bc = v._mpf_
    assert man and bc <= 53, v
    return s, man << (53 - bc), exp + bc - 1


def trunc13(v: MPF) -> MPF:
    if v == 0:
        return v
    s, sig, e = sig_exp(v)
    sig &= ~((1 << (53 - TRUNC_BITS)) - 1)
    return MPF(libmp.from_man_exp(-sig if s else sig, e - 52))


def xtr(v: MPF) -> tuple[MPF, int, bool]:
    """x87_control's XTR: x = 2^k * m, sqrt(1/2) <= m < sqrt(2); exact = (m == 1)."""
    s, sig, e = sig_exp(v)
    assert not s
    ge = sig >= SQRT2_SIG
    m = MPF(libmp.from_man_exp(sig, -52 - int(ge)))
    return m, e + int(ge), sig == 1 << 52


def run(prog: list[Op], entry: int, a, b, cosine: bool = False):
    """Run from `entry` with IN_A = a, IN_B = b (and xneg = b < 0, yneg = a < 0).
    Returns (result, aux, c1, pe): c1 = the final rounding went up in
    magnitude; pe = the precision exception x87_control reports: always, but
    when XTR found m = 1 (x a power of two) only if the final rounding was
    inexact."""
    a, b = MPF(a), MPF(b)
    slots = {"IN_A": a, "IN_B": b}
    const = {name: MPF(v) for name, v in CONSTS}
    swap = idn = quad = 0
    k0, exact = True, False
    xneg, yneg = b < 0, a < 0
    aux = None
    pc = entry

    def fetch(name, neg=False, absv=False, use_id=False, trunc=False):
        if name in SLOT:
            v = slots[name]
        else:
            if use_id:
                base = name[:-1]
                name = f"{base}{int(name[-1]) + idn}"
            v = const[name]
        if trunc:
            v = trunc13(v)
        if absv:
            v = abs(v)
        if neg:
            v = -v
        return v

    while True:
        o = prog[pc]
        pc += 1
        qa = not swap and not xneg
        ok = {"ALWAYS": True, "SWAP": swap == 1, "NOSWAP": swap == 0, "YNEG": yneg,
              "QA": qa, "QB": swap and not xneg,
              "QC": not swap and xneg, "QD": swap and xneg,
              "QA0": qa and idn == 0, "QA1": qa and idn != 0, "KNZ": not k0}
        for k in range(4):
            ok[f"Q{k}"] = quad == k
            ok[f"Q{k}S"] = quad == k and not cosine
            ok[f"Q{k}C"] = quad == k and cosine
        if not ok[o.pred]:
            continue
        x = fetch(o.a, o.a_neg, o.a_abs, o.a_id, o.a_trunc)
        y = fetch(o.b, False, o.b_abs, o.b_id, o.b_trunc) if o.b and o.op != "XTR" else None
        if o.op == "AUX":
            aux = x
            continue
        if o.op == "XTR":
            m, k, exact = xtr(x)
            slots[o.dest] = m
            slots[o.b] = MPF(KBASE + k)
            k0 = k == 0
            continue
        if o.op == "CMP":
            less = x < y
            if o.setp == "SWAP":
                swap = int(less)
            else:
                idn += int(not less)
            continue
        if o.op == "RND":
            r = mpmath.nint(x)
            quad = int(r) & 3
        else:
            r = op53(o.op, x, y)
        if o.final:
            c1, inexact = False, True
            if o.op != "RND":
                ex = op_exact(o.op, x, y)
                c1, inexact = abs(r) > abs(ex), r != ex
            return r, aux, c1, inexact or not exact
        slots[o.dest] = r


def ulp_error(got, exact) -> float:
    exact = MPF(exact)
    if exact == 0:
        return 0.0 if got == 0 else float("inf")
    e = mpmath.floor(mpmath.log(abs(exact), 2))
    return float(abs(MPF(got) - exact) / mpmath.ldexp(1, int(e) - 52))


# F2XM1, FYL2X, FYL2XP1 as x87_control starts them (after x87_executor's
# classification): the entry, and IN_A/IN_B.
TINY = MPF(2) ** -60


def f2xm1(x):
    x = MPF(x)
    return run(PROGRAM, ENTRIES["EXP_TINY" if abs(x) < TINY else "EXP"], x, x)


def fyl2x(y, x):
    return run(PROGRAM, ENTRIES["LOG"], y, x)


def fyl2xp1(y, x):
    x = MPF(x)
    return run(PROGRAM, ENTRIES["LOG1P_TINY" if abs(x) < TINY else "LOG1P"], y, x)


def rand53(rng: random.Random, lo_exp: float, hi_exp: float, sign: bool = True) -> MPF:
    """A random 53-bit value with |v| = 2^U(lo_exp, hi_exp) * [1, 2)."""
    e = math.floor(rng.uniform(lo_exp, hi_exp))
    v = MPF(libmp.from_man_exp(rng.randrange(1 << 52, 1 << 53), e - 52))
    return -v if sign and rng.random() < 0.5 else v


def check(samples: int, limit: float) -> dict[str, float]:
    rng = random.Random(486)
    worst = {"sin": 0.0, "cos": 0.0, "atan2": 0.0}
    where = {}

    def note(name, err, arg):
        if err > worst.get(name, 0.0):
            worst[name], where[name] = err, arg
        elif name not in worst:
            worst[name], where[name] = err, arg

    for i in range(samples):
        e = rng.uniform(-26, 19)                       # |x| from 2^-26 to 2^19
        x = math.copysign(2.0 ** e * rng.uniform(1, 2), rng.choice((-1, 1)))
        for name, cosine, fn in (("sin", False, mpmath.sin), ("cos", True, mpmath.cos)):
            got, aux, _, _ = run(PROGRAM, 0, x, 0.0, cosine)
            note(name, ulp_error(got, fn(mpmath.mpf(x))), x)
            if name == "sin":
                note("cos", ulp_error(aux, mpmath.cos(mpmath.mpf(x))), x)
        y = math.copysign(2.0 ** rng.uniform(-30, 30) * rng.uniform(1, 2), rng.choice((-1, 1)))
        xx = math.copysign(2.0 ** rng.uniform(-30, 30) * rng.uniform(1, 2), rng.choice((-1, 1)))
        got, _, _, _ = run(PROGRAM, ATAN_ENTRY, y, xx, False)
        note("atan2", ulp_error(got, mpmath.atan2(mpmath.mpf(y), mpmath.mpf(xx))), (y, xx))

    # 2^x - 1 on [-1, 1]: random magnitudes, the tiny path, the k boundaries
    # (|x| = 1/2), the k = 1 split (h = -1/4 at x ~ 0.64) and |x| near 1.
    def exp_args():
        for _ in range(samples):
            yield rand53(rng, -70, 0)
        for _ in range(samples // 4):
            yield rand53(rng, -1, 0)
        for c in (MPF(0.5), MPF(1), MPF("0.6393"), MPF("0.6394")):
            for _ in range(samples // 8):
                d = rand53(rng, -53, -2)
                for v in (c + d, -(c + d)):
                    v = r53(v)
                    if 0 < abs(v) < 1:
                        yield v
    for x in exp_args():
        got = f2xm1(x)[0]
        note("f2xm1", ulp_error(got, mpmath.expm1(x * LN2)), x)

    # y*log2(x): x over the whole range (and m80 denormals), near 1, near
    # powers of two and sqrt(2) multiples (the k and m boundaries).
    def log_args():
        for _ in range(samples):
            yield rand53(rng, -30, 30), rand53(rng, -16400, 16384, sign=False)
        for _ in range(samples):
            yield rand53(rng, -30, 30), rand53(rng, -40, 40, sign=False)
        for _ in range(samples):
            d = rand53(rng, -53, -1)
            yield rand53(rng, -30, 30), r53(1 + d)
        for _ in range(samples // 2):
            k = rng.randrange(-16000, 16000)
            d = rand53(rng, -53, -2)
            c = MPF(2) ** k * (rng.choice((1, mpmath.sqrt(2))))
            yield rand53(rng, -30, 30), r53(c * (1 + d))
        for _ in range(samples // 4):                    # m80 denormal x
            sig = rng.randrange(1, 1 << 52)
            yield rand53(rng, -30, 30), MPF(libmp.from_man_exp(sig, -16445 + 11))
    for y, x in log_args():
        if x == 1:
            continue
        got = fyl2x(y, x)[0]
        note("fyl2x", ulp_error(got, y * mpmath.log(x, 2)), (y, x))

    # y*log2(1 + x), |x| < 1 - sqrt(2)/2: tiny, random, near the limit.
    lim = 1 - mpmath.sqrt(2) / 2

    def log1p_args():
        for _ in range(samples):
            yield rand53(rng, -30, 30), rand53(rng, -70, -1.77)
        for _ in range(samples // 4):
            yield rand53(rng, -30, 30), r53((lim - abs(rand53(rng, -53, -6))) *
                                            rng.choice((-1, 1)))
    for y, x in log1p_args():
        if abs(x) >= CONSTS[CONST_INDEX["LOG1P_LIM"]][1]:
            continue
        got = fyl2xp1(y, x)[0]
        note("fyl2xp1", ulp_error(got, y * mpmath.log1p(x) / LN2), (y, x))

    for name, err in worst.items():
        print(f"  {name:7s} max error {err:.3f} ulp at {where.get(name)}")
    for name, err in worst.items():
        if err > limit:
            raise SystemExit(f"{name}: {err:.3f} ulp exceeds {limit}")
    return worst


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


X87_ZERO, X87_NORMAL = 1, 2     # x87_pkg::x87_class_t


def to_reg(v: float) -> int:
    """Double to x87_pkg::x87_reg_t {sign, exp[14:0], sig[52:0], G, R, S, class[2:0]}."""
    bits = struct.unpack(">Q", struct.pack(">d", v))[0]
    sign = bits >> 63
    if v == 0.0:
        return (sign << 74) | X87_ZERO
    exp = (bits >> 52) & 0x7FF
    frac = bits & ((1 << 52) - 1)
    assert 0 < exp < 0x7FF
    sig = (1 << 52) | frac
    return (sign << 74) | ((exp - 1023 + 16383) << 59) | (sig << 6) | X87_NORMAL


TEST_OPS = {"FSIN": 0, "FCOS": 1, "FSINCOS": 2, "FPTAN": 3, "FPATAN": 4}


def write_test(path: Path, count: int) -> None:
    """A real-mode test that runs each operation on random arguments and
    compares the stored doubles with this script's emulation bit for bit."""
    rng = random.Random(4860)
    rows = []

    def d2h(v: float) -> int:
        return struct.unpack(">Q", struct.pack(">d", v))[0]

    for i in range(count):
        op = list(TEST_OPS)[i % len(TEST_OPS)]
        x = math.copysign(2.0 ** rng.uniform(-20, 19) * rng.uniform(1, 2), rng.choice((-1, 1)))
        y = math.copysign(2.0 ** rng.uniform(-20, 20) * rng.uniform(1, 2), rng.choice((-1, 1)))
        if op == "FPATAN":
            r1 = float(run(PROGRAM, ATAN_ENTRY, y, x, False)[0])    # ST1 = y, ST0 = x
            r2 = 0.0
        else:
            sin_x, cos_x, _, _ = run(PROGRAM, 0, x, 0.0, False)
            sin_x, cos_x = float(sin_x), float(cos_x)
            if op == "FSIN":
                r1, r2 = sin_x, 0.0
            elif op == "FCOS":
                r1 = float(run(PROGRAM, 0, x, 0.0, True)[0])
                r2 = 0.0
            elif op == "FSINCOS":
                r1, r2 = cos_x, sin_x                     # ST0 = cos, ST1 = sin
            else:
                r1, r2 = 1.0, sin_x / cos_x               # ST0 = 1.0, ST1 = tan
        rows.append((TEST_OPS[op], x, y, r1, r2))

    asm = [
        "; Generated by x87/build_x87_poly.py --test: FSIN, FCOS, FSINCOS, FPTAN",
        "; and FPATAN on random arguments against the program's emulation, bit",
        "; for bit (which build_x87_poly.py checks against mpmath).",
        "; Fail code (port E4): case index + 1.",
        "", "BITS 16", "org 0", "",
        "STATUS_PORT equ 0xE0", "DATA_PORT   equ 0xE4", "",
        "start:", "    cli", "    mov ax, cs", "    mov ds, ax", "    mov ss, ax", "    mov sp, 0x700",
        "    fninit", "    mov si, cases", "    xor bp, bp",
        "next:", f"    cmp bp, {len(rows)}", "    je pass",
        "    mov al, [si]",
        "    cmp al, 4", "    je do_atan",
        "    fld qword [si + 8]",
        "    cmp al, 0", "    jne .n0", "    fsin", "    jmp one",
        ".n0:", "    cmp al, 1", "    jne .n1", "    fcos", "    jmp one",
        ".n1:", "    cmp al, 2", "    jne .n2", "    fsincos", "    jmp two",
        ".n2:", "    fptan", "    jmp two",
        "do_atan:", "    fld qword [si + 16]", "    fld qword [si + 8]", "    fpatan",
        "one:", "    fstp qword [result]", "    mov eax, [result]", "    cmp eax, [si + 24]", "    jne fail",
        "    mov eax, [result + 4]", "    cmp eax, [si + 28]", "    jne fail", "    jmp advance",
        "two:", "    fstp qword [result]", "    mov eax, [result]", "    cmp eax, [si + 24]", "    jne fail",
        "    mov eax, [result + 4]", "    cmp eax, [si + 28]", "    jne fail",
        "    fstp qword [result]", "    mov eax, [result]", "    cmp eax, [si + 32]", "    jne fail",
        "    mov eax, [result + 4]", "    cmp eax, [si + 36]", "    jne fail",
        "advance:", "    add si, 40", "    inc bp", "    jmp next",
        "pass:", "    mov al, 1", "    out STATUS_PORT, al", "    hlt",
        "fail:", "    mov ax, bp", "    inc ax", "    out DATA_PORT, ax", "    mov al, 0xff",
        "    out STATUS_PORT, al", "    hlt", "",
        "align 8", "result: dq 0", "align 8", "cases:",
    ]
    for op, x, y, r1, r2 in rows:
        asm.append(f"    dd {op}, 0")
        asm.append(f"    dq 0x{d2h(x):016x}, 0x{d2h(y):016x}, 0x{d2h(r1):016x}, 0x{d2h(r2):016x}")
    path.write_text("\n".join(asm) + "\n")


# m80 encodings for the F2XM1/FYL2X/FYL2XP1 tests.
M80_POS_INF = 0x7FFF8000000000000000
M80_INDEFINITE = 0xFFFFC000000000000000


def m80(v) -> int:
    """A value of at most 53 significant bits as an m80 bit pattern (normal or
    denormal); "-0"/"+0" strings for signed zeros."""
    if isinstance(v, str):
        return {"+0": 0, "-0": 1 << 79, "+inf": M80_POS_INF, "-inf": M80_POS_INF | 1 << 79,
                "indef": M80_INDEFINITE}[v]
    v = MPF(v)
    if v == 0:
        return 0
    s, sig, e = sig_exp(v)
    if e < -16382:                                   # denormal: raw * 2^-16445
        raw = MPF(abs(v)) * MPF(2) ** 16445
        assert raw == int(raw) and raw < 2**63
        return (s << 79) | int(raw)
    assert e <= 16383
    return (s << 79) | ((e + 16383) << 64) | (sig << 11)


SW_C1, SW_C2, SW_PE, SW_UE, SW_ZE, SW_DE, SW_IE, SW_ES = 0x200, 0x400, 0x20, 0x10, 0x4, 0x2, 0x1, 0x80
SW_CHECK = 0x3EFF            # exceptions, SF, ES, C1, C2, TOP (not C0/C3, B)
LOGEXP_OPS = {"F2XM1": 0, "FYL2X": 1, "FYL2XP1": 2}


def logexp_case(op: str, x, y=None, cw: int = 0x037F, de: bool = False):
    """A program-computed case: (op, cw, y, x, depth, results, status)."""
    if op == "F2XM1":
        r, _, c1, pe = f2xm1(x)
    elif op == "FYL2X":
        r, _, c1, pe = fyl2x(y, x)
    else:
        r, _, c1, pe = fyl2xp1(y, x)
    sw = (7 << 11) | (SW_C1 if c1 else 0) | (SW_PE if pe else 0) | (SW_DE if de else 0)
    return (op, cw, y, x, [r], sw)


def special_cases() -> list:
    """F2XM1, FYL2X and FYL2XP1 on special operands, as Intel's SDM tables
    give them (and this implementation's choices where Intel leaves the
    result undefined), masked and unmasked: (op, cw, y, x, results, status).
    An unmasked exception leaves the stack as it was (no pop)."""
    qnan = 0x7FFFC000000001234000                 # +QNaN, a payload (above bit 10)
    qnan2 = 0xFFFFC000000005678000                # -QNaN
    snan = 0x7FFFA000000000000000                 # +SNaN (quieted: C000...)
    snan_q = 0x7FFFE000000000000000
    den = MPF(libmp.from_man_exp(0x123456789ABC, -16445 + 11))   # m80 denormal
    one, two, three, half = MPF(1), MPF(2), MPF(3), MPF(0.5)
    t7, t6 = 7 << 11, 6 << 11
    IM, ZM, CW = 0x037E, 0x037B, 0x037F            # IE unmasked, ZE unmasked, all masked
    c = []
    # F2XM1 (masked): exact specials, out of range (ST0 unchanged, C2), flushed
    # tiny results (UE, PE), the tiny path.
    for x, r, sw in (("+0", "+0", 0), ("-0", "-0", 0), ("+inf", "+inf", 0), ("-inf", -one, 0),
                     (qnan, qnan, 0), (snan, snan_q, SW_IE), (one, one, 0), (-one, -half, 0),
                     (two, two, SW_C2), (MPF(-3.5), MPF(-3.5), SW_C2),
                     (1 + MPF(2) ** -52, 1 + MPF(2) ** -52, SW_C2), ("-inf", -one, 0),
                     (den, "+0", SW_DE | SW_UE | SW_PE), (-den, "-0", SW_DE | SW_UE | SW_PE),
                     (MPF(2) ** -16382, "+0", SW_UE | SW_PE)):
        c.append(("F2XM1", CW, None, x, [r], t7 | sw))
    c.append(("F2XM1", IM, None, snan, [snan], t7 | SW_IE | SW_ES))
    c.append(logexp_case("F2XM1", MPF(2) ** -16000 * 1.75))
    c.append(logexp_case("F2XM1", -MPF(2) ** -61))
    # FYL2X (masked): y in ST1, x in ST0; the result replaces ST1 and pops.
    fy = [
        (three, -two, "indef", SW_IE), (three, "-inf", "indef", SW_IE),
        (three, "-0", "-inf", SW_ZE), (three, "+0", "-inf", SW_ZE), (-three, "+0", "+inf", SW_ZE),
        ("+0", "+0", "indef", SW_IE), ("-0", "-0", "indef", SW_IE),
        ("-inf", "+0", "+inf", 0), ("+inf", "-0", "-inf", 0),
        (three, "+inf", "+inf", 0), (-three, "+inf", "-inf", 0),
        ("+0", "+inf", "indef", SW_IE), ("-0", "+inf", "indef", SW_IE),
        ("+inf", "+inf", "+inf", 0), ("-inf", "+inf", "-inf", 0),
        (three, one, "+0", 0), (-three, one, "-0", 0), ("-0", one, "-0", 0), ("+0", one, "+0", 0),
        ("+inf", one, "indef", SW_IE), ("-inf", one, "indef", SW_IE),
        ("+inf", half, "-inf", 0), ("-inf", half, "+inf", 0), ("+inf", two, "+inf", 0),
        ("-inf", two, "-inf", 0),
        ("+0", two, "+0", 0), ("+0", half, "-0", 0), ("-0", half, "+0", 0), ("-0", two, "-0", 0),
        (three, qnan, qnan, 0), (qnan2, three, qnan2, 0), (qnan2, qnan, qnan2, 0),
        (snan, three, snan_q, SW_IE), (three, snan, snan_q, SW_IE), (snan, -two, snan_q, SW_IE),
        (den, two, "+0", SW_DE | SW_UE | SW_PE), (den, half, "-0", SW_DE | SW_UE | SW_PE),
        (-den, two, "-0", SW_DE | SW_UE | SW_PE),
    ]
    for y, x, r, sw in fy:
        c.append(("FYL2X", CW, y, x, [r], t7 | sw))
    # Unmasked: no write, no pop.
    c.append(("FYL2X", IM, three, -two, [-two, three], t6 | SW_IE | SW_ES))
    c.append(("FYL2X", IM, "+0", "+0", ["+0", "+0"], t6 | SW_IE | SW_ES))
    c.append(("FYL2X", ZM, three, "+0", ["+0", three], t6 | SW_ZE | SW_ES))
    c.append(("FYL2X", ZM, "+inf", "+0", ["-inf"], t7))            # no ZE for y = inf
    # Exact powers of two: no PE when the product is exact.
    c.append(logexp_case("FYL2X", MPF(8), three))
    c.append(logexp_case("FYL2X", MPF(2), 1 + MPF(2) ** -52))
    c.append(logexp_case("FYL2X", MPF(2) ** -16382, -MPF(0.75)))
    c.append(logexp_case("FYL2X", den, MPF(1.5), de=True))
    c.append(logexp_case("FYL2X", MPF(2) ** 16383 * 1.9, -three))
    # FYL2XP1 (masked).
    q = MPF(0.25)
    fp = [
        (three, "+0", "+0", 0), (three, "-0", "-0", 0), (-three, "+0", "-0", 0),
        (-three, "-0", "+0", 0), ("+0", "-0", "-0", 0), ("-0", "-0", "+0", 0),
        ("+inf", "+0", "indef", SW_IE), ("-inf", "-0", "indef", SW_IE),
        ("+inf", q, "+inf", 0), ("+inf", -q, "-inf", 0), ("-inf", -q, "+inf", 0),
        ("+0", q, "+0", 0), ("+0", -q, "-0", 0), ("-0", q, "-0", 0), ("-0", -q, "+0", 0),
        (three, -one, "-inf", SW_ZE), (-three, -one, "+inf", SW_ZE), ("+0", -one, "indef", SW_IE),
        ("+inf", -one, "-inf", 0), ("-inf", -one, "+inf", 0),
        (three, -two, "indef", SW_IE), (three, "-inf", "indef", SW_IE),
        (three, "+inf", "+inf", 0), (-three, "+inf", "-inf", 0), ("+0", "+inf", "indef", SW_IE),
        (three, qnan, qnan, 0), (qnan2, q, qnan2, 0), (snan, q, snan_q, SW_IE),
        (three, den, "+0", SW_DE | SW_UE | SW_PE), (-three, den, "-0", SW_DE | SW_UE | SW_PE),
        (three, -den, "-0", SW_DE | SW_UE | SW_PE),
        (den, q, "+0", SW_DE | SW_UE | SW_PE), (den, -q, "-0", SW_DE | SW_UE | SW_PE),
    ]
    for y, x, r, sw in fp:
        c.append(("FYL2XP1", CW, y, x, [r], t7 | sw))
    c.append(("FYL2XP1", IM, three, -two, [-two, three], t6 | SW_IE | SW_ES))
    c.append(("FYL2XP1", ZM, three, -one, [-one, three], t6 | SW_ZE | SW_ES))
    # Outside Intel's domain: FYL2X of 1 + x; exact when 1 + x is a power of two.
    for x in (one, three, -half, half, MPF(100.5), MPF(-0.75), -one + MPF(2) ** -53,
              MPF(2) ** 16000):
        c.append(logexp_case("FYL2XP1", x, three))
    c.append(logexp_case("FYL2XP1", MPF(2) ** -70 * 1.375, -three))
    c.append(logexp_case("FYL2XP1", MPF(2) ** -16300, MPF(2) ** 100))
    return c


def write_logexp_tests(prog_dir: Path, count: int) -> None:
    """tests/programs/x87_trans_logexp.asm: F2XM1, FYL2X and FYL2XP1 on random
    and edge arguments (model results, C1 and PE) and on special operands
    (results and flags, masked and unmasked). Operands and results are m80."""
    rng = random.Random(4861)
    cases = special_cases()
    for i in range(count):
        kind = i % 3
        if kind == 0:
            x = rand53(rng, -62, 0) if i % 2 else r53(rand53(rng, -40, -1) + rng.choice((-1, 1)) * 0.5)
            if not 0 < abs(x) < 1:
                x = MPF(0.375)
            cases.append(logexp_case("F2XM1", x))
        elif kind == 1:
            y = rand53(rng, -40, 40)
            j = i % 4
            if j == 0:
                x = rand53(rng, -16382, 16383, sign=False)
            elif j == 1:
                x = r53(1 + rand53(rng, -52, -2))
            else:
                x = rand53(rng, -10, 10, sign=False)
            if x == 1:
                x = MPF(3)
            cases.append(logexp_case("FYL2X", x, y))
        else:
            y = rand53(rng, -40, 40)
            x = rand53(rng, -64, -1.8)
            cases.append(logexp_case("FYL2XP1", x, y))

    asm = [
        "; Generated by x87/build_x87_poly.py --test: F2XM1, FYL2X and FYL2XP1 on",
        "; special operands (results and flags as Intel's tables give them, masked",
        "; and unmasked) and on random and edge arguments (results, C1 and PE from",
        "; the programs' emulation, which build_x87_poly.py checks against mpmath).",
        "; Operands and results are m80. Each case: FNINIT, FLDCW, FLD y (FYL2X,",
        "; FYL2XP1), FLD x, the operation, FNSTSW (exceptions, SF, ES, C1, C2 and",
        "; TOP checked), FNCLEX, then the stack's values. Fail code (port E4): case",
        "; index + 1 in the low word, the status word in the high word; then the",
        "; value stored last (low dword, middle dword, high word).",
        "", "BITS 16", "org 0", "",
        "STATUS_PORT equ 0xE0", "DATA_PORT   equ 0xE4", "",
        "start:", "    cli", "    mov ax, cs", "    mov ds, ax", "    mov es, ax", "    mov ss, ax",
        "    mov sp, 0xfff0", "    cld",                  # above the case table
        "    mov si, cases", "    xor bp, bp",
        "next:", f"    cmp bp, {len(cases)}", "    je pass",
        "    fninit", "    fldcw [si + 2]",
        "    mov al, [si]",
        "    cmp al, 0", "    je .one",
        "    fld tword [si + 4]",
        ".one:", "    fld tword [si + 14]",
        "    cmp al, 1", "    je .fyl2x",
        "    cmp al, 2", "    je .fyl2xp1",
        "    f2xm1", "    jmp .done",
        ".fyl2x:", "    fyl2x", "    jmp .done",
        ".fyl2xp1:", "    fyl2xp1",
        ".done:", "    fnstsw ax", "    mov [status], ax", "    fnclex",
        "    and ax, 0x%04x" % SW_CHECK, "    cmp ax, [si + 44]", "    jne fail",
        "    fstp tword [result]", "    lea di, [si + 24]", "    call compare", "    jne fail",
        "    cmp byte [si + 1], 2", "    jne advance",
        "    fstp tword [result]", "    lea di, [si + 34]", "    call compare", "    jne fail",
        "advance:", "    add si, 46", "    inc bp", "    jmp next",
        "compare:", "    push si", "    mov si, result", "    mov cx, 5", "    repe cmpsw",
        "    pop si", "    ret",
        "pass:", "    mov al, 1", "    out STATUS_PORT, al", "    hlt",
        "fail:", "    mov ax, [status]", "    shl eax, 16", "    mov ax, bp", "    inc ax",
        "    out DATA_PORT, eax",
        "    mov eax, [result]", "    out DATA_PORT, eax", "    mov eax, [result + 4]",
        "    out DATA_PORT, eax", "    mov ax, [result + 8]", "    out DATA_PORT, ax",
        "    mov al, 0xff", "    out STATUS_PORT, al", "    hlt", "",
        "align 2", "status: dw 0", "result: times 10 db 0", "align 2", "cases:",
        "; op, depth, cw, y, x, result 0 (ST0), result 1 (ST1), status & %04xh" % SW_CHECK,
    ]

    def t80(v) -> str:
        b = (m80(v) if not isinstance(v, int) else v).to_bytes(10, "little")
        return ", ".join(f"0x{x:02x}" for x in b)

    for op, cw, y, x, results, sw in cases:
        y = "+0" if y is None else y
        r0 = results[0]
        r1 = results[1] if len(results) > 1 else "+0"
        asm.append(f"    db {LOGEXP_OPS[op]}, {len(results)}")
        asm.append(f"    dw 0x{cw:04x}")
        asm.append(f"    db {t80(y)}")
        asm.append(f"    db {t80(x)}")
        asm.append(f"    db {t80(r0)}")
        asm.append(f"    db {t80(r1)}")
        asm.append(f"    dw 0x{sw & SW_CHECK:04x}")
    (prog_dir / "x87_trans_logexp.asm").write_text("\n".join(asm) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=int, default=4000)
    ap.add_argument("--limit", type=float, default=1.1)
    ap.add_argument("--test", type=int, metavar="N", default=0,
                    help="also write tests/programs/x87_trans_poly.asm with N cases and "
                         "x87_trans_logexp.asm with the special cases and N more")
    args = ap.parse_args()

    print(f"program: {len(SINCOS)} sin/cos, {len(ATAN)} atan2, {len(EXP) + len(EXP_TINY)} 2^x-1, "
          f"{len(LOG) + len(LOG1P_TINY)} log2 words ({len(PROGRAM)}); {len(CONSTS)} constants")
    check(args.samples, args.limit)

    root = Path(__file__).resolve().parent
    pred_terms = {
        "ALWAYS": "1'b1", "SWAP": "swap", "NOSWAP": "!swap", "YNEG": "yneg",
        "QA": "!swap && !xneg", "QB": "swap && !xneg", "QC": "!swap && xneg", "QD": "swap && xneg",
        "QA0": "!swap && !xneg && (id == 2'd0)", "QA1": "!swap && !xneg && (id != 2'd0)",
        "KNZ": "!k0",
    }
    for k in range(4):
        pred_terms[f"Q{k}"] = f"quadrant == 2'd{k}"
        pred_terms[f"Q{k}S"] = f"quadrant == 2'd{k} && !cosine"
        pred_terms[f"Q{k}C"] = f"quadrant == 2'd{k} && cosine"
    pc, cb = PC_BITS, CONST_BITS
    lines = [
        "// Generated by build_x87_poly.py: x87 transcendental programs. Do not edit.",
        "typedef struct packed {",
        *[(f"    logic {'' if w == 1 else f'[{w - 1}:0]':5} {name + ';':10}" +
           (f" // {cm}" if cm else "")).rstrip() for name, w, cm in FIELDS],
        "} x87_poly_word_t;",
        *[f"localparam logic [2:0] X87_POLY_OP_{name} = 3'd{v};" for name, v in OPS.items()],
        *[f"localparam logic [1:0] X87_POLY_SETP_{name} = 2'd{v};" for name, v in SETP.items()],
        f"localparam int X87_POLY_WORD_BITS = {WORD_BITS};",
        f"localparam int X87_POLY_PC_BITS = {pc};",
        f"localparam int X87_POLY_CONST_BITS = {cb};",
        *[f"localparam logic [{pc - 1}:0] X87_POLY_ENTRY_{name} = {pc}'d{v};"
          for name, v in ENTRIES.items()],
        f"localparam logic [52:0] X87_POLY_SQRT2_SIG = 53'h{SQRT2_SIG:x};  // XTR: m >= sqrt(2) halves",
        "",
        f"function automatic logic [{WORD_BITS - 1}:0] x87_poly_word(input logic [{pc - 1}:0] address);",
        "    case (address)",
    ]
    for i, o in enumerate(PROGRAM):
        mods = "".join(f" {m}" for m in ("a_trunc", "b_trunc") if getattr(o, m))
        desc = (f"{o.op} {o.a or ''}{',' + o.b if o.b else ''} -> {o.dest or '-'} [{o.pred}]"
                + (" final" if o.final else "") + mods)
        lines.append(f"        {pc}'d{i}: return {WORD_BITS}'h{encode(o):0{(WORD_BITS + 3) // 4}x}; // {desc}")
    lines += ["        default: return '0;", "    endcase", "endfunction", "",
              f"function automatic logic [74:0] x87_poly_const(input logic [{cb - 1}:0] address);",
              "    case (address)"]
    for i, (name, v) in enumerate(CONSTS):
        lines.append(f"        {cb}'d{i}: return 75'h{to_reg(v):019x}; // {name} = {v!r}")
    lines += ["        default: return '0;", "    endcase", "endfunction", "",
              "function automatic logic x87_poly_pred(input logic [4:0] pred, input logic [1:0] quadrant,",
              "                                       input logic cosine, input logic swap,",
              "                                       input logic xneg, input logic yneg,",
              "                                       input logic [1:0] id, input logic k0);",
              "    case (pred)"]
    for name in PREDS:
        lines.append(f"        5'd{PRED[name]}: return {pred_terms[name]}; // {name}")
    lines += ["        default: return 1'b0;", "    endcase", "endfunction", ""]
    (root / "x87_poly_rom.svh").write_text("\n".join(lines))
    print("wrote x87_poly_rom.svh")
    if args.test:
        progs = root.parent / "tests" / "programs"
        write_test(progs / "x87_trans_poly.asm", args.test)
        print(f"wrote tests/programs/x87_trans_poly.asm ({args.test} cases)")
        write_logexp_tests(progs, args.test)
        print(f"wrote tests/programs/x87_trans_logexp.asm ({len(special_cases())} special, "
              f"{args.test} more)")


if __name__ == "__main__":
    main()
