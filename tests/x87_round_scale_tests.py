#!/usr/bin/env python3
"""FRNDINT, FSCALE, FPREM and FPREM1 test programs, expected from exact
integer arithmetic (Python's integers and fractions, not a model of the RTL).

    ./x87_round_scale_tests.py      # writes programs/x87_frndint.asm,
                                    # x87_fscale.asm and x87_fprem.asm

What the references take as given (the x87 here keeps a 53-bit significand
plus guard, round and sticky bits per register and stores 53 bits back):

- FRNDINT rounds the value to an integer as RC says, PE when that changes
  it, C1 when the magnitude grew. Operands are 53-bit, or 64-bit below 2^53
  (the rounding then sees every bit). No DE (as the executor has always
  done).
- FSCALE follows Intel's table (NaNs, ST0's first; inf * 2^-inf and
  0 * 2^+inf invalid; zeros and infinities of ST0 as they are; a finite ST0
  times 2^+-inf is inf or 0), DE for a denormal operand, trunc(ST1) exact.
  A result of 2^16384 or more overflows: masked, infinity or the largest
  finite number as RC rounds, OE, PE, C1 for infinity; unmasked, OE alone.
  A result below 2^-16382 is tiny: rounded as RC says to a multiple of
  2^-16434 (the 53-bit significand field's last bit; a carry gives the
  smallest normal), UE and PE when that is inexact and UE is masked, UE
  alone when UE is unmasked.
- FPREM and FPREM1 are what the x87 here computes: the quotient divided at
  53 bits (RC and PC from the control word), rounded to an integer toward
  zero (FPREM) or to nearest (FPREM1), multiplied by ST1 and subtracted from
  ST0 at 53 bits; C0, C3, C1 = the integer quotient's bits 2, 1, 0; C2 = 0;
  no exception flags.
- An unmasked exception leaves ST0 as it was (ES and B set).
"""

from __future__ import annotations

import json
import random
from fractions import Fraction
from pathlib import Path

ZERO, NORMAL, DENORMAL, INF, NAN = "zero", "normal", "denormal", "inf", "nan"
IE, DE, ZE, OE, UE, PE = 1, 2, 4, 8, 16, 32
CW = 0x037F                   # FNINIT's: everything masked, RN, PC 64
SW_CHECK = 0x7FFF             # all but B
INDEFINITE = 0xFFFF_C000_0000_0000_0000
MAX_CASES = 1100              # case data must stay in the 64 KiB segment


def f80(sign: int, exp: int, sig: int) -> int:
    return (sign << 79) | (exp << 64) | sig


class X:
    """An 80-bit register as x87_from_m80 classifies it."""

    def __init__(self, raw: int):
        self.raw = raw
        self.sign = raw >> 79
        self.exp = (raw >> 64) & 0x7FFF
        self.sig = raw & ((1 << 64) - 1)
        if self.exp == 0:
            self.cls = ZERO if self.sig == 0 else DENORMAL
        elif self.exp == 0x7FFF:
            self.cls = INF if (self.sig & ((1 << 63) - 1)) == 0 else NAN
        else:
            self.cls = NORMAL
        self.snan = self.cls == NAN and not (self.sig >> 62) & 1

    def value(self) -> Fraction:
        """Magnitude (finite)."""
        return Fraction(self.sig) * Fraction(2) ** (max(self.exp, 1) - 16383 - 63)

    def quiet(self) -> int:
        # The register keeps 53 bits; a quiet NaN gets bit 62 set.
        return f80(self.sign, 0x7FFF, (self.sig | (3 << 62)) & ~0x7FF)

    def canonical(self) -> int:
        if self.cls == ZERO:
            return f80(self.sign, 0, 0)
        if self.cls == INF:
            return f80(self.sign, 0x7FFF, 1 << 63)
        return self.raw


def ilog2(v: Fraction) -> int:
    """floor(log2(v)) for v > 0."""
    e = v.numerator.bit_length() - v.denominator.bit_length()
    if Fraction(2) ** e > v:
        e -= 1
    return e


def round_int(v: Fraction, sign: int, rc: int) -> int:
    """Round magnitude v to an integer magnitude as RC says for that sign."""
    q, r = divmod(v.numerator, v.denominator)
    if r == 0:
        return q
    if rc == 0:
        twice = 2 * r
        return q + (twice > v.denominator or (twice == v.denominator and q & 1))
    if rc == 1:
        return q + sign
    if rc == 2:
        return q + (1 - sign)
    return q


def pack(sign: int, v: Fraction) -> int:
    """A finite magnitude with at most 64 significant bits, normal range."""
    if v == 0:
        return f80(sign, 0, 0)
    e = ilog2(v)
    m = v / Fraction(2) ** (e - 63)
    assert m.denominator == 1 and m.numerator >> 63 == 1, v
    assert 1 <= e + 16383 < 0x7FFF
    return f80(sign, e + 16383, m.numerator)


def round53(sign: int, v: Fraction, rc: int) -> Fraction:
    """Round a nonzero magnitude to 53 significant bits (normal range)."""
    e = ilog2(v)
    unit = Fraction(2) ** (e - 52)
    m = round_int(v / unit, sign, rc)
    return m * unit


def status(flags: int, masks: int, top: int, c: int = 0) -> int:
    """Status word after FNSTSW: flags, ES, C bits (c: C3 C2 C1 C0 as 14,10,9,8)."""
    es = 0x80 if flags & ~masks & 0x3F else 0
    return flags | es | c | (top << 11)


def frndint_ref(a: X, cw: int) -> tuple[int, int, int]:
    """(result, flags, C1) of FRNDINT."""
    rc = (cw >> 10) & 3
    if a.cls == NAN:
        return a.quiet(), IE if a.snan else 0, 0
    if a.cls in (ZERO, INF):
        return a.canonical(), 0, 0
    v = a.value()
    q = round_int(v, a.sign, rc)
    flags = PE if q != v else 0
    return pack(a.sign, Fraction(q)), flags, int(q > v)


def fscale_ref(a: X, b: X, cw: int) -> tuple[int, int, int]:
    """(result, flags, C1) of FSCALE: ST0 = a, ST1 = b."""
    rc = (cw >> 10) & 3
    om = bool(cw & OE)
    um = bool(cw & UE)
    if a.cls == NAN or b.cls == NAN:
        flags = IE if (a.snan or b.snan) else 0
        return (a if a.cls == NAN else b).quiet(), flags, 0
    if (a.cls == INF and b.cls == INF and b.sign) or (a.cls == ZERO and b.cls == INF and not b.sign):
        return INDEFINITE, IE, 0
    flags = DE if DENORMAL in (a.cls, b.cls) else 0
    if a.cls in (INF, ZERO):
        return a.canonical(), flags, 0
    if b.cls == INF:
        return (f80(a.sign, 0, 0) if b.sign else f80(a.sign, 0x7FFF, 1 << 63)), flags, 0
    sig53 = a.sig >> 11
    if sig53 == 0:
        return f80(a.sign, 0, 0), flags, 0
    n = 0 if b.cls == ZERO else min(int(b.value()), 65535)
    n = -n if b.sign else n
    v = Fraction(sig53) * Fraction(2) ** (max(a.exp, 1) - 16383 - 52 + n)
    e = ilog2(v)
    if e + 16383 >= 0x7FFF:
        to_inf = rc == 0 or (rc == 1 and a.sign) or (rc == 2 and not a.sign)
        flags |= OE | (PE if om else 0)
        res = f80(a.sign, 0x7FFF, 1 << 63) if to_inf else f80(a.sign, 0x7FFE, (1 << 64) - (1 << 11))
        return res, flags, int(om and to_inf)
    if e + 16383 >= 1:
        return pack(a.sign, v), flags, 0
    quantum = Fraction(2) ** -16434
    scaled = v / quantum
    if a.sig & 0x7FF:
        # The register's guard, round and sticky bits (an 80-bit ST0's bits
        # beyond 53) are discarded below all of these: a sticky bit.
        scaled += Fraction(1, 4 * scaled.denominator)
    k = round_int(scaled, a.sign, rc)
    inexact = k != scaled
    flags |= (UE if (inexact or not um) else 0) | (PE if (inexact and um) else 0)
    c1 = int(um and k * quantum > v)
    if k == 0:
        res = f80(a.sign, 0, 0)
    elif k == 1 << 52:
        res = f80(a.sign, 1, 1 << 63)
    else:
        res = f80(a.sign, 0, k << 11)
    return res, flags, c1


def fprem_ref(a: X, b: X, cw: int, nearest: bool) -> tuple[int, int]:
    """(remainder, C bits) of FPREM/FPREM1 as the x87 here computes them
    (finite nonzero normal 53-bit operands, PC 53 or 64)."""
    rc = (cw >> 10) & 3
    va, vb = a.value(), b.value()
    q1 = round53(a.sign ^ b.sign, va / vb, rc)
    q2 = round_int(q1, a.sign ^ b.sign, 0 if nearest else 3)
    low3 = q2 & 7
    if q2 == 0:
        r_sign, r = a.sign, va
    else:
        m = round53(a.sign, q2 * vb, rc)       # q2 * ST1 has ST0's sign
        d = va - m
        if d == 0:
            r_sign, r = int(rc == 1), Fraction(0)
        else:
            r_sign = a.sign ^ (d < 0)
            r = round53(r_sign, abs(d), rc)
    c = ((low3 >> 1) & 1) << 14 | (low3 & 1) << 9 | ((low3 >> 2) & 1) << 8
    return pack(r_sign, r), c


# ---------------------------------------------------------------------------
# Operands
# ---------------------------------------------------------------------------

def val(x: float | Fraction | int, *, sign: int | None = None) -> int:
    """An exactly representable value as m80."""
    v = Fraction(x)
    s = int(v < 0) if sign is None else sign
    return pack(s, abs(v))


def rnd53(rng: random.Random, lo: int, hi: int) -> int:
    """Random 53-bit normal with unbiased exponent in [lo, hi]."""
    e = rng.randint(lo, hi)
    sig = ((1 << 52) | rng.getrandbits(52)) << 11
    return f80(rng.getrandbits(1), e + 16383, sig)


def rnd64(rng: random.Random, lo: int, hi: int) -> int:
    e = rng.randint(lo, hi)
    return f80(rng.getrandbits(1), e + 16383, (1 << 63) | rng.getrandbits(63))


POS_INF = f80(0, 0x7FFF, 1 << 63)
NEG_INF = f80(1, 0x7FFF, 1 << 63)
QNAN = f80(0, 0x7FFF, 0xC123_4567_89AB_C800)
NEG_QNAN = f80(1, 0x7FFF, 0xD000_0000_0000_0000)
SNAN = f80(0, 0x7FFF, 0x8765_4321_0000_0800)
NEG_SNAN = f80(1, 0x7FFF, 0xA000_0000_0000_0000)
MAX_FINITE = f80(0, 0x7FFE, (1 << 64) - (1 << 11))
MIN_NORMAL = f80(0, 1, 1 << 63)
DENORM_A = f80(0, 0, 0x4000_0000_0000_0000)          # 2^-16383
DENORM_B = f80(1, 0, 0x0000_0123_4567_8800)
DENORM_TINY = f80(0, 0, 0x800)                        # 2^-16434, the last 53-bit bit
DENORM_SUB = f80(0, 0, 0x1)                           # below the 53 bits


def frndint_cases(rng: random.Random) -> list[tuple]:
    cases = []
    fixed = [0.5, -0.5, 1.5, 2.5, -2.5, 3.5, 0.25, 0.75, -0.75, 1.0, -1.0,
             0.49999999999999994, 0.5000000000000001, 2**52 - 0.5, -(2**52 - 0.5),
             2**52 - 1.5, 2**51 + 0.5, 2**53, -2**53, 2**53 - 1, 1e300, -1e-300,
             123456.75, -98765.25, 2**31 - 0.5, 4503599627370495.5, 7.0, 1e15 + 0.5]
    values = [val(x) for x in fixed]
    # 64-bit significands below 2^53: every discarded bit counts.
    values += [f80(0, 16383 + 52, (1 << 63) | 0x400),           # 2^52 + 0.5
               f80(1, 16383 + 52, (1 << 63) | 0xC00),           # -(2^52 + 1.5)
               f80(0, 16383 + 52, (1 << 63) | 0x401),
               f80(0, 16383 + 52, ((1 << 64) - 1)),             # 2^53 - 2^-11
               f80(1, 16383 + 51, ((1 << 64) - 1)),
               f80(0, 16383 - 1, (1 << 63) | 1),                # 0.5 + 2^-64
               f80(0, 16383 + 40, (1 << 63) | 0x7FF)]
    values += [0, f80(1, 0, 0), POS_INF, NEG_INF, QNAN, NEG_QNAN, SNAN, NEG_SNAN,
               DENORM_A, DENORM_B, DENORM_TINY, MIN_NORMAL, f80(1, 1, 1 << 63), MAX_FINITE]
    for raw in values:
        for rc in range(4):
            cases.append((raw, CW | rc << 10))
    # Carries into the next power of two: 2^k - 2^-j.
    for _ in range(40):
        k = rng.randint(1, 52)
        j = rng.randint(1, 52 - k + 1) if k < 52 else 1
        v = Fraction(2) ** k - Fraction(1, 2 ** j)
        cases.append((val(v, sign=rng.getrandbits(1)), CW | rng.randint(0, 3) << 10))
    for _ in range(220):
        cases.append((rnd53(rng, -3, 60), CW | rng.randint(0, 3) << 10))
    for _ in range(90):
        cases.append((rnd64(rng, -2, 52), CW | rng.randint(0, 3) << 10))
    # Unmasked: PE (an inexact result stays unwritten), IE.
    for raw in (val(2.5), val(-0.3), val(7.0), SNAN, DENORM_A):
        for rc in (0, 3):
            cases.append((raw, (CW & ~(PE | IE)) | rc << 10))
    return [("frndint", 0, raw, cw) for raw, cw in cases]


def fscale_cases(rng: random.Random) -> list[tuple]:
    cases = []
    st0s = [val(1.5), val(-1.5), val(3.0), val(-0.1), val(1e300), val(-7e-300),
            0, f80(1, 0, 0), POS_INF, NEG_INF, QNAN, SNAN, NEG_QNAN,
            DENORM_A, DENORM_B, DENORM_TINY, DENORM_SUB, MIN_NORMAL, MAX_FINITE,
            f80(1, 0x7FFE, 1 << 63)]
    st1s = [0, f80(1, 0, 0), val(3.0), val(-3.0), val(0.75), val(-0.999), val(2.9),
            val(-2.9), val(100.5), val(-100.5), val(16383.0), val(-16383.0),
            val(65535.9), val(-65536.0), val(1e10), val(-1e10), POS_INF, NEG_INF,
            QNAN, SNAN, DENORM_A, DENORM_B, f80(0, 16383 + 63, (1 << 64) - 1)]
    for a in st0s:
        for b in st1s:
            cases.append((a, b, CW))
    # Overflow and underflow in each rounding mode, masked and unmasked.
    edge = [(val(1.5), val(16383.0)), (val(-1.5), val(16383.0)), (MAX_FINITE, val(1.0)),
            (val(1.0), val(16384.0)), (val(-1.0), val(20000.0)), (MAX_FINITE, val(0.5)),
            (val(1.0), val(-16382.0)), (val(1.0), val(-16383.0)), (val(-1.0), val(-16434.0)),
            (val(1.0), val(-16435.0)), (val(-1.0), val(-16436.0)), (val(1.5), val(-16434.0)),
            (val(1.25), val(-16434.0)), (val(-1.75), val(-16435.0)), (val(1.0), val(-30000.0)),
            (f80(0, 16383, (1 << 64) - (1 << 11)), val(-16383.0)),   # rounds to 2^-16382
            (f80(1, 16383, (1 << 64) - (1 << 11)), val(-16383.0)),
            (val(3.0), val(-16384.0)), (val(-5.0), val(-16400.0)), (DENORM_B, val(-10.0)),
            (DENORM_TINY, val(-1.0)), (DENORM_TINY, val(0.0)), (DENORM_B, val(0.0)),
            (DENORM_A, val(1.0)), (DENORM_B, val(40.0))]
    for a, b in edge:
        for rc in range(4):
            cases.append((a, b, CW | rc << 10))
        cases.append((a, b, CW & ~(OE | UE)))
        cases.append((a, b, CW & ~PE))
    # Denormal operands with DE unmasked; invalid unmasked.
    for a, b in ((DENORM_A, val(2.0)), (val(2.0), DENORM_A), (SNAN, val(1.0)),
                 (POS_INF, NEG_INF), (0, POS_INF)):
        cases.append((a, b, CW & ~(DE | IE)))
    for _ in range(170):
        b = val(rng.randint(-200, 200) + rng.choice([0, 0.25, 0.5, 0.999]))
        cases.append((rnd53(rng, -100, 100), b, CW | rng.randint(0, 3) << 10))
    for _ in range(80):     # near the range ends
        hi = rng.getrandbits(1)
        n = rng.randint(16300, 16480)
        b = val(n if hi else -n)
        a = rnd53(rng, -60, 60)
        cases.append((a, b, CW | rng.randint(0, 3) << 10))
    for _ in range(40):     # fractional 64-bit scale factors
        cases.append((rnd53(rng, -20, 20), rnd64(rng, -3, 14), CW | rng.randint(0, 3) << 10))
    return [("fscale", 1, a, b, cw) for a, b, cw in cases]


def fprem_cases(rng: random.Random) -> list[tuple]:
    cases = []
    fixed = [(17.0, 5.0), (7.0, 2.0), (-7.0, 2.0), (7.0, -2.0), (1000.3, 3.7),
             (5.0, 5.0), (4.5, 1.5), (1.0, 3.0), (-1.0, 3.0), (2.5, 1.0), (3.5, 1.0),
             (1e15, 3.0), (123456789.0, 1000.0), (6.0, 4.0), (10.0, 4.0), (2**52, 3.0)]
    for x, y in fixed:
        for op in (2, 3):
            for rc in (0, 3):
                cases.append((op, val(x), val(y), CW | rc << 10))
    for _ in range(200):
        a = rnd53(rng, -10, 70)
        b = rnd53(rng, -10, 20)
        cases.append((rng.choice([2, 3]), a, b, CW | rng.randint(0, 3) << 10))
    return [("fprem", op, a, b, cw) for op, a, b, cw in cases]


# ---------------------------------------------------------------------------
# Programs
# ---------------------------------------------------------------------------

def expect(case) -> tuple[int, int, int, int, int]:
    """(op, cw, st1, st0, expected st0, expected status)."""
    kind = case[0]
    if kind == "frndint":
        _, op, a_raw, cw = case
        b_raw = 0
        res, flags, c1 = frndint_ref(X(a_raw), cw)
        c = c1 << 9
    elif kind == "fscale":
        _, op, a_raw, b_raw, cw = case
        res, flags, c1 = fscale_ref(X(a_raw), X(b_raw), cw)
        c = c1 << 9
    else:
        _, op, a_raw, b_raw, cw = case
        res, c = fprem_ref(X(a_raw), X(b_raw), cw, op == 3)
        flags = 0
    sw = status(flags, cw & 0x3F, 6, c)
    if sw & 0x80:
        res = a_raw         # unmasked: ST0 is not written
    return op, cw, b_raw, a_raw, res, sw


NAMES = {0: "frndint", 1: "fscale", 2: "fprem", 3: "fprem1"}


def write_program(path: Path, title: str, cases: list) -> None:
    assert len(cases) <= MAX_CASES, len(cases)
    asm = [
        f"; Generated by tests/x87_round_scale_tests.py: {title}",
        "; Expected results and status words from exact integer arithmetic",
        "; (see the script). Each case: FNINIT, FLDCW, FLD ST1, FLD ST0 (m80),",
        "; the operation, FNSTSW (all but B checked), FNCLEX, then ST0 and ST1",
        "; stored and compared (an unmasked exception leaves ST0 unwritten).",
        "; Fail code (port E4): case index + 1 in the low word, the status word",
        "; in the high word; then the value stored last.",
        "",
        "BITS 16",
        "org 0",
        "",
        "STATUS_PORT equ 0xE0",
        "DATA_PORT   equ 0xE4",
        "",
        "start:",
        "    cli",
        "    mov ax, cs",
        "    mov ds, ax",
        "    mov es, ax",
        "    mov ss, ax",
        "    mov sp, 0xfff0",
        "    cld",
        "    mov si, cases",
        "    xor bp, bp",
        "next:",
        f"    cmp bp, {len(cases)}",
        "    je pass",
        "    fninit",
        "    fldcw [si + 2]",
        "    fld tword [si + 4]",
        "    fld tword [si + 14]",
        "    mov al, [si]",
        "    cmp al, 1",
        "    je .fscale",
        "    cmp al, 2",
        "    je .fprem",
        "    cmp al, 3",
        "    je .fprem1",
        "    frndint",
        "    jmp .done",
        ".fscale:",
        "    fscale",
        "    jmp .done",
        ".fprem:",
        "    fprem",
        "    jmp .done",
        ".fprem1:",
        "    fprem1",
        ".done:",
        "    fnstsw ax",
        "    mov [status], ax",
        "    fnclex",
        f"    and ax, 0x{SW_CHECK:04x}",
        "    cmp ax, [si + 34]",
        "    jne fail",
        "    fstp tword [result]",
        "    lea di, [si + 24]",
        "    call compare",
        "    jne fail",
        "    fstp tword [result]",
        "    lea di, [si + 4]",
        "    call compare",
        "    jne fail",
        "    add si, 36",
        "    inc bp",
        "    jmp next",
        "compare:",
        "    push si",
        "    mov si, result",
        "    mov cx, 5",
        "    repe cmpsw",
        "    pop si",
        "    ret",
        "pass:",
        "    mov al, 1",
        "    out STATUS_PORT, al",
        "    hlt",
        "fail:",
        "    mov ax, [status]",
        "    shl eax, 16",
        "    mov ax, bp",
        "    inc ax",
        "    out DATA_PORT, eax",
        "    mov eax, [result]",
        "    out DATA_PORT, eax",
        "    mov eax, [result + 4]",
        "    out DATA_PORT, eax",
        "    mov ax, [result + 8]",
        "    out DATA_PORT, ax",
        "    mov al, 0xff",
        "    out STATUS_PORT, al",
        "    hlt",
        "",
        "align 2",
        "status: dw 0",
        "result: times 10 db 0",
        "align 2",
        "cases:",
        "; op, 0, cw, ST1, ST0, expected ST0, expected status & %04xh" % SW_CHECK,
    ]

    def t80(v: int) -> str:
        return ", ".join(f"0x{x:02x}" for x in v.to_bytes(10, "little"))

    for case in cases:
        op, cw, st1, st0, res, sw = expect(case)
        asm.append(f"    db {op}, 0  ; {NAMES[op]} {st0:020x} {st1:020x} -> {res:020x}")
        asm.append(f"    dw 0x{cw:04x}")
        asm.append(f"    db {t80(st1)}")
        asm.append(f"    db {t80(st0)}")
        asm.append(f"    db {t80(res)}")
        asm.append(f"    dw 0x{sw & SW_CHECK:04x}")
    path.write_text("\n".join(asm) + "\n")
    config = {
        "asm": path.name, "requires_x87": True, "start_mode": "real",
        "cr0": "0x00000010", "cr3": 0, "eip": 0,
        "initial_selectors": {"DS": 0, "SS": 0, "ES": 0, "FS": 0, "GS": 0},
        "d_init": 0, "cycles": 3000000, "page_tables": [],
    }
    path.with_suffix(".json").write_text(json.dumps(config, indent=2) + "\n")


def main() -> None:
    progs = Path(__file__).resolve().parent / "programs"
    rng = random.Random(486)
    frnd = frndint_cases(rng)
    fsc = fscale_cases(rng)
    fpr = fprem_cases(rng)
    write_program(progs / "x87_frndint.asm",
                  "FRNDINT in every rounding mode: halves, carries into the next "
                  "power of two, 64-bit operands below 2^53, zeros, infinities, "
                  "NaNs, denormals, PE and IE unmasked.", frnd)
    write_program(progs / "x87_fscale.asm",
                  "FSCALE: Intel's special cases, integer, fractional, negative, "
                  "huge and 64-bit scale factors, overflow and underflow in every "
                  "rounding mode, masked and unmasked, denormal operands.", fsc)
    write_program(progs / "x87_fprem.asm",
                  "FPREM and FPREM1 (the 53-bit divide, round to integer, multiply "
                  "and subtract): remainders and C0, C1, C3.", fpr)
    print(f"wrote x87_frndint ({len(frnd)}), x87_fscale ({len(fsc)}), "
          f"x87_fprem ({len(fpr)}) cases")


if __name__ == "__main__":
    main()
