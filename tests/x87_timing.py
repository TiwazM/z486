#!/usr/bin/env python3
"""x87 instruction timing on the full z486 (Verilator, ENABLE_X87=1).

Generates one 32-bit protected-mode program (flat CS/DS/SS, base 0x10000, no
paging) holding a steady loop per measurement, assembles it with llvm-mc,
runs obj_dir_timing/Vtb_protected_mode (built by build_timing.sh, which
includes x87_timing_mon.svh) and analyses the per-cycle log the monitor
writes for every measurement window.

Each loop runs twice: a warm-up pass (I/D caches, TLB) and the measured pass,
which is bracketed by OUT 0xD0 markers. The loop ends with FWAIT so the
window includes completion of the last x87 operation.

    ./x87_timing.py run      # generate, assemble, simulate, analyse
    ./x87_timing.py analyse  # re-analyse the last log
"""

import json
import os
import re
import statistics
import struct
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path(os.environ.get("X87T_OUT", HERE / "x87_timing_out"))
SIM = HERE / "obj_dir_timing" / "Vtb_protected_mode"
LLVM = Path(os.environ.get("LLVM_BIN", "/opt/homebrew/opt/llvm/bin"))

# ---------------------------------------------------------------------------
# Data (DS offsets; DS base = 0x10000)
# ---------------------------------------------------------------------------
A = 1.2345678901234567
AINV = 1.0 / A
DATA = {}          # name -> (addr, bytes)
_next = [0x8000]


def data(name, raw, align=16):
    addr = (_next[0] + align - 1) // align * align
    DATA[name] = (addr, raw)
    _next[0] = addr + len(raw)
    return addr


def f32(x): return struct.pack("<f", x)
def f64(x): return struct.pack("<d", x)


def f80(x):
    # exact conversion of a double to extended (normal values only)
    bits = struct.unpack("<Q", struct.pack("<d", x))[0]
    sign = bits >> 63
    exp = (bits >> 52) & 0x7FF
    frac = bits & ((1 << 52) - 1)
    e80 = exp - 1023 + 16383
    m = (1 << 63) | (frac << 11)
    return struct.pack("<QH", m, (sign << 15) | e80)


data("A32", f32(A))
data("AINV32", f32(AINV))
data("A64", f64(A))
data("AINV64", f64(AINV))
data("A80", f80(A))
data("V1_0000123", f64(1.0000123456789))
data("V0_7", f64(0.7))
data("V0_4", f64(0.4))
data("V2_7", f64(2.7))
data("V3_7", f64(3.7))
data("V1000_3", f64(1000.3))
data("V1234_56", f64(1234.5678))
data("V1_7", f64(1.7))
data("V1_5", f64(1.5))
data("V2_5", f64(2.5))
data("V3_5", f64(3.5))
data("V4_5", f64(4.5))
data("V1_3", f64(1.3))
data("V0_6", f64(0.6))
data("I16", struct.pack("<h", 1234))
data("I32", struct.pack("<i", 123456))
data("I64", struct.pack("<q", 1234567890123))
data("CW", struct.pack("<H", 0x037F))
data("CW_PC24", struct.pack("<H", 0x007F))
data("CW_PC53", struct.pack("<H", 0x027F))
data("CW_TRUNC", struct.pack("<H", 0x0F7F))
data("S16", b"\0" * 2)
data("S32", b"\0" * 4)
data("S64", b"\0" * 8)
data("S80", b"\0" * 10)


def D(name):
    return f"0x{DATA[name][0]:x}"


def m32(n): return f"dword ptr [{D(n)}]"
def m64(n): return f"qword ptr [{D(n)}]"
def m80(n): return f"tbyte ptr [{D(n)}]"
def m16(n): return f"word ptr [{D(n)}]"


# ---------------------------------------------------------------------------
# Measurements
# ---------------------------------------------------------------------------
# Each test: name, setup (list of asm lines, run after FNINIT), body (one
# unrolled block, list of asm lines), iterations, and `focus`: the body lines
# whose cost is the measurement (others are helpers whose own steady cost is
# subtracted to give a net figure).
TESTS = []


def T(name, setup, body, n=16, focus=None, helpers=None, intel=None,
      note=""):
    TESTS.append(dict(name=name, setup=setup, body=body, n=n,
                      focus=focus, helpers=helpers or [], intel=intel,
                      note=note))


def rep(lines, k):
    return [l for _ in range(k) for l in lines]


ld_a = [f"fld {m64('A64')}"]

# baseline: the loop alone
T("loop overhead (dec ecx; jnz)", [], [], n=64)

# loads / stores: 4 pushes then 4 pops, per-line attribution
T("FLD m32 / FSTP m32", [], rep([f"fld {m32('A32')}"], 4) + rep([f"fstp {m32('S32')}"], 4), n=32)
T("FLD m64 / FSTP m64", [], rep([f"fld {m64('A64')}"], 4) + rep([f"fstp {m64('S64')}"], 4), n=32)
T("FLD m80 / FSTP m80", [], rep([f"fld {m80('A80')}"], 4) + rep([f"fstp {m80('S80')}"], 4), n=32)
T("FLD ST(i) / FSTP ST(0)", ld_a, rep(["fld st(0)"], 4) + rep(["fstp st(0)"], 4), n=32)
T("FLD1 / FSTP ST(0)", [], rep(["fld1"], 4) + rep(["fstp st(0)"], 4), n=32)
T("FLDZ / FSTP ST(0)", [], rep(["fldz"], 4) + rep(["fstp st(0)"], 4), n=32)
T("FST m32", ld_a, rep([f"fst {m32('S32')}"], 8), n=32)
T("FST m64", ld_a, rep([f"fst {m64('S64')}"], 8), n=32)
T("FST ST(i)", ld_a + ["fld st(0)"], rep(["fst st(1)"], 8), n=32)
T("FXCH", ld_a + [f"fld {m64('V1_7')}"], rep(["fxch st(1)"], 8), n=32)

# integer conversions
T("FILD m16 / FISTP m16", [], rep([f"fild {m16('I16')}"], 4) + rep([f"fistp {m16('S16')}"], 4), n=32)
T("FILD m32 / FISTP m32", [], rep([f"fild {m32('I32')}"], 4) + rep([f"fistp {m32('S32')}"], 4), n=32)
T("FILD m64 / FISTP m64", [], rep([f"fild {m64('I64')}"], 4) + rep([f"fistp {m64('S64')}"], 4), n=32)
T("FIST m16", [f"fld {m64('V1234_56')}"], rep([f"fist {m16('S16')}"], 8), n=32)
T("FIST m32", [f"fld {m64('V1234_56')}"], rep([f"fist {m32('S32')}"], 8), n=32)

# arithmetic, register forms
st1_a = [f"fld {m64('A64')}", f"fld {m64('V1_7')}"]            # ST0=1.7 ST1=a
st12_a = [f"fld {m64('AINV64')}", f"fld {m64('A64')}", f"fld {m64('V1_7')}"]  # ST0=1.7 ST1=a ST2=1/a
T("FADD ST,ST(1) dependent", st1_a, rep(["fadd st, st(1)"], 8), n=32)
T("FSUB ST,ST(1) dependent", st1_a, rep(["fsub st, st(1)"], 8), n=32)
T("FMUL ST,ST(i) dependent", st12_a, rep(["fmul st, st(1)", "fmul st, st(2)"], 4), n=32)
T("FDIV ST,ST(i) dependent", st12_a, rep(["fdiv st, st(1)", "fdiv st, st(2)"], 4), n=16)
indep = [f"fld {m64('V4_5')}", f"fld {m64('V3_5')}", f"fld {m64('V2_5')}",
         f"fld {m64('V1_5')}", f"fld {m64('V1_0000123')}"]
T("FADD ST(i),ST independent", indep, rep(["fadd st(1), st", "fadd st(2), st", "fadd st(3), st", "fadd st(4), st"], 2), n=32)
T("FMUL ST(i),ST independent", indep, rep(["fmul st(1), st", "fmul st(2), st", "fmul st(3), st", "fmul st(4), st"], 2), n=32)
T("FDIV ST(i),ST independent", indep, rep(["fdiv st(1), st", "fdiv st(2), st", "fdiv st(3), st", "fdiv st(4), st"], 2), n=16)
T("FDIV ST,ST(i) PC=24", [f"fldcw {m16('CW_PC24')}"] + st12_a, rep(["fdiv st, st(1)", "fdiv st, st(2)"], 4), n=16)
T("FDIV ST,ST(i) PC=53", [f"fldcw {m16('CW_PC53')}"] + st12_a, rep(["fdiv st, st(1)", "fdiv st, st(2)"], 4), n=16)

# arithmetic, memory forms (dependent through ST0)
st0 = [f"fld {m64('V1_7')}"]
T("FADD m32", st0, rep([f"fadd {m32('A32')}"], 8), n=32)
T("FADD m64", st0, rep([f"fadd {m64('A64')}"], 8), n=32)
T("FMUL m32", st0, rep([f"fmul {m32('A32')}", f"fmul {m32('AINV32')}"], 4), n=32)
T("FMUL m64", st0, rep([f"fmul {m64('A64')}", f"fmul {m64('AINV64')}"], 4), n=32)
T("FDIV m32", st0, rep([f"fdiv {m32('A32')}", f"fdiv {m32('AINV32')}"], 4), n=16)
T("FDIV m64", st0, rep([f"fdiv {m64('A64')}", f"fdiv {m64('AINV64')}"], 4), n=16)

# compares and status
T("FCOM ST(1) + FNSTSW AX", st1_a, rep(["fcom st(1)", "fnstsw ax"], 4), n=32)
T("FCOM m32 + FNSTSW AX", st0, rep([f"fcom {m32('A32')}", "fnstsw ax"], 4), n=32)
T("FCOMP ST(1) + FNSTSW AX", st1_a, rep(["fld st(0)", "fcomp st(2)", "fnstsw ax"], 4), n=32,
  focus=["fcomp st(2)", "fnstsw ax"], helpers=["fld st(0)"])
T("FNSTSW AX", st0, rep(["fnstsw ax"], 8), n=32)
T("FTST", st0, rep(["ftst"], 8), n=32)
T("FCHS", st0, rep(["fchs"], 8), n=32)
T("FABS", st0, rep(["fabs"], 8), n=32)
T("FLDCW", [], rep([f"fldcw {m16('CW')}"], 8), n=32)
T("FNSTCW", [], rep([f"fnstcw {m16('S16')}"], 8), n=32)

# longer operations: copy the argument, operate, discard
T("FSQRT", [f"fld {m64('V2_7')}"], rep(["fld st(0)", "fsqrt", "fstp st(0)"], 2), n=16,
  focus=["fsqrt"], helpers=["fld st(0)", "fstp st(0)"])
T("FRNDINT", [f"fld {m64('V2_7')}"], rep(["fld st(0)", "frndint", "fstp st(0)"], 2), n=16,
  focus=["frndint"], helpers=["fld st(0)", "fstp st(0)"])
T("FSCALE", [f"fld1", f"fld {m64('V1_3')}"], rep(["fscale"], 4), n=16)
T("FPREM", [f"fld {m64('V1000_3')}", f"fld {m64('V3_7')}"], rep(["fld st(1)", "fprem", "fstp st(0)"], 2), n=16,
  focus=["fprem"], helpers=["fld st(1)", "fstp st(0)"])
T("FSIN", [f"fld {m64('V0_7')}"], rep(["fld st(0)", "fsin", "fstp st(0)"], 1), n=16,
  focus=["fsin"], helpers=["fld st(0)", "fstp st(0)"])
T("FCOS", [f"fld {m64('V0_7')}"], rep(["fld st(0)", "fcos", "fstp st(0)"], 1), n=16,
  focus=["fcos"], helpers=["fld st(0)", "fstp st(0)"])
T("FPTAN", [f"fld {m64('V0_7')}"], rep(["fld st(0)", "fptan", "fstp st(0)", "fstp st(0)"], 1), n=16,
  focus=["fptan"], helpers=["fld st(0)", "fstp st(0)", "fstp st(0)"])
T("FPATAN", [f"fld {m64('V0_6')}", f"fld {m64('V1_3')}"], rep(["fld st(1)", "fld st(1)", "fpatan", "fstp st(0)"], 1), n=16,
  focus=["fpatan"], helpers=["fld st(0)", "fld st(0)", "fstp st(0)"])
T("F2XM1", [f"fld {m64('V0_4')}"], rep(["fld st(0)", "f2xm1", "fstp st(0)"], 1), n=16,
  focus=["f2xm1"], helpers=["fld st(0)", "fstp st(0)"])
T("FYL2X", [f"fld {m64('V1_3')}", f"fld {m64('V2_7')}"], rep(["fld st(1)", "fld st(1)", "fyl2x", "fstp st(0)"], 1), n=16,
  focus=["fyl2x"], helpers=["fld st(0)", "fld st(0)", "fstp st(0)"])

# extra: value-stable FADD, isolated FILD, FWAIT, compiler-style sequences
T("FADD/FSUB ST,ST(1) alternating (|dexp|<=1)", st1_a, rep(["fadd st, st(1)", "fsub st, st(1)"], 4), n=32)
T("FADD/FSUB m32 alternating", st0, rep([f"fadd {m32('A32')}", f"fsub {m32('A32')}"], 4), n=32)
T("FADD/FSUB m64 alternating", st0, rep([f"fadd {m64('A64')}", f"fsub {m64('A64')}"], 4), n=32)
T("FILD m16 / FSTP ST(0)", [], rep([f"fild {m16('I16')}"], 4) + rep(["fstp st(0)"], 4), n=32)
T("FILD m32 / FSTP ST(0)", [], rep([f"fild {m32('I32')}"], 4) + rep(["fstp st(0)"], 4), n=32)
T("FWAIT (x87 idle)", [], rep(["fwait"], 8), n=32)
T("FCOMP m64 + FNSTSW AX + SAHF", st0, rep(["fld st(0)", f"fcomp {m64('A64')}", "fnstsw ax", "sahf"], 4), n=32,
  focus=[f"fcomp {m64('A64')}", "fnstsw ax"], helpers=["fld st(0)"])
T("expr m64: FLD/FMUL/FADD/FSTP m64", [], rep([f"fld {m64('A64')}", f"fmul {m64('AINV64')}", f"fadd {m64('A64')}", f"fstp {m64('S64')}"], 2), n=32)
T("expr m32: FLD/FMUL/FADD/FSTP m32", [], rep([f"fld {m32('A32')}", f"fmul {m32('AINV32')}", f"fadd {m32('A32')}", f"fstp {m32('S32')}"], 2), n=32)
T("C ftol: FLD ST/FNSTCW/FLDCW/FISTP m32/FLDCW", [f"fld {m64('V1234_56')}"],
  ["fld st(0)", f"fnstcw {m16('S16')}", f"fldcw {m16('CW_TRUNC')}", f"fistp {m32('S32')}", f"fldcw {m16('CW')}"], n=32)

# integer overlap with a long x87 operation
for nadd in (0, 8, 16, 32, 48, 64, 96, 128):
    T(f"FDIV + {nadd} ADD", st12_a, ["fdiv st, st(1)"] + ["add eax, ebx"] * nadd, n=48,
      focus=["fdiv st, st(1)"])
for nadd in (0, 32, 64, 96, 128):
    T(f"FSQRT + {nadd} ADD", [f"fld {m64('V2_7')}"], ["fsqrt"] + ["add eax, ebx"] * nadd, n=48,
      focus=["fsqrt"])
for nadd in (0, 32, 64, 96):
    T(f"FDIV m32 + {nadd} ADD", st0, [f"fdiv {m32('A32')}"] + ["add eax, ebx"] * nadd + [f"fdiv {m32('AINV32')}"] + ["add eax, ebx"] * nadd, n=32,
      focus=[f"fdiv {m32('A32')}", f"fdiv {m32('AINV32')}"])
for nadd in (0, 32, 64, 96):
    T(f"FDIV m64 + {nadd} ADD", st0, [f"fdiv {m64('A64')}"] + ["add eax, ebx"] * nadd + [f"fdiv {m64('AINV64')}"] + ["add eax, ebx"] * nadd, n=32,
      focus=[f"fdiv {m64('A64')}", f"fdiv {m64('AINV64')}"])
for nadd in (0, 4, 8, 16):
    T(f"FMUL + {nadd} ADD", st12_a, ["fmul st, st(1)", "fmul st, st(2)"] + ["add eax, ebx"] * nadd, n=48,
      focus=["fmul st, st(1)", "fmul st, st(2)"])


# ---------------------------------------------------------------------------
# Program generation
# ---------------------------------------------------------------------------
def gen_asm():
    L = [".intel_syntax noprefix", ".code32", "start:", "  cli", "  cld",
         "  mov esp, 0x30000"]
    for name, (addr, raw) in DATA.items():
        raw = raw + b"\0" * (-len(raw) % 2)
        i = 0
        while i < len(raw):
            if len(raw) - i >= 4:
                v = struct.unpack("<I", raw[i:i + 4])[0]
                L.append(f"  mov dword ptr [0x{addr + i:x}], 0x{v:08x}")
                i += 4
            else:
                v = struct.unpack("<H", raw[i:i + 2])[0]
                L.append(f"  mov word ptr [0x{addr + i:x}], 0x{v:04x}")
                i += 2
    for tid, t in enumerate(TESTS, start=1):
        L.append(f"# test {tid}: {t['name']}")
        L.append("  mov edi, 2")
        L.append(f"t{tid}_pass:")
        L.append("  fninit")
        L += ["  " + s for s in t["setup"]]
        L.append("  xor eax, eax")
        L.append("  mov ebx, 3")
        L.append(f"  mov ecx, {t['n']}")
        L.append("  cmp edi, 1")
        L.append(f"  jne t{tid}_nomark1")
        L.append(f"  mov eax, 0x{0x80000000 | tid:08x}")
        L.append("  out 0xd0, eax")
        L.append("  xor eax, eax")
        L.append(f"t{tid}_nomark1:")
        L.append("  .p2align 4")
        L.append(f"t{tid}_loop:")
        for j, s in enumerate(t["body"]):
            L.append(f"t{tid}_b{j}:")
            L.append("  " + s)
        L.append(f"t{tid}_dec:")
        L.append("  dec ecx")
        L.append(f"  jnz t{tid}_loop")
        L.append(f"t{tid}_end:")
        L.append("  fwait")
        L.append("  cmp edi, 1")
        L.append(f"  jne t{tid}_nomark2")
        L.append("  xor eax, eax")
        L.append("  out 0xd0, eax")
        L.append(f"t{tid}_nomark2:")
        L.append("  dec edi")
        L.append(f"  jnz t{tid}_pass")
    L += ["  mov al, 1", "  out 0xe0, al", "  hlt", "1: jmp 1b"]
    return "\n".join(L) + "\n"


def assemble(src):
    OUT.mkdir(parents=True, exist_ok=True)
    s = OUT / "x87_timing.s"
    o = OUT / "x87_timing.o"
    b = OUT / "x87_timing.bin"
    s.write_text(src)
    subprocess.run([str(LLVM / "llvm-mc"), "-triple=i386-unknown-none-elf",
                    "-x86-asm-syntax=intel", "-filetype=obj", "-o", str(o), str(s)],
                   check=True)
    subprocess.run([str(LLVM / "llvm-objcopy"), "-O", "binary", "-j", ".text",
                    str(o), str(b)], check=True)
    # symbol table: label -> offset
    nm = subprocess.run([str(LLVM / "llvm-nm"), str(o)], capture_output=True,
                        text=True, check=True).stdout
    syms = {}
    for line in nm.splitlines():
        p = line.split()
        if len(p) == 3:
            syms[p[2]] = int(p[0], 16)
    code = b.read_bytes()
    assert len(code) < DATA["A32"][0], "code overlaps data"
    mem = bytearray(0x10000 + 0x10000)
    mem[0x10000:0x10000 + len(code)] = code
    hexf = OUT / "x87_timing.hex"
    hexf.write_text("".join(f"{x:02X}\n" for x in mem))
    (OUT / "symbols.json").write_text(json.dumps(syms, indent=0))
    return hexf, syms


def simulate(hexf, cycles=40_000_000):
    log = OUT / "x87_cycles.log"
    seg = {"cs": (0x10000, 0xFFFFF, 0xA9E0), "ds": (0x10000, 0xFFFFF, 0x29E0),
           "ss": (0x10000, 0xFFFFF, 0x29E0), "es": (0x10000, 0xFFFFF, 0x29E0),
           "fs": (0, 0xFFFFF, 0x29E0), "gs": (0, 0xFFFFF, 0x29E0)}
    cmd = [str(SIM), f"+mem={hexf}", f"+cycles={cycles}", "+eip=0", "+cr0=1",
           "+cr3=0", f"+code_phys_base={0x10000}", "+start_protected=1",
           "+init_cs=8", "+init_ds=16", "+init_ss=24", "+init_es=32",
           "+init_fs=40", "+init_gs=48", "+d_init=1", f"+x87log={log}"]
    for k, (b, l, f) in seg.items():
        cmd += [f"+{k}_base={b}", f"+{k}_limit={l}", f"+{k}_flags={f}"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    (OUT / "sim_stdout.txt").write_text(r.stdout + r.stderr)
    if "TEST PASSED" not in r.stdout:
        print(r.stdout[-3000:], r.stderr[-2000:])
        raise SystemExit("simulation did not pass")
    return log


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
# cpu flags
C_ISSUE, C_FIRST, C_UCACT, C_UCEXEC = 15, 14, 13, 12
C_STMEM, C_STWIO, C_STX87D, C_STALL = 11, 10, 9, 8
C_X87REQ, C_X87ACC, C_REQWR, C_REQFC, C_MEMSERV = 7, 6, 5, 4, 3
# unit flags
U_DACT, U_DISS, U_DVALID, U_DREADY, U_DREG, U_DCMDP, U_DST, U_POSTED = 15, 14, 13, 12, 11, 10, 9, 8
# control flags
K_CMDV, K_CMDR, K_CORE_R, K_BUSYN, K_PEREQ, K_CMDPEND, K_DPEND = 31, 30, 29, 28, 27, 26, 25
K_EXPEND, K_EXBUSY, K_EXDONE, K_PUSH, K_RESW, K_MMATH, K_STORE, K_POP, K_STW = 24, 23, 22, 21, 20, 19, 18, 17, 16
K_WIN, K_RDREQ, K_RDRESP, K_STATRD, K_CCPULSE, K_PRHOLD = 15, 14, 13, 12, 11, 10
K_POLY, K_FPREM, K_FSCALE, K_LOGEXP, K_FPTAN, K_BCD = 9, 8, 7, 6, 5, 4


def bit(v, b):
    return (v >> b) & 1


def parse_log(log):
    windows = {}
    cur = None
    with open(log) as f:
        for line in f:
            p = line.split()
            if p[0] == "O":
                cur = []
                windows[int(p[1])] = cur
                continue
            if p[0] == "X":
                cur = None
                continue
            if cur is None:
                continue
            cur.append((int(p[0]), int(p[1], 16), int(p[2], 16), int(p[3], 16),
                        int(p[4], 16), int(p[5], 16), int(p[6], 16),
                        int(p[7], 16), int(p[8], 16), int(p[9], 16), int(p[10], 16),
                        int(p[11], 16), int(p[12], 16)))
    return windows


def x87_phase(cpu, unit, ctl, kinds):
    """x87-side occupancy of one cycle, highest-priority first."""
    rx = kinds >> 4
    tx = (kinds >> 2) & 3
    if bit(ctl, K_EXBUSY):
        return "exec"
    if bit(ctl, K_POLY) or bit(ctl, K_FPREM) or bit(ctl, K_FSCALE) or \
       bit(ctl, K_LOGEXP) or bit(ctl, K_FPTAN) or bit(ctl, K_BCD):
        return "ctl_seq"          # control-level program between executor ops
    if bit(ctl, K_EXPEND):
        return "exec_launch"
    if bit(ctl, K_EXDONE) or bit(ctl, K_PUSH) or bit(ctl, K_RESW) or \
       bit(ctl, K_POP) or bit(ctl, K_STW):
        return "retire"
    if bit(ctl, K_CMDPEND):
        return "decode"
    if rx != 0:
        return "rx_wait"          # command accepted; operand words still arriving
    if tx != 0:
        return "tx_wait"          # result queued for the CPU to read
    if bit(ctl, K_MMATH) or bit(ctl, K_STORE):
        return "ctl_operand"      # memory-math / store setup in control
    if bit(ctl, K_DPEND):
        return "direct_queue"
    if (unit & 3) != 0:
        return "bridge"
    return "idle"


def cpu_phase(cpu, unit, ctl):
    if bit(cpu, C_ISSUE):
        return "issue"
    if bit(cpu, C_STX87D):
        return "stall_x87_direct"
    if bit(cpu, C_STWIO):
        return "stall_wio"
    if bit(cpu, C_STMEM):
        if bit(cpu, C_X87REQ) or (unit & 3) != 0:
            return "stall_x87_port"
        return "stall_mem"
    if bit(cpu, C_UCEXEC):
        return "ucode"
    if bit(cpu, C_STALL):
        return "stall_other"
    if not bit(cpu, C_UCACT):
        return "frontend"
    return "other"


def body_fops(syms, code):
    """FOP ({opcode[2:0], modrm}) of every x87 body line, keyed (test, line)."""
    out = {}
    for tid, t in enumerate(TESTS, start=1):
        for j in range(len(t["body"])):
            a = syms[f"t{tid}_b{j}"]
            if 0xD8 <= code[a] <= 0xDF:
                out[(tid, j)] = ((code[a] & 7) << 8) | code[a + 1]
            elif code[a] == 0x9B and a + 1 < len(code) and 0xD8 <= code[a + 1] <= 0xDF:
                out[(tid, j)] = ((code[a + 1] & 7) << 8) | code[a + 2]
    return out


X87_BUSY_PHASES = ("decode", "rx_wait", "ctl_operand", "exec_launch", "exec",
                   "ctl_seq", "retire", "tx_wait")


def analyse(log, syms, code):
    windows = parse_log(log)
    fops = body_fops(syms, code)
    results = []
    for tid, t in enumerate(TESTS, start=1):
        rows = windows.get(tid)
        if rows is None:
            print(f"missing window {tid}")
            continue
        loop = syms[f"t{tid}_loop"]
        dec = syms[f"t{tid}_dec"]
        k = len(t["body"])
        line_addr = {syms[f"t{tid}_b{j}"]: j for j in range(k)}
        line_addr[dec] = "dec"
        line_addr[dec + 1] = "jnz"
        issues = [(i, r[0], r[1]) for i, r in enumerate(rows) if bit(r[4], C_ISSUE)]
        iter_starts = [c for (_, c, e) in issues if e == loop]
        # steady region: from the 2nd loop-head issue to the last one
        if len(iter_starts) >= 3:
            lo, hi = iter_starts[1], iter_starts[-1]
            niter = len(iter_starts) - 2
        else:
            lo, hi = rows[0][0], rows[-1][0] + 1
            niter = t["n"]
        it_mean = (hi - lo) / niter
        # issue windows inside the steady region
        per_line = defaultdict(list)
        cpu_line = defaultdict(Counter)
        for (i0, c0, e0), (i1, c1, _) in zip(issues, issues[1:]):
            if not (lo <= c0 < hi):
                continue
            key = line_addr.get(e0)
            if key is None:
                continue
            per_line[key].append(c1 - c0)
            seen_x87 = False
            for r in rows[i0:i1]:
                ph = cpu_phase(r[4], r[5], r[6])
                if bit(r[4], C_X87ACC) or bit(r[5], U_DACT) or bit(r[5], U_DST) or bit(r[5], U_DREG):
                    seen_x87 = True
                if ph == "ucode":
                    ph = "ucode_post" if seen_x87 else "ucode_pre"
                cpu_line[key][ph] += 1
        mean = lambda v: sum(v) / len(v) if v else 0.0
        loop_ovh = mean(per_line["dec"]) + mean(per_line["jnz"])
        # x87 command segmentation (serialised commands; decode = command_pending)
        seg_fop = []
        seg_ph = []
        seg_at = []
        cur = Counter()
        cur_fop = None
        cur_at = None
        gap = Counter()
        for r in rows:
            ph = x87_phase(r[4], r[5], r[6], r[7])
            if ph == "decode":
                if cur_fop is not None:
                    seg_fop.append(cur_fop)
                    seg_ph.append(cur)
                    seg_at.append(cur_at)
                cur = Counter(gap)       # wait before this command's decode
                gap = Counter()
                cur_fop = r[11]
                cur_at = r[0]
                cur["decode"] += 1
                continue
            if ph in X87_BUSY_PHASES:
                if gap:                  # a gap inside a command (e.g. FIFO wait)
                    for g, v in gap.items():
                        cur["mid_" + g] += v
                    gap = Counter()
                cur[ph] += 1
            else:
                gap["wait_" + ph] += 1
        # the window's last command is cut off by the closing marker: dropped
        keep = [(f, c) for f, c, a in zip(seg_fop, seg_ph, seg_at) if lo <= a < hi]
        seg_fop = [f for f, _ in keep]
        seg_ph = [c for _, c in keep]
        by_fop = defaultdict(lambda: [0, Counter()])
        for f, c in zip(seg_fop, seg_ph):
            by_fop[f][0] += 1
            by_fop[f][1].update(c)
        x87h = Counter()
        cpuh = Counter()
        exec_cycles = 0
        for r in rows:
            if lo <= r[0] < hi:
                x87h[x87_phase(r[4], r[5], r[6], r[7])] += 1
                cpuh[cpu_phase(r[4], r[5], r[6])] += 1
        lines = []
        for j, s_ in enumerate(t["body"]):
            v = per_line.get(j, [])
            lines.append(dict(text=s_, mean=mean(v), n=len(v),
                              fop=fops.get((tid, j)),
                              cpu={kk: vv / max(1, len(v)) for kk, vv in cpu_line[j].items()}))
        # group per text
        groups = {}
        for ln in lines:
            g = groups.setdefault(ln["text"], dict(text=ln["text"], fop=ln["fop"], sum=0.0, n=0,
                                                   cpu=Counter(), lines=0))
            g["sum"] += ln["mean"]
            g["lines"] += 1
            for kk, vv in ln["cpu"].items():
                g["cpu"][kk] += vv
        for g in groups.values():
            g["mean"] = g["sum"] / g["lines"]
            g["cpu"] = {kk: vv / g["lines"] for kk, vv in g["cpu"].items()}
        x87_by_fop = {}
        for f, (cnt, c) in by_fop.items():
            x87_by_fop[f"{f:03x}"] = dict(count=cnt, total=sum(c.values()) / cnt,
                                          phases={kk: vv / cnt for kk, vv in c.items()})
        # net cost of the focus lines
        focus_cost = None
        if t["focus"]:
            focus_cost = sum(g["mean"] * g["lines"] for g in groups.values()
                             if g["text"] in t["focus"])
        results.append(dict(
            id=tid, name=t["name"], n=t["n"], k=k, niter=niter,
            iter_mean=it_mean, loop_overhead=loop_ovh,
            per_instr=(it_mean - loop_ovh) / k if k else it_mean,
            groups=list(groups.values()),
            x87_by_fop=x87_by_fop,
            x87={kk: vv / niter for kk, vv in x87h.items()},
            cpu={kk: vv / niter for kk, vv in cpuh.items()},
            focus=t["focus"], helpers=t["helpers"], note=t["note"]))
    return results


PHASE_GROUPS = {
    "cpu_gap": ("wait_idle",),
    "bridge_xfer": ("wait_bridge", "rx_wait", "tx_wait", "mid_bridge", "mid_idle",
                    "mid_direct_queue"),
    "ctl_seq": ("decode", "ctl_operand", "exec_launch", "ctl_seq", "wait_direct_queue"),
    "exec": ("exec",),
    "retire": ("retire",),
}


def group_phases(ph):
    out = {g: 0.0 for g in PHASE_GROUPS}
    for k, v in ph.items():
        for g, members in PHASE_GROUPS.items():
            if k in members:
                out[g] += v
                break
        else:
            out.setdefault("other", 0.0)
            out["other"] += v
    return out


def fmt(d):
    return " ".join(f"{k}={v:.1f}" for k, v in sorted(d.items(), key=lambda kv: -kv[1]) if v >= 0.05)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "run"
    if mode == "run":
        hexf, syms = assemble(gen_asm())
        log = simulate(hexf)
    else:
        syms = json.loads((OUT / "symbols.json").read_text())
        log = OUT / "x87_cycles.log"
    code = (OUT / "x87_timing.bin").read_bytes()
    res = analyse(log, syms, code)
    (OUT / "results.json").write_text(json.dumps(res, indent=1))
    for r in res:
        print(f"[{r['id']:2d}] {r['name']:34s} iter={r['iter_mean']:7.2f} loop={r['loop_overhead']:4.1f} "
              f"(iter-loop)/k={r['per_instr']:7.2f}")
        for g in r["groups"]:
            fop = f"{g['fop']:03x}" if g["fop"] is not None else "   "
            print(f"     {g['text']:34s} x{g['lines']} fop={fop} issue->issue={g['mean']:6.2f}  cpu: {fmt(g['cpu'])}")
        for f, x in sorted(r["x87_by_fop"].items()):
            print(f"     x87 cmd {f} x{x['count']}: {x['total']:6.2f} = {fmt(x['phases'])}")
        print(f"     x87 occupancy/iter: {fmt(r['x87'])}")


if __name__ == "__main__":
    main()
