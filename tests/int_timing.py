#!/usr/bin/env python3
"""Integer instruction timing on the full z486 (reuses x87_timing.py: its
program frame, simulator run and per-cycle monitor log; build the simulator
with build_timing.sh).

    ./int_timing.py run|analyse [TEST_ID...]

INT_BITS=16 runs the loops as 16-bit code in real mode (CS=DS=SS=ES=1000h),
as CPUSPEED and Dr. Hardware do; 32 (default) in the flat 32-bit protected
mode of x87_timing.py. Results go to int_timing_out16/ or int_timing_out32/
(X87T_OUT overrides). Each TEST_ID also prints one steady iteration cycle by
cycle: issue, stalls, prefetch and the microcode address.
"""
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BITS = int(os.environ.get("INT_BITS", "32"))
os.environ.setdefault("X87T_OUT", str(HERE / f"int_timing_out{BITS}"))
sys.path.insert(0, str(HERE))
import x87_timing as xt  # noqa: E402
T, rep = xt.T, xt.rep
xt.TESTS.clear()

if BITS == 16:
    A, B, D, P, SI, BP, SP, W = "ax", "bx", "dx", "bp", "si", "bp", "sp", "word"
else:
    A, B, D, P, SI, BP, SP, W = "eax", "ebx", "edx", "ebp", "esi", "ebp", "esp", "dword"
M0 = f"{W} ptr [{SI}]"
M4 = f"{W} ptr [{SI}+4]"
M8 = f"{W} ptr [{SI}+8]"
# llvm-mc makes 66h-prefixed 32-bit CALL/RET in .code16; 16-bit code gets the plain
# E8 rel16 and C3 that DOS compilers emit.
def CALL(fn):
    return f".byte 0xe8\n  .short {fn} - (. + 2)" if BITS == 16 else f"call {fn}"
RET = ".byte 0xc3" if BITS == 16 else "ret"

setup = [f"mov {SI}, 0x8000", f"mov {D}, 0", f"mov {P}, 0"]
T("loop overhead (dec ecx; jnz)", [], [], n=64)
T("ADD r,r", [], rep([f"add {A}, {B}"], 8), n=32)
T("MOV r,m", setup, rep([f"mov {A}, {M0}"], 8), n=32)
T("ADD r,m (dependent chain)", setup, rep([f"add {A}, {M0}"], 8), n=32)
T("ADD r,m (two chains)", setup, rep([f"add {A}, {M0}", f"add {D}, {M4}"], 4), n=32)
T("ADD r,m + ADD r,r", setup, rep([f"add {A}, {M0}", f"add {D}, {B}"], 4), n=32)
T("ADD r,m + ADD r,r using it", setup, rep([f"add {A}, {M0}", f"add {D}, {A}"], 4), n=32)
T("ADD r,m + MOV m,r storing it", setup, rep([f"add {A}, {M0}", f"mov {M8}, {A}"], 4), n=32)
T("CMP r,m", setup, rep([f"cmp {A}, {M0}"], 8), n=32)
T("CMP m,r", setup, rep([f"cmp {M0}, {A}"], 8), n=32)
T("MOV r,m + ADD r,r (load then use)", setup, rep([f"mov {D}, {M0}", f"add {A}, {D}"], 4), n=32)
T("PUSH r + POP r (adjacent)", [], rep([f"push {A}", f"pop {A}"], 4), n=32)
T("PUSH x4 + POP x4", [], rep([f"push {A}"], 4) + rep([f"pop {D}"], 4), n=32)
T("PUSH, 2 ADD, POP", [], rep([f"push {A}", f"add {B}, 1", f"add {P}, 1", f"pop {D}"], 2), n=32)
T("CALL + RET", [], rep([CALL("fn_ret")], 8), n=32)
T("CALL + ADD + RET", [], rep([CALL("fn_add")], 8), n=32)
T("CALL + C frame + RET", [], rep([CALL("fn_frame")], 4), n=32)
T("JMP short (taken)", [], rep(["jmp 1f\n1:"], 8), n=32)
T("CMP + JNE taken", [], rep([f"cmp {A}, {B}\n  jne 1f\n1:"], 8), n=32)
T("CMP + JE not taken", [], rep([f"cmp {A}, {B}\n  je 1f\n1:"], 8), n=32)
T("SHL r,1", [], rep([f"shl {A}, 1"], 8), n=32)
T("SHL r,4", [], rep([f"shl {A}, 4"], 8), n=32)

FUNCS = f"""
  .p2align 4
fn_ret:
  {RET}
  .p2align 4
fn_add:
  add {A}, {B}
  {RET}
  .p2align 4
fn_frame:
  push {BP}
  mov {BP}, {SP}
  add {A}, {W} ptr [{BP}+{BITS // 8}]
  pop {BP}
  {RET}
"""
_gen = xt.gen_asm


def gen_asm():
    src = _gen()
    if BITS == 16:
        src = src.replace(".code32", ".code16").replace("mov esp, 0x30000", "mov esp, 0xF000")
    return src + FUNCS


xt.gen_asm = gen_asm


def simulate16(hexf, cycles=40_000_000):
    log = xt.OUT / "x87_cycles.log"
    cmd = [str(xt.SIM), f"+mem={hexf}", f"+cycles={cycles}", "+eip=0", "+cr0=0", "+cr3=0",
           f"+code_phys_base={0x10000}", "+start_protected=0", "+init_cs=4096",
           "+init_ds=4096", "+init_ss=4096", "+init_es=4096", "+init_fs=0", "+init_gs=0",
           "+d_init=0", f"+ds_base={0x10000}", f"+ss_base={0x10000}", f"+es_base={0x10000}",
           f"+x87log={log}"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    (xt.OUT / "sim_stdout.txt").write_text(r.stdout + r.stderr)
    if "TEST PASSED" not in r.stdout:
        print(r.stdout[-3000:], r.stderr[-2000:])
        raise SystemExit("simulation did not pass")
    return log


def trace(tid, rows, syms):
    """One steady iteration (the second-to-last), cycle by cycle."""
    loop = syms[f"t{tid}_loop"]
    starts = [i for i, r in enumerate(rows) if r[1] == loop and xt.bit(r[4], xt.C_ISSUE)]
    if len(starts) < 3:
        print(f"  (no steady iteration found for test {tid})")
        return
    a, b = starts[-3], starts[-2]
    names = {v: k for k, v in syms.items() if not k.startswith(".")}
    flags = [(xt.C_ISSUE, "I"), (xt.C_FIRST, "F"), (xt.C_UCACT, "U"), (xt.C_UCEXEC, "X"),
             (xt.C_STMEM, "M"), (xt.C_STALL, "S"), (xt.C_REQWR, "W"), (xt.C_MEMSERV, "V"),
             (2, "P"), (1, "D")]
    print(f"  cycles {rows[b][0] - rows[a][0]}   I issue F first U uc_active X uc_exec "
          f"M stall_mem S stall W dcache write V mem_servicing P pb_valid D d2_ready")
    for r in rows[a:b]:
        cyc, eip, op, modrm, cpu = r[0], r[1], r[2], r[3], r[4]
        f = "".join(c if xt.bit(cpu, bb) else "." for bb, c in flags)
        lab = names.get(eip, "")
        print(f"  {cyc:8d} eip {eip:05x} {lab:14s} op {op:02x} {modrm:02x}  {f}  uc {r[8]:03x}")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "run"
    if mode == "run":
        hexf, syms = xt.assemble(xt.gen_asm())
        log = simulate16(hexf) if BITS == 16 else xt.simulate(hexf)
    else:
        import json
        syms = json.loads((xt.OUT / "symbols.json").read_text())
        log = xt.OUT / "x87_cycles.log"
    code = (xt.OUT / "x87_timing.bin").read_bytes()
    res = xt.analyse(log, syms, code)
    for r in res:
        print(f"[{r['id']:2d}] {r['name']:36s} iter={r['iter_mean']:7.2f} "
              f"(iter-loop)/k={r['per_instr']:6.2f}")
        for g in r["groups"]:
            t = g["text"].replace("\n", " ").replace("  ", " ")
            print(f"     {t:34s} x{g['lines']} issue->issue={g['mean']:6.2f}  cpu: {xt.fmt(g['cpu'])}")
    want = [int(x) for x in sys.argv[2:]]
    if want:
        windows = xt.parse_log(log)
        for tid in want:
            print(f"\n== test {tid}: {xt.TESTS[tid - 1]['name']}")
            trace(tid, windows[tid], syms)


if __name__ == "__main__":
    main()
