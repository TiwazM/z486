#!/usr/bin/env python3
"""Tables from x87_timing_out/results.json (run ./x87_timing.py first)."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import x87_timing as xt  # noqa: E402

res = {r["name"]: r for r in json.loads((xt.OUT / "results.json").read_text())}


def homog(test):
    return res[test]["per_instr"]


def line(test, text_prefix):
    for g in res[test]["groups"]:
        if g["text"].startswith(text_prefix):
            return g["mean"]
    raise KeyError((test, text_prefix))


def x87(test, fop):
    return res[test]["x87_by_fop"][fop]["total"]


def body(test, reps):
    """Cycles of one repetition of the loop body (the loop's own cost removed)."""
    r = res[test]
    return (r["iter_mean"] - r["loop_overhead"]) / reps


# (instruction, z486 cycles, Intel486 low, high, unsure?)
ROWS = [
    ("FLD m32", line("FLD m32 / FSTP m32", "fld dword"), 3, 3, False),
    ("FLD m64", line("FLD m64 / FSTP m64", "fld qword"), 3, 3, False),
    ("FLD m80", line("FLD m80 / FSTP m80", "fld tbyte"), 6, 6, True),
    ("FLD ST(i)", line("FLD ST(i) / FSTP ST(0)", "fld st(0)"), 4, 4, True),
    ("FST m32", homog("FST m32"), 7, 7, False),
    ("FSTP m32", line("FLD m32 / FSTP m32", "fstp dword"), 7, 7, False),
    ("FST m64", homog("FST m64"), 8, 8, True),
    ("FSTP m64", line("FLD m64 / FSTP m64", "fstp qword"), 8, 8, True),
    ("FSTP m80", line("FLD m80 / FSTP m80", "fstp tbyte"), 6, 6, True),
    ("FSTP ST(0)", line("FLD ST(i) / FSTP ST(0)", "fstp st(0)"), 3, 3, False),
    ("FXCH", homog("FXCH"), 4, 4, False),
    ("FADD ST,ST(i) dep. chain", homog("FADD ST,ST(1) dependent"), 8, 20, False),
    ("FADD/FSUB ST,ST(i) |dexp|<=1", homog("FADD/FSUB ST,ST(1) alternating (|dexp|<=1)"), 8, 20, False),
    ("FADD ST(i),ST independent", homog("FADD ST(i),ST independent"), 8, 20, False),
    ("FSUB ST,ST(i)", homog("FSUB ST,ST(1) dependent"), 8, 20, False),
    ("FMUL ST,ST(i) dep. chain", homog("FMUL ST,ST(i) dependent"), 16, 16, False),
    ("FMUL ST(i),ST independent", homog("FMUL ST(i),ST independent"), 16, 16, False),
    ("FDIV ST,ST(i) dep. chain", homog("FDIV ST,ST(i) dependent"), 73, 73, False),
    ("FDIV ST(i),ST independent", homog("FDIV ST(i),ST independent"), 73, 73, False),
    ("FADD m32", homog("FADD m32"), 8, 20, False),
    ("FADD m64", homog("FADD m64"), 8, 20, False),
    ("FMUL m32", homog("FMUL m32"), 11, 11, True),
    ("FMUL m64", homog("FMUL m64"), 14, 14, True),
    ("FDIV m32", homog("FDIV m32"), 73, 73, False),
    ("FDIV m64", homog("FDIV m64"), 73, 73, False),
    ("FILD m16", line("FILD m16 / FSTP ST(0)", "fild word"), 13, 16, False),
    ("FILD m32", line("FILD m32 / FSTP ST(0)", "fild dword"), 9, 12, True),
    ("FILD m64", line("FILD m64 / FISTP m64", "fild qword"), 10, 18, True),
    ("FIST m16", homog("FIST m16"), 29, 34, False),
    ("FIST m32", homog("FIST m32"), 28, 34, True),
    ("FISTP m16", line("FILD m16 / FISTP m16", "fistp word"), 29, 34, False),
    ("FISTP m32", line("FILD m32 / FISTP m32", "fistp dword"), 28, 34, True),
    ("FISTP m64", line("FILD m64 / FISTP m64", "fistp qword"), 28, 34, True),
    ("FCOM ST(1) + FNSTSW AX (pair)", 2 * homog("FCOM ST(1) + FNSTSW AX"), 7, 7, False),
    ("FCOM m32 + FNSTSW AX (pair)", 2 * homog("FCOM m32 + FNSTSW AX"), 7, 7, False),
    ("FCOMP ST(2) + FNSTSW AX (pair)", line("FCOMP ST(1) + FNSTSW AX", "fcomp") +
     line("FCOMP ST(1) + FNSTSW AX", "fnstsw"), 7, 7, False),
    ("FCOMP m64 + FNSTSW AX (pair)", line("FCOMP m64 + FNSTSW AX + SAHF", "fcomp") +
     line("FCOMP m64 + FNSTSW AX + SAHF", "fnstsw"), 7, 7, False),
    ("FNSTSW AX (alone)", homog("FNSTSW AX"), 3, 3, False),
    ("FTST", homog("FTST"), 4, 4, True),
    ("FCHS", homog("FCHS"), 6, 6, True),
    ("FABS", homog("FABS"), 3, 3, False),
    ("FLD1", line("FLD1 / FSTP ST(0)", "fld1"), 4, 4, False),
    ("FLDZ", line("FLDZ / FSTP ST(0)", "fldz"), 4, 4, False),
    ("FSQRT", x87("FSQRT", "1fa"), 83, 87, False),
    ("FRNDINT", x87("FRNDINT", "1fc"), 21, 30, True),
    ("FSCALE", homog("FSCALE"), 30, 32, True),
    ("FPREM (1000.3 rem 3.7)", x87("FPREM", "1f8"), 70, 138, False),
    ("FSIN (0.7)", x87("FSIN", "1fe"), 257, 354, False),
    ("FCOS (0.7)", x87("FCOS", "1ff"), 257, 354, False),
    ("FPTAN (0.7)", x87("FPTAN", "1f2"), 200, 273, True),
    ("FPATAN (0.6/1.3)", x87("FPATAN", "1f3"), 218, 303, False),
    ("F2XM1 (0.4)", x87("F2XM1", "1f0"), 140, 279, True),
    ("FYL2X (1.3*log2 2.7)", x87("FYL2X", "1f1"), 196, 329, True),
    ("FYL2XP1 (1.3*log2 1.2)", x87("FYL2XP1", "1f9"), 171, 326, True),
    ("FLDCW", homog("FLDCW"), 4, 4, False),
    ("FNSTCW", homog("FNSTCW"), 3, 3, False),
    ("FWAIT (x87 idle)", homog("FWAIT (x87 idle)"), 1, 3, True),
    ("FADD ST,ST(i) + FWAIT (pair)", body("FADD + FWAIT", 4), 9, 23, True),
    ("FMUL ST,ST(i) + FWAIT (pair)", body("FMUL + FWAIT", 4), 17, 19, True),
    ("FDIV ST,ST(i) + FWAIT (pair)", body("FDIV + FWAIT", 4), 74, 76, True),
    ("FWAIT FADD + FWAIT FSUB ST,ST(1)", body("FWAIT-prefixed FADD/FSUB ST,ST(1)", 2),
     18, 46, True),
    ("FWAIT FMUL + FWAIT FMUL ST,ST(i)", body("FWAIT-prefixed FMUL ST,ST(i)", 2), 34, 38, True),
    ("FWAIT-prefixed FLD/FMUL/FADD/FSTP m64", body("FWAIT-prefixed expr m64", 2), 37, 57, True),
]


def main_table():
    print("| instruction | z486 cycles | Intel486DX | ratio z486/486 |")
    print("|---|---:|---:|---:|")
    for name, z, lo, hi, unsure in ROWS:
        i486 = f"{lo}" if lo == hi else f"{lo}-{hi}"
        if unsure:
            i486 += " (?)"
        mid = (lo + hi) / 2
        ratio = f"{z / mid:.1f}x" if lo == hi else f"{z / mid:.1f}x ({z / hi:.1f}-{z / lo:.1f})"
        print(f"| {name} | {z:.1f} | {i486} | {ratio} |")


BREAKDOWN = [
    # (label, test, fops): the per-instruction average over these FOPs
    ("FADD ST,ST(i)", "FADD ST,ST(1) dependent", ["0c1"]),
    ("FMUL ST,ST(i)", "FMUL ST,ST(i) dependent", ["0c9", "0ca"]),
    ("FDIV ST,ST(i)", "FDIV ST,ST(i) dependent", ["0f1", "0f2"]),
    ("FADD m32", "FADD m32", ["005"]),
    ("FMUL m32", "FMUL m32", ["00d"]),
    ("FDIV m32", "FDIV m32", ["035"]),
    ("FADD m64", "FADD m64", ["405"]),
    ("FMUL m64", "FMUL m64", ["40d"]),
    ("FDIV m64", "FDIV m64", ["435"]),
    ("FLD m32 (FLD x4/FSTP x4 loop)", "FLD m32 / FSTP m32", ["105"]),
    ("FSTP m32 (same loop)", "FLD m32 / FSTP m32", ["11d"]),
    ("FST m32 (homogeneous)", "FST m32", ["115"]),
    ("FLD m64", "FLD m64 / FSTP m64", ["505"]),
    ("FSTP m64", "FLD m64 / FSTP m64", ["51d"]),
    ("FILD m16", "FILD m16 / FSTP ST(0)", ["705"]),
    ("FILD m32", "FILD m32 / FSTP ST(0)", ["305"]),
    ("FISTP m16", "FILD m16 / FISTP m16", ["71d"]),
    ("FISTP m32", "FILD m32 / FISTP m32", ["31d"]),
    ("FISTP m64", "FILD m64 / FISTP m64", ["73d"]),
]

CPU_KEYS = ["issue", "ucode_pre", "stall_x87_direct", "stall_x87_port", "stall_mem", "ucode_post"]


def avg_x87(test, fops):
    tot, cnt = 0.0, 0
    ph = {}
    for f in fops:
        x = res[test]["x87_by_fop"][f]
        tot += x["total"] * x["count"]
        cnt += x["count"]
        for k, v in x["phases"].items():
            ph[k] = ph.get(k, 0.0) + v * x["count"]
    return tot / cnt, {k: v / cnt for k, v in ph.items()}


def avg_cpu(test, fops):
    s, n = 0.0, 0
    cpu = {}
    for g in res[test]["groups"]:
        if g["fop"] is not None and f"{g['fop']:03x}" in fops:
            s += g["mean"] * g["lines"]
            n += g["lines"]
            for k, v in g["cpu"].items():
                cpu[k] = cpu.get(k, 0.0) + v * g["lines"]
    return s / n, {k: v / n for k, v in cpu.items()}


def breakdown_table():
    print("x87-side: every cycle of the steady loop is charged to the x87 command that owns it;")
    print("cpu_gap = x87 idle waiting for the CPU to deliver this command.\n")
    print("| instruction | total | cpu_gap | bridge/transfer | x87_control seq | executor | retire | detail |")
    print("|---|---:|---:|---:|---:|---:|---:|---|")
    for label, test, fops in BREAKDOWN:
        tot, ph = avg_x87(test, fops)
        g = xt.group_phases(ph)
        print(f"| {label} | {tot:.1f} | {g['cpu_gap']:.1f} | {g['bridge_xfer']:.1f} | "
              f"{g['ctl_seq']:.1f} | {g['exec']:.1f} | {g['retire']:.1f} | {xt.fmt(ph)} |")
    print("\nCPU-side: issue-to-next-issue window of the instruction, by cycle state")
    print("(ucode_pre = microcode before the x87 sees the command; ucode_post = after,"
          " incl. operand loops and BUSY# polling).\n")
    print("| instruction | issue->issue | " + " | ".join(CPU_KEYS) + " |")
    print("|---|---:|" + "---:|" * len(CPU_KEYS))
    for label, test, fops in BREAKDOWN:
        m, cpu = avg_cpu(test, fops)
        print(f"| {label} | {m:.1f} | " + " | ".join(f"{cpu.get(k, 0):.1f}" for k in CPU_KEYS) + " |")


def overlap_table():
    print("| loop body | cycles/iteration | x87 busy cycles/iteration |")
    print("|---|---:|---:|")
    for r in res.values():
        if "ADD" in r["name"] and ("+" in r["name"]) and r["name"].split()[0] in ("FDIV", "FSQRT", "FMUL"):
            busy = sum(v for k, v in r["x87"].items() if k != "idle")
            print(f"| {r['name']} | {r['iter_mean']:.1f} | {busy:.1f} |")


if __name__ == "__main__":
    main_table()
    print()
    breakdown_table()
    print()
    overlap_table()
