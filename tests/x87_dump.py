#!/usr/bin/env python3
"""Dump part of a measurement window cycle by cycle: x87_dump.py <test id> [start_iter] [n_cycles]"""
import json, sys
sys.path.insert(0, __import__('os').path.dirname(__file__))
import x87_timing as xt
tid = int(sys.argv[1]); it0 = int(sys.argv[2]) if len(sys.argv) > 2 else 3
ncyc = int(sys.argv[3]) if len(sys.argv) > 3 else 200
syms = json.loads((xt.OUT / "symbols.json").read_text())
rows = xt.parse_log(xt.OUT / "x87_cycles.log")[tid]
t = xt.TESTS[tid - 1]
lab = {syms[f"t{tid}_b{j}"]: f"b{j}:{s}" for j, s in enumerate(t["body"])}
lab[syms[f"t{tid}_dec"]] = "dec"; lab[syms[f"t{tid}_dec"] + 1] = "jnz"
loop = syms[f"t{tid}_loop"]
starts = [i for i, r in enumerate(rows) if xt.bit(r[4], xt.C_ISSUE) and r[1] == loop]
i0 = starts[min(it0, len(starts) - 1)]
cpuf = ["park","d2rdy","pbv","msrv","FC","wr","x87acc","x87req","STALL","stx87d","stwio","stmem","ucx","uca","first","ISSUE"]
unitf = {15:"dact",14:"diss",13:"dval",12:"drdy",11:"dreg",10:"dcmdp",9:"dst",8:"posted",7:"bracc",6:"brrdc",5:"brwr",4:"brfc"}
ctlf = {31:"cmdv",30:"cmdr",29:"corer",28:"busyn",27:"pereq",26:"cmdpend",25:"dpend",24:"expend",23:"EXBUSY",22:"exdone",21:"push",20:"resw",19:"mmath",18:"store",17:"pop",16:"stw",15:"win",14:"rdreq",13:"rdresp",12:"statrd",11:"ccpulse",10:"prhold",9:"poly",8:"fprem",7:"fscale",6:"logexp",5:"fptan",4:"bcd"}
for r in rows[i0:i0 + ncyc]:
    c, eip, op, modrm, cpu, unit, ctl, kinds, uca, exop, exua, lfop, bes = r
    iss = lab.get(eip, f"{eip:x}") if xt.bit(cpu, 15) else ""
    cf = " ".join(n for b, n in enumerate(cpuf) if xt.bit(cpu, b) and n not in ("d2rdy","pbv","uca"))
    uf = " ".join(n for b, n in unitf.items() if xt.bit(unit, b))
    kf = " ".join(n for b, n in ctlf.items() if xt.bit(ctl, b) and n not in ("busyn","corer"))
    print(f"{c:8d} {iss:28s} uc={uca:03x} br={unit&3} st={(unit>>2)&3} rx={kinds>>4} tx={(kinds>>2)&3} busy={'N' if xt.bit(ctl,28) else 'B'} | {cf:30s} | {uf:22s} | {kf} {xt.x87_phase(cpu,unit,ctl,kinds)} fop={lfop:03x}" + (f" wbe={bes>>4:x}" if xt.bit(ctl,15) else "") + (f" rbe={bes&15:x}" if xt.bit(ctl,14) else ""))
