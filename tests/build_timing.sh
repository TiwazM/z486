#!/bin/sh
# Build the x87-enabled protected-mode testbench (no VCD trace) with the
# x87 timing monitor (x87_timing_mon.svh, +define+X87_TIMING_MON). Run from
# the tests directory, then ./x87_timing.py run.
set -e
cd "$(dirname "$0")"
cp -f ../*.hex .
X87=../x87
verilator --timing -Wall -Wno-fatal -Wno-UNOPTFLAT -Wno-SYNCASYNCNET -cc --exe -O3 --build -j 0 \
  -I.. -I$X87 -Ix87_reference -CFLAGS "-std=c++14 -O3" \
  --Mdir obj_dir_timing -GENABLE_X87=1 +define+X87_TIMING_MON --top-module tb_protected_mode \
  tb_protected_mode.sv \
  ../z486_pkg.sv $X87/x87_pkg.sv $X87/x87_ucode_pkg.sv ../z486.sv \
  ../cpu_throttle.sv ../interrupt_controller.sv \
  ../ucode_rom.sv ../microsequencer.sv ../event_control.sv ../hardwired_control.sv ../data_access.sv \
  ../data_unit.sv ../address_unit.sv ../shifter.sv ../mul_div.sv ../decoder.sv ../alu.sv ../prefetch.sv \
  ../paging_unit.sv ../paging_tlb.sv ../paging_walker.sv \
  ../memory.sv ../bus_unit.sv ../cache_unit.sv ../l1_cache.sv ../l1_icache.sv \
  $X87/x87_bridge.sv $X87/x87_ucode_rom.sv $X87/x87_sequencer.sv $X87/x87_executor.sv \
  $X87/x87_transfer_fifo.sv $X87/x87_stack_mem.sv $X87/x87_command_rom.sv $X87/x87_control.sv $X87/x87_unit.sv \
  ../dsp_mul.sv ../protection.sv ../segmentation_unit.sv \
  sim_main_timing.cpp
