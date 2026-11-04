; A register x87 instruction behind a taken bridge waits for the bridge in
; its first word only when the wait is short: a plain arithmetic operation
; runs and at most one short command is held. The original routine, which
; it falls back to otherwise, waits for BUSY# and takes interrupts while it
; waits; the short wait does not (it ends within one arithmetic operation).
;
; - FSIN runs (long: no command is taken behind it), FADD waits in the
;   bridge, FMUL falls back to the routine. INTR while it waits is taken
;   before FMUL (the handler sees FMUL's address as the return address),
;   and FMUL then runs once: the result is that of the same sequence
;   without the interrupt.
; - FDIV runs, FADD is held behind it, FMUL waits in the bridge and FSUB
;   waits for the bridge (counted): INTR while FSUB waits is taken right
;   after FSUB, and the result is the reference's.
; - FNSTSW AX right behind waiting FMUL reads the finished status.

BITS 16
org 0

STATUS_PORT        equ 0xE0
DATA_PORT          equ 0xE4
SIGNAL_PORT        equ 0xE8
SIGNAL_CYCLES_PORT equ 0xEC
SIGNAL_INSTR_PORT  equ 0xF0
SIGNAL_VECTOR_PORT equ 0xF4
STATUS_PASS        equ 0x01
STATUS_FAIL        equ 0xFF
INTR_VECTOR        equ 0x20

start:
    cli
    cld
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0xf000
    xor ax, ax
    mov es, ax
    mov word [es:INTR_VECTOR * 4], isr
    mov [es:INTR_VECTOR * 4 + 2], cs
    mov ax, cs
    mov es, ax

    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al
    mov dx, SIGNAL_INSTR_PORT
    xor ax, ax                      ; cycle-delay mode
    out dx, ax
    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 200                     ; FSIN runs some 550 cycles
    out dx, ax

    ; ---- reference: no interrupt ----------------------------------------
    mov word [step], 1
    fninit
    fld qword [x]
    fsin
    fadd st0, st0
    fmul st0, st0
    fstp qword [ref_sin]

    ; ---- INTR while FMUL waits for the bridge ---------------------------
    mov word [step], 2
    fninit
    fld qword [x]
    sti
    mov dx, SIGNAL_PORT
    mov al, 1
    out dx, al                      ; INTR 200 cycles from now
    fsin
    fadd st0, st0
fmul_waits:
    fmul st0, st0
    cli
    fstp qword [res]
    cmp word [intr_count], 1
    jne fail
    cmp word [intr_ip], fmul_waits
    jne fail
    mov si, res
    mov di, ref_sin
    mov cx, 8
    repe cmpsb
    jne fail

    ; ---- a short wait: FDIV runs, FADD held, FMUL in the bridge ----------
    mov word [step], 3
    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 30                      ; FDIV runs some 65 cycles
    out dx, ax
    fninit
    fld qword [x]
    fld qword [y]
    fdiv st0, st1
    fadd st0, st0
    fmul st0, st0
    fsub st0, st1
    fstp qword [ref]
    fstp qword [ref + 8]
    mov word [step], 4
    fninit
    fld qword [x]
    fld qword [y]
    sti
    mov dx, SIGNAL_PORT
    mov al, 1
    out dx, al                      ; INTR 30 cycles from now
    fdiv st0, st1
    fadd st0, st0
    fmul st0, st0
    fsub st0, st1                   ; waits for the bridge
after_wait:
    cli
    fstp qword [res]
    fstp qword [res + 8]
    cmp word [intr_count], 2
    jne fail
    cmp word [intr_ip], after_wait
    jne fail
    mov si, res
    mov di, ref
    mov cx, 16
    repe cmpsb
    jne fail

    ; ---- FNSTSW AX right behind the waiting FMUL -------------------------
    mov word [step], 5
    fninit
    fld qword [x]
    fsin
    fadd st0, st0
    fmul st0, st0
    fnstsw ax
    and ax, 0x38ff
    cmp ax, 0x3820                  ; TOP 7, PE (FSIN), nothing pending
    jne fail
    fstp qword [res]
    mov si, res
    mov di, ref_sin
    mov cx, 8
    repe cmpsb
    jne fail

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    mov dx, DATA_PORT
    mov ax, [step]
    out dx, ax
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

isr:                                ; record the return address
    push bp
    mov bp, sp
    inc word [intr_count]
    push ax
    mov ax, [bp + 2]
    mov [intr_ip], ax
    pop ax
    pop bp
    iret

align 8
x:          dq 0.7
y:          dq 2.5
ref_sin:    dq 0
ref:        dq 0, 0
res:        dq 0, 0
step:       dw 0
intr_count: dw 0
intr_ip:    dw 0
