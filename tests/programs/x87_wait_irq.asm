; Interrupts while a WAIT waits for the x87 (16-bit code).
;
; A WAIT holds in place while the x87 runs plain arithmetic, which ends
; within an operation or two: it takes no interrupt there. Behind a long
; operation it runs the original routine, which polls BUSY# and takes
; interrupts, restarting the WAIT.
;
; - Short: FDIV runs; INTR arrives while the WAIT behind it holds. The WAIT
;   completes in place and the interrupt comes right after it (the handler
;   sees the next instruction's address); the result is the divide's.
; - Short, the overlap: FDIV runs, FADD is held, FMUL waits in the bridge,
;   and the WAIT behind them holds through all three; INTR comes after it.
; - Long: FSIN runs some 340 cycles; the WAIT behind it goes to the routine,
;   which takes INTR (the handler sees the WAIT's address) and, restarted,
;   waits FSIN out. The result is FSIN's without the interrupt.

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

%macro INTR_IN 1                    ; INTR %1 cycles from now
    mov dx, SIGNAL_CYCLES_PORT
    mov ax, %1
    out dx, ax
    mov dx, SIGNAL_PORT
    mov al, 1
    out dx, al
%endmacro

; The m64 results at res and ref (%1 bytes) must be equal.
%macro SAME 1
    mov si, res
    mov di, ref
    mov cx, %1
    repe cmpsb
    jne fail
%endmacro

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

    ; ---- short: INTR while the WAIT holds behind FDIV ------------------
    mov word [step], 1
    fninit
    fld qword [y]
    fld qword [x]
    fdiv st0, st1
    fstp qword [ref]
    fninit
    fld qword [y]
    fld qword [x]
    sti
    INTR_IN 30                      ; FDIV runs some 65 cycles
    fdiv st0, st1
    fwait
after_short:
    cli
    cmp word [intr_count], 1
    jne fail
    cmp word [intr_ip], after_short
    jne fail
    fstp qword [res]
    SAME 8

    ; ---- short, the overlap: FDIV, FADD held, FMUL in the bridge ---------
    mov word [step], 2
    fninit
    fld qword [y]
    fld qword [x]
    fdiv st0, st1
    fadd st0, st0
    fmul st0, st0
    fstp qword [ref]
    fstp qword [ref + 8]
    fninit
    fld qword [y]
    fld qword [x]
    sti
    INTR_IN 40
    fdiv st0, st1
    fadd st0, st0
    fmul st0, st0
    fwait
after_overlap:
    cli
    cmp word [intr_count], 2
    jne fail
    cmp word [intr_ip], after_overlap
    jne fail
    fstp qword [res]
    fstp qword [res + 8]
    SAME 16

    ; ---- long: INTR while the WAIT waits behind FSIN ---------------------
    mov word [step], 3
    fninit
    fld qword [x]
    fsin
    fstp qword [ref]
    fninit
    fld qword [x]
    sti
    INTR_IN 200
    fsin
fwait_long:
    fwait
    cli
    cmp word [intr_count], 3
    jne fail
    cmp word [intr_ip], fwait_long
    jne fail
    fstp qword [res]
    SAME 8

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
ref:        dq 0, 0
res:        dq 0, 0
step:       dw 0
intr_count: dw 0
intr_ip:    dw 0
