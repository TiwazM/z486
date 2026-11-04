; x87 commands taken behind a running operation (x87_control holds them until
; it retires, then dispatches them in the cycle its result is written, taking
; a row they read from that write) and register commands that wait for the
; bridge instead of falling back.
;
; Every sequence runs twice: with every exception masked, where commands are
; taken behind running arithmetic, and with only the denormal exception
; unmasked (no operand here is denormal, so the results are the same), where
; nothing is: no command is held, no register command waits for the bridge
; and loads go through the original routine. FSAVE then compares status
; word, tag word and registers, and the words each sequence stores.
;
; - Dependent chains through ST0 and ST(i), destination ST0 and ST(i),
;   reversed operands, both operands the destination (FMUL ST,ST(0)),
;   FSQRT and FRNDINT ahead of and behind arithmetic.
; - FSCALE ahead of and behind arithmetic, FSQRT and FRNDINT, with positive
;   and negative scale factors, a held FXCH and FLD ST(i), and an overflow
;   (masked: OE, PE).
; - Pops ahead of the held command (FMULP, FADDP, FCOMP, FCOMPP, FSUBRP):
;   it sees TOP after them. FADDP ST(0),ST, whose result goes to the row it
;   pops, then FLD ST(7) of that row.
; - FXCH, FLD ST(i), FST/FSTP ST(i), FCHS, FABS, constants, FFREE,
;   FDECSTP/FINCSTP behind arithmetic.
; - Memory arithmetic (m32, m64, m16/m32 integer) and loads of every format
;   behind arithmetic and behind each other.
; - Compares behind arithmetic with FNSTSW AX/m16 right after (0-3 NOPs,
;   FWAIT), FTST, FXAM, FUCOM.
; - Masked exceptions raised by the earlier of two (ZE, IE, PE, a stack
;   underflow and overflow of the held command).
; - FLDCW (precision, rounding) taken behind a running divide: the divide
;   keeps the control word it started with.
;
; Then, with NE set, unmasked exceptions (not differential):
; - FLDCW unmasking ZE, FDIV by zero right after it (posted while the
;   masks are still set), and FLD ST(2): #MF at the FLD, which has not run
;   (it pushes once, after the handler); the divide left ST(1) alone.
; - FMUL running, FLDCW unmasking ZE taken behind it, FDIV by zero posted
;   behind that while the masks are still set, FADD: #MF at the FADD.
;
; The JSON pins how many commands were held and how many register commands
; waited for the bridge.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

; Run sequence %1 (id %2) masked, then with DM unmasked; compare.
%macro RUN 2
    mov word [seq_id], %2
    mov word [cw_low], 0x7f
    call set_cw
    mov bx, buf_a
    fninit
    fldcw [cw_def]
    %1
    fsave [img_a]
    mov word [cw_low], 0x7d
    call set_cw
    mov bx, buf_b
    fninit
    fldcw [cw_def]
    %1
    fsave [img_b]
    call compare_runs
%endmacro

; Status word (AX after FNSTSW) and %1 must be %2 (in the masked run only:
; called right after a sequence's first run, it checks absolute values).
%macro CHECK_AX 2
    and ax, %1
    cmp ax, %2
    jne fail
%endmacro

%macro SEQ_CHAIN 0
    fld qword [va]
    fld qword [vc]
    fadd st0, st1
    fmul st0, st0
    fsub st0, st1
    fdivr st0, st1
    fmul st0, st1
    fadd st1, st0
    fmul st0, st1
    fsubr st1, st0
    fdiv st1, st0
    fxch st1
    fsqrt
    fmul st0, st1
    frndint
    fadd st0, st1
    fmul st1, st0
    fdivr st1, st0
    fsub st0, st1
    fst qword [bx]
    fmul st0, st0
    fmul st0, st0
    fmul st0, st1
    fstp qword [bx + 8]
%endmacro

%macro SEQ_SCALE 0
    fld qword [vneg]                ; -2
    fld qword [vthree]              ; 3, -2
    fld qword [va]                  ; 1.5, 3, -2
    fscale                          ; 12
    fmul st0, st0                   ; 144, behind FSCALE
    fscale                          ; 1152, behind the multiply
    fsqrt                           ; 33.94...
    fscale                          ; 271.5...
    frndint                         ; 272, behind FSCALE (PE)
    fxch st2                        ; -2, 3, 272
    fld st2                         ; 272, -2, 3, 272
    fscale                          ; 68
    fdivr st0, st3                  ; 4
    fscale                          ; 1
    fst qword [bx]
    fld qword [vbig]                ; 20000, 1, -2, 3, 272
    fxch st1
    fscale                          ; 1 * 2^20000: inf (OE, PE)
    fchs                            ; behind FSCALE
    fstp qword [bx + 8]
%endmacro

%macro SEQ_POPS 0
    fld qword [va]
    fld qword [vb]
    fld qword [vc]
    fld qword [vd]
    fmulp st1, st0
    fmul st0, st0
    faddp st2, st0
    fld st1
    fcomp st1
    fdivp st1, st0
    fld qword [ve]
    fcompp
    fld qword [va]
    fld qword [vb]
    fsubrp st1, st0
    fchs
    fabs
    fldpi
    fmulp st1, st0
    fld1
    faddp st1, st0
    fldz
    fcomp st1
    fmul st0, st0
    fstp qword [bx]
    fld qword [va]
    fld qword [vb]
    db 0xde, 0xc0                   ; faddp st(0), st: the result's row popped
    fld st7
    fmul st0, st1
    fld qword [vd]
    fmul st0, st0
    fcompp
    fld qword [va]
    fld qword [vb]
    fsqrt
    fmulp st1, st0
    fld st0
    fstp qword [bx + 8]
%endmacro

%macro SEQ_STACK 0
    fld qword [va]
    fld qword [vb]
    fmul st0, st1
    fxch st1
    fmul st0, st1
    fld st1
    fadd st0, st2
    fst st3
    fstp st2
    fchs
    fmul st0, st1
    fabs
    fld1
    fdiv st0, st1
    fldl2e
    fmul st0, st2
    fldz
    fxch st2
    fsub st0, st1
    ffree st2
    fmul st0, st3
    fdecstp
    fincstp
    fmul st0, st1
    fld st0
    fxch st2
    fdiv st0, st2
    fxch st1
    fxch st4
    fmul st0, st0
    fst st3
    fld st5
    fmul st0, st0
    fstp st1
    fst qword [bx]
%endmacro

%macro SEQ_MEMORY 0
    fld qword [va]
    fmul qword [vb]
    fadd dword [f1]
    fmul st0, st0
    fdiv qword [vc]
    fsub dword [f2]
    fcom dword [f1]
    fiadd word [i16]
    fimul dword [i32]
    fisub word [i16]
    fidivr dword [i32]
    fsubr qword [vd]
    fdivr dword [f1]
    fmul st0, st0
    fld dword [f2]
    fmul st0, st1
    fld qword [vd]
    fadd st0, st1
    fld tword [t80]
    fmul st0, st1
    fild word [i16]
    fmul st0, st1
    fild dword [i32]
    fild qword [i64]
    fadd st0, st2
    fcomp qword [va]
    fmul dword [f2]
    fdiv dword [f1]
    fstp qword [bx]
    fmul st0, st1
    fst dword [bx + 8]
%endmacro

%macro SEQ_STATUS 0
    fld qword [vd]
    fld qword [va]
    fld qword [vb]
    fmul st0, st1                   ; 4.125
    fcom st1                        ; > 1.5
    fnstsw ax
    mov [bx], ax
    fmul st0, st1                   ; 6.1875
    fcomp qword [vd]                ; < 7: C0; popped
    nop
    fnstsw ax
    mov [bx + 2], ax
    fadd st0, st0                   ; 3
    ftst                            ; > 0
    nop
    nop
    fnstsw ax
    mov [bx + 4], ax
    fmul st0, st0                   ; 9
    fxam                            ; normal, positive: C2
    nop
    nop
    nop
    fnstsw ax
    mov [bx + 6], ax
    fdiv st0, st1                   ; 9/7
    fucom st1                       ; < 7: C0
    fnstsw [bx + 8]
    fmul st0, st1                   ; 9
    fcom dword [f1]                 ; > 1.25
    fwait
    fnstsw ax
    mov [bx + 10], ax
    fsqrt                           ; 3
    fcom st1                        ; < 7: C0
    fnstsw ax
    mov [bx + 12], ax
    fld st0
    fmul st0, st0                   ; 9
    fcompp                          ; 9 > 3; both popped
    fnstsw [bx + 14]
%endmacro

%macro SEQ_MASKED 0
    fld qword [vz]
    fld qword [va]
    fdiv st0, st1                   ; ZE: +inf
    fmul st0, st1                   ; inf * 0: IE, indefinite
    fnstsw [bx]
    fld qword [vthree]
    fld1
    fdiv st0, st1                   ; PE
    fcom st1
    fnstsw [bx + 2]
    fld qword [vneg]
    fsqrt                           ; IE
    fadd st0, st1
    fnstsw [bx + 4]
    fninit
    fldcw [cw_def]
    fld qword [va]
    fmul st0, st0
    fadd st0, st5                   ; ST(5) empty: IE, SF, indefinite
    fnstsw [bx + 6]
    fninit
    fldcw [cw_def]
    fld qword [va]
    fld st0
    fld st0
    fld st0
    fld st0
    fld st0
    fld st0
    fld st0
    fmul st0, st0
    fld st0                         ; full: IE, SF, C1, indefinite
    fnstsw [bx + 8]
%endmacro

%macro SEQ_CONTROL 0
    fld qword [vthree]
    fld qword [va]
    fdiv st0, st1
    fldcw [cw_pc24]
    fdiv st0, st1
    fldcw [cw_chop]
    fmul st0, st1
    fldcw [cw_pc53]
    fdiv st0, st1
    fldcw [cw_def]
    fadd st0, st1
    fst qword [bx]
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
    mov word [es:0x10 * 4], mf_handler     ; #MF (INT 16) in real mode
    mov [es:0x10 * 4 + 2], cs
    mov ax, cs
    mov es, ax

    RUN SEQ_CHAIN, 1
    RUN SEQ_POPS, 2
    RUN SEQ_STACK, 3
    RUN SEQ_MEMORY, 4
    RUN SEQ_STATUS, 5
    mov si, buf_a                   ; the condition codes, C1 clear
    mov di, status_cc
    mov cx, 8
.cc:
    lodsw
    and ax, 0x4700
    cmp ax, [di]
    jne fail
    add di, 2
    loop .cc
    RUN SEQ_MASKED, 6
    mov ax, [buf_a]
    CHECK_AX 0x00ff, 0x0005         ; ZE, IE
    mov ax, [buf_a + 6]
    CHECK_AX 0x02ff, 0x0041         ; IE, SF, C1 clear (underflow)
    mov ax, [buf_a + 8]
    CHECK_AX 0x02ff, 0x0241         ; IE, SF, C1 set (overflow)
    RUN SEQ_CONTROL, 7
    RUN SEQ_SCALE, 10

    ; ---- unmasked exceptions (NE: #MF) --------------------------------------
    mov eax, cr0
    or eax, 0x20
    mov cr0, eax
    mov word [seq_id], 8
    fninit
    fld qword [vthree]
    fld1
    fldz                            ; 0, 1, 3
    fldcw [cw_unmask_ze]
    fdiv st1, st0                   ; 1/0: ZE unmasked, ST(1) kept
mf_at_1:
    fld st2                         ; #MF here, then it runs (once)
    cmp word [mf_count], 1
    jne fail
    cmp word [mf_ip], mf_at_1
    jne fail
    mov ax, [mf_sw]
    CHECK_AX 0x38ff, 0x2884         ; TOP 5, ES, ZE
    fnstsw ax
    CHECK_AX 0x3800, 0x2000         ; TOP 4: one push
    fstp qword [res64]              ; 3
    cmp dword [res64 + 4], 0x40080000
    jne fail
    fstp qword [res64]              ; 0
    cmp dword [res64 + 4], 0
    jne fail
    fstp qword [res64]              ; 1
    cmp dword [res64 + 4], 0x3ff00000
    jne fail

    mov word [seq_id], 9
    fninit
    fldz
    fld1                            ; 1, 0
    fmul st0, st0                   ; runs while...
    fldcw [cw_unmask_ze]            ; ...this is taken behind it
    fdiv st0, st1                   ; posted behind both: 1/0, ZE unmasked
mf_at_2:
    fadd st0, st0                   ; #MF here, then 1 + 1
    cmp word [mf_count], 2
    jne fail
    cmp word [mf_ip], mf_at_2
    jne fail
    mov ax, [mf_sw]
    CHECK_AX 0x38ff, 0x3084
    fstp qword [res64]
    cmp dword [res64 + 4], 0x40000000
    jne fail
    fninit

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    mov dx, DATA_PORT
    mov ax, [seq_id]
    out dx, ax
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

mf_handler:                         ; #MF: record the status and return IP
    push bp
    mov bp, sp
    push ax
    inc word [mf_count]
    fnstsw ax
    mov [mf_sw], ax
    mov ax, [bp + 2]
    mov [mf_ip], ax
    fnclex
    pop ax
    pop bp
    iret

; cw_def, cw_pc24, cw_pc53, cw_chop: with the masks in cw_low.
set_cw:
    mov ax, [cw_low]
    or ax, 0x0300
    mov [cw_def], ax
    and ax, 0x00ff
    mov [cw_pc24], ax
    or ax, 0x0200
    mov [cw_pc53], ax
    or ax, 0x0f00
    mov [cw_chop], ax
    ret

; The two runs left the same status word, tag word, registers and words.
compare_runs:
    mov si, img_a + 2
    mov di, img_b + 2
    mov cx, 4
    repe cmpsb
    jne fail
    mov si, img_a + 14
    mov di, img_b + 14
    mov cx, 80
    repe cmpsb
    jne fail
    mov si, buf_a
    mov di, buf_b
    mov cx, 16
    repe cmpsb
    jne fail
    ret

align 8
va:      dq 1.5
vb:      dq 2.75
vc:      dq 0.3
vd:      dq 7.0
ve:      dq -3.25
vz:      dq 0.0
vthree:  dq 3.0
vbig:    dq 20000.0
vneg:    dq -2.0
i64:     dq -123456789
t80:     dq 0xc90fdaa22168c235
         dw 0x4000
f1:      dd 1.25
f2:      dd 0.1
i32:     dd -7
i16:     dw 5
cw_unmask_ze: dw 0x037b
status_cc: dw 0x0000, 0x0100, 0x0000, 0x0400, 0x0100, 0x0000, 0x0100, 0x0000
cw_low:  dw 0
cw_def:  dw 0
cw_pc24: dw 0
cw_pc53: dw 0
cw_chop: dw 0
seq_id:  dw 0
mf_count: dw 0
mf_sw:   dw 0
mf_ip:   dw 0
align 8
res64:   dq 0
buf_a:   times 16 db 0
buf_b:   times 16 db 0
img_a:   times 94 db 0
img_b:   times 94 db 0
