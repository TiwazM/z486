; Direct x87 stores: every form, bit-exact, from 16-bit code.
;
; FST/FSTP m32 and FIST/FISTP m32 (x87-st-m32), FST/FSTP m64 and FISTP m64
; (x87-st-m64), FSTP m80 and FBSTP (x87-st-m80) and FIST/FISTP m16
; (x87-st-m16) check their destination and write the x87's result with the
; overlays' WR words. In 16-bit code the overlays first set the operand size
; (dword parts, word m16 and m80 tail). Each value is loaded from its own
; memory format and stored back, so the stored bytes must equal the source
; (FILD m64 and FBLD keep 53 bits, so those values fit in 53). Every
; destination sits aligned and at every misaligned offset inside a field of
; A5 bytes; afterwards the field must hold the result and A5 everywhere else,
; so no part writes past the operand.
;
; FNSTSW AX, FNSTSW and FNSTCW m16 and FLDCW take their direct paths too: a
; control word loaded right before a store rounds it, FNSTCW reads it back.
;
; The last section checks the real-mode limit (FFFF) over each destination:
; an m32 whose last bytes pass it, an m64 whose second part does, an m80
; whose tail does, an FBSTP whose second part does, an m16 and an FISTP m64:
; each is #GP(0) (vector 13) before anything is written, with the x87
; unchanged (FSTP has not popped); destinations that end exactly at FFFF are
; written. (A part that starts at 10000 wraps to offset 0 in real mode, on
; the original routine as well; such destinations are not tested.)

BITS 16
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

; Fill the field with A5, run "%1 %2 [field + 8 + %4]" and compare the field
; with A5 and the %3-byte result %5 at offset %4.
%macro CHECKST 5
    call fill_field
    %1 %2 [field + 8 + %4]
    mov si, %5
    mov cx, %3
    mov bx, %4
    call check_field
%endmacro

; TOP must be %1 and no exception flagged.
%macro EXPECT_TOP 1
    fnstsw ax
    and ax, 0x38ff
    cmp ax, (%1) << 11
    jne fail
%endmacro

start:
    cli
    cld
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x700
    push es
    xor ax, ax
    mov es, ax
    mov word [es:13 * 4], gp_fault
    mov ax, cs
    mov word [es:13 * 4 + 2], ax
    pop es
    fninit

    ; ---- m32: FST, FSTP, FIST, FISTP --------------------------------------
    fld dword [r32]
    CHECKST fst, dword, 4, 0, r32
    CHECKST fst, dword, 4, 1, r32
    CHECKST fst, dword, 4, 2, r32
    CHECKST fst, dword, 4, 3, r32
    EXPECT_TOP 7
    CHECKST fstp, dword, 4, 1, r32
    EXPECT_TOP 0
    fild dword [i32]
    CHECKST fist, dword, 4, 0, i32
    CHECKST fist, dword, 4, 3, i32
    CHECKST fistp, dword, 4, 2, i32
    EXPECT_TOP 0

    ; ---- m64: FST, FSTP, FISTP --------------------------------------------
    fld qword [r64]
    CHECKST fst, qword, 8, 0, r64
    CHECKST fst, qword, 8, 1, r64
    CHECKST fst, qword, 8, 2, r64
    CHECKST fst, qword, 8, 3, r64
    CHECKST fst, qword, 8, 5, r64
    CHECKST fst, qword, 8, 7, r64
    EXPECT_TOP 7
    CHECKST fstp, qword, 8, 6, r64
    EXPECT_TOP 0
    fild qword [i64a]
    CHECKST fistp, qword, 8, 0, i64a
    fild qword [i64a]
    CHECKST fistp, qword, 8, 3, i64a
    fild qword [i64b]
    CHECKST fistp, qword, 8, 5, i64b
    EXPECT_TOP 0

    ; ---- m80: FSTP, FBSTP --------------------------------------------------
%assign off 0
%rep 8
    fld tword [pi80]
    CHECKST fstp, tword, 10, off, pi80
%assign off off + 1
%endrep
    fbld tword [bcd]
    CHECKST fbstp, tword, 10, 0, bcd
    fbld tword [bcd]
    CHECKST fbstp, tword, 10, 3, bcd
    fbld tword [bcd]
    CHECKST fbstp, tword, 10, 6, bcd
    EXPECT_TOP 0

    ; ---- m16: FIST, FISTP ---------------------------------------------------
    fild word [i16]
    CHECKST fist, word, 2, 0, i16
    CHECKST fist, word, 2, 1, i16
    CHECKST fist, word, 2, 3, i16
    EXPECT_TOP 7
    CHECKST fistp, word, 2, 2, i16
    EXPECT_TOP 0

    ; ---- control and status words -------------------------------------------
    fld tword [pi80]
    fldcw [cw_up]
    fist word [field]               ; pi rounded up: 4
    cmp word [field], 4
    jne fail
    fnstcw [field]
    cmp word [field], 0x0b7f
    jne fail
    fldcw [cw_chop]
    fistp word [field]              ; chopped: 3
    cmp word [field], 3
    jne fail
    fnstsw [field]
    cmp word [field], 0x0020        ; TOP 0, PE
    jne fail
    fld1
    fchs
    fsqrt                           ; invalid (masked)
    fnstsw ax
    and ax, 0x38ff
    cmp ax, 0x3821
    jne fail
    fninit
    fnstcw [field]
    cmp word [field], 0x037f
    jne fail

    ; ---- the real-mode limit over the whole destination --------------------
    push ds
    push es
    mov ax, 0x2000
    mov ds, ax
    mov es, ax
    mov di, 0xfff0
    mov cx, 16
    mov al, 0xa5
    rep stosb
    fld tword [cs:pi80]                      ; TOP 7 throughout
    mov word [cs:resume], gp1
    fst dword [0xfffe]                       ; bytes FFFE-10001
gp1:
    mov word [cs:resume], gp2
    fstp qword [0xfffa]                      ; second part FFFE-10001
gp2:
    mov word [cs:resume], gp3
    fstp tword [0xfff7]                      ; tail FFFF-10000
gp3:
    mov word [cs:resume], gp4
    fbstp tword [0xfff9]                     ; second part FFFD-10000
gp4:
    mov word [cs:resume], gp5
    fist word [0xffff]
gp5:
    mov word [cs:resume], gp6
    fistp qword [0xfffb]                     ; second part FFFF-10002
gp6:
    mov word [cs:resume], gp_unexpected
    mov di, 0xfff0                           ; nothing written
    mov cx, 16
    mov al, 0xa5
    repe scasb
    jne fail_pop
    fst qword [0xfff8]                       ; ends at FFFF
    mov si, 0xfff8
    mov di, r64_pi
    mov cx, 8
    call compare_cs
    fst dword [0xfff4]
    mov si, 0xfff4
    mov di, r32_pi
    mov cx, 4
    call compare_cs
    fist word [0xfffe]
    cmp word [0xfffe], 3
    jne fail_pop
    fstp tword [0xfff6]
    mov si, 0xfff6
    mov di, pi80
    mov cx, 10
    call compare_cs
    pop es
    pop ds
    cmp byte [gp_count], 6
    jne fail
    cmp byte [gp_bad_top], 0
    jne fail
    fnstsw ax
    and ax, 0x38ff
    cmp ax, 0x0020                           ; popped once: TOP 0; only PE (pi)
    jne fail

pass:
    mov al, STATUS_PASS
    jmp report

fail_pop:
    pop es
    pop ds
fail:
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fill_field:
    mov di, field
    mov al, 0xa5
    mov cx, 32
    rep stosb
    ret

; The field must be A5 except for the CX bytes at SI, at offset 8 + BX.
check_field:
    push cx
    mov di, expect
    mov al, 0xa5
    mov cx, 32
    rep stosb
    pop cx
    lea di, [bx + expect + 8]
    rep movsb
    mov si, field
    mov di, expect
    mov cx, 32
    repe cmpsb
    jne fail
    ret

; Compare CX bytes at DS:SI with CS:DI.
compare_cs:
    mov al, [si]
    cmp al, [cs:di]
    jne fail_pop
    inc si
    inc di
    loop compare_cs
    ret

; #GP(0): the faulting store has changed no x87 state (ST0 = pi, TOP 7).
gp_fault:
    inc byte [cs:gp_count]
    push ax
    fnstsw ax
    and ax, 0x38ff
    cmp ax, 0x3800
    je .top_ok
    inc byte [cs:gp_bad_top]
.top_ok:
    pop ax
    add sp, 2                                ; discard IP
    push word [cs:resume]
    iret

gp_unexpected:
    jmp fail_pop

gp_count:   db 0
gp_bad_top: db 0
resume:     dw 0

align 16
field:      times 32 db 0
expect:     times 32 db 0

r32:        dd 0xc0912345               ; -4.5357...
i32:        dd -123456789
i16:        dw -12345
align 4
r64:        dq 0x3ff3c0ca428c59fb       ; 1.2345678901234567
i64a:       dq 0x7123456789abc800       ; 53 significant bits
i64b:       dq -0x0013456789abcdef
pi80:       dq 0xc90fdaa22168c235
            dw 0x4000
bcd:        db 0x78, 0x56, 0x34, 0x12, 0x10, 0x32, 0x54, 0x76, 0x00, 0x80
cw_up:      dw 0x0b7f
cw_chop:    dw 0x0f7f
r64_pi:     dq 0x400921fb54442d18       ; pi rounded to m64
r32_pi:     dd 0x40490fdb               ; ...and to m32
