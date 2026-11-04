; Direct x87 memory loads: every operand size, bit-exact, from 16-bit code.
;
; FLD m64/m80, FILD m16/m32/m64, FBLD and the m64 (DC) and integer m16/m32
; (DE/DA) arithmetic and compares take the direct paths: one-dword operands a
; paging-owned read, the others the RD words of the x87-m16/m64/m80-load
; overlays. In 16-bit code those overlays first set the operand size (dword
; parts, word m16 and m80 tail). Operands sit aligned and at every misaligned
; offset, so parts are split across dword boundaries. Every result is stored
; as an m80 and compared byte for byte with its exact value; results are
; chosen exact in 53 bits, which is what the x87's arithmetic and its FILD
; m64 and FBLD conversions keep (on the original routine as well).
;
; The last section checks the real-mode limit (FFFF) over each operand: a
; dword whose last bytes pass the limit, an m64 whose second part does, an
; m80 whose tail does and an m16, all #GP(0) (vector 13) with the x87 state
; unchanged; operands that end exactly at FFFF load.

BITS 16
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

%macro CHECK80 2                ; last instruction, expected m80
    %1
    fstp tword [result]
    mov di, %2
    call check80
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

    ; ---- loads, aligned and misaligned -----------------------------------
    CHECK80 {fld qword [a64_0]}, e_a
    CHECK80 {fld qword [a64_1]}, e_a
    CHECK80 {fld qword [a64_2]}, e_a
    CHECK80 {fld qword [a64_3]}, e_a
    CHECK80 {fld tword [pi80_0]}, pi80_0
    CHECK80 {fld tword [pi80_1]}, pi80_0
    CHECK80 {fld tword [pi80_2]}, pi80_0
    CHECK80 {fld tword [pi80_3]}, pi80_0
    CHECK80 {fld tword [pi80_6]}, pi80_0
    CHECK80 {fild word [i16_0]}, e_i16
    CHECK80 {fild word [i16_1]}, e_i16
    CHECK80 {fild word [i16_3]}, e_i16
    CHECK80 {fild dword [i32_0]}, e_i32
    CHECK80 {fild dword [i32_1]}, e_i32
    CHECK80 {fild dword [i32_3]}, e_i32
    CHECK80 {fild qword [i64a_0]}, e_i64a
    CHECK80 {fild qword [i64a_3]}, e_i64a
    CHECK80 {fild qword [i64b_2]}, e_i64b
    CHECK80 {fbld tword [bcd_0]}, e_bcd
    CHECK80 {fbld tword [bcd_1]}, e_bcd
    fnstsw ax
    test ax, 0x38ff                          ; TOP back to 0, no exceptions
    jnz fail

    ; ---- m64 arithmetic (DC); exact results, so all 53 operand bits count -
    fldz
    CHECK80 {fadd qword [a64_1]}, e_a
    fld1
    CHECK80 {fmul qword [a64_3]}, e_a
    fld1
    fadd st0, st0
    CHECK80 {fmul qword [a64_2]}, e_2a
    fldz
    CHECK80 {fsub qword [a64_0]}, e_na
    fldz
    CHECK80 {fsubr qword [a64_3]}, e_a
    fld qword [a64_0]
    CHECK80 {fdiv qword [a64_1]}, e_one
    fld1
    CHECK80 {fdivr qword [a64_2]}, e_a

    fld1
    fcom qword [a64_3]                       ; 1.0 < 1.2345...
    fnstsw ax
    and ax, 0x4500
    cmp ax, 0x0100
    jne fail
    fld qword [a64_0]
    fcomp qword [a64_3]                      ; equal, then pop
    fnstsw ax
    and ax, 0x7d00
    cmp ax, 0x7800                           ; C3, TOP 7 (1.0 left)
    jne fail
    fcomp qword [a64_1]                      ; 1.0 < 1.2345..., pop
    fnstsw ax
    and ax, 0x7d00
    cmp ax, 0x0100
    jne fail

    ; ---- integer m16 arithmetic (DE), ST0 = 16, operand -12345 ------------
    fild word [sixteen16]
    CHECK80 {fiadd word [i16_1]}, e_16p
    fild word [sixteen16]
    CHECK80 {fimul word [i16_3]}, e_16t
    fild word [sixteen16]
    CHECK80 {fisub word [i16_0]}, e_16m
    fild word [sixteen16]
    CHECK80 {fisubr word [i16_1]}, e_m16
    fild word [sixteen16]
    CHECK80 {fidivr word [i16_3]}, e_d16
    fild word [i16_0]
    fimul word [four16]
    CHECK80 {fidiv word [i16_1]}, e_four

    fild word [sixteen16]
    ficom word [i16_3]                       ; 16 > -12345
    fnstsw ax
    test ax, 0x4500
    jnz fail
    fild word [i16_1]
    ficomp word [i16_0]                      ; equal, then pop
    fnstsw ax
    and ax, 0x7d00
    cmp ax, 0x7800                           ; C3, TOP 7 (16 left)
    jne fail
    fstp st0

    ; ---- integer m32 arithmetic (DA), ST0 = 16, operand -123456789 --------
    fild word [sixteen16]
    CHECK80 {fiadd dword [i32_0]}, e_16pi
    fild word [sixteen16]
    CHECK80 {fimul dword [i32_3]}, e_16ti
    fild word [sixteen16]
    CHECK80 {fisub dword [i32_1]}, e_16mi
    fild word [sixteen16]
    CHECK80 {fisubr dword [i32_0]}, e_im16
    fild word [sixteen16]
    CHECK80 {fidivr dword [i32_3]}, e_id16
    fild dword [i32_1]
    fimul word [four16]
    CHECK80 {fidiv dword [i32_3]}, e_four

    fild word [sixteen16]
    ficomp dword [i32_1]                     ; 16 > -123456789, pop
    fnstsw ax
    test ax, 0x7d00
    jnz fail

    ; ---- the real-mode limit over the whole operand ----------------------
    push ds
    mov ax, 0x2000
    mov ds, ax
    fld1                                     ; TOP 7 throughout
    mov word [cs:resume], gp1
    fld dword [0xfffe]                       ; bytes FFFE-10001
gp1:
    mov word [cs:resume], gp2
    fld qword [0xfffa]                       ; second part FFFE-10001
gp2:
    mov word [cs:resume], gp3
    fld tword [0xfff7]                       ; tail FFFF-10000
gp3:
    mov word [cs:resume], gp4
    fild word [0xffff]
gp4:
    mov word [cs:resume], gp5
    fiadd word [0xffff]
gp5:
    mov word [cs:resume], gp6
    fadd qword [0xfffb]
gp6:
    mov word [cs:resume], gp_unexpected
    fld qword [0xfff8]                       ; ends at FFFF
    fld tword [0xfff6]
    fild word [0xfffe]
    fld dword [0xfffc]
    pop ds
    cmp byte [gp_count], 6
    jne fail
    cmp byte [gp_bad_top], 0
    jne fail
    fnstsw ax
    and ax, 0x3800
    cmp ax, 0x1800                           ; TOP 3: five values
    jne fail
    fninit

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; Compare the stored m80 with the expected value at DI.
check80:
    mov si, result
    mov cx, 10
    repe cmpsb
    jne fail
    ret

; #GP(0): the faulting load has changed no x87 state (ST0 = 1.0, TOP 7).
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
    jmp fail

gp_count:   db 0
gp_bad_top: db 0
resume:     dw 0

align 16
result:     times 10 db 0
align 16
sixteen16:  dw 16
four16:     dw 4

; 1.2345678901234567 and its exact m80
align 16
a64_0:      dq 0x3ff3c0ca428c59fb
align 16
            db 0xa5
a64_1:      dq 0x3ff3c0ca428c59fb
align 16
            db 0xa5, 0xa5
a64_2:      dq 0x3ff3c0ca428c59fb
align 16
            db 0xa5, 0xa5, 0xa5
a64_3:      dq 0x3ff3c0ca428c59fb
e_a:        db 0x00, 0xd8, 0xcf, 0x62, 0x14, 0x52, 0x06, 0x9e, 0xff, 0x3f
e_na:       db 0x00, 0xd8, 0xcf, 0x62, 0x14, 0x52, 0x06, 0x9e, 0xff, 0xbf
e_2a:       db 0x00, 0xd8, 0xcf, 0x62, 0x14, 0x52, 0x06, 0x9e, 0x00, 0x40
e_one:      db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x3f
e_four:     db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x40

; pi as an m80 (loaded and stored unchanged)
align 16
pi80_0:     dq 0xc90fdaa22168c235
            dw 0x4000
align 16
            db 0xa5
pi80_1:     dq 0xc90fdaa22168c235
            dw 0x4000
align 16
            db 0xa5, 0xa5
pi80_2:     dq 0xc90fdaa22168c235
            dw 0x4000
align 16
            db 0xa5, 0xa5, 0xa5
pi80_3:     dq 0xc90fdaa22168c235
            dw 0x4000
align 16
            times 6 db 0xa5
pi80_6:     dq 0xc90fdaa22168c235
            dw 0x4000

align 16
i16_0:      dw -12345
align 16
            db 0xa5
i16_1:      dw -12345
align 16
            db 0xa5, 0xa5, 0xa5
i16_3:      dw -12345
e_i16:      db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0xc0, 0x0c, 0xc0
e_16p:      db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa4, 0xc0, 0x0c, 0xc0
e_16t:      db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0xc0, 0x10, 0xc0
e_16m:      db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0xc1, 0x0c, 0x40
e_m16:      db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0xc1, 0x0c, 0xc0
e_d16:      db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0xc0, 0x08, 0xc0

align 16
i32_0:      dd -123456789
align 16
            db 0xa5
i32_1:      dd -123456789
align 16
            db 0xa5, 0xa5, 0xa5
i32_3:      dd -123456789
e_i32:      db 0x00, 0x00, 0x00, 0x00, 0xa0, 0xa2, 0x79, 0xeb, 0x19, 0xc0
e_16pi:     db 0x00, 0x00, 0x00, 0x00, 0xa0, 0xa0, 0x79, 0xeb, 0x19, 0xc0
e_16ti:     db 0x00, 0x00, 0x00, 0x00, 0xa0, 0xa2, 0x79, 0xeb, 0x1d, 0xc0
e_16mi:     db 0x00, 0x00, 0x00, 0x00, 0xa0, 0xa4, 0x79, 0xeb, 0x19, 0x40
e_im16:     db 0x00, 0x00, 0x00, 0x00, 0xa0, 0xa4, 0x79, 0xeb, 0x19, 0xc0
e_id16:     db 0x00, 0x00, 0x00, 0x00, 0xa0, 0xa2, 0x79, 0xeb, 0x15, 0xc0

; 53 significant bits: the x87's FILD m64 keeps no more (either path)
align 16
i64a_0:     dq 0x7123456789abc800
align 16
            db 0xa5, 0xa5, 0xa5
i64a_3:     dq 0x7123456789abc800
e_i64a:     db 0x00, 0x90, 0x57, 0x13, 0xcf, 0x8a, 0x46, 0xe2, 0x3d, 0x40
align 16
            db 0xa5, 0xa5
i64b_2:     dq -0x0013456789abcdef
e_i64b:     db 0x00, 0x78, 0x6f, 0x5e, 0x4d, 0x3c, 0x2b, 0x9a, 0x33, 0xc0

; -7654321012345678 as packed BCD (53 significant bits, as FILD m64)
align 16
bcd_0:      db 0x78, 0x56, 0x34, 0x12, 0x10, 0x32, 0x54, 0x76, 0x00, 0x80
align 16
            db 0xa5
bcd_1:      db 0x78, 0x56, 0x34, 0x12, 0x10, 0x32, 0x54, 0x76, 0x00, 0x80
e_bcd:      db 0x00, 0x70, 0x5a, 0x80, 0xe7, 0x83, 0x8c, 0xd9, 0x33, 0xc0
