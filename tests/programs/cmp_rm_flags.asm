; cmp_rm_flags.asm - CMP r,m as a direct ALU load: flags only, at once
;
; CMP r,m (3A/3B) is a direct load whose writeback computes the flags and
; writes no register. Each check sets the flags the other way first, then
; CMP r,m, then reads the flags straight after: a Jcc either way, LAHF,
; PUSHF (OF), ADC (CF), SETcc; byte registers (AL, AH), words and dwords;
; the compared register must be unchanged. Twenty passes, so all but the
; first find the operands in the data cache.
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

; Fail with code %2 if condition %1 holds.
%macro FAILIF 2
    j%-1 %%ok
    mov ax, %2
    jmp fail
%%ok:
%endmacro

start:
    cli
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0xF000
    mov cx, 20

pass:
; equal: ZF set, after flags with ZF clear
    mov ax, 5
    or ax, ax                   ; ZF=0
    cmp ax, [w5]
    jne .f1
    cmp ax, 5
    FAILIF ne, 2                ; AX unchanged
    jmp .ok1
.f1: mov ax, 1
    jmp fail
.ok1:

; not equal: ZF clear, after flags with ZF set
    mov ax, 6
    xor bx, bx                  ; ZF=1
    cmp ax, [w5]
    je .f3
    jb .f3                      ; 6 < 5 unsigned: no
    jl .f3                      ; 6 < 5 signed: no
    jmp .ok3
.f3: mov ax, 3
    jmp fail
.ok3:

; below: CF set, after flags with CF clear
    mov ax, 4
    clc
    cmp ax, [w5]
    jae .f4
    mov bx, 0                   ; MOV keeps the flags
    adc bx, 0                   ; BX = CF = 1
    cmp bx, 1
    FAILIF ne, 5
    jmp .ok4
.f4: mov ax, 4
    jmp fail
.ok4:

; LAHF right after: 4 - 5 = FFFFh: SF ZF.AF.PF.CF = 1 0 1 1 1 (+ bit 1)
    mov ax, 4
    cmp ax, [w5]
    lahf
    and ah, 0xD5
    cmp ah, 0x95
    FAILIF ne, 6

; OF: 8000h - 1 = 7FFFh overflows, PUSHF right after
    mov ax, 0x8000
    cmp ax, [w1]
    pushf
    pop bx
    test bx, 0x0800
    FAILIF z, 7
    cmp ax, [w1]                ; again, for JNO straight after
    jno .f8
    jmp .ok8
.f8: mov ax, 8
    jmp fail
.ok8:

; SETcc and signed compare: -1 < 5
    mov ax, 0xFFFF
    cmp ax, [w5]
    setl dl
    cmp dl, 1
    FAILIF ne, 9
    cmp ax, 0xFFFF
    FAILIF ne, 10               ; AX unchanged

; byte registers: AL and AH
    mov ax, 0x0705
    cmp al, [b5]
    jne .f11
    cmp ah, [b5]
    jbe .f11                    ; 7 > 5
    cmp ax, 0x0705
    FAILIF ne, 12
    jmp .ok11
.f11: mov ax, 11
    jmp fail
.ok11:

; dword: CMP EAX,[d] with a difference only in the high word
    mov eax, 0x00010005
    cmp eax, [d5]
    je .f13
    ja .ok13
.f13: mov ax, 13
    jmp fail
.ok13:
    cmp eax, 0x00010005
    FAILIF ne, 14

; back to back, the second's flags win
    mov ax, 5
    cmp ax, [w1]                ; 5 > 1
    cmp ax, [w5]                ; equal
    jne .f15
    cmp ax, [w5]
    cmp ax, [w1]
    je .f15
    jmp .ok15
.f15: mov ax, 15
    jmp fail
.ok15:

    dec cx
    jnz pass

    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

fail:
    movzx eax, ax
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt

align 4
d5: dd 5
w5: dw 5
w1: dw 1
b5: db 5
