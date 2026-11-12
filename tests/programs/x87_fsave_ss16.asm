; x87_fsave_ss16.asm - FNSAVE/FRSTOR from a 32-bit code segment on a 16-bit stack.
;
; OS/2 Warp 4's kernel saves the FPU owner's state with FNSAVE [EBX] (EBX = a
; flat address above 64K) while SS is its 16-bit ring-0 stack (B = 0). On the
; board that #PF'd with CR2 = (EBX & 0FFFFh) + 1Ch: the register area (offset
; 28 on) was written at a 16-bit address. The 32-bit code segment sets the
; address size; the stack segment's B bit must not matter.
;
; SAVE (flat, DS base 0) and its 16-bit alias ALIAS are both filled with EEh.
; FNSAVE must write the whole 108-byte image at SAVE, nothing at ALIAS; FRSTOR
; must read it back from SAVE. SGDT and SIDT (a word, then a dword) are checked
; the same way against a copy stored below 64K. x87_fsave_ss32 runs the same
; code on a 32-bit stack.

BITS 32
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SAVE  equ 0x5A010
ALIAS equ 0x0A010
REF   equ 0x08000             ; below 64K: right with either address width

%macro CHECK 3                  ; operand, expected, fail code
    cmp %1, %2
    mov al, %3
    jne fail
%endmacro

start:
    cli
    cld
    mov edi, SAVE
    mov ecx, 108
    mov al, 0xEE
    rep stosb
    mov edi, ALIAS
    mov ecx, 0x120
    rep stosb
    mov edi, SAVE+0x100
    mov ecx, 0x20
    rep stosb

    fninit
    fld dword [cs:cs_three]        ; ST1 = 3.0 after the next load
    fld dword [cs:cs_five]         ; ST0 = 5.0
    mov ebx, SAVE
    fnsave [ebx]

    CHECK word [SAVE+0], 0x037F, 1          ; control word
    CHECK dword [SAVE+28], 0, 2             ; ST0 = 5.0 = 4001 A000000000000000
    CHECK dword [SAVE+32], 0xA0000000, 3
    CHECK word [SAVE+36], 0x4001, 4
    CHECK dword [SAVE+38], 0, 5             ; ST1 = 3.0 = 4000 C000000000000000
    CHECK dword [SAVE+42], 0xC0000000, 6
    CHECK word [SAVE+46], 0x4000, 7
    CHECK dword [ALIAS+0], 0xEEEEEEEE, 8    ; nothing at the 16-bit alias
    CHECK dword [ALIAS+28], 0xEEEEEEEE, 9
    CHECK dword [ALIAS+104], 0xEEEEEEEE, 10

    ; FRSTOR must read the registers from SAVE, not ALIAS.
    fninit
    frstor [ebx]
    fld dword [cs:cs_five]
    fcompp                      ; 5.0 vs restored ST0, pop both
    fnstsw ax
    and ah, 0x45
    CHECK ah, 0x40, 11
    fld dword [cs:cs_three]
    fcompp                      ; 3.0 vs restored ST1
    fnstsw ax
    and ah, 0x45
    CHECK ah, 0x40, 12

    sgdt [REF]
    sidt [REF+8]
    sgdt [ebx+0x100]
    sidt [ebx+0x108]
    mov eax, [REF+2]
    CHECK [SAVE+0x102], eax, 13
    mov eax, [REF+10]
    CHECK [SAVE+0x10A], eax, 14
    CHECK dword [ALIAS+0x100], 0xEEEEEEEE, 15
    CHECK dword [ALIAS+0x104], 0xEEEEEEEE, 16
    CHECK dword [ALIAS+0x108], 0xEEEEEEEE, 17
    CHECK dword [ALIAS+0x10C], 0xEEEEEEEE, 18

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail:
    mov dx, 0xE4
    out dx, al
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 4
cs_five:  dd 0x40A00000         ; 5.0f
cs_three: dd 0x40400000         ; 3.0f
