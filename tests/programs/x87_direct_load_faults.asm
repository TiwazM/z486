; Precise faults on direct x87 loads (32-bit code, paging on).
;
; The m16, m64 and m80 loads read their operand part by part. A page fault on
; any part, including the second dword, an m80 tail or the second half of a
; part split across the page, must leave the x87 untouched (TOP and ST0 as
; before) and restart the whole instruction once the handler maps the page.
; Operands ending exactly at the end of the page before an absent one load
; without a fault: no part reads past the operand's last byte. GS has a 4 KB
; byte-granular limit: an operand whose last bytes pass it is #GP(0), one that
; ends on it loads; an SS operand past its limit is #SS(0). Each fault leaves
; the x87 untouched.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
CODE_BASE   equ 0x00010000
VARS        equ 0x00020000
PB          equ 0x00021000          ; present data page
PC          equ 0x00022000          ; the page the tests unmap
PD          equ 0x00023000          ; GS base (limit 0xFFF)
PTE_C       equ 0x00001000 + (PC >> 12) * 4

pf_count    equ VARS + 0x00
pf_cr2      equ VARS + 0x04
pf_err      equ VARS + 0x08
pf_sw       equ VARS + 0x0c
gp_count    equ VARS + 0x10
gp_err      equ VARS + 0x14
gp_bad      equ VARS + 0x18
ss_count    equ VARS + 0x1c
resume      equ VARS + 0x20
result      equ VARS + 0x40

A_LO        equ 0x428c59fb          ; 1.2345678901234567
A_HI        equ 0x3ff3c0ca
PI_LO       equ 0x2168c235          ; pi as an m80
PI_MID      equ 0xc90fdaa2
PI_EXP      equ 0x4000

%macro UNMAP_C 0
    and dword [PTE_C], 0xfffffffe
    invlpg [PC]
%endmacro

%macro MAP_C 0
    or dword [PTE_C], 1
    invlpg [PC]
%endmacro

%macro PUT64 1                      ; m64 A at linear %1
    mov dword [%1], A_LO
    mov dword [%1 + 4], A_HI
%endmacro

%macro PUT80 1                      ; m80 pi at linear %1
    mov dword [%1], PI_LO
    mov dword [%1 + 4], PI_MID
    mov word [%1 + 8], PI_EXP
%endmacro

; ST0 must equal the expected m80, and pop it.
%macro EXPECT 1
    fstp tword [result]
    mov esi, result
    mov edi, CODE_BASE + %1
    call check80
%endmacro

; Page fault number %1, on PC, by a supervisor read, with the x87 status as
; before the faulting instruction (%2: TOP, no exceptions).
%macro EXPECT_PF 2
    cmp dword [pf_count], %1
    jne fail
    cmp dword [pf_cr2], PC
    jne fail
    cmp dword [pf_err], 0
    jne fail
    mov ax, [pf_sw]
    and ax, 0x38ff
    cmp ax, %2
    jne fail
%endmacro

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff           ; 0x08: code, base 0x00010000
    dq 0x00cf93000000ffff           ; 0x10: data, base 0, 4 GB
    dq 0
    dq 0x004293000000ffff           ; 0x20: stack, base 0, limit 0x2FFFF
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_BASE + gdt

pf_handler:
    push eax
    mov eax, cr2
    mov [pf_cr2], eax
    mov eax, [esp + 4]
    mov [pf_err], eax
    fnstsw ax
    mov [pf_sw], ax
    inc dword [pf_count]
    MAP_C
    pop eax
    add esp, 4                      ; discard the error code
    iretd

; #GP and #SS: the x87 is untouched (ST0 = 1.0, TOP 7); resume after the load.
gp_handler:
    inc dword [gp_count]
    jmp limit_fault
ss_handler:
    inc dword [ss_count]
limit_fault:
    push eax
    mov eax, [esp + 4]
    or [gp_err], eax
    fnstsw ax
    and ax, 0x38ff
    cmp ax, 0x3800
    je .x87_ok
    inc dword [gp_bad]
.x87_ok:
    mov eax, [resume]
    mov [esp + 8], eax
    pop eax
    add esp, 4
    iretd

align 8
idt:
    times 12 dq 0
    dw ss_handler, 0x0008
    db 0, 0x8e
    dw 0
    dw gp_handler, 0x0008
    db 0, 0x8e
    dw 0
    dw pf_handler, 0x0008
    db 0, 0x8e
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd CODE_BASE + idt

times 0x200 - ($ - $$) db 0x90
start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x8000

    ; ---- a page fault on any part; the load restarts -----------------------
    PUT64 PC - 4                    ; second dword on PC
    UNMAP_C
    fninit
    fld1
    fld qword [PC - 4]
    EXPECT_PF 1, 0x3800
    EXPECT e_a
    EXPECT e_one

    PUT80 PC - 8                    ; m80 tail on PC
    UNMAP_C
    fld1
    fld tword [PC - 8]
    EXPECT_PF 2, 0x3800
    EXPECT pi80
    EXPECT e_one

    PUT64 PC - 6                    ; second dword split across the page
    UNMAP_C
    fld1
    fld qword [PC - 6]
    EXPECT_PF 3, 0x3800
    EXPECT e_a
    EXPECT e_one

    PUT80 PC - 9                    ; tail word split across the page
    UNMAP_C
    fld1
    fld tword [PC - 9]
    EXPECT_PF 4, 0x3800
    EXPECT pi80
    EXPECT e_one

    mov word [PC - 1], -12345       ; m16 split across the page
    UNMAP_C
    fld1
    fild word [PC - 1]
    EXPECT_PF 5, 0x3800
    EXPECT e_i16
    EXPECT e_one

    PUT64 PC - 4                    ; m64 arithmetic: ST0 unchanged until restart
    UNMAP_C
    fld1
    fldz
    fadd qword [PC - 4]
    EXPECT_PF 6, 0x3000             ; TOP 6: 0.0 and 1.0
    EXPECT e_a
    EXPECT e_one

    mov dword [PC - 4], 0x89abc800  ; FILD m64, second dword on PC
    mov dword [PC], 0x71234567
    UNMAP_C
    fld1
    fild qword [PC - 4]
    EXPECT_PF 7, 0x3800
    EXPECT e_i64
    EXPECT e_one

    PUT64 PC                        ; first dword on PC
    UNMAP_C
    fld1
    fld qword [PC]
    EXPECT_PF 8, 0x3800
    EXPECT e_a
    EXPECT e_one

    ; ---- operands ending at the page end before PC load -------------------
    PUT80 PC - 10
    PUT64 PC - 18
    mov word [PC - 20], -12345
    mov dword [PC - 24], 0x3fc00000 ; 1.5
    UNMAP_C
    fld tword [PC - 10]
    EXPECT pi80
    fld qword [PC - 18]
    EXPECT e_a
    fild word [PC - 20]
    EXPECT e_i16
    fld dword [PC - 24]
    EXPECT e_1_5
    PUT64 PC - 8
    fld qword [PC - 8]
    EXPECT e_a
    mov dword [PC - 10], 0x12345678 ; FBLD -7654321012345678
    mov dword [PC - 6], 0x76543210
    mov word [PC - 2], 0x8000
    fbld tword [PC - 10]
    EXPECT e_bcd
    mov word [PC - 2], -12345
    fldz
    fiadd word [PC - 2]
    EXPECT e_i16
    cmp dword [pf_count], 8
    jne fail

    ; ---- page-crossing operands, both pages present -----------------------
    MAP_C
    PUT64 PC - 3
    fld qword [PC - 3]
    EXPECT e_a
    PUT80 PC - 5
    fld tword [PC - 5]
    EXPECT pi80
    PUT80 PC - 2
    fld tword [PC - 2]
    EXPECT pi80
    cmp dword [pf_count], 8
    jne fail

    ; ---- the GS limit (0xFFF) over the whole operand ----------------------
    PUT64 PD + 0xff8
    PUT80 PD + 0xff6 - 0x20
    fld1
    fld qword [gs:0xff8]            ; ends on the limit
    EXPECT e_a
    fld tword [gs:0xfd6]
    EXPECT pi80
    fild word [gs:0xffe]
    fstp st0
    fld dword [gs:0xffc]
    fstp st0
    fld tword [gs:0xff6]
    fstp st0
    mov dword [resume], gp1
    fld qword [gs:0xffa]            ; second part passes the limit
gp1:
    mov dword [resume], gp2
    fld tword [gs:0xff7]            ; only the tail passes it
gp2:
    mov dword [resume], gp3
    fild word [gs:0xfff]
gp3:
    mov dword [resume], gp4
    fld dword [gs:0xffd]
gp4:
    mov dword [resume], gp5
    fmul qword [gs:0xffc]
gp5:
    mov dword [resume], gp6
    fiadd dword [gs:0xffd]
gp6:
    mov dword [resume], gp7
    ficom word [gs:0xfff]
gp7:
    cmp dword [gp_count], 7
    jne fail

    ; ---- SS with a 0x2FFFF limit -------------------------------------------
    mov ax, 0x20
    mov ss, ax
    mov esp, 0x2f000
    mov ebp, 0x2fff8
    PUT64 0x2fff8
    fld qword [ebp]                 ; ends on the limit
    EXPECT e_a
    mov dword [resume], ss1
    fld qword [ebp + 2]
ss1:
    mov dword [resume], ss2
    fld tword [ebp - 1]
ss2:
    mov ax, 0x10
    mov ss, ax
    mov esp, 0x8000
    cmp dword [ss_count], 2
    jne fail
    cmp dword [gp_err], 0
    jne fail
    cmp dword [gp_bad], 0
    jne fail
    EXPECT e_one                    ; the 1.0 under every faulting load
    fnstsw ax
    test ax, 0x3800
    jnz fail

    mov al, 1
    out STATUS_PORT, al
    hlt

fail:
    mov al, 0xff
    out STATUS_PORT, al
    hlt

check80:
    mov ecx, 10
    repe cmpsb
    jne fail
    ret

align 16
e_one:  db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x3f
e_1_5:  db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x3f
e_a:    db 0x00, 0xd8, 0xcf, 0x62, 0x14, 0x52, 0x06, 0x9e, 0xff, 0x3f
pi80:   dd PI_LO, PI_MID
        dw PI_EXP
e_i16:  db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0xc0, 0x0c, 0xc0
e_i64:  db 0x00, 0x90, 0x57, 0x13, 0xcf, 0x8a, 0x46, 0xe2, 0x3d, 0x40
e_bcd:  db 0x00, 0x70, 0x5a, 0x80, 0xe7, 0x83, 0x8c, 0xd9, 0x33, 0xc0

times 0x1000 - ($ - $$) db 0
