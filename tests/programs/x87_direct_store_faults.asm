; Precise faults on direct x87 stores (32-bit code, paging on).
;
; The m16, m64 and m80 stores check every part of their destination for
; writing before the x87 sees the command. A page fault on any part,
; including the second dword, an m80 tail or the second half of a part split
; across the page, must leave memory and the x87 untouched: no byte of the
; destination written (the part on the present page included), TOP and ST0
; as before (FSTP, FISTP and FBSTP have not popped). Once the handler maps
; the page the restarted store writes exactly the operand. (The original
; routine writes the first part, then faults with the x87 mid-transfer.)
; Destinations ending exactly at the end of the page before an absent one
; are written without a fault; page-crossing destinations with both pages
; present are written. GS is a read-only data segment with a 4 KB
; byte-granular limit: a store to it is #GP(0), and so is one whose last
; bytes pass the limit of a writable copy; one ending on the limit is
; written. An SS destination past its limit is #SS(0). No fault writes a
; byte or changes the x87. FNSTSW and FNSTCW m16 to the absent page fault on
; their write and restart (they change nothing else).

BITS 32
ORG 0

STATUS_PORT equ 0xE0
CODE_BASE   equ 0x00010000
VARS        equ 0x00020000
PB          equ 0x00021000          ; present data page
PC          equ 0x00022000          ; the page the tests unmap
PD          equ 0x00023000          ; FS/GS base (limit 0xFFF)
PTE_C       equ 0x00001000 + (PC >> 12) * 4

pf_count    equ VARS + 0x00
pf_cr2      equ VARS + 0x04
pf_err      equ VARS + 0x08
pf_sw       equ VARS + 0x0c
pf_bad      equ VARS + 0x10
gp_count    equ VARS + 0x14
gp_err      equ VARS + 0x18
gp_bad      equ VARS + 0x1c
ss_count    equ VARS + 0x20
resume      equ VARS + 0x24
result      equ VARS + 0x40

%macro UNMAP_C 0
    and dword [PTE_C], 0xfffffffe
    invlpg [PC]
%endmacro

%macro MAP_C 0
    or dword [PTE_C], 1
    invlpg [PC]
%endmacro

; Fill the 32 bytes around the page boundary (PC-16 .. PC+15) with A5.
%macro FILL 0
    MAP_C
    mov edi, PC - 16
    mov ecx, 32
    mov al, 0xa5
    rep stosb
%endmacro

; The window must be A5 except for the %2 bytes of %3 at %1.
%macro CHECKW 3
    mov ebx, %1 - (PC - 16)
    mov esi, CODE_BASE + %3
    mov ecx, %2
    call check_window
%endmacro

; Page fault number %1 at %2, a supervisor write to an absent page, with the
; x87 status as before the faulting store (%3: TOP, no exceptions).
%macro EXPECT_PF 3
    cmp dword [pf_count], %1
    jne fail
    cmp dword [pf_cr2], %2
    jne fail
    cmp dword [pf_err], 2
    jne fail
    mov ax, [pf_sw]
    and ax, 0x38ff
    cmp ax, %3
    jne fail
%endmacro

; ST0 must equal the m80 at %1; pop it.
%macro EXPECT 1
    fstp tword [result]
    mov esi, result
    mov edi, CODE_BASE + %1
    mov ecx, 10
    repe cmpsb
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

; #PF: nothing of the destination is written yet (the window is still all
; A5) and the x87 is as before. Map the page and restart the store.
pf_handler:
    push eax
    push ecx
    push edi
    mov eax, cr2
    mov [pf_cr2], eax
    mov eax, [esp + 12]
    mov [pf_err], eax
    fnstsw ax
    mov [pf_sw], ax
    inc dword [pf_count]
    MAP_C
    mov edi, PC - 16
    mov ecx, 32
    mov al, 0xa5
    repe scasb
    je .clean
    inc dword [pf_bad]
.clean:
    pop edi
    pop ecx
    pop eax
    add esp, 4                      ; discard the error code
    iretd

; #GP and #SS: the x87 is untouched (ST0 = pi, TOP 7, no exception but the
; PE of the stores of pi that succeeded); resume after the store.
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
    and ax, 0x38df
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
    cld
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x8000
    fninit

    ; ---- a page fault on any part; the store restarts ---------------------
    FILL
    UNMAP_C
    fld1
    fld qword [CODE_BASE + r64]
    fst qword [PC - 4]              ; second dword on PC
    EXPECT_PF 1, PC, 0x3000
    CHECKW PC - 4, 8, r64
    EXPECT e_a
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fld tword [CODE_BASE + pi80]
    fstp tword [PC - 8]             ; m80 tail on PC
    EXPECT_PF 2, PC, 0x3000
    CHECKW PC - 8, 10, pi80
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fld qword [CODE_BASE + r64]
    fstp qword [PC - 6]             ; second dword split across the page
    EXPECT_PF 3, PC, 0x3000
    CHECKW PC - 6, 8, r64
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fld tword [CODE_BASE + pi80]
    fstp tword [PC - 9]             ; tail split across the page
    EXPECT_PF 4, PC, 0x3000
    CHECKW PC - 9, 10, pi80
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fld tword [CODE_BASE + pi80]
    fstp tword [PC - 6]             ; second dword split, tail on PC
    EXPECT_PF 5, PC, 0x3000
    CHECKW PC - 6, 10, pi80
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fild word [CODE_BASE + i16]
    fistp word [PC - 1]             ; m16 split across the page
    EXPECT_PF 6, PC, 0x3000
    CHECKW PC - 1, 2, i16
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fild qword [CODE_BASE + i64]
    fistp qword [PC - 4]            ; FISTP m64, second dword on PC
    EXPECT_PF 7, PC, 0x3000
    CHECKW PC - 4, 8, i64
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fbld tword [CODE_BASE + bcd]
    fbstp tword [PC - 8]            ; FBSTP, tail on PC
    EXPECT_PF 8, PC, 0x3000
    CHECKW PC - 8, 10, bcd
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fld dword [CODE_BASE + r32]
    fstp dword [PC - 2]             ; m32 split across the page
    EXPECT_PF 9, PC, 0x3000
    CHECKW PC - 2, 4, r32
    EXPECT e_one

    FILL
    UNMAP_C
    fld1
    fld qword [CODE_BASE + r64]
    fstp qword [PC + 4]             ; first dword on PC
    EXPECT_PF 10, PC + 4, 0x3000
    CHECKW PC + 4, 8, r64
    EXPECT e_one
    cmp dword [pf_bad], 0
    jne fail

    FILL
    UNMAP_C
    fld1
    fnstcw [PC]
    EXPECT_PF 11, PC, 0x3800
    cmp word [PC], 0x037f
    jne fail
    FILL
    UNMAP_C
    fnstsw [PC + 2]
    EXPECT_PF 12, PC + 2, 0x3800
    cmp word [PC + 2], 0x3800
    jne fail
    EXPECT e_one

    ; ---- destinations ending at the page end before PC --------------------
    FILL
    UNMAP_C
    fld tword [CODE_BASE + pi80]
    fstp tword [PC - 10]
    mov esi, PC - 10
    mov edi, CODE_BASE + pi80
    mov ecx, 10
    repe cmpsb
    jne fail
    fld qword [CODE_BASE + r64]
    fstp qword [PC - 8]
    mov esi, PC - 8
    mov edi, CODE_BASE + r64
    mov ecx, 8
    repe cmpsb
    jne fail
    fild word [CODE_BASE + i16]
    fistp word [PC - 2]
    cmp word [PC - 2], -12345
    jne fail
    fld dword [CODE_BASE + r32]
    fstp dword [PC - 12]
    cmp dword [PC - 12], 0xc0912345
    jne fail
    cmp dword [pf_count], 12
    jne fail

    ; ---- page-crossing destinations, both pages present -------------------
    FILL
    fld qword [CODE_BASE + r64]
    fstp qword [PC - 3]
    CHECKW PC - 3, 8, r64
    FILL
    fld tword [CODE_BASE + pi80]
    fstp tword [PC - 5]
    CHECKW PC - 5, 10, pi80
    FILL
    fld tword [CODE_BASE + pi80]
    fstp tword [PC - 2]
    CHECKW PC - 2, 10, pi80
    FILL
    fild word [CODE_BASE + i16]
    fistp word [PC - 1]
    CHECKW PC - 1, 2, i16
    cmp dword [pf_count], 12
    jne fail
    fnstsw ax
    test ax, 0x38ff
    jnz fail

    ; ---- segment checks ----------------------------------------------------
    mov edi, PD + 0xff0
    mov ecx, 16
    mov al, 0xa5
    rep stosb
    fld tword [CODE_BASE + pi80]    ; TOP 7 throughout
    mov dword [resume], gp1
    fst qword [gs:0xf00]            ; read-only segment
gp1:
    mov dword [resume], gp2
    fstp qword [fs:0xffa]           ; second part passes the limit
gp2:
    mov dword [resume], gp3
    fstp tword [fs:0xff7]           ; only the tail passes it
gp3:
    mov dword [resume], gp4
    fist word [fs:0xfff]
gp4:
    mov dword [resume], gp5
    fbstp tword [fs:0xff9]
gp5:
    mov dword [resume], gp6
    fst dword [fs:0xffd]
gp6:
    mov dword [resume], gp_unexpected
    mov edi, PD + 0xff0             ; nothing written
    mov ecx, 16
    mov al, 0xa5
    repe scasb
    jne fail
    fst qword [fs:0xff8]            ; ends on the limit
    mov esi, PD + 0xff8
    mov edi, CODE_BASE + r64_pi
    mov ecx, 8
    repe cmpsb
    jne fail
    fld st0
    fstp tword [fs:0xff6]
    mov esi, PD + 0xff6
    mov edi, CODE_BASE + pi80
    mov ecx, 10
    repe cmpsb
    jne fail
    cmp dword [gp_count], 6
    jne fail

    ; ---- SS with a 0x2FFFF limit -------------------------------------------
    mov ax, 0x20
    mov ss, ax
    mov esp, 0x2f000
    mov ebp, 0x2fff8
    mov dword [ebp], 0xa5a5a5a5
    mov dword [ebp + 4], 0xa5a5a5a5
    mov dword [resume], ss1
    fst qword [ebp + 2]
ss1:
    mov dword [resume], ss2
    fstp tword [ebp - 1]
ss2:
    mov dword [resume], gp_unexpected
    cmp dword [ebp], 0xa5a5a5a5
    jne fail
    cmp dword [ebp + 4], 0xa5a5a5a5
    jne fail
    fst qword [ebp]                 ; ends on the limit
    cmp dword [ebp + 4], 0x400921fb
    jne fail
    mov ax, 0x10
    mov ss, ax
    mov esp, 0x8000
    cmp dword [ss_count], 2
    jne fail
    cmp dword [gp_err], 0
    jne fail
    cmp dword [gp_bad], 0
    jne fail
    EXPECT pi80                     ; the pi under every faulting store
    fnstsw ax
    and ax, 0x38df
    jnz fail                        ; TOP 0, only PE (pi to m64)

    mov al, 1
    out STATUS_PORT, al
    hlt

gp_unexpected:
fail:
    mov al, 0xff
    out STATUS_PORT, al
    hlt

; The window PC-16 .. PC+15 must be A5 except for ECX bytes from ESI at
; window offset EBX.
check_window:
    xor edx, edx
.loop:
    mov al, 0xa5
    cmp edx, ebx
    jb .cmp
    lea edi, [ebx + ecx]
    cmp edx, edi
    jae .cmp
    mov edi, edx
    sub edi, ebx
    mov al, [esi + edi]
.cmp:
    cmp al, [PC - 16 + edx]
    jne fail
    inc edx
    cmp edx, 32
    jb .loop
    ret

align 16
r32:    dd 0xc0912345
i16:    dw -12345
align 4
r64:    dq 0x3ff3c0ca428c59fb       ; 1.2345678901234567
i64:    dq 0x7123456789abc800       ; 53 significant bits
pi80:   dq 0xc90fdaa22168c235
        dw 0x4000
bcd:    db 0x78, 0x56, 0x34, 0x12, 0x10, 0x32, 0x54, 0x76, 0x00, 0x80
r64_pi: dq 0x400921fb54442d18
align 16
e_one:  db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x3f
e_a:    db 0x00, 0xd8, 0xcf, 0x62, 0x14, 0x52, 0x06, 0x9e, 0xff, 0x3f

times 0x1000 - ($ - $$) db 0
