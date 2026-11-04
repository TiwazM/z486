; When a direct x87 load must not go direct (32-bit code).
;
; With CR0.TS or CR0.EM set every load falls back to the original routine,
; which raises #NM before it touches memory; the handler clears the bit and
; the restarted load goes direct. With an unmasked exception pending (or
; just unmasked) the loads fall back too; the original routine checks ERROR#
; only while the x87 is busy, so #MF comes here at the FWAIT after the load.
; A mask load (FLDCW, FLDENV) still in the x87 when the next load issues
; makes that load fall back as well: the masks it unmasks would leave a
; queued direct operand waiting for ever. The JSON pins the number of loads
; that went direct. Direct loads record FCS:FOP and the operand offset as the
; original routine does, which FNSTENV reports.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
CODE_BASE   equ 0x00010000

%macro EXPECT 1                     ; ST0 must equal the m80 at %1; pop it
    fstp tword [result]
    mov esi, result
    mov edi, %1
    call check80
%endmacro

%macro SET_CR0 1
    mov eax, cr0
    or eax, %1
    mov cr0, eax
%endmacro

%macro PEND_ZE 0                   ; ZE unmasked: 1.0 / 0.0 leaves ZE pending
    fninit
    fldcw [cw_unmask_ze]
    fldz
    fld1
    fdiv st0, st1                   ; ST0 = 1.0 kept, ST1 = 0.0
%endmacro

; #MF number %1, for the pending ZE.
%macro EXPECT_MF 1
    fwait
    cmp dword [mf_count], %1
    jne fail
    mov ax, [mf_sw]
    and ax, 0x0084
    cmp ax, 0x0084
    jne fail
%endmacro

%macro EXPECT_ENV 2                 ; FOP, operand offset of the last x87 load
    fnstenv [env]
    cmp dword [env + 16], (%1 << 16) | 0x0008
    jne fail
    cmp dword [env + 20], %2
    jne fail
%endmacro

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff           ; 0x08: code, base 0x00010000
    dq 0x00cf93010000ffff           ; 0x10: data, base 0x00010000
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_BASE + gdt

nm_handler:                         ; #NM: clear TS and EM
    inc dword [nm_count]
    push eax
    mov eax, cr0
    and eax, 0xfffffff3
    mov cr0, eax
    pop eax
    iretd

mf_handler:                         ; #MF: clear the pending exception
    inc dword [mf_count]
    push eax
    fnstsw ax
    mov [mf_sw], ax
    pop eax
    fnclex
    iretd

align 8
idt:
    times 7 dq 0
    dw nm_handler, 0x0008
    db 0, 0x8e
    dw 0
    times 8 dq 0
    dw mf_handler, 0x0008
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
    mov esp, 0x7000
    fninit

    ; ---- CR0.TS, then CR0.EM: #NM, then the load restarts ----------------
    SET_CR0 8
    fld qword [real_a]
    EXPECT e_a
    SET_CR0 8
    fld tword [pi80]
    EXPECT pi80
    SET_CR0 8
    fild word [int16]
    EXPECT e_i16
    fldz
    SET_CR0 8
    fadd qword [real_a]             ; #NM before the add; ST0 = 0.0 kept
    EXPECT e_a
    SET_CR0 8
    fld dword [real_1_5]
    EXPECT e_1_5
    SET_CR0 4
    fld qword [real_a]
    EXPECT e_a
    SET_CR0 4
    fbld tword [bcd]
    EXPECT e_bcd
    fldz
    SET_CR0 4
    fiadd word [int16]
    EXPECT e_i16
    cmp dword [nm_count], 8
    jne fail
    fninit

    ; ---- an unmasked exception pending: the loads fall back ---------------
    SET_CR0 0x20                    ; NE: errors raise #MF
    PEND_ZE
    fld qword [real_a]
    EXPECT_MF 1
    EXPECT e_a
    EXPECT e_one
    PEND_ZE
    fild word [int16]
    EXPECT_MF 2
    EXPECT e_i16
    EXPECT e_one
    PEND_ZE
    fld tword [pi80]
    EXPECT_MF 3
    EXPECT pi80
    EXPECT e_one
    PEND_ZE
    fmul qword [real_a]             ; 1.0 * A
    EXPECT_MF 4
    EXPECT e_a
    PEND_ZE
    fld dword [real_1_5]
    EXPECT_MF 5
    EXPECT e_1_5
    EXPECT e_one
    fninit

    ; ---- mask loads in flight when the next load issues -------------------
    ; Each FLDCW unmasks IE while every exception was masked before it.
%macro UNMASK_IE 0
    fldcw [cw_default]
    fnstsw ax                       ; the masked control word is in
    fldcw [cw_unmask_ie]
%endmacro
    UNMASK_IE
    fld dword [real_1_5]
    UNMASK_IE
    fld qword [real_a]
    UNMASK_IE
    fild word [int16]
    UNMASK_IE
    fld tword [pi80]
    UNMASK_IE
    fild dword [int32]
    EXPECT e_i32
    EXPECT pi80
    EXPECT e_i16
    EXPECT e_a
    EXPECT e_1_5
    fnstenv [env]
    mov word [env], 0x037e          ; FLDENV unmasks IE
    fldenv [env]
    fld qword [real_a]
    EXPECT e_a
    fldenv [env]
    fld tword [pi80]
    EXPECT pi80
    fldcw [cw_default]
    fnstsw ax                       ; waits for the mask load
    fld qword [real_a]              ; masked again: direct
    EXPECT e_a
    cmp dword [mf_count], 5
    jne fail

    ; ---- FCS:FOP and the operand offset (FNSTENV, 32-bit format) ----------
    fninit
    fld qword [real_a]
    EXPECT_ENV 0x505, real_a
    fld dword [real_1_5]
    EXPECT_ENV 0x105, real_1_5
    fild word [int16]
    EXPECT_ENV 0x705, int16
    fld tword [pi80]
    EXPECT_ENV 0x32d, pi80
    fadd qword [real_a]
    EXPECT_ENV 0x405, real_a
    fiadd word [int16]
    EXPECT_ENV 0x605, int16
    fiadd dword [int32]
    EXPECT_ENV 0x205, int32
    mov ebx, real_a - 4
    fld qword [ebx + 4]
    EXPECT_ENV 0x543, real_a
    fninit

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

align 4
nm_count:       dd 0
mf_count:       dd 0
mf_sw:          dw 0
cw_default:     dw 0x037f
cw_unmask_ie:   dw 0x037e
cw_unmask_ze:   dw 0x037b
align 16
real_a:            dq 0x3ff3c0ca428c59fb   ; 1.2345678901234567
real_1_5:            dd 0x3fc00000           ; 1.5
int16:            dw -12345
align 4
int32:            dd 0x0001e240           ; 123456
align 16
pi80:           dq 0xc90fdaa22168c235
                dw 0x4000
align 16
bcd:            db 0x78, 0x56, 0x34, 0x12, 0x10, 0x32, 0x54, 0x76, 0x00, 0x80
align 16
e_one:          db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x3f
e_1_5:          db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x3f
e_a:            db 0x00, 0xd8, 0xcf, 0x62, 0x14, 0x52, 0x06, 0x9e, 0xff, 0x3f
e_i16:          db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0xc0, 0x0c, 0xc0
e_bcd:          db 0x00, 0x70, 0x5a, 0x80, 0xe7, 0x83, 0x8c, 0xd9, 0x33, 0xc0
e_i32:          db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xf1, 0x0f, 0x40
align 16
result:         times 10 db 0
align 16
env:            times 28 db 0

times 0x1000 - ($ - $$) db 0
