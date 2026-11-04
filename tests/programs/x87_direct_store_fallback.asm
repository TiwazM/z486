; Direct x87 stores and status reads: fallbacks, exceptions, ordering
; (32-bit code).
;
; - With CR0.TS or CR0.EM set every store, FNSTSW and FNSTCW falls back to
;   the original routine, which raises #NM before it touches anything; the
;   handler clears the bit and the restarted instruction goes direct.
; - With an unmasked exception pending they fall back too; the original
;   routine checks ERROR# only while the x87 is busy, so the store is done
;   and #MF comes at the FWAIT after it. FNSTSW reads the pending ES.
; - An unmasked exception raised by the store itself (invalid FIST/FISTP of
;   a value out of range or a NaN, FBSTP of a NaN, overflow of FST m64 and
;   FSTP m32): the direct store writes what the original routine writes
;   (the integer or BCD indefinite, infinity), pops as it pops (not after
;   an invalid integer store; this x87 pops after FSTP's overflow) and #MF
;   comes at the next FWAIT. The program passes on the original routine as
;   well, but for the counts in the JSON.
; - FNSTSW AX and FNSTSW m16 right after FCOM, FDIV and FSQRT see the
;   status those leave when they finish (the divide by zero, the invalid
;   square root, the compare's condition codes); FNSTCW right after FLDCW
;   sees the new control word.
; - A store right after FLDCW uses the new rounding control; the C
;   float-to-int sequence (FNSTCW, FLDCW chop, FISTP m32, FLDCW) truncates.
; - A direct FLDCW whose control word keeps every exception masked is no
;   mask load to the loads and FLDCWs right after it: they go direct, queued
;   behind it, and load exactly. After one that unmasks an exception the next
;   load falls back (the masks it loads decide whether a load may queue).
; - FNSTENV reports FCS:FOP and the operand offset of the last direct
;   store; FNSTSW and FNSTCW change neither.
;
; The JSON pins how many stores and status reads went direct.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
CODE_BASE   equ 0x00010000

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

; #MF number %1 at the FWAIT, with status bits %2 (of 0x38ff) when it came.
%macro EXPECT_MF 2
    fwait
    cmp dword [mf_count], %1
    jne fail
    mov ax, [mf_sw]
    and ax, 0x38ff
    cmp ax, %2
    jne fail
%endmacro

; AX & %1 must be %2.
%macro EXPECT_AX 2
    and ax, %1
    cmp ax, %2
    jne fail
%endmacro

; ST0 must equal the m80 at %1; pop it.
%macro EXPECT 1
    fstp tword [res80]
    mov esi, res80
    mov edi, %1
    mov ecx, 10
    repe cmpsb
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

mf_handler:                         ; #MF: record and clear the exception
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
    cld
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x7000
    fninit

    ; ---- CR0.TS, then CR0.EM: #NM, then the instruction restarts ----------
    fld qword [real_a]
    SET_CR0 8
    fst qword [res64]
    cmp dword [res64 + 4], 0x3ff3c0ca
    jne fail
    SET_CR0 4
    fist word [res16]
    cmp word [res16], 1
    jne fail
    fld tword [pi80]
    SET_CR0 8
    fstp tword [res80]
    cmp word [res80 + 8], 0x4000
    jne fail
    SET_CR0 8
    fnstsw ax
    EXPECT_AX 0x38ff, 0x3820        ; TOP 7, PE (FIST of a)
    SET_CR0 4
    fnstsw [res16]
    cmp word [res16], 0x3820
    jne fail
    SET_CR0 8
    fnstcw [res16]
    cmp word [res16], 0x037f
    jne fail
    SET_CR0 4
    fistp qword [res64]
    cmp dword [res64], 1
    jne fail
    cmp dword [nm_count], 7
    jne fail
    fnstsw ax
    EXPECT_AX 0x38ff, 0x0020

    ; ---- an unmasked exception pending: everything falls back -------------
    SET_CR0 0x20                    ; NE: errors raise #MF
    PEND_ZE
    fst qword [res64]               ; stored; #MF at the FWAIT
    cmp dword [res64 + 4], 0x3ff00000
    jne fail
    EXPECT_MF 1, 0x3084
    PEND_ZE
    fist word [res16]
    cmp word [res16], 1
    jne fail
    EXPECT_MF 2, 0x3084
    PEND_ZE
    fld tword [pi80]
    fstp tword [res80]
    cmp word [res80 + 8], 0x4000
    jne fail
    EXPECT_MF 3, 0x3084
    PEND_ZE
    fnstsw ax                       ; no wait: no #MF, ES and B pending
    EXPECT_AX 0xb8ff, 0xb084
    fnstsw [res16]
    cmp word [res16], 0xb084
    jne fail
    fnstcw [res16]
    cmp word [res16], 0x037b
    jne fail
    EXPECT_MF 4, 0x3084
    fninit

    ; ---- an unmasked exception raised by the store itself -------------------
    fldcw [cw_unmask_io]            ; IE and OE unmasked
    fld qword [big]                 ; 1e10: no int16, no int32
    fistp word [res16]              ; invalid, no pop
    mov ax, [res16]
    mov [st_i16], ax
    EXPECT_MF 5, 0x3881
    fist dword [res32]
    mov eax, [res32]
    mov [st_i32], eax
    EXPECT_MF 6, 0x3881
    fstp st0
    fld tword [qnan80]
    fistp qword [res64]
    mov eax, [res64]
    mov [st_i64], eax
    mov eax, [res64 + 4]
    mov [st_i64 + 4], eax
    EXPECT_MF 7, 0x3881
    fbstp tword [res80]
    mov eax, [res80]
    mov [st_bcd], eax
    mov eax, [res80 + 4]
    mov [st_bcd + 4], eax
    mov ax, [res80 + 8]
    mov [st_bcd + 8], ax
    EXPECT_MF 8, 0x3881
    fstp st0
    fld tword [huge80]              ; 2^2000: no m64, no m32
    fst qword [res64]               ; overflow
    mov eax, [res64]
    mov [st_r64], eax
    mov eax, [res64 + 4]
    mov [st_r64 + 4], eax
    fnstsw ax
    mov [st_sw64], ax
    EXPECT_MF 9, 0x38a8
    fstp dword [res32]              ; overflow, then the pop
    mov eax, [res32]
    mov [st_r32], eax
    EXPECT_MF 10, 0x00a8
    fnstsw ax
    EXPECT_AX 0x3800, 0x0000
    fninit
    mov esi, st_i16
    mov edi, exp_store_exc
    mov ecx, exp_store_exc_end - exp_store_exc
    repe cmpsb
    jne fail

    ; ---- FNSTSW right after FCOM, FDIV, FSQRT ---------------------------------
    fld1
    fld qword [real_a]
    fcom st1                        ; a > 1
    fnstsw ax
    EXPECT_AX 0x4700, 0x0000
    fcom qword [real_a]             ; equal
    fnstsw ax
    EXPECT_AX 0x4700, 0x4000
    fxch
    fcom st1                        ; 1 < a
    fnstsw ax
    EXPECT_AX 0x4700, 0x0100
    fninit
    fld1
    fldz
    fdivp st1, st0                  ; 1/0: ZE (masked) when it finishes
    fnstsw ax
    EXPECT_AX 0x38ff, 0x3804
    fnclex
    fld qword [real_a]
    fdiv st0, st1                   ; inf / a: no exception
    fnstsw ax
    EXPECT_AX 0x38ff, 0x3000
    fninit
    fld qword [minus_one]
    fsqrt                           ; invalid (masked)
    fnstsw ax
    EXPECT_AX 0x38ff, 0x3801
    fnclex
    fld1
    fld qword [three]
    fdivp st1, st0                  ; 1/3: PE, rounded down
    fnstsw [res16]
    mov ax, [res16]
    EXPECT_AX 0x3aff, 0x3020
    fld qword [real_a]
    fdiv st0, st1                   ; then a compare after a divide
    fcomp st1
    fnstsw ax
    EXPECT_AX 0x4700, 0x0000        ; 3a > 1/3
    fninit

    ; ---- a mask or rounding change right before a store --------------------
    fld qword [two_seven]
    fldcw [cw_chop]
    fist dword [res32]              ; 2.7 -> 2
    cmp dword [res32], 2
    jne fail
    fldcw [cw_up]
    fist dword [res32]              ; -> 3
    cmp dword [res32], 3
    jne fail
    fnstcw [res16]                  ; right after the FLDCW
    cmp word [res16], 0x0b7f
    jne fail
    fldcw [cw_down]
    fnstcw [res16]
    cmp word [res16], 0x077f
    jne fail
    fld1
    fld qword [three]
    fdivp st1, st0
    fst dword [res32]               ; 1/3 rounded down
    cmp dword [res32], 0x3eaaaaaa
    jne fail
    fldcw [cw_up]
    fstp dword [res32]              ; 1/3 rounded up
    cmp dword [res32], 0x3eaaaaab
    jne fail
    fninit

    ; ---- FLDCW right before loads ------------------------------------------
    fninit
    fldcw [cw_chop]                 ; masked: no mask load to the next load
    fld qword [real_a]
    fldcw [cw_up]
    fldcw [cw_down]                 ; back to back
    fld tword [pi80]
    fldcw [cw_up]
    fild word [int16]
    fldcw [cw_default]
    fld dword [real_1_5]
    fnstcw [res16]
    cmp word [res16], 0x037f
    jne fail
    EXPECT e_1_5
    EXPECT e_i16
    EXPECT pi80
    EXPECT e_a
    fldcw [cw_unmask_ie]            ; unmasks IE: the next load falls back
    fld qword [real_a]
    fldcw [cw_default]              ; IE unmasked: falls back as well
    fld tword [pi80]
    EXPECT pi80
    EXPECT e_a
    fnstsw ax
    test ax, 0x38ff
    jnz fail

    ; ---- the C float-to-int sequence --------------------------------------
    mov esi, ftol_in
    mov edi, ftol_out
    mov ebx, 4
.ftol:
    fld qword [esi]
    fnstcw [cw_save]
    movzx eax, word [cw_save]
    or ax, 0x0c00
    mov [cw_new], ax
    fldcw [cw_new]
    fistp dword [res32]
    fldcw [cw_save]
    mov eax, [res32]
    cmp eax, [edi]
    jne fail
    add esi, 8
    add edi, 4
    dec ebx
    jnz .ftol
    fnstcw [res16]
    cmp word [res16], 0x037f
    jne fail

    ; ---- FCS:FOP and the operand offset of the last store --------------------
    fninit
    fld qword [real_a]
    fst qword [res64]
    fnstsw ax
    fnstcw [res16]
    fnstenv [env]
    cmp dword [env + 16], (0x515 << 16) | 0x0008
    jne fail
    cmp dword [env + 20], res64
    jne fail
    fistp word [res16]
    fnstenv [env]
    cmp dword [env + 16], (0x71d << 16) | 0x0008
    jne fail
    cmp dword [env + 20], res16
    jne fail
    fld tword [pi80]
    mov ebx, res80 - 4
    fstp tword [ebx + 4]
    fnstenv [env]
    cmp dword [env + 16], (0x37b << 16) | 0x0008
    jne fail
    cmp dword [env + 20], res80
    jne fail
    fninit

    mov al, 1
    out STATUS_PORT, al
    hlt

fail:
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 4
nm_count:       dd 0
mf_count:       dd 0
mf_sw:          dw 0
cw_unmask_ze:   dw 0x037b
cw_unmask_io:   dw 0x0376
cw_unmask_ie:   dw 0x037e
cw_default:     dw 0x037f
cw_chop:        dw 0x0f7f
cw_up:          dw 0x0b7f
cw_down:        dw 0x077f
cw_save:        dw 0
cw_new:         dw 0
align 16
real_a:         dq 0x3ff3c0ca428c59fb   ; 1.2345678901234567
minus_one:      dq -1.0
three:          dq 3.0
two_seven:      dq 2.7
real_1_5:       dd 1.5
int16:          dw -12345
align 16
e_a:            db 0x00, 0xd8, 0xcf, 0x62, 0x14, 0x52, 0x06, 0x9e, 0xff, 0x3f
e_1_5:          db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x3f
e_i16:          db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0xc0, 0x0c, 0xc0
big:            dq 1.0e10
pi80:           dq 0xc90fdaa22168c235
                dw 0x4000
qnan80:         dq 0xc000000000000000
                dw 0x7fff
huge80:         dq 0x8000000000000000
                dw 0x3fff + 2000
align 16
ftol_in:        dq 2.7, -2.7, 1000000000.75, -0.5
ftol_out:       dd 2, -2, 1000000000, 0
align 16
res16:          dw 0
align 4
res32:          dd 0
align 8
res64:          dq 0
align 16
res80:          times 10 db 0
align 16
env:            times 28 db 0

; What the stores with an unmasked exception wrote, in order, and what the
; original routine writes.
align 4
st_i16:         dw 0
st_i32:         dd 0
st_i64:         dq 0
st_bcd:         times 10 db 0
st_r64:         dq 0
st_sw64:        dw 0
st_r32:         dd 0
st_end:
exp_store_exc:
                dw 0x8000               ; integer indefinite
                dd 0x80000000
                dq 0x8000000000000000
                dq 0xc000000000000000   ; BCD indefinite
                dw 0xffff
                dq 0x7ff0000000000000   ; +inf
                dw 0xb8a8               ; B, ES, OE and PE, TOP 7
                dd 0x7f800000           ; +inf
exp_store_exc_end:

times 0x1000 - ($ - $$) db 0
