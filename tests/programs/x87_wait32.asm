; WAIT (9B) in 32-bit protected-mode code.
;
; - With the x87 idle WAIT completes like a NOP (32 in under 3 cycles each);
;   behind FDIV it holds until the divide is done.
; - FWAIT before every FPU instruction leaves the results the same code
;   without the FWAITs leaves.
; - CR0.NE=1: an unmasked divide by zero is reported at the WAIT behind it:
;   #MF with the WAIT's CS:EIP, B, ES and ZE, TOP 6; ST0 keeps 1.0. After
;   FNCLEX the restarted WAIT completes.
; - CR0.NE=0 (+x87_irq13, IRQ13 at vector 21h): FERR# raises IRQ13 at that
;   WAIT; the handler writes port F0h (IGNNE#) and the restarted WAIT
;   completes with the exception pending, until FNCLEX.
; - INTR during a short wait (FDIV) comes after the WAIT; during a long one
;   (FSIN) the original routine takes it at the WAIT and waits FSIN out.
; - CR0.MP and TS: #NM at the WAIT.

BITS 32
ORG 0

STATUS_PORT        equ 0xE0
DATA_PORT          equ 0xE4
SIGNAL_PORT        equ 0xE8
SIGNAL_CYCLES_PORT equ 0xEC
SIGNAL_VECTOR_PORT equ 0xF4
CYCLE_PORT         equ 0xFC
FPU_PORT           equ 0xF0         ; the AT's coprocessor port (+x87_irq13)
CODE_BASE          equ 0x00010000
INTR_VECTOR        equ 0x20

%macro STAMP 1                      ; %1 = the cycle counter
    mov dx, CYCLE_PORT
    in eax, dx
    mov %1, eax
%endmacro

%macro INTR_IN 1                    ; INTR %1 cycles from now
    mov dx, SIGNAL_CYCLES_PORT
    mov ax, %1
    out dx, ax
    mov dx, SIGNAL_PORT
    mov al, 1
    out dx, al
%endmacro

%macro PEND_ZE 0                    ; ZE unmasked: 1.0 / 0.0 leaves ZE pending
    fninit
    fldcw [cw_unmask_ze]
    fldz
    fld1
%endmacro

; %1 bytes at res and ref must be equal.
%macro SAME 1
    mov esi, res
    mov edi, ref
    mov ecx, %1
    repe cmpsb
    jne fail
%endmacro

; With and without an FWAIT before every FPU instruction (FW). Results at
; %1 (19 bytes): m64, m32 integer, m32, status word, the branch taken.
%macro COMPILED 1
    FW
    fninit
    FW
    fld qword [val_a]
    FW
    fmul qword [val_b]
    FW
    fadd qword [val_c]
    FW
    fld st0
    FW
    fsqrt
    FW
    fdivp st1, st0
    FW
    fild word [val_i16]
    FW
    fmulp st1, st0
    FW
    fld dword [val_d]
    FW
    fcom st1
    FW
    fnstsw ax
    sahf
    mov [%1 + 16], ax
    jb %%below
    mov byte [%1 + 18], 1
    jmp %%done
%%below:
    mov byte [%1 + 18], 2
%%done:
    FW
    fxch st1
    FW
    fistp dword [%1 + 8]
    FW
    fstp qword [%1]
    FW
    fild dword [val_i32]
    FW
    fchs
    FW
    fstp dword [%1 + 12]
    FW
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

%macro GATE 1
    dw %1, 0x0008
    db 0, 0x8e
    dw 0
%endmacro

align 8
idt:
    times 7 dq 0
    GATE nm_handler                 ; 7
    times 8 dq 0
    GATE mf_handler                 ; 16
    times 15 dq 0
    GATE isr                        ; 20h
    GATE irq13_handler              ; 21h
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd CODE_BASE + idt

nm_handler:                         ; #NM: record where, clear TS
    push eax
    inc dword [nm_count]
    mov eax, [esp + 4]
    mov [nm_eip], eax
    clts
    pop eax
    iretd

mf_handler:                         ; #MF: record, clear the exception
    push eax
    inc dword [mf_count]
    mov eax, [esp + 4]
    mov [mf_eip], eax
    mov eax, [esp + 8]
    mov [mf_cs], eax
    fnstsw ax
    mov [mf_sw], ax
    fnclex
    pop eax
    iretd

isr:                                ; INTR: record the return address
    push eax
    inc dword [intr_count]
    mov eax, [esp + 4]
    mov [intr_eip], eax
    pop eax
    iretd

irq13_handler:                      ; IRQ13: record, IGNNE# through port F0h
    push eax
    inc dword [irq13_count]
    mov eax, [esp + 4]
    mov [irq13_eip], eax
    xor al, al
    out FPU_PORT, al
    pop eax
    iretd

times 0x400 - ($ - $$) db 0x90
start:
    cli
    cld
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x8000
    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al

    ; ---- the x87 idle: WAIT as a NOP ----------------------------------
    mov dword [step], 1
    fninit
    STAMP ebx
    STAMP ecx
    sub ecx, ebx
    STAMP ebx
    times 32 fwait
    STAMP eax
    sub eax, ebx
    sub eax, ecx
    cmp eax, 3 * 32
    jae fail

    ; ---- behind FDIV ------------------------------------------------------
    mov dword [step], 2
    fninit
    fld qword [val_y]
    fld qword [val_x]
    fdiv st0, st1
    fstp qword [ref]
    fninit
    fld qword [val_y]
    fld qword [val_x]
    STAMP ebx
    fdiv st0, st1
    fwait
    STAMP eax
    sub eax, ebx
    cmp eax, 60
    jb fail
    fstp qword [res]
    SAME 8

    ; ---- FWAIT before every FPU instruction ----------------------------
    mov dword [step], 3
%define FW fwait
    COMPILED res_w
%undef FW
%define FW
    COMPILED res_n
%undef FW
    mov esi, res_w
    mov edi, res_n
    mov ecx, 19
    repe cmpsb
    jne fail
    cmp byte [res_w + 18], 2
    jne fail
    cmp dword [res_w + 8], 14
    jne fail

    ; ---- CR0.NE=1: #MF at the WAIT behind the divide ----------------------
    mov dword [step], 4
    mov eax, cr0
    or eax, 0x20
    mov cr0, eax
    PEND_ZE
    fdiv st0, st1
mf_wait:
    fwait
    cmp dword [mf_count], 1
    jne fail
    cmp dword [mf_eip], mf_wait
    jne fail
    cmp word [mf_cs], 0x0008
    jne fail
    mov ax, [mf_sw]
    and ax, 0xb8ff
    cmp ax, 0xb084                  ; B, TOP 6, ES, ZE
    jne fail
    fstp dword [res]
    cmp dword [res], 0x3f800000     ; ST0 was not overwritten
    jne fail
    fninit

    ; ---- CR0.NE=0: IRQ13 at the WAIT behind the divide ---------------------
    mov dword [step], 5
    mov eax, cr0
    and eax, 0xffffffdf
    mov cr0, eax
    PEND_ZE
    sti
    fdiv st0, st1
irq_wait:
    fwait
    cli
    cmp dword [irq13_count], 1
    jne fail
    cmp dword [irq13_eip], irq_wait
    jne fail
    fnstsw ax
    test ax, 0x0080
    jz fail
    fnclex
    fwait
    cmp dword [mf_count], 1
    jne fail

    ; ---- INTR during a short wait and a long one ------------------------
    mov dword [step], 6
    fninit
    fld qword [val_y]
    fld qword [val_x]
    sti
    INTR_IN 30
    fdiv st0, st1
    fwait
after_short:
    cli
    cmp dword [intr_count], 1
    jne fail
    cmp dword [intr_eip], after_short
    jne fail
    fstp qword [res]
    SAME 8

    mov dword [step], 7
    fninit
    fld qword [val_x]
    fsin
    fstp qword [ref]
    fninit
    fld qword [val_x]
    sti
    INTR_IN 200
    fsin
wait_long:
    fwait
    cli
    cmp dword [intr_count], 2
    jne fail
    cmp dword [intr_eip], wait_long
    jne fail
    fstp qword [res]
    SAME 8

    ; ---- CR0.MP and TS: #NM -------------------------------------------------
    mov dword [step], 8
    fninit
    mov eax, cr0
    or eax, 0x0a
    mov cr0, eax
    mov ecx, 3
nm_wait:
    fwait
    cmp dword [nm_count], 1
    jne fail
    cmp dword [nm_eip], nm_wait
    jne fail
    fwait

    mov al, 1
    out STATUS_PORT, al
    hlt
    jmp $

fail:
    mov eax, [step]
    out DATA_PORT, eax
    mov al, 0xff
    out STATUS_PORT, al
    hlt
    jmp $

align 8
val_x:          dq 0.7
val_y:          dq 2.5
val_a:          dq 1.5
val_b:          dq 2.25
val_c:          dq 0.75
val_d:          dd 3.0
val_i32:        dd -123456
val_i16:        dw 7
cw_unmask_ze:   dw 0x037b
align 4
ref:            dq 0
res:            dq 0
res_w:          times 20 db 0
res_n:          times 20 db 0xff
step:           dd 0
nm_count:       dd 0
nm_eip:         dd 0
mf_count:       dd 0
mf_eip:         dd 0
mf_cs:          dd 0
mf_sw:          dw 0, 0
intr_count:     dd 0
intr_eip:       dd 0
irq13_count:    dd 0
irq13_eip:      dd 0
