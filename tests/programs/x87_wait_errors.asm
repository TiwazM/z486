; WAIT reporting an unmasked exception (16-bit code), as the original routine
; does: the in-place WAIT runs the routine for it.
;
; - CR0.NE=1: #MF at the WAIT, whether it holds behind the faulting divide or
;   the exception was pending before it issued. The handler sees the WAIT's
;   CS:IP and B, ES and ZE with TOP 6; ST0 keeps 1.0 (an unmasked divide by
;   zero writes nothing). After the handler's FNCLEX the restarted WAIT
;   completes.
; - CR0.NE=0 (+x87_irq13: FERR# raises IRQ13, port F0h asserts IGNNE#): no
;   #MF. A WAIT behind the faulting divide freezes and takes IRQ13 there (the
;   handler sees the WAIT's address); once the handler has written port F0h
;   the restarted WAIT completes with the exception still pending. With
;   IGNNE# already asserted (IRQ13 taken before), WAITs pass the pending
;   exception. FNCLEX ends FERR# and IGNNE#.
;
; The program passes on the original routine as well (fast paths off), but
; for the counts in the JSON.

BITS 16
org 0

STATUS_PORT  equ 0xE0
DATA_PORT    equ 0xE4
FPU_PORT     equ 0xF0               ; the AT's coprocessor port (+x87_irq13)
IRQ13_VECTOR equ 0x75
STATUS_PASS  equ 0x01
STATUS_FAIL  equ 0xFF

%macro PEND_ZE 0                    ; ZE unmasked: 1.0 / 0.0 leaves ZE pending
    fninit
    fldcw [cw_unmask_ze]
    fldz
    fld1
%endmacro

; #MF number %1 at CS:%2, with B, ES, ZE and TOP 6.
%macro EXPECT_MF 2
    cmp word [mf_count], %1
    jne fail
    cmp word [mf_ip], %2
    jne fail
    mov ax, cs
    cmp [mf_cs], ax
    jne fail
    mov ax, [mf_sw]
    and ax, 0xb8ff
    cmp ax, 0xb084
    jne fail
    fstp dword [res]
    cmp dword [res], 0x3f800000     ; ST0 was not overwritten
    jne fail
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
    mov word [es:16 * 4], mf_handler
    mov [es:16 * 4 + 2], cs
    mov word [es:IRQ13_VECTOR * 4], irq13_handler
    mov [es:IRQ13_VECTOR * 4 + 2], cs
    mov ax, cs
    mov es, ax

    ; ---- CR0.NE=1: #MF at a WAIT that holds behind the divide ----------
    mov word [step], 1
    mov eax, cr0
    or eax, 0x20
    mov cr0, eax
    PEND_ZE
    fdiv st0, st1
mf_wait1:
    fwait
    EXPECT_MF 1, mf_wait1
    fninit

    ; ---- the exception pending before the WAIT issues -----------------
    mov word [step], 2
    PEND_ZE
    fdiv st0, st1
    fnstsw ax                       ; no-wait: the finished divide's status
    test ax, 0x0080
    jz fail
    mov cx, 16
.idle:
    loop .idle
mf_wait2:
    fwait
    EXPECT_MF 2, mf_wait2
    fninit
    cmp word [irq13_count], 0
    jne fail

    ; ---- CR0.NE=0: FERR#, IRQ13 at the WAIT, IGNNE# ---------------------
    mov word [step], 3
    mov eax, cr0
    and eax, 0xffffffdf
    mov cr0, eax
    PEND_ZE
    sti
    fdiv st0, st1
irq_wait1:
    fwait
    cli
    cmp word [irq13_count], 1
    jne fail
    cmp word [irq13_ip], irq_wait1
    jne fail
    fnstsw ax
    test ax, 0x0080                 ; still pending: IGNNE# let it pass
    jz fail
    fwait                           ; IGNNE# still asserted
    fnclex                          ; FERR# and IGNNE# end
    fwait
    fnstsw ax
    test ax, 0x0080
    jnz fail

    ; ---- IRQ13 before the WAIT: IGNNE# lets WAITs pass ----------------
    mov word [step], 4
    PEND_ZE
    sti
    fdiv st0, st1
    mov cx, 200
.spin:
    loop .spin                      ; IRQ13 comes here
    cli
    cmp word [irq13_count], 2
    jne fail
    fwait
    fwait
    fnstsw ax
    test ax, 0x0080
    jz fail
    fninit
    fwait
    cmp word [mf_count], 2          ; never #MF with CR0.NE=0
    jne fail

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    mov dx, DATA_PORT
    mov ax, [step]
    out dx, ax
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

mf_handler:                         ; #MF: record, clear the exception
    push bp
    mov bp, sp
    push ax
    inc word [mf_count]
    mov ax, [bp + 2]
    mov [mf_ip], ax
    mov ax, [bp + 4]
    mov [mf_cs], ax
    fnstsw ax
    mov [mf_sw], ax
    fnclex
    pop ax
    pop bp
    iret

irq13_handler:                      ; IRQ13: record, IGNNE# through port F0h
    push bp
    mov bp, sp
    push ax
    inc word [irq13_count]
    mov ax, [bp + 2]
    mov [irq13_ip], ax
    xor al, al
    out FPU_PORT, al
    pop ax
    pop bp
    iret

align 4
res:            dd 0
cw_unmask_ze:   dw 0x037b
step:           dw 0
mf_count:       dw 0
mf_ip:          dw 0
mf_cs:          dw 0
mf_sw:          dw 0
irq13_count:    dw 0
irq13_ip:       dw 0
