; call_gate16_ring2_ts.asm - #TS from a gate CALL's ring-2 stack: the error code's EXT bit
;
; OS/2 Warp 4 (2026-11-12, whole-PC sim, then the board): OS/2 keeps an invalid SS2 in the
; TSS (0400h: RPL 0, a DPL-0 data segment, limit 0), so every 16-bit call gate from ring 3
; into its ring-2 IOPL code raises #TS(0400h), and the #TS handler switches to ring 2 by
; itself. z486 pushed 0401h: the gate CALL's microcode (5B9) sets the flag the fault path's
; JEXTFT (878) tested for EXT, so the CALL's own fault looked external. OS/2 did not
; recognise 0401h and stopped the program: "c0000005 DOSCALL1.DLL 0003:419A".
;
; EXT is 1 only for a fault while an exception or external interrupt is being delivered
; (486 PRM 9.9). Three cases, at increasing stages:
;   1. ring 3 CALL through a DPL-3 16-bit gate into DPL-2 code, SS2 = SEL_SS2BAD (RPL 0):
;      #TS with error code SEL_SS2BAD (EXT 0), frame at the CALL, from ring 3
;   2. INT 30h at CPL 0 with IDT[30h] not present: #NP(30h*8 + 2) = 0182h (EXT 0)
;   3. UD2 with IDT[6] not present: the #UD's delivery faults, #NP(6*8 + 2 + 1) = 0033h (EXT 1)
; Any other exception fails with its vector, stage, EIP and error code.
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE0   equ 0x08    ; ring 0 32-bit code
SEL_DATA0   equ 0x10    ; ring 0 data (SS0)
SEL_DATA3   equ 0x20    ; ring 3 data
SEL_TSS     equ 0x28
SEL_CODE2   equ 0x38    ; ring 2 16-bit code (the gate's target)
SEL_SS2BAD  equ 0x40    ; DPL-0 data, limit 0 (as OS/2's 0400h); in the TSS with RPL 0
SEL_CODEC   equ 0x48    ; conforming 16-bit code, DPL 2, run at CPL 3
SEL_GATE    equ 0x50    ; 16-bit call gate, DPL 3, 4 words

SEL_DATA3_RPL3 equ (SEL_DATA3 | 3)
SEL_CODEC_RPL3 equ (SEL_CODEC | 3)
SEL_GATE_RPL3  equ (SEL_GATE | 3)

STACK0_TOP  equ 0x3000
STACK3_TOP  equ 0x4000

start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov eax, cr0
    or eax, 1
    mov cr0, eax
    jmp dword SEL_CODE0:pm_entry

BITS 32
pm_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, STACK0_TOP
    mov ax, SEL_TSS
    ltr ax
    mov word [stage], 1

    ; IRET to ring 3, into the conforming DPL-2 segment (CPL stays 3 there)
    push dword SEL_DATA3_RPL3   ; SS3
    push dword STACK3_TOP       ; ESP3
    push dword 0x00000002       ; EFLAGS IOPL=0, IF=0
    push dword SEL_CODEC_RPL3
    push dword ring3_entry
    iretd

BITS 16
ring3_entry:
    mov ax, SEL_DATA3_RPL3
    mov ds, ax
    push word 0x1111
    push word 0x2222
    push dword 0x33334444
call_site:
    call SEL_GATE_RPL3:0        ; #TS(SEL_SS2BAD): SS2's RPL 0 is not the new CPL 2
    or word [stage], 0x80       ; the CALL went through: fail
    hlt
    jmp $

gate_ring2:                     ; never reached
    retf 8

;------------------------------------------------------------------
; Ring 0 handlers (DPL0 386 interrupt gates)
;------------------------------------------------------------------
BITS 32
isr_ts:
    ; [esp]=err, +4 EIP, +8 CS, +12 EFLAGS, +16 ESP3, +20 SS3
    push ds
    push eax
    mov ax, SEL_DATA0
    mov ds, ax
    cmp word [stage], 1
    jne .bad
    cmp dword [esp+8], SEL_SS2BAD       ; error code: the selector, EXT 0
    jne .bad
    cmp dword [esp+12], call_site
    jne .bad
    cmp word [esp+16], SEL_CODEC_RPL3
    jne .bad
    cmp dword [esp+24], STACK3_TOP - 8  ; the caller's SP, nothing pushed yet
    jne .bad
    cmp word [esp+28], SEL_DATA3_RPL3
    jne .bad

    ; stage 2: a software INT through a not-present gate
    mov esp, STACK0_TOP
    mov ax, SEL_DATA0
    mov es, ax
    mov word [stage], 2
    int 0x30
    or word [stage], 0x80
    jmp fail_here
.bad:
    pop eax
    pop ds
    push dword 0x0A
    jmp fault_common

isr_np:
    ; [esp]=err, +4 EIP, +8 CS, +12 EFLAGS (same level)
    push ds
    push eax
    mov ax, SEL_DATA0
    mov ds, ax
    cmp word [stage], 2
    jne .stage3
    cmp dword [esp+8], (0x30 << 3) | 2  ; IDT, EXT 0
    jne .bad
    ; stage 3: an exception through a not-present gate
    mov esp, STACK0_TOP
    mov word [stage], 3
.ud_site:
    ud2
    or word [stage], 0x80
    jmp fail_here
.stage3:
    cmp word [stage], 3
    jne .bad
    cmp dword [esp+8], (6 << 3) | 2 | 1 ; IDT, EXT 1: during the #UD's delivery
    jne .bad
    cmp dword [esp+12], .ud_site
    jne .bad
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
.bad:
    pop eax
    pop ds
    push dword 0x0B
    jmp fault_common

%macro FAULT_ERR 1
isr_%1:
    push dword %1
    jmp fault_common
%endmacro
%macro FAULT_NOERR 1
isr_%1:
    push dword 0
    push dword %1
    jmp fault_common
%endmacro
FAULT_NOERR 0x00
FAULT_NOERR 0x01
FAULT_NOERR 0x03
FAULT_NOERR 0x04
FAULT_NOERR 0x05
FAULT_NOERR 0x07
FAULT_ERR 0x08
FAULT_ERR 0x0C
FAULT_ERR 0x0D
FAULT_ERR 0x0E

fail_here:
    push dword 0
    push dword 0xEE

fault_common:
    ; [esp]=vector, +4 err, +8 EIP
    mov ax, SEL_DATA0
    mov ds, ax
    mov eax, [esp+8]            ; EIP
    shl eax, 16
    mov al, [esp]               ; vector
    mov ah, [stage]
    mov dx, DATA_PORT
    out dx, eax
    mov eax, [esp+4]            ; error code
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt

;==================================================================
; Data
;==================================================================
align 4
stage:   dw 0

align 8
gdt:
    dq 0x0000000000000000
    dq 0x00CF9B010000FFFF       ; 0x08 ring 0 32-bit code, base 0x10000
    dq 0x00CF93010000FFFF       ; 0x10 ring 0 data, base 0x10000
    dq 0x0000000000000000       ; 0x18 unused
    dq 0x00CFF3010000FFFF       ; 0x20 ring 3 data, base 0x10000
    ; 0x28: available 386 TSS (type 9), base = tss386+0x10000, limit 0x67
    dw 0x0067
    dw tss386
    db 0x01
    db 0x89
    db 0x00
    db 0x00
    dq 0x0000000000000000       ; 0x30 unused
    dq 0x0000DB010000FFFF       ; 0x38 ring 2 16-bit code, base 0x10000
    dq 0x0000920000000000       ; 0x40 DPL-0 data, base 0, limit 0 (OS/2's 0400h)
    dq 0x0000DF010000FFFF       ; 0x48 conforming 16-bit code, DPL 2, base 0x10000
    ; 0x50: 16-bit call gate, DPL 3, 4 words, target 0x38:gate_ring2
    dw gate_ring2
    dw SEL_CODE2
    db 4
    db 0xE4
    dw 0
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

%macro IGATE 1
    dw %1
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
%endmacro
%macro IGATE_NP 0
    dw 0
    dw SEL_CODE0
    db 0
    db 00001110b                ; P = 0
    dw 0
%endmacro

align 8
idt:
    IGATE isr_0x00
    IGATE isr_0x01
    IGATE_NP                    ; 2 NMI, unused
    IGATE isr_0x03
    IGATE isr_0x04
    IGATE isr_0x05
    IGATE_NP                    ; 6 #UD: not present (stage 3)
    IGATE isr_0x07
    IGATE isr_0x08
    IGATE_NP                    ; 9
    IGATE isr_ts                ; 0A #TS
    IGATE isr_np                ; 0B #NP
    IGATE isr_0x0C
    IGATE isr_0x0D
    IGATE isr_0x0E
    times (0x30 - 0x0F) dq 0
    IGATE_NP                    ; 30h: not present (stage 2)
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

; 386 TSS: SS0:ESP0 valid, SS2 = SEL_SS2BAD with RPL 0, ESP2 0 (as OS/2), no I/O bitmap
align 4
tss386:
    dd 0
    dd STACK0_TOP               ; +0x04 ESP0
    dd SEL_DATA0                ; +0x08 SS0
    dd 0                        ; +0x0C ESP1
    dd 0                        ; +0x10 SS1
    dd 0                        ; +0x14 ESP2
    dd SEL_SS2BAD               ; +0x18 SS2 (RPL 0)
    times 19 dd 0
    dw 0
    dw 104                      ; +0x66: I/O map base past the limit
tss386_end:
