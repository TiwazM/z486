; call_gate16_ring2_ldt.asm - the ring 3 -> ring 2 call gate with its descriptors in the LDT
;
; As call_gate16_ring2 (below), with the gate, its ring-2 target and the ring-2 stack (SS2 in
; the TSS) in the LDT, as OS/2 keeps a process's selectors (DOSCALL1's DS 9817h, ES 981Fh,
; SS 980Fh at the fault: TI = 1).
;
; (call_gate16_ring2.asm:)
; OS/2 Warp 4 (2026-11-11, board): DOSCALL1.DLL's 16-bit code (object 3: a conforming
; IOPL segment, DPL 2, run at CPL 3) calls DOSFSRAMSEMREQUEST, a 286 call gate entry with 4
; parameter words into its IOPL object 1, which OS/2 runs at ring 2:
;     push ds / push 00C0h / push dword -1 / call far gate      (CS:IP E01F:419A)
; and that CALL faulted (c0000005, P2 = FFFFFFFF). Nothing else tests an inner level other
; than 0: the stack comes from the TSS's SS2:ESP2.
;
; Here: ring 3 code in a conforming DPL-2 16-bit segment (as object 3), a DPL-3 16-bit gate
; with 4 words to a DPL-2 16-bit code segment, SS0/SS1/SS2 all different (a wrong slot
; shows). The ring-2 routine records CS, SS, SP and the copied words; RETF 8 returns. Ring 3
; checks them and SS:SP; the test ends with a HLT at CPL 3, whose #GP passes. Any other fault
; fails with its vector, EIP and error code.
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
SEL_CODE2   equ 0x0C    ; LDT 1: ring 2 16-bit code (the gate's target, as object 1)
SEL_STACK2  equ 0x14    ; LDT 2: ring 2 16-bit stack (SS2)
SEL_CODEC   equ 0x48    ; conforming 16-bit code, DPL 2, run at CPL 3 (as object 3)
SEL_GATE    equ 0x1C    ; LDT 3: 16-bit call gate, DPL 3, 4 words
SEL_STACK1  equ 0x58    ; ring 1 stack (SS1), never used
SEL_LDT     equ 0x30

SEL_DATA3_RPL3 equ (SEL_DATA3 | 3)
SEL_CODEC_RPL3 equ (SEL_CODEC | 3)
SEL_GATE_RPL3  equ (SEL_GATE | 3)
SEL_STACK2_RPL2 equ (SEL_STACK2 | 2)
SEL_CODE2_RPL2 equ (SEL_CODE2 | 2)

STACK0_TOP  equ 0x3000
STACK1_TOP  equ 0x3800
STACK2_TOP  equ 0x5000
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
    mov ax, SEL_LDT
    lldt ax

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
    mov word [stage], 1
    mov ax, cs
    mov [r3_cs], ax
    push word 0x1111
    push word 0x2222
    push dword 0x33334444
call_site:
    call SEL_GATE_RPL3:0
    mov word [stage], 2
    cmp word [r3_cs], SEL_CODEC_RPL3
    jne r3_fail
    mov word [stage], 3
    cmp sp, STACK3_TOP          ; RETF 8 dropped the words
    jne r3_fail
    mov ax, ss
    cmp ax, SEL_DATA3_RPL3
    jne r3_fail
    mov word [stage], 4
    cmp word [seen_cs], SEL_CODE2_RPL2
    jne r3_fail
    mov word [stage], 5
    cmp word [seen_ss], SEL_STACK2_RPL2
    jne r3_fail
    mov word [stage], 6
    cmp word [seen_sp], STACK2_TOP - 16
    jne r3_fail
    mov word [stage], 7
    cmp word [seen_w + 0], 0x4444
    jne r3_fail
    cmp word [seen_w + 2], 0x3333
    jne r3_fail
    cmp word [seen_w + 4], 0x2222
    jne r3_fail
    cmp word [seen_w + 6], 0x1111
    jne r3_fail
    mov word [stage], 8
    cmp word [seen_w + 8], STACK3_TOP - 8      ; the caller's SP
    jne r3_fail
    cmp word [seen_w + 10], SEL_DATA3_RPL3     ; the caller's SS
    jne r3_fail
    cmp word [seen_ip], call_site + 5
    jne r3_fail
    cmp word [seen_ip + 2], SEL_CODEC_RPL3
    jne r3_fail

    mov word [stage], 0x10
done_site:
    hlt                         ; #GP at CPL 3: the handler passes
    jmp $

r3_fail:
    or word [stage], 0x80
    hlt                         ; #GP at CPL 3 with stage >= 0x80: fails
    jmp $

; Ring 2 routine (DPL-2 16-bit code), reached through the gate. DS is still the caller's.
gate_ring2:
    mov ax, cs
    mov [seen_cs], ax
    mov ax, ss
    mov [seen_ss], ax
    mov [seen_sp], sp
    mov bp, sp
    ; inner stack: +0 IP, +2 CS, +4..+10 the 4 words, +12 SP, +14 SS
    mov ax, [bp + 0]
    mov [seen_ip], ax
    mov ax, [bp + 2]
    mov [seen_ip + 2], ax
    mov ax, [bp + 4]
    mov [seen_w + 0], ax
    mov ax, [bp + 6]
    mov [seen_w + 2], ax
    mov ax, [bp + 8]
    mov [seen_w + 4], ax
    mov ax, [bp + 10]
    mov [seen_w + 6], ax
    mov ax, [bp + 12]
    mov [seen_w + 8], ax
    mov ax, [bp + 14]
    mov [seen_w + 10], ax
    retf 8

;------------------------------------------------------------------
; Ring 0 fault handlers (DPL0 386 interrupt gates)
;------------------------------------------------------------------
BITS 32
%macro FAULT_ERR 1
isr_%1:
    push dword %1
    jmp fault_common
%endmacro
FAULT_ERR 0x0A
FAULT_ERR 0x0B
FAULT_ERR 0x0C

isr_gp:
    ; frame after the pushes: [esp]=DS, +4 EAX, +8 err, +12 EIP, +16 CS
    push eax
    push ds
    mov ax, SEL_DATA0
    mov ds, ax
    movzx eax, word [esp+16]    ; CS
    cmp ax, SEL_CODEC_RPL3
    jne .other
    mov eax, [esp+12]           ; EIP
    cmp eax, done_site
    jne .other
    cmp word [stage], 0x10
    jne .other
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
.other:
    pop ds
    pop eax
    push dword 0x0D
    ; fall through

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
r3_cs:   dw 0
seen_cs: dw 0
seen_ss: dw 0
seen_sp: dw 0
seen_ip: dw 0, 0
seen_w:  dw 0, 0, 0, 0, 0, 0

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
    ; 0x30: LDT, base = ldt+0x10000
    dw ldt_end - ldt - 1
    dw ldt
    db 0x01
    db 0x82
    db 0x00
    db 0x00
    dq 0x0000000000000000       ; 0x38 unused
    dq 0x0000000000000000       ; 0x40 unused
    dq 0x0000DF010000FFFF       ; 0x48 conforming 16-bit code, DPL 2, base 0x10000
    dq 0x0000000000000000       ; 0x50 unused
    dq 0x0000B3010000FFFF       ; 0x58 ring 1 16-bit stack (SS1), base 0x10000
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
ldt:
    dq 0x0000000000000000       ; LDT 0 unused
    dq 0x0000DB010000FFFF       ; LDT 1 (0Ch): ring 2 16-bit code, base 0x10000
    dq 0x0000D3010000FFFF       ; LDT 2 (14h): ring 2 16-bit stack (B=0), base 0x10000
    ; LDT 3 (1Ch): 16-bit call gate, DPL 3, 4 words, target LDT 1 (0Ch):gate_ring2
    dw gate_ring2
    dw SEL_CODE2
    db 4
    db 0xE4
    dw 0
ldt_end:

%macro IGATE 1
    dw %1
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
%endmacro

align 8
idt:
    times 0x0A dq 0
    IGATE isr_0x0A
    IGATE isr_0x0B
    IGATE isr_0x0C
    IGATE isr_gp
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

; 386 TSS: three different inner stacks, no I/O bitmap
align 4
tss386:
    dd 0
    dd STACK0_TOP               ; +0x04 ESP0
    dd SEL_DATA0                ; +0x08 SS0
    dd STACK1_TOP               ; +0x0C ESP1
    dd SEL_STACK1 | 1           ; +0x10 SS1
    dd STACK2_TOP               ; +0x14 ESP2
    dd SEL_STACK2_RPL2          ; +0x18 SS2
    times 19 dd 0
    dw 0
    dw 104                      ; +0x66: I/O map base past the limit
tss386_end:
