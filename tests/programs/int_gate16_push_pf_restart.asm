; int_gate16_push_pf_restart.asm - an INT through a 386 gate to a 16-bit code
; segment whose frame push faults restarts at the INT's full 32-bit EIP
;
; Ring 3 runs in a flat 32-bit code segment at offsets above 0FFFFh and
; executes INT 31h through a DPL 3 interrupt gate to a 16-bit DPL 3 code
; segment (no stack switch), as DJGPP calls CWSDPMI. The 12-byte frame push
; falls into a not-present stack page. By then the INT has loaded the
; handler's CS descriptor (D=0) into the cache; the #PF must still name the
; INT at its full EIP (z486 restored TMPeIP masked by that D: the restart went
; to the low 16 bits and DJGPP's TEST27 died at eip=00000de1). The ring-0
; #PF handler maps the page; the restarted INT reaches the 16-bit handler,
; whose IRETD returns after the INT.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
CODE_BASE   equ 0x00010000
USTACK      equ 0x00030000                  ; user stack page, top at +0x1000
USTACK_PTE  equ 0x00001000 + (USTACK >> 12) * 4
KSTACK_TOP  equ 0x00038000

SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
SEL_CODE3   equ 0x18
SEL_DATA3   equ 0x20
SEL_TSS     equ 0x28
SEL_CODE16  equ 0x30

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff                   ; 0x08: ring 0 code, base 0x10000
    dq 0x00cf93000000ffff                   ; 0x10: ring 0 data, base 0
    dq 0x00cffb000000ffff                   ; 0x18: ring 3 code, flat
    dq 0x00cff3000000ffff                   ; 0x20: ring 3 data, flat
    dw 0x0067                               ; 0x28: 386 TSS
    dw tss
    db 0x01
    db 0x89
    db 0x00
    db 0x00
    dq 0x0000fb010000ffff                   ; 0x30: ring 3 16-bit code, base 0x10000
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_BASE + gdt

align 8
idt:
    times 14 dq 0
    dw pf_handler                           ; 0x0E: #PF, ring 0
    dw SEL_CODE0
    db 0
    db 0x8e
    dw 0
    times 0x31 - 15 dq 0
    dw handler16                            ; 0x31: 386 interrupt gate, DPL 3,
    dw SEL_CODE16                           ;       to the 16-bit segment
    db 0
    db 0xee
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd CODE_BASE + idt

align 4
tss:
    dd 0
    dd KSTACK_TOP                           ; ESP0
    dd SEL_DATA0                            ; SS0
    times 23 dd 0

align 4
pf_count:  dd 0
h16_count: dd 0

pf_handler:
    ; frame: [esp]=error code, +4 EIP, +8 CS, +12 EFLAGS, +16 ESP3, +20 SS3
    push eax
    push ds
    mov ax, SEL_DATA0
    mov ds, ax
    inc dword [CODE_BASE + pf_count]
    mov eax, 6
    cmp dword [CODE_BASE + pf_count], 1
    jne report                              ; one fault only
    mov eax, 7
    cmp dword [esp + 16], SEL_CODE3 | 3     ; the INT's own CS
    jne report
    mov eax, 8
    cmp dword [esp + 12], CODE_BASE + int1  ; the INT's own full EIP
    jne report
    mov eax, 9
    cmp dword [esp + 24], USTACK + 0x1000   ; the INT's own ESP
    jne report
    or dword [USTACK_PTE], 1
    pop ds
    pop eax
    add esp, 4                              ; error code
    iretd
report:
    out DATA_PORT, eax
    mov al, 0xff
    out STATUS_PORT, al
    jmp $

BITS 16
handler16:
    a32 inc dword [CODE_BASE + h16_count]
    o32 iret
BITS 32

times 0x300 - ($ - $$) db 0x90
start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, KSTACK_TOP
    mov ax, SEL_TSS
    ltr ax
    and dword [USTACK_PTE], 0xFFFFFFFE
    invlpg [USTACK]
    push dword SEL_DATA3 | 3                ; SS3
    push dword USTACK + 0x1000              ; ESP3
    push dword 0x00003002                   ; EFLAGS, IOPL 3
    push dword SEL_CODE3 | 3                ; CS3
    push dword CODE_BASE + ring3            ; flat: EIP above 0FFFFh
    iretd

ring3:
    mov ax, SEL_DATA3 | 3
    mov ds, ax
    mov es, ax
int1:
    int 0x31
    mov edx, 1
    cmp esp, USTACK + 0x1000
    jne fail
    mov edx, 2
    cmp dword [CODE_BASE + h16_count], 1
    jne fail
    mov edx, 3
    cmp dword [CODE_BASE + pf_count], 1
    jne fail
    mov edx, 4
    mov eax, [USTACK + 0xffc]               ; the frame's EFLAGS
    and eax, 0xFFFEFFFF                     ; RF: the INT ran again after the #PF
    cmp eax, 0x00003002
    jne fail
    mov edx, 5
    cmp dword [USTACK + 0xff4], CODE_BASE + int1 + 2
    jne fail
    mov al, 1
    out STATUS_PORT, al
    jmp $

fail:
    mov eax, edx
    out DATA_PORT, eax
    mov al, 0xff
    out STATUS_PORT, al
    jmp $

times 0x400 - ($ - $$) db 0
