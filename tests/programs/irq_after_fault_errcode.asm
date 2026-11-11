; irq_after_fault_errcode.asm - an external interrupt taken right after an exception
; with an error code pushes no error code of its own (i486 PRM 9.9)
;
; OS/2 Warp 4: a #PF through a trap gate (IF stays set), and the RTC's IRQ 8 taken at
; the #PF handler's first instruction. The IRQ pushed an error code too (16 bytes
; instead of 12): the sequencer's error-code flag of the #PF was still set, as only an
; instruction issue cleared it. The IRQ handler's IRETD then popped the #PF handler's
; EIP as CS: TRAP 000D.
;
; Here: INTR pending while IF=0, then STI and MOV DS with a selector past the GDT limit,
; in STI's shadow. #GP(58h) goes through a 32-bit trap gate; INTR is taken at the #GP
; handler's first instruction. The ISR checks its frame: EIP = the #GP handler's entry,
; CS, EFLAGS with IF, and ESP 12 bytes below the #GP frame. The #GP handler checks its
; error code and skips the MOV.
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 failure code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
SIGNAL_PORT equ 0xE8
SIGNAL_CYCLES_PORT equ 0xEC
SIGNAL_INSTR_PORT  equ 0xF0
SIGNAL_VECTOR_PORT equ 0xF4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

INTR_VECTOR equ 0x20

SEL_CODE0 equ 0x08
SEL_DATA0 equ 0x10
SEL_BAD   equ 0x58          ; past the GDT limit: #GP(58h)

PM_STACK_TOP equ 0x9000

start:
    cli
    cld
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov eax, cr0
    or  eax, 1
    mov cr0, eax
    db 0x66, 0xEA
    dd pm32_entry
    dw SEL_CODE0

BITS 32
pm32_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, PM_STACK_TOP

    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al
    mov dx, SIGNAL_INSTR_PORT
    xor eax, eax
    out dx, ax
    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 1
    out dx, ax

    ; INTR pending, masked
    mov dx, SIGNAL_PORT
    mov al, 3
    out dx, al
    mov ecx, 50
.settle:
    dec ecx
    jnz .settle

    mov [esp_before], esp
    mov ax, SEL_BAD
    sti
fault_insn:
    mov ds, ax                  ; #GP(58h) in STI's shadow
after_fault:
    cli
    mov ax, SEL_DATA0
    mov ds, ax

    mov eax, 0x10
    cmp dword [gp_count], 1
    jne fail
    mov eax, 0x11
    cmp dword [isr_count], 1
    jne fail
    mov eax, 0x12
    cmp dword [isr_eip], gp_handler
    jne fail
    mov eax, 0x13
    cmp dword [isr_cs], SEL_CODE0
    jne fail
    mov eax, 0x14
    test dword [isr_eflags], 0x200
    jz fail
    ; the IRQ frame sits right below the #GP frame (12 bytes, no error code)
    mov eax, 0x15
    mov ecx, [gp_esp]
    sub ecx, 12
    cmp [isr_esp], ecx
    jne fail
    mov eax, 0x16
    cmp esp, [esp_before]
    jne fail

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; #GP handler (32-bit trap gate). Its first instruction is where INTR is taken.
gp_handler:
    mov [gp_esp], esp
    inc dword [gp_count]
    mov eax, 0x20
    cmp dword [esp], SEL_BAD    ; error code
    jne fail
    mov eax, 0x21
    cmp dword [esp+4], fault_insn
    jne fail
    mov dword [esp+4], after_fault
    add esp, 4
    iretd

; INTR handler (32-bit interrupt gate)
isr_intr:
    mov [isr_esp], esp
    push eax
    mov eax, [esp+4]
    mov [isr_eip], eax
    mov eax, [esp+8]
    mov [isr_cs], eax
    mov eax, [esp+12]
    mov [isr_eflags], eax
    inc dword [isr_count]
    pop eax
    cmp dword [esp], gp_handler ; with an error code pushed, IRETD would go astray
    je .ok
    mov eax, 0x30
    jmp fail
.ok:
    iretd

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

BITS 16
align 8
gdt:
    dq 0x0000000000000000   ; null
    dq 0x00CF9B010000FFFF   ; ring 0 code, base=0x10000, 4GB
    dq 0x00CF93010000FFFF   ; ring 0 data, base=0x10000, 4GB
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times 0x0D dq 0
    ; vector 0Dh: 32-bit trap gate (IF stays set)
    dw gp_handler
    dw SEL_CODE0
    db 0
    db 10001111b
    dw 0
    times (INTR_VECTOR - 0x0E) dq 0
    ; vector 20h: 32-bit interrupt gate
    dw isr_intr
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

BITS 32
align 4
esp_before: dd 0
gp_count:   dd 0
gp_esp:     dd 0
isr_count:  dd 0
isr_esp:    dd 0
isr_eip:    dd 0
isr_cs:     dd 0
isr_eflags: dd 0
