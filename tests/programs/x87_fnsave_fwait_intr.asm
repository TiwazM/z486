; x87_fnsave_fwait_intr.asm - Linux's FNSAVE; FWAIT; FRSTOR with INTR swept across it.
;
; CorePlus (Linux 6.18, i386, no FXSR) hung on the board with CS:EIP 0060:C013C700
; in nearly every sample: `fnsave [esi]` at C013C6FE had issued, the `fwait` at
; C013C700 (the start of a new 16-byte code line) not. OS/2's freeze had the same
; shape (an instruction ending on a line boundary, then nothing). Here FNSAVE sits
; at offset 0EFEh so its FWAIT starts the next line, and for every delay 1..DMAX the
; bench raises INTR that many cycles after the OUT that arms it. Each round the
; interrupt must be taken once and ST0 = 1.0, ST1 = 2.0 must come back from FRSTOR.
; A lockup is the bench's TIMEOUT.

BITS 16
org 0

STATUS_PORT  equ 0xE0
DATA_PORT    equ 0xE4
SIGNAL_PORT  equ 0xE8
DELAY_PORT   equ 0xEC
VECTOR_PORT  equ 0xF4
STATUS_PASS  equ 0x01
STATUS_FAIL  equ 0xFF
INTR_VECTOR  equ 0x20
DMAX         equ 300
SEL_CODE     equ 0x08
SEL_DATA     equ 0x10
STACK_TOP    equ 0x9000
SAVE         equ 0x8000             ; 108-byte image (segment base 10000h)

start:
    cli
    cld
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    xor eax, eax
    mov cr3, eax
    mov eax, cr0
    or eax, 0x80000023              ; PG, NE (as Linux), MP, PE
    mov cr0, eax
    db 0x66, 0xEA
    dd pm32
    dw SEL_CODE

BITS 32
pm32:
    mov ax, SEL_DATA
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, STACK_TOP
    mov dx, VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al

    mov ebx, 1                      ; ebx = the delay
.next:
    fninit
    fld dword [two]
    fld1                            ; ST0 = 1.0, ST1 = 2.0
    mov dword [intr_count], 0
    mov dx, DELAY_PORT
    mov eax, ebx
    out dx, ax
    mov dx, SIGNAL_PORT
    mov al, 1
    sti
    out dx, al                      ; INTR ebx cycles from here
    call do_save
    mov ecx, 400
.wait:
    cmp dword [intr_count], 0
    jne .taken
    loop .wait
.taken:
    cli
    cmp dword [intr_count], 1
    mov cl, 1
    jne fail
    fld1
    fcomp st1                       ; 1.0 vs ST0
    fnstsw ax
    and ah, 0x45
    cmp ah, 0x40
    mov cl, 2
    jne fail
    fld dword [two]
    fcomp st2                       ; 2.0 vs ST1
    fnstsw ax
    and ah, 0x45
    cmp ah, 0x40
    mov cl, 3
    jne fail
    cmp word [SAVE], 0x037F         ; the saved control word
    mov cl, 4
    jne fail
    inc ebx
    cmp ebx, DMAX
    jbe .next

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail:                               ; cl = check, ebx = delay
    mov eax, ebx
    shl eax, 8
    mov al, cl
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

isr:
    inc dword [intr_count]
    iretd

align 4
intr_count: dd 0
two:        dd 0x40000000           ; 2.0f

BITS 16
align 8
gdt:
    dq 0
    dq 0x00CF9B010000FFFF           ; 08 code32, base 10000h
    dq 0x00CF93010000FFFF           ; 10 data32, base 10000h
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times INTR_VECTOR dq 0
    dw isr
    dw SEL_CODE
    db 0
    db 10001110b
    dw 0
idt_end:
idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

BITS 32
; FNSAVE ends at the end of a 16-byte line (as at C013C6FE), FWAIT starts the next.
    times 0x0EF0 - ($ - $$) db 0xCC
do_save:
    mov esi, SAVE                   ; 0EF0
    mov eax, [esi + 200]            ; 0EF5 a load before, as kernel code has
    nop                             ; 0EFB
    nop
    nop
    fnsave [esi]                    ; 0EFE  dd 36
    fwait                           ; 0F00  9b
    frstor [esi]                    ; 0F01  dd 26
    ret
