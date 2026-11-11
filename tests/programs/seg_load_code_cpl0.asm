; seg_load_code_cpl0.asm - DS/ES/FS loads of code segments at CPL 0 (i486 PRM 6.3.x)
;
; A readable code segment may be loaded into DS, ES, FS or GS; an execute-only one
; raises #GP(selector). OS/2 Warp 4's kernel relies on both: 16-bit kernel code does
; PUSH CS / POP DS, and its fault handler reads a faulting instruction's opcode with
; LDS EDI, [frame CS:EIP] after making the code segment readable with an OR to the
; descriptor's access byte, the instruction right before the LDS.
;
; Each case records the vector it raised (0 if none); the handlers resume at the
; case's end with CS = SEL_CODE0. The table of vectors is compared at the end.
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 code
; (case << 16 | got << 8 | expected).

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE0   equ 0x08    ; 32-bit code, DPL 0
SEL_DATA0   equ 0x10
SEL_RCODE16 equ 0x18    ; 16-bit readable code, DPL 0, limit D0B5h (as OS/2's 0148h)
SEL_XCODE16 equ 0x20    ; 16-bit execute-only code, DPL 0

STACK0_TOP  equ 0x3000

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
    mov fs, ax
    mov gs, ax                  ; GS stays DATA0: results
    mov ss, ax
    mov esp, STACK0_TOP

; %1 case, %2 the label the case ends at (a fault resumes there)
%macro BEGIN 2
    mov byte [gs:cur], %1
    mov dword [gs:resume], %2
    mov [gs:saved_esp], esp
%endmacro
%macro END 1
%1:
    mov esp, [gs:saved_esp]
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov fs, ax
%endmacro

    BEGIN 0, end0                 ; MOV DS, readable code: ok
    mov ax, SEL_RCODE16
    mov ds, ax
    END end0

    BEGIN 1, end1                 ; MOV ES, readable code: ok
    mov ax, SEL_RCODE16
    mov es, ax
    END end1

    BEGIN 2, end2                 ; POP DS, readable code: ok
    push dword SEL_RCODE16
    pop ds
    END end2

    BEGIN 3, end3                 ; LDS, readable code: ok
    lds edi, [fp_rcode]
    END end3

    BEGIN 4, end4                 ; LES, readable code: ok
    les edi, [fp_rcode]
    END end4

    BEGIN 5, end5                 ; OS/2's sequence on a readable code descriptor: ok
    mov ebx, gdt + SEL_RCODE16
    push dword SEL_RCODE16
    push dword 0x1234
    mov al, [ebx+5]
    push eax
    or byte [ebx+5], 2
    lds edi, [esp+4]
    END end5

    BEGIN 6, end6                 ; the same on an execute-only one: the OR makes it readable: ok
    mov ebx, gdt + SEL_XCODE16
    push dword SEL_XCODE16
    push dword 0x1234
    mov al, [ebx+5]
    push eax
    or byte [ebx+5], 2
    lds edi, [esp+4]
    END end6
    and byte [gs:gdt + SEL_XCODE16 + 5], 0xFD   ; execute-only again

    BEGIN 7, end7                 ; LDS, execute-only code: #GP
    lds edi, [fp_xcode]
    END end7

    BEGIN 8, end8                 ; MOV DS, execute-only code: #GP
    mov ax, SEL_XCODE16
    mov ds, ax
    END end8

    BEGIN 9, end9                 ; LFS, readable code: ok
    lfs edi, [fp_rcode]
    END end9

    BEGIN 10, end10               ; 16-bit code: PUSH CS / POP DS / PUSH CS / POP ES: ok
    call SEL_RCODE16:code16_pushcs
    END end10

    BEGIN 11, end11               ; 16-bit code: LDS from its own segment: ok
    call SEL_RCODE16:code16_lds
    END end11

    BEGIN 12, end12               ; LDS with the running 32-bit code segment: ok
    lds edi, [fp_code0]
    END end12

    mov byte [gs:cur], 0xFF

    ; compare
    xor esi, esi
.cmp:
    mov al, [gs:got + esi]
    cmp al, [gs:want + esi]
    jne .fail
    inc esi
    cmp esi, NCASE
    jb .cmp
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    jmp $
.fail:
    mov eax, esi
    shl eax, 8
    mov al, [gs:got + esi]
    shl eax, 8
    mov al, [gs:want + esi]
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    jmp $

BITS 16
code16_pushcs:
    push cs
    pop ds
    push cs
    pop es
    mov ax, [cs_word]           ; a read through DS = the code segment
    cmp ax, 0xC0DE
    jne .bad
    o32 retf
.bad:
    mov byte [gs:got + 10], 0xEE
    o32 retf

code16_lds:
    push cs
    pop ds
    lds si, [fp16]              ; DS:SI <- RCODE16:5678h
    o32 retf

cs_word:
    dw 0xC0DE
fp16:
    dw 0x5678, SEL_RCODE16

BITS 32
; Fault handlers: record the vector, resume at the case's end in CS = SEL_CODE0
%macro FH 1
isr_%1:
    push eax
    mov al, %1
    jmp fault_common
%endmacro
FH 0x0B
FH 0x0C
FH 0x0D

fault_common:
    ; [esp]=EBX, +4 EAX, +8 err, +12 EIP, +16 CS
    push ebx
    movzx ebx, byte [gs:cur]
    cmp ebx, NCASE
    jae .bad
    mov [gs:got + ebx], al
    mov ebx, [gs:resume]
    mov [esp+12], ebx
    mov dword [esp+16], SEL_CODE0
    pop ebx
    pop eax
    add esp, 4
    iretd
.bad:
    shl eax, 8
    mov al, 0xEE
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt

;==================================================================
NCASE equ 13
cur:
    db 0
got:
    times NCASE db 0
want:
    db 0, 0, 0, 0, 0, 0, 0, 13, 13, 0
    db 0, 0, 0
align 4
resume:
    dd 0
saved_esp:
    dd 0
fp_rcode:
    dd 0x1234
    dw SEL_RCODE16
fp_xcode:
    dd 0x1234
    dw SEL_XCODE16
fp_code0:
    dd 0x1234
    dw SEL_CODE0

align 8
gdt:
    dq 0x0000000000000000
    dq 0x00CF9B010000FFFF       ; 0x08 code0, 32-bit
    dq 0x00CF93010000FFFF       ; 0x10 data0
    dq 0x00009B010000D0B5       ; 0x18 16-bit readable code, DPL 0, limit D0B5h
    dq 0x000099010000D0B5       ; 0x20 16-bit execute-only code, DPL 0
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

%macro IG 1
    dw %1
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
%endmacro

align 8
idt:
    times 0x0B dq 0
    IG isr_0x0B
    IG isr_0x0C
    IG isr_0x0D
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000
