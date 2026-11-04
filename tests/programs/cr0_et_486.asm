; CR0.ET is hardwired to 1 on the Intel486: MOV CR0 with ET clear leaves it
; set (also when the testbench starts with it clear, as a 386 may), and
; SMSW, MOV from CR0, CLTS and LMSW all see it set afterwards.

BITS 16
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    mov eax, cr0
    and eax, 0xffffffef
    mov cr0, eax
    mov eax, cr0
    test al, 0x10
    jz fail
    smsw bx
    test bl, 0x10
    jz fail
    clts
    mov ecx, cr0
    test cl, 0x10
    jz fail
    mov ax, 0x0008                  ; TS set, ET clear in the word
    lmsw ax
    smsw dx
    and dx, 0x001f
    cmp dx, 0x0018
    jne fail
    mov eax, cr0
    and eax, 0xfffffff7             ; TS off again
    mov cr0, eax
    mov eax, cr0
    and eax, 0x1f
    cmp eax, 0x10
    jne fail

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $
