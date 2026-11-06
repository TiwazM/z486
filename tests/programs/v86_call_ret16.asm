; v86_call_ret16.asm - 16-bit near CALL and RET in VM86 mode
;
; A 16-bit RET in VM86 mode (16-bit SS and CS, limits FFFFh) is a direct load
; of its target, as in real mode; a 16-bit CALL rel16 is a hardwired branch
; with its ESP commit. The VM86 code checks the return address on the stack,
; SP and IP wrapping at 64K, RET imm16, odd SPs on both sides of the
; SP[1:0] = 3 split, recursion deeper than the return-address stack, a RET
; whose return address was changed, and a loop of calls. It reports with
; INT 22h (pass) or INT 23h (fail, the code in AX).
;
; Setup as v86_int_stack_reuse.asm: protected mode, a USE16 ring-0 monitor,
; IRETD into VM86 with IOPL=3.
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

VEC_DONE    equ 0x22
VEC_FAIL    equ 0x23

SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
SEL_TSS     equ 0x18

STACK0_TOP  equ 0x7000
VM86_SEG    equ 0x1000
VM86_SP     equ 0xE100
WRAP_SEG    equ 0x3000          ; 30000h-3FFFFh: code at both ends of a segment

; Fail with code %2 if condition %1 holds (cmp/test before it).
%macro FAILIF 2
    j%-1 %%ok
    mov ax, %2
    int VEC_FAIL
%%ok:
%endmacro

start:
    cli
    cld

    mov ax, cs
    mov ds, ax

    lgdt [gdt_desc]
    lidt [idt_desc]

    mov eax, cr0
    or  eax, 1
    mov cr0, eax

    jmp SEL_CODE0:pm16_entry

pm16_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, STACK0_TOP

    mov ax, SEL_TSS
    ltr ax

    push dword VM86_SEG          ; GS
    push dword VM86_SEG          ; FS
    push dword VM86_SEG          ; DS
    push dword VM86_SEG          ; ES
    push dword VM86_SEG          ; SS
    push dword VM86_SP           ; ESP
    push dword 0x00023202        ; EFLAGS: VM=1, IF=1, IOPL=3
    push dword VM86_SEG          ; CS
    push dword vm86_entry        ; EIP
    iretd

; ---------------------------------------------------------------------------
; VM86 code.  CS=DS=SS=ES=FS=GS=0x1000.
; ---------------------------------------------------------------------------
vm86_entry:
    mov ax, cs
    mov ds, ax
    mov ss, ax

; 1: the return address on the stack, and SP back after the RET
    mov sp, 0x8000
    call get_ret
back1: cmp ax, back1
    FAILIF ne, 1
    cmp bx, 0x7FFE
    FAILIF ne, 2
    cmp sp, 0x8000
    FAILIF ne, 3

; 2: CALL at SP=0 pushes at FFFEh; its RET wraps SP to 0
    xor sp, sp
    call get_ret
back2: cmp ax, back2
    FAILIF ne, 4
    cmp bx, 0xFFFE
    FAILIF ne, 5
    test sp, sp
    FAILIF nz, 6

; 3: RET imm16 wraps SP: FFEEh + 2 + 20h -> 0010h
    mov sp, 0xFFF0
    call ret_20
    cmp sp, 0x0010
    FAILIF ne, 7

; 4: odd SPs: the return address at 8001h (lane 1) and at 7FFFh (SP[1:0] = 3)
    mov sp, 0x8003
    call get_ret
back4: cmp ax, back4
    FAILIF ne, 8
    cmp sp, 0x8003
    FAILIF ne, 9
    mov sp, 0x8001
    call get_ret
back5: cmp ax, back5
    FAILIF ne, 10
    cmp sp, 0x8001
    FAILIF ne, 11

; 6: IP wraps at 64K (see call_ret16.asm)
    mov ax, WRAP_SEG
    mov es, ax
    mov byte [es:0x0000], 0xE8
    mov word [es:0x0001], 0xFFF0
    mov byte [es:0x0003], 0xEA
    mov word [es:0x0004], back6
    mov word [es:0x0006], VM86_SEG
    mov byte [es:0x0100], 0xC3
    mov byte [es:0xFFF3], 0xC3
    mov byte [es:0xFFFD], 0xE8
    mov word [es:0xFFFE], 0x0100
    mov sp, 0x8000
    jmp WRAP_SEG:0xFFFD
back6: cmp sp, 0x8000
    FAILIF ne, 12
    cmp word [ss:0x7FFE], 0x0003
    FAILIF ne, 13

; 7: recursion ten deep
    mov cx, 10
    call sum_rec
    cmp ax, 55
    FAILIF ne, 14
    cmp sp, 0x8000
    FAILIF ne, 15

; 8: the callee changes its return address
    call redirect_ret
    mov ax, 16
    int VEC_FAIL
back8: cmp sp, 0x8000
    FAILIF ne, 17

; 9: fifty calls in a loop, then indirect CALLs
    xor ax, ax
    mov cx, 50
.loop:
    call inc_ax
    loop .loop
    mov bx, inc_ax
    call bx
    call [ind_ptr]
    cmp ax, 52
    FAILIF ne, 18
    cmp sp, 0x8000
    FAILIF ne, 19

    int VEC_DONE
    hlt
    jmp $

get_ret:
    mov bx, sp
    mov ax, [ss:bx]
    ret

ret_20:
    ret 0x20

sum_rec:
    test cx, cx
    jz .zero
    push cx
    dec cx
    call sum_rec
    pop cx
    add ax, cx
    ret
.zero:
    xor ax, ax
    ret

redirect_ret:
    mov bx, sp
    mov word [ss:bx], back8
    ret

inc_ax:
    inc ax
    ret

align 2
ind_ptr: dw inc_ax

; ---------------------------------------------------------------------------
; Ring-0 USE16 monitor handlers.
; ---------------------------------------------------------------------------
done_handler:
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail_handler:
    movzx eax, ax
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; ---------------------------------------------------------------------------
; Descriptor tables.
; ---------------------------------------------------------------------------
align 8
gdt:
    dq 0x0000000000000000

    ; Ring-0 USE16 code, base=0x10000, limit=0xffff.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10011011b
    db 00000000b
    db 0x00

    ; Ring-0 USE16 data/stack, base=0x10000, limit=0xffff.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10010011b
    db 00000000b
    db 0x00

    ; 32-bit TSS, base=0x10000+tss, including the I/O bitmap.
tss_desc:
    dw tss_end - tss - 1
    dw tss
    db 0x01
    db 10001001b
    db 00000000b
    db 0x00
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times VEC_DONE dq 0

    ; INT 22h: pass, from VM86 (386 interrupt gate, DPL=3, USE16 ring-0 code).
    dw done_handler
    dw SEL_CODE0
    db 0
    db 11101110b
    dw 0

    ; INT 23h: fail with the code in AX, from VM86.
    dw fail_handler
    dw SEL_CODE0
    db 0
    db 11101110b
    dw 0

    times (256 - VEC_FAIL - 1) dq 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

align 4
tss:
    dd 0                    ; +00 backlink
    dd STACK0_TOP           ; +04 ESP0
    dd SEL_DATA0            ; +08 SS0
    dd 0                    ; +0C ESP1
    dd 0                    ; +10 SS1
    dd 0                    ; +14 ESP2
    dd 0                    ; +18 SS2
    dd 0                    ; +1C CR3
    dd 0                    ; +20 EIP
    dd 0                    ; +24 EFLAGS
    dd 0                    ; +28 EAX
    dd 0                    ; +2C ECX
    dd 0                    ; +30 EDX
    dd 0                    ; +34 EBX
    dd 0                    ; +38 ESP
    dd 0                    ; +3C EBP
    dd 0                    ; +40 ESI
    dd 0                    ; +44 EDI
    dd 0                    ; +48 ES
    dd 0                    ; +4C CS
    dd 0                    ; +50 SS
    dd 0                    ; +54 DS
    dd 0                    ; +58 FS
    dd 0                    ; +5C GS
    dd 0                    ; +60 LDTR
    dw 0                    ; +64 debug trap
    dw io_bitmap - tss      ; +66 IOPB offset

io_bitmap:
    times 5 db 0            ; Allow ports 00h-27h, including PIC port 20h.
    db 0xff                 ; Required all-ones terminator.
tss_end:

