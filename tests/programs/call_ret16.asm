; call_ret16.asm - 16-bit near CALL and RET in real mode
;
; A 16-bit CALL rel16 is a hardwired branch with its ESP commit (as the
; 32-bit one); a 16-bit RET is a direct load of its target when its 16-bit
; stack and CS hold every 16-bit offset and SP[1:0] is not 3 (the word at
; SP=FFFFh straddles the stack's limit and faults on the ROM path). This
; walks their edges: SP and IP wrapping at 64K, RET imm16, odd SPs on both
; sides of the SP[1:0] = 3 split, a RET that faults, recursion deeper than
; the return-address stack, a RET whose return address was changed, indirect
; CALLs, and single-stepping through a CALL and its RET.
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF
CODE_SEG    equ 0x1000          ; the harness loads the code at 10000h
STACK_SEG   equ 0x5000          ; 50000h-5FFFFh: all 64K of it in RAM
WRAP_SEG    equ 0x3000          ; 30000h-3FFFFh: code at both ends of a segment

; Fail with code %2 unless condition %1 holds (cmp/test before it).
%macro FAILIF 2
    j%-1 %%ok
    mov ax, %2
    jmp fail
%%ok:
%endmacro

start:
    cli
    mov ax, cs
    mov ds, ax
    xor ax, ax
    mov es, ax
    mov word [es:1*4], int1_handler
    mov word [es:1*4+2], cs
    mov word [es:12*4], int12_handler
    mov word [es:12*4+2], cs
    mov word [es:13*4], int13_handler
    mov word [es:13*4+2], cs
    mov ax, STACK_SEG
    mov ss, ax

; 1: the return address on the stack, and SP back after the RET
    mov sp, 0x8000
    call get_ret                ; ax = [sp], bx = sp in the callee
back1: cmp ax, back1
    FAILIF ne, 1
    cmp bx, 0x7FFE
    FAILIF ne, 2
    cmp sp, 0x8000
    FAILIF ne, 3

; 2: CALL at SP=0 pushes at FFFEh; its RET (at SP=FFFEh) wraps SP to 0
    xor sp, sp
    call get_ret
back2: cmp ax, back2
    FAILIF ne, 4
    cmp bx, 0xFFFE
    FAILIF ne, 5
    test sp, sp
    FAILIF nz, 6
    cmp word [ss:0xFFFE], back2
    FAILIF ne, 7

; 3: RET imm16 wraps SP: FFEEh + 2 + 20h = 10010h -> 0010h
    mov sp, 0xFFF0
    call ret_20
    cmp sp, 0x0010
    FAILIF ne, 8

; 4: a RET at SP=FFFEh with no CALL before it
    mov word [ss:0xFFFE], back4
    mov sp, 0xFFFE
    ret
    mov ax, 9
    jmp fail
back4: test sp, sp
    FAILIF nz, 10

; 5: odd SPs: the return address at 8001h (lane 1, a direct load) and at
; 7FFFh (SP[1:0] = 3, the ROM path)
    mov sp, 0x8003
    call get_ret
back5: cmp ax, back5
    FAILIF ne, 11
    cmp sp, 0x8003
    FAILIF ne, 12
    mov sp, 0x8001
    call get_ret
back6: cmp ax, back6
    FAILIF ne, 13
    cmp sp, 0x8001
    FAILIF ne, 14

; 7: a RET at SP=FFFFh faults (the word straddles the limit) before it
; changes SP or IP
    mov word [fault_vec], 0
    mov word [fault_resume], back7
    mov sp, 0xFFFF
t7_ret:
    ret
    mov ax, 15
    jmp fail
back7: mov sp, 0x8000
    mov word [fault_resume], unexpected_fault
    cmp word [fault_vec], 0
    FAILIF e, 16
    cmp word [fault_ip], t7_ret
    FAILIF ne, 17
    cmp word [fault_sp], 0xFFF9 ; the frame went below SP=FFFFh
    FAILIF ne, 18

; 8: IP wraps at 64K. In WRAP_SEG:
;   0000: call FFF3h         (3 + FFF0h wraps to FFF3h)
;   0003: jmp far CODE_SEG:back8
;   0100: ret
;   FFF3: ret
;   FFFD: call 0100h         (the return address wraps to 0000h)
    mov ax, WRAP_SEG
    mov es, ax
    mov byte [es:0x0000], 0xE8
    mov word [es:0x0001], 0xFFF0
    mov byte [es:0x0003], 0xEA
    mov word [es:0x0004], back8
    mov word [es:0x0006], CODE_SEG
    mov byte [es:0x0100], 0xC3
    mov byte [es:0xFFF3], 0xC3
    mov byte [es:0xFFFD], 0xE8
    mov word [es:0xFFFE], 0x0100
    mov sp, 0x8000
    jmp WRAP_SEG:0xFFFD
back8: cmp sp, 0x8000
    FAILIF ne, 19
    cmp word [ss:0x7FFE], 0x0003 ; the second CALL's return address
    FAILIF ne, 20

; 9: recursion ten deep (the return-address stack holds four)
    mov sp, 0x8000
    mov cx, 10
    call sum_rec                ; ax = 10 + 9 + ... + 1
    cmp ax, 55
    FAILIF ne, 21
    cmp sp, 0x8000
    FAILIF ne, 22

; 10: the callee changes its return address (a wrong prediction)
    call redirect_ret
    mov ax, 23
    jmp fail
back10: cmp sp, 0x8000
    FAILIF ne, 24

; 11: RET 4 drops the arguments
    push word 0x1111
    push word 0x2222
    call add_args
    cmp ax, 0x3333
    FAILIF ne, 25
    cmp sp, 0x8000
    FAILIF ne, 26

; 12: fifty calls in a loop, then indirect CALLs (register and memory)
    xor ax, ax
    mov cx, 50
.loop:
    call inc_ax
    loop .loop
    mov bx, inc_ax
    call bx
    call [ind_ptr]
    cmp ax, 52
    FAILIF ne, 27
    cmp sp, 0x8000
    FAILIF ne, 28

; 13: single-step through a CALL and its RET: a trap after each, at the
; callee and then at the return address
    mov word [trap_count], 0
    pushf
    pop ax
    or ax, 0x0100
    push ax
    popf                        ; TF: the trap comes after the next instruction
    call tf_func
t13_back:
    nop                         ; the handler clears TF here
    cmp word [trap_count], 2
    FAILIF ne, 29
    cmp word [trap_ip], tf_func
    FAILIF ne, 30
    cmp word [trap_ip+2], t13_back
    FAILIF ne, 31
    cmp sp, 0x8000
    FAILIF ne, 32

    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

unexpected_fault:
    mov ax, 0x99
fail:
    movzx eax, ax
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt

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
    mov word [ss:bx], back10
    ret

add_args:
    mov bx, sp
    mov ax, [ss:bx+2]
    add ax, [ss:bx+4]
    ret 4

inc_ax:
    inc ax
    ret

tf_func:
    ret

int12_handler:
    mov word [cs:fault_vec], 12
    jmp fault_common
int13_handler:
    mov word [cs:fault_vec], 13
fault_common:
    mov [cs:fault_sp], sp
    mov bp, sp
    mov ax, [ss:bp]
    mov [cs:fault_ip], ax
    jmp [cs:fault_resume]       ; abandon the frame

int1_handler:
    push bp
    mov bp, sp
    push ax
    push bx
    mov bx, [cs:trap_count]
    cmp bx, 4
    jae .full
    shl bx, 1
    mov ax, [ss:bp+2]
    mov [cs:trap_ip+bx], ax
    inc word [cs:trap_count]
.full:
    cmp word [ss:bp+2], t13_back
    jne .done
    and word [ss:bp+6], 0xFEFF  ; clear TF in the saved FLAGS
.done:
    pop bx
    pop ax
    pop bp
    iret

align 2
fault_vec    dw 0
fault_ip     dw 0
fault_sp     dw 0
fault_resume dw unexpected_fault
trap_count   dw 0
trap_ip      times 4 dw 0
ind_ptr      dw inc_ax
