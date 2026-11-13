; os2_arpl_store_intr.asm - OS/2 Warp 4's 16-bit API thunk with INTR swept across it.
;
; On the MEGA65 board OS/2 4.52 froze four times out of four, about 26 s into its
; hard-disk boot, with CS:EIP = 0148:7E92: the store `mov [bp+14h],ax` right after
; `arpl ax,bx` had issued (EIP past it), the load after it never issued, no memory
; request was outstanding (L1 and L2 off changed nothing), and the timer and RTC
; interrupts stayed pending in the PIC, never acknowledged.
;
; The thunk's bytes from 0148:7E58 to 7EA6 are copied here at the same offset in
; the page (E58h) and run from a 16-bit code segment on a 16-bit stack, as there.
; For every delay 0..DMAX the test bench raises INTR that many cycles after the
; OUT that arms it; the call runs the thunk, then the interrupt must have been
; taken exactly once and the stores must hold the ARPL results. A lockup is the
; bench's TIMEOUT. Two passes: ARPL adjusts the RPL (ZF = 1), then it does not.

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
DMAX         equ 260

SEL_CODE16   equ 0x08             ; base 10000h, 16-bit, the thunk's segment
SEL_DATA16   equ 0x10             ; base 10000h, limit FFFFh
SEL_STACK16  equ 0x18             ; base 30000h, limit FFFFh, B = 0
SEL_CODE32   equ 0x20             ; base 10000h, 32-bit, the ISR
SEL_FLAT     equ 0x28             ; base 0, 4 GB
SEL_OS2ES    equ 0x0A00           ; OS/2's ES selector, base 40000h

STACK_TOP    equ 0xF000
BPV          equ STACK_TOP - 4      ; the thunk's bp: after the near call and push bp

start:
    cli
    cld
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    xor eax, eax
    mov cr3, eax                  ; identity page tables at 0 (OS/2 runs paged)
    mov eax, cr0
    or eax, 0x80000001
    mov cr0, eax
    jmp SEL_CODE16:pm16

pm16:
    mov ax, SEL_DATA16
    mov ds, ax
    mov ax, SEL_FLAT
    mov fs, ax
    mov ax, SEL_STACK16
    mov ss, ax
    mov sp, STACK_TOP

    ; OS/2's data: es:[0D9Eh] bit 7 clear (the jz is taken), ss:[FFF2h] -> a
    ; per-task block whose word at +0F98h is the selector ARPL takes its RPL from.
    mov byte [fs:dword 0x40000 + 0x0D9E], 0
    mov word [ss:0xFFF2], 0x0100

    mov dx, VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al

    mov word [pass], 0
.pass:
    mov si, 1                     ; si = the delay (0 never fires in this bench)
.next:
    ; The caller's selectors at [bp+10h] and [bp+14h] (the thunk's bp is the
    ; caller's sp - 4: the near call's return address and push bp).
    mov ax, [pass]
    test ax, ax
    jnz .noadj
    mov word [ss:0x0100 + 0x0F98], 0x0F13   ; RPL 3: both callers' RPL 0 get adjusted
    mov word [ss:BPV + 0x14], 0x1230
    mov word [ss:BPV + 0x10], 0x4560
    jmp .armed
.noadj:
    mov word [ss:0x0100 + 0x0F98], 0x0F10   ; RPL 0: nothing to adjust
    mov word [ss:BPV + 0x14], 0x1233
    mov word [ss:BPV + 0x10], 0x4562
.armed:
    mov word [intr_count], 0
    mov dx, DELAY_PORT
    mov ax, si
    out dx, ax
    mov dx, SIGNAL_PORT
    mov al, 1
    sti
    out dx, al                    ; INTR si cycles from here
    call thunk
    mov cx, 400                   ; a late INTR: give it time to arrive
.wait:
    cmp word [intr_count], 0
    jne .taken
    loop .wait
.taken:
    cli
    cmp word [intr_count], 1
    mov bl, 1
    jne fail
    mov ax, [ss:BPV + 0x14]
    mov bx, [ss:BPV + 0x10]
    cmp word [pass], 0
    jne .chk_noadj
    cmp ax, 0x1233
    mov bl, 2
    jne fail
    cmp word [ss:BPV + 0x10], 0x4563
    mov bl, 3
    jne fail
    jmp .ok
.chk_noadj:
    cmp ax, 0x1233
    mov bl, 4
    jne fail
    cmp word [ss:BPV + 0x10], 0x4562
    mov bl, 5
    jne fail
.ok:
    cmp sp, STACK_TOP
    mov bl, 6
    jne fail
    inc si
    cmp si, DMAX
    jbe .next
    inc word [pass]
    cmp word [pass], 2
    jb .pass

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail:                             ; bl = check, si = delay, [pass]
    mov ax, [pass]
    shl eax, 16
    mov ax, si
    shl eax, 8
    mov al, bl
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 4
pass:       dw 0
intr_count: dw 0

BITS 16
; OS/2 4.52 OS2KRNL, selector 0148 (base FFEE9000h), offsets 7E58h..7EA6h, byte
; for byte, at the same page offset (E58h); then this test's own epilogue.
    times 0x0E58 - ($ - $$) db 0xCC
thunk:
    db 0x66, 0x0F, 0xB7, 0xE4                  ; 7E58 movzx esp, sp
    db 0x55                                    ; 7E5C push bp
    db 0x8B, 0xEC                              ; 7E5D mov bp, sp
    db 0x66, 0x33, 0xC0                        ; 7E5F xor eax, eax
    db 0x1E                                    ; 7E62 push ds
    db 0x06                                    ; 7E63 push es
    db 0x0F, 0xA0                              ; 7E64 push fs
    db 0x0F, 0xA8                              ; 7E66 push gs
    db 0x66, 0x60                              ; 7E68 pushad
    db 0x06                                    ; 7E6A push es
    db 0x68, 0x00, 0x0A                        ; 7E6B push 0A00h
    db 0x07                                    ; 7E6E pop es
    db 0x26, 0xF6, 0x06, 0x9E, 0x0D, 0x80      ; 7E6F test byte es:[0D9Eh], 80h
    db 0x0F, 0x84, 0x06, 0x00                  ; 7E75 jz 7E7F
    db 0x68, 0x0A, 0x18                        ; 7E79 push 180Ah   (not run)
    db 0xE8, 0xC9, 0x30                        ; 7E7C call AF48h   (not run)
    db 0x07                                    ; 7E7F pop es
    db 0x8B, 0x46, 0x14                        ; 7E80 mov ax, [bp+14h]
    db 0x36, 0x8B, 0x1E, 0xF2, 0xFF            ; 7E83 mov bx, ss:[0FFF2h]
    db 0x36, 0x8B, 0x9F, 0x98, 0x0F            ; 7E88 mov bx, ss:[bx+0F98h]
    db 0x63, 0xD8                              ; 7E8D arpl ax, bx
    db 0x89, 0x46, 0x14                        ; 7E8F mov [bp+14h], ax   <- board: issued
    db 0x8B, 0x46, 0x10                        ; 7E92 mov ax, [bp+10h]   <- never issued
    db 0x36, 0x8B, 0x1E, 0xF2, 0xFF            ; 7E95 mov bx, ss:[0FFF2h]
    db 0x36, 0x8B, 0x9F, 0x98, 0x0F            ; 7E9A mov bx, ss:[bx+0F98h]
    db 0x63, 0xD8                              ; 7E9F arpl ax, bx
    db 0x89, 0x46, 0x10                        ; 7EA1 mov [bp+10h], ax
    db 0x8B, 0x5E, 0x0E                        ; 7EA4 mov bx, [bp+0Eh]
    popad
    pop gs
    pop fs
    pop es
    pop ds
    pop bp
    ret

align 8
gdt:
    dq 0
    dq 0x00009B010000FFFF          ; 08 code16, base 10000h
    dq 0x000093010000FFFF          ; 10 data16, base 10000h
    dq 0x000093030000FFFF          ; 18 stack16, base 30000h, B = 0
    dq 0x00CF9B010000FFFF          ; 20 code32, base 10000h
    dq 0x00CF93000000FFFF          ; 28 flat data
    times (SEL_OS2ES / 8 - 6) dq 0
    dq 0x000093040000FFFF          ; 0A00 data16, base 40000h
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times INTR_VECTOR dq 0
    dw isr32                       ; 386 interrupt gate to the 32-bit ISR
    dw SEL_CODE32
    db 0
    db 10001110b
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

BITS 32
isr32:
    inc word [intr_count]          ; DS = SEL_DATA16 everywhere
    iretd
