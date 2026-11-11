; unreal_odd_cs.asm - setting CR0.PE keeps the CS selector, RPL bits included
;
; SpeedSys 4.78 enters "unreal mode" from a code segment whose selector value has RPL bits
; set (469Dh): MOV CR0 (PE=1), LGDT, MOV FS/GS/DS/ES with a 4 GB DPL-0 data selector, MOV CR0
; (PE=0), no far jump. On the Intel486 CS keeps 469Dh and CPL is 0 throughout. z486 cleared
; CS[1:0] to make CPL 0, so CS came back as 469Ch: the next real-mode interrupt pushed 469Ch
; and its IRET resumed 16 bytes off, and SpeedSys ran into its data (an INT 21h AH=01h wait).
;
; Here, at CS = 0FFFh (RPL bits 11, the same linear addresses as CS = 1000h):
;   1) PE on, MOV BX,CS (must stay 0FFFh), MOV FS with a DPL-0 4 GB data selector (CPL must be
;      0 although CS's RPL bits are 3), PE off; CS still 0FFFh; FS reads above 64K (unreal).
;   2) A real-mode INT/IRET pushes CS = 0FFFh and comes back.
;   3) PE on again and a far JMP to a code selector: CS = 08h (the RPL is the CPL, 0).
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 failure code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE16 equ 0x08
SEL_FLAT   equ 0x10
ODD_CS     equ 0x0FFF          ; linear base 0FFF0h: offsets are label + 10h
INT_VEC    equ 0x60

start:
    cli
    cld
    xor ax, ax
    mov ds, ax
    mov word [INT_VEC * 4], int_handler + 0x10
    mov word [INT_VEC * 4 + 2], ODD_CS
    mov ax, 0x1000
    mov ds, ax
    mov ss, ax
    mov sp, 0xF000
    lgdt [gdt_desc]
    jmp ODD_CS:(odd_cs + 0x10)

odd_cs:
    mov eax, 0x01
    mov dx, cs
    cmp dx, ODD_CS
    jne fail

    ; 1) unreal mode
    mov eax, cr0
    or al, 1
    mov cr0, eax
    jmp short .pm
.pm:
    mov bx, cs                  ; still 0FFFh
    mov cx, SEL_FLAT
    mov fs, cx                  ; DPL 0: CPL must be 0
    and al, 0xFE
    mov cr0, eax
    jmp short .rm
.rm:
    mov [cs_in_pm], bx
    mov eax, 0x02
    cmp word [cs_in_pm], ODD_CS
    jne fail
    mov eax, 0x03
    mov dx, cs
    cmp dx, ODD_CS
    jne fail
    mov eax, 0x04
    mov ebx, 0x10000            ; past FS's real-mode limit: the 4 GB limit stays
    mov ecx, [fs:ebx]
    cmp ecx, [0]                ; DS = 1000h: the program's first bytes
    jne fail

    ; 2) a real-mode INT/IRET
    int INT_VEC
    mov eax, 0x05
    cmp word [int_count], 1
    jne fail
    mov eax, 0x06
    cmp word [int_cs], ODD_CS
    jne fail
    mov eax, 0x07
    mov dx, cs
    cmp dx, ODD_CS
    jne fail

    ; 3) PE on, far JMP: the new CS gets RPL 0
    mov eax, cr0
    or al, 1
    mov cr0, eax
    jmp SEL_CODE16:pm16
pm16:
    mov eax, 0x08
    mov dx, cs
    cmp dx, SEL_CODE16
    jne fail

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

int_handler:
    push bp
    mov bp, sp
    mov ax, [bp + 4]            ; the pushed CS
    mov [int_cs], ax
    inc word [int_count]
    pop bp
    iret

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 8
gdt:
    dq 0x0000000000000000   ; null
    dq 0x00009B010000FFFF   ; 08h: 16-bit code, base 10000h, 64K
    dq 0x00CF93000000FFFF   ; 10h: data, base 0, 4 GB
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 2
cs_in_pm:  dw 0
int_count: dw 0
int_cs:    dw 0
