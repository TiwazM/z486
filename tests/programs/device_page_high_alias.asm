; device_page_high_alias.asm - stores above 128 MB never reach the L1
;
; The L1 tags hold 27 address bits, so an access at or above 128 MB must be a
; device access: a PCI BAR (zSST's Voodoo registers at E0000000) aliases RAM
; at the same low 27 bits. Glide's write of 0 to dAdY (BAR+070h) went through
; the fast store path into the cached IVT line and cleared IVT[1Ch].
;
; RAM dwords at 20070h are cached, then written through a page mapped to
; physical E0020070h (paging on: a plain store and a read-modify-write) and
; through the flat physical address E0020074h (paging off). The testbench
; drops writes beyond its memory, so the RAM dwords must keep their values,
; in the L1 and, after WBINVD, in memory.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
DATA        equ 0x00020070                  ; RAM, cached
DEVLIN      equ 0x00050000                  ; linear page -> physical E0020000
DEVPHYS     equ 0xE0020000                  ; aliases 00020000 in 27 bits

times 0x200 - ($ - $$) db 0x90
start:
    mov dword [DATA], 0x11111111
    mov dword [DATA + 4], 0x22222222
    mov dword [DATA + 8], 0x33333333
    mov eax, [DATA]                         ; the line is in the L1
    add eax, [DATA + 4]
    add eax, [DATA + 8]

    ; paging on: a store and a read-modify-write to the device page
    mov dword [DEVLIN + 0x70], 0xDEADBEEF
    add dword [DEVLIN + 0x78], 1
    mov edx, 1
    cmp dword [DATA], 0x11111111
    jne fail
    mov edx, 2
    cmp dword [DATA + 8], 0x33333333
    jne fail

    ; paging off: the physical address itself
    mov eax, cr0
    and eax, 0x7FFFFFFF
    mov cr0, eax
    jmp short .flat
.flat:
    mov dword [DEVPHYS + 0x74], 0xCAFEF00D
    mov edx, 3
    cmp dword [DATA + 4], 0x22222222
    jne fail

    ; and memory itself
    wbinvd
    mov edx, 4
    cmp dword [DATA], 0x11111111
    jne fail
    mov edx, 5
    cmp dword [DATA + 4], 0x22222222
    jne fail
    mov edx, 6
    cmp dword [DATA + 8], 0x33333333
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
