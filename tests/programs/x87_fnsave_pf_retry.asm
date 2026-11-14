; x87_fnsave_pf_retry.asm - FNSAVE into a page that faults, then fixed up, then FNSAVE again.
;
; CorePlus (Linux 6.18, i386, no FXSR) crawled on the board with nearly every sample at the
; FNSAVE/FWAIT of copy_fpstate_to_sigframe(): the kernel saves the x87 state straight into a
; signal frame on the user stack with page faults disabled; on a fault it clears the buffer with
; ordinary stores (which fault the page in or make it writable) and saves again, for as long as
; the save faults. Linux makes a page writable (after copy-on-write) without INVLPG: a fault
; drops the stale TLB entry, and the retried access walks the tables again.
;
; Each case: the x87 holds ST0 = 2.0, ST1 = 1.0. FNSAVE into the buffer must fault (the #PF
; handler sends it to a fixup label, as the kernel's exception table does); REP STOSB clears the
; buffer with the handler in "fault in" mode (it sets P and R/W in the PTE, no INVLPG, and
; returns); then FNSAVE again must not fault and must store the state the x87 held.
;   A: the page read-only, its translation in the TLB (a load before), CR0.WP = 1
;   B: a buffer crossing into a read-only page (the first page writable)
;   C: the page not present
;   D: a writable page with D = 0 in the TLB: FNSAVE must set D, not fault
;   E: FNSTENV as A; the x87 stack must survive the faulting store
;   F: FRSTOR from a buffer crossing into a not-present page: the read faults mid-load, the
;      handler maps the page and FRSTOR runs again (it must not hang on the open transfer)
;   G: FLDENV the same way
; A fault mid-transfer left the x87 waiting for the rest of the words, and the restarted
; instruction's routine waited for PEREQ to fall first: a deadlock (CorePlus's crawl).
; Fail codes (port E4h): low byte = check, byte 1 = case, bytes 2-3 = the last error code.

BITS 16
org 0

STATUS_PORT  equ 0xE0
DATA_PORT    equ 0xE4
STATUS_PASS  equ 0x01
STATUS_FAIL  equ 0xFF
SEL_CODE     equ 0x08               ; base 10000h, 32-bit
SEL_DATA     equ 0x10               ; base 10000h
SEL_FLAT     equ 0x18               ; base 0
STACK_TOP    equ 0x9000
PT0          equ 0x1000             ; the page table for 0-4 MB (the runner's layout)
TP           equ 0x40000            ; test pages (identity mapped)
TP2          equ 0x41000
SCRATCH      equ 0x1A000            ; a writable buffer for check images
PTE_P        equ 0x01
PTE_RW       equ 0x02
PTE_A        equ 0x20
PTE_D        equ 0x40

%macro SETPTE 2                     ; page, flags (then INVLPG: a clean start)
    mov dword [fs:PT0 + (%1 >> 12) * 4], %1 | %2
    invlpg [fs:%1]
%endmacro

%macro CHECK 3                      ; operand, expected, check code
    cmp %1, %2
    mov cl, %3
    jne fail
%endmacro

start:
    cli
    cld
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    xor eax, eax
    mov cr3, eax
    mov eax, cr0
    or eax, 0x80010023              ; PG, WP, NE, MP, PE (as Linux)
    mov cr0, eax
    db 0x66, 0xEA
    dd pm32
    dw SEL_CODE

BITS 32
pm32:
    mov ax, SEL_DATA
    mov ds, ax
    mov ss, ax
    mov esp, STACK_TOP
    mov ax, SEL_FLAT
    mov es, ax
    mov fs, ax

; ---- A: read-only page, translation cached --------------------------------------------
    mov byte [case], 0x0A
    SETPTE TP, PTE_P | PTE_A
    SETPTE TP2, PTE_P | PTE_RW | PTE_A | PTE_D
    mov eax, [fs:TP + 0x100]        ; the read-only translation into the TLB
    mov edi, TP + 0x100
    call save_case
    CHECK dword [last_err], 3, 0x10 ; present, write, supervisor
    mov eax, [last_cr2]
    and eax, 0xFFFFF000
    CHECK eax, TP, 0x11

; ---- B: a buffer crossing into a read-only page -------------------------------------------
    mov byte [case], 0x0B
    SETPTE TP, PTE_P | PTE_RW | PTE_A | PTE_D
    SETPTE TP2, PTE_P | PTE_A
    mov eax, [fs:TP2]               ; both translations cached
    mov eax, [fs:TP + 0xFF0]
    mov edi, TP2 - 0x30             ; 30h bytes in TP, the rest in TP2
    call save_case
    CHECK dword [last_err], 3, 0x10
    mov eax, [last_cr2]
    and eax, 0xFFFFF000
    CHECK eax, TP2, 0x11

; ---- C: not present ------------------------------------------------------------------------
    mov byte [case], 0x0C
    SETPTE TP, PTE_A                ; P = 0
    SETPTE TP2, PTE_P | PTE_RW | PTE_A | PTE_D
    mov edi, TP + 0x200
    call save_case
    CHECK dword [last_err], 2, 0x10 ; not present, write
    mov eax, [last_cr2]
    and eax, 0xFFFFF000
    CHECK eax, TP, 0x11

; ---- D: writable, D = 0, translation cached: no fault, D set ------------------------------
    mov byte [case], 0x0D
    SETPTE TP, PTE_P | PTE_RW       ; A = D = 0
    mov eax, [fs:TP + 0x300]        ; sets A, caches D = 0
    call fpu_load
    mov dword [pf_count], 0
    mov dword [pf_mode], 0          ; any fault fails
    mov edi, TP + 0x300
    fnsave [es:edi]
    fwait
    CHECK dword [pf_count], 0, 0x20
    call check_image
    mov eax, [fs:PT0 + (TP >> 12) * 4]
    and eax, PTE_D | PTE_A
    CHECK eax, PTE_D | PTE_A, 0x21

; ---- E: FNSTENV into a read-only page: as A; the stack survives -------------------------
    mov byte [case], 0x0E
    SETPTE TP, PTE_P | PTE_A
    mov eax, [fs:TP + 0x400]
    call fpu_load
    mov dword [pf_count], 0
    mov dword [pf_mode], 1
    mov dword [fixup_eip], env_fixed
    mov edi, TP + 0x400
    fnstenv [es:edi]
    fwait
    mov cl, 1
    jmp fail
env_fixed:
    CHECK dword [pf_count], 1, 2
    mov dword [pf_mode], 2
    push edi
    xor eax, eax
    mov ecx, 28
    rep stosb
    pop edi
    mov dword [pf_mode], 1
    mov dword [fixup_eip], save_case.refault
    fnstenv [es:edi]
    fwait
    mov dword [pf_mode], 0
    CHECK word [es:edi], 0x037F, 4
    CHECK dword [last_err], 3, 0x10
    mov edi, SCRATCH
    fnsave [es:edi]
    fwait
    call check_image

; ---- F: FRSTOR from a buffer crossing into a not-present page (read fault mid-load) ------
    mov byte [case], 0x0F
    SETPTE TP, PTE_P | PTE_RW | PTE_A | PTE_D
    SETPTE TP2, PTE_P | PTE_RW | PTE_A | PTE_D
    call fpu_load
    mov edi, TP2 - 0x30
    fnsave [es:edi]                 ; the image, both pages present
    fwait
    fldz                            ; the x87 holds something else
    SETPTE TP2, PTE_A               ; its last 4Ch bytes not present
    mov eax, [fs:TP + 0xFF0]
    mov dword [pf_count], 0
    mov dword [pf_mode], 2          ; fault in, retry
    frstor [es:edi]
    fwait
    mov dword [pf_mode], 0
    CHECK dword [pf_count], 1, 0x22
    CHECK dword [last_err], 0, 0x23 ; not present, read
    mov edi, SCRATCH
    fnsave [es:edi]
    fwait
    call check_image

; ---- G: FLDENV from a buffer crossing into a not-present page ---------------------------
    mov byte [case], 0x10
    SETPTE TP2, PTE_P | PTE_RW | PTE_A | PTE_D
    fninit
    mov edi, TP2 - 0x10
    fnstenv [es:edi]
    mov word [es:edi], 0x027F       ; PC = 53 bits
    SETPTE TP2, PTE_A               ; its last 12 bytes not present
    mov dword [pf_count], 0
    mov dword [pf_mode], 2
    fldenv [es:edi]
    fnstcw [cw_out]
    mov dword [pf_mode], 0
    CHECK dword [pf_count], 1, 0x22
    CHECK word [cw_out], 0x027F, 0x24

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; FNSAVE [EDI] must fault once; clear the buffer (faulting it in); FNSAVE again must not fault.
save_case:
    call fpu_load
    mov dword [pf_count], 0
    mov dword [pf_mode], 1
    mov dword [fixup_eip], .fixed
    fnsave [es:edi]
    fwait
    mov cl, 1                       ; the first save did not fault
    jmp fail
.fixed:
    CHECK dword [pf_count], 1, 2
    mov dword [pf_mode], 2          ; clear_user: faults bring the page in
    push edi
    xor eax, eax
    mov ecx, 108
    rep stosb
    pop edi
    mov dword [pf_mode], 1
    mov dword [fixup_eip], .refault
    mov ebx, [pf_count]
    mov [pf_clear], ebx
    fnsave [es:edi]                 ; must not fault now
    fwait
    mov dword [pf_mode], 0
    call check_image
    ret
.refault:
    mov dword [pf_mode], 0
    mov cl, 3                       ; the retried FNSAVE faulted again (CorePlus's livelock)
    jmp fail

fpu_load:
    fninit
    fld1
    fld dword [two]                 ; ST0 = 2.0, ST1 = 1.0
    ret

check_image:                        ; the 108-byte image at ES:EDI
    CHECK word [es:edi], 0x037F, 4                  ; control word
    CHECK dword [es:edi + 28], 0, 5                 ; ST0 = 2.0 = 4000 8000000000000000
    CHECK dword [es:edi + 32], 0x80000000, 6
    CHECK word [es:edi + 36], 0x4000, 7
    CHECK dword [es:edi + 38], 0, 8                 ; ST1 = 1.0 = 3FFF 8000000000000000
    CHECK dword [es:edi + 42], 0x80000000, 9
    CHECK word [es:edi + 46], 0x3FFF, 10
    ret

fail:                               ; cl = check
    mov eax, [last_err]
    shl eax, 8
    mov al, [case]
    shl eax, 8
    mov al, cl
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

pf_unexpected:
    mov cl, 0x30
    jmp fail

; #PF: records CR2 and the error code; mode 1 = fixup (EIP := fixup_eip), mode 2 = fault in
; (P and R/W set in the PTE, no INVLPG, retry), mode 0 = unexpected.
pf_handler:
    push eax
    push ebx
    mov eax, cr2
    mov [last_cr2], eax
    mov ebx, [esp + 8]
    mov [last_err], ebx
    inc dword [pf_count]
    cmp dword [pf_count], 40
    ja .bad                         ; a fault that never goes away
    cmp dword [pf_mode], 1
    je .fix
    cmp dword [pf_mode], 2
    je .faultin
.bad:
    mov dword [esp + 12], pf_unexpected
    jmp .out
.fix:
    mov ebx, [fixup_eip]
    mov [esp + 12], ebx
    jmp .out
.faultin:
    shr eax, 12
    or dword [fs:PT0 + eax * 4], PTE_P | PTE_RW
.out:
    pop ebx
    pop eax
    add esp, 4
    iretd

align 4
two:        dd 0x40000000           ; 2.0f
pf_mode:    dd 0
pf_count:   dd 0
pf_clear:   dd 0
fixup_eip:  dd 0
last_cr2:   dd 0
last_err:   dd 0
cw_out:     dw 0
case:       db 0

BITS 16
align 8
gdt:
    dq 0
    dq 0x00CF9B010000FFFF           ; 08 code32, base 10000h
    dq 0x00CF93010000FFFF           ; 10 data32, base 10000h
    dq 0x00CF93000000FFFF           ; 18 flat data
gdt_end:
gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times 14 dq 0
    dw pf_handler                   ; 0E #PF, 386 interrupt gate
    dw SEL_CODE
    db 0
    db 10001110b
    dw 0
idt_end:
idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000
