; Chris's 3D Benchmark's case with the x87 still busy: FSTP m32 into an absent page two
; instructions after FSUBP (BENCH.EXE 3DEA..3DF2), DJGPP's control word 037Ah, FLD m32 right
; behind it. With an exception unmasked and the x87 busy, the store does not take its direct
; path (which checks the destination first) but the 386 routine, which commands the x87 (it
; converts and pops) before it writes: the write's page fault then restarts the FSTP on an empty
; stack (IE, stack underflow). Pass: one fault, TOP 7 and no IE at the fault, after the restart
; TOP 7 (popped once, then the FLD) and IE clear, the value stored.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
DATA_BASE   equ 0x20000000

; variables in the mapped data page (DS base DATA_BASE)
V_EXPECT    equ 0x000          ; the faulting EIP the handler expects
V_COUNT     equ 0x004          ; faults taken
V_SW        equ 0x008          ; the status word at the fault
V_SW2       equ 0x00C          ; after the restart
V_CASE      equ 0x010
V_REDIRECT  equ 0x014          ; where EBX points after the fault
RES         equ 0x100          ; the redirected destination
R80         equ 0x200          ; an m80's

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff     ; 0x08: code, base 0x00010000
    dq 0x20cf93000000ffff     ; 0x10: data, base 0x20000000
    dq 0x30cf93000000ffff     ; 0x18: stack, base 0x30000000
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd 0x00010000 + gdt

pf_handler:
    push eax
    mov eax, cr2
    sub eax, DATA_BASE + 0x1000
    cmp eax, 0x1000
    jae .bad                                ; not the absent page
    mov eax, [ss:esp + 8]                   ; the faulting EIP
    cmp eax, [V_EXPECT]
    jne .bad
    fnstsw [V_SW]
    inc dword [V_COUNT]
    mov ebx, [V_REDIRECT]
    pop eax
    add esp, 4                              ; the error code
    iretd
.bad:
    mov eax, [V_CASE]
    or eax, 0xE0000
    out DATA_PORT, eax
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 8
idt:
    times 14 dq 0
    dw pf_handler
    dw 0x0008
    db 0
    db 0x8e
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd 0x00010000 + idt

; %1 case number, %2 the store, %3 TOP before it (at the fault), %4 TOP after, %5 the
; destination's offset in the absent page, %6 the redirected destination
%macro STORE_CASE 6
    mov dword [V_CASE], %1
    mov dword [V_EXPECT], %%fault
    mov dword [V_COUNT], 0
    mov dword [V_REDIRECT], %6
    mov ebx, 0x1000 + %5
%%fault:
    %2
    fld dword [cs:fc]
    fnstsw [V_SW2]
    mov eax, %1 | 0x10000                   ; 1xxxx: not one fault
    cmp dword [V_COUNT], 1
    jne fail
    mov eax, %1 | 0x20000                   ; 2xxxx: the x87 changed before the fault
    movzx ecx, word [V_SW]
    and ecx, 0x3801
    cmp ecx, (%3) << 11
    jne fail
    mov eax, %1 | 0x30000                   ; 3xxxx: TOP or IE after the restart
    movzx ecx, word [V_SW2]
    and ecx, 0x380d
    cmp ecx, (%4) << 11
    jne fail
%endmacro

; the stored bytes at %1 must be %2 (%3 bytes from cs:%2)
%macro SAME 3
    mov esi, %2
    mov edi, %1
    mov ecx, %3
%%cmp:
    mov dl, [cs:esi]
    cmp dl, [edi]
    jne fail
    inc esi
    inc edi
    loop %%cmp
%endmacro

times 0x400 - ($ - $$) db 0x90
start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]
    mov esp, 0x8000
    fninit
    fldcw [cs:cw]

    ; BENCH.EXE 3DD1..3DF5, the case set up first so that only a MOV comes between FSUBP and FSTP
    mov dword [V_CASE], 1
    mov dword [V_EXPECT], .fault
    mov dword [V_COUNT], 0
    mov dword [V_REDIRECT], RES
    mov ebx, 0x1000
    fld dword [cs:fa]
    fmul dword [cs:fb]
    fld dword [cs:fc]
    fmul dword [cs:fd]
    fsubp st1, st0                          ; DE E9
    mov eax, esi
.fault:
    fstp dword [ebx]
    fld dword [cs:fc]
    fnstsw [V_SW2]
    mov eax, 0x10001                        ; not one fault
    cmp dword [V_COUNT], 1
    jne fail
    mov eax, 0x20001                        ; the x87 changed before the fault
    movzx ecx, word [V_SW]
    and ecx, 0x3801
    cmp ecx, 7 << 11
    jne fail
    mov eax, 0x30001                        ; TOP or IE after the restart
    movzx ecx, word [V_SW2]
    and ecx, 0x380d
    cmp ecx, 7 << 11
    jne fail
    mov eax, 0x40001
    SAME RES, r_m32, 4
    fstp st0

    mov al, 1
    out STATUS_PORT, al
    hlt

fail:
    out DATA_PORT, eax
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 4
cw:     dw 0x037a
fa:     dd 1.5
fb:     dd 2.0
fc:     dd 0.5
fd:     dd 3.0
r_m32:  dd 1.5                  ; 1.5 * 2 - 0.5 * 3
r_sum:  dd 2.0                  ; 0.5 + 1.5
q64:    dq -12345.6789
t80:    dt 3.14159265358979323846
i32:    dd -123456
i16:    dw 31000
i64:    dq -9876543210


