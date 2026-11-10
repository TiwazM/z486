; x87_store_page_fault with DJGPP's control word 0372h (invalid, zero divide and overflow
; unmasked), as Chris's 3D Benchmark runs: z486 then stores through the 386 routines
; instead of the direct paths. Same cases and checks; a stack fault on the restart now shows
; as IE with ES (the checks read the status with FNSTSW, which does not wait).

; Direct x87 stores and page faults: a store to an absent page must fault before the x87
; changes (FSTP has not popped, no exception flagged) and, restarted after the handler, store
; the value and pop exactly once. Chris's 3D Benchmark (BENCH.EXE, DJGPP under CWSDPMI, which
; maps a page on its first touch) stopped with "Floating Point exception at eip=00003df5", the
; instruction after an FSTP m32 into its 770 KB array.
;
; Each case points EBX into the absent page (DATA_BASE + 0x1000); the #PF handler checks CR2
; and the faulting EIP, records the status word, points EBX into the mapped page and returns
; to the same instruction. Then the case checks the handler ran once, the status word at the
; fault (TOP as before the store, IE clear), the status after (TOP, IE clear) and the stored
; bytes. The control word masks everything, so a stack fault shows as IE instead of an
; exception. The first case is BENCH.EXE's sequence: two products, FSUBP (DE E9), an integer
; move, FSTP m32.

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

    ; 1: BENCH.EXE 3DD1..3DF2: a*b - c*d stored as m32 (FSTP pops to TOP 0)
    fld dword [cs:fa]
    fmul dword [cs:fb]
    fld dword [cs:fc]
    fmul dword [cs:fd]
    fsubp st1, st0                          ; DE E9
    mov eax, esi
    STORE_CASE 1, {fstp dword [ebx]}, 7, 0, 0, RES
    mov eax, 0x40001
    SAME RES, r_m32, 4

    ; 2: FST m32 (no pop)
    fld dword [cs:fa]
    STORE_CASE 2, {fst dword [ebx]}, 7, 7, 0x10, RES + 4
    mov eax, 0x40002
    SAME RES + 4, fa, 4
    fstp st0

    ; 3: FSTP m64
    fld qword [cs:q64]
    STORE_CASE 3, {fstp qword [ebx]}, 7, 0, 0x20, RES + 8
    mov eax, 0x40003
    SAME RES + 8, q64, 8

    ; 4: FSTP m80
    fld tword [cs:t80]
    STORE_CASE 4, {fstp tword [ebx]}, 7, 0, 0x30, R80
    mov eax, 0x40004
    SAME R80, t80, 10

    ; 5: FISTP m32
    fild dword [cs:i32]
    STORE_CASE 5, {fistp dword [ebx]}, 7, 0, 0x40, RES + 16
    mov eax, 0x40005
    SAME RES + 16, i32, 4

    ; 6: FISTP m16
    fild word [cs:i16]
    STORE_CASE 6, {fistp word [ebx]}, 7, 0, 0x50, RES + 20
    mov eax, 0x40006
    SAME RES + 20, i16, 2

    ; 7: FISTP m64
    fild qword [cs:i64]
    STORE_CASE 7, {fistp qword [ebx]}, 7, 0, 0x60, RES + 24
    mov eax, 0x40007
    SAME RES + 24, i64, 8

    ; 8: FIST m32 (no pop)
    fild dword [cs:i32]
    STORE_CASE 8, {fist dword [ebx]}, 7, 7, 0x70, RES + 32
    mov eax, 0x40008
    SAME RES + 32, i32, 4
    fstp st0

    ; 9: FSTP m32 right after an arithmetic result, two deep
    fld dword [cs:fa]
    fld dword [cs:fc]
    fadd st0, st1
    STORE_CASE 9, {fstp dword [ebx]}, 6, 7, 0x80, RES + 36
    mov eax, 0x40009
    SAME RES + 36, r_sum, 4
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
cw:     dw 0x0372
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


