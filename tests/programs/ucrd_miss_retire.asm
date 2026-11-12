; ucrd_miss_retire.asm - a microcode read that misses L1 must not retire with a stale OPR_R
;
; CorePlus (Linux 6.x, whole-PC sim, 2026-11-12): `mov eax, fs:[current_task]` (64 A1) right
; after a RET returned ESI's old value, a later NULL-pointer oops at address 0C17h. The A1 ran on
; the microcode read path (ua 019/01A), its translation was known good (the sidecar held the
; page), and its line had been evicted from L1. The read resolved as a miss with a fill from the
; probe while the RNI issued the next instruction in the same cycle; the hardwired MOV's deferred
; register write copied OPR_R (still the previous read's value) into EAX, and the CALL after it
; flushed the queue, which dropped the fill wait, so OPR_R never got the line's data either.
;
; Each round evicts the target line through its set (8 conflicting lines, 4-way L1), keeps the
; page in the TLB through another line of it, leaves a known value in OPR_R with a read, and RETs
; into the load, followed by a CALL or a branch. Variants:
;   1-4  A1 + CALL, 0-3 NOPs before the RET          5-7  A1 + CMP/JNE (not taken)
;   8    A1 + taken JE                               9    MOV EAX,FS:[EDX] + CALL
;   10   MOVZX EAX,WORD FS:[moffs] + CALL            11   ADD EAX,FS:[moffs] + CALL
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4: rounds left << 8 | variant, EAX.

BITS 32
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

TARGET      equ 0x100C              ; fs: (base 0x21000000 -> phys 0x41000), L1 set 0
TLB_TOUCH   equ 0x1100              ; same page, set 0x10
VALUE       equ 0xC1035D80
STALE       equ 0x00000C17
EAX_INIT    equ 0x5A5A5A5A
ROUNDS      equ 24

%macro EVICT 0
    mov ecx, [0x0000]               ; DS lines in set 0 (stride 0x800)
    mov ecx, [0x0800]
    mov ecx, [0x1000]
    mov ecx, [0x1800]
    mov ecx, [0x2000]
    mov ecx, [0x2800]
    mov ecx, [0x3000]
    mov ecx, [0x3800]
%endmacro

; %1 variant, %2 NOPs before the RET, %3 expected EAX, %4 the load, %5: 0 CALL, 1 CMP/JNE, 2 CMP/JE
%macro ROUND 5
    EVICT
    mov ecx, [fs:TLB_TOUCH]         ; the target's page in the TLB
    mov esi, STALE
    mov [0x4000], esi
    mov esi, [0x4000]               ; a read whose value OPR_R may keep
    mov eax, EAX_INIT
    mov edx, TARGET
    mov ecx, %3
    mov bl, %1
    push dword %%target
    times %2 nop
    ret
%%target:
    %4
%if %5 == 0
    call check
%elif %5 == 1
    cmp eax, ecx
    jne fail
%else
    cmp eax, ecx
    je %%ok
    jmp fail
%%ok:
%endif
%endmacro

start:
    mov esp, 0x8000
    mov dword [fs:TARGET], VALUE
    mov dword [fs:TLB_TOUCH], 0x11001100
    times 16 nop
    mov edi, ROUNDS
.loop:
    ROUND 1, 0, VALUE, {mov eax, [fs:TARGET]}, 0
    ROUND 2, 1, VALUE, {mov eax, [fs:TARGET]}, 0
    ROUND 3, 2, VALUE, {mov eax, [fs:TARGET]}, 0
    ROUND 4, 3, VALUE, {mov eax, [fs:TARGET]}, 0
    ROUND 5, 0, VALUE, {mov eax, [fs:TARGET]}, 1
    ROUND 6, 1, VALUE, {mov eax, [fs:TARGET]}, 1
    ROUND 7, 2, VALUE, {mov eax, [fs:TARGET]}, 1
    ROUND 8, 0, VALUE, {mov eax, [fs:TARGET]}, 2
    ROUND 9, 0, VALUE, {mov eax, [fs:edx]}, 0
    ROUND 10, 0, (VALUE & 0xFFFF), {movzx eax, word [fs:TARGET]}, 0
    ROUND 11, 0, ((EAX_INIT + VALUE) & 0xFFFFFFFF), {add eax, [fs:TARGET]}, 0
    dec edi
    jnz .loop

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
hang:
    hlt
    jmp hang

check:
    cmp eax, ecx
    jne fail
    ret

fail:
    mov ebp, eax                    ; the value the load returned
    mov eax, edi
    shl eax, 8
    mov al, bl                      ; round count left, variant
    mov dx, DATA_PORT
    out dx, eax
    mov eax, ebp
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    jmp hang
