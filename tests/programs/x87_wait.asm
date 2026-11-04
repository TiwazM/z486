; WAIT (9B) in 16-bit code: in place when the x87 has nothing to report.
;
; - With the x87 idle, WAIT completes like a NOP: 32 of them take under 3
;   cycles each (the original routine needs 6).
; - Behind FDIV, FSQRT and a direct FDIV m64 it holds until the x87 has
;   finished: the cycle counter (port FCh) has moved on by the divide's time.
;   Behind FDIV with FADD held in x87_control and FMUL in the bridge (the
;   overlap) it waits for all three. The results are those without WAIT.
; - A 16-bit compiler's code (FWAIT before every FPU instruction, FSTSW m16
;   and FWAIT before reading it) leaves the results the same code without
;   the FWAITs leaves.
; - CR0: TS without MP and EM do not stop WAIT; MP and TS together raise
;   #NM (the original routine), also when MOV CR0 sets them right before
;   the WAIT (beside its issue); CLTS right before it clears TS in time.
;
; The JSON pins how many WAITs completed in place, held, and ran the
; original routine.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
CYCLE_PORT  equ 0xFC
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

%macro STAMP 1                      ; %1 = the cycle counter
    mov dx, CYCLE_PORT
    in eax, dx
    mov %1, eax
%endmacro

; The same code with and without an FWAIT before every FPU instruction: FW
; is defined as fwait, or as nothing. Results at %1 (19 bytes): m64, m32
; integer, m32, status word, the branch taken.
%macro COMPILED 1
    FW
    fninit
    FW
    fld qword [val_a]
    FW
    fmul qword [val_b]
    FW
    fadd qword [val_c]
    FW
    fld st0
    FW
    fsqrt
    FW
    fdivp st1, st0
    FW
    fild word [val_i16]
    FW
    fmulp st1, st0
    FW
    fld dword [val_d]
    FW
    fcom st1
    FW
    fnstsw word [%1 + 16]
    FW
    mov ax, [%1 + 16]
    sahf
    jb %%below
    mov byte [%1 + 18], 1
    jmp %%done
%%below:
    mov byte [%1 + 18], 2
%%done:
    FW
    fxch st1
    FW
    fistp dword [%1 + 8]
    FW
    fstp qword [%1]
    FW
    fild dword [val_i32]
    FW
    fchs
    FW
    fstp dword [%1 + 12]
    FW
%endmacro

start:
    cli
    cld
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0xf000
    xor ax, ax
    mov es, ax
    mov word [es:7 * 4], nm_handler
    mov [es:7 * 4 + 2], cs
    mov word [es:16 * 4], mf_handler
    mov [es:16 * 4 + 2], cs
    mov ax, cs
    mov es, ax

    ; ---- the x87 idle: WAIT as a NOP ----------------------------------
    mov word [step], 1
    fninit
    STAMP ebx
    STAMP ecx                       ; an empty bracket: the stamps' own cost
    sub ecx, ebx
    STAMP ebx
    times 32 fwait
    STAMP eax
    sub eax, ebx
    sub eax, ecx
    cmp eax, 3 * 32
    jae fail

    ; ---- behind FDIV and FSQRT: held until they are done --------------
    mov word [step], 2
    fninit
    fld qword [val_y]
    fld qword [val_x]
    fdiv st0, st1
    fstp qword [ref]
    fld st0
    fsqrt
    fstp qword [ref + 8]
    fninit
    fld qword [val_y]
    fld qword [val_x]
    STAMP ebx
    fdiv st0, st1
    fwait
    STAMP eax
    sub eax, ebx
    cmp eax, 60                     ; the divide runs 61 cycles
    jb fail
    fstp qword [res]
    fld st0
    STAMP ebx
    fsqrt
    fwait
    STAMP eax
    sub eax, ebx
    cmp eax, 40
    jb fail
    fstp qword [res + 8]
    fstp st0
    mov si, res
    mov di, ref
    mov cx, 16
    repe cmpsb
    jne fail

    ; A divide by a memory operand goes to the x87 through the direct
    ; queue: the WAIT waits for it as well.
    mov word [step], 3
    fninit
    fld qword [val_x]
    fdiv qword [val_y]
    fstp qword [ref]
    fninit
    fld qword [val_x]
    STAMP ebx
    fdiv qword [val_y]
    fwait
    STAMP eax
    sub eax, ebx
    cmp eax, 60
    jb fail
    fstp qword [res]
    mov si, res
    mov di, ref
    mov cx, 8
    repe cmpsb
    jne fail

    ; ---- the overlap: FDIV runs, FADD held, FMUL in the bridge --------
    mov word [step], 4
    fninit
    fld qword [val_y]
    fld qword [val_x]
    fdiv st0, st1
    fadd st0, st0
    fmul st0, st0
    fstp qword [ref]
    fstp qword [ref + 8]
    fninit
    fld qword [val_y]
    fld qword [val_x]
    STAMP ebx
    fdiv st0, st1
    fadd st0, st0
    fmul st0, st0
    fwait
    STAMP eax
    sub eax, ebx
    cmp eax, 70                     ; all three ran
    jb fail
    fnstsw ax
    and ax, 0x38ff
    cmp ax, 0x3020                  ; TOP 6, PE, nothing pending
    jne fail
    fstp qword [res]
    fstp qword [res + 8]
    mov si, res
    mov di, ref
    mov cx, 16
    repe cmpsb
    jne fail

    ; ---- a 16-bit compiler's code: FWAIT before every FPU instruction --
    mov word [step], 5
%define FW fwait
    COMPILED res_w
%undef FW
%define FW
    COMPILED res_n
%undef FW
    mov si, res_w
    mov di, res_n
    mov cx, 19
    repe cmpsb
    jne fail
    cmp byte [res_w + 18], 2        ; 3.0 below the product
    jne fail
    cmp dword [res_w + 8], 14
    jne fail
    cmp dword [res_w + 12], 0x47f12000 ; 123456.0
    jne fail

    ; ---- CR0 -------------------------------------------------------------
    mov word [step], 6
    fninit
    mov eax, cr0
    mov [cr0_saved], eax
    and eax, 0xfffffff1             ; MP, EM, TS clear
    or eax, 0x08                    ; TS without MP: no #NM
    mov cr0, eax
    mov cx, 3                       ; (the CR0 write is done)
    fwait
    fwait
    and eax, 0xfffffff1
    or eax, 0x06                    ; EM and MP: no #NM
    mov cr0, eax
    mov cx, 3
    fwait
    fwait
    cmp word [nm_count], 0
    jne fail

    mov word [step], 7
    and eax, 0xfffffff1
    or eax, 0x0a                    ; MP and TS: #NM, the handler clears TS
    mov cr0, eax
    mov cx, 3
nm_wait1:
    fwait
    cmp word [nm_count], 1
    jne fail
    cmp word [nm_ip], nm_wait1
    jne fail

    ; MOV CR0 right before the WAIT: its write is beside the WAIT's issue.
    mov word [step], 8
    mov eax, cr0
    or eax, 0x0a
    mov cr0, eax
nm_wait2:
    fwait
    cmp word [nm_count], 2
    jne fail
    cmp word [nm_ip], nm_wait2
    jne fail

    ; CLTS right before it: TS is clear in time.
    mov word [step], 9
    mov eax, cr0
    or eax, 0x0a
    mov cr0, eax
    mov cx, 3
    clts
    fwait
    cmp word [nm_count], 2
    jne fail
    mov eax, [cr0_saved]
    mov cr0, eax
    fninit
    fwait
    cmp word [mf_count], 0
    jne fail

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    mov dx, DATA_PORT
    mov ax, [step]
    out dx, ax
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

nm_handler:                         ; #NM: record where, clear TS
    push bp
    mov bp, sp
    inc word [nm_count]
    push ax
    mov ax, [bp + 2]
    mov [nm_ip], ax
    pop ax
    pop bp
    clts
    iret

mf_handler:                         ; no exception is pending here
    inc word [mf_count]
    fnclex
    iret

align 8
val_x:        dq 0.7
val_y:        dq 2.5
val_a:        dq 1.5
val_b:        dq 2.25
val_c:        dq 0.75
val_d:        dd 3.0
val_i32:        dd -123456
val_i16:        dw 7
ref:        dq 0, 0
res:        dq 0, 0
res_w:      times 20 db 0
res_n:      times 20 db 0xff
cr0_saved:  dd 0
step:       dw 0
nm_count:   dw 0
nm_ip:      dw 0
mf_count:   dw 0
