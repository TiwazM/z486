; FNSTSW AX, FNSTSW m16 and FNSTCW right behind the x87 instruction whose
; result they read, with 0 to 3 NOPs between: loads of every format (TOP),
; compares (C3 C2 C0, the C idiom FCOM; FNSTSW AX; SAHF), arithmetic that
; flags PE or IE when it completes, a store's pop, and FLDCW. The status read
; must wait for the instruction before it, however it was sent to the x87
; (m32 path, RD-word operand, register command, store overlay).

BITS 16
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

; Run %1, %2 NOPs, FNSTSW AX; AX AND %3 must be %4.
%macro STS 4
    %1
    times %2 nop
    fnstsw ax
    and ax, %3
    cmp ax, %4
    jne fail
%endmacro

; The same with FNSTSW m16.
%macro STSM 4
    %1
    times %2 nop
    fnstsw [out16]
    mov ax, [out16]
    and ax, %3
    cmp ax, %4
    jne fail
%endmacro

start:
    cli
    cld
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x700

%assign n 0
%rep 4
    fninit
    STS {fld qword [two]}, n, 0x3800, 0x3800        ; TOP 7
    STS {fld qword [one]}, n, 0x3800, 0x3000        ; TOP 6: 1, 2
    STS {fcom st1}, n, 0x7dff, 0x3100               ; 1 < 2: C0
    STS {fcom qword [one]}, n, 0x7dff, 0x7000       ; equal: C3
    STS {fcom dword [half32]}, n, 0x7dff, 0x3000    ; greater
    STS {ficom word [i16_one]}, n, 0x7dff, 0x7000   ; equal
    STS {ftst}, n, 0x7dff, 0x3000                   ; greater than 0
    STS {fcomp qword [two]}, n, 0x7dff, 0x3900      ; less, popped: TOP 7
    STSM {fcom qword [three]}, n, 0x7dff, 0x3900    ; 2 < 3
    STSM {fcom st0}, n, 0x7dff, 0x7800              ; equal
    STS {fild word [i16_one]}, n, 0x3800, 0x3000
    STS {fild dword [i32_v]}, n, 0x3800, 0x2800
    STS {fild qword [i64_v]}, n, 0x3800, 0x2000
    STS {fld tword [pi80]}, n, 0x3800, 0x1800
    STS {fld dword [half32]}, n, 0x3800, 0x1000
    STS {fld st0}, n, 0x3800, 0x0800
    STSM {fld1}, n, 0x3800, 0x0000                  ; all eight in use
    STS {fdiv qword [three]}, n, 0x38ff, 0x0020     ; 1/3: PE
    STS {fsin}, n, 0x3cff, 0x0020                   ; C2 clear
    fninit
    fld qword [one]
    fchs
    STS {fsqrt}, n, 0x38ff, 0x3801                  ; sqrt(-1): IE
    fninit
    fld qword [three]
    STS {fstp qword [out64]}, n, 0x38ff, 0x0000     ; popped, exact
    cmp word [out64 + 6], 0x4008
    jne fail
    fld qword [three]
    fld qword [one]
    STS {fdiv st0, st1}, n, 0x38ff, 0x3020          ; 1/3 in ST0: PE
    STS {fcomp st1}, n, 0x7dff, 0x3920              ; 1/3 < 3, popped
    STSM {fistp word [out16]}, n, 0x38ff, 0x0020    ; 3, popped
    fnclex
    fldcw [cw_a]
    times n nop
    fnstcw [out16]
    cmp word [out16], 0x0c7f
    jne fail
    fldcw [cw_def]
    times n nop
    fnstcw [out16]
    cmp word [out16], 0x037f
    jne fail
    STS {fldcw [cw_a]}, n, 0x38ff, 0x0000
    fldcw [cw_def]
%assign n n + 1
%endrep

pass:
    mov al, STATUS_PASS
    jmp report

fail:
    mov al, STATUS_FAIL

report:
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 8
one:     dq 0x3ff0000000000000
two:     dq 0x4000000000000000
three:   dq 0x4008000000000000
i64_v:   dq 7
pi80:    dq 0xc90fdaa22168c235
         dw 0x4000
half32:  dd 0x3f000000
i32_v:   dd 5
i16_one: dw 1
cw_a:    dw 0x0c7f
cw_def:  dw 0x037f
align 8
out64:   dq 0
out16:   dw 0
