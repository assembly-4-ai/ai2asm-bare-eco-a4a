; idt64_uefi.asm: IDT Setup for Exceptions (post-UEFI)
; Depends on: boot_defs_temp.inc

BITS 64
global setup_final_idt64_exceptions
global idt64_pointer_var ; Variable holding IDTR value
global setup_final_idt64
global load_idt64
global setup_final_idt64
extern panic64           ; For common ISR handler

%include "boot_defs_temp.inc" ; Needs CODE64_SEL

struc Idt64Entry
    .OffsetLow      resw 1
    .Selector       resw 1
    .IST            resb 1  ; Interrupt Stack Table index
    .TypeAttr       resb 1
    .OffsetMid      resw 1
    .OffsetHigh     resd 1
    .Reserved       resd 1
endstruc
Idt64Entry_size equ 16

; Gate Types for IDT Entry's TypeAttr field
IDT_TYPE_INT_GATE   equ 0x8E ; Interrupt Gate, P=1, DPL=0, Type=0xE
IDT_TYPE_TRAP_GATE  equ 0x8F ; Trap Gate, P=1, DPL=0, Type=0xF

section .data align=8
idt64_pointer_var:
    dw 0 ; Limit (set at runtime)
    dq 0 ; Base (set at runtime)

isr_panic_msg db "Unhandled Exception! ISR: 0x", 0

; Table of ISR Stub addresses (only first 32 for exceptions)
isr_stub_table:
    dq isr0_stub, isr1_stub, isr2_stub, isr3_stub, isr4_stub
    dq isr5_stub, isr6_stub, isr7_stub, isr8_stub, isr9_stub
    dq isr10_stub, isr11_stub, isr12_stub, isr13_stub, isr14_stub
    dq isr15_stub
    dq isr16_stub, isr17_stub, isr18_stub, isr19_stub
    dq isr_generic_stub, isr_generic_stub, isr_generic_stub, isr_generic_stub ; 20-23
    dq isr_generic_stub, isr_generic_stub, isr_generic_stub, isr_generic_stub ; 24-27
    dq isr_generic_stub, isr_generic_stub, isr_generic_stub, isr_generic_stub ; 28-31

; Attributes for first 20 exceptions (matching table above)
; Use Trap Gate for Debug, Breakpoint; Interrupt Gate otherwise
isr_attrs:
    db IDT_TYPE_INT_GATE, IDT_TYPE_TRAP_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_TRAP_GATE ; 0-3
    db IDT_TYPE_TRAP_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE ; 4-7
    db IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE ; 8-11
    db IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE ; 12-15
    db IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE, IDT_TYPE_INT_GATE ; 16-19
    ; Rest are generic, use INT_GATE
isr_ist: ; Interrupt Stack Table index (0 = default stack)
    db 0, 0, 1, 0, 0, 0, 0, 0, 1, 0 ; Use IST 1 for NMI (2) and DF (8)
    db 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; IST 0 for others

section .text

;--------------------------------------------------------------------------
; setup_final_idt64_exceptions: Sets up IDT entries for ISRs 0-31.
; Input:
;   RDI = Physical address of allocated memory for IDT (IDT_MAX_ENTRIES * 16 bytes)
; Output: None, IDT written at [RDI], idt64_pointer_var updated
; Destroys: RAX, RBX, RCX, RDX, RDI, RSI
;--------------------------------------------------------------------------
setup_final_idt64_exceptions:
    push rdi ; Save IDT base address

    ; Zero the entire IDT table first
    mov rcx, IDT_MAX_ENTRIES * Idt64Entry_size
    xor al, al
    rep stosb ; Simple zeroing

    pop rdi ; Restore IDT base address
    mov rsi, rdi ; RSI = Current IDT entry pointer

    ; Setup exceptions 0-31 using the table
    xor rcx, rcx ; ISR number / table index
.isr_loop:
    cmp rcx, 32 ; Only setup first 32 exceptions
    jae .isr_loop_done

    mov rax, [isr_stub_table + rcx*8] ; Get stub address
    ; Determine attributes and IST (use defaults if beyond specific tables)
    cmp rcx, 20
    jae .use_generic_attr
    mov bl, [isr_attrs + rcx] ; TypeAttr
    mov dl, [isr_ist + rcx]   ; IST index
    jmp .set_entry
.use_generic_attr:
    mov bl, IDT_TYPE_INT_GATE ; Default attribute
    xor dl, dl                ; Default IST (0)

.set_entry:
    ; Deconstruct handler address (RAX) and write to entry (pointed by RSI)
    mov word [rsi + Idt64Entry.OffsetLow], ax
    mov word [rsi + Idt64Entry.Selector], CODE64_SEL
    mov byte [rsi + Idt64Entry.IST], dl          ; IST index
    mov byte [rsi + Idt64Entry.TypeAttr], bl     ; Type and attributes
    shr rax, 16
    mov word [rsi + Idt64Entry.OffsetMid], ax
    shr rax, 16
    mov dword [rsi + Idt64Entry.OffsetHigh], eax
    mov dword [rsi + Idt64Entry.Reserved], 0     ; Reserved field must be 0

    add rsi, Idt64Entry_size ; Move to next IDT entry
    inc rcx
    jmp .isr_loop
.isr_loop_done:

    ; Update IDTR variable
    mov rax, rdi ; IDT Base address
    mov word [idt64_pointer_var], (IDT_MAX_ENTRIES * Idt64Entry_size - 1) ; Limit
    mov [idt64_pointer_var + 2], rax ; Base

    ret

; --- ISR Stubs (push ISR number, push error code if applicable) ---
; Error codes pushed by CPU for: 8, 10, 11, 12, 13, 14, 17, 21, 29, 30
%macro ISR_STUB 2-3 0 ; Params: ISR_Num, PushErrorCode (0 or 1), ErrorCodeValue (optional)
isr%1_stub:
    %if %2 == 0
        push qword %3 ; Push dummy error code (or specific value)
    %endif
    push %1 ; Push ISR number
    jmp common_isr_handler
%endmacro

ISR_STUB 0, 0 ; Divide Error
ISR_STUB 1, 0 ; Debug
ISR_STUB 2, 0 ; NMI
ISR_STUB 3, 0 ; Breakpoint
ISR_STUB 4, 0 ; Overflow
ISR_STUB 5, 0 ; Bound Range Exceeded
ISR_STUB 6, 0 ; Invalid Opcode
ISR_STUB 7, 0 ; Device Not Available
ISR_STUB 8, 1 ; Double Fault (Pushes Error Code)
ISR_STUB 9, 0 ; Coprocessor Segment Overrun
ISR_STUB 10, 1 ; Invalid TSS (Pushes Error Code)
ISR_STUB 11, 1 ; Segment Not Present (Pushes Error Code)
ISR_STUB 12, 1 ; Stack-Segment Fault (Pushes Error Code)
ISR_STUB 13, 1 ; General Protection Fault (Pushes Error Code)
ISR_STUB 14, 1 ; Page Fault (Pushes Error Code)
ISR_STUB 15, 0, -1 ; Reserved, push -1 as dummy code
ISR_STUB 16, 0 ; x87 Floating-Point Exception
ISR_STUB 17, 1 ; Alignment Check (Pushes Error Code)
ISR_STUB 18, 0 ; Machine Check
ISR_STUB 19, 0 ; SIMD Floating-Point Exception
; ISRs 20-31 are Reserved or used for virtualization/security

; Generic stub for other vectors (pushes vector number)
isr_generic_stub:
    ; We get here for vectors 20-31 if they occur
    push 0 ; Dummy error code
    push -1 ; Placeholder vector number
    jmp common_isr_handler

;--------------------------------------------------------------------------
; common_isr_handler: Common handler for CPU exceptions
; Stack state on entry seen by C: struct InterruptFrame* frame;
; Stack state on assembly entry:
;   [RSP+16] ISR Number
;   [RSP+8]  Error Code (or dummy)
;   [RSP]    Return RIP (pushed by CPU interrupt)
;   [RSP-8]  Return CS
;   [RSP-16] Return RFLAGS
;   [RSP-24] Return RSP
;   [RSP-32] Return SS
;--------------------------------------------------------------------------
common_isr_handler:
    ; Save all general purpose registers
    push r15; push r14; push r13; push r12
    push r11; push r10; push r9;  push r8
    push rbp; push rdi; push rsi
    push rdx; push rcx; push rbx; push rax

    ; Pass ISR number (now at RSP + 15*8 + 16) and error code (RSP + 15*8 + 8) to panic
    mov rdi, [rsp + 15*8 + 16] ; ISR Number
    mov rsi, [rsp + 15*8 + 8]  ; Error Code
    ; Optional: Pass stack frame pointer (RSP before pushes)
    ; mov rdx, rsp
    ; add rdx, 15*8 + 24 ; Calculate original RSP where RIP was pushed
    ; Call panic64(isr_num, error_code)
    call panic64 ; panic64 needs to work post-ExitBS and print RDI/RSI

    ; Halt after panic - panic64 should not return
.halt_isr:
    cli
    hlt
    jmp .halt_isr