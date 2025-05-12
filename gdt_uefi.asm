; gdt_uefi.asm: GDT Setup for post-UEFI 64-bit
; Depends on: boot_defs_temp.inc

BITS 64
%include "boot_defs_temp.inc" ; Needs CODE64_SEL, DATA64_SEL

global setup_final_gdt64
global load_gdt_and_segments64
global gdt64_pointer_var    ; Variable holding the GDTR value

section .data align=8
gdt64_pointer_var:
    dw 0 ; Limit (set at runtime)
    dq 0 ; Base (set at runtime)

section .text
;--------------------------------------------------------------------------
; setup_final_gdt64: Writes GDT descriptors to allocated memory.
; Input:
;   RDI = Physical address of allocated memory for GDT (at least 40 bytes)
; Output: None, GDT written at [RDI], gdt64_pointer_var updated
; Destroys: RAX
;--------------------------------------------------------------------------
setup_final_gdt64:
    mov rax, rdi ; RAX = GDT base physical address

    ; Null descriptor (index 0)
    mov qword [rax], 0

    ; Code64: Selector CODE64_SEL (0x18 -> index 3)
    ; Base=0, Limit=4G, Type=Code, Seg=1, DPL=0, Pres=1, L=1, D/B=0, Gran=1
    mov qword [rax + CODE64_SEL], 0x00AF9A00

    ; Data64: Selector DATA64_SEL (0x20 -> index 4)
    ; Base=0, Limit=4G, Type=Data, Seg=1, DPL=0, Pres=1, L=0, D/B=0, Gran=1
    mov qword [rax + DATA64_SEL], 0x00CF9200

    ; Update GDTR variable
    mov word [gdt64_pointer_var], (DATA64_SEL + 8 - 1) ; Limit = size - 1
    mov [gdt64_pointer_var + 2], rax ; Base address

    ret

;--------------------------------------------------------------------------
; load_gdt_and_segments64: Loads the GDT and refreshes segment registers
; Assumes gdt64_pointer_var is correctly populated.
; Call this AFTER ExitBootServices
;--------------------------------------------------------------------------
load_gdt_and_segments64:
    lgdt [gdt64_pointer_var]
    ; Reload CS via far jump/ret
    ; Need to push RSP value that will be correct AFTER iretq cleans stack
    push DATA64_SEL ; Target SS (Data segment is fine)
    mov rax, rsp    ; Get current RSP
    add rax, 8*4    ; Calculate RSP after pushes and before IRETQ pops
    push rax        ; Push target RSP
    pushfq          ; Push RFLAGS
    push CODE64_SEL ; Push target CS
    lea rax, [rel .reload_cs] ; Push target RIP
    push rax
    iretq           ; Load CS, SS, RSP, RFLAGS, RIP
.reload_cs:
    ; Now running with new CS
    ; Reload data segments
    mov ax, DATA64_SEL
    mov ds, ax
    mov es, ax
    mov fs, ax ; Or set FS/GS to 0 or specific values later
    mov gs, ax
    ; SS was loaded by IRETQ
    ret