; apic.asm: Basic APIC Initialization and EOI (64-bit)
; Depends on: boot_defs_temp.inc

BITS 64
%include "boot_defs_temp.inc" ; ** Only include needed **

global apic_init, lapic_send_eoi, ioapic_unmask_irq
extern map_page           ; From paging module
extern panic64
extern scr64_print_string ; ** Ensure this is correct **

section .data align=8
lapic_base_phys dq 0xFEE00000
lapic_base_virt dq 0
ioapic_base_phys dq 0xFEC00000
ioapic_base_virt dq 0

apic_init_msg db "Initializing APIC...", 0Dh, 0Ah, 0
apic_init_fail_msg db "APIC Init Failed!", 0Dh, 0Ah, 0
apic_init_ok_msg db "APIC Initialized OK.", 0Dh, 0Ah, 0

section .text

;--------------------------------------------------------------------------
lapic_read: ; ECX=Offset -> EAX=Value
    mov rax, [lapic_base_virt]; test rax, rax; jz .err_lapic_unmapped; add rax, rcx; mov eax, [rax]; ret
.err_lapic_unmapped: mov eax, 0xFFFFFFFF; ret
;--------------------------------------------------------------------------
lapic_write: ; ECX=Offset, EAX=Value
    mov rdx, [lapic_base_virt]; test rdx, rdx; jz .ret_lapic_unmapped; add rdx, rcx; mov [rdx], eax;
.ret_lapic_unmapped: ret
;--------------------------------------------------------------------------
lapic_send_eoi:
    push rax; push rcx; mov rcx, LAPIC_REG_EOI; mov eax, 0; call lapic_write; pop rcx; pop rax; ret
;--------------------------------------------------------------------------
ioapic_read: ; ECX=Index -> EAX=Value
    mov rdx, [ioapic_base_virt]; test rdx, rdx; jz .err_ioapic_unmapped; mov [rdx + IOAPIC_INDEX_REG], ecx; mov eax, [rdx + IOAPIC_DATA_REG]; ret
.err_ioapic_unmapped: mov eax, 0xFFFFFFFF; ret
;--------------------------------------------------------------------------
ioapic_write: ; ECX=Index, EAX=Value
    mov rdx, [ioapic_base_virt]; test rdx, rdx; jz .ret_ioapic_unmapped; mov [rdx + IOAPIC_INDEX_REG], ecx; mov [rdx + IOAPIC_DATA_REG], eax;
.ret_ioapic_unmapped: ret
;--------------------------------------------------------------------------
ioapic_write_rte: ; CL=IRQ Line (0-23), RDX:RAX = 64-bit RTE value
    movzx r8, cl; shl r8, 1; add r8, IOAPIC_REG_REDTBL_BASE; mov ecx, r8d; mov eax, eax; call ioapic_write ; Write low
    inc r8d; mov ecx, r8d; mov eax, edx; call ioapic_write ; Write high
    ret
;--------------------------------------------------------------------------
ioapic_mask_irq: ; CL = IRQ line (0-23)
    push rax; push rdx; push r8; push rcx
    movzx r8, cl; shl r8, 1; add r8, IOAPIC_REG_REDTBL_BASE; mov ecx, r8d; call ioapic_read; or eax, RTE_MASKED; call ioapic_write
    pop rcx; pop r8; pop rdx; pop rax; ret
;--------------------------------------------------------------------------
ioapic_unmask_irq: ; CL = IRQ line (0-23)
    push rax; push rdx; push r8; push rcx
    movzx r8, cl; shl r8, 1; add r8, IOAPIC_REG_REDTBL_BASE; mov ecx, r8d; call ioapic_read; and eax, ~RTE_MASKED; call ioapic_write
    pop rcx; pop r8; pop rdx; pop rax; ret
;--------------------------------------------------------------------------
disable_pic:
    mov al, 0xFF; out PIC1_DATA, al; out PIC2_DATA, al; ret ; Uses PIC constants
;--------------------------------------------------------------------------
apic_init: ; ** Main APIC Init Function **
    push rbx; push rcx; push rdx; push rsi; push rdi; push r8; push r9; push r10; push r11; push r12

    mov rsi, apic_init_msg; call scr64_print_string ; Uses EXTERN

    call disable_pic

    ; Find and Map LAPIC
    mov ecx, MSR_APIC_BASE; rdmsr; mov [lapic_base_phys], rax; and qword [lapic_base_phys], 0xFFFFFFFFFFFFF000
    or rax, MSR_APIC_BASE_ENABLE; wrmsr
    mov rdi, 0xFFFFFFFF80000000; mov rsi, [lapic_base_phys]; mov rdx, PTE_PRESENT | PTE_WRITABLE | PTE_XD | PTE_CD; call map_page ; Uses EXTERN
    test rax, rax; jnz .apic_init_fail_map_lapic ; Unique label
    mov [lapic_base_virt], rdi

    ; Find and Map I/O APIC (Using default, needs ACPI)
    mov rdi, 0xFFFFFFFF80001000; mov rsi, [ioapic_base_phys]; mov rdx, PTE_PRESENT | PTE_WRITABLE | PTE_XD | PTE_CD; call map_page ; Uses EXTERN
    test rax, rax; jnz .apic_init_fail_map_ioapic ; Unique label
    mov [ioapic_base_virt], rdi

    ; Initialize LAPIC
    mov ecx, LAPIC_REG_DFR; mov eax, 0xFFFFFFFF; call lapic_write
    mov ecx, LAPIC_REG_LDR; call lapic_read; and eax, 0x00FFFFFF; mov r8d, 1; shl r8d, 24; or eax, r8d; call lapic_write
    mov ecx, LAPIC_REG_TPR; xor eax, eax; call lapic_write
    mov ecx, LAPIC_REG_SPURIOUS; call lapic_read; or eax, LAPIC_SPURIOUS_ENABLE; or al, 0xFF; call lapic_write

    ; Initialize I/O APIC
    mov ecx, IOAPIC_REG_VER; call ioapic_read; mov r12d, eax; shr eax, 16; and eax, 0xFF
    mov r8, 0
.apic_init_mask_rte_loop: ; Unique label
    cmp r8, rax; jg .apic_init_mask_rte_done
    mov cl, r8b; call ioapic_mask_irq
    inc r8; jmp .apic_init_mask_rte_loop
.apic_init_mask_rte_done: ; Unique label

    ; Route Keyboard: IRQ 1 -> Vector 0x21 (Uses PIC1_IRQ_START)
    xor r9, r9; shl r9, RTE_DESTINATION_SHIFT ; Dest LAPIC ID = 0
    mov rax, (PIC1_IRQ_START + KB_IRQ) | RTE_DELIVERY_MODE_FIXED | RTE_TRIGGER_MODE_EDGE
    mov rdx, r9; mov cl, KB_IRQ; call ioapic_write_rte

    ; Route AHCI: IRQ 14 -> Vector 0x2E (Uses PIC1_IRQ_START)
    mov rax, (PIC1_IRQ_START + 14) | RTE_DELIVERY_MODE_FIXED | RTE_TRIGGER_MODE_LEVEL | RTE_INT_POLARITY_LOW
    mov rdx, r9; mov cl, 14; call ioapic_write_rte

    mov rsi, apic_init_ok_msg; call scr64_print_string ; Uses EXTERN
    xor rax, rax; clc; jmp .apic_init_init_done ; Unique label

.apic_init_fail_map_lapic: ; Unique label
.apic_init_fail_map_ioapic: ; Unique label
    mov rax, 2; stc; mov rsi, apic_init_fail_msg; call scr64_print_string ; Uses EXTERN
.apic_init_init_done: ; Unique label
    pop r12; pop r11; pop r10; pop r9; pop r8; pop rdi; pop rsi; pop rdx; pop rcx; pop rbx
    ret