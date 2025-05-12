; pci.asm: PCI Bus Scan (Legacy I/O Method)
; Depends on: boot_defs_temp.inc

BITS 64
%include "boot_defs_temp.inc" ; ** Ensure this is included **

global pci_init, pci_find_ahci_controller
extern scr64_print_string, scr64_print_hex, scr64_print_dec ; ** Added Externs **
extern panic64 ; If errors need to halt

section .data align=8 ; Use 8-byte alignment for data section
pci_scan_msg    db "Scanning PCI Bus for AHCI...", 0Dh, 0Ah, 0
pci_found_msg   db "AHCI Controller Found: Bus=", 0
pci_dev_msg     db ", Dev=", 0
pci_func_msg    db ", Func=", 0
pci_bar5_msg    db ", BAR5 Addr=0x", 0
pci_done_msg    db 0Dh, 0Ah, 0
pci_not_found_msg db "AHCI Controller Not Found!", 0Dh, 0Ah, 0

section .text

;--------------------------------------------------------------------------
; pci_read_config_dword: Read a DWORD from PCI config space (Legacy IO)
; Input: R8B=bus, R9B=device, R10B=func, R11B=reg offset (dword aligned)
; Output: EAX = value read
; Destroys: EAX, EDX, ECX (low part)
;--------------------------------------------------------------------------
pci_read_config_dword:
    mov eax, 0x80000000 ; Enable bit
    mov cl, r8b ; Bus number
    shl ecx, 16
    or eax, ecx ; Add bus
    mov cl, r9b ; Device number
    shl ecx, 11
    or eax, ecx ; Add device
    mov cl, r10b ; Function number
    shl ecx, 8
    or eax, ecx ; Add function
    and r11b, 0xFC ; Clear lower 2 bits of register
    or al, r11b ; Add register offset

    mov dx, PCI_CONFIG_ADDRESS ; Use defined constant
    out dx, eax ; Write address

    mov dx, PCI_CONFIG_DATA ; Use defined constant
    in eax, dx ; Read data
    ret

;--------------------------------------------------------------------------
; pci_init: Placeholder
;--------------------------------------------------------------------------
pci_init:
    ret ; Nothing needed currently

;--------------------------------------------------------------------------
; pci_find_ahci_controller: Scan PCI bus for AHCI controller using Class Code
; Output: RAX = AHCI MMIO Base Address (BAR5), or 0 if not found. Carry set on error.
; Destroys: RAX, RBX, RCX, RDX, RSI, RDI, R8-R11
;--------------------------------------------------------------------------
pci_find_ahci_controller:
    push r12; push r13; push r14; push r15 ; Save non-volatiles

    mov rsi, pci_scan_msg
    call scr64_print_string

    mov r8, 0 ; Bus
pci_bus_loop: ; ** Use unique label **
    cmp r8, 256
    jge pci_not_found ; ** Use unique label **

    mov r9, 0 ; Device
pci_device_loop: ; ** Use unique label **
    cmp r9, 32
    jge pci_next_bus ; ** Use unique label **

    mov r10, 0 ; Function
pci_function_loop: ; ** Use unique label **
    cmp r10, 8
    jge pci_next_device ; ** Use unique label **

    ; Read Vendor/Device ID
    mov r11b, 0x00
    call pci_read_config_dword
    cmp ax, 0xFFFF
    je pci_next_function ; ** Use unique label **

    ; Read Class Code / Subclass / Prog IF
    mov r11b, 0x08
    call pci_read_config_dword
    shr eax, 8

    cmp al, PCI_PROGIF_AHCI; jne pci_next_function
    shr eax, 8
    cmp al, PCI_SUBCLASS_SATA; jne pci_next_function
    shr eax, 8
    cmp al, PCI_CLASS_MASS_STORAGE; jne pci_next_function

    ; --- AHCI Controller Found! ---
    mov rsi, pci_found_msg; call scr64_print_string
    mov rax, r8; call scr64_print_dec
    mov rsi, pci_dev_msg; call scr64_print_string
    mov rax, r9; call scr64_print_dec
    mov rsi, pci_func_msg; call scr64_print_string
    mov rax, r10; call scr64_print_dec

    ; Read BAR5
    mov r11b, 0x24
    call pci_read_config_dword
    ; RAX = BAR5 value

    ; Basic check: Memory mapped (bit 0 = 0) and non-zero
    test al, 1; jnz pci_not_found ; Must be memory mapped
    and eax, 0xFFFFFFF0 ; Mask flags
    test eax, eax; jz pci_not_found ; Must be non-zero

    mov rsi, pci_bar5_msg; call scr64_print_string
    ; RAX already contains the base address to print and return
    call scr64_print_hex
    mov rsi, pci_done_msg; call scr64_print_string
    clc ; Success
    jmp pci_done ; ** Use unique label **

pci_next_function: ; ** Use unique label **
    cmp r10, 0; jne pci_inc_func ; Only check header type for function 0
    mov r11b, 0x0C; call pci_read_config_dword; shr eax, 16; test al, 0x80
    jz pci_next_device ; Not multi-function, skip funcs 1-7
pci_inc_func: ; ** Use unique label **
    inc r10
    jmp pci_function_loop

pci_next_device: ; ** Use unique label **
    inc r9
    jmp pci_device_loop

pci_next_bus: ; ** Use unique label **
    inc r8
    jmp pci_bus_loop

pci_not_found: ; ** Use unique label **
    mov rsi, pci_not_found_msg
    call scr64_print_string
    xor rax, rax ; Return 0
    stc ; Set carry flag for error
pci_done: ; ** Use unique label **
    pop r15; pop r14; pop r13; pop r12
    ret