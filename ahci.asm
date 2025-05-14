;**File 12: `ahci.asm` (Structural Implementation)**

;```assembly
; ahci.asm: AHCI Driver (64-bit, Interrupt-Driven)
; Depends on: boot_defs_temp.inc (with AHCI defs), pmm64_uefi.asm

BITS 64
global ahci_init, ahci_read_sectors, ahci_write_sectors, ahci_flush_cache
global ahci_irq_handler
extern pmm_alloc_frame, pmm_free_frame ; For allocating command structures
extern scr64_print_string, scr64_print_hex, panic64
extern memcpy64, memset64 ; Assumed utilities

%include "boot_defs_temp.inc" ; Includes AHCI defs

section .data align=64
ahci_abar dq 0 ; AHCI Base Address Register (MMIO Base), set by ahci_init
ahci_ports_implemented dd 0 ; Bitmap of implemented ports
ahci_cmd_list_base dq 0 ; Physical address of Command List Headers (array of 32)
ahci_fis_rx_base dq 0 ; Physical address of FIS Receive Area (array of 32)
; Pointers to Command Tables (one per slot, or dynamically allocated?)
; For simplicity, allocate a pool or one per active command for now.
ahci_cmd_table_pool dq 0 ; Physical address of a pool of command tables
ahci_cmd_slot_busy_mask dq 0 ; Bitmap for tracking busy command slots

; IRQ synchronization flag (simple example)
ahci_irq_flags dq 0 ; Bit 0=completion, Bit 1=error

ahci_init_msg db "AHCI Init: BAR=0x", 0
ahci_init_ok_msg db " Ports Found: ", 0
ahci_ports_enabled_msg db " Ports Enabled: ", 0
ahci_port_msg db "Port ", 0
ahci_init_fail_msg db " AHCI Init Failed!", 0Dh, 0Ah, 0
ahci_cmd_fail_msg db "AHCI Command Failed! Port=", 0
ahci_err_msg db " Error=0x", 0
ahci_status_msg db " Status=0x", 0
ahci_done_msg db "AHCI Op Done", 0Dh, 0Ah, 0 

MAX_AHCI_PORTS equ 32

section .bss align=4096
; Allocate Command List Headers (32 slots * 32 bytes/header = 1KB) - Needs to be 1KB aligned
ahci_cmd_list_buffer resb AHCI_CMD_LIST_SIZE
; Allocate FIS Receive Area (32 ports * 256 bytes/FIS = 8KB) - Needs to be 256-byte aligned
ahci_fis_rx_buffer resb AHCI_FIS_RX_SIZE * MAX_AHCI_PORTS
; Allocate Command Tables (32 slots * 8KB/table = 256KB example) - Needs 128-byte alignment min
ahci_cmd_table_pool_buffer resb AHCI_CMD_TABLE_SIZE * AHCI_CMD_SLOTS

section .text

;--------------------------------------------------------------------------
; find_free_cmd_slot: Finds an available command slot (0-31)
; Input: RDI = Port number
; Output: RAX = Slot number (0-31), or -1 if none available
; Destroys: RAX, RBX, RCX
;--------------------------------------------------------------------------
find_free_cmd_slot:
    ; Read Port Command Issue (CI) and SATA Active (SACT) registers
    mov rbx, [ahci_abar]
    imul rcx, rdi, AHCI_PORT_REGS_SIZE
    add rbx, 0x100
    add rbx, rcx ; RBX = Port Base Address
    mov ecx, [rbx + AHCI_PxCI]  ; ECX = Command Issue bitmap
    mov edx, [rbx + AHCI_PxSACT] ; EDX = SATA Active bitmap (optional check?)
    or ecx, edx ; Combine active/issued slots
    not ecx ; Invert, free slots are now 1
    bsf rax, rcx ; Find first free slot index
    jz .no_slot_found ; If BSF result is zero, no bits were set (no free slots)
    ; RAX now holds the free slot index (0-31)
    ret
.no_slot_found:
    mov rax, -1
    ret

;--------------------------------------------------------------------------
; wait_for_cmd_completion: Waits for IRQ handler to signal completion/error
; Input: RDI = Port number, RCX = Slot number
; Output: Carry flag set on error
; Destroys: RAX, RBX, RCX (timeout counter), EDX
;--------------------------------------------------------------------------
wait_for_cmd_completion:
    mov rbx, AHCI_CMD_TIMEOUT_LOOPS ; Timeout counter
.wait_loop:
    ; Check our custom IRQ flags (should be cleared before issuing command)
    mov rax, [ahci_irq_flags]
    test rax, (AHCI_IRQ_FLAG_COMPLETE | AHCI_IRQ_FLAG_ERROR) ; Check completion or error bit
    jnz .irq_received

    ; Optional: Check PxCI bit directly (polling backup/alternative)
    ; mov rdx, [ahci_abar]
    ; ... calculate port base ...
    ; test dword [PortBase + AHCI_PxCI], (1 << cl) ; Test if bit for our slot is still set
    ; jnz .still_busy

    ; Check timeout
    dec rbx
    jnz .wait_loop
    ; Timeout occurred
    stc
    ret

;.still_busy:
;    pause ; Hint to CPU we are spin-waiting
;    jmp .wait_loop

.irq_received:
    ; Check error flag first
    test rax, AHCI_IRQ_FLAG_ERROR
    jnz .error_occurred
    ; Completion flag set, success
    clc
    ret
.error_occurred:
    stc
    ret


;--------------------------------------------------------------------------
; ahci_init: Initialize AHCI Controller
; Input: RAX = AHCI Base Address (ABAR from PCI)
; Output: RAX = 0 on success, error code otherwise. Carry set on error.
;--------------------------------------------------------------------------
ahci_init:
    push rbx; push rcx; push rdx; push rsi; push rdi; push r12; push r13; push r14; push r15

    test rax, rax
    jz .fail_no_bar       ; Fail if BAR is 0
    mov [ahci_abar], rax  ; Store base address

    ; Print BAR
    mov rsi, ahci_init_msg; call scr64_print_string
    call scr64_print_hex

    ; --- Take ownership from BIOS (Optional but recommended) ---
    ; TODO: Implement BOHC logic if needed

    ; --- Enable AHCI Mode ---
    mov rbx, [ahci_abar]
    mov ecx, [rbx + AHCI_GHC]
    or ecx, AHCI_GHC_AE     ; Set AHCI Enable bit
    mov [rbx + AHCI_GHC], ecx
    ; Optional: Wait for AE to be set?

    ; --- Allocate and Setup Command List / FIS Receive Areas ---
    ; Use physical addresses of buffers allocated in .bss
    ; Ensure they are physically contiguous if PMM doesn't guarantee it.
    ; For simplicity, assume direct physical mapping for now.
    ; TODO: Allocate these using PMM instead of fixed .bss buffers for flexibility.
    lea rdi, [rel ahci_cmd_list_buffer] ; Get virtual address of buffer
    ; Need physical address for AHCI registers! Assume identity mapped for now.
    mov [ahci_cmd_list_base], rdi ; Store physical address
    lea rdi, [rel ahci_fis_rx_buffer]
    mov [ahci_fis_rx_base], rdi
    lea rdi, [rel ahci_cmd_table_pool_buffer]
    mov [ahci_cmd_table_pool], rdi

    ; --- Initialize Each Implemented Port ---
    mov ecx, [rbx + AHCI_PI] ; ECX = Ports Implemented bitmap
    mov [ahci_ports_implemented], ecx
    mov r12d, ecx ; Save PI bitmap
    mov r13, 0 ; Port number
    mov r14, 0 ; Enabled port count

.port_init_loop:
    cmp r13, MAX_AHCI_PORTS
    jge .ports_init_done
    bt r12d, r13d ; Test if port R13 is implemented
    jnc .next_port

    ; Port R13 is implemented, calculate its base address
    mov r15, [ahci_abar] ; ABAR
    imul rdi, r13, AHCI_PORT_REGS_SIZE ; RDI = port_offset
    add r15, 0x100
    add r15, rdi ; R15 = Port Base Address

    ; --- Stop Port Command Processing ---
    mov edx, [r15 + AHCI_PxCMD]
    test edx, AHCI_PxCMD_ST ; Check if Start (ST) is set
    jz .port_st_clear
    ; Need to clear ST and wait for CR to clear
    and edx, ~AHCI_PxCMD_ST
    mov [r15 + AHCI_PxCMD], edx
.wait_st_clear:           ; Wait up to 500ms for PxCMD.CR to clear
    ; TODO: Add proper timeout logic using HPET/Timer
    mov edx, [r15 + AHCI_PxCMD]
    test edx, AHCI_PxCMD_CR ; Is Command List Running?
    jnz .wait_st_clear      ; Yes, wait
.port_st_clear:
    test edx, AHCI_PxCMD_FRE ; Check if FIS Receive Enable (FRE) is set
    jz .port_fre_clear
    ; Need to clear FRE and wait for FR to clear
    and edx, ~AHCI_PxCMD_FRE
    mov [r15 + AHCI_PxCMD], edx
.wait_fre_clear:          ; Wait up to 500ms for PxCMD.FR to clear
    ; TODO: Add proper timeout logic
    mov edx, [r15 + AHCI_PxCMD]
    test edx, AHCI_PxCMD_FR ; Is FIS Receive Running?
    jnz .wait_fre_clear     ; Yes, wait
.port_fre_clear:

    ; --- Configure Command List Base Address (CLB/CLBU) ---
    mov rsi, [ahci_cmd_list_base] ; Get physical address of command list headers
    imul rdx, r13, AHCI_CMD_LIST_SIZE ; Offset for this port's headers
    add rsi, rdx ; RSI = Physical address of Command List for Port R13
    mov [r15 + AHCI_PxCLB], esi ; Low 32 bits
    shr rsi, 32
    mov [r15 + AHCI_PxCLBU], esi ; High 32 bits

    ; --- Configure FIS Receive Base Address (FB/FBU) ---
    mov rsi, [ahci_fis_rx_base] ; Get physical address of FIS receive area
    imul rdx, r13, AHCI_FIS_RX_SIZE ; Offset for this port's FIS area
    add rsi, rdx ; RSI = Physical address of FIS area for Port R13
    mov [r15 + AHCI_PxFB], esi ; Low 32 bits
    shr rsi, 32
    mov [r15 + AHCI_PxFBU], esi ; High 32 bits

    ; --- Clear Port Interrupt Status & SATA Error Register ---
    mov edx, [r15 + AHCI_PxIS] ; Read IS to clear bits? Spec says write 1 to clear.
    mov [r15 + AHCI_PxIS], edx ; Write back to clear any pending interrupts
    mov edx, [r15 + AHCI_PxSERR]
    mov [r15 + AHCI_PxSERR], edx ; Write back to clear SATA errors

    ; --- Enable FIS Receive (FRE) and Start Command Processing (ST) ---
    mov edx, [r15 + AHCI_PxCMD]
    or edx, AHCI_PxCMD_FRE | AHCI_PxCMD_ST
    mov [r15 + AHCI_PxCMD], edx

    ; --- Enable Desired Interrupts for this Port ---
    ; Enable Task File Error, Descriptor Processed (optional), Device->Host FIS
    mov edx, AHCI_PxIE_TFEE | AHCI_PxIE_DPE | AHCI_PxIE_DHRE
    mov [r15 + AHCI_PxIE], edx

    ; --- Optional: Spin-up device if needed (PxCMD.SUD) ---
    ; --- Optional: Wait for device detection (PxTFD.BSY=0, PxTFD.DRQ=0) ---

    inc r14 ; Increment enabled port count

.next_port:
    inc r13
    jmp .port_init_loop
.ports_init_done:

    ; --- Enable Global Interrupts in AHCI controller ---
    mov rbx, [ahci_abar]
    mov ecx, [rbx + AHCI_GHC]
    or ecx, AHCI_GHC_IE     ; Set Interrupt Enable bit
    mov [rbx + AHCI_GHC], ecx

    ; Print success summary
    mov rsi, ahci_init_ok_msg; call scr64_print_string
    mov rax, r14; call scr64_print_dec
    mov rsi, ahci_done_msg; call scr64_print_string

    xor rax, rax ; Success
    clc
    jmp .init_done

.fail_no_bar:
    mov rax, 1 ; Example error code
    stc
    mov rsi, ahci_init_fail_msg; 
    call scr64_print_string

.init_done:
    pop r15;
    pop r14;
    pop r13;
    pop r12;
    pop rdi;
    pop rsi;
    pop rdx;
    pop rcx;
    pop rbx
    ret

;--------------------------------------------------------------------------
; ahci_read_write_sectors: Read/Write Sectors using AHCI DMA
; Input:
;   RDI = Port Number (0-31)
;   RSI = Starting LBA (64-bit)
;   RDX = Sector Count (Word preferred by FIS, max 65536)
;   RCX = Buffer Physical Address (must be physically contiguous for simplicity here)
;   R8B = Direction (0 = Read, 1 = Write)
; Output: RAX = 0 on success, error code otherwise. Carry set on error.
;--------------------------------------------------------------------------
ahci_read_write_sectors:
    push rbx;
    push rcx;
    push rdx;
    push rsi;
    push rdi;
    push r8;
    push r9;
    push r10;
    push r11;
    push r12;
    push r13;
    push r14;
    push r15

    ; Save arguments
    mov r12, rdi ; Port
    mov r13, rsi ; LBA
    mov r14, rdx ; Sector Count
    mov r15, rcx ; Buffer Address
    mov r11b, r8b ; Direction

    ; --- Validate Port Number ---
    bt dword [ahci_ports_implemented], r12d
    jnc .fail_port_invalid

    ; --- Find a Free Command Slot ---
    mov rdi, r12 ; Pass port number
    call find_free_cmd_slot
    cmp rax, -1
    je .fail_no_slot
    mov r10, rax ; R10 = Free Slot Number

    ; --- Get Port Base Address ---
    mov rbx, [ahci_abar]
    imul rcx, r12, AHCI_PORT_REGS_SIZE
    add rbx, 0x100
    add rbx, rcx ; RBX = Port Base Address

    ; --- Get Command Header Address for this slot ---
    mov rdi, [ahci_cmd_list_base]
    imul rax, r10, AHCI_COMMAND_HEADER_SIZE
    add rdi, rax ; RDI = Command Header address

    ; --- Get Command Table Address for this slot ---
    ; Simple pool allocation: use slot number * table size offset
    mov rsi, [ahci_cmd_table_pool]
    imul rax, r10, AHCI_CMD_TABLE_SIZE
    add rsi, rax ; RSI = Command Table address
    ; Clear the command table? Zeroing is safer.
    push rsi;
    push rcx;
    mov rcx, AHCI_CMD_TABLE_SIZE / 8;
    xor eax, eax;
    rep stosq;
    pop rcx;
    pop rsi

    ; --- Build the Command FIS (in Command Table at offset 0) ---
    mov byte [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_TYPE], AHCI_FIS_TYPE_REG_H2D
    mov byte [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_PM_PORT], 0x80 ; Set C bit
    ; Set command based on direction
    test r11b, r11b ; Test write flag
    jnz .set_cmd_write
    mov byte [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_COMMAND], ATA_CMD_READ_DMA_EXT
    jmp .set_cmd_done
.set_cmd_write:
    mov byte [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_COMMAND], ATA_CMD_WRITE_DMA_EXT
.set_cmd_done:
    ; LBA and Sector Count (LBA48)
    mov rax, r13 ; LBA
    mov [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_LBA0], al
    shr rax, 8;
    mov [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_LBA1], al
    shr rax, 8;
    mov [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_LBA2], al
    shr rax, 8;
    mov [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_LBA3], al
    shr rax, 8;
    mov [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_LBA4], al
    shr rax, 8;
    mov [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_LBA5], al
    mov byte [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_DEVICE], 1 << 6 ; LBA mode
    mov ax, r14w ; Sector Count
    mov [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_COUNT_L], al
    shr ax, 8;
    mov [rsi + AHCI_CT_CFIS_OFFSET + AHCI_FIS_COUNT_H], al
    ; Features L/H, Control, ICC are 0

    ; --- Build the PRDT (in Command Table at offset 0x80) ---
    ; Assuming single PRDT entry for simplicity (buffer <= 4MB)
    ; TODO: Handle splitting large buffers across multiple PRDT entries
    lea r8, [rsi + AHCI_CT_PRDT_OFFSET] ; R8 = PRDT Entry 0 Address
    mov rax, r15 ; Buffer Physical Address
    mov [r8 + AHCI_PRDT_DBA], eax ; Low 32 bits
    shr rax, 32
    mov [r8 + AHCI_PRDT_DBAU], eax ; High 32 bits
    ; Calculate Data Byte Count (SectorCount * 512) - 1
    mov rax, r14 ; Sector Count
    shl rax, 9   ; Bytes = Count * 512
    dec rax      ; DBC = Bytes - 1
    or eax, AHCI_PRDT_DBC_I ; Set interrupt on completion bit
    mov [r8 + AHCI_PRDT_DBC], eax

    ; --- Fill the Command Header (pointed by RDI) ---
    ; DW0: Flags (CFL, W), PRDTL
    mov eax, AHCI_FIS_REG_H2D_SIZE / 4 ; CFL = 20 / 4 = 5
    test r11b, r11b ; Write flag
    jz .set_hdr_read
    or eax, AHCI_CH_FLAGS_WRITE ; Set Write bit if writing
.set_hdr_read:
    mov edx, 1 ; PRDTL = 1 entry
    shl edx, AHCI_CH_FLAGS_PRDTL_SHIFT
    or eax, edx ; Combine flags and PRDTL
    mov [rdi + AHCI_CH_FLAGS], eax ; Write DW0 (Flags + PRDTL)
    ; DW1: PRDBC (Bytes transferred) - Cleared initially, filled by HBA
    mov dword [rdi + AHCI_CH_PRDBC], 0
    ; DW2-3: Command Table Base Address
    mov rax, rsi ; Command Table Physical Address
    mov [rdi + AHCI_CH_CTBA], eax
    shr rax, 32
    mov [rdi + AHCI_CH_CTBAU], eax

    ; --- Issue the command ---
    ; Clear status flags before issuing
    mov qword [ahci_irq_flags], 0
    ; Set the bit corresponding to the chosen slot (R10) in PxCI
    mov eax, 1      ; Start with 1 (bit 0)
    mov cl, r10b    ; ** Move slot number (0-31) into CL **
                    ;    (Ensure R10 only holds 0-31, use r10b for safety)
    shl eax, cl     ; ** EAX = 1 << slot_number (Calculate the mask correctly) **
    mov rdi, [rbx + AHCI_PxCI] ; Get Port CI Register Address (RBX=Port Base)
    mov [rdi], eax  ; ** Write the calculated mask to Command Issue **

    ; --- Wait for completion (Interrupt based) ---
    mov rdi, r12 ; Port number
    mov rcx, r10 ; Slot number
    call wait_for_cmd_completion
    jnc .success_cmd ; Jump if Carry is clear (success)

    ; --- Command Failed (Timeout or Error Flag) ---
.fail_cmd:
    ; Read port status for debugging
    mov eax, [rbx + AHCI_PxTFD] ; Get Task File Data (Status/Error)
    ; TODO: Clear command? Reset port?
    mov rsi, ahci_cmd_fail_msg;
    call scr64_print_string
    mov rax, r12;
    call scr64_print_dec ; Print Port
    mov rsi, ahci_status_msg; call scr64_print_string
    mov eax, [rbx + AHCI_PxTFD];
    call scr64_print_hex ; Print TFD
    mov rsi, ahci_err_msg;
    call scr64_print_string
    mov eax, [rbx + AHCI_PxSERR];
    call scr64_print_hex ; Print SError
    mov rsi, ahci_done_msg;
    call scr64_print_string
    mov rax, FAT_ERR_DISK_ERROR ; Return disk error
    stc
    jmp .done_rw

.success_cmd:
    xor rax, rax ; Success
    clc
    jmp .done_rw

.fail_port_invalid:
    mov rax, FAT_ERR_INVALID_PARAM
    stc
    jmp .done_rw_pop
.fail_no_slot:
    mov rax, FAT_ERR_DISK_ERROR ; Or a specific "busy" error
    stc
    jmp .done_rw_pop

.done_rw:
.done_rw_pop:
    pop r15; pop r14; pop r13; pop r12; pop r11; pop r10; pop r9; pop r8; pop rdi; pop rsi; pop rdx; pop rcx; pop rbx
    ret

ahci_read_sectors: ; Wrapper for read
    mov r8b, 0 ; Read direction
    jmp ahci_read_write_sectors

ahci_write_sectors: ; Wrapper for write
    mov r8b, 1 ; Write direction
    jmp ahci_read_write_sectors


;--------------------------------------------------------------------------
; ahci_flush_cache: Flush Disk Write Cache
; Input: RDI = Port Number
; Output: RAX = 0 on success, error code otherwise. Carry set on error.
;--------------------------------------------------------------------------
ahci_flush_cache:
    ; Similar structure to read/write:
    ; 1. Find free command slot
    ; 2. Get Port Base, Cmd Header Addr, Cmd Table Addr
    ; 3. Build Command FIS (ATA_CMD_CACHE_FLUSH_EXT) - No LBA/Count needed
    ; 4. Build Command Header (CFL=5, PRDTL=0, No Write/ATAPI/Prefetch)
    ; 5. Issue command (PxCI)
    ; 6. Wait for completion (Interrupt or Polling)
    ; 7. Return status
    mov rax, FAT_ERR_NOT_IMPLEMENTED ; Placeholder
    stc
    ret


;--------------------------------------------------------------------------
; ahci_irq_handler: Interrupt Handler for AHCI
; NOTE: Needs to be registered for the correct vector (MSI or legacy IRQ)
;       Assumes it's called with interrupt context (registers saved)
;--------------------------------------------------------------------------
ahci_irq_handler:
    push rax; push rbx; push rcx; push rdx; push rsi; push rdi; push r8; push r9; push r10; push r11; push r12; push r13; push r14; push r15

    mov r15, [ahci_abar] ; Get AHCI Base
    test r15, r15
    jz .irq_exit_no_abar ; Skip if AHCI not initialized

    ; Check Global Interrupt Status first
    mov r14d, [r15 + AHCI_IS] ; Read IS
    test r14d, r14d
    jz .irq_exit_no_status ; No interrupts pending?

    ; Iterate through implemented ports to see which one interrupted
    mov r13d, [ahci_ports_implemented]
    mov r12, 0 ; Port number
.irq_port_loop:
    cmp r12, MAX_AHCI_PORTS
    jge .irq_clear_global ; Checked all ports

    bt r13d, r12d ; Is this port implemented?
    jnc .irq_next_port
    bt r14d, r12d ; Does this port show interrupt pending in global IS?
    jnc .irq_next_port

    ; --- Interrupt pending for Port R12 ---
    ; Calculate Port Base
    mov rbx, r15 ; ABAR
    imul rcx, r12, AHCI_PORT_REGS_SIZE
    add rbx, 0x100
    add rbx, rcx ; RBX = Port Base Address

    ; Read Port Interrupt Status
    mov eax, [rbx + AHCI_PxIS]
    test eax, eax
    jz .irq_next_port ; No status bits set for this port?

    ; Clear handled interrupt bits by writing them back
    mov [rbx + AHCI_PxIS], eax

    ; Check for errors first
    test eax, AHCI_PxIS_TFES ; Task File Error?
    jnz .irq_error
    ; TODO: Check other error bits (IFS, HBDS, HBFS, etc.) if enabled/needed

    ; Check for command completion bits
    ; DHRS, PSS, DSS, SDBS, UFS, DPS all indicate some form of completion/activity
    ; For simplicity, set completion flag if no error
    or qword [ahci_irq_flags], AHCI_IRQ_FLAG_COMPLETE
    jmp .irq_next_port ; Check other ports

.irq_error:
    or qword [ahci_irq_flags], AHCI_IRQ_FLAG_ERROR | AHCI_IRQ_FLAG_COMPLETE
    ; Fall through to check next port

.irq_next_port:
    inc r12
    jmp .irq_port_loop

.irq_clear_global:
    ; Clear handled bits in Global Interrupt Status register
    mov [r15 + AHCI_IS], r14d

.irq_exit_no_abar:
.irq_exit_no_status:
    ; Send EOI (End Of Interrupt)
    ; TODO: Determine if PIC or APIC EOI is needed based on IRQ vector/system config
    mov al, PIC_EOI ; Assume PIC for now
    out PIC1_COMMAND, al
    ; If from PIC2 or using APIC, EOI is different

    pop r15; pop r14; pop r13; pop r12; pop r11; pop r10; pop r9; pop r8; pop rdi; pop rsi; pop rdx; pop rcx; pop rbx; pop rax
    iretq
