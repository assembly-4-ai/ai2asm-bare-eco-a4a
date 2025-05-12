; main_uefi_loader.asm - Merged UEFI Bootloader + GDT/IDT/PIC Setup

BITS 64

global _start
%include "boot_defs_temp.inc" ; Includes UEFI/AHCI/FAT/PIC Defs now

; --- Externals ---

;extern pmm64_init_uefi, pmm_alloc_frame, pmm_free_frame, pmm_mark_region_used
 ;Paging
;extern paging_init_64_uefi, reload_cr3, kernel_pml4
;GDT
;extern setup_final_gdt64, load_gdt_and_segments64, gdt64_pointer_var
;IDT
extern setup_final_idt64
extern load_idt64
extern dt64_pointer_var ; Use combined setup
; Screen Driver (Post-BS)
;extern scr64_init, scr64_print_string, scr64_print_hex, scr64_print_dec
; Keyboard Driver (Post-BS)
;extern keyboard_init, getchar_from_buffer
; PCI
;extern pci_init, pci_find_ahci_controller
; AHCI
;extern ahci_init, ahci_read_sectors, ahci_write_sectors
; FAT32
extern fat32_init
;PIC
extern pic_remap
; Payload / Panic
;extern shell_run, panic64
; 

;extern gop_pixel_format, gop_v_res, gop_framebuffer_size


; Globals defined elsewhere or by linker
global efi_image_base, efi_image_size
;global numa_node_count
global pmm_node_base_addrs, pmm_node_addr_limits
global key_buffer, key_buffer_head, key_buffer_tail ; Keyboard buffer

section .data align=64
    gImageHandle dq 0
    gSystemTable dq 0
    gMemoryMap dq 0             ; Pointer to allocated map buffer
    gMemoryMapSize dq 4096 * 4  ; Initial map buffer size (updated by GetMemoryMap)
    gMapKey dq 0
    gDescriptorSize dq 0
    gDescriptorVersion dq 0
    gFinalGdtBase dq 0
    gFinalIdtBase dq 0
    gAhciBaseAddr dq 0          ; Store found AHCI BAR here
    gFat32PartitionLba dq 0     ; Store found FAT32 LBA start here
    gAhciPortNum dd 0           ; Default AHCI port to use


    ;------------------------------------
    ; screen_gop defs
    ;------------------------------------
 
    section .data 
    

    ; Default NUMA info
    ;numa_node_count db 1
    ;pmm_node_base_addrs dq 0, 0, 0, 0, 0, 0, 0, 0
    ;pmm_node_addr_limits dq 0xFFFFFFFFFFFFFFFF, 0, 0, 0, 0, 0, 0, 0

    ; Messages (keep as before)
    msg_welcome db "UEFI Loader Initializing...", 0Dh, 0Ah, 0
    msg_loaded_image_ok db "Loaded Image Info OK", 0Dh, 0Ah, 0
    msg_loaded_image_fail db "Failed to get Loaded Image Protocol!", 0Dh, 0Ah, 0
    msg_gop_ok db "Graphics Output Protocol OK", 0Dh, 0Ah, 0
    msg_gop_fail db "Failed to get GOP!", 0Dh, 0Ah, 0
    msg_memmap_ok db "Memory Map OK", 0Dh, 0Ah, 0
    msg_memmap_fail db "Failed to get Memory Map!", 0Dh, 0Ah, 0
    msg_alloc_gdt_fail db "Failed to allocate GDT!", 0Dh, 0Ah, 0
    msg_alloc_idt_fail db "Failed to allocate IDT!", 0Dh, 0Ah, 0
    msg_pmm_ok db "PMM Initialized", 0Dh, 0Ah, 0
    msg_pmm_fail db "PMM Initialization Failed!", 0Dh, 0Ah, 0
    msg_paging_ok db "Paging Initialized", 0Dh, 0Ah, 0
    msg_paging_fail db "Paging Initialization Failed!", 0Dh, 0Ah, 0
    msg_gdt_ok db "GDT Setup OK", 0Dh, 0Ah, 0
    msg_exit_bs_fail db "ExitBootServices Failed!", 0Dh, 0Ah, 0
    msg_post_exit db "Exited Boot Services. Setting up HW...", 0Dh, 0Ah, 0
    msg_pic_ok db "PIC Remapped OK", 0Dh, 0Ah, 0
    msg_pci_ok db "PCI Init OK", 0Dh, 0Ah, 0
    msg_ahci_ok db "AHCI Init OK", 0Dh, 0Ah, 0
    msg_ahci_fail db "AHCI Init Failed!", 0Dh, 0Ah, 0
    msg_gpt_ok db "FAT32 Partition Found OK", 0Dh, 0Ah, 0
    msg_gpt_fail db "FAT32 Partition Not Found!", 0Dh, 0Ah, 0
    msg_fat32_ok db "FAT32 Init OK", 0Dh, 0Ah, 0
    msg_fat32_fail db "FAT32 Init Failed!", 0Dh, 0Ah, 0
    msg_kbd_ok db "Keyboard Init OK", 0Dh, 0Ah, 0
    msg_idt_ok db "IDT Setup OK. Enabling Interrupts.", 0Dh, 0Ah, 0
    msg_jumping db "Jumping to Shell...", 0Dh, 0Ah, 0
    msg_shell_return_err db "Shell Returned Unexpectedly!", 0Dh, 0Ah, 0

    
    section .bss align=4096
    initial_mmap_buffer resb 4096 * 4
    efi_image_base resq 1
    efi_image_size resq 1
    key_buffer resb 32
    key_buffer_head resb 4
    key_buffer_tail resb 4
    disk_read_buffer resb 4096

section .text
global efi_main


_start:
; --- Helper to unmask specific PIC IRQ line ---
pic_unmask_irq: ; Input: AL = IRQ line (0-15)
    push ax;
    push dx
    cmp al, 8
    jb .master_mask
.slave_mask:
    mov dx, PIC2_DATA ; Slave data port (0xA1)
    in al, dx         ; Read current mask
    mov ah, al        ; Save it
    pop dx            ; Get IRQ line back into dl
    sub dl, 8         ; Convert to slave line number (0-7)
    mov al, 1
    push cx
    mov cl, dl
    shl al, cl       ; Create bitmask
    pop cx
    not al            ; Invert mask (0 to unmask)
    and ah, al        ; Clear the specific bit in saved mask
    mov al, ah        ; Move new mask to AL
    mov dx, PIC2_DATA
    out dx, al        ; Write new mask to slave PIC
    jmp .mask_done
.master_mask:
    mov dx, PIC1_DATA ; Master data port (0x21)
    in al, dx         ; Read current mask
    mov ah, al        ; Save it
    pop dx            ; Get IRQ line back into dl
    mov al, 1
    push cx
    mov cl,dl
    shl al, cl        ; Create bitmask
    pop cx
    not al            ; Invert mask (0 to unmask)
    and ah, al        ; Clear the specific bit in saved mask
    mov al, ah        ; Move new mask to AL
    mov dx, PIC1_DATA
    out dx, al        ; Write new mask to master PIC
.mask_done:
    pop dx; pop ax
    ret


; --- Main UEFI Entry Point ---
efi_main:
    push rbp;
    mov rbp, rsp;
    sub rsp, 64;
    and rsp, -16
    push rbx;
    push rsi;
    push rdi;
    push r12;
    push r13;
    push r14;
    push r15
    mov [gImageHandle], rdi;
    mov [gSystemTable], rsi

    ; 1. Print Welcome
    mov rcx, [gSystemTable];
    mov rdx, msg_welcome;
    call UefiPrint

    ; 2. Get Loaded Image Info
    mov rcx, [gImageHandle];
    mov rdx, EFI_LOADED_IMAGE_PROTOCOL_GUID;
    mov r8, rsp;
    call UefiHandleProtocol
    test rax, rax;
    jnz .loaded_image_fail;
    mov rbx, [rsp];
    mov rax, [rbx + OFFSET_LOADED_IMAGE_IMAGEBASE];
    mov [efi_image_base], rax
    mov rax, [rbx + OFFSET_LOADED_IMAGE_IMAGESIZE];
    mov [efi_image_size], rax;
    mov rcx, [gSystemTable];
    mov rdx, msg_loaded_image_ok;
    call UefiPrint

    ; 3. Get GOP Info
    mov rcx, EFI_GRAPHICS_OUTPUT_PROTOCOL_GUID;
    mov rdx, 0;
    mov r8, rsp;
    call UefiLocateProtocol
    test rax, rax;
    jnz .gop_fail;
    mov rbx, [rsp];
    mov rcx, [rbx + OFFSET_GOP_MODE];
    mov rax, [rcx + OFFSET_GOP_MODE_FBBASE];
    mov [gop_framebuffer_base], rax
    mov rax, [rcx + OFFSET_GOP_MODE_FBSIZE];
    mov [gop_framebuffer_size], rax;
    mov rdx, [rcx + OFFSET_GOP_MODE_INFO];
    mov eax, [rdx + OFFSET_GOP_INFO_HRES]
    mov [gop_h_res], eax;
    mov eax, [rdx + OFFSET_GOP_INFO_VRES];
    mov [gop_v_res], eax;
    mov eax, [rdx + OFFSET_GOP_INFO_PIXELFMT];
    mov [gop_pixel_format], eax
    mov eax, [rdx + OFFSET_GOP_INFO_PIXELSPERSCANLINE] ; Use correct offset
    mov [gop_pixels_per_scanline], eax
    mov rcx, [gSystemTable];
    mov rdx, msg_gop_ok;
    call UefiPrint

    ; 4. Get Memory Map (First time)
    mov rdi, gMemoryMap;
    push rax
    xor rax,rax
    mov rax, initial_mmap_buffer
    mov [rdi], rax
    pop rax;
    mov rcx, rdi;
    mov rdx, gMemoryMapSize;
    mov r8, gMapKey;
    mov r9, gDescriptorSize;
    push gDescriptorVersion;
    call UefiGetMemoryMap
    pop rax;
    test rax, rax;
    jnz .memmap_fail;
    mov rcx, [gSystemTable];
    mov rdx, msg_memmap_ok;
    call UefiPrint

    ; 5. Allocate GDT/IDT memory
    mov rcx, AllocateAnyPages;
    mov rdx, EfiLoaderData;
    mov r8, 1;
    lea r9, [gFinalGdtBase];
    call UefiAllocatePages;
    test rax, rax;
    jnz .alloc_gdt_fail
    mov rcx, AllocateAnyPages;
    mov rdx, EfiLoaderData;
    mov r8, 1;
    lea r9, [gFinalIdtBase];
    call UefiAllocatePages;
    test rax, rax;
    jnz .alloc_idt_fail

    ; 6. Initialize PMM
    mov rcx, [gMemoryMap];
    mov rdx, [gMemoryMapSize];
    mov r8, [gDescriptorSize];
    call pmm64_init_uefi;
    test rax, rax;
    jnz .pmm_fail
    mov rax, [gFinalGdtBase];
    mov rdx, PAGE_SIZE_4K;
    add rdx, rax;
    call pmm_mark_region_used
    mov rax, [gFinalIdtBase];
    mov rdx, PAGE_SIZE_4K;
    add rdx, rax;
    call pmm_mark_region_used
    mov rax, [gop_framebuffer_base];
    mov rdx, [gop_framebuffer_size];
    add rdx, rax;
    call pmm_mark_region_used
    mov rcx, [gSystemTable];
    mov rdx, msg_pmm_ok;
    call UefiPrint

    ; 7. Load Kernel/Payload - Skipped

    ; 8. Setup Final Paging
    mov rcx, [gMemoryMap];
    mov rdx, [gMemoryMapSize];
    mov r8, [gDescriptorSize];
    mov r9, kernel_pml4;
    call paging_init_64_uefi;
    test rax, rax;
    jnz .paging_fail
    mov rcx, [gSystemTable];
    mov rdx, msg_paging_ok;
    call UefiPrint

    ; 9. Setup GDT Descriptors
    mov rdi, [gFinalGdtBase];
    call setup_final_gdt64;
    mov rcx, [gSystemTable];
    mov rdx, msg_gdt_ok;
    call UefiPrint

    ; 10. Get Memory Map AGAIN for ExitBootServices Key
    mov rcx, gMemoryMap;
    mov rdx, gMemoryMapSize;
    mov r8, gMapKey;
    mov r9, gDescriptorSize;
    push gDescriptorVersion;
    call UefiGetMemoryMap
    pop rax;
    test rax, rax;
    jnz .memmap_fail

    ; 11. Exit Boot Services
    mov rdi, [gImageHandle];
    mov rcx, rdi;
    mov rdx, [gMapKey];
    call UefiExitBootServices;
    test rax, rax;
    jnz .exit_bs_fail

    ; --- UEFI Boot Services are GONE ---

    ; 12. Load GDT/Segments, Init Screen
    call load_gdt_and_segments64
    mov rdi, [gop_framebuffer_base];
    mov esi, [gop_h_res];
    mov edx, [gop_v_res];
    mov ecx, [gop_pixels_per_scanline];
    mov r8d, [gop_pixel_format];
    call scr64_init
    mov rsi, msg_post_exit;
    call scr64_print_string

    ; 13. Remap PIC
    call pic_remap
    mov rsi, msg_pic_ok;
    call scr64_print_string

    ; 14. Setup IDT and Load
    mov rdi, [gFinalIdtBase];
    call setup_final_idt64
    call load_idt64
    mov rsi, msg_idt_ok; 
    call scr64_print_string

    ; 15. Initialize PCI & AHCI
    call pci_init ; msg_pci_ok
    call pci_find_ahci_controller;
    test rax, rax;
    jz .ahci_fail;
    mov [gAhciBaseAddr], rax
    call ahci_init ; Pass BAR in RAX from previous call
    test rax, rax;
    jnz .ahci_fail; ;msg_ahci_ok

    ; 16. Find FAT32 Partition (Placeholder)
    ; call FindFat32PartitionLba ; Needs implementation using AHCI
    ; test rax, rax; jnz .gpt_fail
    mov qword [gFat32PartitionLba], 2048 ;### HARCODED LBA FOR TESTING ###
    ; msg_gpt_ok

    ; 17. Initialize FAT32
    mov rdi, [gFat32PartitionLba];
    mov edx, [gAhciPortNum];
    call fat32_init
    test rax, rax;
    jnz .fat32_fail
    ; msg_fat32_ok

    ; 18. Initialize Keyboard & Unmask IRQ
    call keyboard_init
    mov al, KB_IRQ ; Unmask Keyboard IRQ (IRQ 1)
    call pic_unmask_irq
    ; msg_kbd_ok

    ; 19. Enable Interrupts
    sti

    ; 20. Jump to Shell
    mov rsi, msg_jumping;
    call scr64_print_string
    call shell_run

    ; Should not return
    mov rsi, msg_shell_return_err;
    call panic64

; --- Error Handling ---
.loaded_image_fail:
mov rsi, msg_loaded_image_fail; 
jmp .panic_or_print
.gop_fail: 
mov rsi, msg_gop_fail;
jmp .panic_or_print
.memmap_fail:
mov rsi, msg_memmap_fail;
jmp .panic_or_print
.alloc_gdt_fail:
mov rsi, msg_alloc_gdt_fail;
jmp .panic_or_print
.alloc_idt_fail:
mov rsi, msg_alloc_idt_fail;
jmp .panic_or_print
.pmm_fail:
mov rsi, msg_pmm_fail;
jmp .panic_or_print
.paging_fail:
mov rsi, msg_paging_fail;
jmp .panic_or_print
.exit_bs_fail:
jmp .halt_critical ;Cannot print ExitBS failure reliably
.ahci_fail:
mov rsi, msg_ahci_fail;
call scr64_print_string;
jmp .halt
.gpt_fail:
mov rsi, msg_gpt_fail;
call scr64_print_string;
jmp .halt
.fat32_fail:
mov rsi, msg_fat32_fail;
call scr64_print_string;
jmp .halt
.panic_or_print:
mov rcx, [gSystemTable];
mov rdx, rsi;
call UefiPrint;
jmp .halt
.halt_critical:; Halt when printing might not work
.halt:
cli;
.spin:
hlt;
jmp .spin
.clean_exit:
xor rax, rax;
pop r15;
pop r14;
pop r13;
pop r12;
pop rdi;
pop rsi;
pop rbx;
add rsp, 64;
pop rbp;
ret

;--------------------------------------------------------------------------
; UefiPrint: Prints a null-terminated Unicode string.
; Input: RCX = (Ignored, uses gSystemTable)
;        RDX = Pointer to Null-terminated Unicode String
; Output: RAX = Status
;--------------------------------------------------------------------------
UefiPrint:
    push rbp
    mov rbp, rsp
    sub rsp, 32+8 ; Shadow space for 4 registers + 8 for alignment/scratch
    and rsp, -16  ; Ensure 16-byte alignment before call

    push rbx
    push rsi
    push rdi
    push r10 ; Using r10 for function pointer

    mov rsi, [gSystemTable]
    test rsi, rsi
    jz .print_fail_no_systable

    mov rbx, [rsi + OFFSET_ST_CONOUT]
    test rbx, rbx
    jz .print_fail_no_conout

    mov r10, [rbx + OFFSET_CONOUT_OUTPUTSTRING]
    test r10, r10
    jz .print_fail_no_func

    ; Args for ConOut->OutputString:
    ; RCX = This (ConOut Protocol Pointer)
    ; RDX = String (Passed as input RDX)
    mov rcx, rbx
    ; RDX is already set
    call r10
    ; RAX holds status
    jmp .print_done

.print_fail_no_systable:
    mov rax, EFI_INVALID_PARAMETER ; SystemTable not set
    jmp .print_done
.print_fail_no_conout:
.print_fail_no_func:
    mov rax, EFI_UNSUPPORTED
.print_done:
    pop r10
    pop rdi
    pop rsi
    pop rbx
    mov rsp, rbp
    pop rbp
    ret

;--------------------------------------------------------------------------
; UefiAllocatePages: Allocates physical memory pages.
; Input: RCX = AllocateType
;        RDX = MemoryType
;        R8  = Pages (Number of pages)
;        R9  = Address of variable to store result (Memory Ptr)
; Output: RAX = Status
;--------------------------------------------------------------------------
UefiAllocatePages:
    push rbp
    mov rbp, rsp
    sub rsp, 32+8
    and rsp, -16
    push rbx
    push rsi
    push rdi
    push r10

    mov rsi, [gSystemTable]
    test rsi, rsi
    jz .alloc_fail_no_systable

    mov rbx, [rsi + OFFSET_ST_BOOTSERVICES]
    test rbx, rbx
    jz .alloc_fail_no_bs

    mov r10, [rbx + OFFSET_BS_ALLOCATEPAGES]
    test r10, r10
    jz .alloc_fail_no_func

    ; Args for BS->AllocatePages are already in RCX, RDX, R8, R9 from caller
    call r10
    ; RAX holds status
    jmp .alloc_done

.alloc_fail_no_systable:
.alloc_fail_no_bs:
.alloc_fail_no_func:
    mov rax, EFI_NOT_READY
.alloc_done:
    pop r10
    pop rdi
    pop rsi
    pop rbx
    mov rsp, rbp
    pop rbp
    ret

;--------------------------------------------------------------------------
; UefiGetMemoryMap: Gets the UEFI memory map.
; Input:
;   RCX = Pointer to variable to store Map Buffer address
;   RDX = Pointer to variable to store Map Size (input: initial size, output: actual size)
;   R8  = Pointer to variable to store Map Key
;   R9  = Pointer to variable to store Descriptor Size
;   [RSP+40] (stack) = Pointer to variable to store Descriptor Version
; Output: RAX = Status. Variables pointed to by args are updated.
; NOTE: Simplified. Robust version needs to handle EFI_BUFFER_TOO_SMALL.
;--------------------------------------------------------------------------
UefiGetMemoryMap:
    push rbp
    mov rbp, rsp
    sub rsp, 32+8       ; Shadow + 1 stack arg
    and rsp, -16
    push rbx
    push rsi
    push rdi
    push r10
    push r11
    push r12 ; For DescriptorVersion Ptr from stack

    mov rsi, [gSystemTable]
    test rsi, rsi
    jz .getmap_fail_no_systable

    mov rbx, [rsi + OFFSET_ST_BOOTSERVICES]
    test rbx, rbx
    jz .getmap_fail_no_bs

    mov r11, [rbx + OFFSET_BS_GETMEMORYMAP]
    test r11, r11
    jz .getmap_fail_no_func

    ; Args for BS->GetMemoryMap:
    ; Arg1 (RCX_call): MemoryMapSize (Pointer to a UINTN)
    ; Arg2 (RDX_call): MemoryMap (Pointer to a buffer)
    ; Arg3 (R8_call):  MapKey (Pointer to a UINTN)
    ; Arg4 (R9_call):  DescriptorSize (Pointer to a UINTN)
    ; Arg5 (Stack_call): DescriptorVersion (Pointer to a UINT32)

    ; Caller provided: RCX=&gMemMapVar, RDX=&gMemMapSizeVar, R8=&gMapKeyVar, R9=&gDescSizeVar
    ; Stack arg was pushed by caller.
    mov r10, rcx ; Save Ptr to gMemMapVar
    mov rcx, rdx ; RCX_call = Ptr to gMemMapSizeVar
    mov rdx, [r10] ; RDX_call = Actual Map Buffer address (from value of gMemMapVar)
    ; R8 is Ptr to gMapKeyVar
    ; R9 is Ptr to gDescSizeVar

    ; Get stack argument (DescriptorVersion Ptr)
    ; Original RBP + Return Addr + Pushed Regs (rbp,rbx,rsi,rdi,r10,r11,r12)
    mov r12, [rbp + 16 + (7*8)]
    push r12 ; Push Ptr to DescriptorVersion for the call

    call r11 ; Call BS->GetMemoryMap
    ; RAX = Return Status

    add rsp, 8 ; Clean up stack argument

    ; TODO: Implement EFI_BUFFER_TOO_SMALL handling by:
    ; 1. Reading the new required size from the variable pointed to by original RDX.
    ; 2. Freeing the old buffer (pointed to by original value of [RCX]).
    ; 3. Allocating a new buffer of the required size using UefiAllocatePages.
    ; 4. Updating the variable pointed to by original RCX with the new buffer address.
    ; 5. Updating the variable pointed to by original RDX with the new buffer size.
    ; 6. Retrying the GetMemoryMap call.
    ; For now, we just return the status.
    jmp .getmap_done

.getmap_fail_no_systable:
.getmap_fail_no_bs:
.getmap_fail_no_func:
    mov rax, EFI_NOT_READY
.getmap_done:
    pop r12
    pop r11
    pop r10
    pop rdi
    pop rsi
    pop rbx
    mov rsp, rbp
    pop rbp
    ret

;--------------------------------------------------------------------------
; UefiExitBootServices: Exits UEFI Boot Services.
; Input: RCX = ImageHandle, RDX = MapKey
; Output: RAX = Status
;--------------------------------------------------------------------------
UefiExitBootServices:
    push rbp
    mov rbp, rsp
    sub rsp, 32+8
    and rsp, -16
    push rbx
    push rsi
    push rdi
    push r10

    mov rsi, [gSystemTable]
    test rsi, rsi
    jz .exitbs_fail_no_systable

    mov rbx, [rsi + OFFSET_ST_BOOTSERVICES]
    test rbx, rbx
    jz .exitbs_fail_no_bs

    mov r10, [rbx + OFFSET_BS_EXITBOOTSERVICES]
    test r10, r10
    jz .exitbs_fail_no_func

    ; RCX = ImageHandle, RDX = MapKey (already set by caller)
    call r10
    ; RAX = Return Status
    jmp .exitbs_done

.exitbs_fail_no_systable:
.exitbs_fail_no_bs:
.exitbs_fail_no_func:
    mov rax, EFI_NOT_READY
.exitbs_done:
    pop r10
    pop rdi
    pop rsi
    pop rbx
    mov rsp, rbp
    pop rbp
    ret

;--------------------------------------------------------------------------
; UefiLocateProtocol: Finds an instance of a protocol.
; Input:
;   RCX = Protocol GUID Ptr
;   RDX = Registration (NULL if not used)
;   R8  = Pointer to variable to store Interface Ptr
; Output: RAX = Status
;------------------------------------------------------------------------
;UefiLocateProtocol:
    
UefiLocateProtocol:
    push rbp
    mov rbp, rsp
    sub rsp, 32+8
    and rsp, -16
    push rbx
    push rsi
    push rdi
    push r9 ; R9 is scratch, using it for BS pointer
    push r10

    mov rsi, [gSystemTable]
    test rsi, rsi
    jz .locate_fail_no_systable

    mov r9, [rsi + OFFSET_ST_BOOTSERVICES] ; R9 = BootServices Table Pointer
    test r9, r9
    jz .locate_fail_no_bs

    mov r10, [r9 + OFFSET_BS_LOCATEPROTOCOL] ; R10 = BS->LocateProtocol function pointer
    test r10, r10
    jz .locate_fail_no_func

    ; Args for BS->LocateProtocol are already in RCX, RDX, R8 from caller
    call r10
    ; RAX = Return Status
    jmp .locate_done

.locate_fail_no_systable:
.locate_fail_no_bs:
.locate_fail_no_func:
    mov rax, EFI_NOT_READY
.locate_done:
    pop r10
    pop r9
    pop rdi
    pop rsi
    pop rbx
    mov rsp, rbp
    pop rbp
    ret

;--------------------------------------------------------------------------
; UefiHandleProtocol: Gets protocol interface from a specific handle.
; Input:
;   RCX = Handle
;   RDX = Protocol GUID Ptr
;   R8  = Pointer to variable to store Interface Ptr
; Output: RAX = Status
;--------------------------------------------------------------------------
UefiHandleProtocol:
    push rbp
    mov rbp, rsp
    sub rsp, 32+8
    and rsp, -16
    push rbx
    push rsi
    push rdi
    push r9
    push r10

    mov rsi, [gSystemTable]
    test rsi, rsi
    jz .handle_fail_no_systable

    mov r9, [rsi + OFFSET_ST_BOOTSERVICES] ; R9 = BootServices Table Pointer
    test r9, r9
    jz .handle_fail_no_bs

    mov r10, [r9 + OFFSET_BS_HANDLEPROTOCOL] ; R10 = BS->HandleProtocol function pointer
    test r10, r10
    jz .handle_fail_no_func

    ; Args for BS->HandleProtocol are already in RCX, RDX, R8 from caller
    call r10
    ; RAX = Return Status
    jmp .handle_done

.handle_fail_no_systable:
.handle_fail_no_bs:
.handle_fail_no_func:
    mov rax, EFI_NOT_READY
.handle_done:
    pop r10
    pop r9
    pop rdi
    pop rsi
    pop rbx
    mov rsp, rbp
    pop rbp
    ret

; Add UefiOpenProtocol / UefiCloseProtocol / UefiReadFile / UefiCloseFile stubs/impl as needed
; For example:
; UefiOpenFile:
;   mov rax, EFI_UNSUPPORTED
;   ret

; --- Include other modules ---
%include "gdt_uefi.asm"
%include "idt64_uefi.asm"
%include "pmm64_uefi.asm"
%include "paging64_uefi.asm"
%include "screen_gop.asm"
%include "keyboard.asm"
%include "pci.asm"
%include "ahci.asm"
%include "fat32.asm"
%include "shell.asm"
%include "apic.asm" ; Include PIC code
;%include "uefi_wrappers.asm" ; Assumed