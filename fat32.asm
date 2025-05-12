; fat32.asm: FAT32 Filesystem Driver (64-bit, uses AHCI Driver)
; Depends on: boot_defs_temp.inc (which includes ahci_defs.inc and uefi_defs.inc)
;             ahci.asm (for ahci_read_sectors etc.)

BITS 64
global fat32_init_partition, fat32_read_file, fat32_write_file, fat32_create_file
global fat32_find_entry, fat32_get_next_cluster, fat32_init

; --- Externals ---
extern ahci_read_sectors, ahci_write_sectors, ahci_flush_cache
extern memcpy64, memset64
extern scr64_print_string, scr64_print_hex, scr64_print_dec
extern panic64

%include "boot_defs_temp.inc" ; **This now brings in all FAT32 constants**

section .data align=64
fat32_initialized       db 0
fat32_partition_lba     dq 0
fat32_ahci_port         dd 0
fat32_bytes_per_sector  dw 512 ; Default, verified from BPB
fat32_sectors_per_cluster db 0
fat32_reserved_sectors  dw 0
fat32_num_fats          db 0
fat32_sectors_per_fat   dd 0
fat32_root_cluster      dd 0
fat32_first_data_sector dd 0
fat32_fat_lba           dd 0
fat32_bytes_per_cluster dd 0
fat32_fsinfo_sector     dw 0

fat32_fat_cache_lba     dq -1
fat32_fat_cache_dirty   db 0

current_dir_cluster     dd 0

; Messages
msg_fat32_bpb_ok   db "FAT32: BPB Parsed OK", 0Dh, 0Ah, 0
msg_fat32_not_fat32 db "FAT32: Filesystem not FAT32!", 0Dh, 0Ah, 0
msg_fat32_bad_bpb  db "FAT32: BPB Invalid!", 0Dh, 0Ah, 0
msg_fat32_read_err db "FAT32: Disk Read Error!", 0Dh, 0Ah, 0
msg_fat32_write_err db "FAT32: Disk Write Error!", 0Dh, 0Ah, 0
msg_fat32_fat_err  db "FAT32: FAT Access Error!", 0Dh, 0Ah, 0

fs_type_string_fat32 db "FAT32   " ; Must be 8 chars, space padded

section .bss align=4096
sector_buffer resb PAGE_SIZE_4K

section .text

;--------------------------------------------------------------------------
fat32_flush_fat_cache:
    cmp byte [fat32_fat_cache_dirty], 1; jne .flush_not_dirty
    mov rsi, [fat32_fat_cache_lba]; cmp rsi, -1; je .flush_not_dirty
    mov edi, [fat32_ahci_port]; mov edx, 1; lea rcx, [sector_buffer]; mov r8b, 1
    call ahci_write_sectors; jc .flush_error; test rax, rax; jnz .flush_error
    mov byte [fat32_fat_cache_dirty], 0
.flush_not_dirty: xor rax, rax; clc; ret
.flush_error: mov rax, FAT_ERR_DISK_ERROR; stc; ret

;--------------------------------------------------------------------------
fat32_read_fat_sector: ; Input RSI = Absolute LBA of FAT sector
    push rsi
    cmp rsi, [fat32_fat_cache_lba]; je .read_fat_ok_pop
    call fat32_flush_fat_cache; jc .read_fat_error_pop
.read_fat_do_read:
    pop rsi; push rsi ; Get requested LBA
    mov edi, [fat32_ahci_port]; mov edx, 1; lea rcx, [sector_buffer]; mov r8b, 0
    call ahci_read_sectors; jnc .read_fat_ahci_ok; jmp .read_fat_error_pop
.read_fat_ahci_ok: test rax, rax; jnz .read_fat_error_pop
    pop rsi; mov [fat32_fat_cache_lba], rsi; mov byte [fat32_fat_cache_dirty], 0
.read_fat_ok_pop: pop rsi; xor rax, rax; clc; ret
.read_fat_error_pop: pop rsi; mov qword [fat32_fat_cache_lba], -1
    cmp rax, 0; jnz .read_fat_error_ret; mov rax, FAT_ERR_FAT_READ
.read_fat_error_ret: stc; ret

;--------------------------------------------------------------------------
fat32_init_partition: ; Input RDI=PartLBA, EDX=AHCIPort
    push rbx; push rsi; push rdi; push rdx; push r12; push r13
    mov [fat32_partition_lba], rdi; mov [fat32_ahci_port], edx
    mov byte [fat32_initialized], 0; mov qword [fat32_fat_cache_lba], -1; mov byte [fat32_fat_cache_dirty], 0
    mov esi, [fat32_partition_lba]; mov edx, 1; lea rcx, [sector_buffer]; mov r8b, 0; mov edi, [fat32_ahci_port]
    call ahci_read_sectors; jnc .read_bpb_ok; jmp .init_disk_error_msg
.read_bpb_ok: test rax, rax; jnz .init_disk_error_msg
    lea rsi, [sector_buffer]; cmp word [rsi + 510], 0xAA55; jne .init_bad_bpb_msg
    mov ax, [rsi + FAT_BPB_BytesPerSector]; cmp ax, 512; jne .init_bad_bpb_msg; mov [fat32_bytes_per_sector], ax
    movzx bx, byte [rsi + FAT_BPB_SectorsPerCluster]; test bl, bl; jz .init_bad_bpb_msg; mov [fat32_sectors_per_cluster], bl
    mov ax, [rsi + FAT_BPB_ReservedSectors]; cmp ax, 0; je .init_bad_bpb_msg; mov [fat32_reserved_sectors], ax
    movzx bx, byte [rsi + FAT_BPB_NumberOfFATs]; cmp bl, 1; jb .init_bad_bpb_msg; mov [fat32_num_fats], bl
    mov eax, [rsi + FAT32_BPB_SectorsPerFAT32]; test eax, eax; jz .init_not_fat32_msg; mov [fat32_sectors_per_fat], eax
    mov eax, [rsi + FAT32_BPB_RootCluster]; cmp eax, FAT32_CLUSTER_MIN_VALID; jb .init_bad_bpb_msg; mov [fat32_root_cluster], eax; mov [current_dir_cluster], eax
    mov ax, [rsi + FAT32_BPB_FSInfo]; mov [fat32_fsinfo_sector], ax
    mov rdi, rsi; add rdi, FAT32_BPB_FSType; lea r12, [fs_type_string_fat32]; mov ecx, 8; repe cmpsb; jne .init_not_fat32_msg
    mov eax, [fat32_sectors_per_fat]; movzx ebx, byte [fat32_num_fats]; mul ebx; movzx ebx, word [fat32_reserved_sectors]; add eax, ebx; mov [fat32_first_data_sector], eax
    mov rbx, [fat32_partition_lba]; movzx eax, word [fat32_reserved_sectors]; add rbx, rax; mov [fat32_fat_lba], ebx
    movzx eax, byte [fat32_sectors_per_cluster]; movzx ebx, word [fat32_bytes_per_sector]; mul ebx; mov [fat32_bytes_per_cluster], eax
    mov byte [fat32_initialized], 1; mov rsi, msg_fat32_bpb_ok; call scr64_print_string; xor rax, rax; clc; jmp .init_done
.init_disk_error_msg: mov rsi, msg_fat32_read_err; call scr64_print_string; mov rax, FAT_ERR_DISK_ERROR; stc; jmp .init_done
.init_bad_bpb_msg: mov rsi, msg_fat32_bad_bpb; call scr64_print_string; mov rax, FAT_ERR_BAD_BPB; stc; jmp .init_done
.init_not_fat32_msg: mov rsi, msg_fat32_not_fat32; call scr64_print_string; mov rax, FAT_ERR_NOT_FAT32; stc
.init_done: pop r13; pop r12; pop rdx; pop rdi; pop rsi; pop rbx; ret

;--------------------------------------------------------------------------
cluster_to_lba: ; Input EDI = cluster
    cmp edi, FAT32_CLUSTER_MIN_VALID; jb .invalid_cluster_ctl; mov eax, edi; sub eax, 2; movzx ecx, byte [fat32_sectors_per_cluster]; mul ecx; add eax, [fat32_first_data_sector]; add rax, [fat32_partition_lba]; clc; ret
.invalid_cluster_ctl: xor rax, rax; stc; ret

;--------------------------------------------------------------------------
fat32_get_next_cluster: ; Input EDI = current cluster
    push rbx; push rdx; push rdi; push rsi
    mov esi, edi; mov eax, esi; shl eax, 2; mov ebx, [fat32_bytes_per_sector]; xor edx, edx; div ebx
    mov rdi, rax; add rdi, [fat32_fat_lba]; call fat32_read_fat_sector; jc .fat_op_error_exit
    mov eax, [sector_buffer + rdx]; and eax, 0x0FFFFFFF
    cmp eax, FAT32_CLUSTER_BAD; je .fat_bad_cluster_exit
    cmp eax, FAT32_CLUSTER_EOF_MIN; jae .fat_eof_exit
    clc; jmp .fat_get_done
.fat_bad_cluster_exit: mov rax, FAT_ERR_BAD_CLUSTER; stc; jmp .fat_get_done
.fat_eof_exit: mov rax, FAT32_CLUSTER_EOF_MIN; clc; jmp .fat_get_done
.fat_op_error_exit:
.fat_get_done: pop rsi; pop rdi; pop rdx; pop rbx; ret

;--------------------------------------------------------------------------
read_cluster_chain: ; Input RDI=StartCluster, RSI=BufferPtr, RDX=BytesToRead
    push rbx; push r10; push r11; push r12; push r13; push r14; push r15
    mov r10, rdi; mov r11, rsi; mov r12, rdx; mov r14, 0; mov r15, rdi; mov r13, [fat32_bytes_per_cluster]; test r12, r12; jz .rcc_done_ok
.rcc_loop: cmp r10d, FAT32_CLUSTER_MIN_VALID; jb .rcc_error_bad_cluster; mov r15, r10; mov edi, r10d; call cluster_to_lba; jc .rcc_error_bad_cluster; mov r9, rax
    mov rbx, r13; cmp rbx, r12; cmova rbx, r12
    mov rcx, rbx; add rcx, [fat32_bytes_per_sector]-1; mov rax, rcx; xor edx, rdx; div qword [fat32_bytes_per_sector]; mov rcx, rax
    mov rdi, [fat32_ahci_port]; mov rsi, r9; mov rdx, rcx; mov rcx, r11; mov r8b, 0; call ahci_read_sectors; jnc .rcc_read_ok; jmp .rcc_error_disk
.rcc_read_ok: test rax, rax; jnz .rcc_error_disk; add r11, rbx; sub r12, rbx; add r14, rbx; test r12, r12; jz .rcc_done_ok
    mov edi, r10d; call fat32_get_next_cluster; jc .rcc_error_fat_read; cmp rax, FAT32_CLUSTER_EOF_MIN; jae .rcc_done_eof; mov r10, rax; jmp .rcc_loop
.rcc_done_ok: mov rax, r14; mov rcx, r15; clc; jmp .rcc_done
.rcc_done_eof: mov rax, r14; mov rcx, r15; clc; jmp .rcc_done
.rcc_error_disk: mov rax, FAT_ERR_DISK_ERROR; jmp .rcc_error_common
.rcc_error_bad_cluster: mov rax, FAT_ERR_BAD_CLUSTER; jmp .rcc_error_common
.rcc_error_fat_read: mov rax, FAT_ERR_FAT_READ
.rcc_error_common: mov rcx, r15; stc
.rcc_done: pop r15; pop r14; pop r13; pop r12; pop r11; pop r10; pop rbx; ret

;--------------------------------------------------------------------------
fat32_read_file: ; Input RDI=Filename (8.3 Padded), RSI=BufferPtr, RDX=BytesToRead
    push rbx; push r10; push r11; push r12; push r13; push r14
    mov r12, rdi; mov r13, rsi; mov r14, rdx
    cmp byte [fat32_initialized], 1; jne .read_not_init
    mov rdi, r12; mov esi, [current_dir_cluster]; call fat32_find_entry; jc .read_find_error
    test rax, rax; jz .read_not_found; mov r10, rbx; mov r11, rcx; cmp r14, r11; cmovb r14, r11
    mov rdi, r10; mov rsi, r13; mov rdx, r14; call read_cluster_chain; jc .read_chain_error; clc; jmp .read_done
.read_not_init: mov rax, FAT_ERR_NOT_INIT; stc; jmp .read_done_final
.read_find_error: stc; jmp .read_done_final
.read_not_found: mov rax, FAT_ERR_NOT_FOUND; stc; jmp .read_done_final
.read_chain_error: stc
.read_done:
.read_done_final: pop r14; pop r13; pop r12; pop r11; pop r10; pop rbx; ret

;--------------------------------------------------------------------------
fat32_find_entry: ; Input RDI=Filename (8.3 Padded), ESI=Start Cluster
    push rdi; push rsi; push rdx; push r8; push r9; push r10; push r11; push r12; push r13; push r14; push r15
    mov r10, rdi; mov r11, rsi; mov r12, FAT_DIRENT_SIZE
.fde_cluster_loop: cmp r11d, FAT32_CLUSTER_MIN_VALID; jb .fde_error_bad_cluster; mov edi, r11d; call cluster_to_lba; jc .fde_error_bad_cluster; mov r13, rax
    movzx rdx, byte [fat32_sectors_per_cluster]
.fde_sector_loop: test rdx, rdx; jz .fde_next_cluster; mov edi, [fat32_ahci_port]; mov rsi, r13; mov r8d, 1; lea rcx, [sector_buffer]; mov r9b, 0; call ahci_read_sectors; jnc .fde_read_ok; jmp .fde_error_disk
.fde_read_ok: test rax, rax; jnz .fde_error_disk; lea rsi, [sector_buffer]; mov rbx, 512; add rbx, rsi
.fde_entry_loop: cmp rsi, rbx; jae .fde_next_sector; mov al, [rsi + FAT_DIRENT_Name]; test al, al; jz .fde_not_found_ok; cmp al, 0xE5; je .fde_next_entry_skip
    mov cl, [rsi + FAT_DIRENT_Attributes]; and cl, FAT_ATTR_LFN_MASK; cmp cl, FAT_ATTR_LONG_NAME; je .fde_next_entry_skip
    mov rdi, rsi; mov r8, r10; mov rcx, 11; repe cmpsb; je .fde_found
.fde_next_entry_skip: add rsi, r12; jmp .fde_entry_loop
.fde_next_sector: inc r13; dec rdx; jmp .fde_sector_loop
.fde_next_cluster: mov edi, r11d; call fat32_get_next_cluster; jc .fde_error_fat; cmp rax, FAT32_CLUSTER_EOF_MIN; jae .fde_not_found_ok; mov r11, rax; jmp .fde_cluster_loop
.fde_found: mov rax, rsi; movzx ebx, word [rsi + FAT_DIRENT_FstClusLO]; movzx r8d, word [rsi + FAT_DIRENT_FstClusHI]; shl r8d, 16; or ebx, r8d; mov ecx, [rsi + FAT_DIRENT_FileSize]; clc; jmp .fde_done
.fde_error_disk: mov rax, FAT_ERR_DISK_ERROR; stc; jmp .fde_error_common
.fde_error_fat: mov rax, FAT_ERR_FAT_READ; stc; jmp .fde_error_common
.fde_error_bad_cluster: mov rax, FAT_ERR_BAD_CLUSTER; stc; jmp .fde_error_common
.fde_not_found_ok: xor rax, rax; clc
.fde_error_common: xor rbx, rbx; xor rcx, rcx
.fde_done: pop r15; pop r14; pop r13; pop r12; pop r11; pop r10; pop r9; pop r8; pop rsi; pop rdi; ret

; --- Stubs for Write/Create functionality ---
fat32_write_file: mov rax, FAT_ERR_NOT_IMPLEMENTED; stc; ret
fat32_create_file: mov rax, FAT_ERR_NOT_IMPLEMENTED; stc; ret
update_fat_entry: mov rax, FAT_ERR_NOT_IMPLEMENTED; stc; ret
find_free_cluster: mov rax, FAT_ERR_NOT_IMPLEMENTED; stc; ret
allocate_cluster_chain: mov rax, FAT_ERR_NOT_IMPLEMENTED; stc; ret
update_dir_entry: mov rax, FAT_ERR_NOT_IMPLEMENTED; stc; ret
clear_cluster: mov rax, FAT_ERR_NOT_IMPLEMENTED; stc; ret