; pmm64_uefi.asm: UEFI-centric 64-bit Physical Memory Manager (Optimized)
; Depends on: boot_defs_temp.inc (with UEFI defs)
; Depends on: uefi_wrappers.asm (for uefi_AllocatePagesWrapper)

BITS 64
%include "boot_defs_temp.inc" ; Include UEFI defs!

; Globals defined/used by PMM
global pmm64_init_uefi, pmm_alloc_frame, pmm_free_frame
global pmm_alloc_frame_node, pmm_free_frame_node
global pmm_alloc_large_frame, pmm_free_large_frame
global pmm_mark_region_used, pmm_mark_region_free
global pmm_total_frames, pmm_used_frames, pmm_max_ram_addr
global pmm_total_large_frames, pmm_used_large_frames
global numa_node_count ; Ensure this is defined/set elsewhere

; External UEFI / System Info needed

extern uefi_AllocatePagesWrapper ; Assumed: RCX=Type, RDX=MemType, R8=Pages, R9=AddrPtr -> RAX=Status, [R9]=PhysAddr
extern efi_image_base       ; Physical base address of loaded EFI app
extern efi_image_size       ; Size of loaded EFI app in bytes
; extern AcpiParseSrat      ; Optional: extern void AcpiParseSrat(void); called before pmm init

%ifndef SMP
    %define SMP 0
%endif
%if SMP
    %define LOCK_PREFIX lock
%else
    %define LOCK_PREFIX
%endif

section .data align=64
; --- PMM State Variables ---
numa_node_count: db 1 ; Default to 1 node
pmm_total_frames dq 0
pmm_used_frames dq 0
pmm_total_large_frames dq 0
pmm_used_large_frames dq 0
pmm_max_ram_addr dq 0
; NUMA node data structures (Max 8 nodes assumed by array size)
pmm_node_base_addrs dq 0, 0, 0, 0, 0, 0, 0, 0
pmm_node_addr_limits dq 0, 0, 0, 0, 0, 0, 0, 0 ; Store MAX address limit (exclusive)
pmm_node_bitmaps dq 0, 0, 0, 0, 0, 0, 0, 0 ; Pointers allocated by UEFI
pmm_node_large_bitmaps dq 0, 0, 0, 0, 0, 0, 0, 0 ; Pointers allocated by UEFI
pmm_node_frame_counts dq 0, 0, 0, 0, 0, 0, 0, 0 ; Calculated from UEFI map + NUMA info
pmm_node_large_frame_counts dq 0, 0, 0, 0, 0, 0, 0, 0
last_alloc_index_4k dq 0, 0, 0, 0, 0, 0, 0, 0 ; Allocation hints
last_alloc_index_large dq 0, 0, 0, 0, 0, 0, 0, 0
page_size_4k_minus_1 dq PAGE_SIZE_4K - 1
page_size_2m_minus_1 dq PAGE_SIZE_2M - 1
temp_phys_addr dq 0 ; Temp storage for AllocatePages result

; Error codes for PMM init
PMM_INIT_ERR_OK         equ 0
PMM_INIT_ERR_NO_MEM     equ 1
PMM_INIT_ERR_ALLOC_FAIL equ 2
PMM_INIT_ERR_NUMA_CFG   equ 3

section .text

;-----------------------------------------------------------------------------
; pmm_get_node_for_addr: Determines NUMA node for a physical address
; Input: RDI = Physical Address
; Output: RAX = Node ID (0 to numa_node_count-1), or 0 if not found/single node
; Destroys: RCX, RDX
; Assumes pmm_node_base_addrs/limits are populated correctly.
;-----------------------------------------------------------------------------
pmm_get_node_for_addr:
    movzx rcx, byte [numa_node_count]
    cmp rcx, 1
    jle .force_node_0       ; If only 0 or 1 node, always return 0

    xor rax, rax            ; Start checking Node 0
.node_check_loop:
    cmp rax, rcx
    jae .force_node_0       ; Address not found in any configured node range? Default to 0.

    mov rdx, [pmm_node_base_addrs + rax*8]
    cmp rdi, rdx
    jl .next_node           ; Address is lower than this node's base

    mov rdx, [pmm_node_addr_limits + rax*8] ; Check against upper limit (exclusive)
    cmp rdi, rdx
    jge .next_node          ; Address is >= this node's limit

    ; Address is within the current node's range [base, limit)
    ret                     ; Return current Node ID in RAX

.next_node:
    inc rax
    jmp .node_check_loop

.force_node_0:
    xor rax, rax            ; Return Node 0
    ret

;-----------------------------------------------------------------------------
; zero_physical_pages: Helper to zero contiguous physical pages
; Input: RDI = Start Physical address, RCX = Number of 4KB pages
; Destroys: RAX, RDI, RCX, RDX, YMM0 (if AVX used)
;-----------------------------------------------------------------------------
zero_physical_pages:
    test rcx, rcx
    jz .done_zero
    ; Calculate end address (exclusive)
    mov rdx, rcx
    shl rdx, 12 ; rdx = byte_count
    add rdx, rdi ; rdx = end address

.zero_loop:
    cmp rdi, rdx
    jae .done_zero
    ; Zero one page using rep stosq for simplicity/cycles if AVX not guaranteed
    push rcx ; Save outer loop count
    push rdi ; Save current page address
    xor eax, eax
    mov rcx, PAGE_SIZE_4K / 8
    rep stosq ; Zero using 64-bit stores
    pop rdi
    add rdi, PAGE_SIZE_4K ; Move RDI to start of next page
    pop rcx ; Restore outer loop count
    jmp .zero_loop
.done_zero:
    ret

;-----------------------------------------------------------------------------
; calculate_bitmap_size_pages: Calculates pages needed for a bitmap
; Input: RDI = number of frames to track
; Output: RAX = number of 4KB pages needed for the bitmap
; Destroys: RAX, RDX
;-----------------------------------------------------------------------------
calculate_bitmap_size_pages:
    mov rax, rdi ; frame_count
    add rax, 7   ; Add 7 for rounding up division by 8
    shr rax, 3   ; bytes needed = ceil(frame_count / 8)
    add rax, PAGE_SIZE_4K - 1 ; Add page_size-1 for rounding up division
    shr rax, 12  ; pages needed = ceil(bytes_needed / PAGE_SIZE_4K)
    ret

;-----------------------------------------------------------------------------
; mark_bitmap_region: Marks a range of bits in a bitmap
; Input: RDI = Bitmap phys base, RAX = Start frame idx, RDX = End frame idx (excl),
;        R8 = Total frames in node, R9 = Value (0=free, 1=used)
;-----------------------------------------------------------------------------
mark_bitmap_region:
    ; Bounds check start/end indices
    xor rcx, rcx
    cmp rax, rcx; cmovl rax, rcx
    cmp rdx, r8; cmovg rdx, r8
    cmp rax, rdx; jae .done_mark

    mov rbx, rax; shr rbx, 6; mov sil, al; and sil, 63 ; start qword/bit
    mov r10, rdx; shr r10, 6; mov r8b, dl; and r8b, 63 ; end qword/bit (r8=end_q_idx, r10b=end_b_off)

    lea r11, [rdi + rbx*8] ; ptr to start qword

    test r9, r9; jnz .mark_used

.mark_free: ; AND with inverted mask
    cmp rbx, r8; jne .free_multi
    mov rax, -1; mov rcx, -1; shl rax, sil; test r10b, r10b; jz .f_s_ne; shr rcx, (64 - r10b); and rax, rcx; .f_s_ne: not rax; LOCK_PREFIX and [r11], rax; jmp .done_mark
.free_multi:
    mov rax, -1; shl rax, sil; not rax; LOCK_PREFIX and [r11], rax; add r11, 8; inc rbx
    mov rax, 0
.free_middle: cmp rbx, r8; jae .free_last; mov [r11], rax; add r11, 8; inc rbx; jmp .free_middle
.free_last: test r10b, r10b; jz .done_mark; mov rax, -1; shr rax, (64 - r10b); not rax; LOCK_PREFIX and [r11], rax; jmp .done_mark

.mark_used: ; OR with mask
    cmp rbx, r8; jne .used_multi
    mov rax, -1; mov rcx, -1; shl rax, sil; test r10b, r10b; jz .u_s_ne; shr rcx, (64 - r10b); and rax, rcx; .u_s_ne: LOCK_PREFIX or [r11], rax; jmp .done_mark
.used_multi:
    mov rax, -1; shl rax, sil; LOCK_PREFIX or [r11], rax; add r11, 8; inc rbx
    mov rax, -1
.used_middle: cmp rbx, r8; jae .used_last; mov [r11], rax; add r11, 8; inc rbx; jmp .used_middle
.used_last: test r10b, r10b; jz .done_mark; mov rax, -1; shr rax, (64 - r10b); LOCK_PREFIX or [r11], rax

.done_mark: ret

;-----------------------------------------------------------------------------
; pmm64_init_uefi: Initialize PMM using UEFI Memory Map.
; Input: RCX=MapPtr, RDX=MapSize, R8=DescSize
; Output: RAX=Status (0=OK)
;-----------------------------------------------------------------------------
pmm64_init_uefi:
    push rbp; mov rbp, rsp; sub rsp, 8; push rbx; push r12; push r13; push r14; push r15

    mov r12, rcx; mov r13, rdx; mov r14, r8 ; Store inputs
    mov r15, 0; mov qword [pmm_total_frames], 0; mov qword [pmm_total_large_frames], 0
    movzx rcx, byte [numa_node_count]; lea rdi, [pmm_node_frame_counts]; xor eax, eax; push rcx; rep stosq
    lea rdi, [pmm_node_large_frame_counts]; pop rcx; push rcx; rep stosq; pop rcx

    mov rbx, r12; mov r9, r13 ; Ptr and remaining size
.calc_loop: ; Phase 1: Calculate sizes
    cmp r9, r14; jl .calc_done
    mov edi, [rbx + EFI_MEMORY_DESCRIPTOR.Type]; mov rsi, [rbx + EFI_MEMORY_DESCRIPTOR.PhysicalStart]
    mov r10, [rbx + EFI_MEMORY_DESCRIPTOR.NumberOfPages]; mov rax, r10; shl rax, 12; add rax, rsi
    cmp rax, r15; cmova r15, rax ; Update max addr
    cmp edi, EfiConventionalMemory; jne .next_calc_desc
    add [pmm_total_frames], r10; push rsi; push r10; mov rdi, rsi; call pmm_get_node_for_addr
    pop r10; pop rsi; add [pmm_node_frame_counts + rax*8], r10
.next_calc_desc: add rbx, r14; sub r9, r14; jmp .calc_loop
.calc_done: mov [pmm_max_ram_addr], r15
    cmp qword [pmm_total_frames], 0; je .fail_no_mem

    movzx rcx, byte [numa_node_count]; xor rbx, rbx ; Node index
.calc_large_loop: ; Calculate large counts
    cmp rbx, rcx; jae .calc_large_done; mov rax, [pmm_node_frame_counts + rbx*8]
    mov rdx, rax; shr rdx, 9; mov [pmm_node_large_frame_counts + rbx*8], rdx; add [pmm_total_large_frames], rdx
    inc rbx; jmp .calc_large_loop
.calc_large_done:

    movzx rcx, byte [numa_node_count]; xor rbx, rbx ; Node index
.alloc_bitmap_loop: ; Phase 2: Allocate bitmaps
    cmp rbx, rcx; jae .alloc_bitmaps_done
    mov rdi, [pmm_node_frame_counts + rbx*8]; test rdi, rdi; jz .skip_4k_alloc
    call calculate_bitmap_size_pages; mov r8, rax; test r8, r8; jz .skip_4k_alloc
    mov r10, rcx; mov r11, rdx; mov rcx, AllocateAnyPages; mov rdx, EfiLoaderData; lea r9, [temp_phys_addr]
    call uefi_AllocatePagesWrapper; mov rcx, r10; mov rdx, r11; test rax, rax; jnz .fail_alloc
    mov rdi, [temp_phys_addr]; mov [pmm_node_bitmaps + rbx*8], rdi; push rbx; push rcx; mov rcx, r8; call zero_physical_pages; pop rcx; pop rbx
.skip_4k_alloc:
    mov rdi, [pmm_node_large_frame_counts + rbx*8]; test rdi, rdi; jz .skip_large_alloc
    call calculate_bitmap_size_pages; mov r8, rax; test r8, r8; jz .skip_large_alloc
    mov r10, rcx; mov r11, rdx; mov rcx, AllocateAnyPages; mov rdx, EfiLoaderData; lea r9, [temp_phys_addr]
    call uefi_AllocatePagesWrapper; mov rcx, r10; mov rdx, r11; test rax, rax; jnz .fail_alloc
    mov rdi, [temp_phys_addr]; mov [pmm_node_large_bitmaps + rbx*8], rdi; push rbx; push rcx; mov rcx, r8; call zero_physical_pages; pop rcx; pop rbx
.skip_large_alloc: inc rbx; jmp .alloc_bitmap_loop
.alloc_bitmaps_done:

    movzx rcx, byte [numa_node_count]; xor rbx, rbx ; Node index
.init_bitmap_loop: ; Phase 3: Mark all used
    cmp rbx, rcx; jae .init_bitmaps_done
    mov rdi, [pmm_node_bitmaps + rbx*8]; test rdi, rdi; jz .next_init_bitmap_part
    mov rsi, [pmm_node_frame_counts + rbx*8]; add rsi, 63; shr rsi, 6; test rsi, rsi; jz .next_init_bitmap_part
    mov rax, -1; push rcx; push rbx; mov rcx, rsi; rep stosq; pop rbx; pop rcx
.next_init_bitmap_part:
    mov rdi, [pmm_node_large_bitmaps + rbx*8]; test rdi, rdi; jz .next_init_node
    mov rsi, [pmm_node_large_frame_counts + rbx*8]; add rsi, 63; shr rsi, 6; test rsi, rsi; jz .next_init_node
    mov rax, -1; push rcx; push rbx; mov rcx, rsi; rep stosq; pop rbx; pop rcx
.next_init_node: inc rbx; jmp .init_bitmap_loop
.init_bitmaps_done:

    mov rbx, r12; mov r9, r13 ; Ptr and remaining size
.free_conv_loop: ; Phase 4: Free conventional memory
    cmp r9, r14; jl .free_conv_done
    mov edi, [rbx + EFI_MEMORY_DESCRIPTOR.Type]; cmp edi, EfiConventionalMemory; jne .next_free_desc
    mov rsi, [rbx + EFI_MEMORY_DESCRIPTOR.PhysicalStart]; mov r10, [rbx + EFI_MEMORY_DESCRIPTOR.NumberOfPages]
    mov rax, rsi; mov rdx, r10; shl rdx, 12; add rdx, rax; call pmm_mark_region_free
.next_free_desc: add rbx, r14; sub r9, r14; jmp .free_conv_loop
.free_conv_done:

    ; Phase 5: Mark critical regions used
    mov rax, [efi_image_base]; mov rdx, [efi_image_size]; add rdx, rax; call pmm_mark_region_used
    movzx rcx, byte [numa_node_count]; xor rbx, rbx
.mark_bitmap_loop:
    cmp rbx, rcx; jae .mark_bitmaps_done
    mov rax, [pmm_node_bitmaps + rbx*8]; test rax, rax; jz .next_mark_bitmap_part
    mov rdi, [pmm_node_frame_counts + rbx*8]; call calculate_bitmap_size_pages; mov rdx, rax; shl rdx, 12; add rdx, rax; call pmm_mark_region_used
.next_mark_bitmap_part:
    mov rax, [pmm_node_large_bitmaps + rbx*8]; test rax, rax; jz .next_mark_node
    mov rdi, [pmm_node_large_frame_counts + rbx*8]; call calculate_bitmap_size_pages; mov rdx, rax; shl rdx, 12; add rdx, rax; call pmm_mark_region_used
.next_mark_node: inc rbx; jmp .mark_bitmap_loop
.mark_bitmaps_done:

    mov rax, PMM_INIT_ERR_OK; jmp .exit
.fail_no_mem: mov rax, PMM_INIT_ERR_NO_MEM; jmp .exit_final
.fail_alloc: mov rax, PMM_INIT_ERR_ALLOC_FAIL; jmp .exit_final
.exit: pop r15; pop r14; pop r13; pop r12; pop rbx; add rsp, 8; pop rbp
.exit_final: ret

; --- Core Allocation/Freeing/Marking Functions ---
; pmm_mark_region_used, pmm_mark_region_free, pmm_alloc_frame_node,
; pmm_alloc_frame, pmm_free_frame_node, pmm_free_frame,
; pmm_alloc_large_frame, pmm_free_large_frame remain mostly the same
; as provided in the previous version, relying on the node structures
; and calling mark_bitmap_region. Ensure they use the *_minus_1 vars for alignment checks.

; Include the rest of the functions (alloc, free, mark) from the previous pmm64.asm version here...
; Make sure alignment checks use page_size_4k_minus_1 / page_size_2m_minus_1

; --- Placeholder for brevity - Copy the functions below from previous pmm64.asm ---
pmm_mark_region_used:
    ; ... (Copy implementation from previous pmm64.asm, ensure it uses page_size_*_minus_1) ...
    ret
pmm_mark_region_free:
    ; ... (Copy implementation from previous pmm64.asm, ensure it uses page_size_*_minus_1) ...
    ret
pmm_alloc_frame_node:
    ; ... (Copy implementation from previous pmm64.asm) ...
    ret
pmm_alloc_frame:
    mov rcx, -1
    jmp pmm_alloc_frame_node
pmm_free_frame_node:
    ; ... (Copy implementation from previous pmm64.asm, ensure it uses page_size_*_minus_1) ...
    ret
pmm_free_frame:
    jmp pmm_free_frame_node
pmm_alloc_large_frame:
    ; ... (Copy implementation from previous pmm64.asm) ...
     ; Needs input RCX = node_id (-1 for any)
    ret
pmm_free_large_frame:
    ; ... (Copy implementation from previous pmm64.asm, ensure it uses page_size_*_minus_1) ...
    ret