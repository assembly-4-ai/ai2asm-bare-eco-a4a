;**File 14: `shell.asm`**

;```assembly
; shell.asm: Command Shell (Post-ExitBootServices)
; Depends on: boot_defs_temp.inc, screen_gop.asm, keyboard.asm,
;             fat32.asm, pmm64_uefi.asm, compute libraries

BITS 64
global shell_run
; Required Externals
extern fat32_read_file ; From fat32.asm
extern pmm_alloc_large_frame, pmm_free_large_frame ; From pmm64_uefi.asm
extern scr64_print_string, scr64_print_hex, scr64_print_dec, scr64_print_char ; From screen_gop.asm
extern getchar_from_buffer ; From keyboard.asm (or wrapper calling it)
extern panic64 ; From payload/panic module
; Compute function externals (to be linked)
extern ggml_matmul, llama_model_load, parallel_run_model, gpu_matmul
; Potentially needed helpers
extern memcpy64, memset64, strcmp64, itoa64 ; Assumed utilities

%include "boot_defs_temp.inc"

section .data align=64
sh_prompt       db "> ", 0 ; Shortened prompt
sh_eol          db 0Dh, 0Ah, 0
cmd_help        db "help", 0
cmd_exit        db "exit", 0
cmd_matmul      db "matmul", 0 ; Usage: matmul <size_mb>
cmd_loadmodel   db "loadmodel", 0 ; Usage: loadmodel <size_mb> <filename>
cmd_runmodel    db "runmodel", 0 ; Usage: runmodel <filename> (loads then runs)
cmd_cls         db "cls", 0 ; Example: Clear screen
msg_help        db "Commands: help, exit, cls, matmul, loadmodel, runmodel", 0Dh, 0Ah, 0
msg_unknown_cmd db "Unknown command", 0Dh, 0Ah, 0
msg_matmul_start db "Starting Matmul...", 0Dh, 0Ah, 0
msg_matmul_done db "Matmul Done!", 0Dh, 0Ah, 0
msg_alloc_fail  db "Allocation Failed!", 0Dh, 0Ah, 0
msg_parse_fail  db "Parse Error!", 0Dh, 0Ah, 0
msg_load_fail   db "Load Failed: ", 0
msg_loading_model db "Loading Model: ", 0
msg_model_size  db " Size (MB): ", 0
msg_model_loaded db " Model Loaded!", 0Dh, 0Ah, 0
msg_running_model db "Running Model: ", 0
msg_run_fail    db "Run Failed!", 0Dh, 0Ah, 0
msg_run_ok      db "Run OK!", 0Dh, 0Ah, 0

; Simple argument storage
MAX_ARGS equ 8
argv dq 0, 0, 0, 0, 0, 0, 0, 0 ; Array of pointers to tokens
argc dq 0 ; Argument count

section .bss align=16
sh_input_buffer resb 256
sh_token_buffer resb 256 ; Buffer to hold token pointers and null terminators

section .text

;--------------------------------------------------------------------------
; strcmp64_simple: Simple string compare for commands
; Input: RDI = string1 (null terminated), RSI = string2 (null terminated)
; Output: ZF=1 if equal, ZF=0 if not equal
; Destroys: AL, BL, RDI, RSI
;--------------------------------------------------------------------------
strcmp64_simple:
.loop:
    mov al, [rdi]
    mov bl, [rsi]
    cmp al, bl
    jne .noteq
    test al, al ; Check for null terminator
    jz .eq ; Both ended at the same time
    inc rdi
    inc rsi
    jmp .loop
.noteq:
    cmp al, bl ; Set ZF=0
    ret
.eq:
    ; ZF is already set
    ret

;--------------------------------------------------------------------------
; parse_uint64: Converts ASCII decimal string to 64-bit unsigned integer
; Input: RSI = Pointer to string
; Output: RAX = Resulting integer, RDX = Pointer after last digit parsed.
;         Carry set if invalid char found or overflow (overflow not checked here).
; Destroys: RAX, RCX, RDX
;--------------------------------------------------------------------------
parse_uint64:
    xor rax, rax ; Result
    mov rdx, rsi ; Keep track of position
    mov rcx, 10 ; Base
.parse_loop:
    movzx r8, byte [rdx] ; Get char
    test r8b, r8b
    jz .done_ok ; End of string

    cmp r8b, '0'
    jb .fail
    cmp r8b, '9'
    ja .fail

    ; Valid digit
    sub r8b, '0' ; Convert char to integer value
    ; Check for overflow before multiplying? For simplicity, skip now.
    imul rax, rax, 10 ; result *= 10 (Use IMUL for potential neg later?)
    add rax, r8 ; result += digit
    inc rdx ; Next char
    jmp .parse_loop

.fail:
    stc ; Set carry on error
    ret
.done_ok:
    clc ; Clear carry on success
    ret

;--------------------------------------------------------------------------
; read_input: Reads a line from keyboard into buffer using getchar
; Input: RDI = buffer, RCX = max size
; Output: RAX = length read (excluding null)
;--------------------------------------------------------------------------
read_input:
    push rbx; push rcx; push rdi
    xor rbx, rbx ; Current buffer index/length
.read_loop:
    call getchar_from_buffer ; Get char (blocking) -> AL
    cmp al, 0x0D ; Enter key?
    je .input_done
    cmp al, 0x08 ; Backspace?
    je .backspace

    ; Normal character
    cmp rbx, rcx ; Check buffer overflow (leave space for null)
    jge .read_loop ; Buffer full, ignore char

    mov [rdi + rbx], al ; Store char
    inc rbx
    ; Echo character
    push rax; call scr64_print_char; pop rax
    jmp .read_loop

.backspace:
    test rbx, rbx
    jz .read_loop ; Nothing to backspace
    dec rbx
    ; Echo backspace, space, backspace
    mov al, 8; call scr64_print_char
    mov al, ' '; call scr64_print_char
    mov al, 8; call scr64_print_char
    jmp .read_loop

.input_done:
    mov byte [rdi + rbx], 0 ; Null terminate
    mov rax, rbx ; Return length
    ; Print newline
    push rax; mov al, 10; call scr64_print_char; pop rax
    pop rdi; pop rcx; pop rbx
    ret

;--------------------------------------------------------------------------
; tokenize: Splits input string into space-separated tokens
; Input: RSI = Input string, RDI = argv buffer (array of pointers)
;        RCX = Max Args
; Output: RAX = argc, RDI contains argv pointers
; Destroys: RDX, R8, R9, R10
; Modifies input string by inserting null terminators!
;--------------------------------------------------------------------------
tokenize:
    mov r8, rdi ; Save start of argv buffer
    mov r9, 0 ; Current arg count (argc)
    mov r10, rcx ; Save max args

.skip_leading_whitespace:
    mov dl, [rsi]
    test dl, dl ; End of string?
    jz .tokenize_done
    cmp dl, ' ' ; Space?
    je .found_space
    cmp dl, 9 ; Tab?
    je .found_space
    ; Found start of a token
    jmp .start_token
.found_space:
    inc rsi
    jmp .skip_leading_whitespace

.start_token:
    cmp r9, r10 ; Check max args
    jge .tokenize_done ; Too many args

    mov [rdi], rsi ; Store pointer to start of token in argv
    add rdi, 8 ; Next argv slot
    inc r9 ; Increment argc

.scan_token:
    mov dl, [rsi]
    test dl, dl
    jz .tokenize_done ; End of string is end of token
    cmp dl, ' '
    je .end_token
    cmp dl, 9
    je .end_token
    ; Character is part of token
    inc rsi
    jmp .scan_token

.end_token:
    mov byte [rsi], 0 ; Null terminate the token in the input string
    inc rsi
    jmp .skip_leading_whitespace ; Look for next token

.tokenize_done:
    
    mov [rdi], byte 0 ; Null terminate argv array
    mov rax, r9 ; Return argc
    mov rdi, r8 ; Return original argv buffer start
    ret

;--------------------------------------------------------------------------
; shell_run: Main shell loop
;--------------------------------------------------------------------------
shell_run:
    push rbx; push r12; push r13; push r14; push r15

.prompt_loop:
    ; Print prompt
    mov rsi, sh_prompt
    call scr64_print_string

    ; Read input
    lea rdi, [sh_input_buffer]
    mov rcx, 255 ; Max chars for input buffer
    call read_input
    test rax, rax ; Check if any input was read
    jz .prompt_loop ; Empty line, just re-prompt

    ; Tokenize input
    lea rdi, [argv] ; Destination argv array
    lea rsi, [sh_input_buffer] ; Source input string
    mov rcx, MAX_ARGS
    call tokenize
    mov [argc], rax ; Store argc

    ; Process command if argc > 0
    cmp rax, 0
    jle .prompt_loop ; No command entered

    ; Get first token (command)
    mov rbx, [argv] ;    RBX = argv[0] (command string)

    ; --- Command Dispatch ---
    mov rdi, rbx;
    lea rsi, [cmd_help];
    call strcmp64_simple;
    jz .cmd_help_handler
    mov rdi, rbx;
    lea rsi, [cmd_exit];
    call strcmp64_simple;
    jz .cmd_exit_handler
    mov rdi, rbx;
    lea rsi, [cmd_matmul];
    call strcmp64_simple;
    jz .cmd_matmul_handler
    mov rdi, rbx;
    lea rsi, [cmd_loadmodel];
    call strcmp64_simple;
    jz .cmd_loadmodel_handler
    mov rdi, rbx;
    lea rsi, [cmd_runmodel];
    call strcmp64_simple;
    jz .cmd_runmodel_handler
    mov rdi, rbx;
    lea rsi, [cmd_cls];
    call strcmp64_simple;
    jz .cmd_cls_handler

    ; Unknown command
    mov rsi, msg_unknown_cmd
    call scr64_print_string
    jmp .prompt_loop

.cmd_help_handler:
    mov rsi, msg_help
    call scr64_print_string
    jmp .prompt_loop

.cmd_exit_handler:
    jmp .shell_exit

.cmd_cls_handler:
    ; TODO: Implement screen clear using scr64 functions
    jmp .prompt_loop

.cmd_matmul_handler:
    cmp qword [argc], 2 ; Check for "matmul <size>"
    jne .parse_fail_shell
    mov rsi, [argv+8] ; Argv[1] = size string
    call parse_uint64
    jc .parse_fail_shell ; Carry set means invalid number
    ; RAX = size in MB
    ; Allocate buffers (Example: A=size, B=size, C=size)
    mov r12, rax ; R12 = size_mb
    shl r12, 20 ; size_bytes
    mov rcx, -1;
    call pmm_alloc_large_frame;
    mov r13, rax ; Alloc A
    mov rcx, -1;
    call pmm_alloc_large_frame;
    mov r14, rax ; Alloc B
    mov rcx, -1;
    call pmm_alloc_large_frame;
    mov r15, rax ; Alloc C
    test r13, r13;
    jz .alloc_fail_matmul
    test r14, r14;
    jz .alloc_fail_matmul
    test r15, r15;
    jz .alloc_fail_matmul
    ; Buffers allocated
    mov rsi, msg_matmul_start;
    call scr64_print_string
    ; --- Call ggml_matmul ---
    mov rdi, r13;
    mov rsi, r14;
    mov rdx, r15 ; Args: A, B, C buffers
    mov rcx, r12;
    shr rcx, 2 ; Size in elements? Assuming 4-byte floats? Adjust as needed.
    ;Need to disable interrupts around compute?
    cli
    call ggml_matmul
    sti
    ; --- Compute Done ---
    mov rsi, msg_matmul_done;
    call scr64_print_string
    ; Free buffers
    mov rdi, r15;
    mov rcx, -1;
    call pmm_free_large_frame
    mov rdi, r14;
    mov rcx, -1;
    call pmm_free_large_frame
    mov rdi, r13;
    mov rcx, -1;
    call pmm_free_large_frame
    jmp .prompt_loop
.alloc_fail_matmul:
    ; Free any successfully allocated buffers before failing
    test r15, r15;
    jz .check_b_fail;
    mov rdi, r15;
    mov rcx, -1;
    call pmm_free_large_frame
.check_b_fail:
    test r14, r14;
    jz .check_a_fail;
    mov rdi, r14;
    mov rcx, -1;
    call pmm_free_large_frame
.check_a_fail:
    test r13, r13;
    jz .alloc_msg_done;
    mov rdi, r13;
    mov rcx, -1;
    call pmm_free_large_frame
.alloc_msg_done:
    mov rsi, msg_alloc_fail; call scr64_print_string
    jmp .prompt_loop


.cmd_loadmodel_handler:
    cmp qword [argc], 3 ; Check for "loadmodel <size_mb> <filename>"
    jne .parse_fail_shell
    ; Parse size
    mov rsi, [argv+8] ; Argv[1] = size string
    call parse_uint64
    jc .parse_fail_shell
    test rax, rax
    jz .parse_fail_shell ; Size cannot be 0
    mov r12, rax ; R12 = size_mb
    shl r12, 20 ; R12 = size_bytes
    ; Get filename
    mov rdi, [argv+16] ; Argv[2] = filename

    ; Print messages
    mov rsi, msg_loading_model;
    call scr64_print_string
    mov rsi, rdi;
    call scr64_print_string ; Print filename
    mov rsi, msg_model_size;
    call scr64_print_string
    mov rax, r12;
    shr rax, 20;
    call scr64_print_dec ; Print size_mb
    mov rsi, sh_eol;
    call scr64_print_string

    ; Allocate buffer for model
    mov rcx, -1; call pmm_alloc_large_frame ; Request pages based on size_bytes (pmm_alloc_large_frame needs size/page count?)
    ; *** NOTE: pmm_alloc_large_frame needs modification to take size/pages ***
    ; For now, assume it allocates one large frame - THIS IS LIKELY WRONG
    test rax, rax
    jz .alloc_fail_shell
    mov rsi, rax ; RSI = buffer physical address

    ; Read file into buffer
    ; RDI=Filename, RSI=Buffer, RDX=Size, R8B=Drive(needs setting?)
    mov rdx, r12 ; Bytes to read
    ; Need boot drive number? mov r8b, [gBootDriveNum]
    call fat32_read_file
    jnc .load_ok
    ; Read failed
    mov r14, rax ; Save error code
    mov rdi, rsi ; Buffer to free
    mov rcx, -1;
    call pmm_free_large_frame
    mov rsi, msg_load_fail;
    call scr64_print_string
    mov rax, r14;
    call scr64_print_hex ; Print FAT error code
    mov rsi, sh_eol;
    call scr64_print_string
    jmp .prompt_loop
.load_ok:
    ; RAX contains bytes read, compare with expected size?
    mov rsi, msg_model_loaded; call scr64_print_string
    ; --- Call llama_model_load ---
    ; Assuming it takes buffer pointer and size
    mov rdi, rsi ; Buffer address
    mov rsi, r12 ; Size in bytes
    call llama_model_load
    ; llama_model_load might take ownership or copy, free buffer? Depends on its API.
    ; If llama_model_load doesn't copy, DON'T free the buffer here.
    mov rdi, [rsp+8];
    mov rcx, -1;
    call pmm_free_large_frame ; Free if copied
    jmp .prompt_loop

.cmd_runmodel_handler:
    ; Similar to loadmodel, but then calls parallel_run_model or similar
    ; Needs implementation
    ;mov rsi, todo runmodel needs inplementation 
    ;call scr64_print_string
    mov rsi, sh_eol;
    call scr64_print_string
    jmp .prompt_loop


.parse_fail_shell:
    mov rsi, msg_parse_fail
    call scr64_print_string
    jmp .prompt_loop
.alloc_fail_shell:
    mov rsi, msg_alloc_fail;
    call scr64_print_string
    jmp .prompt_loop


.shell_exit:
    ; Maybe print a message before halting?
    pop r15;
    pop r14;
    pop r13;
    pop r12;
    pop rbx
    ret ; Return to caller (main_uefi_loader)
