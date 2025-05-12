;**File 10: `keyboard.asm`**

;```assembly
; keyboard.asm: Interrupt-driven PS/2 Keyboard Handler (64-bit)
; Depends on: boot_defs_temp.inc

BITS 64
global keyboard_init, keyboard_interrupt_handler, getchar_from_buffer
extern pic_send_eoi ; Assumes PIC is used and EOI function exists

%include "boot_defs_temp.inc"

; Requires these globals defined elsewhere (e.g., main_uefi_loader.asm .bss)
extern key_buffer, key_buffer_head, key_buffer_tail

section .data align=1
kb_lshift_pressed db 0
kb_rshift_pressed db 0
kb_capslock_on    db 0
; Scancode Set 1 to ASCII (US Layout)
scan_code_table: ; Normal (unshifted) keys - Use WORD for simpler indexing
    dw 0, 27, '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', 8, 9   ; 0x00-0x0F (NULL,ESC,1-0,-,=,BS,TAB)
    dw 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', 13, 0, 'a', 's' ; 0x10-0x1F (Q-P,[,],Enter,LCtrl,A,S)
    dw 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', '`', 0, '\\', 'z', 'x', 'c', 'v' ; 0x20-0x2F (D-L,;,',`,LShift,\,Z-V)
    dw 'b', 'n', 'm', ',', '.', '/', 0, 0, 0, ' ', 0, 0, 0, 0, 0, 0   ; 0x30-0x3F (B-M,,,.,/,RShift,KP*,LAlt,Space,Caps,F1-F6)
    dw 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0x40-0x4F (F7-F12 placeholders, others...) Map Kp7-Kp1 later if needed
    dw 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0x50-0x58 Map Kp2,3,0,. later if needed
SCAN_CODE_TABLE_SIZE equ ($ - scan_code_table) / 2
shifted_scan_code_table: ; Shifted keys
    dw 0, 27, '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '_', '+', 8, 9
    dw 'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '{', '}', 13, 0, 'A', 'S'
    dw 'D', 'F', 'G', 'H', 'J', 'K', 'L', ':', '"', '~', 0, '|', 'Z', 'X', 'C', 'V'
    dw 'B', 'N', 'M', '<', '>', '?', 0, 0, 0, ' ', 0, 0, 0, 0, 0, 0
    dw 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    dw 0, 0, 0, 0, 0, 0, 0, 0, 0
SHIFTED_SCAN_CODE_TABLE_SIZE equ ($ - shifted_scan_code_table) / 2

section .text

;--------------------------------------------------------------------------
; keyboard_init: Initialize keyboard state flags
;--------------------------------------------------------------------------
keyboard_init:
    mov byte [kb_lshift_pressed], 0
    mov byte [kb_rshift_pressed], 0
    mov byte [kb_capslock_on], 0
    ret

;--------------------------------------------------------------------------
; keyboard_interrupt_handler: PS/2 Keyboard IRQ Handler
; NOTE: This is pointed to directly by the IDT entry. It must save/restore
;       all potentially modified registers and end with IRETQ.
;--------------------------------------------------------------------------
keyboard_interrupt_handler:
    ; Save registers potentially clobbered before panic/EOI/buffer access
    push rax; push rbx; push rcx; push rdx; push rsi; push rdi; push r8

    in al, KB_DATA_PORT     ; Read scancode from keyboard controller
    mov cl, al              ; Use CL for scancode processing

    test cl, 0x80           ; Test high bit (key release)
    jnz .key_release

.key_press:
    cmp cl, SC_LSHIFT_MAKE  ; Check for modifier presses
    je .lshift_press
    cmp cl, SC_RSHIFT_MAKE
    je .rshift_press
    cmp cl, SC_CAPSLOCK_MAKE
    je .capslock_toggle

    ; --- Key Press - Translate Scancode ---
    movzx rsi, cl           ; RSI = scancode index (zero-extended)
    cmp rsi, SCAN_CODE_TABLE_SIZE ; Bounds check
    jae .ignore_key         ; If index too high, ignore

    ; Determine if shift is active
    mov r8b, 0              ; R8B = shift_active flag
    cmp byte [kb_lshift_pressed], 1
    je .shift_is_active
    cmp byte [kb_rshift_pressed], 1
    je .shift_is_active
    jmp .get_base_char      ; No shift pressed

.shift_is_active:
    mov r8b, 1              ; Set shift_active flag

.get_base_char:
    mov ax, [scan_code_table + rsi*2] ; Get base character (or 0)
    mov dl, al              ; Save base char in DL for caps logic

    ; Apply Caps Lock only to 'a'-'z'
    cmp al, 'a'; jl .apply_shift
    cmp al, 'z'; jg .apply_shift
    ; Is Alpha char
    test r8b, 1             ; Shift pressed?
    jnz .alpha_shift_pressed
    ; Shift NOT pressed
    cmp byte [kb_capslock_on], 1 ; Caps Lock ON?
    jne .use_base_char      ; No, use lowercase base char (already in AX)
    mov ax, [shifted_scan_code_table + rsi*2] ; Yes, use uppercase char
    jmp .char_translated
.alpha_shift_pressed:
    ; Shift IS pressed
    cmp byte [kb_capslock_on], 1 ; Caps Lock ON?
    jne .use_shifted_char   ; No, use standard shifted char (uppercase)
    mov ax, dx              ; Yes, use inverted caps (lowercase, saved in DL)
    jmp .char_translated

.apply_shift:               ; Not an alpha char, apply normal shift
    test r8b, 1             ; Shift pressed?
    jz .use_base_char       ; No, use base char

.use_shifted_char:          ; Get shifted char
    mov ax, [shifted_scan_code_table + rsi*2]
    jmp .char_translated

.use_base_char:
    ; AX already contains base char from first lookup
    ; Fall through

.char_translated:
    or ax, ax               ; Check if valid char (not 0 from table)
    jz .ignore_key          ; Ignore if 0

    ; --- Add character to buffer ---
    movzx rdi, word [key_buffer_head] ; Current head index
    movzx rcx, word [key_buffer_tail] ; Current tail index

    mov rbx, rdi            ; Save head index before incrementing
    inc di                  ; Increment head index
    cmp di, KEY_BUFFER_SIZE ; Wrap around buffer size?
    jne .no_wrap
    xor di, di              ; Wrap to 0
.no_wrap:
    cmp di, cx              ; Check if head caught up to tail (buffer full)
    je .buffer_full

    ; Buffer not full, store character (AL) and update head
    mov [key_buffer + rbx], al ; Store char at old head position
    mov [key_buffer_head], di ; Update head pointer
    jmp .send_eoi           ; Go send End-Of-Interrupt

.buffer_full:
    ; Optional: Signal buffer full error? For now, just ignore key.
    jmp .ignore_key

.key_release:
    and cl, 0x7F            ; Mask off the release bit (high bit)
    cmp cl, SC_LSHIFT_MAKE
    je .lshift_release
    cmp cl, SC_RSHIFT_MAKE
    je .rshift_release
    ; Ignore other key releases
    jmp .ignore_key

.lshift_press: mov byte [kb_lshift_pressed], 1; jmp .ignore_key
.lshift_release: mov byte [kb_lshift_pressed], 0; jmp .ignore_key
.rshift_press: mov byte [kb_rshift_pressed], 1; jmp .ignore_key
.rshift_release: mov byte [kb_rshift_pressed], 0; jmp .ignore_key
.capslock_toggle: mov al, [kb_capslock_on]; xor al, 1; mov [kb_capslock_on], al; jmp .ignore_key

.ignore_key:
    ; Key was ignored (modifier, release, unknown, or buffer full)
.send_eoi:
    ; Send End-Of-Interrupt to PIC
    ; TODO: If using APIC, send EOI to local APIC instead!
    mov al, PIC_EOI
    out PIC1_COMMAND, al
    ; If IRQ > 7, also send to PIC2:
    ; cmp cl, 8 ; Check original scancode? Need to save it if checking here.
    ; jb .eoi_done
    ; out PIC2_COMMAND, al
.eoi_done:
    ; Restore registers and return from interrupt
    pop r8; pop rdi; pop rsi; pop rdx; pop rcx; pop rbx; pop rax
    iretq

;--------------------------------------------------------------------------
; getchar_from_buffer: Reads next character from keyboard buffer.
; Output: AL = Character, or 0 if buffer is empty.
;--------------------------------------------------------------------------
getchar_from_buffer:
    push rbx ; Save clobbered register
    movzx rbx, word [key_buffer_tail] ; Get current tail index
    movzx rcx, word [key_buffer_head] ; Get current head index
    cmp bx, cx ; Is tail == head?
    je .empty ; Yes, buffer is empty

    ; Buffer not empty, read character and advance tail
    mov al, [key_buffer + rbx] ; Read char from buffer
    inc bx                  ; Increment tail index
    cmp bx, KEY_BUFFER_SIZE ; Wrap around buffer size?
    jne .tail_no_wrap
    xor bx, bx              ; Wrap to 0
.tail_no_wrap:
    mov [key_buffer_tail], bx ; Update tail pointer
.not_empty:
    ; AL contains character
    pop rbx
    ret
.empty:
    ; AL is implicitly 0 from CMP? Ensure it is.
    xor al, al ; Return 0 for empty buffer
    pop rbx
    ret