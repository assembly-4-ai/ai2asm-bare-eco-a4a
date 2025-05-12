; screen_gop.asm: Screen Output using UEFI GOP Framebuffer (Post-ExitBS)
; Uses CALL/RET. Strict one instruction per line.
; Depends on: boot_defs_temp.inc

BITS 64
global scr64_init, scr64_print_string, scr64_print_hex, scr64_print_dec, scr64_print_char

extern panic64
extern itoa64
extern simple_font_bitmap ; Assume 8x16 font bitmap data provided
;extern gop_framebuffer_base ,gop_fb_base, gop_h_res,gop_v_res,gop_pixels_per_scanline, gop_pixel_format ,PixelBlueGreenRedReserved8BitPerColor ; Default assumption
;extern cursor_x,cursor_y,font_height, font_width, font_fg_color, font_bg_color, hex_digits

%include "boot_defs_temp.inc"

section .data allign=8

    gop_framebuffer_base dq 0;
    gop_framebuffer_size dq 0;
    gop_fb_base dq 0
    gop_h_res dd 0
    gop_v_res dd 0
    gop_pixels_per_scanline dd 0
    gop_pixel_format dd PixelBlueGreenRedReserved8BitPerColor ; Default assumption
    cursor_x dw 0
    cursor_y dw 0
    font_height db 16
    font_width db 8
    font_fg_color dd 0x00FFFFFF ; White BGRA
    font_bg_color dd 0x00000000 ; Black BGRA
    hex_digits db "0123456789ABCDEF"

section .bss align=16
dec_buffer resb 32

section .text


;--------------------------------------------------------------------------
scr64_init: ; Input: RDI=FB Base, RSI=HRes, RDX=VRes, RCX=PixelsPerScanline, R8=PixelFormat
    mov [gop_fb_base], rdi
    mov [gop_h_res], esi     ; Store HRes (as dword)
    mov [gop_v_res], edx     ; Store VRes (as dword)
    mov [gop_pixels_per_scanline], ecx ; Store PixelsPerScanline (as dword)
    mov [gop_pixel_format], r8d ; Store PixelFormat (as dword)
    mov word [cursor_x], 0
    mov word [cursor_y], 0
    ret

;--------------------------------------------------------------------------
putpixel: ; Input: ECX=x, EDX=y, R8D=color (BGRA32)
    ; Save potentially clobbered registers if necessary for caller
    push rax
    push rdi
    push r10
    push r11

    ; Bounds check
    cmp ecx, [gop_h_res]
    jge .putpixel_exit
    cmp edx, [gop_v_res]
    jge .putpixel_exit

    ; Calculate offset: offset = (y * pixels_per_scanline + x) * 4
    mov r10d, edx           ; R10D = y
    imul r10, [gop_pixels_per_scanline] ; R10 = y * PixelsPerScanline ; was r10,r10,[gop_pixels_per_scanline]
    mov r11d, ecx           ; R11D = x
    add r10, r11            ; R10 = y * ppsl + x
    shl r10, 2              ; R10 = offset in bytes ( * 4 for 32bpp)

    mov rdi, [gop_fb_base]  ; Framebuffer base
    add rdi, r10            ; RDI = Pixel address
    mov [rdi], r8d          ; Write color (dword)

.putpixel_exit:
    pop r11
    pop r10
    pop rdi
    pop rax
    ret

;--------------------------------------------------------------------------
scroll_screen:
    ; Save registers used
    push rsi
    push rdi
    push rcx
    push rdx
    push rax
    push r10
    push r11

    ; Calculate bytes per text line
    movzx r10, byte [font_height]
    mov r11d, [gop_pixels_per_scanline]
    imul r10, r11 ; R10 = pixels per text line
    shl r10, 2    ; R10 = bytes per text line (32bpp)

    ; Calculate source/destination for scroll copy
    mov rsi, [gop_fb_base]
    add rsi, r10  ; Source = FB + one line
    mov rdi, [gop_fb_base] ; Destination = FB start

    ; Calculate bytes to copy
    mov ecx, [gop_v_res]
    movzx edx, byte [font_height]
    sub ecx, edx  ; Number of lines to scroll
    imul rcx, r10 ; Total bytes to copy

    test rcx, rcx
    jle .scroll_clear ; Skip copy if no lines or negative

    ; Copy lines up
    rep movsb

.scroll_clear:
    ; Calculate start address of last text line to clear
    mov rdi, [gop_fb_base]
    mov eax, [gop_v_res]
    movzx edx, byte [font_height]
    sub eax, edx
    imul rax, [gop_pixels_per_scanline]
    shl rax, 2
    add rdi, rax ; RDI = Start of last line

    ; Clear using background color (DWORDs)
    mov rcx, r10 ; Bytes per line
    shr rcx, 2   ; Number of DWORDs per line
    mov eax, [font_bg_color]
    rep stosd

    ; Restore registers
    pop r11
    pop r10
    pop rax
    pop rdx
    pop rcx
    pop rdi
    pop rsi
    ret

;--------------------------------------------------------------------------
scr64_putchar_at: ; Input: AL=Char, BX=X_char_pos, CX=Y_char_pos
    ; Save ALL potentially used registers (inc. by putpixel)
    push rax; push rbx; push rcx; push rdx; push rsi; push rdi
    push r8; push r9; push r10; push r11; push r12; push r13; push r14; push r15

    ; Keep original char/coords safe
    mov r14b, al ; Char
    mov r14w, bx ; X pos
    mov r15w, cx ; Y pos

    ; Calculate pixel coordinates top-left corner
    movzx r10, byte [font_width]
    movzx r11, byte [font_height]
    movzx rbx, r14w ; X char pos
    imul rbx, r10 ; RBX = start x_pixel
    movzx rcx, r15w ; Y char pos
    imul rcx, r11 ; RCX = start y_pixel

    ; Calculate offset in font bitmap data
    movzx rax, r14b ; Char ASCII value
    imul rax, r11 ; RAX = ASCII * font_height (16)
    lea rsi, [simple_font_bitmap] ; Base of font data
    add rsi, rax ; RSI points to start of char bitmap

    ; Loop through font pixels (y_font = 0 to font_height-1)
    mov r10, 0 ; y_font
.font_y_loop:
    cmp r10b, [font_height]
    jge .putchar_done

    mov r11, 0 ; x_font
    mov dl, [rsi + r10] ; DL = font data byte for this row

.font_x_loop:
    cmp r11b, [font_width]
    jge .font_next_row

    ; Calculate screen coordinates for this pixel
    mov r12d, ebx ; start x_pixel
    add r12b, r11b ; + x_font
    mov r13d, ecx ; start y_pixel
    add r13b, r10b ; + y_font

    ; Create mask and test font bit
    mov al, 1
    mov r9b, 7      ; Bit position to check (7 down to 0)
    sub r9b, r11b
    push rcx        ; Save register potentially used by shift (CL)
    mov cl, r9b
    shl al, cl      ; AL = mask (1 << bit_pos)
    pop rcx
    test dl, al     ; Test font byte DL against mask AL
    jz .pixel_off   ; If zero, bit is off

; Pixel on:
    mov r8d, [font_fg_color]
    mov ecx, r12d ; Screen X
    mov edx, r13d ; Screen Y
    call putpixel
    jmp .pixel_next

.pixel_off:
    mov r8d, [font_bg_color]
    mov ecx, r12d ; Screen X
    mov edx, r13d ; Screen Y
    call putpixel

.pixel_next:
    inc r11 ; Next x_font
    jmp .font_x_loop

.font_next_row:
    inc r10 ; Next y_font
    jmp .font_y_loop

.putchar_done:
    ; Restore registers
    pop r15; pop r14; pop r13; pop r12; pop r11; pop r10; pop r9; pop r8
    pop rdi; pop rsi; pop rdx; pop rcx; pop rbx; pop ax
    ret

;--------------------------------------------------------------------------
scr64_print_char: ; Input: AL = Character
    push rax
    push rbx
    push rcx
    push rdx
    push r10 ; For division results

    cmp al, 10 ; Newline
    je .newline

    cmp al, 8 ; Backspace
    je .backspace

    ; --- Normal character ---
    mov bl, [cursor_x]
    mov cl, [cursor_y]
    call scr64_putchar_at ; AL is char, BL/CL are coords

    ; --- Advance cursor ---
    mov bx, [cursor_x]
    inc bx
    ; Max chars per line = HRes / font_width
    mov ax, [gop_h_res]
    xor dx, dx ; Clear DX for word division
    movzx cx, byte [font_width] ; Divisor
    div cx ; AX = Quotient (Max Chars), DX = Remainder
    cmp bx, ax ; Compare current X (BX) with max X (AX)
    jl .update_cursor_x

    ; --- Wrap cursor ---
.newline_logic:
    xor bx, bx ; X = 0
    mov cx, [cursor_y]
    inc cx
    ; Max text rows = VRes / font_height
    mov ax, [gop_v_res]
    xor dx, dx
    movzx r10w, byte [font_height] ; Divisor
    div r10w ; AX = Quotient (Max Rows)
    cmp cx, ax
    jl .set_cursor_y
    ; Scroll
    dec cx ; Stay on last line
    call scroll_screen
.set_cursor_y:
    mov [cursor_y], cx
.update_cursor_x:
    mov [cursor_x], bx
    jmp .print_char_done

.newline:
    jmp .newline_logic ; Jump to shared newline logic

.backspace:
    mov bx, [cursor_x]
    test bx, bx
    jz .print_char_done ; At start of line, do nothing
    dec bx
    mov [cursor_x], bx
    ; Erase character at new cursor position
    mov cx, [cursor_y]
    mov al, ' '
    call scr64_putchar_at
    jmp .print_char_done

.print_char_done:
    pop r10
    pop rdx
    pop rcx
    pop rbx
    pop rax
    ret

;--------------------------------------------------------------------------
scr64_print_string: ; Input: RSI = string pointer
    push rsi
    push rax
.loop:
    mov al, [rsi]
    test al, al
    jz .done
    call scr64_print_char
    inc rsi
    jmp .loop
.done:
    pop rax
    pop rsi
    ret

;--------------------------------------------------------------------------
scr64_print_hex: ; Input: RAX = value
    push rax ; Save original value
    push rbx
    push rcx
    push rdx
    push rsi

    mov rbx, rax ; Value to print
    mov rcx, 16 ; Loop counter (16 hex digits)
    lea rsi, [hex_digits]
.hex_loop:
    dec rcx
    ; Extract nibble using SHR
    mov rax, rbx
    push rcx    ; Save loop counter
    mov cl, 4
    mul cl      ; RAX = loop_counter * 4 (implicit)
    mov cl, 60  ; Start shift for highest nibble
    sub cl, al  ; CL = correct right shift amount (60, 56, .. 0)
    pop rcx     ; Restore loop counter
    shr rax, cl ; Shift desired nibble into low bits
    and al, 0x0F ; Isolate nibble
    mov dl, [rsi + rax] ; Get hex char
    ; Print character DL
    push rax ; Save registers around print call
    push rbx
    push rcx
    push rdx
    push rsi
    mov al, dl
    call scr64_print_char
    pop rsi
    pop rdx
    pop rcx
    pop rbx
    pop rax

    test rcx, rcx
    jnz .hex_loop
.hex_done:
    pop rsi
    pop rdx
    pop rcx
    pop rbx
    pop rax ; Restore original value
    ret

;--------------------------------------------------------------------------
scr64_print_dec: ; Input: RAX = value
    push rdi
    push rsi
    push rax ; Save original value for itoa64

    lea rdi, [dec_buffer] ; Buffer for itoa64
    ; RAX already has the value
    call itoa64
    ; itoa64 returns RDI pointing to start of string
    mov rsi, rdi ; RSI = String pointer for print
    call scr64_print_string

    pop rax ; Restore original value (or just discard if not needed)
    pop rsi
    pop rdi
    ret