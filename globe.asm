; ******************************************************************
; *
; *		BeebAsm demo
; *
; *		Spinning star globe
; *
; *     Original 6502 by Rich Talbot-Watkins.
; *		Ported to Acorn Archimedes / ARM by kieran.
; *
; *     TODO: Description of the raster timings etc.
; *
; ******************************************************************

; Define globals

.equ numdots,               2000
.equ radius,                100
.equ debugrasters, 	        1
.equ centrex,               160
.equ centrey,               128

.equ Screen_Banks,          2
.equ Screen_Mode,           9
.equ Screen_Width,          320 
.equ Screen_Height,         256
.equ Screen_PixelsPerByte,  2
.equ Screen_Stride,         Screen_Width/Screen_PixelsPerByte
.equ Screen_Bytes,          Screen_Stride*Screen_Height

; Define swis and other values used by RISCOS.

.include "swis.h.asm"

; Set start address

.org 0x8000

; ******************************************************************
; *	The entry point of the demo
; ******************************************************************

start:
    adrl sp, stack_base
	b main

; In RISCOS it is standard to define your own application stack.

.skip 1024
stack_base:

; Define some local variables.

vsync:
    .long 0

scr_bank:
    .long 0

angle:
    .long 0

speed:
    .long 1<<16

sinus_table_p:
    .long sinus_table_no_adr

dot_array_p:
    .long dot_array_no_adr

; ******************************************************************
; *	Actual code start
; ******************************************************************

main:
    ; Make tables.

    bl MakeSinus
    bl MakeDotArray

    ; Set MODE (VDU 22).

	mov r0, #22
	swi OS_WriteC
	mov r0, #Screen_Mode
	swi OS_WriteC

	; Set screen size for number of buffers.

	mov r0, #DynArea_Screen
	swi OS_ReadDynamicArea
	mov r0, #DynArea_Screen
	mov r2, #Screen_Bytes * Screen_Banks
	subs r1, r2, r1
	swi OS_ChangeDynamicArea
	mov r0, #DynArea_Screen
	swi OS_ReadDynamicArea
	cmp r1, #Screen_Bytes * Screen_Banks
	adrcc r0, error_noscreenmem ; failed to allocate screen memory.
	swicc OS_GenerateError

    ; Disable cursor

	mov r0, #23
	swi OS_WriteC
	mov r0, #1
	swi OS_WriteC
	mov r0, #0
	swi OS_WriteC
	swi OS_WriteC
	swi OS_WriteC
	swi OS_WriteC
	swi OS_WriteC
	swi OS_WriteC
	swi OS_WriteC
	swi OS_WriteC

	; Clear all screen buffers

	mov r0, #OSByte_WriteDisplayBank
	mov r1, #1
    swi OS_Byte
.1:
	str r1, scr_bank

	; CLS buffer N
	mov r0, #OSByte_WriteVDUBank
	swi OS_Byte
	mov r0, #12
	SWI OS_WriteC

	ldr r1, scr_bank
	add r1, r1, #1
	cmp r1, #Screen_Banks
	ble .1

    ; Writing to buffer N

    bl get_screen_addr

    ; Set 16 colour palette

    adr r2, pal_gradient
    mov r3, #0
.2:
    mov r0, r3
    ldr r4, [r2], #4
    bl pal_set_col
    add r3, r3, #1
    cmp r3, #16
    bne .2

	; First we wait for 'vsync' so we are synchronised

    ; TODO: Demonstrate vsync Event.

    mov r0, #19
    swi OS_Byte

mainloop:

	; Plot every dot on the screen

    .if debugrasters
    mov r0, #24
    mov r4, #0x000000ff     ; red
    bl pal_set_col
    .endif

    ; Get pointer to screen buffer

    ldr r12, screen_addr

    ; Set up draw loop

    mov r11, #numdots
    ldr r10, dot_array_p
    ldr r9, sinus_table_p
    ldr r8, angle

plotdotloop:

    ; Get a dot

    ldmia r10!, {r5-r7}         ; read dotx, doty, dotr

    add r0, r5, r8              ; x = dotx + angle

    ; Calculate colour.

    sub r4, r0, #64<<16         ; add quarter turn
    mov r4, r4, lsr #20         ; [0,255] {s15.16}
    and r4, r4, #0xf            ;  -> [0,15] {4.0}

    ; R0=sin(x)

    mov r0, r0, asl #8                      ; clamp brads [0,255]
    mov r0, r0, lsr #Sinus_TableShift       ; remove insignificant bits
    ldr r0, [r9, r0, lsl #2]                ; lookup in sinus table

	; Calculate sin(x) * radius
    
    mov r0, r0, asr #8          ; sin(x)                {s7.8}
    mov r7, r7, asr #8          ; dotr                  {s7.8}
    mul r0, r7, r0              ; r0 = sin(x) * dotr    {s15.16}

    add r5, r0, #centrex<<16    ; add centre of globe
    add r6, r6, #centrey<<16

    mov r5, r5, asr #16         ; r5=INT(dotx)
    mov r6, r6, asr #16         ; r6=INT(doty)

    ; Calculate ptr to screen byte

    add r3, r12, r6, lsl #7     ; scr_ptr = scr_base + y * 128
    add r3, r3, r6, lsl #5      ;                    + y * 32
    add r3, r3, r5, lsr #1      ;                    + x / 2

    ; Plot a pixel in MODE 9

    ldrb r1, [r3]               ; load byte from screen
    tst r5, #1                  ; left or right pixel in byte?
	andeq r1, r1, #0xF0		    ; mask out left hand pixel
	orreq r1, r1, r4			; mask in colour as left hand pixel
	andne r1, r1, #0x0F		    ; mask out right hand pixel
	orrne r1, r1, r4, lsl #4	; mask in colour as right hand pixel
    strb r1, [r3]               ; store byte to screen

	; TODO: if the dot is in front, we double its size?

    subs r11, r11, #1
    bne plotdotloop

    ; Add to rotation

    ldr r7, speed
    add r8, r8, r7
    str r8, angle

    ; Set next display buffer from vsync

	ldr r1, scr_bank
	mov r0, #OSByte_WriteDisplayBank
    swi OS_Byte

    ; Wait for VSync here

    .if debugrasters
    mov r0, #24
    mov r4, #0x00ff00ff     ; magenta
    bl pal_set_col
    .endif

    mov r0, #19
    swi OS_Byte

    .if debugrasters
    mov r0, #24
    mov r4, #0x00000000     ; black
    bl pal_set_col
    .endif

    ; Set next write buffer for screen.

    ldr r1, scr_bank
	add r1, r1, #1
	cmp r1, #Screen_Banks
	movgt r1, #1
	str r1, scr_bank

	mov r0, #OSByte_WriteVDUBank
    swi OS_Byte

    bl get_screen_addr

	; exit if Escape is pressed
	mov r0, #OSByte_ReadKey
	mov r1, #IKey_Escape
	mov r2, #0xff
	swi OS_Byte
	
	cmp r1, #0xff
	cmpeq r2, #0xff
	beq exit
	
    .if debugrasters
    mov r0, #24
    mov r4, #0x00ff0000     ; blue
    bl pal_set_col
    .endif

	; Now delete all the old dots (clear screen)

    ldr r12, screen_addr

    mov r0, #0
    mov r1, r0
    mov r2, r0
    mov r3, r0
    mov r4, r0
    mov r5, r0
    mov r6, r0
    mov r7, r0
    mov r8, r0
    mov r9, r0

    mov r10, #Screen_Height
clsloop:
    .rept Screen_Stride/40
    stmia r12!, {r0-r9}     ; 40 bytes
    .endr
    subs r10, r10, #1
    bne clsloop

    ; TODO: keypresses
	b mainloop

exit:
	; Display whichever bank we've just written to
	mov r0, #OSByte_WriteDisplayBank
	ldr r1, scr_bank
	swi OS_Byte
	; and write to it
	mov r0, #OSByte_WriteVDUBank
	ldr r1, scr_bank
	swi OS_Byte

	swi OS_Exit     ; return to RISCOS.

; ******************************************************************

get_screen_addr:
	str lr, [sp, #-4]!
	adr r0, screen_addr_input
	adr r1, screen_addr
	swi OS_ReadVduVariables
	ldr pc, [sp], #4
	
screen_addr_input:
	.long VD_ScreenStart, -1

screen_addr:
	.long 0					; ptr to the current VIDC screen bank being written to.

error_noscreenmem:
	.long 0
	.byte "Cannot allocate screen memory!"
	.align 4
	.long 0

vdu_set_mode_disable_cursor:
    .byte 22, Screen_Mode, 23,1,0,0,0,0,0,0,0,0,17,7
.p2align 2

; ******************************************************************

MakeDotArray:
	str lr, [sp, #-4]!

    mov r11, #numdots
    ldr r10, dot_array_p
    ldr r9, sinus_table_p

    mov r8, #-1<<16         ; iterate x over [-1,1]         {s1.16}
.1:
    ; Calculate doty

    mov r0, r8, asl #16                     ; remove integer part
    mov r0, r0, lsr #Sinus_TableShift       ; remove insignificant bits
    ldr r0, [r9, r0, lsl #2]                ; R0=sin(x)     {s1.16}

    mov r0, r0, asr #3      ; sin(x)*0.125                  {s1.16}
    sub r6, r8, r0          ; x-sinx(x)*0.125               {s1.16}
    mov r0, #radius
    mul r6, r0, r6          ; doty=(x-sin(x)*1.25)*radius   {s15.16}

    ; Calulate dotr

    mov r0, #10000<<16      ; radius*radius                 {16.16}
    mov r4, r6, asr #8      ;                               {8.8}
    mov r3, r6, asr #8      ;                               {8.8}
    mul r4, r3, r4          ; y*y                           {16.16}
    sub r0, r0, r4          ; radius*radius-y*y             {16.16}

    mov r0, r0, asr #16     ;                               {16.0}
    bl sqrt_i32_to_fx16_16  ; returns SQRT(R0) in R3        {16.16}
    ; Trashes R1, R2, R4

    mov r7, r3              ; SQRT(radius*radius-y*y)       {16.16}

    ; Calculate dotx

    bl rnd                  ; R0=rand(MAX_UINT)             {32.0}
    mov r5, r0, lsr #8      ; shift down to brad [0,256)    {8.16}

    ; Store {dotx, doty, dotr}

    stmia r10!, {r5-r7}

    add r8, r8, #2<<16/(numdots)
    subs r11, r11, #1
    bne .1
	ldr pc, [sp], #4

; Taken from https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
; sqrt_i32_to_fx16_16 computes the square root of a 32bit integer and returns
; a fixed point value with 16bit fractional part. It requires that v is positive.
; The computation use only 32 bit registers and simple operations.

; R0=int32_t v
; Return R3=fx16_16_t sqrt(v) [16.16]
; Trashes: r1, r2, r4
sqrt_i32_to_fx16_16:
;   uint32_t t, q, b, r;
    cmp r0, #0
    moveq pc, lr        ;    if (v == 0) return 0;

    mov r1, r0          ;    r = v;
    mov r2, #0x40000000 ;    b = 0x40000000;
    mov r3, #0          ;    q = 0;
.1:
    cmp r2, #0          ;    while( b > 0 )
    beq .2

    add r4, r3, r2      ;        t = q + b;

    cmp r1, r4          ;        if( r >= t )
    blt .3

    sub r1, r1, r4      ;           r -= t;
    add r3, r4, r2      ;           q = t + b;
    .3:

    mov r1, r1, asl #1  ;   r <<= 1;
    mov r2, r2, lsr #1  ;   b >>= 1;
    b .1

.2:
    cmp r1, r3
    addgt r3, r3, #1    ; if( r > q ) ++q;
    mov pc, lr          ; return q;

; ******************************************************************
; * Makes sine values [0-0x10000]
; * Converted to ARM from https://github.com/askeksa/Rose/blob/master/engine/Sinus.S
; ******************************************************************

.equ Sinus_TableBits,     14                  ; 16384
.equ Sinus_TableSize,     1<<Sinus_TableBits
.equ Sinus_TableShift,    32-Sinus_TableBits

MakeSinus:
    ldr r8, sinus_table_p
    mov r10, #Sinus_TableSize/2*4       ; offset halfway through the table.
    sub r11, r10, #4            ; #Sinus_TableSize/2*4-4

    mov r0, #0
    str r0, [r8], #4
    add r9, r8, r11             ; #Sinus_TableSize/2*4-4
    str r0, [r9]

    mov r7, #1
.1:
    mov r1, r7
    mul r1, r7, r1              ; r7 ^2
    mov r1, r1, asr #8

    mov r0, #2373
    mul r0, r1, r0
    mov r0, r0, asr #16
    rsb r0, r0, #0
    add r0, r0, #21073
    mul r0, r1, r0
    mov r0, r0, asr #16
    rsb r0, r0, #0
    add r0, r0, #51469
    mul r0, r7, r0
    mov r0, r0, asr #13

    mov r0, r0, asl #2          ; NB. Rose originally [0x0, 0x4000]

    str r0, [r8], #4
    str r0, [r9, #-4]!
    rsb r0, r0, #0
    str r0, [r9, r10]           ; #Sinus_TableSize/2*4
    str r0, [r8, r11]           ; #Sinus_TableSize/2*4-4

    add r7, r7, #1
    cmp r7, #Sinus_TableSize/4
    blt .1

    rsb r0, r0, #0
    str r0, [r9, #-4]!
    rsb r0, r0, #0
    str r0, [r9, r10]
    mov pc, lr

; ******************************************************************
; * Additional library code
; ******************************************************************

rnd_seed:
    .long 0x87654321

rnd_bit:
    .long 0x11111111

; R0=rand(MAX_UINT)
rnd:
    ldr  r0, rnd_seed
    ldr  r3, rnd_bit
    ; R4=temp
    TST  r3, r3, LSR #1                     ; top bit into Carry
    MOVS r4, r4, RRX                        ; 33 bit rotate right
    ADC  r3, r3, r3                         ; carry into lsb of spare bits
    EOR  r4, r4, r0, LSL #12                ; (involved!)
    EOR  r0, r4, r4, LSR #20                ; (similarly involved!)
    str  r0, rnd_seed
    str  r3, rnd_bit
    mov  pc, lr

; R0 = logical colour (24=border)
; R4 = RGB colour as 0x00BBGGRR
; Uses R0,R1 
pal_set_col:
    adrl r1, pal_osword_block
    strb r0, [r1, #0]       ; logical colour

    cmp r0, #24             ; border
    movne r0, #16           ; physical colour
    strb r0, [r1, #1]       ; mode

    strb r4, [r1, #2]       ; red
    mov r4, r4, lsr #8
    strb r4, [r1, #3]       ; green
    mov r4, r4, lsr #8
    strb r4, [r1, #4]       ; blue

    mov r0, #12
    swi OS_Word

    mov pc,lr

pal_osword_block:
    .skip 8
    ; logical colour
    ; physical colour (16)
    ; red
    ; green
    ; blue
    ; (pad)

pal_gradient:
	.long 0x00000000
	.long 0x00000099
	.long 0x000000aa
	.long 0x000000bb
	.long 0x000000cc
	.long 0x000000dd
	.long 0x000000ee
	.long 0x000000ff
	.long 0x00888800
	.long 0x00999900
	.long 0x00AAAA00
	.long 0x00BBBB00
	.long 0x00CCCC00
	.long 0x00DDDD00
	.long 0x00EEEE00
	.long 0x00FFFF00

; ******************************************************************
; * BSS segment (uninitialised data)
; * This is not stored in the executable.
; ******************************************************************

.bss

; ******************************************************************
; * Sine table with 16384 entries in {s1.16} fixed point format.
; ******************************************************************

sinus_table_no_adr:
    .skip Sinus_TableSize*4

; ******************************************************************
; * Dot array.
; * Stored interleaved so these can be read in one instruction on ARM.
; * contains the phase of this dot in brads [0,256) {8.16}
; * contains the y position of the dot {s15.16}
; * contains the radius of the ball at this y position (s15.16)

dot_array_no_adr:
    .skip numdots*4*3           ; {doyx, doty, dotr}

; ******************************************************************
