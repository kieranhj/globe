; ******************************************************************
; *
; *     BeebAsm demo
; *
; *     Spinning star globe
; *
; *     Original 6502 by Rich Talbot-Watkins.
; *     Ported to Acorn Archimedes / ARM by kieran.
; *
; *     Change the speed of rotation with Z and X keys
; *     Press Esc to quit
; *
; *     Try assembling the code with debugrasters = 1 (line ~37)
; *     It shows how the frame is split up into various stages of
; *     processing.
; *
; *     The red part is where we start to plot the dots - on Archimedes
; *     we are using two screen buffers (double buffering) so there is
; *     no need to 'chase the raster'.
; *
; *     The magenta part is a small loop where we wait for 'vsync'.
; *     Rather than use SWI OS_Byte 19 (*FX19) we check the vsync
; *     counter which is updated in the VSync event handler. Events
; *     are easier and more compatible than IRQs handlers under RISCOS.
; *
; *     The blue part is the time when we are erasing dots.  Because we're
; *     double buffered, it is quicker to clear the entire screen on
; *     Archimedes using a semi-unrolled loop of store multiple registers.
; *
; ******************************************************************

; Define globals

.equ _UnrollPlotCode,       0
.equ _EvenYDistribution,    1
.equ _Mode12,               1

.if _UnrollPlotCode
.equ numdots,               7800
.else
.equ numdots,               2048        ; cf. 160 on 6502 @ 2MHz!
.endif
.equ radius,                120
.equ debugrasters,          1
.equ centrex,               160
.equ centrey,               128
.equ radsqr,                radius*radius

.equ Screen_Banks,          2
.if _Mode12
.equ Screen_Mode,           12
.equ Screen_Width,          640
.else
.equ Screen_Mode,           9
.equ Screen_Width,          320
.endif
.equ Screen_Height,         256
.equ Screen_PixelsPerByte,  2
.equ Screen_Stride,         Screen_Width/Screen_PixelsPerByte
.equ Screen_Bytes,          Screen_Stride*Screen_Height

.equ KeyBit_Escape,         0
.equ KeyBit_Z,              1
.equ KeyBit_X,              2

; Define swis and other values used by RISCOS.

.include "swis.h.asm"

; Set start address

.org 0x8000

; ******************************************************************
; * The entry point of the demo
; ******************************************************************

main:
    ; In RISCOS it is standard to define your own application stack.

    ldr sp, stack_p

    ; Make tables.

    bl MakeSinus
    .if _UnrollPlotCode
    bl MakeUnrolled
    .else
    bl MakeDotArray
    .endif

    ; Set MODE (VDU 22).

    swi OS_WriteI + 22
    swi OS_WriteI + Screen_Mode

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

    swi OS_RemoveCursors

    ; Display screen buffer 1.

    mov r1, #1
    mov r0, #OSByte_WriteDisplayBank
    swi OS_Byte

    ; Clear all screen buffers
    mov r1, #1
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

    ; Get base address of screen buffer we're writing to

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

    ; Claim the Event vector

    mov r0, #EventV
    adr r1, event_handler
    mov r2, #0
    swi OS_AddToVector

    ; Enable Vsync event

    mov r0, #OSByte_EventEnable
    mov r1, #Event_VSync
    SWI OS_Byte

    ; Enable key pressed event.

    mov r0, #OSByte_EventEnable
    mov r1, #Event_KeyPressed
    SWI OS_Byte

mainloop:

    ; Plot every dot on the screen

    .if debugrasters
    mov r0, #24             ; border
    mov r4, #0x000000ff     ; red
    bl pal_set_col
    .endif

    ; Get pointer to screen buffer

    ldr r12, screen_addr

    ; Set up draw loop

.if _UnrollPlotCode
    ldr r8, angle
    mov r8, r8, asl #8
    mov r8, r8, asr #24         ; INT(a)
    ldr r9, premult_sines_p
    add r9, r9, r8, lsl #2      ; shift base of tables
    adr lr, unrolled_code_return
    ldr pc, unrolled_code_p
unrolled_code_return:
    ldr r8, angle
.else

    mov r11, #numdots
    ldr r10, dot_array_p
    ldr r9, sinus_table_p
    ldr r8, angle

plotdotloop:

    ; Get next dot from the array

    ldmia r10!, {r5-r7}         ; read {dotx, doty, dotr}

    add r0, r5, r8              ; x = dotx + angle      {s15.16}

    ; Calculate its colour

    sub r4, r0, #64<<16         ; add quarter turn      {s15.16}
    mov r4, r4, lsr #20         ; [0,255]               {12.0}
    and r4, r4, #0xf            ;  -> [0,15]            {4.0}
    .if _Mode12
    orr r4, r4, r4, lsl #4
    .endif

    ; Calculate x position

    ; R0=sin(x)

    mov r0, r0, asl #8                      ; clamp brads [0,255]
    mov r0, r0, lsr #Sinus_TableShift       ; remove insignificant bits
    ldr r0, [r9, r0, lsl #2]                ; lookup in sinus table

    ; Calculate sin(x) * radius
    
    mov r0, r0, asr #8          ; sin(x)                {s7.8}
    mov r7, r7, asr #8          ; dotr                  {s7.8}
    mul r0, r7, r0              ; r0 = sin(x) * dotr    {s15.16}

    ; Convert into screen coordinates

    add r5, r0, #centrex<<16    ; add centre of globe
    add r6, r6, #centrey<<16

    mov r5, r5, asr #16         ; r5=INT(dotx)
    mov r6, r6, asr #16         ; r6=INT(doty)

    ; Calculate ptr to screen byte

    .if _Mode12
    add r3, r12, r6, lsl #8     ; scr_ptr = scr_base + y * 256
    add r3, r3, r6, lsl #6      ;                    + y * 64
    strb r4, [r3, r5]
    .else
    add r3, r12, r6, lsl #7     ; scr_ptr = scr_base + y * 128
    add r3, r3, r6, lsl #5      ;                    + y * 32
    add r3, r3, r5, lsr #1      ;                    + x / 2

    ; Plot a pixel in MODE 9

    ; Interesting performance note!

    tst r5, #1                  ; left or right pixel in byte?

    ; This load instruction is faster if aligned PC & 0xc == 4...

    ldrb r1, [r3]               ; load byte from screen

    andeq r1, r1, #0xF0         ; mask out left hand pixel
    orreq r1, r1, r4            ; mask in colour as left hand pixel
    andne r1, r1, #0x0F         ; mask out right hand pixel
    orrne r1, r1, r4, lsl #4    ; mask in colour as right hand pixel
    strb r1, [r3]               ; store byte to screen
    .endif

    ; TODO: If the dot is in front, we double its size?

    subs r11, r11, #1
    bne plotdotloop
.endif

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
    mov r0, #24             ; border
    mov r4, #0x00ff00ff     ; magenta
    bl pal_set_col
    .endif

    ldr r0, vsync
.1:
    ldr r1, vsync
    cmp r0, r1
    beq .1

    ; Set next write buffer for screen.

    ldr r1, scr_bank
    add r1, r1, #1
    cmp r1, #Screen_Banks
    movgt r1, #1
    str r1, scr_bank

    mov r0, #OSByte_WriteVDUBank
    swi OS_Byte

    bl get_screen_addr

    .if debugrasters
    mov r0, #24             ; border
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

    .if debugrasters
    mov r0, #24             ; border
    mov r4, #0x00000000     ; black
    bl pal_set_col
    .endif

    ; Get mask of key bits that are both pressed and changed

    ldr r0, keyboard_pressed_mask
    ldr r2, keyboard_prev_mask
    mvn r2, r2                ; ~old
    and r2, r0, r2            ; new & ~old        ; diff bits
    str r0, keyboard_prev_mask
    and r4, r2, r0            ; diff bits & key down bits

    ; Update speed according to pressed keys

    ldr r0, speed
    tst r4, #1<<KeyBit_Z
    subne r0, r0, #1<<14     ; in 0.25 increments
    tst r4, #1<<KeyBit_X
    addne r0, r0, #1<<14
    str r0, speed

    ; Exit if Escape is pressed

    tst r4, #1<<KeyBit_Escape
    beq mainloop

exit:
    ; Disable key press event

    mov r0, #OSByte_EventDisable
    mov r1, #Event_KeyPressed
    swi OS_Byte

    ; Disable vsync event

    mov r0, #OSByte_EventDisable
    mov r1, #Event_VSync
    swi OS_Byte

    ; Release our event handler
    mov r0, #EventV
    adr r1, event_handler
    mov r2, #0
    swi OS_Release

    ; Display whichever bank we've just written to

    mov r0, #OSByte_WriteDisplayBank
    ldr r1, scr_bank
    swi OS_Byte

    ; And write to it

    mov r0, #OSByte_WriteVDUBank
    ldr r1, scr_bank
    swi OS_Byte

    ; Flush keyboard buffer (removes keypresses)

    mov r0, #15
    mov r1, #1
    swi OS_Byte

    swi OS_Exit     ; return to RISCOS.

; R0=event number
event_handler:
    cmp r0, #Event_VSync
    bne keypress_handler

    str r0, [sp, #-4]!          ; Must preserve all registers

    ; Update the vsync counter

    ldr r0, vsync
    add r0, r0, #1
    str r0, vsync

    ldr r0, [sp], #4
    mov pc, lr

keypress_handler:
    cmp r0, #Event_KeyPressed
    movnes pc, lr

    ; R1=0 key up or 1 key down
    ; R2=internal key number (RMKey_*)

    str r0, [sp, #-4]!          ; Must preserve all registers

    ldr r0, keyboard_pressed_mask
    cmp r1, #0
    beq .1

    ; Key down
    cmp r2, #RMKey_Esc
    orreq r0, r0, #1<<KeyBit_Escape
    cmp r2, #RMKey_Z
    orreq r0, r0, #1<<KeyBit_Z
    cmp r2, #RMKey_X
    orreq r0, r0, #1<<KeyBit_X
    b .2

.1:
    ; Key up
    cmp r2, #RMKey_Esc
    orreq r0, r0, #1<<KeyBit_Escape
    cmp r2, #RMKey_Z
    biceq r0, r0, #1<<KeyBit_Z
    cmp r2, #RMKey_X
    biceq r0, r0, #1<<KeyBit_X

.2:
    str r0, keyboard_pressed_mask
    ldr r0, [sp], #4
    mov pc, lr

; ******************************************************************
; * Define some local variables.
; ******************************************************************

vsync:
    .long 0

scr_bank:
    .long 0                     ; Screen bank number being written so

screen_addr:
    .long 0                     ; Address of the current VIDC screen bank being written to

angle:
    .long 0                     ; {s15.16}

speed:
    .long 1<<16                 ; {s15.16}

keyboard_pressed_mask:
    .long 0

keyboard_prev_mask:
    .long 0

stack_p:
    .long stack_base_no_adr

sinus_table_p:
    .long sinus_table_no_adr    ; address patched by the linker from bss segment

dot_array_p:
    .long dot_array_no_adr      ; address patched by the linker from bss segment

.if _UnrollPlotCode
premult_sines_p:
    .long premult_sine_tables_no_adr

unrolled_code_p:
    .long unrolled_code_no_adr
.endif

; ******************************************************************

get_screen_addr:
    adr r0, screen_addr_input
    adr r1, screen_addr
    swi OS_ReadVduVariables
    mov pc, lr

screen_addr_input:
    .long VD_ScreenStart, -1

error_noscreenmem:
    .long 0
    .byte "Cannot allocate screen memory!"
    .align 4
    .long 0

; ******************************************************************

.if !_UnrollPlotCode
MakeDotArray:
    str lr, [sp, #-4]!

    mov r11, #numdots
    ldr r10, dot_array_p
    ldr r9, sinus_table_p

    mov r8, #-1<<24         ; iterate x over [-1,1]         {s1.24}
    add r8, r8, #1<<24/numdots   ;                          {s1.16}
.1:
    ; Calculate doty

.if _EvenYDistribution
    mov r0, #radius         ;                               {8.0}
    mul r6, r0, r8          ; doty=x*radius                 {s1.24}
    mov r6, r6, asr #8
.else
    mov r0, r8, asl #8                  ; remove integer part of x      {0.32}
    mov r0, r0, lsr #Sinus_TableShift   ; remove insignificant bits     {14.0}
    ldr r0, [r9, r0, lsl #2]            ; Look up sin(x)    {s1.16}

    mov r0, r0, asl #5      ; sin(x)*0.125                  {s1.24}
    sub r6, r8, r0          ; x-sinx(x)*0.125               {s1.24}
    mov r6, r6, asr #8      ;                               {s1.16}
    mov r0, #radius         ;                               {s15.0}
    mul r6, r0, r6          ; doty=(x-sin(x)*1.25)*radius   {s15.16}
.endif

    ; Calulate dotr

    mov r0, #radsqr<<16     ; radius*radius                 {16.16}
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

    add r8, r8, #2<<24/(numdots) ;                          {s1.16}
    subs r11, r11, #1
    bne .1
    ldr pc, [sp], #4
.endif

; ******************************************************************
; * Taken from https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
; * sqrt_i32_to_fx16_16 computes the square root of a 32bit integer and returns
; * a fixed point value with 16bit fractional part. It requires that v is positive.
; * The computation use only 32 bit registers and simple operations.
; ******************************************************************

; R0=int32_t v
; Return R3=fx16_16_t sqrt(v) [16.16]
; Trashes: R1, R2, R4
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
    tst  r3, r3, LSR #1                     ; top bit into Carry
    movs r4, r4, RRX                        ; 33 bit rotate right
    adc  r3, r3, r3                         ; carry into lsb of spare bits
    eor  r4, r4, r0, LSL #12                ; (involved!)
    eor  r0, r4, r4, LSR #20                ; (similarly involved!)
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
    swi OS_Word             ; NB. this is super slow!

    mov pc,lr

pal_osword_block:
    .skip 8
    ; logical colour
    ; physical colour (16)
    ; red
    ; green
    ; blue
    ; (pad)

; ******************************************************************
; * DATA segment (initialised)
; ******************************************************************

pal_gradient:
    ;     0x00BbGgRr
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
    ; Unroll notes:
    ;  dotx doesn't change for a dot so can be made immediate.
    ;  instead of adding angle, shift the base of the table being looked up.
    ;  doty doesn't change, so we already know the scanline offset.
    ;  don't need to calculate this, can prebake it.

    ; Remove all the fixed point nonsense.
    ; What's the simplest possible set of instructions to plot?
; ******************************************************************

.if _UnrollPlotCode
MakeUnrolled:
    str lr, [sp, #-4]!

    ldr r9, sinus_table_p
    ldr r8, premult_sines_p

    ; Loop per Y
    mov r6, #radius<<16
.1:
    mov r0, #radsqr<<16     ; radius*radius                 {16.16}
    mov r4, r6, asr #8      ;                               {8.8}
    mov r3, r6, asr #8      ;                               {8.8}
    mul r4, r3, r4          ; y*y                           {16.16}
    sub r0, r0, r4          ; radius*radius-y*y             {16.16}

    mov r0, r0, asr #16     ;                               {16.0}
    bl sqrt_i32_to_fx16_16  ; returns SQRT(R0) in R3        {16.16}
    ; Trashes R1, R2, R4

    mov r3, r3, asr #8      ; radius                        {8.8}

    ; Copy a sine table and multiply it for this Y.
    mov r10, #0                     ; x=0
.2:
    and r2, r10, #0x3fc0            ; wrap after 256 - could use shift
    ldr r0, [r9, r2, lsl #2]        ; sin(x)                {s1.16}

    mov r0, r0, asr #8              ;                       {s1.8}
    mul r0, r3, r0                  ; radius*sin(x)         {s8.16}

    mov r0, r0, asr #16             ; {s8.0}
    add r0, r0, #centrex            ; {9.0}
    mov r0, r0, lsl #16             ; {9.0} << 16

    sub r4, r10, #64<<6
    mov r4, r4, lsr #10             ; [0,255]
    and r4, r4, #0xf                ;  -> [0,15]            {4.0}
    orr r4, r4, r4, lsl #4

    orr r0, r0, r4                  ; xpos | col

    str r0, [r8], #4

    add r10, r10, #1<<6             ; x+=1
    cmp r10, #512<<6
    blt .2

    subs r6, r6, #1<<16
    bpl .1

    ; Make the unrolled code.

    ldr r12, unrolled_code_p
    mov r11, #numdots
    ldr r10, dot_array_p
    ldr r9, sinus_table_p

    mov r8, #-1<<24         ; iterate x over [-1,1]         {s1.24}
    add r8, r8, #1<<24/numdots   ;                          {s1.16}
.3:
    ; Calculate doty

.if _EvenYDistribution
    mov r0, #radius         ;                               {8.0}
    mul r6, r0, r8          ; doty=x*radius                 {s1.24}
    mov r6, r6, asr #8
.else
    mov r0, r8, asl #8                  ; remove integer part of x      {0.32}
    mov r0, r0, lsr #Sinus_TableShift   ; remove insignificant bits     {14.0}
    ldr r0, [r9, r0, lsl #2]            ; Look up sin(x)    {s1.16}

    mov r0, r0, asl #5      ; sin(x)*0.125                  {s1.24}
    sub r6, r8, r0          ; x-sinx(x)*0.125               {s1.24}
    mov r6, r6, asr #8      ;                               {s1.16}
    mov r0, #radius         ;                               {s15.0}
    mul r6, r0, r6          ; doty=(x-sin(x)*1.25)*radius   {s15.16}
.endif

    ; Calulate dotr

    mov r0, #radsqr<<16     ; radius*radius                 {16.16}
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

    mov r5, r5, asr #16         ; INT(dota)         {s8.0}
    movs r6, r6, asr #16        ; INT(doty)         {s8.0}

    addle r7, r6, #radius       ; table_no=radius+INT(doty)
    rsbgt r7, r6, #radius       ; table_no=radius-INT(doty)

    add r6, r6, #centrey        ; doty+=centrey

    adr r0, unrolled_snippet
    ldmia r0, {r0-r4}           ; read 5 words

    ; add r1, r9, #N * 2048
    tst r7, #1
    moveq r7, r7, lsr #1        ; even N>>1
    movne r7, r7, lsl #1        ; odd N<<1
    orrne r0, r0, #0x100        ; Odd numbers 0x1b N<<1
    bic r0, r0, #0xff           ; Even numbers 0x1a N>>1
    orr r0, r0, r7

    ; ldr r5, [r1, #dota * 4]
    mov r5, r5, lsl #2          ; dota * 4
    orr r1, r1, r5

    ; add r3, r12, #doty * 256
    bic r2, r2, #0xff
    orr r2, r2, r6              ; doty * 256

    ; add r3, r3, #doty * 64
    bic r3, r3, #0xff
    orr r3, r3, r6              ; doty * 64

    stmia r12!, {r0-r4}          ; write 5 words

    add r8, r8, #2<<24/(numdots) ;                          {s1.16}
    subs r11, r11, #1
    bne .3

    ; Copy mov pc, lr
    adr r0, unrolled_snippet
    ldr r0, [r0, #5*4]
    str r0, [r12], #4

    ldr pc, [sp], #4

unrolled_snippet:
    add r1, r9, #2 * 2048   ; sin_table_for_y + angle (2*256 entries)
    ldr r5, [r1, #0]        ; xpos<<16 | colour
    add r3, r12, #255 * 256
    add r3, r3, #255 * 64
    strb r5, [r3, r5, lsr #16]
    mov pc, lr
.endif


; ******************************************************************
; * Space reserved for tables but not initialised with anything
; * Therefore these are not saved in the executable
; *
; * BSS segment (uninitialised data)
; * Note that labels cannot be referenced across segments using the ADR directive
; * Hence my arbitrary convention of suffixing such labels with _no_adr as a reminder!
; ******************************************************************

.bss

; ******************************************************************
; * Sine table with 16384 entries in {s1.16} fixed point format.
; ******************************************************************

sinus_table_no_adr:
    .skip Sinus_TableSize*4

; ******************************************************************
; * Dot array
; * Stored interleaved so these can be read in one instruction on ARM
; * contains the phase of this dot in brads [0,256) {8.16}
; * contains the y position of the dot {s15.16}
; * contains the radius of the ball at this y position (s15.16)

dot_array_no_adr:
    .skip numdots*4*3           ; {doyx, doty, dotr}

; ******************************************************************

stack_no_adr:
    .skip 1024
stack_base_no_adr:

.if _UnrollPlotCode
; ******************************************************************
premult_sine_tables_no_adr:
    .skip 256*2*4*(radius+1)

unrolled_code_no_adr:

; ******************************************************************
.endif
