@ This code should be loaded at 0x08000000

.syntax unified     @ Use modern syntax for thumb instructions
.cpu cortex-m0
.thumb

RCC_AHBENR = 0x40021014
GPIOAEN_Pos = 17 

GPIOA_MODER = 0x48000000
PIN_5_MODE_Pos = 10

GPIOA_BSRR = 0x48000018
PIN_5_Set_Pos = 5
PIN_5_Reset_Pos = (16 + 5) 

.section .VectorTable, "a" @ allocable section
.word _StackEnd     @ top of stack (from linker script); msp is loaded with this value on reset
.word ResetHandler  @ execution begins at this address
.space 0xb4         @ total size of vector table = 47*4 = 0xbc

.text

/*----------------------------------*/
@ Execution begins here
.type ResetHandler, %function
ResetHandler:
    ldr r0, =-1
    movs r1, #7
    subs r2, r0, r1
    cmp r0, r1
    bhi xyz
    ldr r0, =0
    bl FillRAM
    nop
.align 2
xyz:
    bl Setup
    bl MainLoop

/*----------------------------------*/
@ just playing around here; write 1024 0xcc bytes
@ to RAM starting at 0x20000000
.type FillRAM, %function
FillRAM:
    ldr r0, =256    @ counter
    ldr r1, =0x20000000
    ldr r2, =0xcccccccc
fill_loop:
    str r2, [r1]
    adds r1, #4
    adds r0, #-1
    bne fill_loop
    bx lr

/*----------------------------------*/
.type Setup, %function
Setup:
    /* Enable GPIOA */
    ldr r0, =RCC_AHBENR
    ldr r1, [r0]
    movs r2, #1
    lsls r2, #GPIOAEN_Pos 
    orrs r1, r2
    str r1, [r0]

    /* PA5: general-purpose output (push-pull is default) */
    ldr r0, =GPIOA_MODER
    ldr r1, [r0]
    movs r2, #1
    lsls r2, #PIN_5_MODE_Pos
    orrs r1, r2
    str r1, [r0]
    bx lr

/*----------------------------------*/
.type MainLoop, %function
MainLoop:
    /* Turn on LED */
    ldr r0, =GPIOA_BSRR
    movs r1, #1
    lsls r1, #PIN_5_Set_Pos
    str r1, [r0]

    /* Call delay(800000) */
    ldr r0, =800000
    bl Delay

    /* Turn off LED */
    ldr r0, =GPIOA_BSRR
    movs r1, #1
    lsls r1, #PIN_5_Reset_Pos
    str r1, [r0]

    /* Call delay(800000) */
    ldr r0, =800000
    bl Delay

    b MainLoop

/*----------------------------------*/
.type Delay %function
@ r0: delay iterations
Delay:
    adds r0, #-1
    bne Delay
    bx lr
