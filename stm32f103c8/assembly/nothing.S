.syntax unified
.cpu cortex-m3
.thumb

.word 0x20000400
.word 0x080000ed
.space 0xe4

nop		@ Do Nothing
ldr r0, =123456789
mov r1, r0
ldr r0, =14
mov r1, r0
b .		@ Endless loop
