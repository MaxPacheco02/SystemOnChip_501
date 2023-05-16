.section	.data

A: 	.word		5
B:	.word		2
C:	.word		8
D:	.word		4
E:	.word		1
R:	.word		0 //location with largest number

	.section	.text
	.syntax 	unified
	.global 	main

main:
	ldr r0, =A
	ldrb r1, [r0]
	ldr r0, =B
	ldrb r2, [r0]
	ldr r0, =C
	ldrb r3, [r0]
	ldr r0, =D
	ldrb r4, [r0]
	ldr r0, =E
	ldrb r5, [r0]
	ldr r0, =R
	ldrb r6, [r0]

	cmp r1, r2
	ble may2
	ldr r0, =A
	ldr r7, [r0]
	b cmp2
	ldr r0, =B
may2:ldr r7, [r0]

cmp2:cmp r3, r7
	ble cmp3
	ldr r0, =C
	ldr r7, [r0]

cmp3:cmp r4, r7
	ble cmp4
	ldr r0, =D
	ldr r7, [r0]

cmp4:cmp r5, r7
	ble end
	ldr r0, =E
	ldr r7, [r0]

end:ldr r0, =R
	strb r7, [r0]

LoopForever:
	b LoopForever
