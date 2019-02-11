#include <avr/io.h>



.text

;----------------------------------------------------------------------------------
; extern unsigned long shr_u32_10(unsigned long v);
.global shr_u32_10
;----------------------------------------------------------------------------------
; Shift right unsigned long (32-bit) by 10 bits.
; 
; Input: R22 (LSB) - R25(MSB)
; Return: R22 (LSB) - R25(MSB)
;
; Clock cycles: 14
;----------------------------------------------------------------------------------





shr_u32_10:

	; >>=1
	lsr r25
	ror r24
	ror r23
	; >>=1
	lsr r25
	ror r24
	ror r23

	mov r22,r23
	mov r23,r24
	mov r24,r25
	clr r25

	ret
	
	
	
;----------------------------------------------------------------------------------
; extern unsigned long u32_mul1000_shr_10(unsigned long v);
.global u32_mul1000_shr_10
;----------------------------------------------------------------------------------
; Takes an unsigned long (32-bit) v and multiply it by 1000 and shift it right by 10 bits.
; The output is the 32-bit number v*1000/1024.
; Internally uses a 48-bit representation to ensure no loss of resolution.
; Uses call-used registers as much as possible (R18-R27, R30-R31)
; 
; Two possible implementations: start by multiplying high bytes, or start by low bytes.
; It is more efficient to start by low bytes as the range of register to add is more limited.
; This implementation starts by low bytes
;
; Input: R22 (LSB) - R25(MSB)
; Return: R22 (LSB) - R25(MSB)
;
; Clock cycles: 59
;
;
; 1000d = 03E8h
;----------------------------------------------------------------------------------


u32_mul1000_shr_10:
	; Multiply by 1000, in r27:r26
	ldi r26,lo8(1000)
	ldi r27,hi8(1000)

	; clear result regs
	clr r20
	clr r21
	clr r30
	clr r31

	; ll*l r22*r26  -> r18:r19 (max: FE01)
	mul r22,r26
	movw r18,r0

	; lh*l r23*r26 -> r19:r20 (max: FEFF01 min 24 bit)
	mul r23,r26
	add r19,r0
	adc r20,r1

	; ll*h r22*27 -> r19:r20:r21 (max: 01FD0001 min 32 bit)
	mul r22,r27
	add r19,r0
	adc r20,r1
	adc r21,r31

	; lh*h r23*r27 -> r20:r21 (max: FFFE0001 min 32 bit)
	mul r23,r27
	add r20,r0
	adc r21,r1

	; hl*l r24*r26 -> r20:r21:r30 (max: 01FDFF0001 min 40 bits)
	mul r24,r26
	add r20,r0
	adc r21,r1
	adc r30,r31

	; hl*h r24*r27 -> r21:r30 (max: FFFEFF0001 min 40 bits)
	mul r24,r27
	add r21,r0
	adc r30,r1

	; hh*l r25*r26 -> r21:r30:r31 (max: 01FDFFFF0001 min 48 bit)
	mul r25,r26
	add r21,r0
	adc r30,r1
	adc r31,r31

	; hh*h r25*r27 -> r30:r31 (max: FFFEFFFF0001 min 48 bit)
	mul r25,r27
	add r30,r0
	adc r31,r1

	; 40 clock cycles until here


	; Move result to destination shifting right by 8 bits
	mov r22,r19
	mov r23,r20
	mov r24,r21
	mov r25,r30

	; >>=1
	lsr r31
	ror r25
	ror r24
	ror r23
	ror r22
	; >>=1
	lsr r31
	ror r25
	ror r24
	ror r23
	ror r22

	; Restore registers
	clr r1


	ret
	
;----------------------------------------------------------------------------------
; extern unsigned short u16_mul5925_shr_16(unsigned short v);
.global u16_mul5925_shr_16
;----------------------------------------------------------------------------------
; Takes an unsigned int (16-bit) v and multiply it by 5925 and shift it right by 16 bits.
; The output is the 16-bit number v*5925/65536.
; Internally uses a 32-bit representation to ensure no loss of resolution.
; Uses call-used registers as much as possible (R18-R27, R30-R31)
; 
;
; Input: R24 (LSB) - R25(MSB)
; Return: R24 (LSB) - R25(MSB)
;
; Clock cycles: 25
;
;
; 1000d = 03E8h
;----------------------------------------------------------------------------------


u16_mul5925_shr_16:
	; Multiply by 1000, in r27:r26
	ldi r26,lo8(5925)
	ldi r27,hi8(5925)

	; zero reg
	clr r22

	; l*l r24*r26  -> r18:r19
	mul r24,r26
	movw r18,r0

	; h*h r27*r25 -> r20:r21
	mul r27,r25
	movw r20,r0

	; h*l r25*r26 -> r19:r20:r21
	mul r25,r26
	add r19,r0
	adc r20,r1
	adc r21,r22

	; l*h r24*27 -> r19:r20:r21
	mul r24,r27
	add r19,r0
	adc r20,r1
	adc r21,r22

	

	; 19 clock cycles until here

	; Move result to destination shifting right by 16 bits
	movw r24,r20

	; Restore registers
	clr r1

	ret