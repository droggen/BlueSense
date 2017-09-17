#include <avr/io.h>

; Helper functions to conver the MPU SPI buffer data into the MPUMOTIONDATA structure.
; Call-used registers (r18-r27, r30-r31) - can be clobbered by asm code
; Call-saved registers (r2-r17, r28-r29) - must be preserved by asm code
; r0 - can be clobbered anytime except by ints
; r1 - always 0
; Calling convention: allocated left to right, r25 to r8
; Return value: r24 (8 bit), r25:r24 (16 bit), r22:r25 (32 bit), etc.
; X: R27:R26
; Y: R29:R28
; Z: R31:R30

.text

;----------------------------------------------------------------------------------
; extern void __mpu_copy_spibuf_to_mpumotiondata_asm(unsigned char *spibuf,MPUMOTIONDATA *mpumotiondata);
.global __mpu_copy_spibuf_to_mpumotiondata_asm
;----------------------------------------------------------------------------------
; R25:R24: spibuf
; R23:R22: mpumotiondata
;-------------------------
__mpu_copy_spibuf_to_mpumotiondata_asm:
	
	; Place mpumotiondata pointer into Z register
	movw r30,r22
	; Place spibuf pointer into X register
	movw r26,r24
	
	; Test it works by writing data to spibuf and mpumotiondata
	;ldi r18,12	
	;st Z,r18	
	;ldi r18,13	
	;st X,r18
	
	; Increment spibuf to handle the big endian
	adiw r26,2
	 
	; Acceleration (big endian)
	ld r0,-x
	st z+,r0
	ld r0,-x
	st z+,r0
	
	adiw r26,4
	
	ld r0,-x
	st z+,r0
	ld r0,-x
	st z+,r0
	
	adiw r26,4
	
	ld r0,-x
	st z+,r0
	ld r0,-x
	st z+,r0
	
	; Temp (big endian)
	
	adiw r26,4
	adiw r30,13
	
	ld r0,-x
	st z+,r0
	ld r0,-x
	st z+,r0
	
	; Gyro (big endian)
	
	adiw r26,4
	sbiw r30,15
	
	ld r0,-x
	st z+,r0
	ld r0,-x
	st z+,r0
	
	adiw r26,4
	
	ld r0,-x
	st z+,r0
	ld r0,-x
	st z+,r0
	
	adiw r26,4
	
	ld r0,-x
	st z+,r0
	ld r0,-x
	st z+,r0
	
	
	; Magnetic (little endian)
	
	adiw r26,2
	
	ld r0,x+	; MX
	st z+,r0
	ld r0,x+
	st z+,r0
	
	ld r0,x+	; MY
	st z+,r0
	ld r0,x+
	st z+,r0
	
	ld r0,x+	; MZ
	st z+,r0
	ld r0,x+
	st z+,r0
	
	ld r0,x+	; MS
	st z+,r0
	
	
	
	
	

	ret
	
;----------------------------------------------------------------------------------
; extern "C" void mpu_mag_correct2_asm(signed short *mx,signed short *my,signed short *mz);
.global mpu_mag_correct2_asm
;----------------------------------------------------------------------------------
; Input parameters:
; R25:R24: address of mx
; R23:R22: address of my
; R21:R20: address of mz
; Register use
; R23:R22:	M? to process 
; R21:R20:	bias or sensitivity
; R18:R17:R16:	Multiplication result
;-------------------------
mpu_mag_correct2_asm:

	; Load mx in r23:r22
	movw r30,r24
	ld	r22,z+
	ld	r23,z+
	
	; Load bias in r20
	lds r20,_mpu_mag_bias
	lds r21,_mpu_mag_bias+1
	
	; Add bias to m
	add	r22,r20
	adc	r23,r21
	
	;st -z,r19		; Store mx+bias
	;st -z,r18
	
	; Load sens in r20
	lds r20,_mpu_mag_sens
	lds r21,_mpu_mag_sens+1
	
	; Multiply M(R23:rR22) x sens(R21:R20)
	call mul16x16_24	; result in r18:16
	; Shift right by 7: shift left by 1 and drop least significant byte
	;add r16,r16
	;adc r17,r17
	;adc r18,r18
	
	; Store mx
	st -z,r17
	st -z,r16
	
	clr r1

	ret
	
;******************************************************************************
;*
;* FUNCTION
;*	mul16x16_24
;* DECRIPTION
;*	Unsigned multiply of two 16bits numbers with 24bits result.
;* USAGE
;*	r18:r17:r16 = r23:r22 * r21:r20
;* STATISTICS
;*	Cycles :	14 + ret
;*	Words :		10 + ret
;*	Register usage: r0 to r1, r16 to r18 and r20 to r23 (9 registers)
;* NOTE
;*	Full orthogonality i.e. any register pair can be used as long as
;*	the 24bit result and the two operands does not share register pairs.
;*	The routine is non-destructive to the operands.
;*
;******************************************************************************

mul16x16_24:
	mul		r23, r21		; ah * bh
	mov		r18, r0
	mul		r22, r20		; al * bl
	movw	r16, r0
	mul		r23, r20		; ah * bl
	add		r17, r0
	adc		r18, r1
	mul		r21, r22		; bh * al
	add		r17, r0
	adc		r18, r1
	ret

