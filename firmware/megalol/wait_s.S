#include <avr/io.h>


.global timer_ms_get_asm
.text
; Assembler replacement of timer_ms_get
;
; Relies on the following global variables: _timer_time_1_in_ms, _timer_time_1024, _timer_lastmillisec
; 
; Implements: 
; ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
;	{
;		t1 = _timer_time_1_in_ms;
;		t1024 = _timer_time_1024;
;	}
;	t1024=u32_mul1000_shr_10(t1024);
;	t = t1 + t1024;
;	if(t<_timer_lastmillisec)
;		return _timer_lastmillisec;
;	_timer_lastmillisec = t;
;	return t;
;
; 
.extern _timer_time_1_in_ms
.extern _timer_time_1024
.extern _timer_lastmillisec

timer_ms_get_asm:
	
	push r14
	push r15
	push r16
	push r17
	
	
	; Disable interrupts and quickly copy the timers
	lds r31,SREG
	cli
	; Load time_1024 in R22-R25
	lds r22,_timer_time_1024
	lds r23,_timer_time_1024+1
	lds r24,_timer_time_1024+2
	lds r25,_timer_time_1024+3
	; Load time_1 in R14-R17
	lds r14,_timer_time_1_in_ms
	lds r15,_timer_time_1_in_ms+1
	lds r16,_timer_time_1_in_ms+2
	lds r17,_timer_time_1_in_ms+3
	sts SREG,r31

	; Multiply R22-R25 by 1000 in R26:R27
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

	; Add timer 1 (R14-R17) to R22-R25
	add r22,r14
	adc r23,r15
	adc r24,r16
	adc r25,r17


	pop r17
	pop r16
	pop r15
	pop r14


	clr r1

	ret
	
	
	
.global timer_ms_get_asm_fast
.text
; Assembler replacement of timer_ms_get
;
; Relies on the following global variables: _timer_time_1_in_ms, _timer_time_1024, _timer_lastmillisec
;
; Assumes that _timer_time_1024 is a 16-bit variable (maximum 60 seconds), which should always be respected
; when the 1Hz RTC is active.
; The multiplication of 32x16->48 is changed to 16x16->32 compared to timer_ms_get_asm_fast
; 
; 

timer_ms_get_asm_fast:
	
	push r14
	push r15
	push r16
	
	
	
	; Disable interrupts and quickly copy the timers
	lds r31,SREG
	cli
	; Load time_1024 in R22-R25
	lds r22,_timer_time_1024
	lds r23,_timer_time_1024+1
	; Load time_1 in R14:R15:R16:R30
	lds r14,_timer_time_1_in_ms
	lds r15,_timer_time_1_in_ms+1
	lds r16,_timer_time_1_in_ms+2
	lds r30,_timer_time_1_in_ms+3
	sts SREG,r31

	; Multiply R22-R25 by 1000 in R26:R27
	ldi r26,lo8(1000)
	ldi r27,hi8(1000)

	; clear result regs
	clr r25


	; l*l r22*r26  -> r18:r19 (max: FE01)
	mul r22,r26
	movw r18,r0
	
	; h*h r23*r27  -> r20:r21 (max: FE01)
	mul r23,r27
	movw r20,r0

	; h*l r23*r26 -> r19:r20 (max: FEFF01 min 24 bit)
	mul r23,r26
	add r19,r0
	adc r20,r1
	adc r21,r25
	
	; l*h r22*r27 -> r19:r20:r21 (max: FEFF01 min 24 bit)
	mul r22,r27
	add r19,r0
	adc r20,r1
	adc r21,r25

	; Move result to destination shifting right by 8 bits
	mov r22,r19
	mov r23,r20
	mov r24,r21

	; >>=1
	lsr r24
	ror r23
	ror r22
	; >>=1
	lsr r24
	ror r23
	ror r22

	; Add timer 1 (R14-R17) to R22-R25
	add r22,r14
	adc r23,r15
	adc r24,r16
	adc r25,r30



	pop r16
	pop r15
	pop r14
	
	clr r1

	ret
		
		
.global timer_us_get_asm_fast
.text
; Assembler replacement of timer_us_get
;
; Relies on the following global variables: _timer_time_1_in_ms, _timer_time_1024, and timer 1 TCNT1 register
;
; Assumes that _timer_time_1024 is a 16-bit variable (maximum 60 seconds), which should always be respected
; when the 1Hz RTC is active.
; The multiplication of 32x16->48 is changed to 16x16->32 compared to timer_ms_get_asm_fast
; 
; 

timer_us_get_asm_fast:
	
	push r12
	push r13
	push r14
	push r15
	push r16
	push r17
	
	
	
	; Disable interrupts and quickly copy the timers
	lds r31,SREG
	cli
	; TCNT1 in R12-R13
	lds r12,TCNT1L
	lds r13,TCNT1H
	; Load time_1024 in R22-R25
	lds r22,_timer_time_1024
	lds r23,_timer_time_1024+1
	; Load time_1 in R14:R15:R16:R30
	lds r14,_timer_time_1_in_ms
	lds r15,_timer_time_1_in_ms+1
	lds r16,_timer_time_1_in_ms+2
	lds r30,_timer_time_1_in_ms+3
	sts SREG,r31

	; Multiply R22-R25 by 1000 in R26:R27
	ldi r26,lo8(1000)
	ldi r27,hi8(1000)

	; clear result regs
	clr r25


	; l*l r22*r26  -> r18:r19 (max: FE01)
	mul r22,r26
	movw r18,r0
	
	; h*h r23*r27  -> r20:r21 (max: FE01)
	mul r23,r27
	movw r20,r0

	; h*l r23*r26 -> r19:r20 (max: FEFF01 min 24 bit)
	mul r23,r26
	add r19,r0
	adc r20,r1
	adc r21,r25
	
	; l*h r22*r27 -> r19:r20:r21 (max: FEFF01 min 24 bit)
	mul r22,r27
	add r19,r0
	adc r20,r1
	adc r21,r25

	; Move result to destination shifting right by 8 bits
	mov r22,r19
	mov r23,r20
	mov r24,r21

	; >>=1
	lsr r24
	ror r23
	ror r22
	; >>=1
	lsr r24
	ror r23
	ror r22

	; Add timer 1 (R14-R17) to R22-R25
	add r22,r14
	adc r23,r15
	adc r24,r16
	adc r25,r30

	; At this stage: R22-R25 is time in millisecond
	
	; Multiply R22-R25 by 1000; as the result is a 32bit, we simplify the multiplication to drop bytes 5 and 6 (r30,r31)
	; Multiply R22-R25 by 1000 in R26:R27
	ldi r26,lo8(1000)
	ldi r27,hi8(1000)

	; clear result regs
	clr r20
	clr r21
	;clr r30
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
	;adc r30,r31

	; hl*h r24*r27 -> r21:r30 (max: FFFEFF0001 min 40 bits)
	mul r24,r27
	add r21,r0
	;adc r30,r1

	; hh*l r25*r26 -> r21:r30:r31 (max: 01FDFFFF0001 min 48 bit)
	mul r25,r26
	add r21,r0
	;adc r30,r1
	;adc r31,r31

	; hh*h r25*r27 -> r30:r31 (max: FFFEFFFF0001 min 48 bit)
	;mul r25,r27
	;add r30,r0
	;adc r31,r1

	; Copy to destination
	movw r22,r18
	movw r24,r20
	; At this stage, R22-R25 are microseconds, not including tcnt1
	
	; Convert tcnt R12-R13 to microseconds: multiply by 5925/2^16 result in R18-R21
	
	; Multiply by 5925, in r27:r26
	ldi r26,lo8(5925)
	ldi r27,hi8(5925)

	; R31 is zero here

	; l*l r12*r26  -> r18:r19
	mul r12,r26
	movw r18,r0

	; h*h r13*r27 -> r20:r21
	mul r13,r27
	movw r20,r0

	; h*l r13*r26 -> r19:r20:r21:r22
	mul r13,r26
	add r19,r0
	adc r20,r1
	adc r21,r31

	; l*h r12*27 -> r19:r20:r21:r22
	mul r13,r27
	add r19,r0
	adc r20,r1
	adc r21,r31
	
	; Shift by 16 (ignore r18-r19) and add to time in us
	add r22,r20
	adc r23,r21
	adc r24,r31
	adc r25,r31
	
	; Here R22-R25 is time in us




	pop r17
	pop r16
	pop r15
	pop r14
	pop r13
	pop r12
	
	clr r1

	ret
		