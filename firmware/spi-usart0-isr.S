#include <avr/io.h>

; Handles ISR for SPI-USART0
; Same behaviour as C version
; Assumes slave select line is PORTA,4

;----------------------------------------------------------------------------------
; extern void spiisr(void);
.global spiisr
.global __vector_22
;----------------------------------------------------------------------------------
; SPI interrupt-driven data transmission with callback
;	*_spiusart0_bufferptr=SPDR;
;	if(_spiusart0_n==0)
;		return
;	_spiusart0_n--;
;	_spiusart0_bufferptr++;
;	SPDR = *_spiusart0_bufferptr;	
; 
; 
; Clock cycles [TODO: UPDATE - CLOCK CYCLES ARE SLIGHTLY LOWER AFTER OPTIMISATION: 
;		61 clock cycles if callback not called (incl ret)
;		101 clock cycles if callback called, with empty callback (only ret) (incl ret)

; Interrupt call+return adds 5+5 cycles (not clear if return cycles overlap reti). If sleep: 5 more, plus wake-up time.
; Worst case w/o callback: 61+5+5=71 in normal mode; 76 in sleep mode
; Worst case w/  callback: 101+5+5=111 in normal mode; 116 in sleep mode
;
; Interrupt-driven SPI transfer is only worthwhile if the SPI clock is slow enough to allow the processor 
; to perform other operations (or sleep) while data is transferred.
; 
; Assuming sleep and transfer of n bytes:
; Clock=76*(n-1)+116
; For data transfers of size n->inf, interrupt-driven data transfer is worth for spiclk=fclk/(9.5+)
; For data transfers of n=1: spiclk=fclk/(14.5+)
; For data transfers of n=2: spiclk=fclk/(12+)
; For data transfers of n=4: spiclk=fclk/(10.75+)
; For data transfers of n=8: spiclk=fclk/(10.125+)
; For data transfers of n=16: spiclk=fclk/(9.8125+)
; 
; Assuming normal and transfer of n bytes:
; Clock=71*(n-1)+111
; For data transfers of size n->inf, interrupt-driven data transfer is worth for spiclk=fclk/(8.875+)
; For data transfers of n=1: spiclk=fclk/(13.875+)
; For data transfers of n=4: spiclk=fclk/(10.125+)
; 		
;----------------------------------------------------------------------------------
; Must use "call used" registers in case callback called to minimize push/pop:
; R0, R18–R27, R30-R31
;----------------------------------------------------------------------------------

spiisr:
__vector_22:
	; Save SREG, R30, R31
	push r30
	in r30,_SFR_IO_ADDR(SREG)
	push r30
	push r31

	; Save R26, R27, R0
	push r26
	push r27
	push r0

	; Load _spiusart0_bufferptr in Z
	lds	r30,_spiusart0_bufferptr
	lds	r31,_spiusart0_bufferptr+1

	; Get SPDR
	;in r0,_SFR_IO_ADDR(SPDR)
	lds r0,UDR0

	; Store SPDR at pointer, increment pointer
	st z+,r0

	; Load address of _spiusart0_bufferptr in X
	ldi r26,lo8(_spiusart0_bufferptr)
	ldi r27,hi8(_spiusart0_bufferptr)

	; Store incremented pointer to address of pointer
	st x+,r30
	st x+,r31

	; Load _spiusart0_n
	lds r26,_spiusart0_n
	lds r27,_spiusart0_n+1

	; Decrement _spiusart0_n
	sbiw r26,1

	; Compare _spiusart0_n to 0; previous subtraction sets Z flag
	brbs 1,spiisr_cb

spiisr_exchange:
	; Load spibuf data
	ld r0,z

	; Output data
	;out _SFR_IO_ADDR(SPDR),r0
	sts UDR0,r0

	; Load address of _spiusart0_n
	ldi r30,lo8(_spiusart0_n);
	ldi r31,hi8(_spiusart0_n);

	; Store decremented _spiusart0_n
	st z+,r26
	st z+,r27

	

	;jmp spiisr_end
	; End of routine: restore
	pop r0
	pop r27
	pop r26
	pop r31
	pop r30
	;sts SREG,r30
	out _SFR_IO_ADDR(SREG),r30
	pop r30
	;ret
	reti

	; Reached zero: call the callback
spiisr_cb:
	
	; Deselect device: PINA=0b00010000;
	sbi _SFR_IO_ADDR(PINA),4
	
	; Deactivate interrupt: UCSR0B &= 0b10111111;
	lds r30,UCSR0B
	andi r30,0b10111111
	sts UCSR0B,r30

	; _spiusart0_ongoing=0;
	clr r30
	sts _spiusart0_ongoing,r30
	

	; Load address of callback in Z
	lds r30,_spiusart0_callback
	lds r31,_spiusart0_callback+1

	; Check if callback is non-null
	sbiw r30,0
	brbs 1,spiisr_cb_end


	; Call-used registers are: R0, R18–R27, R30-R31, T flag. Expected: R1=0
	; Save remainder of call-used register
	push r1
	clr r1
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25

	icall

	pop r25
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r1

	



spiisr_cb_end:
	; End of routine: restore
	pop r0
	pop r27
	pop r26
	pop r31
	pop r30
	;sts SREG,r30
	out _SFR_IO_ADDR(SREG),r30
	pop r30
	;ret
	reti

	


;spiisr_end:
	; Restore
	;pop r0
	;pop r27
	;pop r26
	;pop r31
	;pop r30
	;;sts SREG,r30
	;out _SFR_IO_ADDR(SREG),r30
	;pop r30
	;;ret
	;reti


