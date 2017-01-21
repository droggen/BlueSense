//#define _SFR_ASM_COMPAT 1
#include <avr/io.h>

#define SSPI_PORT _SFR_IO_ADDR(PORTA)
#define SSPI_MOSI 0
#define SSPI_CLK 1
#define SSPI_mosimask (1<<SSPI_MOSI)
#define SSPI_clkmask (1<<SSPI_CLK)

;----------------------------------------------------------------------------------
; extern void softspi_bang(unsigned char d);
.global softspi_bang
;----------------------------------------------------------------------------------
; Parameters:
;	r24: d
; 
; Cycles: 3 + 8*4 + 4 = 39 cycles
; Cycles incl. function call: 39+5=44
;
; Use: T flag, R19, R25
;
; Return: -
;----------------------------------------------------------------------------------
softspi_bang:

	
		; Get current port state
		in r25, SSPI_PORT     
		; Clear clock
		andi r25, ~SSPI_clkmask
		; Load clock toggle
		ldi r19, SSPI_clkmask
		
		; Bit 7
		bst r24,7
		bld r25,SSPI_MOSI
		out SSPI_PORT, r25    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 6
		bst r24,6
		bld r25,SSPI_MOSI
		out SSPI_PORT, r25    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 5
		bst r24,5
		bld r25,SSPI_MOSI
		out SSPI_PORT, r25    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 4
		bst r24,4
		bld r25,SSPI_MOSI
		out SSPI_PORT, r25    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 3
		bst r24,3
		bld r25,SSPI_MOSI
		out SSPI_PORT, r25    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 2
		bst r24,2
		bld r25,SSPI_MOSI
		out SSPI_PORT, r25    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 1
		bst r24,1
		bld r25,SSPI_MOSI
		out SSPI_PORT, r25    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 0
		bst r24,0
		bld r25,SSPI_MOSI
		out SSPI_PORT, r25    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		
	
		
		ret




;----------------------------------------------------------------------------------
; extern void softspi_bangn(const unsigned char *data,unsigned char n)
.global softspi_bangn
;----------------------------------------------------------------------------------
; Parameters:
;	R25:R24: data
; R23:R22: n (assume 8-bit hence only R22 used in code)
; 
; Cycles: 
; Cycles incl. function call: 
;
; Use: T flag, R18, R19, R20, R25, R26, R27
;
; Return: -
;----------------------------------------------------------------------------------

softspi_bangn:
		; Get current port state
		in r18, SSPI_PORT     
		; Clear clock
		andi r18, ~SSPI_clkmask
		; Load clock toggle
		ldi r19, SSPI_clkmask
		
		; Move the pointer to x register
		movw	r26,r24
		
		; Initial test if counter equal zero
		tst r22
softspi_bangn_loop:
		; test or dec set the zero flag
		brne softspi_bangn_cont
		ret
softspi_bangn_cont:
		; read the data
		ld r20, x+
		
		; bitbang
		
		; Bit 7
		bst r20,7
		bld r18,SSPI_MOSI
		out SSPI_PORT, r18    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 6
		bst r20,6
		bld r18,SSPI_MOSI
		out SSPI_PORT, r18    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 5
		bst r20,5
		bld r18,SSPI_MOSI
		out SSPI_PORT, r18    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 4
		bst r20,4
		bld r18,SSPI_MOSI
		out SSPI_PORT, r18    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 3
		bst r20,3
		bld r18,SSPI_MOSI
		out SSPI_PORT, r18    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 2
		bst r20,2
		bld r18,SSPI_MOSI
		out SSPI_PORT, r18    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 1
		bst r20,1
		bld r18,SSPI_MOSI
		out SSPI_PORT, r18    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi
		; Bit 0
		bst r20,0
		bld r18,SSPI_MOSI
		out SSPI_PORT, r18    ; clk low + data
		out SSPI_PORT-2, r19  ; clk hi

		dec	r22
		jmp	softspi_bangn_loop
		
		
;----------------------------------------------------------------------------------
; extern void softspi_bangfast(unsigned char d);
.global softspi_bangfast
;----------------------------------------------------------------------------------
; Parameters:
;	r24: d
; 
; Cycles: 3 + 8*4 + 4 = 39 cycles
; Cycles incl. function call: 39+5=44
;
; Use: T flag, R18, R19, R25
; R0: counter
; R18: D0 CLK0
; R19: D1 CLK0
; R20: bitflag to toggle clk
; R24: data
; R25: tmp for multiplication
; R28,R29: address of functions
; Return: -
;----------------------------------------------------------------------------------
softspi_bangfast_table:
	rjmp softspi_bangfast_00
	rjmp softspi_bangfast_01
	rjmp softspi_bangfast_02
	rjmp softspi_bangfast_03
	rjmp softspi_bangfast_04
	rjmp softspi_bangfast_05
	rjmp softspi_bangfast_06
	rjmp softspi_bangfast_07
	rjmp softspi_bangfast_08
	rjmp softspi_bangfast_09
	rjmp softspi_bangfast_10
	rjmp softspi_bangfast_11
	rjmp softspi_bangfast_12
	rjmp softspi_bangfast_13
	rjmp softspi_bangfast_14
	rjmp softspi_bangfast_15
softspi_bangfast:


	; R0 counter for 2 nibbles
	clr R0

	; Get current port state
	in r18, SSPI_PORT     
    andi r18, ~(SSPI_clkmask+SSPI_mosimask)
	mov r19,r18
	ori r19,SSPI_mosimask
	ldi r20,SSPI_clkmask


softspi_bangfast_start:

softspi_bangfast_startnibble:
	; Swap nibble to start with MSB
	swap r24
	

	;http://www.avrfreaks.net/forum/how-do-you-make-jump-table-avr-assembly

	mov r25,r24			 
	andi r25,15
	; Load base address
	ldi ZL, lo8(pm(softspi_bangfast_table))
	ldi ZH, hi8(pm(softspi_bangfast_table))
	; Add offset
	add ZL, r25
	adc ZH, r1
	; Indirect jump to R31:R30
	ijmp



softspi_bangfast_00:	
	; 0000
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_01:
	; 0001
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_02:
	; 0010
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_03:
	; 0011
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_04:	
	; 0100
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_05:
	; 0101
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_06:
	; 0110
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_07:
	; 0111
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_08:	
	; 1000
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_09:
	; 1001
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_10:
	; 1010
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_11:
	; 1011
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_12:	
	; 1100
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_13:
	; 1101
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_14:
	; 1110
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_15:
	; 1111
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	jmp softspi_bangfast_end
	nop
	nop
	nop
	nop
	nop
	nop
	nop
softspi_bangfast_end:
	
	; Process the next 4 bits if needed
    eor r0, r20         ; R0 is counter, R20 has one bit set to one
    breq softspi_bangfast_ret
	rjmp softspi_bangfast_startnibble

softspi_bangfast_ret:


	ret



;----------------------------------------------------------------------------------
; extern void softspi_bangfast2(unsigned char d);
.global softspi_bangfast2
;----------------------------------------------------------------------------------
; Parameters:
;	r24: d
; 
; Cycles: 3 + 8*4 + 4 = 39 cycles
; Cycles incl. function call: 39+5=44
;
; Use: T flag, R18, R19, R25
; R0: counter
; R18: D0 CLK0
; R19: D1 CLK0
; R20: bitflag to toggle clk
; R24: data
; R25: tmp for multiplication
; R28,R29: address of functions
; Return: -
;----------------------------------------------------------------------------------
softspi_bangfast2_table:
	rjmp softspi_bangfast2_00
	rjmp softspi_bangfast2_01
	rjmp softspi_bangfast2_02
	rjmp softspi_bangfast2_03
	rjmp softspi_bangfast2_04
	rjmp softspi_bangfast2_05
	rjmp softspi_bangfast2_06
	rjmp softspi_bangfast2_07
	rjmp softspi_bangfast2_08
	rjmp softspi_bangfast2_09
	rjmp softspi_bangfast2_10
	rjmp softspi_bangfast2_11
	rjmp softspi_bangfast2_12
	rjmp softspi_bangfast2_13
	rjmp softspi_bangfast2_14
	rjmp softspi_bangfast2_15
softspi_bangfast2_table2:
	rjmp softspi_bangfast2_00b
	rjmp softspi_bangfast2_01b
	rjmp softspi_bangfast2_02b
	rjmp softspi_bangfast2_03b
	rjmp softspi_bangfast2_04b
	rjmp softspi_bangfast2_05b
	rjmp softspi_bangfast2_06b
	rjmp softspi_bangfast2_07b
	rjmp softspi_bangfast2_08b
	rjmp softspi_bangfast2_09b
	rjmp softspi_bangfast2_10b
	rjmp softspi_bangfast2_11b
	rjmp softspi_bangfast2_12b
	rjmp softspi_bangfast2_13b
	rjmp softspi_bangfast2_14b
	rjmp softspi_bangfast2_15b
softspi_bangfast2:

	; Get current port state
	in r18, SSPI_PORT     
    andi r18, ~(SSPI_clkmask+SSPI_mosimask)
	mov r19,r18
	ori r19,SSPI_mosimask
	ldi r20,SSPI_clkmask


softspi_bangfast2_start:

softspi_bangfast2_startnibble:
	; Swap nibble to start with MSB
	swap r24
	
	mov r25,r24			 
	andi r25,15
	; Load base address
	ldi ZL, lo8(pm(softspi_bangfast2_table))
	ldi ZH, hi8(pm(softspi_bangfast2_table))
	; Add offset
	add ZL, r25
	adc ZH, r1
	; Indirect jump to R31:R30
	ijmp



softspi_bangfast2_00:	
	; 0000
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_01:
	; 0001
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_02:
	; 0010
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_03:
	; 0011
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_04:	
	; 0100
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_05:
	; 0101
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_06:
	; 0110
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_07:
	; 0111
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_08:	
	; 1000
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_09:
	; 1001
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_10:
	; 1010
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_11:
	; 1011
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_12:	
	; 1100
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_13:
	; 1101
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_14:
	; 1110
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next
softspi_bangfast2_15:
	; 1111
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	rjmp softspi_bangfast2_next

softspi_bangfast2_next:
	
	; Swap nibble to start with MSB
	swap r24
	
	andi r24,15
	; Load base address
	ldi ZL, lo8(pm(softspi_bangfast2_table2))
	ldi ZH, hi8(pm(softspi_bangfast2_table2))
	; Add offset
	add ZL, r24
	adc ZH, r1
	; Indirect jump to R31:R30
	ijmp

softspi_bangfast2_00b:	
	; 0000
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_01b:
	; 0001
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_02b:
	; 0010
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_03b:
	; 0011
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_04b:
	; 0100
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_05b:
	; 0101
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_06b:
	; 0110
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_07b:
	; 0111
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_08b:	
	; 1000
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_09b:
	; 1001
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_10b:
	; 1010
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_11b:
	; 1011
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_12b:	
	; 1100
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_13b:
	; 1101
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_14b:
	; 1110
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r18
	out SSPI_PORT-2,r20	
	ret
softspi_bangfast2_15b:
	; 1111
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20
	out SSPI_PORT,r19
	out SSPI_PORT-2,r20	
	ret

softspi_bangfast2_end:


	ret


;----------------------------------------------------------------------------------
; extern unsigned char softspi_jttest(unsigned char d);
.global softspi_jttest
;----------------------------------------------------------------------------------
; Parameters:
;	r24: d
; 
;----------------------------------------------------------------------------------
softspi_jttest_table:
	rjmp softspi_jttest_00
	rjmp softspi_jttest_01
	rjmp softspi_jttest_02
	rjmp softspi_jttest_03
softspi_jttest:

	; only 2 bits
	andi r24,3
	; Base address
	ldi ZL, lo8(pm(softspi_jttest_table))
	ldi ZH, hi8(pm(softspi_jttest_table))
	; Offset address
	add ZL, r24
	adc ZH, r1

	; Indirect jump to R31:R30
	ijmp



softspi_jttest_00:	
	ldi	r24,55
	jmp softspi_jttest_end
softspi_jttest_01:
	ldi	r24,44
	jmp softspi_jttest_end
softspi_jttest_02:
	ldi	r24,33
	jmp softspi_jttest_end
softspi_jttest_03:
	ldi	r24,22
	jmp softspi_jttest_end

softspi_jttest_end:

	ret

.end