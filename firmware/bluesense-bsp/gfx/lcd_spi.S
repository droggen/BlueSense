#include <avr/io.h>

;----------------------------------------------------------------------------------
; Replacement functions to lcd_spi_write8 and lcd_spi_writen
; When including this asm file make sure the C equivalent code is disabled.
;----------------------------------------------------------------------------------

#define SSPI_PORT _SFR_IO_ADDR(PORTA)
#define SSPI_MOSI 0
#define SSPI_CLK 1
#define SSPI_mosimask (1<<SSPI_MOSI)
#define SSPI_clkmask (1<<SSPI_CLK)

;----------------------------------------------------------------------------------
; extern void lcd_spi_write8(unsigned char d);
.global lcd_spi_write8
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
lcd_spi_write8:

	
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
; extern void lcd_spi_writen(const unsigned char *data,unsigned char n)
.global lcd_spi_writen
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

lcd_spi_writen:
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
lcd_spi_writen_loop:
		; test or dec set the zero flag
		brne lcd_spi_writen_cont
		ret
lcd_spi_writen_cont:
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
		jmp	lcd_spi_writen_loop
		





.end


