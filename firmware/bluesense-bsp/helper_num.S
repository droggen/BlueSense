#include <avr/io.h>


.global add1
.text

add1:	
		adiw r24,2
		ret
	

; Can use: r0, r18-r27, r30-r31
; Must preserve: r1, r2-r17, r28-r29
; r0 is tmp
; r1 is 0

; extern unsigned char ht1(void v1,unsigned char *ptr);
; r25:r24: v1
; r23:r22: ptr
; 
; X: r27:r26

.global ht1

ht1:

	; Move ptr to the pointer register
	movw	r26,r22
	st	x+,r1
	st	x,r1
	
;----------------------------------------------------------------------------------
; extern unsigned char ht2(unsigned short v,unsigned char *ptr);
.global ht2
;----------------------------------------------------------------------------------
; Parameters
; r25:r24: v
; r23:r22: ptr
;
;input: R17, R16 = 16 bit value 0 ... 65535
;output: R20, R19, R18, R17, R16 = 5 digits (ASCII)
;----------------------------------------------------------------------------------
ht2:
				; save R16,R17
				push r16
				push r17
	
	
				; Move input v (r25:r24) to r17:r16
				movw	r16,r24

        ldi     r20, -1 + '0'
_bcd1:  inc     r20
        subi    r16, lo8(10000)         ;-10000
        sbci    r17, hi8(10000)
        brcc    _bcd1

        ldi     r19, 10 + '0'
_bcd2:  dec     r19
        subi    r16, lo8(-1000)         ;+1000
        sbci    r17, hi8(-1000)
        brcs    _bcd2

        ldi     r18, -1 + '0'
_bcd3:  inc     r18
        subi    r16, lo8(100)           ;-100
        sbci    r17, hi8(100)
        brcc    _bcd3

        ldi     r17, 10 + '0'
_bcd4:  dec     r17
        subi    r16, -10                ;+10
        brcs    _bcd4

        subi    r16, -'0'
        
        ; Store the results into the string
        ; X (r27:r26) points to string
        movw	r26,r22
        st x+,r20
        st x+,r19
        st x+,r18
        st x+,r17
        st x+,r16
        st x+,r1       
        
        ; restore r16, r17
        pop r17
        pop r16
        
        ret
	
	
;----------------------------------------------------------------------------------
; extern unsigned char ht3(unsigned short v,unsigned char *ptr);
.global ht3
;----------------------------------------------------------------------------------
; Parameters
; r25:r24: v
; r23:r22: ptr
;
;input: R17, R16 = 16 bit value 0 ... 65535
;output: R20, R19, R18, R17, R16 = 5 digits (ASCII)
;----------------------------------------------------------------------------------
ht3:

        ldi     r20, -1 + '0'
_ht3_bcd1:  inc     r20
        subi    r24, lo8(10000)         ;-10000
        sbci    r25, hi8(10000)
        brcc    _ht3_bcd1

        ldi     r19, 10 + '0'
_ht3_bcd2:  dec     r19
        subi    24, lo8(-1000)         ;+1000
        sbci    r25, hi8(-1000)
        brcs    _ht3_bcd2

        ldi     r18, -1 + '0'
_ht3_bcd3:  inc     r18
        subi    r24, lo8(100)           ;-100
        sbci    r25, hi8(100)
        brcc    _ht3_bcd3

        ldi     r25, 10 + '0'
_ht3_bcd4:  dec     r25
        subi    r24, -10                ;+10
        brcs    _ht3_bcd4

        subi    r24, -'0'
        
        ; Store the results into the string
        ; X (r27:r26) points to string
        ; result in: r20 r19 r18 r25 r24?
        movw	r26,r22
        st x+,r20
        st x+,r19
        st x+,r18
        st x+,r25
        st x+,r24
        st x+,r1       
        
        
        ret

;----------------------------------------------------------------------------------
; extern unsigned char u16toa(unsigned short v,unsigned char *ptr);
.global u16toa
;----------------------------------------------------------------------------------
; Converts a 16-bit unsigned number into a 6 byte ascii string (5 bytes for the
; number + one trailing null).
; 
; C to assembler parameter mapping:
; 	r25:r24: v
; 	r23:r22: ptr
;
;	Uses pointer X (R27:R26) to point to the string. 
;	
;----------------------------------------------------------------------------------
u16toa:

				; Store the results into the string
        ; X (r27:r26) points to string
        movw	r26,r22

        ldi     r20, -1 + '0'
_u16toa_bcd1:  inc     r20
        subi    r24, lo8(10000)         ;-10000
        sbci    r25, hi8(10000)
        brcc    _u16toa_bcd1

				st x+,r20


        ldi     r20, 10 + '0'
_u16toa_bcd2:  dec     r20
        subi    24, lo8(-1000)         ;+1000
        sbci    r25, hi8(-1000)
        brcs    _u16toa_bcd2
        
        st x+,r20

        ldi     r20, -1 + '0'
_u16toa_bcd3:  inc     r20
        subi    r24, lo8(100)           ;-100
        sbci    r25, hi8(100)
        brcc    _u16toa_bcd3
        
				st x+,r20

        ldi     r20, 10 + '0'
_u16toa_bcd4:  dec     r20
        subi    r24, -10                ;+10
        brcs    _u16toa_bcd4

				st x+,r20

        subi    r24, -'0'
        
        st x+,r24
        
        st x+,r1             
        
        ret
        
;----------------------------------------------------------------------------------
; extern unsigned char u32toa(unsigned long v,unsigned char *ptr);
.global u32toa
;----------------------------------------------------------------------------------
; Converts a 32-bit unsigned number into a 11 byte ascii string (10 bytes for the
; number + one trailing null).
; 
; C to assembler parameter mapping:
; 	r25:r22: v
; 	r21:r20: ptr
;
;	Uses pointer X (R27:R26) to point to the string. 
;	
;----------------------------------------------------------------------------------
u32toa:

				; Store the results into the string
        ; X (r27:r26) points to string
        movw	r26,r20
        
        
        ldi     r20, -1 + '0'
_u32toa_bcd1:  inc     r20
        subi    r22, lo8(1000000000)         ;-1000000000
        sbci    r23, hi8(1000000000)
        sbci    r24, hh8(1000000000)
        sbci    r25, hhi8(1000000000)
        brcc    _u32toa_bcd1

				st x+,r20
        
        
        ldi     r20, 10 + '0'
_u32toa_bcd2:  dec     r20
        subi    r22, lo8(-100000000)         ;+100000000
        sbci    r23, hi8(-100000000)
        sbci    r24, hh8(-100000000)
        sbci    r25, hhi8(-100000000)
        brcs    _u32toa_bcd2

				st x+,r20


        ldi     r20, -1 + '0'
_u32toa_bcd3:  inc     r20
        subi    r22, lo8(10000000)         ;-10000000
        sbci    r23, hi8(10000000)
        sbci    r24, hh8(10000000)
        sbci    r25, hhi8(10000000)
        brcc    _u32toa_bcd3

				st x+,r20
        
        
        ldi     r20, 10 + '0'
_u32toa_bcd4:  dec     r20
        subi    r22, lo8(-1000000)         ;+1000000
        sbci    r23, hi8(-1000000)
        sbci    r24, hh8(-1000000)
        sbci    r25, hhi8(-1000000)
        brcs    _u32toa_bcd4

				st x+,r20


        ldi     r20, -1 + '0'
_u32toa_bcd5:  inc     r20
        subi    r22, lo8(100000)         ;-100000
        sbci    r23, hi8(100000)
        sbci    r24, hh8(100000)
        sbci    r25, hhi8(100000)
        brcc    _u32toa_bcd5

				st x+,r20
        
        
        ldi     r20, 10 + '0'
_u32toa_bcd6:  dec     r20
        subi    r22, lo8(-10000)         ;+10000
        sbci    r23, hi8(-10000)
        sbci    r24, hh8(-10000)
        sbci    r25, hhi8(-10000)
        brcs    _u32toa_bcd6

				st x+,r20



  			ldi     r20, -1 + '0'
_u32toa_bcd7:  inc     r20
        subi    r22, lo8(1000)         ;-1000
        sbci    r23, hi8(1000)
        sbci    r24, hh8(1000)
        sbci    r25, hhi8(1000)
        brcc    _u32toa_bcd7

				st x+,r20
        
        
        ldi     r20, 10 + '0'
_u32toa_bcd8:  dec     r20
        subi    r22, lo8(-100)         ;+100
        sbci    r23, hi8(-100)
        sbci    r24, hh8(-100)
        sbci    r25, hhi8(-100)
        brcs    _u32toa_bcd8

				st x+,r20



  			ldi     r20, -1 + '0'
_u32toa_bcd9:  inc     r20
        subi    r22, lo8(10)         ;-10
        sbci    r23, hi8(10)
        sbci    r24, hh8(10)
        sbci    r25, hhi8(10)
        brcc    _u32toa_bcd9

				st x+,r20
        
        
        ldi     r20, 10 + '0'
_u32toa_bcd10:  dec     r20
        subi    r22, lo8(-1)         ;+1
        sbci    r23, hi8(-1)
        sbci    r24, hh8(-1)
        sbci    r25, hhi8(-1)
        brcs    _u32toa_bcd10

				st x+,r20



        
        st x+,r1       
        
        
        ret
	

.end