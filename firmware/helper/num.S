;*************************************************************************
;*                                                                       *
;*                      Convert unsigned 16 bit to 5 digit ASCII         *
;*                                                                       *
;*              Author: Peter Dannegger                                  *
;*                                                                       *
;*************************************************************************
;
;input: R17, R16 = 16 bit value 0 ... 65535
;output: R20, R19, R18, R17, R16 = 5 digits (ASCII)
;cycle: 20 ... 170
;

.global bin2bcd3

.section text

bin16_ascii:

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
        ret
;-------------------------------------------------------------------------
.global add1
.section text

add1:	
	adiw r24,1
	ret

;-------------------------------------------------------------------------

