

//************************************************************************
//*	these are used to test issues
//*	http://code.google.com/p/arduino/issues/detail?id=505
//*	Reported by mark.stubbs, Mar 14, 2011
//*	The STK500V2 bootloader is comparing the seqNum to 1 or the current sequence 
//*	(IE: Requiring the sequence to be 1 or match seqNum before continuing).  
//*	The correct behavior is for the STK500V2 to accept the PC's sequence number, and echo it back for the reply message.
#define	_FIX_ISSUE_505_
//************************************************************************
//*	Issue 181: added watch dog timmer support
#define	_FIX_ISSUE_181_

#include "cpu.h"

#include	<inttypes.h>
#include	<avr/io.h>
#include	<avr/interrupt.h>
#include	<avr/boot.h>
#include	<avr/pgmspace.h>
#include	<util/delay.h>
#include	<avr/eeprom.h>
#include	<avr/common.h>
#include	<stdlib.h>
#include	<stdio.h>
#include "serial.h"
#include "command.h"
#include "system.h"
#include "main.h"


extern FILE* file_usb;

#undef		ENABLE_MONITOR

#ifndef EEWE
	#define EEWE    1
#endif
#ifndef EEMWE
	#define EEMWE   2
#endif

//#define	_DEBUG_SERIAL_
//#define	_DEBUG_WITH_LEDS_


/*
 * Uncomment the following lines to save code space
 */
//#define	REMOVE_PROGRAM_LOCK_BIT_SUPPORT		// disable program lock bits
//#define	REMOVE_BOOTLOADER_LED				// no LED to show active bootloader
//#define	REMOVE_CMD_SPI_MULTI				// disable processing of SPI_MULTI commands, Remark this line for AVRDUDE <Worapoht>
//

// Define to read actual battery voltage
#define ENABLE_TRUEVTARGET


//************************************************************************
//*	LED on pin "PROGLED_PIN" on port "PROGLED_PORT"
//*	indicates that bootloader is active
//*	PG2 -> LED on Wiring board
//************************************************************************
#define		BLINK_LED_WHILE_WAITING

#define PROGLED_PORT	PORTC
#define PROGLED_DDR		DDRC
#define PROGLED_PIN		PINC6






#define	_BLINK_LOOP_COUNT_	(F_CPU / 2250)
/*
 * UART Baudrate, AVRStudio AVRISP only accepts 115200 bps
 */

#ifndef BAUDRATE
	#define BAUDRATE 115200
#endif

/*
 *  Enable (1) or disable (0) USART double speed operation
 */
#ifndef UART_BAUDRATE_DOUBLE_SPEED
	#if defined (__AVR_ATmega32__)
		#define UART_BAUDRATE_DOUBLE_SPEED 0
	#else
		#define UART_BAUDRATE_DOUBLE_SPEED 1
	#endif
#endif

/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW	0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH	0
#define CONFIG_PARAM_HW_VER				0x0F
#define CONFIG_PARAM_SW_MAJOR			2
#define CONFIG_PARAM_SW_MINOR			0x0A

/*
 * Calculate the address where the bootloader starts from FLASHEND and BOOTSIZE
 * (adjust BOOTSIZE below and BOOTLOADER_ADDRESS in Makefile if you want to change the size of the bootloader)
 */
//#define BOOTSIZE 1024
#if FLASHEND > 0x0F000
	#define BOOTSIZE 8192
#else
	#define BOOTSIZE 2048
#endif

#define APP_END  (FLASHEND -(2*BOOTSIZE) + 1)

/*
 * Signature bytes are not available in avr-gcc io_xxx.h
 */
#if defined (__AVR_ATmega8__)
	#define SIGNATURE_BYTES 0x1E9307
#elif defined (__AVR_ATmega16__)
	#define SIGNATURE_BYTES 0x1E9403
#elif defined (__AVR_ATmega32__)
	#define SIGNATURE_BYTES 0x1E9502
#elif defined (__AVR_ATmega8515__)
	#define SIGNATURE_BYTES 0x1E9306
#elif defined (__AVR_ATmega8535__)
	#define SIGNATURE_BYTES 0x1E9308
#elif defined (__AVR_ATmega162__)
	#define SIGNATURE_BYTES 0x1E9404
#elif defined (__AVR_ATmega128__)
	#define SIGNATURE_BYTES 0x1E9702
#elif defined (__AVR_ATmega1280__)
	#define SIGNATURE_BYTES 0x1E9703
#elif defined (__AVR_ATmega2560__)
	#define SIGNATURE_BYTES 0x1E9801
#elif defined (__AVR_ATmega2561__)
	#define SIGNATURE_BYTES 0x1e9802
#elif defined (__AVR_ATmega1284P__)
	#define SIGNATURE_BYTES 0x1e9705
#elif defined (__AVR_ATmega640__)
	#define SIGNATURE_BYTES  0x1e9608
#elif defined (__AVR_ATmega64__)
	#define SIGNATURE_BYTES  0x1E9602
#elif defined (__AVR_ATmega169__)
	#define SIGNATURE_BYTES  0x1e9405
#elif defined (__AVR_AT90USB1287__)
	#define SIGNATURE_BYTES  0x1e9782
#else
	#error "no signature definition for MCU available"
#endif





/*
 * States used in the receive state machine
 */
#define	ST_START		0
#define	ST_GET_SEQ_NUM	1
#define ST_MSG_SIZE_1	2
#define ST_MSG_SIZE_2	3
#define ST_GET_TOKEN	4
#define ST_GET_DATA		5
#define	ST_GET_CHECK	6
#define	ST_PROCESS		7

/*
 * use 16bit address variable for ATmegas with <= 64K flash
 */
#if defined(RAMPZ)
	typedef uint32_t address_t;
#else
	typedef uint16_t address_t;
#endif

/*
 * function prototypes
 */
static void sendchar(char c);
static unsigned char recchar(void);

/*
 * since this bootloader is not linked against the avr-gcc crt1 functions,
 * to reduce the code size, we need to provide our own initialization
 */
void __jumpMain	(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
#include <avr/sfr_defs.h>

//#define	SPH_REG	0x3E
//#define	SPL_REG	0x3D

//*****************************************************************************
void __jumpMain(void)
{
//*	July 17, 2010	<MLS> Added stack pointer initialzation
//*	the first line did not do the job on the ATmega128

	asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );

//*	set stack pointer to top of RAM

	asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );

	asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );

	asm volatile ( "clr __zero_reg__" );									// GCC depends on register r1 set to 0
	asm volatile ( "out %0, __zero_reg__" :: "I" (_SFR_IO_ADDR(SREG)) );	// set SREG to 0
	asm volatile ( "jmp main");												// jump to main()
}


//*****************************************************************************
void delay_ms(unsigned int timedelay)
{
	unsigned int i;
	for (i=0;i<timedelay;i++)
	{
		_delay_ms(0.5);
	}
}


//*****************************************************************************
/*
 * send single byte to USART, wait until transmission is completed
 */
static void sendchar(char c)
{
	putchar(c);
}


//************************************************************************
static int	Serial_Available(void)
{
	return fgetrxbuflevel(file_usb) ? 1:0;
}


//*****************************************************************************
/*
 * Read single byte from USART, block if no data available
 */
static unsigned char recchar(void)
{
	serial_setblocking(file_usb,1);
	return getchar();
	
}

#define	MAX_TIME_COUNT	(F_CPU >> 1)
//*****************************************************************************
static unsigned char recchar_timeout(void)
{
	return getchar();
	/*
uint32_t count = 0;

	while (!(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE)))
	{
		// wait for data
		count++;
		if (count > MAX_TIME_COUNT)
		{
		unsigned int	data;
		#if (FLASHEND > 0x10000)
			data	=	pgm_read_word_far(0);	/	get the first word of the user program
		#else
			data	=	pgm_read_word_near(0);	/	get the first word of the user program
		#endif
			if (data != 0xffff)					/	make sure its valid before jumping to it.
			{
				asm volatile(
						"clr	r30		\n\t"
						"clr	r31		\n\t"
						"ijmp	\n\t"
						);
			}
			count	=	0;
		}
	}
	return UART_DATA_REG;*/
}

//*	for watch dog timer startup
void (*app_start)(void) = 0x0000;

void jumpusr(void)
{
		boot_rww_enable();				// enable application section


	asm volatile(
			"clr	r30		\n\t"
			"clr	r31		\n\t"
			"ijmp	\n\t"
			);
//	asm volatile ( "push r1" "\n\t"		// Jump to Reset vector in Application Section
//					"push r1" "\n\t"
//					"ret"	 "\n\t"
//					::);
	while(1);
}


// 0 success, 1 failure
unsigned char getmessage(unsigned char *buffer,unsigned short *size,unsigned char *seq)
{
	unsigned char	checksum = 0;
	unsigned char c;
	
	// Blocking 
	serial_setblocking(file_usb,1);
	
	// Get start byte
	c=getchar();
	fprintf(file_bt,"Read %02X\n",c);
	if(c!=MESSAGE_START)
		return 1;
	checksum = MESSAGE_START^0;
	fprintf(file_bt,"Got start\n");
	
	// Get sequence number
	c=getchar();
	fprintf(file_bt,"Read %02X\n",c);
	*seq = c;
	checksum ^= c;
	fprintf(file_bt,"Got seq: %u\n",c);
	
	// Get size
	c=getchar();
	fprintf(file_bt,"Read %02X\n",c);
	*size = c<<8;
	checksum ^= c;
	c=getchar();
	fprintf(file_bt,"Read %02X\n",c);
	*size = *size|c;
	checksum ^= c;
	
	fprintf(file_bt,"Got size: %u\n",*size);

	// Get token
	c=getchar();
	fprintf(file_bt,"Read %02X\n",c);
	if(c!=TOKEN)
		return 2;
	checksum ^= c;
	fprintf(file_bt,"Got token\n");
	
	for(unsigned i=0;i<*size;i++)
	{
		c=getchar();
		fprintf(file_bt,"Data %02X\n",c);
		buffer[i]=c;
		checksum ^= c;
	}
	
	c=getchar();
	fprintf(file_bt,"Got checksum: %02X against %02X\n",c,checksum);
	
	if(c==checksum)
		return 0;
	
	
	return 3;
}
// Always succeed; returns the next sequence number (seq+1)
unsigned char sendmessage(unsigned char *buffer, unsigned short size, unsigned char seq)
{
	unsigned char checksum;
	unsigned char c;
	
	sendchar(MESSAGE_START);
	checksum	=	MESSAGE_START^0;
	
	sendchar(seq);
	checksum	^=	seq;
	
	c			=	((size>>8)&0xFF);
	sendchar(c);
	checksum	^=	c;
	
	c			=	size&0x00FF;
	sendchar(c);
	checksum ^= c;
	
	sendchar(TOKEN);
	checksum ^= TOKEN;
	
	for(unsigned short i=0;i<size;i++)
	{
		c=buffer[i];
		sendchar(c);
		checksum ^=c;
	}
	sendchar(checksum);
	return seq++;
}

void stk500_int(void)
{
	unsigned char	buffer[285];
	unsigned short size;
	unsigned char seq;
	unsigned char ctr=0;
	unsigned char leave=0;
	
	fprintf(file_bt,"stk500\n");
	delay_ms(100);

start:
	// Wait for a character
	
	while(ctr<50)
	{
		if(fgetrxbuflevel(file_usb))
			break;
		_delay_ms(100);
		system_led_toggle(0b1);
		ctr++;
	}
	// Check if we timed out (no character received)
	/*if(!fgetrxbuflevel(file_usb))
	{
		jumpusr();
	}*/
	if(!fgetrxbuflevel(file_usb))
	{
		fprintf(file_bt,"Timed out\n");
		return;
	}
	/*serial_setblocking(file_usb,1);
	for(int i=0;i<fgetrxbuflevel(file_usb);i++)
	{
		int c = getchar();
		fprintf(file_usb,"Got %02X\n",c);
	}*/
	// get message
	unsigned char rv = getmessage(buffer,&size,&seq);
	fprintf(file_bt,"get message returned: %d. size: %d. seq: %d. command. %02x\n",rv,size,seq,buffer[0]);
	if(rv!=0)
	{
		fprintf(file_bt,"msg rcv failed: %d\n",rv);
		return;
	}
				
	switch(buffer[0])
	{
		case CMD_SIGN_ON:
			fprintf(file_bt,"CMD_SIGN_ON\n");
			size = 11;
			buffer[1] 	=	STATUS_CMD_OK;
			buffer[2] 	=	8;
			buffer[3] 	=	'A';
			buffer[4] 	=	'V';
			buffer[5] 	=	'R';
			buffer[6] 	=	'I';
			buffer[7] 	=	'S';
			buffer[8] 	=	'P';
			buffer[9] 	=	'_';
			buffer[10]	=	'2';
			break;		
		case CMD_SPI_MULTI:
			{
				unsigned char answerByte;
								
				fprintf(file_bt,"CMD_SPI_MULTI. numtx: %d. numrx: %d. rxstart: %d. txdata: ",buffer[1],buffer[2],buffer[3]);
				for(unsigned short i=0;i<size-4;i++)
					fprintf(file_bt,"%02x ",buffer[4+i]);
				fprintf(file_bt,"\n");
	
				if(buffer[4]==0x30)
				{
					unsigned char signatureIndex	=	buffer[6];	
					if(signatureIndex == 0)
					{
						answerByte	=	(SIGNATURE_BYTES >> 16) & 0x000000FF;
					}
					else if ( signatureIndex == 1 )
					{
						answerByte	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
					}
					else
					{
						answerByte	=	SIGNATURE_BYTES & 0x000000FF;
					}
				}
				else if(buffer[4]==0x50 && buffer[5]==0x00)
				{
					answerByte	=	boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
				}
				else if(buffer[4]==0x58 && buffer[5]==0x08)
				{
					answerByte	=	boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
				}
				else if(buffer[4]==0x50 && buffer[5]==0x08)
				{
					answerByte	=	boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
				}
				else
					answerByte = 0;
				
				size		=	7;
				buffer[1]	=	STATUS_CMD_OK;
				buffer[2]	=	0;
				buffer[3]	=	buffer[4];
				buffer[4]	=	0;
				buffer[5]	=	answerByte;
				buffer[6]	=	STATUS_CMD_OK;
	
			}
			break;
		case CMD_GET_PARAMETER:
			{
				unsigned char value;

				fprintf(file_bt,"CMD_GET_PARAMETER. Param: %d\n",buffer[1]);
				switch(buffer[1])
				{
					case PARAM_BUILD_NUMBER_LOW:
						value	=	CONFIG_PARAM_BUILD_NUMBER_LOW;
						break;
					case PARAM_BUILD_NUMBER_HIGH:
						value	=	CONFIG_PARAM_BUILD_NUMBER_HIGH;
						break;
					case PARAM_HW_VER:
						value	=	CONFIG_PARAM_HW_VER;
						break;
					case PARAM_SW_MAJOR:
						value	=	CONFIG_PARAM_SW_MAJOR;
						break;
					case PARAM_SW_MINOR:
						value	=	CONFIG_PARAM_SW_MINOR;
						break;
					case PARAM_VTARGET:
					{
						#ifdef ENABLE_TRUEVTARGET
							unsigned short bv = ADC_read(7,1);
							unsigned short bv2= Battery_ADC2mv(bv);
							value = bv2/100;
						#else
							value=245;
						#endif
						break;
					}
					default:
						value	=	0;
						break;
				}
				size = 3;
				buffer[1] = STATUS_CMD_OK;
				buffer[2] = value;
			}
			break;
		case CMD_SET_PARAMETER:
		case CMD_ENTER_PROGMODE_ISP:
		case CMD_LEAVE_PROGMODE_ISP:
		default:
			size = 2;
			buffer[1]	=	STATUS_CMD_OK;
			if(buffer[0]==CMD_LEAVE_PROGMODE_ISP)
				leave=1;
			break;
	}
	fprintf(file_bt,"Sending answer\n");
	seq = sendmessage(buffer,size,seq);
	if(!leave)
		goto start;
	fprintf(file_bt,"Leaving ISP mode\n");
}

unsigned char echodbg2bt(unsigned char c)
{
	fputc(c,file_bt);
	return 1;
}

void stk500(void)
{
	cli();
	dbg_rx_callback=echodbg2bt;
	sei();
	
	while(1)
		stk500_int();
	
	cli();
	dbg_rx_callback=0;
	sei();
	
}


//*****************************************************************************
int bootmain(void)
{
	address_t		address			=	0;
	address_t		eraseAddress	=	0;
	unsigned char	msgParseState;
	unsigned int	ii				=	0;
	unsigned char	checksum		=	0;
	unsigned char	seqNum			=	0;
	unsigned int	msgLength		=	0;
	unsigned char	msgBuffer[285];
	unsigned char	c, *p;
	unsigned char   isLeave = 0;

	unsigned long	boot_timeout;
	unsigned long	boot_timer;
	unsigned int	boot_state;


	//*	some chips dont set the stack properly
	/*asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );*/

/*#ifdef _FIX_ISSUE_181_
	//
	//	Dec 29,	2011	<MLS> Issue #181, added watch dog timmer support
	//	handle the watch dog timer
	uint8_t	mcuStatusReg;
	mcuStatusReg	=	MCUSR;

	__asm__ __volatile__ ("cli");
	__asm__ __volatile__ ("wdr");
	MCUSR	=	0;
	WDTCSR	|=	_BV(WDCE) | _BV(WDE);
	WDTCSR	=	0;
	__asm__ __volatile__ ("sei");
	// check if WDT generated the reset, if so, go straight to app
	if (mcuStatusReg & _BV(WDRF))
	{
		app_start();
	}

#endif
*/

	boot_timer	=	0;
	boot_state	=	0;

#ifdef BLINK_LED_WHILE_WAITING
//	boot_timeout	=	 90000;		//*	should be about 4 seconds
//	boot_timeout	=	170000;
	boot_timeout	=	 20000;		//*	should be about 1 second
#else
	boot_timeout	=	3500000; // 7 seconds , approx 2us per step when optimize "s"
#endif
	
	


	delay_ms(500);
	sendchar('s');
	sendchar('t');
	sendchar('k');
	sendchar('5');
	sendchar('0');
	sendchar('0');
	sendchar('v');
	sendchar('2');
	sendchar(0x0d);
	sendchar(0x0a);

	delay_ms(100);

	// Wait for a character
	unsigned char ctr=0;
	while(ctr<50)
	{
		if(fgetrxbuflevel(file_usb))
			break;
		_delay_ms(100);
		system_led_toggle(0b1);
		ctr++;
	}
	// Check if we timed out (no character received)
	if(!fgetrxbuflevel(file_usb))
	{
		jumpusr();
	}
	

	
	//*	main loop
	while (!isLeave)
	{
		/*
		 * Collect received bytes to a complete message
		 */
		msgParseState	=	ST_START;
		while ( msgParseState != ST_PROCESS )
		{
			if (boot_state==1)
			{
				boot_state	=	0;
				c			=	getchar();
			}
			else
			{
			//	c	=	recchar();
				c	=	recchar_timeout();
				
			}

		
			}	//	switch
		}	//	while(msgParseState)

		/*
		 * Now process the STK500 commands, see Atmel Appnote AVR068
		 */

		switch (msgBuffer[0])
		{


			case CMD_READ_SIGNATURE_ISP:
				{
					unsigned char signatureIndex	=	msgBuffer[4];
					unsigned char signature;

					if ( signatureIndex == 0 )
						signature	=	(SIGNATURE_BYTES >>16) & 0x000000FF;
					else if ( signatureIndex == 1 )
						signature	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
					else
						signature	=	SIGNATURE_BYTES & 0x000000FF;

					msgLength		=	4;
					msgBuffer[1]	=	STATUS_CMD_OK;
					msgBuffer[2]	=	signature;
					msgBuffer[3]	=	STATUS_CMD_OK;
				}
				break;

			case CMD_READ_LOCK_ISP:
				msgLength		=	4;
				msgBuffer[1]	=	STATUS_CMD_OK;
				msgBuffer[2]	=	boot_lock_fuse_bits_get( GET_LOCK_BITS );
				msgBuffer[3]	=	STATUS_CMD_OK;
				break;

			case CMD_READ_FUSE_ISP:
				{
					unsigned char fuseBits;

					if ( msgBuffer[2] == 0x50 )
					{
						if ( msgBuffer[3] == 0x08 )
							fuseBits	=	boot_lock_fuse_bits_get( GET_EXTENDED_FUSE_BITS );
						else
							fuseBits	=	boot_lock_fuse_bits_get( GET_LOW_FUSE_BITS );
					}
					else
					{
						fuseBits	=	boot_lock_fuse_bits_get( GET_HIGH_FUSE_BITS );
					}
					msgLength		=	4;
					msgBuffer[1]	=	STATUS_CMD_OK;
					msgBuffer[2]	=	fuseBits;
					msgBuffer[3]	=	STATUS_CMD_OK;
				}
				break;

#ifndef REMOVE_PROGRAM_LOCK_BIT_SUPPORT
			case CMD_PROGRAM_LOCK_ISP:
				{
					unsigned char lockBits	=	msgBuffer[4];

					lockBits	=	(~lockBits) & 0x3C;	// mask BLBxx bits
					boot_lock_bits_set(lockBits);		// and program it
					boot_spm_busy_wait();

					msgLength		=	3;
					msgBuffer[1]	=	STATUS_CMD_OK;
					msgBuffer[2]	=	STATUS_CMD_OK;
				}
				break;
#endif
			case CMD_CHIP_ERASE_ISP:
				eraseAddress	=	0;
				msgLength		=	2;
			//	msgBuffer[1]	=	STATUS_CMD_OK;
				msgBuffer[1]	=	STATUS_CMD_FAILED;	//*	isue 543, return FAILED instead of OK
				break;

			case CMD_LOAD_ADDRESS:
#if defined(RAMPZ)
				address	=	( ((address_t)(msgBuffer[1])<<24)|((address_t)(msgBuffer[2])<<16)|((address_t)(msgBuffer[3])<<8)|(msgBuffer[4]) )<<1;
#else
				address	=	( ((msgBuffer[3])<<8)|(msgBuffer[4]) )<<1;		//convert word to byte address
#endif
				msgLength		=	2;
				msgBuffer[1]	=	STATUS_CMD_OK;
				break;

			case CMD_PROGRAM_FLASH_ISP:
			case CMD_PROGRAM_EEPROM_ISP:
				{
					unsigned int	size	=	((msgBuffer[1])<<8) | msgBuffer[2];
					unsigned char	*p	=	msgBuffer+10;
					unsigned int	data;
					unsigned char	highByte, lowByte;
					address_t		tempaddress	=	address;


					if ( msgBuffer[0] == CMD_PROGRAM_FLASH_ISP )
					{
						// erase only main section (bootloader protection)
						if (eraseAddress < APP_END )
						{
							boot_page_erase(eraseAddress);	// Perform page erase
							boot_spm_busy_wait();		// Wait until the memory is erased.
							eraseAddress += SPM_PAGESIZE;	// point to next page to be erase
						}

						/* Write FLASH */
						do {
							lowByte		=	*p++;
							highByte 	=	*p++;

							data		=	(highByte << 8) | lowByte;
							boot_page_fill(address,data);

							address	=	address + 2;	// Select next word in memory
							size	-=	2;				// Reduce number of bytes to write by two
						} while (size);					// Loop until all bytes written

						boot_page_write(tempaddress);
						boot_spm_busy_wait();
						boot_rww_enable();				// Re-enable the RWW section
					}
					else
					{
						//*	issue 543, this should work, It has not been tested.
						uint16_t ii = address >> 1;
						/* write EEPROM */
						while (size) {
							eeprom_write_byte((uint8_t*)ii, *p++);
							address+=2;						// Select next EEPROM byte
							ii++;
							size--;
						}
					}
					msgLength		=	2;
					msgBuffer[1]	=	STATUS_CMD_OK;
				}
				break;

			case CMD_READ_FLASH_ISP:
			case CMD_READ_EEPROM_ISP:
				{
					unsigned int	size	=	((msgBuffer[1])<<8) | msgBuffer[2];
					unsigned char	*p		=	msgBuffer+1;
					msgLength				=	size+3;

					*p++	=	STATUS_CMD_OK;
					if (msgBuffer[0] == CMD_READ_FLASH_ISP )
					{
						unsigned int data;

						// Read FLASH
						do {
					//#if defined(RAMPZ)
					#if (FLASHEND > 0x10000)
							data	=	pgm_read_word_far(address);
					#else
							data	=	pgm_read_word_near(address);
					#endif
							*p++	=	(unsigned char)data;		//LSB
							*p++	=	(unsigned char)(data >> 8);	//MSB
							address	+=	2;							// Select next word in memory
							size	-=	2;
						}while (size);
					}
					else
					{
						/* Read EEPROM */
						do {
							EEARL	=	address;			// Setup EEPROM address
							EEARH	=	((address >> 8));
							address++;					// Select next EEPROM byte
							EECR	|=	(1<<EERE);			// Read EEPROM
							*p++	=	EEDR;				// Send EEPROM data
							size--;
						} while (size);
					}
					*p++	=	STATUS_CMD_OK;
				}
				break;

			default:
				msgLength		=	2;
				msgBuffer[1]	=	STATUS_CMD_FAILED;
				break;
		}

		/*
		 * Now send answer message back
		 */
		sendchar(MESSAGE_START);
		checksum	=	MESSAGE_START^0;

		sendchar(seqNum);
		checksum	^=	seqNum;

		c			=	((msgLength>>8)&0xFF);
		sendchar(c);
		checksum	^=	c;

		c			=	msgLength&0x00FF;
		sendchar(c);
		checksum ^= c;

		sendchar(TOKEN);
		checksum ^= TOKEN;

		p	=	msgBuffer;
		while ( msgLength )
		{
			c	=	*p++;
			sendchar(c);
			checksum ^=c;
			msgLength--;
		}
		sendchar(checksum);
		seqNum++;



	
	
	
//	for(;;);
	return 0;
}

/*
base address = f800

avrdude: Device signature = 0x1e9703
avrdude: safemode: lfuse reads as FF
avrdude: safemode: hfuse reads as DA
avrdude: safemode: efuse reads as F5
avrdude>


base address = f000
avrdude: Device signature = 0x1e9703
avrdude: safemode: lfuse reads as FF
avrdude: safemode: hfuse reads as D8
avrdude: safemode: efuse reads as F5
avrdude>
*/

