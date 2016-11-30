#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "i2c.h"
#include "i2c_internal.h"
#include "main.h"
#include "circbuf.h"
#include "helper.h"
#include "dbg.h"
#include "system.h"
#include "wait.h"
#include "serial.h"
#include "serial1.h"
#include "adc.h"

//#define DBG_DBG

CIRCULARBUFFER _dbg_tx_state,_dbg_rx_state;
unsigned char _dbg_tx_buffer[DBG_BUFFER_SIZE];
unsigned char _dbg_rx_buffer[DBG_BUFFER_SIZE];
unsigned char _dbg_rx_readstate=0;

volatile unsigned long dbg_tot_tx=0,dbg_tot_rx=0;

I2C_TRANSACTION _dbg_trans_tx,_dbg_trans_query1,_dbg_trans_query2,_dbg_trans_read;

volatile unsigned char dbg_rxlevel=0;
volatile unsigned char dbg_general_state=0;
volatile unsigned char dbg_general_busy=0;
unsigned char _dbg_numtxbeforerx=5;			// 5: 13743 bytes/sec @1024Hz	6880 bytes/sec @512Hz
//unsigned char _dbg_numtxbeforerx=1;		// 1: 8254 bytes/sec @1024Hz	4127 bytes/sec @512Hz
//unsigned char _dbg_numtxbeforerx=10;	// 10: 14894 bytes/sec @1024Hz	7447 bytes/sec @512Hz
unsigned char _dbg_newnumtxbeforerx=0;


/*
	State 0: send data if any
	State 1: send query data
	State 2: receive data
*/

void dbg_init(void)
{
	_dbg_tx_state.buffer = _dbg_tx_buffer;
	_dbg_tx_state.size = DBG_BUFFER_SIZE;
	_dbg_tx_state.mask = DBG_BUFFER_SIZE-1;
	
	_dbg_rx_state.buffer = _dbg_rx_buffer;
	_dbg_rx_state.size = DBG_BUFFER_SIZE;
	_dbg_rx_state.mask = DBG_BUFFER_SIZE-1;
	
	dbg_general_state=0;
	
	dbg_clearbuffers();
	
	// Transmit data transaction
	i2c_transaction_setup(&_dbg_trans_tx,DBG_ADDRESS,I2C_WRITE,1,0);
	_dbg_trans_tx.callback = _dbg_write_callback;
	// Query data transaction select register
	i2c_transaction_setup(&_dbg_trans_query1,0,I2C_WRITE,0,1);				// No stop bit: dataheet indicates the next read transaction must use a repeat start
	_dbg_trans_query1.data[0]=0x0C;
	_dbg_trans_query1.callback = _dbg_query_callback1;
	// Query data transaction read register
	i2c_transaction_setup(&_dbg_trans_query2,DBG_ADDRESS,I2C_READ,1,1);
	_dbg_trans_query2.callback=_dbg_query_callback2;
	// Read data transaction
	i2c_transaction_setup(&_dbg_trans_read,DBG_ADDRESS,I2C_READ,1,0);
	_dbg_trans_read.callback=_dbg_read_callback;
}
void dbg_clearbuffers(void)
{
	buffer_clear(&_dbg_tx_state);
	buffer_clear(&_dbg_rx_state);
}
void dbg_setnumtxbeforerx(unsigned char c)
{
	if(c<1)
		c=1;
	if(c>128)
		c=128;
	_dbg_newnumtxbeforerx=c;
}
int dbg_fputchar(char c, FILE*stream)
{
	// If not connected: don't fill buffer
	if(!system_isusbconnected())
		return 0;
	
	// Wait until send buffer is free. 
	while( buffer_isfull(&_dbg_tx_state) );
	// Store the character in the buffer
	buffer_put(&_dbg_tx_state,c);
	
	return 0;
}
int dbg_fputchar_nonblock(char c, FILE*stream)
{
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	//{
		// Wait until send buffer is free. 
		if(buffer_isfull(&_dbg_tx_state))
			return 1;
		// Store the character in the buffer
		buffer_put(&_dbg_tx_state,c);
		return 0;
	//}
}

int dbg_fgetchar(FILE *stream)
{
	int c;
	do{ c=dbg_fgetchar_nonblock(stream);}
	while(c==EOF && serial_isblocking(stream));
	return c;	
}
int dbg_fgetchar_nonblock(FILE *stream)
{
	char c;
	if(buffer_isempty(&_dbg_rx_state))
		return EOF;
	c = buffer_get(&_dbg_rx_state);
	return ((int)c)&0xff;
}
CIRCULARBUFFER *dbg_get_rxbuf(void)
{
	return &_dbg_rx_state;
}
CIRCULARBUFFER *dbg_get_txbuf(void)
{
	return &_dbg_tx_state;
}


/******************************************************************************
	function: dbg_putbuf
*******************************************************************************	
	Atomically writes a buffer to a stream, or fails if the buffer is full.
			
	Returns:	
		zero		-	success
		nonzero		-	error

******************************************************************************/
unsigned char dbg_putbuf(char *data,unsigned char n)
{
	//if(buffer_level(&_dbg_tx_state)<n)
	// If not connected: don't fill buffer
	if(system_isusbconnected() && buffer_level(&_dbg_tx_state)<n)
	{
		for(unsigned char i=0;i<n;i++)
			buffer_put(&_dbg_tx_state,data[i]);
		return 0;
	}
	return 1;
}

/******************************************************************************
	dbg_rx_callback
*******************************************************************************	
	Pointer to a callback function called everytime a character is received.
	This callback must return 1 to place the character in the receive queue, 
	0 if the character must be discarded (e.g. because processed otherwise).
		
	If the pointer is null, no callback is called.
******************************************************************************/
unsigned char  (*dbg_rx_callback)(unsigned char)=0;



unsigned char dbg_callback(unsigned char p)
{
	unsigned char r;
	unsigned short lvl;
	//static unsigned char ctr=0;
	//char b[32];
	
	// If not connected we don't run the callback at all
	//return 0;
	if(!system_isusbconnected())
		return 0;
		
	
	//system_led_toggle(0b1);
	//_delay_us(20);
	/*if( (ctr&0x1f)==0)
	{
		b[0]='c'; 
		b[1]=hex2chr((dbg_general_state>>0)&0xf);
		b[2]=' ';
		b[3]=hex2chr((dbg_tot_tx>>8)&0xf);
		b[4]=hex2chr((dbg_tot_tx>>4)&0xf);
		b[5]=hex2chr((dbg_tot_tx>>0)&0xf);
		b[6]=' ';
		b[7]=hex2chr((dbg_tot_rx>>8)&0xf);
		b[8]=hex2chr((dbg_tot_rx>>4)&0xf);
		b[9]=hex2chr((dbg_tot_rx>>0)&0xf);
		b[10]=' ';
		b[11]=hex2chr((ctr>>4)&0xf);
		b[12]=hex2chr((ctr>>0)&0xf);
		b[13]='\n';
		b[14]=0;
		fputs(b,file_bt);
	}
	else
	{
		//b[0]='c'; 
		//b[1]=hex2chr((dbg_general_state>>0)&0xf);
		//b[2]='\n';
		//b[3]=0;
		//fputs(b,file_bt);
	}
	ctr++;*/
	
	// If busy, wait for transaction to finish - can happen with low I2C speed and high callback speed
	if(dbg_general_busy)
		return 0;
	//system_led_toggle(0b10);
	//fprintf(file_bt,"s %d\n",dbg_general_state);
	
	// Send data
	if(dbg_general_state<_dbg_numtxbeforerx)
	{
		
		// Update the number of tx before rx, in case this changed
		if(dbg_general_state==0 && _dbg_newnumtxbeforerx!=0)
		{
			_dbg_numtxbeforerx=_dbg_newnumtxbeforerx;
			_dbg_newnumtxbeforerx=0;
		}
		lvl = buffer_level(&_dbg_tx_state);
		/*b[0]='l';b[1]='v';b[2]='l';b[3]=hex2chr(lvl>>4);b[4]=hex2chr(lvl&0xf);b[5]='\n';
		uart1_fputbuf_int(b,6);*/
		if(lvl==0)
		{
			// Two possibilities: 1) if nothing to send go to inquire state; 2) if nothing to send go to next state (tx or inquire). 
			// Option 1 puts higher load on CPU.
			
			// 1) If nothing to send, then go to inquire state.
			dbg_general_state = _dbg_numtxbeforerx;
			goto inquire_state;
			// 2) if nothing to send go to next state (tx or inquire). 
			//dbg_general_state++;
			//return 0;
		}
		
		// Clamp max write to DBG_MAXPAYLOAD (<I2C buffer)
		if(lvl>DBG_MAXPAYLOAD)
			lvl=DBG_MAXPAYLOAD;	
				
		// Setup write transaction
		_dbg_trans_tx.dodata = lvl;
		for(unsigned char i=0;i<lvl;i++)
			_dbg_trans_tx.data[i]=buffer_get(&_dbg_tx_state);	
		//uart1_fputbuf_int("strttrns\n",9);
		r = i2c_transaction_queue(1,0,&_dbg_trans_tx);
		if(r)
		{
			//uart1_fputbuf_int("strttrns fail\n",14);
			return 2;
		}	
		//uart1_fputbuf_int("strttrns suc\n",13);
		dbg_tot_tx+=lvl;
		dbg_general_busy=1;
		return 0;
	}
	inquire_state:
	// Inquire available data
	if(dbg_general_state==_dbg_numtxbeforerx)
	{
		// Should only inquire if dbg_rxlevel is <DBG_MAXPAYLOAD, otherwise we know we have enough data that can be read without inquiring to fill a complete I2C transaction; this speeds up receiving data
		//if(dbg_rxlevel<DBG_MAXPAYLOAD)
		//{	
			// If there is no space in rx buffer, no need to inquire space, and instead go to send
			lvl = buffer_freespace(&_dbg_rx_state);
			if(lvl==0)
			{
				dbg_general_state=0;
				return 0;
			}			
			
			r = i2c_transaction_queue(2,0,&_dbg_trans_query1,&_dbg_trans_query2);
			if(r)
			{
				// Does not change state, i.e. retries
				return 2;
			}
			dbg_general_busy=1;
			return 0;
		//}
		//else
		//{
			// Go to next state which is read
		//	dbg_general_state++;
		//}
		
	}
	if(dbg_general_state>_dbg_numtxbeforerx)
	{
		/*if(dbg_rxlevel==0)
			system_led_set(0b000);
		else
			system_led_set(0b100);*/
	
	
		lvl = buffer_freespace(&_dbg_rx_state);
		
		/*char b[32];
		b[0]='x'; 
		b[1]=hex2chr((rxs>>8)&0xf); b[2]=hex2chr((rxs>>4)&0xf); b[3]=hex2chr(rxs&0x0f); 
		b[4]='\n';
		b[5]=0;		
		fputs(b,file_bt);*/
		
		unsigned char n;
		// Read available data
		if(dbg_rxlevel==0 || lvl==0)
		{
			// Goes to send state if no data to read, or no space to store
			// If the inquire state is modified to inquire only of receive buffer has space, then the test for 
			// rx buffer space here could be avoided
			dbg_general_state=0;
			return 0;
		}
		
		if(dbg_rxlevel>DBG_MAXPAYLOAD)
		{
			n=DBG_MAXPAYLOAD;			
		}
		else
		{
			n=dbg_rxlevel;
		}
		// Clamp by the amount of space in the rx buffer. Better leave data unread in the FTDI, that will signal to the host to stop transmitting, otherwise data will be lost
		if(n>lvl)
			n=lvl;
			
		/*char b[32];
		b[0]='x'; 
		b[1]=hex2chr(dbg_rxlevel>>4); b[2]=hex2chr(dbg_rxlevel&0x0f); 
		b[3]=hex2chr((rxs>>8)&0xf); b[4]=hex2chr((rxs>>4)&0xf); b[5]=hex2chr(rxs&0x0f); 
		b[6]=hex2chr(n>>4); b[7]=hex2chr(n&0x0f); 
		b[8]='\n'; b[9]=0;
		fputs(b,file_bt);*/
			
		dbg_rxlevel-=n;
		
			
		_dbg_trans_read.dodata = n;
		r = i2c_transaction_queue(1,0,&_dbg_trans_read);
		if(r)
		{
			return 2;
		}
		dbg_tot_rx+=n;
		dbg_general_busy=1;
		return 0;
	}
	// Should not arrive here
	return 0;
}


unsigned char _dbg_write_callback(I2C_TRANSACTION *t)
{
	
	dbg_general_busy=0;
	if(t->status==0)
	{
		dbg_general_state++;	
	
		return 0;
	}
	// Error, possibly buffer full on FTDI (e.g. when USB disconnected) or other
	//system_blink(4,50,0b01);
	//uart1_fputbuf_int("wrcb\n",5);
	#ifdef DBG_DBG
	char b[32];
	b[0]='D'; b[1]='W'; b[2]=hex2chr(t->status>>4); b[3]=hex2chr(t->status&0x0f); b[4]=hex2chr(t->i2cerror>>4); b[5]=hex2chr(t->i2cerror&0x0f); b[6]='\n';
	uart1_fputbuf_int(b,6);
	#endif
	return 1;
}
unsigned char _dbg_query_callback1(I2C_TRANSACTION *t)
{
	// First transaction successful -> do nothing
	if(t->status==0)
		return 0;
	// First transaction failed -> not busy, go to initial state
	// The transactions were linked, so the transaction2 should be removed by the i2c engine 
	dbg_general_busy=0;
	dbg_general_state=0;	
	
	//system_blink(4,40,0b01);	
	#ifdef DBG_DBG
	// Print data 
	/*char b[32];
	b[0]='Q'; b[1]=hex2chr(t->status>>4); b[2]=hex2chr(t->status&0x0f); b[3]=hex2chr(t->i2cerror>>4); b[4]=hex2chr(t->i2cerror&0x0f); b[5]='\n'; b[6]=0;
	fputs(b,file_bt);*/
	fputc('Q',file_bt);
	#endif
	return 1;
}
unsigned char _dbg_query_callback2(I2C_TRANSACTION *t)
{
	// Second transaction
	if(t->status==0)
	{
		// Successful - copy data, go to initial state
		dbg_rxlevel = t->data[0];
		dbg_general_busy=0;
		if(dbg_rxlevel!=0)
		{
			//system_led_set(1);
			dbg_general_state++;		
			//char b[32];
			//b[0]='A'; b[1]=hex2chr(dbg_rxlevel>>4); b[2]=hex2chr(dbg_rxlevel&0x0f); 
			//b[3]='\n'; b[4]=0;
			//fputs(b,file_bt);
		}
		else
		{
			//system_led_set(0);
			dbg_general_state=0;
		}
		
		return 0;
	}
	// Second transaction failed
	//system_blink(4,30,0b01);	
	#ifdef DBG_DBG
	//char b[32];
	//b[0]='w'; b[1]=hex2chr(t->status>>4); b[2]=hex2chr(t->status&0x0f); b[3]=hex2chr(t->i2cerror>>4); b[4]=hex2chr(t->i2cerror&0x0f); b[5]='\n'; b[6]=0;
	//fputs(b,file_bt);
	fputc('w',file_bt);
	#endif
	
	dbg_general_busy=0;
	dbg_general_state=0;		
	return 1;
}
unsigned char _dbg_read_callback(I2C_TRANSACTION *t)
{
	// Copy data in receive buffer
	if(t->status==0)
	{
		for(unsigned char i=0;i<t->dodata;i++)
		{
			// The general callback is avoiding issuing a read transaction larger than the space available in the rx buffer, hence no needs to test for isfull here.
			//if(!buffer_isfull(&_dbg_rx_state))
			//{
				if(dbg_rx_callback==0 || (*dbg_rx_callback)(t->data[i])==1)
					buffer_put(&_dbg_rx_state,t->data[i]);
			//}
		}
		dbg_general_busy=0;
		// Check RX level
		if(dbg_rxlevel==0)
		{
			// Empty: go to write cycle
			dbg_general_state=0;	
		}
		else
		{
			//char b[32];
			//b[0]='r'; b[1]=hex2chr(dbg_rxlevel>>4); b[2]=hex2chr(dbg_rxlevel&0x0f); 
			//b[3]='\n'; b[4]=0;
			//fputs(b,file_bt);
			// Not empty: give priority to read and do another read cycle
			dbg_general_state=_dbg_numtxbeforerx+1;
		}
	}
	else
	{
		// Error
		//system_blink(4,20,0b01);	
		#ifdef DBG_DBG
		char b[32];
		b[0]='R'; b[1]=hex2chr(t->status>>4); b[2]=hex2chr(t->status&0x0f); b[3]=hex2chr(t->i2cerror>>4); b[4]=hex2chr(t->i2cerror&0x0f); b[5]='\n'; b[6]=0;
		fputs(b,file_bt);
		#endif
		// Return to write cycle
		dbg_general_busy=0;
		dbg_general_state=0;	
	}
	
	
	
	return 0;
}


/*
void dbg_bench(void)
{
	unsigned long int t1,t2;
	unsigned int n=1024;
	t1 = timer_ms_get();
	for(unsigned int i=0;i<n;i++)
	{
		//fprintf(file_usb,"%04d 6789012345\n",i);
		fprintf(file_usb,"123456789012345\n");
	}
	t2 = timer_ms_get();
	printf("Elapsed time: %ld ms\n",t2-t1);
	printf("bytes/sec: %ld\n",n*16l*1000l/(t2-t1));
}
*/

void dbg_setioparam(unsigned char period,unsigned char txbeforerx)
{
//	printf("txbeforerx: %d\n",txbeforerx);
	
	// Unregister callback and wait for last operation to complete
	timer_unregister_callback(dbg_callback);
	while(dbg_general_busy);
	// Reset state
	dbg_general_state=0;
	// Set new parameters and register callback
	dbg_setnumtxbeforerx(txbeforerx);
	timer_register_callback(dbg_callback,period);
}