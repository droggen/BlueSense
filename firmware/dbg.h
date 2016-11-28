#ifndef __DBG_H
#define __DBG_H
#include "circbuf.h"
#include "i2c_internal.h"

//#define DBG_BUFFER_SIZE 32
//#define DBG_BUFFER_SIZE 16
//#define DBG_BUFFER_SIZE 256
#define DBG_BUFFER_SIZE 512
//#define DBG_BUFFER_SIZE 1024
#define DBG_ADDRESS 0x22
#define DBG_MAXPAYLOAD 16

extern volatile unsigned long dbg_tot_tx,dbg_tot_rx;

extern volatile CIRCULARBUFFER _dbg_tx_state,_dbg_rx_state;

extern volatile unsigned char dbg_rxlevel;

// Callbacks for hooking into the interrupt routines.
extern unsigned char  (*dbg_rx_callback)(unsigned char);

void dbg_init(void);
void dbg_clearbuffers(void);
void dbg_setnumtxbeforerx(unsigned char c);
int dbg_fputchar(char c, FILE*stream);
int dbg_fputchar_nonblock(char c, FILE*stream);
int dbg_fgetchar(FILE *stream);
int dbg_fgetchar_nonblock(FILE *stream);
unsigned char dbg_putbuf(unsigned char *data,unsigned char n);
CIRCULARBUFFER *dbg_get_rxbuf(void);
CIRCULARBUFFER *dbg_get_txbuf(void);

unsigned char dbg_callback(unsigned char p);
unsigned char dbg_callback_rx(unsigned char p);
unsigned char dbg_callback(unsigned char p);
void dbg_bench(void);
void dbg_setioparam(unsigned char period,unsigned char txbeforerx);


// Internal
unsigned char _dbg_read_callback(I2C_TRANSACTION *t);
unsigned char _dbg_query_callback1(I2C_TRANSACTION *t);
unsigned char _dbg_query_callback2(I2C_TRANSACTION *t);
unsigned char _dbg_write_callback(I2C_TRANSACTION *t);



#endif