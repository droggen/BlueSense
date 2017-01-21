#ifndef __SPI_USART0_H
#define __SPI_USART0_H

#define SPIUSART0_SELECT PORTA=(PORTA&0b11101111);
//#define SPIUSART0_DESELECT PORTA=(PORTA|0b00010000);
#define SPIUSART0_DESELECT PINA=0b00010000;

extern volatile unsigned char *_spiusart0_bufferptr;
extern volatile unsigned short _spiusart0_n;
extern volatile unsigned char _spiusart0_ongoing;
extern void (*_spiusart0_callback)(void);


void spiusart0_init(void);
void spiusart0_deinit(void);
void _spiusart0_waitavailandreserve(void);
unsigned char spiusart0_rw(unsigned char d);
void spiusart0_rwn(unsigned char *ptr,unsigned char n);
void spiusart0_rwn_int(unsigned char *ptr,unsigned char n);
unsigned char spiusart0_rwn_int_cb(unsigned char *ptr,unsigned char n,void (*cb)(void));
//unsigned char spiusart0_isongoing(void);

// Callbacks for interrupt-driven read with user callback
//unsigned char _mpu_i2c_trans_agt_select_cb(volatile struct _I2C_TRANSACTION *it);
//unsigned char _mpu_i2c_trans_read_agt_cb(volatile struct _I2C_TRANSACTION *it);



#endif