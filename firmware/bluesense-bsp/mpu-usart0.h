#ifndef __MPU_USART0
#define __MPU_USART0

extern unsigned char *_mpu_tmp_reg;
extern volatile unsigned char _mpu_ongoing;				

void mpu_writereg(unsigned char reg,unsigned char v);
unsigned char mpu_readreg(unsigned char reg);
unsigned short mpu_readreg16(unsigned char reg);
void mpu_readregs(unsigned char *d,unsigned char reg,unsigned char n);
void mpu_readregs_int(unsigned char *d,unsigned char reg,unsigned char n);
unsigned char mpu_readregs_int_try_raw(unsigned char *d,unsigned char reg,unsigned char n);
unsigned char mpu_readregs_int_cb(unsigned char *d,unsigned char reg,unsigned char n,void (*cb)(void));
unsigned char mpu_readregs_int_cb_raw(unsigned char reg,unsigned char n,void (*cb)(void));
void __mpu_readregs_int_cb_cb(void);


#endif