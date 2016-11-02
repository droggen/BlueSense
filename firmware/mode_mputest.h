#ifndef __MODE_MPUTEST_H
#define __MODE_MPUTEST_H

unsigned char CommandParserMPUTest_ReadReg(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_ReadReg(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Start(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Stop(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Calibrate(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Reset(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Fifo(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_FifoEn(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Test(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Magn(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_MagnMode(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Interface(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_External(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Shadow(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Off(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Poll(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Auto(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Bench(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Quaternion(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_Quaternion2(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_MagneticCalib(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_MagneticSelfTest(unsigned char *buffer,unsigned char size);
unsigned char CommandParserMPUTest_BenchMath(unsigned char *buffer,unsigned char size);
void mode_mputest(void);

extern const char help_mt_r[];

#endif