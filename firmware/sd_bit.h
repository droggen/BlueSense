#ifndef __SD_BIT_H
#define __SD_BIT_H

unsigned long _sd_bit2val(char *data,unsigned int startbit,unsigned int n);
// Internal functions
unsigned char _sd_crc7command(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4);
unsigned char _sd_crc7byte(unsigned char crc7,unsigned char d);
unsigned char _sd_crc7end(unsigned char crc7);


//unsigned short sd_crc16(unsigned short crc16,unsigned char d);
//unsigned short sd_crc16end(unsigned short crc16);


#endif