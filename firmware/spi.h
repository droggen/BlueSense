#ifndef __SPI_H
#define __SPI_H

#define SPI_SELECT PORTB=(PORTB&0b11101111);
#define SPI_DESELECT PINB=0b00010000;

#define SPI_DIV_128 3
#define SPI_DIV_64 2
#define SPI_DIV_32 6
#define SPI_DIV_16 1
#define SPI_DIV_8 5
#define SPI_DIV_4 0
#define SPI_DIV_2 4

//#define SPI_DIV SPI_DIV_128
#define SPI_DIV SPI_DIV_2



void spi_init(unsigned char spidiv);
unsigned char spi_rw(char d);
unsigned char spi_rw_noselect(char data);
void spi_rwn(char *ptr,unsigned char n);
void spi_rwn_noselect(char *ptr,unsigned short n);
void spi_rwn_int(char *ptr,unsigned char n);
void spi_wn_noselect(char *ptr,unsigned short n);


#endif