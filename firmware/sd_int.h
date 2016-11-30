#ifndef __SD_INT_H
#define __SD_INT_H



typedef struct _CID {
	unsigned char MID;
	unsigned short OID;
	unsigned char PNM[6];
	unsigned char PRV;
	unsigned long PSN;
	unsigned short MDT;
} CID;

typedef struct _CSD {
																		// CSD1							CSD2 (x if not exist or new address range)
	unsigned char CSD;								// [127:126]
	unsigned char TAAC;								// [119:112]
	unsigned char NSAC;								// [111:104]
	unsigned char TRAN_SPEED;					// [103:96]
	unsigned short CCC;								// [95:84]
	unsigned char READ_BL_LEN;				// [83:80]
	unsigned char READ_BL_PARTIAL;		// [79:79]
	unsigned char WRITE_BLK_MISALIGN;	// [78:78]
	unsigned char READ_BLK_MISALIGN;	// [77:77]
	unsigned char DSR_IMP;						// [76:76]
	unsigned long C_SIZE;							// [73:62]					[69:48]
	unsigned char VDD_R_CURR_MIN;			// [61:59]					x
	unsigned char VDD_R_CURR_MAX;			// [58:56]					x
	unsigned char VDD_W_CURR_MIN;			// [55:53]					x
	unsigned char VDD_W_CURR_MAX;			// [52:50]					x
	unsigned char C_SIZE_MULT;				// [49:47]					x
	unsigned char ERASE_BLK_EN;				// [46:46]					
	unsigned char SECTOR_SIZE;				// [45:39]					
	unsigned char WP_GRP_SIZE;				// [38:32]				
	unsigned char WP_GRP_ENABLE;			// [31:31]
	unsigned char R2W_FACTOR;					// [28:26]					
	unsigned char WRITE_BL_LEN;				// [25:22]
	unsigned char WRITE_BL_PARTIAL;		// [21:21]
	unsigned char FILE_FORMAT_GRP;		// [15:15]
	unsigned char COPY;								// [14:14]
	unsigned char PERM_WRITE_PROTECT;	// [13:13]
	unsigned char TMP_WRITE_PROTECT;	// [12:12]
	unsigned char FILE_FORMAT;				// [11:10]
	unsigned char CRC;								// [7:1]
} CSD;

typedef struct _OCR {
	unsigned char busy;
	unsigned char CCS;
	unsigned char UHSII;
	unsigned char S18A;
	unsigned char v3536;
	unsigned char v3435;
	unsigned char v3334;
	unsigned char v3233;
	unsigned char v3132;
	unsigned char v3031;
	unsigned char v2930;
	unsigned char v2829;
	unsigned char v2728;
} OCR;



// Low-level
unsigned char _sd_command_rn_retry(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4,char *response,unsigned short n, unsigned char answermask,unsigned char okanswer);
unsigned char _sd_command_rn_retry_crc(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4, unsigned char crc,char *response,unsigned short n, unsigned char answermask,unsigned char okanswer);
unsigned char _sd_command_r1_retry(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4, unsigned char answermask,unsigned char okanswer,unsigned char *r1);
unsigned char _sd_command_r1_retry_crc(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4, unsigned char crc, unsigned char answermask,unsigned char okanswer,unsigned char *r1);

unsigned char _sd_command_rn(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4,unsigned char crc,char *response, unsigned short n);
unsigned char _sd_command_rn_ns(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4,unsigned char crc,char *response, unsigned short n);

unsigned char _sd_readbock_ns(char *buffer,unsigned short n,unsigned short *checksum);
unsigned char _sd_command_r1_readblock(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4,char *r1,char *block,unsigned short n,unsigned short *checksum);

// Commands
unsigned char _sd_cmd9(CSD *csd,unsigned long *capacity_sector);
unsigned char _sd_cmd10(CID *cid);
unsigned char _sd_cmd58(OCR *ocr);



// Internal block writes
unsigned char _sd_block_open(unsigned long addr);
unsigned char _sd_block_close(void);
unsigned char _sd_block_stop(void);
unsigned char _sd_block_stop_nowait(void);
unsigned char _sd_block_stop_dowait(void);
void _sd_writebuffer(char *buffer,unsigned short size);
void _sd_writeconst(unsigned char b,unsigned short size);



// Internal multiblock writes
unsigned char _sd_multiblock_open(unsigned long addr);
unsigned char _sd_multiblock_close(void);



#endif
