#ifndef __UFAT_H
#define __UFAT_H

typedef struct {
	unsigned char boot;
	unsigned char chsbegin[3];
	unsigned char type;
	unsigned char chsend[3];
	unsigned long lbabegin;
	unsigned long numsec;
} PARTITION;

typedef struct {
	unsigned char jump[3];
	char oem[8];
	unsigned short sectorsize;
	unsigned char sectperclust;
	unsigned short reservedsectors;
	unsigned char numfat;
	unsigned short numrootentries;
	unsigned short totsectors_short;
	unsigned char mediadescriptor;
	unsigned short sectorsperfat;
	unsigned short sectorpertrack;
	unsigned short numberheads;
	unsigned long numberhiddensectors;
	unsigned long totsectors_long;
	unsigned long fat32_sectorsperfat;
	unsigned short fat32_flags;
	unsigned short fat32_versionnumber;
	unsigned long fat32_rootcluster;
	unsigned short fat32_fsinfosector;
	unsigned short fat32_bootbackupsector;
	unsigned char fat32_reserved[12];
	unsigned char drivenumber;
	unsigned char head;
	unsigned char bootsignature;
	unsigned long volid;
	char vollabel[11];
	char fstype[8];
} BOOTSECT_FAT32;

typedef struct {
	// Data about card. This data is obtained from the low-level SD card interface
	unsigned long card_capacity_sector;					// Card capacity as reported by the SD-card
	// Data about partition. This data is obtained from the boot sector of the partition.
	unsigned long partition_capacity_sector;			// Size of the partition in sectors, including boot, fat, root, data
	unsigned long partition_sector;						// This is the address of the BOOT sector of the first partition
	unsigned long fat_sector;							// Sector containing the FAT1
	unsigned long fat2_sector;							// Sector containing the FAT2 or 0 if there is no FAT2
	unsigned long cluster_begin;						// Sector where the first cluster is located; the first cluster is cluster 2 (cluster 0,1 don't exist). The fat however has clusters 0,1 in its entries, i.e. the first FAT sector contains cluster 0-127
	unsigned char sectors_per_cluster;					// Number of sectors per cluster
	unsigned long root_cluster;							// Location of the root cluster
	unsigned long numclusters;
	// Data about logs. This information is hidden in last entry of root.
	unsigned char lognum;
	unsigned long logstartcluster;
	unsigned long logsizecluster;
	unsigned long logsizebytes;
	// Availability
	unsigned char fs_available;							// Indicates that the card is formatted with the uFAT and therefore files can be written
	unsigned char card_available;						// Indicates that low-level card initialisation is successful: an sd-card is available
} FSINFO;

typedef struct {
	char name[8];
	char ext[3];
	unsigned char attrib;
	unsigned char ntres;
	unsigned char createtimetenth;
	unsigned short createtime;
	unsigned short createdate;
	unsigned short accessdate;
	unsigned short clusterhi;
	unsigned short writetime;
	unsigned short writedate;	
	unsigned short clusterlo;
	unsigned long size;
} FILEENTRYRAW;
typedef struct {
	char name[8];
	char ext[3];
	unsigned char attrib;
	unsigned long cluster;
	unsigned long size;
} FILEENTRY;

typedef struct {
	char name[8];
	char ext[3];
	unsigned long startsector,startcluster;
	//unsigned long cursector;
	unsigned long size;
	//unsigned char used;
} LOGENTRY;

// Start location of the partition; there is no fixed rule defining where it should start except after the MBR. 
#define _UFAT_PARTITIONSTART 8192											

extern FSINFO _fsinfo;														// Summary of key info here
extern char ufatblock[];

unsigned long _ufat_getfilentrycluster(FILEENTRYRAW *fer);

void _ufat_getpart(char *block,unsigned char pn,PARTITION *p);
unsigned char _ufat_format_mbr_boot(void);

//unsigned char ufat_format_alllinkedfat(unsigned long fat_sector,unsigned long firstcluster,unsigned long totclusters);
unsigned char _ufat_format_fat_log(unsigned char i,unsigned long fat_sector);
unsigned char _ufat_write_root(unsigned char numlogfile);

unsigned char _ufat_format_fat_root(unsigned long fat_sector);
unsigned char _ufat_mbr_boot_read(void);

unsigned char ufat_format(unsigned char numlogfile);
unsigned char ufat_mbr_boot_init(void);
unsigned char _ufat_init_sd(void);
unsigned char _ufat_init_fs(void);
void _ufat_bs2keyinfo(BOOTSECT_FAT32 *bs,FSINFO *fi);
void ufat_print_boot(FILE *f,BOOTSECT_FAT32 *bs);
void ufat_print_fsinfo(FILE *f,FSINFO *fi);
void ufat_print_loginfo(FILE *f);
void ufat_fileentryraw2logentry(FILEENTRYRAW *fer,LOGENTRY *le);
//void ufat_fileentryraw2fileentry(FILEENTRYRAW *fer,FILEENTRY *fe);


unsigned char ufat_init(void);
unsigned char ufat_available(void);
FILE *ufat_log_open(unsigned char n);
int _ufat_log_fputchar(char c,FILE *f);
unsigned char _ufat_log_fputbuf(char *buffer,unsigned char size);
unsigned char ufat_log_close(void);
void ufat_log_test(unsigned char lognum,unsigned long size,unsigned char ch,unsigned bsize);
void log_printstatus(void);
unsigned long ufat_log_getmaxsize(void);
unsigned long ufat_log_getsize(void);

#endif