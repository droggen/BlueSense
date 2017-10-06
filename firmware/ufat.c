#include "cpu.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "wait.h"
#include "spi.h"
#include "main.h"
#include "sd.h"
#include "ufat.h"
#include "serial.h"
#include "helper.h"

/*
	File: ufat
	
	uFAT filesystem.
	
	This provides a minimalistic uFAT filesystem, compatible with FAT32, optimised for streaming writes and which can be read seamlessly from a card reader. 
	
	This library assumes a carefully laid out filesystem which has been designed to optimise streaming write speed and to minimise code usage. 
	The tradeoff of this optimisation are the following:
	
	* Only file write is possible
	* Only one file can be written to at any time
	* Seek/append operations are not implemented
	* The maximum number of files in the file system is limited to 14
	* Only legacy 8.3 file names are supported
	* Folders are not supported
	* File names are pre-defined and cannot be changed
	* The maximum size of files is the same for all files and pre-defined during formatting based on the total card capacity (i.e. MaxFileSize=CardCapacity/NumFiles)
	* The filesystem must only be formatted by this library; formatting on a computer will not result in a card that this library can use
	* Any file write, file rename or file move operation performed on a computer will lead to errors or data loss the next time that this library attempts to write to the card. 
	  There is no issue if the card is only used on a computer after such an operation. However, if the card were used again with this library a formatting would be required.
	* The first sector of the ROOT entry contains 16 directory entries. The entry 0 is the volume ID; entries 1 to n are the log files, entry n+1 to 14 are dummy files marked as erased to prevent the 
	  OS from modifying this; entry 15 is uFAT metadata, stored as a file marked as erased. Upon checking the filesystem a checksum is run on this metadata to ensure no other operating system
	  has tampered with them.
	* During formatting all the clusters that the log files will use when having the maximum size are marked as used (cluster linked, i.e. nonzero value). 
	  This prevents another OS from using clusters that uFAT needs as the log file grows.
	  Consequently the disk free space reported by a conventional OS will be constant and tiny (in the MB range), regardless of the size of the uFAT files.
	
	
	The filesystem is implemented with the following characteristics:
	
	* There is only one FAT; uFAT does not create the secondary FAT commonly created by traditional OSes
	* The size of the sectors is 512 bytes
	* The size of the clusters is 64 sectors (64*512=32KB)
	* The ROOT directory for the uFAT files is stored in exactly one sector; this limits the number of ROOT entries to a maximum of 16, and therefore the number of files to 14 (volumeID+14 log files+metadata stored as a fake file).
	* The first cluster of each files is allocated on a cluster that is the first cluster of a sector of the FAT (i.e. start location is a multiple of 128 clusters). This simplifies the FAT update.
	* Upon formatting, files are pre-allocated on consecutive clusters which avoid fragmentation. The FAT is programmed accordingly. As data is written to the file, only the file length needs to be updated. This avoids slow FAT updates.
	* The file size is updated only upon closing a file. This speeds-up streaming writes at the expense of lower reliability if the platform crashes before a file is closed.

	It is recommended to ensure another operating system never writes to a uFAT formatted sd-card. 
	Windows generally creates a "System Volume Information" and associated files when an SD-card is plugged in, without user intervention. 
	uFAT tweaks the FAT to minimise the risk of Windows writing to a location that uFAT would use (e.g. it marks the first 16 ROOT entries as used or deleted files; it marks all clusters that could be used by log files as used/linked). 
	This strategy appears to work but has not been extensively tested. An alternative is to prevent any OS write to the disk. 
	The following link indicates how to prevent Windows creating such folders/files on a removable media: http://www.thewindowsclub.com/prevent-system-volume-information-folder-usb
	


	*Public functions*
	
	
	* ufat_format:						Format a card with the uFAT filesystem. The filesystem is ready to be used immediately afterwards.
	* ufat_init:						Initialise the uFAT filesystem including low-level card initialisation and filesystem check.
	* ufat_available:					Indicates whether the system successfully detected a disk with uFAT.
	* ufat_log_open:					Opens the indicated log file for write operations using fprintf, fputc, fputbuf, etc
	* ufat_log_close:					Close the previously opened log file
	* ufat_log_test:					Test writing data to a log file
	* ufat_log_getmaxsize: 				Returns the maximum size of files in the given filesystem.
	* ufat_log_getsize: 				Returns the size of the currently open file.
	* ufat_log_getnumlogs:				Returns the number of logs available
	


	*Dependencies*
	
	* spi
	* Card on SPI interface: this library assumes the card is interfaced on the SPI interface. The SPI interface must be 
	initialised before using this library.
	
	
	
	
	
	*Usage in interrupts*
	
	Not suitable for use in interrupts.
	
	
	*Possible improvements*
	
	* The card is formatted with type 0x0a (FAT32). Some documentation indicate that this limits partitions to less than 2GB. While no issues have been encountered using type 0x0C could be tested.
	* Some SD cards are formatted without a partition table but start straight with a boot sector, as in old diskettes. 
	  Formatting the card in this way would reduce slightly the code size of this library.
	* When writing a log and updating the FAT we assume that FAT sectors contain only a cluster chain related to uFAT and all the unused clusters are reset to 0. If another
	  operating system would use these free clusters, writing a log would corrupt these files.
	* Further robustness testing is required if another operating system writes to the disk to check for possible corruption.
	
*/

/*
Documentation on FAT32 can be found here:
* https://www.pjrc.com/tech/8051/ide/fat32.html
* http://www.compuphase.com/mbr_fat.htm
* http://codeandlife.com/2012/04/02/simple-fat-and-sd-tutorial-part-1/

Disk structure
--------------

Sector 0: Master Boot Record
----------------------------
The sector 0 contains the Master Boot Record with boot code and a Partition Table with 4 16-bytes Partition Entry
+--------------------------------+
I                                I
I                                I
I                                I
I                                I
I                                I
I        Boot code               I
I                                I
I                                I
I                                I
I                                I
I                                I
I                            P1P1I
IP1P1P1P1P1P1P1P1P1P1P1P1P1P1P2P2I
IP2P2P2P2P2P2P2P2P2P2P2P2P2P2P3P3I
IP3P3P3P3P3P3P3P3P3P3P3P3P3P3P4P4I
IP4P4P4P4P4P4P4P4P4P4P4P4P4P455AAI
+--------------------------------+

Partition Entry (16 bytes):
---------------------------

+--------------------------------+
IBfChsbegTcChsendLbabegnNumsectorI
+--------------------------------+
Bf: 		[1] boot flag
Chsbeg: 	[3] CHS begin
Tc: 		[1] type code
Chsend: 	[3] CHS end
Lbabegn: 	[4] LBA begin: indicates the sector where the Volume ID aka Boot Sector is located
Numsector: 	[4] Number of sectors


Partition/volume organisation
-----------------------------
A partition is organised as follows:
- Boot sector 						Size: 1 sector
									Location: _fsinfo.partition_sector=_UFAT_PARTITIONSTART=typically 8192
- Backup boot sector				Size: 1 sector
									Location: typically 6 sectors after boot sector (_UFAT_PARTITIONSTART+6)
- Reserved Sectors 					0x20 sectors in this implementation.
									Reserved sectors are from Boot sector to FAT1. 
									I.e.: sector 0 of partition is boot; sector 6 is backup boot, sector 0x20 is FAT1
- FAT1								The FAT size in sectors function of the capacity. In FAT32 one sector (512 bytes) can hold 512/4=128 clusters. 
                                    In this implementation FAT1 is in the sector immediately after the boot sector
									The FAT1 is at sector number _fsinfo.fat_sector
									In this implementation, the FAT entry corresponding to the ROOT folder is in the first entry of the first sector of the FAT.
									The FAT entry corresponding to the first log file is in the second sector of the FAT.
									
- [FAT2]							Optional second fat. The FAT2 is at sector number _fsinfo.fat2_sector
									In this implementation there is no second fat implemented
- ROOT/Cluster begin (cluster 2)	In this implementation, the first cluster is immediately after the FAT1. The cluster begin in sector number is in _fsinfo.cluster_begin. The first cluster is referred to as cluster 2 in FAT32.
                                    The first cluster contains the ROOT directory






Volume ID, aka Boot Sector (512 bytes):
---------------------------------------
0x000 	Intel 80x86 jump instruction 																												3
0x003 	OEM name (not the volume name, see offset 0x02B) 																							8
0x00B 	Sector size in bytes 																														2
0x00D 	Number of sectors per cluster 																												1
0x00E 	Reserved sectors (including the boot sector)																								2
0x010 	Number of FATs 																																1
0x011 	Number of directory entries in the root directory (N.A. for FAT32) 																			2
0x013 	Total number of sectors on the disk/partition, or zero for disks/partitions bigger than 32 MiB (the field at offset 0x020 should then be used) 	2
0x015 	Media descriptor (bit #2 holds whether the disk is removable) 																				1
0x016 	Number of sectors used for one FAT table (N.A. for FAT32) 																					2
0x018 	Number of sectors per track (cylinder), CHS addressing 																						2
0x01a 	Number of heads, CHS addressing 																											2
0x01c 	Number of hidden sectors, which is the number of sectors before the boot sector (this field may be set to zero, though) 					4
0x020 	Total number of sectors on the disk/partition, if this is above 32 MiB (only valid if the field at offset 0x013 is zero) 					4
0x024 	FAT32 only: Number sectors in the one FAT, replaces the field at offset 0x016 																4
0x028 	FAT32 only: Flags for FAT mirroring & active FAT 																							2
0x02a 	FAT32 only: File system version number 																										2
0x02c 	FAT32 only: Cluster number for the root directory, typically 2 																				4
0x030 	FAT32 only: Sector number for the _fsinfo structure, typically 1 																			2
0x032 	FAT32 only: Sector number for a backup copy of the boot sector, typically 6 																2
0x034 	FAT32 only: Reserved for future expansion 																									12
0x040 	Drive number (this field is at offset 0x024 in FAT12/FAT16) 																				1
0x041 	Current head (internal to DOS; this field is at offset 0x025 in FAT12/FAT16) 																1
0x042 	Boot signature, the value 0x29 indicates the three next fields are valid (this field is at offset 0x026 in FAT12/FAT16) 					1
0x043 	Volume ID (serial number; this field is at offset 0x027 in FAT12/FAT16) 																	4
0x047 	Volume label (this field is at offset 0x02b in FAT12/FAT16) 																				11
0x052 	File system type (this field is at offset 0x036 in FAT12/FAT16) 																			8
0x08a 	Boot code (starts at offset 0x03e in FAT12/FAT16, and is 448 bytes) 																		372
0x1fe 	Boot sector signature, must be 0x55AA 																										2 


FAT:
----
The FAT is a bitmap with 4 bytes per cluster indicating what is the next cluster of the file.
A FAT sector can hold 128 clusters. The first two entries corresponds to cluster 0 and 1, but cluster 0 and 1 do not exist, as the first cluster on the disk is cluster 2.



	Partition sector: used to indicate maximum number of log files (<=15) and their size
		- Offset 100: maximum number of log files [1 byte]
		- Offset 101-104: Size of log files [4 bytes]
*/

// Various
const char _str_ufat[] = "uFAT: ";

//#define UFATDBG

unsigned long _log_current_sector,_log_current_size;
unsigned char _log_current_log;
FILE _log_file;
SERIALPARAM _log_file_param;


#define _UFAT_NUMLOGENTRY 14								// Maximum number 14; 16 root entries=volid+logs+metadata
char ufatblock[512];								// Multiuse buffer
//unsigned char *ufatblock=_log_buffer[0];					// Multiuse buffer -> points to log buffer as no need for both at same time
LOGENTRY _logentries[_UFAT_NUMLOGENTRY];					// Predefined number of log entries
FSINFO _fsinfo;												// Summary of key info here
unsigned long _logoffsetcluster=128;						// Offset of the logging area from the cluster start (in cluster). 128 ensures the first cluster of the first log file is the first cluster of the second FAT sector

//unsigned long testfilesize=60000;
//unsigned long testfilecluster=3;

/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC 
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/


/******************************************************************************
	function: ufat_format
*******************************************************************************	
	Format a card with the uFAT filesystem. The filesystem is ready to be used immediately afterwards.
	
	Performs a low-level initialisation of the SD card, formats the card, and 
	initialises the internal data structures.
	
	After formatting there is no need to call ufat_init to initialise the filesystem.
	
	This function assumes that the SPI peripheral has been initialised.
	It performs low-level SD-card initialisation followed by formatting.
	It attemps to strike a balance between erasing all the sectors of the card, which would be too slow on an avr (with a streaming block write this could takes ~23 hours at ~200KB/s with a 16GB card), and erasing a minimum set.
	
	The sectors corresponding to 3 clusters of the ROOT (3*64 sectors) are cleared, as _ufat_format_fat_root initialises 3 clusters for the ROOT (note: this 3-cluster assumption is based on reverse engineering of a computer formatted file system and not fully analysed).
	The sectors of the FAT1 and FAT2 (i.e. all sectors between the beginning of FAT1 and the start of the cluster area) are cleared.
	
	
	Parameters:
		numlogfile			-	Number of log files to create (between 1 and _UFAT_NUMLOGENTRY).
				
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char ufat_format(unsigned char numlogfile)
{
	unsigned char rv;
	
	// Sanity check of parameter value
	if(numlogfile<1)
		numlogfile=1;
	if(numlogfile>_UFAT_NUMLOGENTRY)
		numlogfile=_UFAT_NUMLOGENTRY;
		
	// Initialise the SD card; this initialises _fsinfo.card_capacity_sector
	if(_ufat_init_sd())
		return 1;


	// Erase the content of the card
	fprintf_P(file_pri,PSTR("%sErasing card\n"),_str_ufat);
	if(sd_erase(0,_fsinfo.card_capacity_sector-1))
	{
		fprintf_P(file_pri,PSTR("%sError erasing card\n"),_str_ufat);
		return 1;
	}
		
	// Optional: clear first sectors with specific pattern to make it easier to spot modifications made to the file system
	/*for(unsigned i=0;i<1024;i++)
	{
		memset(ufatblock,0x10+(i&0x07),512);
		// Sectors around the MBR
		unsigned char rv = sd_block_write(i,ufatblock);
		if(rv!=0)
		{
			printf("Error clearing %d\n",i);
			return 1;
		}
		// First sectors of the first partition
		rv = sd_block_write(_UFAT_PARTITIONSTART+i,ufatblock);
		if(rv!=0)
		{
			printf("Error clearing %d\n",i);
			return 1;
		}
	}*/
	
	
	// Starts low level formatting: this creates the MBR (sector 0) with the partition table and the boot sector of the partition 0 (sector _UFAT_PARTITIONSTART)
	if(_ufat_format_mbr_boot())
		return 1;
		
	// The ROOT and FAT should be entirely cleared prior to initialising the ROOT and FAT to the desired state in order to avoid leftover files and leftover used clusters
	memset(ufatblock,0x00,512);				// Clear our temp buffer
	// Erase the root in range [_fsinfo.cluster_begin; _fsinfo.cluster_begin+3*_fsinfo.sectors_per_cluster[ 
	// The assumption is that the ROOT takes 3 clusters, as _ufat_format_fat_root initialises 3 clusters for the ROOT. This assumption has not been fully analysed.
	fprintf_P(file_pri,PSTR("%sClearing ROOT and FAT\n"),_str_ufat);
	for(unsigned char i=0;i<3*_fsinfo.sectors_per_cluster;i++)
	{
		rv = sd_block_write(_fsinfo.cluster_begin+i,ufatblock);
		if(rv)
		{
			fprintf_P(file_pri,PSTR("%sError clearing root at sector %lu\n"),_str_ufat,_fsinfo.cluster_begin+i);
			return 1;
		}
	}
	// Erase the FAT1 and FAT2 (this works regardless as to whether the FAT2 is used) by clearing sectors in range [_fsinfo.fat_sector; _fsinfo.cluster_begin[
	for(unsigned long i=_fsinfo.fat_sector;i<_fsinfo.cluster_begin;i++)
	{
		rv = sd_block_write(i,ufatblock);
		if(rv)
		{
			fprintf_P(file_pri,PSTR("%sError clearing FAT at sector %lu\n"),i);
			return 1;
		}
	}
	
		
	
	// Read the MBR and boot: this reads the card and initialises the internal data structure _fsinfo. This is actually not needed, as the _fsinfo structure is already initialised by the previous call to format with _ufat_format_mbr_boot.
	/*rv = _ufat_mbr_boot_read();
	if(rv)
	{
		printf_P(PSTR("Cannot read the MBR and BOOT sector\n"));
		return 1;
	}*/

	// Print key info	
	//ufat_print_fsinfo(file_pri,&_fsinfo);



	// Write ROOT: this is the ROOT entry which comprises the volume label, the log file entries, and the uFAT metadata. 
	// Below we compute the log file characteristics (size, starting location) and initialise the log file entries in order to be able to call _ufat_write_root
	// Compute the size of the log files
	// - Assumption: the root never takes more than one cluster
	// - Available space: capacity_sector-_fsinfo.cluster_begin - 1 (for root)
	// - The first cluster all all log files must fit in the first entry of a FAT sector (to avoid read/write across logs when modifying one FAT sector). Hence the start location must be rounded down to multiple of 128 clusters.
	// Start sector where log files placed
	_fsinfo.logstartcluster=_logoffsetcluster;
	// Number of available sectors in partition: capacity minus space between start of cluster and start of partition - reserved space for root.
	unsigned long availclust = _fsinfo.numclusters-_logoffsetcluster;
	#ifdef UFATDBG
		printf_P("%sClusters for log files: %lu (total clusters: %lu, offset: %lu)\n",_str_ufat,availclust,_fsinfo.numclusters,_logoffsetcluster);
	#endif
	_fsinfo.logsizecluster = availclust/numlogfile;
	// Round down to multiple of 128 clusters to ensure each file first cluster starts on a new fat sector
	_fsinfo.logsizecluster>>=7;
	_fsinfo.logsizecluster<<=7;
	_fsinfo.logsizebytes = _fsinfo.logsizecluster*_fsinfo.sectors_per_cluster*512;
	#ifdef UFATDBG
		printf_P("%sLog file size: %lu clusters, %lu bytes\n",_str_ufat,_fsinfo.logsizecluster,_fsinfo.logsizebytes);
	#endif
	
	// Populate the log entries
	memset(_logentries,0,sizeof(LOGENTRY)*_UFAT_NUMLOGENTRY);
	for(unsigned i=0;i<numlogfile;i++)			// One more to indicate end 
	{
		sprintf(_logentries[i].name,"LOG-%04u",i);
		_logentries[i].startcluster=_fsinfo.logstartcluster+_fsinfo.logsizecluster*i;
		_logentries[i].startsector=_fsinfo.cluster_begin + (_logentries[i].startcluster-2)*_fsinfo.sectors_per_cluster;			// Cluster numbering starts at 2
		//_logentries[i].size=_fsinfo.logsizecluster*_fsinfo.sectors_per_cluster*512;
		_logentries[i].size=0;
		/*if(i==0)
			_logentries[i].size=64*64*512l;
		if(i==1)
			_logentries[i].size=127*64*512l;
		if(i==2)
			_logentries[i].size=128*64*512l;
		if(i==3)
			_logentries[i].size=129*64*512l;*/
		//_logentries[i].used=0;
	}
	
	
	//ufat_print_loginfo(file_pri,&_logentries[i]);
	
	// Write root containing numlogfiles as described by _logentries.
	rv = _ufat_write_root(numlogfile);
	if(rv)
	{
		printf_P(PSTR("Format: error writing root\n"));
		return 1;
	}
		
	_ufat_format_fat_root(_fsinfo.fat_sector);
	//if(_fsinfo.fat2_sector) _ufat_format_fat_root(_fsinfo.fat2_sector);
	for(unsigned l=0;l<numlogfile;l++)
	{
		fprintf_P(file_pri,PSTR("%sWriting FAT for log %d\n"),_str_ufat,l);
		_ufat_format_fat_log(l,_fsinfo.fat_sector);
		//if(_fsinfo.fat2_sector) _ufat_format_fat_log(l,_fsinfo.fat2_sector);
	}
	
	// Here must reread root to initialise the FS; while we should be okay and could just set fs as available, we piggy back on _ufat_init_fs
	_ufat_init_fs();
	
	return 0;
}
/******************************************************************************
	function: ufat_init
*******************************************************************************	
	Initialise the uFAT filesystem including low-level card initialisation and filesystem check.
	The filesystem is ready to be used immediately afterwards.
	
	This function must be called prior to using the uFAT system.
	
	This function assumes that the SPI peripheral has been initialised.
	It performs low-level SD-card initialisation followed by filesystem initialisation.

	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char ufat_init(void)
{
	// Init the sd subsystem: obtains card availability and capacity in _fsinfo
	if(_ufat_init_sd())
		return 1;
	
	// Init the filesystem info: read card filesystem and initialise the fsinfo structure 
	if(_ufat_init_fs())
	{
		fprintf_P(file_pri,PSTR("%serror, reformat card\r"),_str_ufat);
		return 1;
	}
	fprintf_P(file_pri,PSTR("%sinitialised\n"),_str_ufat);

	return 0;
}

/******************************************************************************
	function: ufat_available
*******************************************************************************	
	Indicates whether the system successfully detected a disk with uFAT.
	
	This function must only be called after a ufat_init or ufat_format call.
	
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char ufat_available(void)
{
	if(_fsinfo.fs_available==0)
		return 0;
	return 1;
}



/******************************************************************************
	function: ufat_log_open
*******************************************************************************	
	Opens the indicated log file for write operations using fprintf, fputc, fputbuf, etc.
	
	Only one log file can be open at any time.

	Parameters:
		n			-	Number of the log file to open; the maximum number available depends on how the card was formatted.

	Returns:
		0			-	Error
		nonzero		-	FILE* for write operations
******************************************************************************/
FILE *ufat_log_open(unsigned char n)
{
	//printf("ufat log open\n");
	// Sanity check: fs must be initialised and lognumber available
	if(_fsinfo.fs_available==0)
	{
		#ifdef UFATDBG
			printf("fs not available\n");
		#endif
		return 0;
	}
	if(n>=_fsinfo.lognum)
	{
		#ifdef UFATDBG
			printf("lognum larger than available\n");
		#endif
		return 0;
	}
	
	// Initialise a FILE structure for writing using 
	fdev_setup_stream(&_log_file,_ufat_log_fputchar,0,_FDEV_SETUP_WRITE);
	
	// Initialise our parameter structure with the buffer write function and store it in the FILE structure
	_log_file_param.putbuf = _ufat_log_fputbuf;
	_log_file_param.txbuf = 0;
	_log_file_param.rxbuf = 0;
	fdev_set_udata(&_log_file,(void*)&_log_file_param);
	
	// Clear the size of file
	_log_current_log = n;
	_log_current_size=_logentries[n].size=0;
	_log_current_sector=_logentries[n].startsector;
	
	
	// Erase file area; this seems more effective than the pre-erase command and helps reduce latency of writes
	fprintf_P(file_pri,PSTR("%sErase sectors %lu-%lu\n"),_str_ufat,_log_current_sector,_log_current_sector+(_fsinfo.logsizebytes>>9)-1);
	if(sd_erase(_log_current_sector,_log_current_sector+(_fsinfo.logsizebytes>>9)-1))
	{
		#ifdef UFATDBG
			printf("Error erasing file\n");
		#endif
		return 0;
	}
	
	fprintf_P(file_pri,PSTR("%sStreaming write at sector %lu\n"),_str_ufat,_log_current_sector);
	// Open stream specifying a pre-erase size
	sd_stream_open(_log_current_sector,_fsinfo.logsizebytes>>9);
	
	return &_log_file;
}
/******************************************************************************
	function: ufat_log_close
*******************************************************************************	
	Close the previously opened log file

	Returns:
		0			-	always
		
******************************************************************************/
unsigned char ufat_log_close(void)
{
	unsigned char rv;
	//printf("ufat_log_close\n");
	//fdev_close(_log_file);			// ? why is this commented out?
	
	rv = sd_streamcache_close(0);
	if(rv!=0)
	{
		printf_P(PSTR("%sFailed sd_write_stream_close\n"),_str_ufat);
	}
	
	// Must write last sector if any
	log_printstatus();
	// Here must write root
	_logentries[_log_current_log].size = _log_current_size;
	rv = _ufat_write_root(_fsinfo.lognum);
	if(rv)
	{
		printf_P(PSTR("%sError writing root\n"),_str_ufat);
	}
	// Two variant of the code exist. If _ufat_format_fat_log reserves only the minimum number of clusters needed for the file size then it must be called again here.
	// Currently _ufat_format_fat_log reserves all clusters on formatting, and this does not need to be called here.
	// _ufat_format_fat_log(_log_current_log,_fsinfo.fat_sector);
	//if(_fsinfo.fat2_sector) _ufat_format_fat_log(_log_current_log,_fsinfo.fat2_sector);
	return 0;
}
/******************************************************************************
	function: ufat_log_test2
*******************************************************************************	
	Test writing data to a log file.
	The data is written in blocks using the putbuf function; bsize indicate the size
	of the blocks.
	In practice floor(size/bsize)*bsize bytes are written to the file.

	Parameters:
		lognum			-	Log number to write to
		size			-	Number of bytes to write
		ch				-	Byte to write
		bsize			-	Size of the blocks written to the file, maximum 512.


******************************************************************************/
/*void ufat_log_test2(unsigned char lognum,unsigned long size,unsigned char ch,unsigned bsize)
{
	FILE *log;
	unsigned long t1,t2,t3;
	unsigned long dt;
	
	if(bsize>512)
		bsize=512;
	
	memset(ufatblock,ch,bsize);
	
	printf_P(PSTR("Benchmarking log %u writing %u bytes %02X in %u long blocks\n"),lognum,size,ch,bsize);
	log = ufat_log_open(lognum);
	if(!log)
	{
		printf("Error opening log\n");
		return;
	}
	log_printstatus();	
	
	t1 = timer_ms_get();
	for(unsigned long i=0;i<size/bsize;i++)
	//for(unsigned long i=0;i<size;i++)
	{
		_ufat_log_fputbuf(ufatblock,bsize);	
		//fputc(ch,&_log_file);
	}
	t2 = timer_ms_get();
	ufat_log_close();
	t3 = timer_ms_get();
	
	dt = t2-t1;
	// size/1024 [KB] / dt/1000 -> size/1024*1000/dt -> size*125/128/dt
	printf_P(PSTR("Time: %lu ms. Speed: %lu kB/s Close time: %lu ms\n"),dt,size*125/128/dt,t3-t2);
}*/
/******************************************************************************
	function: ufat_log_test
*******************************************************************************	
	Writes test data to a log file; this can be used to test write speed and 
	consistency to validate SD cards.
	
	The data is being written is a fake payload of 192B, which is about
	the maximum data written when logging motion sensor data.
	
	The file contains: <CurrentTime><PktCtr><NumFail><UfatLogSize><SizeWritten><AllZeros>
	
	Numfail indicates the number of failures in the fputbuf command. It should be zero.
	UfatLogSize is the size reported by ufat, which should be the same as SizeWritten
	unless there is a write error.
	
	The data file can be loaded for analysis of failures (should be zero) and 
	time interval
	
	Parameters:
		lognum			-	Log number to write to
		size			-	Number of bytes to write
		reportevery		-	Report average speed and status every reportevery bytes (usually 32768).
							A lower value of reportevery (e.g. 4096) would allow to spot hiccups in write
							speed due to SD card maintenance.
							A larger value (32768) would report an 

******************************************************************************/
void ufat_log_test(unsigned char lognum,unsigned long size,unsigned long reportevery)
{
	FILE *log;
	unsigned int szbuf=192;
	char buf[szbuf];
	unsigned long t1;
	//unsigned long dt;
	unsigned long cursize;
	unsigned long pkt;
	unsigned long numfail;
	unsigned long lastsize,lastspeedsize;
	//unsigned long reportevery=65536;
	unsigned long speedevery=4096;
	unsigned long dtworst=0;
	unsigned long dtworstspeed=0;
	unsigned long tlast,tlastspeed;
	
	memset(buf,'0',szbuf);
	buf[szbuf-1]=0;
	buf[szbuf-2]='\n';
	
	
	//memset(ufatblock,ch,bsize);
	
	//_sd_acmd23(9765);
	
	printf_P(PSTR("Benchmarking log %u writing %d bytes up to %lu\n"),lognum,szbuf,size);
	log = ufat_log_open(lognum);
	if(!log)
	{
		printf("Error opening log\n");
		return;
	}
	log_printstatus();	
	
	cursize=0;
	pkt=0;
	numfail=0;
	lastsize=lastspeedsize=0;
	
	tlast=tlastspeed=timer_ms_get();
	
	while(cursize<size)
	{
		// Do something
		//_delay_ms(1);
		//_delay_us(500);
		if(cursize>=lastspeedsize+speedevery)
		{
			lastspeedsize=cursize;
			unsigned long t=timer_ms_get();
			unsigned long dt=t-tlastspeed;
			if(dt>dtworstspeed)
				dtworstspeed=dt;
			tlastspeed=t;
		}
		if(cursize>=lastsize+reportevery)
		{
			lastsize=cursize;
			unsigned long t=timer_ms_get();
			unsigned long dt=t-tlast;
			if(dt>dtworst)
				dtworst=dt;
				
			fprintf_P(file_pri,PSTR("Written: %lu. %lu ms for %lu = %lu KB/s (worst: %lu for %lu = %lu KB/s) average. "),cursize,dt,reportevery,reportevery*1000/dt/1024,dtworst,reportevery,reportevery*1000/dtworst/1024);
			fprintf_P(file_pri,PSTR("Worst peak: %lu ms for %lu = %lu KB/s. Fail: %lu\n"),dtworstspeed,speedevery,speedevery*1000/dtworstspeed/1024,numfail);
			tlast=t;
			
		}
		
		
		t1=timer_ms_get();
		char *strptr = buf;		
		strptr=format1u32(strptr,t1);		
		strptr=format1u32(strptr,pkt);
		strptr=format1u32(strptr,numfail);
		strptr=format1u32(strptr,ufat_log_getsize());
		strptr=format1u32(strptr,cursize);
				
		if(fputbuf(log,buf,szbuf-1))
		{
			numfail++;
		}	
		else
		{
			cursize+=szbuf-1;
		}
		
		pkt++;
		
		
	}
	
	ufat_log_close();
	
	
		
}

/******************************************************************************
	function: ufat_log_getmaxsize
*******************************************************************************	
	Returns the maximum size of files in the given filesystem.
	
	This function requires the filesystem to be initialised.

	Returns:
		0			-	Error
		nonzero		-	Maximum size of the files
******************************************************************************/
unsigned long ufat_log_getmaxsize(void)
{
	// Sanity check: fs must be initialised and lognumber available
	if(_fsinfo.fs_available==0)
	{
		#ifdef UFATDBG
			printf("fs not available\n");
		#endif
		return 0;
	}
	return _fsinfo.logsizebytes;
}
/******************************************************************************
	function: ufat_log_getsize
*******************************************************************************	
	Returns the size of the currently open file.
	
	This function requires the filesystem to be initialised and a log file to be
	open to return a valid result.

	Returns:
		0			-	Error
		nonzero		-	Maximum size of the files
******************************************************************************/
unsigned long ufat_log_getsize(void)
{
	// Sanity check: fs must be initialised and lognumber available
	if(_fsinfo.fs_available==0)
	{
		#ifdef UFATDBG
			printf("fs not available\n");
		#endif
		return 0;
	}
	return _log_current_size;
}
/******************************************************************************
	function: ufat_log_getnumlogs
*******************************************************************************	
	Returns the number of logs available.
	
	If the filesystem is not to be initialised and a log file to be
	open to return a valid result.

	Returns:
		Number of logs available, or 0 if the filesystem is not available.
******************************************************************************/
unsigned char ufat_log_getnumlogs(void)
{
	if(_fsinfo.fs_available==0)
	{
		#ifdef UFATDBG
			printf("fs not available\n");
		#endif
		return 0;
	}
	return _fsinfo.lognum;
}

/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL 
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/


/******************************************************************************
	function: _ufat_init_sd
*******************************************************************************	
	Initialise the SD-card subsystem and check that the SD card is suitable.

	Updates the global variable _fsinfo with the uFAT state:
	_fsinfo.card_available according to card availability
	_fsinfo.fs_available to false
	_fsinfo.card_capacity_sector tp the card capacity

	Parameters:
		capacity_sector		-	Pointer to hold the card capacity of the card in number of sectors

	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char _ufat_init_sd(void)
{
	CID cid;
	CSD csd;
	SDSTAT sdstat;
	unsigned long capacity_sector;
	
	// Initialise fsinfo with card and fs not available
	_fsinfo.card_available=0;
	_fsinfo.fs_available=0;
	
	// Initialise the SD card
	fprintf_P(file_pri,PSTR("%sInit SD...\n"),_str_ufat);
	if(sd_init(&cid,&csd,&sdstat,&capacity_sector))
	{
		fprintf_P(file_pri,PSTR("%sInit SD error\r"),_str_ufat);
		return 1;
	}
	// Check that we deal with an SDHC card with block length of 512 bytes
	if(csd.CSD!=1 || csd.READ_BL_LEN!=9 || csd.WRITE_BL_LEN!=9)
	{
		fprintf_P(file_pri,PSTR("%sSD unsuitable\n"),_str_ufat);
		return 1;
	}
	_fsinfo.card_available=1;
	_fsinfo.card_capacity_sector=capacity_sector;
	return 0;	
}

/******************************************************************************
	function: _ufat_init_fs
*******************************************************************************	
	Initialise the uFAT filesystem data structures from a uFAT-formatted card. 
	
	Must be called after _ufat_init_sd
	
	Assumes a card formatted with uFAT. Reads the MBR, boot sector, and root entry and 
	initialise the FSINFO structure with data related to the card.
	
	Updates:
	_fsinfo.fs_available
	_fsinfo.logstartcluster
	_fsinfo.logsizecluster
	_fsinfo.logsizebytes
	_fsinfo.lognum
	
	Parameters:
		capacity_sector		-	Capacity of the card in number of sectors
				
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char _ufat_init_fs(void)
{
	unsigned char rv;
	
	// The filesystem is not available yet 
	_fsinfo.fs_available=0;
	
	// Read the MBR and boot: this reads the card and initialises the internal data structure _fsinfo
	rv = _ufat_mbr_boot_read();
	if(rv)
	{
		//printf_P(PSTR("Cannot read the MBR and BOOT sector\n"));
		return 1;
	}
	ufat_print_fsinfo(file_pri,&_fsinfo);
	
	// 4. Get root
	// This formula "_fsinfo.cluster_begin+(_fsinfo.root_cluster-2)*_fsinfo.sectors_per_cluster" could be simplified
	// as the root directory is in the first cluster anyways; we could simply read the block at "_fsinfo.cluster_begin", 
	// which is what we do when we write the root
	sd_block_read(_fsinfo.cluster_begin+(_fsinfo.root_cluster-2)*_fsinfo.sectors_per_cluster,ufatblock);	
		
	// 5. Interpret root
	/*for(unsigned i=0;i<16;i++)
	{
		FILEENTRYRAW *fer = (FILEENTRYRAW*)(ufatblock+32*i);
		printf("File %02d. %.8s.%.3s attrib: %02X cluster: %08lX size: %08lX\n",i,fer->name,fer->ext,fer->attrib,_ufat_getfilentrycluster(fer),fer->size);
	}*/
	
	// 6. Get log info in last entry
	FILEENTRYRAW *fe=(FILEENTRYRAW*)(ufatblock+480);
	unsigned char checksum=0;
	for(unsigned i=0;i<8;i++)
		checksum+=fe->name[i];
	checksum+=fe->ext[0];
	checksum+=fe->ext[1];
	fprintf_P(file_pri,PSTR("%sid: %02X checksum: %02X (obtained: %02X)\n"),_str_ufat,fe->name[0],fe->ext[2],checksum);
	if(!(fe->name[0]==0xE5 && fe->ext[2]==checksum))
	{
		fprintf_P(file_pri,PSTR("%serror: log data field invalid\n"),_str_ufat);
		//return 1;
	}
	_fsinfo.logstartcluster = *(unsigned long*)(fe->name+1);
	_fsinfo.logsizecluster = *(unsigned long*)(fe->name+5);		
	_fsinfo.lognum=fe->ext[1];
	_fsinfo.logsizebytes=_fsinfo.logsizecluster*_fsinfo.sectors_per_cluster*512;	
	
	fprintf_P(file_pri,PSTR("%snumlogs: %d startcluster: %lu sizecluster: %lu sizebytes: %lu\n"),_str_ufat,_fsinfo.lognum,_fsinfo.logstartcluster,_fsinfo.logsizecluster,_fsinfo.logsizebytes);
	
	// 7. Convert root to _logentries
	for(unsigned l=0;l<_fsinfo.lognum;l++)
	{
		FILEENTRYRAW *fer = (FILEENTRYRAW*)(ufatblock+32+32*l);
		ufat_fileentryraw2logentry(fer,&_logentries[l]);
	}
	
	// 7. Check that the logs are all valid
	ufat_print_loginfo(file_pri);

	
	// The filesystem is available
	_fsinfo.fs_available=1;
	
	return 0;
	
}



/******************************************************************************
	function: _ufat_getpart
*******************************************************************************	
	Copies the partition entry from a buffer comprising the master boot record into a PARTITION structure.
	
	
	Parameters:
		block		-	Pointer to a buffer holding the master boot record
		pn			-	Partition number, from 0 to 3
		p			-	Pointer to a PARTITION structure that will receive the partition entry
				
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
void _ufat_getpart(char *block,unsigned char pn,PARTITION *p)
{
	memcpy(p,block+446+pn*16,16);
}


/******************************************************************************
	function: _ufat_format_mbr_boot
*******************************************************************************	
	Performs formatting of the card MBR and the BOOT sector of partition 0.
	
	This is used internally by ufat_format.
	This function initialises the fsinfo structure.
	
	
	Parameters:
		capacity_sector		-	Card capacity in sectors
				
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char _ufat_format_mbr_boot(void)
{
	unsigned char rv;
	
	// Create a MBR with only one partition entry
	// ------------------------------------------
	// Clear the ufatblock buffer
	memset(ufatblock,0,512);
	// Access the first partition entry
	PARTITION *p;
	p = (PARTITION*)(ufatblock+446);
	p->type = 0x0b;							// Type FAT32. 
	p->lbabegin = _UFAT_PARTITIONSTART;		// Start location
	p->numsec = _fsinfo.card_capacity_sector;	// Card capacity reported by the SD interface
	p->numsec-=p->lbabegin;					// Define the partition size as spanning the entire card from lbabegin to the end.
	ufatblock[510] = 0x55;					// 0x55
	ufatblock[511] = 0xaa;					// 0xaa
	
	fprintf_P(file_pri,PSTR("%sWriting MBR... "),_str_ufat);
	rv = sd_block_write(0,ufatblock);
	if(rv!=0)
	{
		fprintf_P(file_pri,PSTR("error\n"));
		return 1;
	}
	fprintf_P(file_pri,PSTR("\n"));
	
	// Write the FAT32 Volume ID for the first partition, aka boot sector
	// ------------------------------------------	
	// Clear the ufatblock buffer
	memset(ufatblock,0,512);
	BOOTSECT_FAT32 *bs=(BOOTSECT_FAT32*)ufatblock;
	// Init only nonzero entries
	bs->jump[0] = 0xEB;
	bs->jump[2] = 0x90;
	strcpy(bs->oem,"BLUESENS");
	bs->sectorsize=512;
	bs->sectperclust=64;								// Number of sector per cluster is 64: 64*512=32KB clusters
	bs->numfat = 1;										// One fat only
	//bs->totsectors_short=0;							// This must be zero for totsectors_long to be used; zeroed by memset
	bs->mediadescriptor=248;							// Found by reverse engineering
	bs->sectorpertrack=63;								// Found by reverse engineering
	bs->numberheads=255;								// Found by reverse engineering
	bs->numberhiddensectors=_UFAT_PARTITIONSTART;		// Number of sectors hidden by the OS; those are the number of sectors before the boot sector, i.e. _UFAT_PARTITIONSTART
	bs->totsectors_long=_fsinfo.card_capacity_sector-bs->numberhiddensectors;	// Capacity of the partition is card capacity minus number of hidden sectors
	//bs->fat32_sectorsperfat=?							// This is computed below with spf
	bs->fat32_rootcluster=2;							// Location of the ROOT directory in cluster number; cluster numbering starts at 2, so the root cluster is in the first cluster
	bs->fat32_fsinfosector=1;							// Sector number of the _fsinfo structure
	bs->fat32_bootbackupsector=6;						// Sector number for a copy of the boot sector, typically 6
	bs->drivenumber=128;								// Found by reverse engineering
	bs->bootsignature=0x29;								// 0x29 indicates that the next three fields (volume id, volume label, file system type) are valid
	bs->volid=0x778B96D2;								// Random number?
	strcpy(bs->vollabel,"BLUESENSE-2");					// Volume label
	strcpy(bs->fstype,"FAT32   ");						// File system type
	ufatblock[510]=0x55;								// 0x55 - we write directly to ufatblock as the BOOTSECT_FAT32 structure only comprises the first fields of the boot sector to save space
	ufatblock[511]=0xAA;								// 0xaa
	// Compute dynamic information based on card size
	unsigned long spf;		// Sector per fat, enough to store all the clusters in the partition
	// The number of clusters is roundup(totsectors_long/sectperclust). Here roundup is implemented by adding sectperclust-1 before dividing. 
	// In practice, this is too much space for the FAT as some sectors among totsectors_long will be used for the boot and the FAT itself. 
	// However this approximation simplifies computation.
	// Each cluster link in the FAT uses 4 bytes. Therefore a single FAT sector can represent 128 clusters.
	// The number of sectors per FAT are again rounded up. Therefore the sector per fat is: roundup(roundup(totsectors_long/sectperclust)/128)
	spf = (((bs->totsectors_long+bs->sectperclust-1)/bs->sectperclust)+127)/128;
	bs->fat32_sectorsperfat=spf;						// Number of sectors per fat
	/*bs->reservedsectors=bs->numfat*spf;
	bs->reservedsectors+1024;							// Round up some reserved space; unclear if needed
	bs->reservedsectors>>=9;
	bs->reservedsectors<<=9;*/
	bs->reservedsectors=0x20;							// Documentation says usually 0x20
	
	fprintf_P(file_pri,PSTR("%sWriting bootsect... "),_str_ufat);
	rv = sd_block_write(_UFAT_PARTITIONSTART,ufatblock);
	if(rv!=0)
	{
		fprintf_P(file_pri,PSTR("error\n"));
		return 1;
	}
	fprintf_P(file_pri,PSTR("\n%sWriting bkp bootsect... "),_str_ufat);
	rv = sd_block_write(_UFAT_PARTITIONSTART+bs->fat32_bootbackupsector,ufatblock);
	if(rv!=0)
	{
		fprintf_P(file_pri,PSTR("error\n"));
		return 1;
	}
	fprintf_P(file_pri,PSTR("\n"));
	
	// Initialise the fsinfo structure from the boot sector
	_ufat_bs2keyinfo(bs,&_fsinfo);
		
	return 0;
}

/*unsigned char _ufat_write_root_test(void)
{
	FILEENTRYRAW *fe;
	// Assumes _logentries were created
	memset(ufatblock,0,512);
	// Volume ID
	fe=(FILEENTRYRAW*)ufatblock;
	strcpy(fe->name,"BLUESENS");
	strcpy(fe->ext,"LOG");
	
	fe->attrib = 0x08;
	fe=(FILEENTRYRAW*)(ufatblock+32+0*32);
	strcpy(fe->name,"TESTFILE");
	strcpy(fe->ext,"EXT");
	fe->attrib = 0x20;	// archive
	fe->writetime = 0b0100000010000100; 	// 8h8mn8"
	fe->writedate = 0b0001000100101000;		// 8.08.1980
	fe->createdate=fe->accessdate=fe->writedate;
	fe->createtime=fe->writetime;
	fe->ntres=0;
	fe->createtimetenth=0;
	fe->size = testfilesize;
	
	
	fe->clusterhi = testfilecluster>>16;
	fe->clusterlo = testfilecluster&0xffff;

	// Write
	printf("Writing root... ");
	unsigned char rv = sd_block_write(_fsinfo.cluster_begin,ufatblock);
	if(rv!=0)
	{
		printf("failed\n");
		return 1;
	}
	printf("ok\n");
	return 0;	
}*/

/******************************************************************************
	function: _ufat_write_root
*******************************************************************************	
	Creates the root entries with the available files and metadata.
	
	This is used internally by ufat_format and ufat_log_close.
	The variables _logentries and _fsinfo must be initialised before calling this function.
	
	
	Parameters:
		numlogfile		-	Number of log files
				
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char _ufat_write_root(unsigned char numlogfile)
{
	FILEENTRYRAW *fe;
	memset(ufatblock,0,512);
	// ROOT entry 0: Volume ID
	fe=(FILEENTRYRAW*)ufatblock;
	strcpy(fe->name,"BLUESENS");
	strcpy(fe->ext,"LOG");
	fe->attrib = 0x08;
	// ROOT entry 1 to n: log files
	for(unsigned i=0;i<numlogfile;i++)
	{
		fe=(FILEENTRYRAW*)(ufatblock+32+i*32);
		strcpy(fe->name,_logentries[i].name);
		strcpy(fe->ext,_logentries[i].name);
		fe->attrib = 0x20;	// Read-only and archive
		fe->writetime = 0b0100000010000100; 	// 8h8mn8"
		fe->writedate = 0b0001000100101000;		// 8.08.1980
		fe->createdate=fe->accessdate=fe->writedate;
		fe->createtime=fe->writetime;
		fe->ntres=0;
		fe->createtimetenth=0;
		fe->size = _logentries[i].size;
		fe->clusterhi = _logentries[i].startcluster>>16;
		fe->clusterlo = _logentries[i].startcluster&0xffff;
	}
	// ROOT entry numlogfiles to_UFAT_NUMLOGENTRY: "dummy" files appearing as erased to reserve space in the ROOT and avoid Windows to create e.g. "System Volume Information" and overwriting our metadata
	for(unsigned i=numlogfile;i<_UFAT_NUMLOGENTRY;i++)
	{
		fe=(FILEENTRYRAW*)(ufatblock+32+i*32);
		memset(fe,0,sizeof(FILEENTRYRAW));
		sprintf(fe->name,"DUMMY%d",i);
		fe->name[0]=0xE5;		// Mark of an erased file
		//printf("file %s\n",fe->name);
		strcpy(fe->ext,"DUM");
		fe->attrib = 0x20;	// Read-only and archive
		fe->writetime = 0b0100000010000100; 	// 8h8mn8"
		fe->writedate = 0b0001000100101000;		// 8.08.1980
		fe->createdate=fe->accessdate=fe->writedate;
		fe->createtime=fe->writetime;
		fe->ntres=0;
		fe->createtimetenth=0;
		fe->size = 0;
		//fe->clusterhi = _logentries[i].startcluster>>16;
		//fe->clusterlo = _logentries[i].startcluster&0xffff;
		fe->clusterhi = 0;
		fe->clusterlo = _fsinfo.logstartcluster;
		//fe->clusterlo = 0;
	}
	
	// ROOT entry 15 (last root entry): this entry is hacked to store metadata about the logging system
	fe=(FILEENTRYRAW*)(ufatblock+480);
	fe->name[0]=0xE5;				// Mark of an erased file
	*(unsigned long*)(fe->name+1) = _fsinfo.logstartcluster;		// Start cluster
	*(unsigned long*)(fe->name+5) = _fsinfo.logsizecluster;			// Max file size in clusters
	fe->ext[1] = numlogfile;
	unsigned checksum=0;
	for(unsigned i=0;i<8;i++)
		checksum+=fe->name[i];
	checksum+=fe->ext[0];
	checksum+=fe->ext[1];
	fe->ext[2]=checksum;
	/*fe->name[1]=0;
	fe->ext[0]='J';
	fe->ext[1]=0;*/
	// Additional attributes
	fe->attrib = 0x20;	// Read-only and archive
	fe->writetime = 0b0100000010000100; 	// 8h8mn8"
	fe->writedate = 0b0001000100101000;		// 8.08.1980
	fe->createdate=fe->accessdate=fe->writedate;
	fe->createtime=fe->writetime;
	fe->ntres=0;
	fe->createtimetenth=0;
	fe->size = 0;
	fe->clusterhi = 0;
	fe->clusterlo = _logoffsetcluster;
	

	// Write root sector
	fprintf_P(file_pri,PSTR("%sWriting root... "),_str_ufat);
	unsigned char rv = sd_block_write(_fsinfo.cluster_begin,ufatblock);
	if(rv!=0)
	{
		fprintf_P(file_pri,PSTR("error\n"));
		return 1;
	}
	// Write the sector afterwards as all zeroes
	memset(ufatblock,0,512);
	rv = sd_block_write(_fsinfo.cluster_begin+1,ufatblock);
	if(rv!=0)
	{
		fprintf_P(file_pri,PSTR("error\n"));
		return 1;
	}
	fprintf_P(file_pri,PSTR("\n"));
	return 0;	
}

/*unsigned char ufat_format_alllinkedfat(unsigned long fat_sector,unsigned long firstcluster,unsigned long totclusters)
{
	unsigned long cluster;
	unsigned long oldfatsect,newfatsect;
	// Creates a list of cluster from the first pointing to the next
	for(cluster=firstcluster,oldfatsect=cluster/128;cluster<totclusters;cluster++)
	{
		newfatsect=cluster/128;
		if(newfatsect!=oldfatsect)
		{
			// Write old fat sect
			printf_P(PSTR("Writing FAT sector %lu... (%lu/%lu)"),fat_sector+oldfatsect,cluster,totclusters);
			unsigned char rv = sd_block_write(fat_sector+oldfatsect,ufatblock);
			if(rv!=0)
			{
				printf_P(PSTR("failed\n"));
				return 1;
			}
			printf_P(PSTR("ok\n"));
			oldfatsect=newfatsect;
			memset(ufatblock,0,512);
		}
		// Offset within existing fat
		unsigned long *c = (unsigned long*)ufatblock;
		c[cluster&0x7f] = cluster+1;
	}
	// Write last sector
	printf_P(PSTR("Writing FAT sector %lu... "),fat_sector+newfatsect);
	unsigned char rv = sd_block_write(fat_sector+newfatsect,ufatblock);
	if(rv!=0)
	{
		printf_P(PSTR("failed\n"));
		return 1;
	}
	printf_P(PSTR("ok\n"));
	
}*/
/******************************************************************************
	function: _ufat_format_fat_log
*******************************************************************************	
	Creates the FAT cluster chain for one log entry.
	Assumes the all fat sectors comprise only clusters from a single file (assumption respected by having file sizes multiples of 128 clusters)	
	
	Parameters:
		i				-	Number of log files
		fat_sector		-	Location of the FAT
				
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char _ufat_format_fat_log(unsigned char i,unsigned long fat_sector)
{
	unsigned long firstcluster;
	unsigned long cluster;
	unsigned long oldfatsect;						// oldfatsect is offset from fat_sector, it is the current sector of the FAT being written.
	unsigned long fsc;
	
	//memset(ufatblock,0,512);						// This flags all the clusters as unused. The linked chain is built after.
	memset(ufatblock,0xff,512);						// This flags all the clusters as used (but end of file). The linked chain is built after. 
													// Marking as used is to prevent another OS from using these clusters. A disk analysis tool would indicate that these clusters are incorrectly reserved.
	
	// Find the first and last cluster of the log entry i. The last cluster is determined by the file size.
	firstcluster = _logentries[i].startcluster;	
	/*
	// This version of the code writes only as many clusters as needed for the given file size.
	// This can lead to issue when another operating system looks for empty clusters to store files: the other OS would use clusters that uFAT will require if the file grows
	if(_logentries[i].size)
		fsc = (_logentries[i].size+32767)>>15;			// File size in cluster rounded up
	else
		fsc = 1;										// If file is size 0 reserve a minimum of 1 cluster*/
	// This version of the code writes all the clusters needed for the maximum possible size of the file; this should prevent another OS to reclaim clusters that uFAT will require if the file grows.
	fsc = _fsinfo.logsizecluster;					// To reserve all clusters of the file
		
	// Creates a list of cluster from the first pointing to the next
	// Iterate from cluster 0 to fsc. Each FAT sector can hold 128 clusters.
	// Write the FAT sector at regular intervals when: the last cluster that fits in the current FAT sector is updated, or when the last cluster of the file is put in the FAT
	// Initialise oldfactsect with the FAT sector holding the first cluster of the file
	for(cluster=0,oldfatsect=(firstcluster+cluster)/128;cluster<fsc;cluster++)
	{
		// Find the offset of the cluster within the current FAT sector; this is done by a bitmask with 0x7F, as there are 128 clusters per FAT sector
		unsigned long *c = (unsigned long*)ufatblock;
		if(cluster!=fsc-1)
			c[cluster&0x7f] = firstcluster+cluster+1;				// If the current cluster is not the last one, then point to the next cluster
		else
			//c[cluster&0x7f] = 0xFFFFFFF8;
			c[cluster&0x7f] = 0x0FFFFFFF;							// If the current cluster is the last one, flag the cluster as the end of the chain.
		
		// Write the FAT sector either if the cluster is the last one of the file, or if the next cluster would be on the next fat sector
		// The last cluster of the current fat sector is when cluster is 127, 255, ...
		if( (cluster&0x7f)==0x7f || cluster==fsc-1)
		{
			// Write old fat sect
			//printf_P(PSTR("Writing FAT sector %lu (%lu/%lu)... "),fat_sector+oldfatsect,cluster,fsc);
			unsigned char rv = sd_block_write(fat_sector+oldfatsect,ufatblock);
			if(rv!=0)
			{
				fprintf_P(file_pri,PSTR("Error writing FAT sector %lu (%lu/%lu)... "),fat_sector+oldfatsect,cluster,fsc);
				return 1;
			}
			//printf_P(PSTR("\n"));
			oldfatsect++;											// Point to the next FAT sector
			memset(ufatblock,0,512);								// Reset our buffer
		}
		
	}
	return 0;
}

/******************************************************************************
	function: _ufat_format_fat_root
*******************************************************************************	
	Creates the FAT cluster chain for the ROOT directory. 
	
	
	Parameters:
		fat_sector		-	Location of the FAT
				
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char _ufat_format_fat_root(unsigned long fat_sector)
{
	// Format the FAT for the root directory
	memset(ufatblock,0,512);
	
	unsigned long *c = (unsigned long*)ufatblock;
	// Replicates the entries obtained with sdformatter. Meaning of the entries not analysed.
	c[0]=0x0ffffff8;
	c[1]=0x0fffffff;
	c[2]=0x0fffffff;	
	fprintf_P(file_pri,PSTR("%sWriting FAT for root... "),_str_ufat);
	unsigned char rv = sd_block_write(fat_sector,ufatblock);
	if(rv!=0)
	{
		fprintf_P(file_pri,PSTR("error\n"));
		return 1;
	}
	fprintf_P(file_pri,PSTR("\n"));
	return 0;
}

/*unsigned char ufat_format_fat_root_test(unsigned long fat_sector)
{
	// Format the FAT for the root directory
	memset(ufatblock,0,512);
	
	unsigned long *c = (unsigned long*)ufatblock;
	// Replicates the entries obtained with sdformatter
	c[0]=0x0ffffff8;
	c[1]=0x0fffffff;
	c[2]=0x0fffffff;
	
	unsigned long ie = (testfilesize+32767)/32768;
	for(unsigned long i=0;i<ie;i++)
	{
		if(i==ie-1)
			c[testfilecluster+i]=0x0fffffff;
		else
			c[testfilecluster+i]=testfilecluster+i+1;		
	}
	
	
	
	
	unsigned char rv = sd_block_write(fat_sector,ufatblock);
	if(rv!=0)
	{
		printf_P(PSTR("failed\n"));
		return 1;
	}
	printf_P(PSTR("ok\n"));
	return 0;
}*/


/******************************************************************************
	function: _ufat_bs2keyinfo
*******************************************************************************	
	Summarize key info from boot sector in an _fsinfo structure

	
	Parameters:
		bs					-	Boot sector structure
		fi					-	_fsinfo structure containing the key info about the filesystem
	
******************************************************************************/
void _ufat_bs2keyinfo(BOOTSECT_FAT32 *bs,FSINFO *fi)
{
	fi->partition_sector = bs->numberhiddensectors;								// This is the address of the BOOT sector of the first partition
	fi->partition_capacity_sector = bs->totsectors_long;						// Size of the partition in sectors, including boot, fat, root, data
	fi->fat_sector = fi->partition_sector + bs->reservedsectors;				// Sector containing the FAT1
	if(bs->numfat==2)
		fi->fat2_sector=fi->fat_sector+bs->fat32_sectorsperfat;					// Sector containing the FAT2
	else
		fi->fat2_sector=0;														// or 0 if there is no FAT2
	fi->cluster_begin = _fsinfo.partition_sector + bs->reservedsectors + bs->numfat*bs->fat32_sectorsperfat;		// Sector where the first cluster is located, which is immediately after the reserved sectors and FAT sectors
	fi->sectors_per_cluster = bs->sectperclust;									// Number of sectors per cluster
	fi->numclusters = (fi->partition_capacity_sector-(fi->cluster_begin-fi->partition_sector))/fi->sectors_per_cluster;		// Number of clusters. 
																															// This is the number of sectors in the partition minus the reserved sectors, minus the space for the FAT,
																															// divided by the number of sectors per cluster
	fi->root_cluster = bs->fat32_rootcluster;									// Location of the root cluster
}

/******************************************************************************
	function: _ufat_mbr_boot_read
*******************************************************************************	
	Initialises the internal data structure _fsinfo to access the FAT by reading the card MBR and boot sector.

	Reads the MBR and first partition BOOT sector, performs sanity checking, and extracts the key parameters of the 
	filesystem in the global _fsinfo structure.
	
	Returns:
		0					-	Success
		1					-	Error
******************************************************************************/
unsigned char _ufat_mbr_boot_read(void)
{
	PARTITION p[4];
	
	fprintf_P(file_pri,PSTR("%sReading MBR... "),_str_ufat);
	
	// 1. Read MBR and partition table	
	sd_block_read(0,ufatblock);	
	// Sanity check
	if(ufatblock[510]!=0x55 || ufatblock[511]!=0xAA)
	{
		fprintf_P(file_pri,PSTR("invalid\n"));
		return 1;
	}
	printf_P(PSTR("\n"));
	// Decode partition table and partition 1
	for(unsigned char i=0;i<4;i++)
	{
		_ufat_getpart(ufatblock,i,&p[i]);
		#ifdef UFATDBG
			printf_P(PSTR("Part %d: Boot: %d Type: %02X: LBA begin: %lu Num sector: %lu\n"),i,p[i].boot,p[i].type,p[i].lbabegin,p[i].numsec);
		#endif
	}
	if(p[0].type!=0x0b)
	{
		fprintf_P(file_pri,PSTR("%sInvalid partition table\n"),_str_ufat);
		return 1;
	}

	
	// 2. Read first partition
	fprintf_P(file_pri,PSTR("%sReading bootsect... "),_str_ufat);
	sd_block_read(p[0].lbabegin,ufatblock);	
	BOOTSECT_FAT32 *bs;	
	bs = (BOOTSECT_FAT32*)ufatblock;		// cast to bootsector to simplify checking	
	// Sanity checks:
	// - Type FAT32
	// - 0x55 0xaa terminator
	if(! (strncmp(bs->fstype,"FAT32   ",8)==0 && ufatblock[510]==0x55 && ufatblock[511]==0xaa) )
	{
		fprintf_P(file_pri,PSTR("invalid uFAT\n"));
		return 1;
	}
	fprintf_P(file_pri,PSTR("\n"));
	// Print info
	#ifdef UFATDBG
		ufat_print_boot(file_pri,bs);
	#endif
	// Sanity checks:
	// - hidden sector in boot sector must be equal to lbabegin in partition table
	// - size in partition table must be equal to totsector in boot sector
	// - number of FAT is 1 with uFAT
	if(! (bs->sectorsize==512 && p[0].lbabegin==bs->numberhiddensectors && bs->numfat==1 && p[0].numsec==bs->totsectors_long) )
	{
		fprintf_P(file_pri,PSTR("invalid uFAT\n"));
		return 1;
	}
	
	// 3. Get key data about partition
	_ufat_bs2keyinfo(bs,&_fsinfo);
	return 0;
}


unsigned long _ufat_getfilentrycluster(FILEENTRYRAW *fer)
{
	unsigned long c;
	c=fer->clusterhi;
	c<<=16;
	c|=fer->clusterlo;
	return c;
}
void ufat_fileentryraw2logentry(FILEENTRYRAW *fer,LOGENTRY *le)
{
	memcpy(le->name,fer->name,8);
	memcpy(le->ext,fer->ext,3);
	le->startcluster=fer->clusterhi;
	le->startcluster<<=16;
	le->startcluster|=fer->clusterlo;
	le->startsector=_fsinfo.cluster_begin + (le->startcluster-2)*_fsinfo.sectors_per_cluster;				// Cluster numbering starts at 2
	le->size=fer->size;
	//le->used=le->name[0]==0xE5?0:1;
}
/******************************************************************************
	function: ufat_print_boot
*******************************************************************************	
	Print boot sector information
	
	Parameters:
		f		-		File on which to print information
		bs 		-		Boot sector structure
******************************************************************************/
void ufat_print_boot(FILE *f,BOOTSECT_FAT32 *bs)
{
	fprintf_P(f,PSTR("Boot sector:\n"));
	fprintf_P(f,PSTR("\tjump: %02X %02X %02X\n"),bs->jump[0],bs->jump[1],bs->jump[2]);
	fprintf_P(f,PSTR("\toem: %.8s\n"),bs->oem);
	fprintf_P(f,PSTR("\tSector size: %d\n"),bs->sectorsize);
	fprintf_P(f,PSTR("\tSector/cluster: %d\n"),bs->sectperclust);
	fprintf_P(f,PSTR("\tReserved sectors: %d\n"),bs->reservedsectors);
	fprintf_P(f,PSTR("\tNumber of fats: %d\n"),bs->numfat);
	fprintf_P(f,PSTR("\tNumber of root entries: %d\n"),bs->numrootentries);
	fprintf_P(f,PSTR("\tTotal number of sectors short: %d\n"),bs->totsectors_short);
	fprintf_P(f,PSTR("\tMedia descriptor: %d\n"),bs->mediadescriptor);
	fprintf_P(f,PSTR("\tSectors/FAT: %d\n"),bs->sectorsperfat);
	fprintf_P(f,PSTR("\tSectors/track (CHS): %d\n"),bs->sectorpertrack);
	fprintf_P(f,PSTR("\tNumber of heads (CHS): %d\n"),bs->numberheads);
	fprintf_P(f,PSTR("\tNumber of hidden sectors: %d\n"),bs->numberhiddensectors);
	fprintf_P(f,PSTR("\tTotal number of sectors long: %lu\n"),bs->totsectors_long);
	fprintf_P(f,PSTR("\tFAT32 sectors/FAT: %d\n"),bs->fat32_sectorsperfat);
	fprintf_P(f,PSTR("\tFAT32 flags: %d\n"),bs->fat32_flags);
	fprintf_P(f,PSTR("\tFAT32 root first cluster: %ld\n"),bs->fat32_rootcluster);
	fprintf_P(f,PSTR("\tFAT32 sectors for _fsinfo: %d\n"),bs->fat32_fsinfosector);
	fprintf_P(f,PSTR("\tFAT32 sector of boot backup: %d\n"),bs->fat32_bootbackupsector);
	fprintf_P(f,PSTR("\tDrive number: %d\n"),bs->drivenumber);
	fprintf_P(f,PSTR("\tHead: %d\n"),bs->head);
	fprintf_P(f,PSTR("\tBoot signature: %02X\n"),bs->bootsignature);
	fprintf_P(f,PSTR("\tVolume ID: %lX\n"),bs->volid);
	fprintf_P(f,PSTR("\tVolume label: %.11s\n"),bs->vollabel);
	fprintf_P(f,PSTR("\tFS type: %.8s\n"),bs->fstype);
}

/******************************************************************************
	function: ufat_print_fsinfo
*******************************************************************************	
	Print uFAT key informations
	
	Parameters:
		f		-		File on which to print information
		bs 		-		Boot sector structure
******************************************************************************/
void ufat_print_fsinfo(FILE *f,FSINFO *fi)
{
	fprintf_P(f,PSTR("%skey info:\n"),_str_ufat);
	fprintf_P(f,PSTR("\tCard capacity: %lu sectors\n"),fi->card_capacity_sector);
	fprintf_P(f,PSTR("\tPartition start sector: %lu\n"),fi->partition_sector);
	fprintf_P(f,PSTR("\tPartition capacity: %lu sectors\n"),fi->partition_capacity_sector);
	fprintf_P(f,PSTR("\tFAT1 start sector: %lu\n"),fi->fat_sector);
	fprintf_P(f,PSTR("\tFAT2 start sector: %lu\n"),fi->fat2_sector);
	fprintf_P(f,PSTR("\tCluster start sector: %lu\n"),fi->cluster_begin);
	fprintf_P(f,PSTR("\tSectors/clusters: %u\n"),fi->sectors_per_cluster);
	fprintf_P(f,PSTR("\tRoot cluster: %lu\n"),fi->root_cluster);
	fprintf_P(f,PSTR("\tNumber of data clusters: %lu\n"),fi->numclusters);
}
void ufat_print_loginfo(FILE *f)
{
	fprintf_P(f,PSTR("%sLogs:\n"),_str_ufat);
	for(unsigned i=0;i<_fsinfo.lognum;i++)
	{
		//LOGENTRY l = _logentries[i];
		//fprintf_P(f,PSTR("\t%.8s.%.3s start sector: %lu start cluster: %lu size: %08lXh used: %d\n"),_logentries[i].name,_logentries[i].ext,_logentries[i].startsector,_logentries[i].startcluster,_logentries[i].size,_logentries[i].used);	
		fprintf_P(f,PSTR("\t%.8s.%.3s start sector: %lu start cluster: %lu size: %08lXh\n"),_logentries[i].name,_logentries[i].ext,_logentries[i].startsector,_logentries[i].startcluster,_logentries[i].size);	
	}
}


/******************************************************************************
	function: _ufat_log_fputbuf
*******************************************************************************	
	Internally used to write data to logs when fputbuf is called.
	Do not call directly.
	
	Parameters:
		buffer		-		Buffer containing the data
		size 		-		Size of buffer
	Returns:
		0			-		Success
		EOF			-		Error
******************************************************************************/
unsigned char _ufat_log_fputbuf(char *buffer,unsigned char size)
{
	// Check if space to write to file
	if(_log_current_size+size>_fsinfo.logsizebytes)
	{
		return EOF;
	}

	unsigned char rv = sd_streamcache_write(buffer,size,0);
	_log_current_size+=size;
	if(rv!=0)
	{
		printf("Writing block to sector %lu failed\n",_log_current_sector);
		return EOF;
	}

	return 0;	
	
}

/******************************************************************************
	function: _ufat_log_fputchar
*******************************************************************************	
	Internally used to write one byte to logs when fprintf/fputs/fpuc are called
	
	Parameters:
		c			-		Character to write
		f 			-		File stream
	Returns:
		0			-		Success
		EOF			-		Error
******************************************************************************/
int _ufat_log_fputchar(char c,FILE *f)
{
	// Check if space to write to file
	if(_log_current_size>=_fsinfo.logsizebytes)
	{
		return EOF;
	}
	
	unsigned char rv = sd_streamcache_write(&c,1,0);
	_log_current_size++;
	if(rv!=0)
	{
		printf("Writing block to sector %lu failed\n",_log_current_sector);
		return EOF;
	}
	return 0;
}



void log_printstatus(void)
{
	printf_P(PSTR("%sCurrent log: %u\n"),_str_ufat,_log_current_log);
	printf_P(PSTR("\tsize: %lu\n"),_log_current_size);
	printf_P(PSTR("\tsector: %lu\n"),_log_current_sector);
}


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 