#ifndef __COMMAND_H
#define __COMMAND_H


#define COMMANDMAXSIZE 16

/*typedef struct 
{
	unsigned char h,m,s;
} COMMANDDATATIME;
typedef struct 
{
	unsigned char d,m,y;
} COMMANDDATADATE;
typedef struct
{
	unsigned long period;;
	unsigned char mask;
} COMMANDDATAADC;
*/
/*typedef union {
	COMMANDDATATIME time;
	COMMANDDATADATE date;
	COMMANDDATAADC adc;
} COMMANDDATA;*/

typedef struct {
	unsigned char cmd;
	unsigned char (*parser)(unsigned char *,unsigned char size);
	const char *help;
} COMMANDPARSER;

extern const COMMANDPARSER *CommandParsersCurrent;
extern unsigned char CommandParsersCurrentNum;



unsigned char CommandProcess(const COMMANDPARSER *CommandParsers,unsigned char CommandParsersNum);
unsigned char CommandGet(const COMMANDPARSER *CommandParsers,unsigned char CommandParsersNum,unsigned char *msgid);
unsigned char CommandDecodeExec(const COMMANDPARSER *CommandParsers,unsigned char CommandParsersNum,unsigned char *buffer,unsigned char size,unsigned char *msgid);
void CommandSet(char *script,unsigned char n);

#endif