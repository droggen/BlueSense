#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "helper.h"
#include "command.h"
#include "ds3232.h"


/*
	File: command
	
	Command line processing functions
*/
	

unsigned char CommandBuffer[COMMANDMAXSIZE];
unsigned char CommandBufferPtr=0;

const char CommandSuccess[] PROGMEM = "CMDOK\n";
const char CommandInvalid[] PROGMEM = "CMDINV\n";
const char CommandError[] PROGMEM = "CMDERR\n";

const COMMANDPARSER *CommandParsersCurrent;
unsigned char CommandParsersCurrentNum;


/******************************************************************************
	function: CommandProcess
*******************************************************************************	
	Reads file_pri while data available or a command is received, then execute the 
	command.
	
	Returns:
		0		-	Nothing to process anymore
		1		-	Something was processed	
******************************************************************************/
unsigned char CommandProcess(const COMMANDPARSER *CommandParsers,unsigned char CommandParsersNum)
{
	unsigned char rv,msgid;
	
	// Assign to current commands
	CommandParsersCurrent = CommandParsers;
	CommandParsersCurrentNum = CommandParsersNum;
	
	rv = CommandGet(CommandParsers,CommandParsersNum,&msgid);
	if(!rv)
	{
		return 0;
	}
	if(rv==3)
	{
		fputs_P(CommandInvalid,file_pri);
		fputs_P(CommandInvalid,file_dbg);
		return 1;
	}
	if(rv==1)
	{
		fputs_P(CommandSuccess,file_pri);
		fputs_P(CommandSuccess,file_dbg);
		return 1;
	}
	fputs_P(CommandError,file_pri);
	fputs_P(CommandError,file_dbg);	
	return 1;
}

/******************************************************************************
	function: CommandGet
*******************************************************************************	
	Reads file_pri until empty into the command buffer.
	
	Afterwards, process the command buffer and identifies which command to run and returns.
	If a message is received its ID is placed in msgid.
	
	Returns:
		0	-	no message
		1	-	message execution ok (message valid)
		2	-	message execution error (message valid)
		3	-	message invalid 
******************************************************************************/
void printcmd(void)
{
	printf("cmd: ");
	for(int i=0;i<COMMANDMAXSIZE;i++)
		printf("%02X ",CommandBuffer[i]);
	printf(" (%d)\n",CommandBufferPtr);
}
unsigned char CommandGet(const COMMANDPARSER *CommandParsers,unsigned char CommandParsersNum,unsigned char *msgid)
{
	unsigned char rv;
	short c;
	
	//printf("read cmd before: %d\n",CommandBufferPtr);
	// If connected to a primary source, read that source until the source is empty or the command buffer is full
	if(file_pri)
	{
		while(CommandBufferPtr<COMMANDMAXSIZE)
		{
			if((c=fgetc(file_pri)) == -1)
				break;
			CommandBuffer[CommandBufferPtr++] = c;
		}
	}
	//printf("read cmd after: %d\n",CommandBufferPtr);
	// Fast path: command buffer empty
	if(CommandBufferPtr==0)
		return 0;
	//printf("gotdata "); printcmd();
	// Process the command buffer: find first cr or lf
	for(unsigned char i=0;i<CommandBufferPtr;i++)
	{
		if(CommandBuffer[i]==10 || CommandBuffer[i]==13)
		{
			//printf("crlf at %d\n",i);
			// Found a CR or lF			
			if(i==0)
			{
				// First character is a cr/lf: return 'no command'
				rv=0;		
			}
			else
			{
				// The first character is not a cr/lf, hence a command is received
				// Null-terminate the command
				CommandBuffer[i] = 0;
				//printf("cmd string is '%s'\n",CommandBuffer);
				// Decode the command
				rv = CommandDecodeExec(CommandParsers,CommandParsersNum,CommandBuffer,i,msgid);
				//printf("rv is: %d\n",rv);
			}
			//printcmd();
			// Remove the processed command or character from the buffer
			memmove(CommandBuffer,CommandBuffer+1+i,COMMANDMAXSIZE-1-i);
			CommandBufferPtr-=1+i;
			//printcmd();
			//printf("New CommandBufferPtr: %d\n",CommandBufferPtr);
			return rv;
		}
	}
	// If we arrive at this stage: either the newline has not been read yet, or the command buffer is full.
	// Do nothing if the newline hasn't been read yet.
	if(CommandBufferPtr<COMMANDMAXSIZE)
	{
		//printf("no nr|lf yet\n");
		return 0;
	}
	//printf("buff full clear\n");
	// Clear buffer if buffer is full.
	CommandBufferPtr=0;
	return 3;	
}



/******************************************************************************
	function: CommandDecodeExec
*******************************************************************************	
	Identify which parser is appropriate and call it for decoding and execution.


	Parameters:
		CommandParsers		-		Array of COMMANDPARSER
		CommandParsersNum	-		Total number of available commands
		buffer				-		Buffer containing the command; must be null terminated
		size				-		Size of the command in the buffer
		msgid				-		Returns the ID of the command, if a valid command was found

	Returns:
		0		-	No message
		1		-	Message execution ok (message valid)
		2		-	Message execution error (message valid)
		3		-	Message invalid 
		
	Parsers must return:
		0		-	Message execution ok (message valid)
		1		-	Message execution error (message valid)
		2		-	Message invalid 		
******************************************************************************/
unsigned char CommandDecodeExec(const COMMANDPARSER *CommandParsers,unsigned char CommandParsersNum,unsigned char *buffer,unsigned char size,unsigned char *msgid)
{
	unsigned char rv;
	
	if(size==0)
		return 0;
		
	buffer[size]=0;
	
	fprintf_P(file_pri,PSTR("Got message of len %d: '%s'\n"),size,buffer);
	
	for(unsigned char i=0;i<CommandParsersNum;i++)
	{
		if(CommandParsers[i].cmd == buffer[0])
		{
			*msgid = i;
			rv = CommandParsers[i].parser(buffer+1,size-1);
			//fprintf_P(file_pri,PSTR("Parser %d: %d\n"),i,rv);
			return rv+1;
		}		
	}
	// Message invalid
	return 3;
}

/******************************************************************************
	Function: CommandSet
*******************************************************************************	
	Sets a command (one or more, newline delimited) in the command buffer.
	This can be used to set a command script to execute e.g. on startup.

	Parameters:
		script		-		command/script to copy in buffer
		n			-		size of script; note that only up to COMMANDMAXSIZE
							bytes are copied in the command buffer.
******************************************************************************/
void CommandSet(char *script,unsigned char n)
{
	if(n>COMMANDMAXSIZE)
		n=COMMANDMAXSIZE;
	memcpy(CommandBuffer,script,n);
	CommandBufferPtr=n;
}




