/*
   MEGALOL - ATmega LOw level Library
	Menu Module
   Copyright (C) 2009,2010:
         Daniel Roggen, droggen@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "helper.h"




/*
   Displays menu and handles user interaction on stream 'file'
*/
int menu_DisplayMenu(FILE *file,PGM_P title,PGM_P *me,int n,int active)
{
   char userinput[16];
   char *r;
   char *trimmedinput;
   unsigned char select_valid;
   char sep1,sep2;


   while(1)
   {
      // Display the menu title
      fputs_P(crlf,file);
      fputs_P(title,file);
      fputs_P(PSTR(":\r\n"),file);
      for(unsigned char i=0;i<strlen_P(title)+1;i++)
      	fputc('-',file);
      fputs_P(crlf,file);
	
		// Display the menu entries
      for(int i=0;i<n;i++)
      {
         if(i==active)
         {
            sep1='[';
            sep2=']';
         }
         else
         	sep1=sep2=32;
			fputc(32	,file);
			fputc(sep1,file);
			fputs_P(me[2*i],file);
			fputc(sep2,file);
			fputc(9,file);
			fputs_P(me[2*i+1],file);
			fputs_P(crlf,file);
      }
      fputs_P(PSTR("Select: "),file);
      
      // Get user input
      r = fgets(userinput,16,file);
      fputs_P(crlf,file);
      
      // Trim the input from carriage return / linefeed
      trimmedinput = trim(userinput);
      
      // No input provided - display the menu again.
      if(strlen(trimmedinput)==0)
         continue;

      // Input provided: look to which command it maps
      select_valid=0;
      for(int i=0;i<n;i++)
      {
         if(strcmp_P(trimmedinput,me[2*i])==0)    // found the command
         {
            select_valid=1;
            return i;
         }
      }

   	fputs_P(PSTR("Invalid input"),file);
   }
   return -1;
}

