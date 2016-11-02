#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "serial.h"
//#include "global.h"
//#include "helper.h"
#include "main.h"
//#include "test.h"
//#include "wait.h"
#include "i2c.h"
#include "adc.h"
#include "i2c_internal.h"

void i2c_test_int0(void)
{
	/*printf("Enable TWI and TWI interrupt\n");
	_delay_ms(100);
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
	_delay_ms(1000);
	printf("Manually set TWINT\n");
	TWCR = TWCR|(1<<TWINT);
	_delay_ms(1000);
	*/
	printf("Enable TWI and TWI interrupt\n");
	_delay_ms(100);
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
	_delay_ms(1000);
	while(1)
	{
		printf("Generate a start condition\n");
		_delay_ms(100);
		TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE);
		_delay_ms(1000);
		
		printf("Generate a stop condition\n");
		_delay_ms(100);
		TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		_delay_ms(1000);
		printf("End\n");
	}
	
}