/*
 *	Projekt: LED_Scanner
 *	Version: 1.0
 *	Created: 27.01.2015 09:05:21
 *  Author: Hm
 */ 


/* Microchip: MCP23017T-E/ML


	***I2C Adressen***

IC I   READ = 0x41 | WRITE = 0x40
IC II  READ = 0x43 | WRITE = 0x42
IC III READ = 0x45 | WRITE = 0x44

	***REGISTER*** 

I/O DIRECTION REGISTER:
Addr = 0x00 | Wert = 0xFF
***************************
INPUT POLARITY REGISTER:
Addr = 0x01 | Wert = 0x00
***************************
INTERRUPT-ON-CHANGE CONTROL REGISTER:
Addr = 0x02 | Wert = 0x00
***************************
DEFAULT COMPARE REGISTER FOR INTERRUPT-ON-CHANGE:
Addr = 0x03 | Wert = 0x00
***************************
INTERRUPT CONTROL REGISTER:
Addr = 0x04 | Wert = ?
***************************
IOCON – I/O EXPANDER CONFIGURATION REGISTER:
Addr = 0x05 | Wert = ?
***************************
PULL-UP RESISTOR CONFIGURATION REGISTER:
Addr = 0x06 | Wert = ?
***************************
INTERRUPT FLAG REGISTER:
Addr = 0x07 | Wert = ?
***************************
INTERRUPT CAPTURE REGISTER:
Addr = 0x08 | Wert = ?
***************************
PORT REGISTER:
Addr = 0x09 | Wert = ?
***************************
OUTPUT LATCH REGISTER (OLAT):
Addr = 0x0A | Wert = ?
***************************

*/


#define F_CPU 16000000


#define RESET_IO

/* I2C Addressé´s */
#define MCP23017_I		0x40
#define MCP23017_II		0x42
#define MCP23017_III	0x44

/* Sensors direct on the µC */
#define PT49			PD4
#define PT50			PD5
#define PT51			PD6

// registers
#define IODIRA			0x00
#define IODIRB			0x01
#define IPOLA			0x02
#define IPOLB			0x03
#define GPINTENA		0x04
#define GPINTENB		0x05
#define DEFVALA			0x06
#define DEFVALB			0x07
#define INTCONA			0x08
#define INTCONB			0x09
#define IOCONA			0x0A
#define IOCONB			0x0B
#define GPPUA			0x0C
#define GPPUB			0x0D
#define INTFA			0x0E
#define INTFB			0x0F
#define INTCAPA			0x10
#define INTCAPB			0x11
#define GPIOA			0x12
#define GPIOB			0x13
#define OLATA			0x14
#define OLATB			0x15
#define INT_ERR			255

/* 9600 baud */
#define UART_BAUD_RATE  9600

/* State LED */
#define STATE_LED_ON		PORTC &= ~(1<<PC7)
#define STATE_LED_OFF		PORTC |=  (1<<PC7)
#define STATE_LED_TOGGLE	PORTC ^=  (1<<PC7)

/* Including Header´s */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "uart.h"
#include "i2cmaster.h"
#include "CRC.h"

/* Variables */
volatile uint8_t ret, ID_Counter, time_cnt;
uint16_t Info_Tmp;
char Buffer[20];	

/* Subroutines */
uint16_t readGPIOAB(uint8_t I2C_Addr);

void MCP23017_init(uint8_t I2C_Addr);
uint8_t check_i2c_Slave(uint8_t i2c_slave_address);
void aviable_I2C_Chips(void);

int main(void)
{
	/* Output for STATE LED */
	DDRC |= (1<<PC7);
	
	/* setting the TIMER1 */
	TCCR1B |= ((1<<WGM12) | (1<<CS12) | (1<<CS10)); //CTC Mode , Prescaler : 1024
	TIMSK  |= (1<<OCIE1A); // OutputCompareEnable 
	OCR1A   = ((F_CPU / 1024 / 100) -1);

	/* initalise the UART Unit */
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	
	/* initalise the I2C Unit */ 
	i2c_init();
	
	/* Enable Interrupts global */
	sei();
	
	/* initialize the Port Expander */
//  MCP23017_init(MCP23017_I);
//  MCP23017_init(MCP23017_II);
//  MCP23017_init(MCP23017_III);
	 
	/* start message for the User */
	uart_puts("LED Scanner V1.0");
	uart_puts("\r");
	uart_puts("Firmware by Hm");
	uart_puts("\n");

	/* LED default @ GND level */
	STATE_LED_ON;
	
	
    while(1)
    {
			
		/* aviable I2C Devices? */
		aviable_I2C_Chips(); // all I2C Chips online?
				
		/* read the 16 Input Bit´s from Chip one */
        Info_Tmp = readGPIOAB(MCP23017_I); // read MCP23017 I
		sprintf(Buffer, "%u ", Info_Tmp); // Integer to ASCII
		uart_puts(Buffer); // UART out
		uart_puts("\n\r");
		
 		/* read the 16 Input Bit´s from Chip two */
		Info_Tmp = readGPIOAB(MCP23017_II); // read MCP23017 II
		sprintf(Buffer, "%u ", Info_Tmp); // Integer to ASCII
		uart_puts(Buffer); // UART out

 		
 		/* read the 16 Input Bit´s from Chip three */
		Info_Tmp = readGPIOAB(MCP23017_III); // read MCP23017 III
		sprintf(Buffer, "%u ", Info_Tmp); // Integer to ASCII
		uart_puts(Buffer); // UART out
	

 		/* read the Sensors direct on the µC */
		Info_Tmp = PIND; // read logic level @ PIND
		Info_Tmp = (Info_Tmp & 0b01110000); // mask the result
		Info_Tmp = Info_Tmp >> 4; // result = 0b00000111
		sprintf(Buffer, "%u ", Info_Tmp); // integer to ASCII
		uart_puts(Buffer); // UART out

		
		
		_delay_ms(80); // short delay 


	}
}

/* live the CPU? ca. 10ms. */
ISR(TIMER1_COMPA_vect)
{
	time_cnt++; // every ~ 10ms. +1
	
	if (time_cnt >= 100) // ca. 1000 ms
	{
		time_cnt = 0x00; // reset time counter
		STATE_LED_TOGGLE; // state led toggle ca. 1 Hz		
	}
	

}

/* Reads all 16 pins (port A and B) into a single 16 bits variable */
uint16_t readGPIOAB(uint8_t I2C_Addr) 
{

	/* read the current GPIO output latches */
	i2c_start_wait(I2C_Addr + I2C_WRITE);
	i2c_write(GPIOA);
	i2c_stop();
	
	i2c_start_wait(I2C_Addr + I2C_READ);
	uint8_t a   = i2c_readAck();
	uint16_t ba = i2c_readNak();
	i2c_stop();
	
	ba <<= 8;
	ba |= a;
	
	return ((uint16_t) ba);
}

/* initialize Routine for MCP23017 */
void MCP23017_init(uint8_t I2C_Addr)
{
	i2c_start_wait(I2C_Addr + I2C_WRITE);
	i2c_write(IODIRA);
	i2c_write(0x00);
	i2c_write(IODIRB);
	i2c_write(0x00);
	i2c_stop();
}

/* check all I2C Device´s */
uint8_t check_i2c_Slave(uint8_t i2c_slave_address)
{
	ret = i2c_start(i2c_slave_address+I2C_WRITE);
	if (ret)
	{

		//SLAVE_NICHT_ANWESEND;
		//STATE_LED_OFF;

	}
	else
	{
		//SLAVE_ANWESEND;
		//STATE_LED_ON;
		
		if (i2c_slave_address == MCP23017_I)
		{
			ID_Counter = ID_Counter + 1;
		}
		else if (i2c_slave_address == MCP23017_II)
		{
			ID_Counter = ID_Counter + 2;
		}
		else if (i2c_slave_address == MCP23017_III)
		{
			ID_Counter = ID_Counter + 8;
		}
		
		
	}		
	
	i2c_stop();
	
	return ID_Counter;
}

void aviable_I2C_Chips()	
{
	ID_Counter = 0;
	
	/* the IC´s present? */
	check_i2c_Slave(MCP23017_I);
	check_i2c_Slave(MCP23017_II);
	check_i2c_Slave(MCP23017_III);

	/* Checksumme ( IC1 = 1 | IC2 = 2 | IC3 = 8 ) */
	char Buffer[4];
	sprintf(Buffer, "%u ", ID_Counter);
	uart_puts(Buffer);
}