// generated with AvrWiz (by g@greschenz.de)
// -----------------------------------------
// cpu:     ATmega64
// speed:   15.36 mhz (max: 16 mhz)
// voltage: 5 V
// ram:     4096 bytes (0x0100-0x10ff)
// rom:     65536 bytes (0x0000-0xffff)
// eeprom:  2048 bytes (0x0000-0x07ff)
// -----------------------------------------

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "macro.h"
#include "io_conf.h"
#include "watchdog.h"
#include "usart.h"
#include "AD7367.h"
#include "int_init.h"
#include "timer.h"
#include <math.h>
#include "power.h"
#include "TWI_Master.h"

/***********************************************************/
/***********************************************************/
#if !defined (__AVR_ATmega64__)
#error __AVR_ATmega64__ not defined !
#endif

#if !defined (F_CPU)
#define F_CPU 15360000
#endif

#define RAMSTART 0x0100
#define RAMSIZE (RAMEND-RAMSTART)

#define sleep()
/***********************************************************/
/***********************************************************/
#define	START_TIMER1	TCCR1B = 0x0B;//启动定时器
#define	STOP_TIMER1		TCCR1B = 0x00;//启动定时器
//#define _PART_EEPROM_

unsigned int Vw;
unsigned int Vr;
unsigned int Vi;
unsigned int Ad;

static int Vref = 0;
//static  int Vtest =	2000;

static int VrefAD_1 = 0;
static int VrefAD_2 = 0;
static int VrefAD_3 = 0;
static int VrefAD_4 = 0;
static int VrefAD_5 = 0;

volatile unsigned char setbackup[22] =
{ 0x3C, 0x16, 0xE9, 0x05, 0xFA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C,
		0x00, 0x00, 0x05, 0x00, 0x00, 0x03, 0x3A, 0xC5, 0x3E };
volatile unsigned char command[40];
volatile unsigned char commandPC[40];
//volatile unsigned char reply[2];
volatile unsigned char C2_reply[4] =
{ 0 };
volatile unsigned char data[12];

volatile static unsigned char core2_replay = 0; // 0 无定义	1 成功	2失败 3request

volatile unsigned char system_status = 0; //0	无状态	1 系统故障	2 系统正常运行中 3加载自校状态
volatile static char intx_status = 0;
volatile static char int1_status = 0;
volatile static char AD_Protext = 0;
volatile static int liangchen = 5;
volatile static char flag_hl = 0; //恒流恒压配置状态标志位 0未配置，1配置过了
//static unsigned char jishu = 0;

static unsigned char btimer = 0; //	0 每秒10次	1 每秒100次
volatile char flag_po = 0; //电源指示

/***********************************************************/
/***********************************************************/
void
delay(int ms);
void
beep(char on_off);
void
ExtIrq_Init(void);
void
usarterror(void);

#ifdef _PART_EEPROM_
void flashwrite(uchar num);
void flashread(uchar num);
#endif

unsigned char
keyscan(void);
void
huandang(int i);
void
jiazai(void);
void
xiezai(void);
void
zhiliu(int i);
void
kaiji(void);
/***********************************************************/
/***********************************************************/
void delay(int ms)
{
	unsigned long i;
	if (ms == 0)
	{
		for (i = 0; i < 20; i++)
			NOP();
		return;
	}
	for (i = 0; i < ms * 269; i++)
		NOP();
}

void beep(char on_off)
{
	if (on_off)
	{
		PORTG |= BIT(2);
	}
	else
	{
		PORTG &= ~BIT(2);
	}
}

// ---------------
// --- ext irq ---
// ---------------

void ExtIrq_Init(void)
{
	port_init();
	usart_init();
	watchdog_ini();
	int_init();
	timer1_init();
	TWI_Master_Initialise();
}

void usarterror(void)
{
	command[0] = head;
	command[11] = tail;
	command[1] = 4;
	command[2] = ~command[1];
	command[3] = fail;
	puts_0((unsigned char *) command, 12);
}

#ifdef _PART_EEPROM_
void flashwrite(uchar num)
{
	eeprom_busy_wait(); //等待 EEPROM 读写就绪
	switch(num)
	{
		case 0: eeprom_write_word(0,VrefAD_1);break;
		case 1: eeprom_write_word(10,VrefAD_2);break;
		case 2: eeprom_write_word(20,VrefAD_3);break;
		case 3: eeprom_write_word(30,VrefAD_4);break;
		case 4: eeprom_write_word(0,VrefAD_1);eeprom_write_word(10,VrefAD_2);eeprom_write_word(20,VrefAD_3);eeprom_write_word(30,VrefAD_4);break;
		case 5: eeprom_write_word(40,VrefAD_5);break;
		case 6: eeprom_write_block(50,setbackup,22);break;
		default:break;
	}
}

void flashread(uchar num)
{
	eeprom_busy_wait(); //等待 EEPROM 读写就绪
	switch(num)
	{
		case 0: VrefAD_1 = eeprom_read_word(0);break;
		case 1: VrefAD_2 = eeprom_read_word(10);break;
		case 2: VrefAD_3 = eeprom_read_word(20);break;
		case 3: VrefAD_4 = eeprom_read_word(30);break;
		case 4: VrefAD_1 = eeprom_read_word(0); VrefAD_2 = eeprom_read_word(10);VrefAD_3 = eeprom_read_word(20);VrefAD_4 = eeprom_read_word(30);break;
		case 5: VrefAD_5 = eeprom_read_word(40);break;
		case 6: eeprom_write_block(setbackup,50,20);break;
		default:break;
	}
}

#endif

unsigned char keyscan(void)
{
	if ((PIND & (BIT(4) | BIT(5) | BIT(6) | BIT(7))) != 0)
	{
		_delay_ms(5);
		if ((PIND & (BIT(4) | BIT(5) | BIT(6) | BIT(7))) == 0)
			return 0;
		// 1 减少//2 增加//3 消警//4 加载自校
		if ((PIND & BIT(4)) != 0)
		{
			beep(1);
			while ((PIND & BIT(4)) != 0)
			{
				_delay_ms(10);
				wdt_reset();
				PORTG ^= BIT(1);
			}
			beep(0);
			return 1;
		}
		if ((PIND & BIT(5)) != 0)
		{
			beep(1);
			while ((PIND & BIT(5)) != 0)
			{
				_delay_ms(10);
				wdt_reset();
				PORTG ^= BIT(1);
			}
			beep(0);
			return 2;
		}
		if ((PIND & BIT(6)) != 0)
		{
			beep(1);
			while ((PIND & BIT(6)) != 0)
			{
				_delay_ms(10);
				wdt_reset();
				PORTG ^= BIT(1);
			}
			beep(0);
			return 4;
		}
		if ((PIND & BIT(7)) != 0)
		{
			beep(1);
			while ((PIND & BIT(7)) != 0)
			{
				_delay_ms(10);
				wdt_reset();
				PORTG ^= BIT(1);
			}
			beep(0);
			return 3;
		}
	}
	else
		return 0;
	return 0;
}

/*
 变量
 电流值	a1	a2	a3	a4	j4
 2uA		1	1	1	1	0
 20uA	0	1	1	1	0
 200uA	0	0	1	1	0
 2mA		1	1	0	1	0
 20mA	0	1	0	0	0
 200mA	0	0	0	0	1

 a1	f1
 a2	f0
 a3	e2
 a4	e3
 j4	a0
 */

void huandang(int i)
{
	switch (i)
	{
	case 1:
	{
		PORTF |= BIT(0) | BIT(1);
		PORTA &= ~BIT(0);
		PORTE |= BIT(2) | BIT(3);
	}
		break;
	case 2:
	{
		PORTF |= BIT(0);
		PORTF &= ~BIT(1);
		PORTA &= ~BIT(0);
		PORTE |= BIT(2) | BIT(3);
	}
		break;
	case 3:
	{
		PORTF &= ~(BIT(0) | BIT(1));
		PORTA &= ~BIT(0);
		PORTE |= BIT(2) | BIT(3);
	}
		break;
	case 4:
	{
		PORTF |= BIT(0) | BIT(1);
		PORTA &= ~BIT(0);
		PORTE |= BIT(3);
		PORTE &= ~BIT(2);
	}
		break;
	case 5:
	{
		PORTF |= BIT(0);
		PORTF &= ~BIT(1);
		PORTA &= ~BIT(0);
		PORTE &= ~(BIT(3) | BIT(2));
	}
		break;
	case 6:
	{
		PORTA |= BIT(0);
		PORTF &= ~(BIT(0) | BIT(1));
		PORTE &= ~(BIT(3) | BIT(2));
	}
		break;
	default:
		break;
	}
}

void jiazai(void)
{
	PORTA |= BIT(3);
	command[0] = head;
	command[1] = 6;
	command[2] = ~command[1];
	command[3] = 1;
	command[4] = ~command[3];
	command[5] = tail;
	puts_1((unsigned char *) command, 6);
}

void xiezai(void)
{
	PORTA &= ~BIT(3);
	command[0] = head;
	command[1] = 6;
	command[2] = ~command[1];
	command[3] = 3;
	command[4] = ~command[3];
	command[5] = tail;
	puts_1((unsigned char *) command, 6);
}

void zhiliu(int i)
{
	if (i >= 0)
	{
		command[5] = 0;
		command[6] = (unsigned char) (i & 0xff);
		command[7] = (unsigned char) ((i >> 8) & 0xff);
	}
	else
	{
		command[5] = 1;
		command[6] = (unsigned char) ((-i) & 0xff);
		command[7] = (unsigned char) (((-i) >> 8) & 0xff);
	}
	command[0] = head;
	command[1] = 9;
	command[2] = ~command[1];
	command[3] = 4;
	command[4] = ~command[3];
	command[8] = tail;
	puts_1((unsigned char *) command, 9);
}

void kaiji(void)
{
	int cnt = 0;
	//AD 校正
//	PORTE |= BIT(4) | BIT(5) | BIT(6) | BIT(7);
	PORTD |= BIT(4);
	_delay_ms(100);
	while (Ad == 0)
	{
		AD_getdata(3);
		_delay_ms(1);
	}
	VrefAD_4 = Ad;
	while (Vw == 0)
	{
		AD_getdata(0);
		_delay_ms(1);
	}
	VrefAD_1 = Vw;
	while (Vr == 0)
	{
		AD_getdata(1);
		_delay_ms(1);
	}
	VrefAD_2 = Vr;
	while (Vi == 0)
	{
		AD_getdata(2);
		_delay_ms(1);
	}
	VrefAD_3 = Vi;
#ifdef _PART_EEPROM_
	flashwrite(4);
#endif	
//	PORTE &= ~(BIT(4) | BIT(5) | BIT(6) | BIT(7));
	PORTD &= ~BIT(4);

#ifdef _PART_EEPROM_
	flashread(5);
#endif
	Vref = VrefAD_5;
	if (Vref >= 0) //？？？
		command[5] = 0;
	else
		command[5] = 1;
	command[0] = head;
	command[1] = 9;
	command[2] = ~command[1];
	command[3] = 6;
	command[4] = ~command[3];
	if (Vref >= 0)
	{
		command[6] = ((Vref >> 8) & 0xFF);
		command[7] = Vref & 0xFF;
	}
	else
	{
		command[6] = (((-Vref) >> 8) & 0xFF);
		command[7] = (-Vref) & 0xFF;
	}
	command[8] = tail;
	puts_1((unsigned char *) command, 9);
	AD_getdata(3);
	Ad -= VrefAD_4;
	while ((Ad > 5) && (Ad < 8192))
	{
		Vref -= 1, VrefAD_5 -= 1;
		command[0] = head;
		command[1] = 9;
		command[2] = ~command[1];
		command[3] = 6;
		command[4] = ~command[3];
		if (Vref >= 0)
			command[5] = 0;
		else
			command[5] = 1;
		if (Vref >= 0)
		{
			command[6] = ((Vref >> 8) & 0xFF);
			command[7] = Vref & 0xFF;
		}
		else
		{
			command[6] = (((-Vref) >> 8) & 0xFF);
			command[7] = (-Vref) & 0xFF;
		}
		command[8] = tail;
		puts_1((unsigned char *) command, 9);
		AD_getdata(3);
		if ((unsigned int) Ad > (unsigned int) VrefAD_4)
			Ad -= VrefAD_4;
	}
	while ((16384 - Ad > 5) && (Ad > 8192))
	{
		Vref += 1, VrefAD_5 += 1;
		command[0] = head;
		command[1] = 9;
		command[2] = ~command[1];
		command[3] = 6;
		command[4] = ~command[3];
		if (Vref >= 0)
			command[5] = 0;
		else
			command[5] = 1;
		if (Vref >= 0)
		{
			command[6] = ((Vref >> 8) & 0xFF);
			command[7] = Vref & 0xFF;
		}
		else
		{
			command[6] = (((-Vref) >> 8) & 0xFF);
			command[7] = (-Vref) & 0xFF;
		}
		command[8] = tail;
		puts_1((unsigned char *) command, 9);
		AD_getdata(3);
		if ((unsigned int) Ad > (unsigned int) VrefAD_4)
			Ad -= VrefAD_4;
	}
	while ((Ad > 1) && (Ad < 8192))
	{
		cnt++;
		if (cnt > 100)
			break;
		Vref -= 1, VrefAD_5 -= 1;
		command[0] = head;
		command[1] = 9;
		command[2] = ~command[1];
		command[3] = 6;
		command[4] = ~command[3];
		if (Vref >= 0)
			command[5] = 0;
		else
			command[5] = 1;
		if (Vref >= 0)
		{
			command[6] = ((Vref >> 8) & 0xFF);
			command[7] = Vref & 0xFF;
		}
		else
		{
			command[6] = (((-Vref) >> 8) & 0xFF);
			command[7] = (-Vref) & 0xFF;
		}
		command[8] = tail;
		puts_1((unsigned char *) command, 9);
		AD_getdata(3);
		if ((unsigned int) Ad > (unsigned int) VrefAD_4)
			Ad -= VrefAD_4;
	}
	cnt = 0;
	while ((16384 - Ad > 1) && (Ad > 8192))
	{
		cnt++;
		if (cnt > 100)
			break;
		Vref += 1, VrefAD_5 += 1;
		command[0] = head;
		command[1] = 9;
		command[2] = ~command[1];
		command[3] = 6;
		command[4] = ~command[3];
		if (Vref >= 0)
			command[5] = 0;
		else
			command[5] = 1;
		if (Vref >= 0)
		{
			command[6] = ((Vref >> 8) & 0xFF);
			command[7] = Vref & 0xFF;
		}
		else
		{
			command[6] = (((-Vref) >> 8) & 0xFF);
			command[7] = (-Vref) & 0xFF;
		}
		command[8] = tail;
		puts_1((unsigned char *) command, 9);
		AD_getdata(3);
		if ((unsigned int) Ad > (unsigned int) VrefAD_4)
			Ad -= VrefAD_4;
	}
#ifdef _PART_EEPROM_
	flashwrite(5);
#endif
	PORTA |= BIT(3);
	PORTF |= BIT(3);
	PORTF |= BIT(2);
//////////////////	
	AD_getdata(0);
	_delay_ms(1);
	VrefAD_1 = Vw;

	AD_getdata(1);
	_delay_ms(1);
	VrefAD_2 = Vr;

	AD_getdata(2);
	_delay_ms(1);
	VrefAD_3 = Vi;
///////////////////		
	PORTA &= ~BIT(3);
}

void volreset()
{
	unsigned char status;
	status = MCUCSR;
	if ((status & (BIT(2) | BIT(3))) != 0)
	{
		commandPC[0] = head;
		commandPC[1] = 6;
		commandPC[2] = ~commandPC[1];
		commandPC[11] = tail;
		xiezai();
		puts_0((unsigned char *) commandPC, 12);
	}
}


static unsigned char TWI_buf[ TWI_BUFFER_SIZE ];    // Transceiver buffer
static unsigned char TWI_msgSize;                   // Number of bytes to be transmitted.
static unsigned char TWI_state = TWI_NO_STATE;      // State byte. Default set to TWI_NO_STATE.

union TWI_statusReg TWI_statusReg = {0};            // TWI_statusReg is defined in TWI_Master.h

/****************************************************************************
Call this function to set up the TWI master to its initial standby state.
Remember to enable interrupts from the main application after initializing the TWI.
****************************************************************************/
void TWI_Master_Initialise(void)
{
  TWBR = TWI_TWBR;                                  // Set bit rate register (Baudrate). Defined in header file.
  TWAR = 0x4b<<1;
// TWSR = TWI_TWPS;                                  // Not used. Driver presumes prescaler to be 00.
  TWDR = 0xFF;                                      // Default content = SDA released.
  TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
         (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt.
         (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
         (0<<TWWC);                                 //
}    
    
/****************************************************************************
Call this function to test if the TWI_ISR is busy transmitting.
****************************************************************************/
unsigned char TWI_Transceiver_Busy( void )
{
  return ( TWCR & (1<<TWIE) );                  // IF TWI Interrupt is enabled then the Transceiver is busy
}

/****************************************************************************
Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation. If there was an error, then the function 
will return the TWI State code. 
****************************************************************************/
unsigned char TWI_Get_State_Info( void )
{
  while ( TWI_Transceiver_Busy() );             // Wait until TWI has completed the transmission.
  return ( TWI_state );                         // Return error state.
}

/****************************************************************************
Call this function to send a prepared message. The first byte must contain the slave address and the
read/write bit. Consecutive bytes contain the data to be sent, or empty locations for data to be read
from the slave. Also include how many bytes that should be sent/read including the address byte.
The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
then initialize the next operation and return.
****************************************************************************/
void TWI_Start_Transceiver_With_Data( unsigned char *msg, unsigned char msgSize )
{
  unsigned char temp;

  while ( TWI_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.

  TWI_msgSize = msgSize;                        // Number of data to transmit.
  TWI_buf[0]  = msg[0];                         // Store slave address with R/W setting.
  if (!( msg[0] & (TRUE<<TWI_READ_BIT) ))       // If it is a write operation, then also copy data.
  {
    for ( temp = 1; temp < msgSize; temp++ )
      TWI_buf[ temp ] = msg[ temp ];
  }
  TWI_statusReg.all = 0;      
  TWI_state         = TWI_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
}

/****************************************************************************
Call this function to resend the last message. The driver will reuse the data previously put in the transceiver buffers.
The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
then initialize the next operation and return.
****************************************************************************/
void TWI_Start_Transceiver( void )
{
  while ( TWI_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.
  TWI_statusReg.all = 0;      
  TWI_state         = TWI_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
}

/****************************************************************************
Call this function to read out the requested data from the TWI transceiver buffer. I.e. first call
TWI_Start_Transceiver to send a request for data to the slave. Then Run this function to collect the
data when they have arrived. Include a pointer to where to place the data and the number of bytes
requested (including the address field) in the function call. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation, before reading out the data and returning.
If there was an error in the previous transmission the function will return the TWI error code.
****************************************************************************/
unsigned char TWI_Get_Data_From_Transceiver( unsigned char *msg, unsigned char msgSize )
{
  unsigned char i;

  while ( TWI_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.

  if( TWI_statusReg.lastTransOK )               // Last transmission competed successfully.              
  {                                             
    for ( i=0; i<msgSize; i++ )                 // Copy data from Transceiver buffer.
    {
      msg[ i ] = TWI_buf[ i ];
    }
  }
  return( TWI_statusReg.lastTransOK );                                   
}

// ********** Interrupt Handlers ********** //
/****************************************************************************
This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
that is whenever a TWI event has occurred. This function should not be called directly from the main
application.
****************************************************************************/
ISR(TWI_vect)
//#pragma vector=TWI_vect
//__interrupt void TWI_ISR(void)
{
  static unsigned char TWI_bufPtr;
  
  switch (TWSR)
  {
/////////////////////////////////////////////////////////////////////////////////////////////
//write
/////////////////////////////////////////////////////////////////////////////////////////////
	case TWI_START:             // START has been transmitted  
    case TWI_REP_START:         // Repeated START has been transmitted
      TWI_bufPtr = 0;                                     // Set buffer pointer to the TWI Address location
    case TWI_MTX_ADR_ACK:       // SLA+W has been tramsmitted and ACK received
    case TWI_MTX_DATA_ACK:      // Data byte has been tramsmitted and ACK received
      if (TWI_bufPtr < TWI_msgSize)
      {
        TWDR = TWI_buf[TWI_bufPtr++];
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to send byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
               (0<<TWWC);                                 //  
      }else                    // Send STOP after last byte
      {
        TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
               (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
               (0<<TWWC);                                 //
      }
      break;
/////////////////////////////////////////////////////////////////////////////////////////////
//read
/////////////////////////////////////////////////////////////////////////////////////////////
    case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK tramsmitted
      TWI_buf[TWI_bufPtr++] = TWDR;
    case TWI_MRX_ADR_ACK:       // SLA+R has been tramsmitted and ACK received
      if (TWI_bufPtr < (TWI_msgSize-1) )                  // Detect the last byte to NACK it.
      {
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to read next byte
               (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send ACK after reception
               (0<<TWWC);                                 //  
      }else                    // Send NACK after next reception
      {
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to read next byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send NACK after reception
               (0<<TWWC);                                 // 
      }    
      break; 
    case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK tramsmitted
      TWI_buf[TWI_bufPtr] = TWDR;
      TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
             (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
             (0<<TWWC);                                 //
 //return the phase;
		command[0] = head;
		commandPC[0] = head;
		commandPC[1] = 9;
		commandPC[2] = ~commandPC[1];
		commandPC[3] =TWI_buf[1];
		commandPC[4] =TWI_buf[2];
		commandPC[5] =TWI_buf[3];
		commandPC[6] =TWI_buf[4];
		commandPC[11] = tail;
		puts_0((unsigned char *) commandPC, 12); 		
      break;      
    case TWI_ARB_LOST:          // Arbitration lost
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag
             (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|           // Initiate a (RE)START condition.
             (0<<TWWC);                                 //
      break;
    case TWI_MTX_ADR_NACK:      // SLA+W has been tramsmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been tramsmitted and NACK received   
    case TWI_MTX_DATA_NACK:     // Data byte has been tramsmitted and NACK received
		_delay_ms(10);     
     TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag
             (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|           // Initiate a (RE)START condition.
             (0<<TWWC);                                 //
      break;    	
//    case TWI_NO_STATE              // No relevant state information available; TWINT = ?0?
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:     
      TWI_state = TWSR;                                 // Store TWSR and automatically sets clears noErrors bit.                                                       // Reset TWI Interface
      TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt
             (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests
             (0<<TWWC);                                 //
  }
}

/***********************************************************/
/***********************************************************/

// --------------
// --- main() ---
// --------------
int main()
{
	unsigned char key = 0xff;
//	int m_liangchen = 0;
	ExtIrq_Init();
	sei(); // enable interrupts
	beep(1);
	_delay_ms(100);
	volreset();
	beep(0);
	{
		//增加开机读取eeprom//初始化指示灯，量程端口
		zhiliu(0);
		_delay_ms(10);
		zhiliu(0);
		_delay_ms(10);
		zhiliu(0);
	}
	kaiji();

	PORTA &= ~BIT(4);
	PORTF &= ~BIT(2);
	while (1)
	{
		my_power();
		if ((PINA & BIT(2)) == 0) //M信号判断
		{
			if (btimer == 1 && liangchen < 6)
			{
			}
			if (intx_status++ < 10)
				_delay_ms(25);
			if (intx_status == 21)
			{
				if (liangchen < 6)
				{
				}
				else
				{
					//亮红灯
					PORTG |= BIT(4);
					//还原标志
					xiezai();
					//关ad
					STOP_TIMER1;
					_delay_ms(10);
					//设定系统状态
					system_status = 1;
					commandPC[0] = head;
					commandPC[1] = 1;
					commandPC[2] = ~commandPC[1];
					commandPC[11] = tail;
					puts_0((unsigned char *) commandPC, 12);
				}
				intx_status = 0;
				AD_Protext = 0;
			}
			else if (liangchen == 6)
			{
				//卸载
				//	xiezai();
				PORTA &= ~BIT(3);
				_delay_ms(20);
				PORTA |= BIT(3);
				//加载
				//	jiazai();
			}
		}
		if (int1_status == 1) //自校状态
		{
			key = keyscan();
			if (key == 3)
			{
				int1_status = 0;
			}
			if (key == 1)
				Vref += 10, VrefAD_5 += 10;
			else if (key == 2)
				Vref -= 10, VrefAD_5 -= 10;
			if ((key == 1) || (key == 2))
			{
				if (Vref >= 0)
					command[5] = 0;
				else
					command[5] = 1;
				command[0] = head;
				command[1] = 9;
				command[2] = ~command[1];
				command[3] = 6;
				command[4] = ~command[3];
				if (Vref >= 0)
				{
					command[6] = ((Vref >> 8) & 0xFF);
					command[7] = Vref & 0xFF;
				}
				else
				{
					command[6] = (((-Vref) >> 8) & 0xFF);
					command[7] = (-Vref) & 0xFF;
				}
				command[8] = tail;
				puts_1((unsigned char *) command, 9);
#ifdef _PART_EEPROM_
				flashwrite(5);
#endif

			}
		}
		else if (system_status == 2) //加载/卸载按钮 开led加载
		{
			key = keyscan();
			if (key == 4)
			{
				system_status = 0;
				//关led
				PORTG &= ~BIT(0);

				//卸载
				xiezai();
			}
			else if (key == 3) //消警操作
			{
				PORTG &= ~BIT(4);
//				 if(system_status == 1)
//				system_status = 0;//恢复系统状态
			}
		}
		else if (system_status == 0)
		{
			key = keyscan();
			if (key == 4)
			{
				if (flag_hl == 0) //默认恒压配置
				{
					PORTF |= BIT(3);
					PORTF |= BIT(2);
					puts_1((unsigned char *) setbackup, 22);
				}
				//0	无状态	1 系统故障	2 系统正常运行中 3加载自校状态
				system_status = 2;
				//开led
				PORTG |= BIT(0);

				//加载
				jiazai();
			}
			if (key == 3) //消警操作
			{
				PORTG &= ~BIT(4);
//				 if(system_status == 1)
//				system_status = 0;//恢复系统状态
			}
		}
		else if (system_status == 1)
		{
			key = keyscan();
			if (key == 3) //消警操作
			{
				PORTG &= ~BIT(4);
				if (system_status == 1)
					system_status = 0; //恢复系统状态
			}
		}
	}
	return 0;
}

/****************************************************************************************/
/************************************* interrput ****************************************/
//T0溢出中断服务程序
//#pragma interrupt_handler timer0_ovf_isr:10
//void timer0_ovf_isr(void)
SIGNAL( SIG_OUTPUT_COMPARE0)
{
	static int i = 0;
	wdt_reset();
	if (i++ == 4)
	{
		i = 0;
		PORTG ^= BIT(1);
	}
}

//串行接收结束中断服务程序
//#pragma interrupt_handler usart_rx_isr:14
//void usart_rx_isr(void)
SIGNAL( SIG_UART1_RECV)
{
	static unsigned char index = 0;
	unsigned char status, data, tch;
	status = UCSR1A;
	data = UDR1;
//	usart_putchar_0(data);
	if ((status & (BIT(4) | BIT(3) | BIT(2))) == 0)
	{
		if (data == head)
			index = 1, C2_reply[0] = data;
		else if (index != 0)
			C2_reply[index++] = data;
		if (index == 4)
		{
//				puts_0((unsigned char *)C2_reply,4);
			tch = ~C2_reply[2];
			if (C2_reply[1] == tch)
			{
				switch (C2_reply[1])
				{
				case 1:
					START_TIMER1
					;
				case 2:
				case 3:
					if (C2_reply[1] == 3)
						STOP_TIMER1
					;
				case 4:
				case 5:
				case 6:
				case 7:
				case 8:
					command[0] = head;
					command[11] = tail;
					command[1] = 4;
					command[2] = ~command[1];
					command[3] = C2_reply[1];
					puts_0((unsigned char *) command, 12);
					break;
				case fail:
					command[0] = head;
					command[11] = tail;
					command[1] = 4;
					command[2] = ~command[1];
					command[3] = fail;
					puts_0((unsigned char *) command, 12);
					break;
				case req_set:
					commandPC[0] = head;
					commandPC[1] = 6;
					commandPC[2] = ~commandPC[1];
					commandPC[11] = tail;
					puts_0((unsigned char *) commandPC, 12);
					TCCR0 = 0x00; //饿死看门狗
					break;
				case req_on:
					command[0] = head;
					commandPC[0] = head;
					commandPC[1] = 6;
					commandPC[2] = ~commandPC[1];
					commandPC[11] = tail;
					puts_0((unsigned char *) commandPC, 12);
					TCCR0 = 0x00; //饿死看门狗
					break;
				default:
					break;
				}
				//初始档位回送
				if (C2_reply[1] == 1)
				{
					commandPC[0] = head;
					commandPC[1] = 3;
					commandPC[2] = ~commandPC[1];
					commandPC[3] = (unsigned char) liangchen;
					commandPC[11] = tail;
					puts_0((unsigned char *) commandPC, 12);
				}
			}
			index = 0;
		}
	}
}

void Set_Voltage_base(uchar F,uchar H,uchar L)
{
	int temp;
	unsigned char send[8];
	if(F)
	{
	// <0
		temp = command[9]*100;
		temp +=command[10];
		temp = 65536*temp/2000;
		temp = -temp;	
	}
	else
	{
	// >0
		temp = command[9]*100;
		temp +=command[10];
		temp = 65536*temp/2000;	
	}
	send[0] = (0x4A<<1);//i2c write
	send[1] = (temp &0xff00)>>8;
	send[2] = (temp &0x00ff);
	send[3] = send[1];
	send[4] = send[2];
	TWI_Start_Transceiver_With_Data(send,5);
}

void Get_Phase(uchar c)
{
	unsigned char send[8];
	send[0] = (0x4A<<1);//i2c write
	send[1] = c;
	send[2] = c;
	send[3] = !c;
	send[4] = !c;
	TWI_Start_Transceiver_With_Data(send,5);
}

//串行接收结束中断服务程序
//#pragma interrupt_handler usart_rx_isr:14
//void usart_rx_isr(void)
SIGNAL( SIG_UART0_RECV)
{
	unsigned int i; //增加PC自校命令//加载后延时B = 1
	unsigned static char index = 0;
	static char flag_transover = 0;
	static unsigned char LEN = 0;
	unsigned char status, data;
	unsigned long tTime = 0;
	wdt_reset();
	status = UCSR0A;
	data = UDR0;
	if ((status & (BIT(4) | BIT(3) | BIT(2))) == 0)
	{
		if (index == 0 && data != head)
		{
			usarterror();
			return;
		}
		else
			commandPC[index++] = data;
		if (index == 2)
			LEN = data;
//			if((index == 3)&&(LEN !=~data))
//				{usarterror();return;}
		if ((index > 3) && (index > LEN))
		{
			usarterror();
			return;
		}
		if ((index == 22) && (data != tail))
		{
			usarterror();
			return;
		}
		if ((index == LEN) && (index > 1))
			flag_transover = 1;
	}
	if (flag_transover)
	{
		index = 0;
		flag_transover = 0;
		for (i = 0; i < LEN; i++)
		{
			command[i] = commandPC[i];
		}
		switch (commandPC[3])
		{
		case 1:
		{
			system_status = 2;

			_delay_ms(200);
			PORTA |= BIT(3); //b = 1
			//START_TIMER1;
			//开led
			PORTG |= BIT(0);
			break;
		}
		case 2:
			break;
		case 3:
		{
			system_status = 0;
			PORTA &= ~BIT(3); //b = 0
			PORTF &= ~BIT(2);
			STOP_TIMER1;
			//关led
			PORTG &= ~BIT(0);
			break;
		}
		case 4:
			break;
		case 5:
		{
			//基准修正?-> 不用 开机自检后，下位机得到零飘修正数据后自动修正
			for (i = 0; i < LEN; i++)
			{
				setbackup[i] = commandPC[i];
			}
#ifdef _PART_EEPROM_
			flashwrite(6);
#endif
			tTime = commandPC[14];
			tTime *= 256;
			tTime += commandPC[15];
			tTime *= 256;
			tTime += commandPC[16];

			if (tTime < 100)
			{
				OCR1A = 2400 - 1; //24000->100ms
				btimer = 1;
			}
			else
			{
				OCR1A = 24000 - 1; //24000->100ms
				btimer = 0;
			}
//set voltage base for assist core
			Set_Voltage_base(commandPC[8],commandPC[9],commandPC[10]);
			break;
		}
		case 6:
		{
			if (command[5] == 1)
				Vref += 10, VrefAD_5 += 10;
			else if (command[5] == 2)
				Vref -= 10, VrefAD_5 -= 10;
			if ((command[5] == 1) || (command[5] == 2))
			{
				if (Vref >= 0)
					command[5] = 0;
				else
					command[5] = 1;
				command[0] = head;
				command[1] = 9;
				command[2] = ~command[1];
				command[3] = 6;
				command[4] = ~command[3];
				if (Vref >= 0)
				{
					command[6] = ((Vref >> 8) & 0xFF);
					command[7] = Vref & 0xFF;
				}
				else
				{
					command[6] = (((-Vref) >> 8) & 0xFF);
					command[7] = (-Vref) & 0xFF;
				}
				command[8] = tail;
				puts_1((unsigned char *) command, 9);
#ifdef _PART_EEPROM_
				flashwrite(5);
#endif
			}
			break;
		}
		case 7:
		{
			flag_hl = 1;
			if (command[5] == 1)
			{
				//ch=0,q1=1,q3=1
				PORTA &= ~BIT(1);
				PORTF |= BIT(3);
				PORTF |= BIT(2);
			}
			else if (command[5] == 2)
			{
				//ch=0,q1=0,q3=1
				PORTA &= ~BIT(1);
				PORTF &= ~BIT(3);
				PORTF |= BIT(2);
			}
			else if (command[5] == 3)
			{
				//ch = 1 b= 0 q1 q3 = 0
				PORTA &= ~BIT(3);
				PORTF &= ~BIT(3);
				PORTF &= ~BIT(2);
				PORTA |= BIT(1);
				START_TIMER1;
				//_delay_ms(2000);
				//PORTA &= ~BIT(1);
			}
			command[0] = head;
			command[11] = tail;
			command[1] = 4;
			command[2] = ~command[1];
			command[3] = 7;
			puts_0((unsigned char *) command, 12);
			break;
		}
		case 8:
		{
			huandang(command[5]);
			liangchen = command[5];
			command[0] = head;
			command[11] = tail;
			command[1] = 4;
			command[2] = ~command[1];
			command[3] = 8;
			puts_0((unsigned char *) command, 12);
			break;
		}
		case 9:
		{
			Get_Phase(command[5]);
			break;
		}

		default:
			break;
		}
		if (commandPC[3] < 6) //6要做中间处理78不用下发
		{
			puts_1((unsigned char *) command, LEN);
		}
	}
}

#if 0
//外中断1服务程序
//#pragma interrupt_handler int0_isr:2
//void int0_isr(void)
SIGNAL( SIG_INTERRUPT1) //自校自锁
{
	//外中断1
//	unsigned char k;
	if (system_status == 0)
	{
//		Vref = 0x7FFF;
		if (int1_status == 0) /////////////////////加B信号控制 B=0
		{
			PORTD |= BIT(0);
			_delay_ms(100);
			wdt_reset();
			PORTG ^= BIT(1);
			_delay_ms(100);
			wdt_reset();
			PORTG ^= BIT(1);
			_delay_ms(100);
			wdt_reset();
			PORTG ^= BIT(1);
			_delay_ms(100);
			wdt_reset();
			PORTG ^= BIT(1);
			_delay_ms(100);
			wdt_reset();
			PORTG ^= BIT(1);
			PORTD &= ~BIT(0); //闪烁
			AD_getdata(4);
			VrefAD_1 = Vw;
			VrefAD_2 = Vr;
			VrefAD_3 = Vi;
			VrefAD_4 = Ad;
#ifdef _PART_EEPROM_
			flashwrite(4);
#endif
			int1_status = 1;
//			PORTA |= BIT(1);					//CH信号
		}
	}
}
#endif

//外中断1服务程序
//#pragma interrupt_handler int0_isr:2
//void int0_isr(void)
SIGNAL( SIG_INTERRUPT7) //自校自锁
{
	//外中断7
	unsigned char send[8];
	send[0] = (0x4A<<1|1);//i2c read
	TWI_Start_Transceiver_With_Data(send,5);
}


//定时器T1匹配中断A服务程序
//#pragma interrupt_handler timer1_compa_isr:8
//void timer1_compa_isr(void)
SIGNAL( SIG_OUTPUT_COMPARE1A) //	AD	定时器	中断
{
	//compare occured TCNT1=OCR1A
//	static	int int_timer1 = 0;
	unsigned char check = 0; //添加量程提示
	unsigned char i;
//	static unsigned char fenpin = 0;
//	static unsigned char dfenpin = 0;
//	int m_liangchen = 0;

//	static unsigned int LL = 0;

//	static unsigned int low = 50000;
//	static unsigned int high = 0;
//	static unsigned char len = 0;
//	static unsigned char clearintx = 0;
	AD_getdata(0);
	AD_getdata(1);
	AD_getdata(2);

//		while(fabs(high_1-Vw)>20)
//		{
//			high_1 = Vw;
//			AD_getdata(0);
//		}
	if ((unsigned int) Vw > (unsigned int) VrefAD_1)
		Vw -= VrefAD_1;
	else
		Vw = (16384 - ((unsigned int) VrefAD_1 - (unsigned int) Vw));
	if ((unsigned int) Vr > (unsigned int) VrefAD_2)
		Vr -= VrefAD_2;
	else
		Vr = (16384 - ((unsigned int) VrefAD_2 - (unsigned int) Vr));
	if ((unsigned int) Vi > (unsigned int) VrefAD_3)
		Vi -= VrefAD_3;
	else
		Vi = (16384 - ((unsigned int) VrefAD_3 - (unsigned int) Vi));

	data[0] = head;
	data[1] = 2;
	data[2] = ~data[1];
	data[3] = (Vw >> 8) & 0xFF;
	data[4] = Vw & 0xFF;
	data[5] = (Vr >> 8) & 0xFF;
	data[6] = Vr & 0xFF;
	data[7] = (Vi >> 8) & 0xFF;
	data[8] = Vi & 0xFF;
	for (i = 3; i < 6; i++)
	{
		check ^= data[i];
	}
	data[9] = check;
	data[10] = ~check;
	data[11] = tail;
//		puts_0("hello",5);
	puts_0((unsigned char *) data, 12);	
}
