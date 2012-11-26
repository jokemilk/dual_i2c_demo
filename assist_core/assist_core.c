//包含所需头文件
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h> 
//#include "TWI_Master.h"
#include "twi_slave.h"

/*------宏定义------*/
#define uchar	unsigned char
#define uint	unsigned int
#define BIT(x)	(1<<(x))
#define NOP()	asm("nop")
#define WDR() 	asm("wdr")

struct CONFIGS{
	int base_voltage;
	uint32_t phase[3];
	uint32_t ticks;
	unsigned char flag;
	unsigned char fetch_no;
};

struct mychars{
	char a4;
	char a3;
	char a2;
	char a1;
};

union Mylong
{
	struct mychars c;
	long l;
};

union Mylong ml;

volatile struct CONFIGS Configs = {0x8000,{0,0,0},0,0,0xff};
static int VREF = 0;


//端口初始化
void port_init(void)
{
	PORTB = 0x00;
	DDRB  = 0x00|BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5);
	PORTC = 0x00;
	DDRC  = 0x00|BIT(1);
	PORTD = 0x00;
	DDRD  = 0x00|BIT(1)|BIT(0);//|BIT(2)|BIT(3);
}


//定时器T0初始化
void timer0_init(void)
{
/*
	TCCR0B  = 0x00;//停止定时器
	TCNT0  = 0x00;//初始值
	OCR0A   = 200;//匹配值
	TCCR0A = 0x00;
	TIMSK0 = 0x02; //timer 0 interrupt sources
	TCCR0B = 0x05;//打开计时器
	*/
	TCCR0B  = 0x00;//停止定时器
	TIMSK0 = _BV(OCIE0A);  // Enable Interrupt TimerCounter0 Compare Match A (SIG_OUTPUT_COMPARE0A)
    TCCR0A = _BV(WGM01);  // Mode = CTC
    TCCR0B = _BV(CS02) | _BV(CS00);   // Clock/1024, 
    OCR0A = 149;          // 
}

#define START_T1 TCCR1B = 0x09//启动定时器
#define STOP_T1 TCCR1B = 0x00//启动定时器

//定时T1初始化
void timer1_init(void)
{
	TCCR1B = 0x00; //停止定时器
	TIMSK1 |= 0x02; //中断允许
	TCNT1H = 0x00;
	TCNT1L = 0x00; //初始值
	OCR1A = 96 - 1;//80k //7680->1ms
	OCR1BH = 0x00;
	OCR1BL = 0x03; //匹配B值
	ICR1H = 0xFF;
	ICR1L = 0xFF; //输入捕捉匹配值
	TCCR1A = 0x00;

}


//定时器T1匹配中断A服务程序
//#pragma interrupt_handler timer1_compa_isr:8
//void timer1_compa_isr(void)
SIGNAL(TIMER1_COMPA_vect)
{
	//compare occured TCNT1=OCR1A
//	PORTB ^=BIT(1);
	Configs.ticks++;
//	PORTD ^= BIT(1);	
}


//SPI初始化
void spi_init(void)
{
	//spi初始化
	SPCR = 0x54+3;
	SPSR = 0x01;
}

//功能:使用SPI发送一个字节
void spi_write(uchar sData)
{
	SPDR = sData;
	while(!(SPSR & BIT(SPIF)));
	//sData=SPDR;//读从机发回来的数据
}


//功能:使用SPI接收一个字节
uchar spi_read(void)
{
	SPDR = 0x00;
	while(!(SPSR & BIT(SPIF)));
	return SPDR;
}

void write(unsigned int temp)
{
	unsigned char SB = 0;
/////////////////////////////////////////////////
	PORTB &= ~(BIT(2));
	SB = (char)((temp>>10)&0xff);
	spi_write(SB);
	SB = (char)((temp>>2)&0xff); 
	spi_write(SB);

	SB = (char)((temp<<6)&0xff); 
	spi_write(SB);
	PORTB ^= BIT(2);
/////////////////////////////////////////////////
}

void adc_init(void)
{
	//adc转换初始化
	ADCSRA	= 0x00;	//禁止AD转换
	ADMUX	= 0x00 | (1<<REFS1)|(1<<REFS0);
    DIDR0 &=~BIT(0);
	ACSR	= 0x80; //禁止模拟比较器
	ADCSRA	= 0x84;
}


unsigned int adc_calc(void)
{
	//计算实际电压
	unsigned long value=0;
	unsigned int voltage=0; //电压单位为(mV)
	ADCSRA|=_BV(ADSC);
	while(ADCSRA&_BV(ADSC));
	value=ADCL;		 //首先读低位
	value|=(int)ADCH << 8; //然后读高位
//	value=(value*2560)>>10;
	voltage = value;
	return voltage;
}

//T0比较中断服务程序
//#pragma interrupt_handler timer0_comp_isr:11
//void timer0_comp_isr(void)
SIGNAL(TIMER0_COMPA_vect)
{
	static int i = 0;
	wdt_reset();
	if (i++ == 4)
	{
		i = 0;
		PORTD ^= BIT(1);
	}
}


//看门狗初始化
void watchdog_init(void)
{
	WDR (); //this prevents a timeout on enabling
	WDTCSR |= (1<<WDCE) | (1<<WDE);/* 30-Oct-2006 Umesh*/  
	WDTCSR = 0x1F; //WATCHDOG ENABLED - dont forget to issue WDRs
}

static unsigned char TWI_buf[TWI_BUFFER_SIZE];     // Transceiver buffer. Set the size in the header file
static unsigned char TWI_msgSize  = 4;             // Number of bytes to be transmitted.
static unsigned char TWI_state    = TWI_NO_STATE;  // State byte. Default set to TWI_NO_STATE.

union TWI_statusReg TWI_statusReg = {0};           // TWI_statusReg is defined in TWI_Slave.h


/**
 * Call this function to set up the TWI slave to its initial standby state.
 * Remember to enable interrupts from the main application after initializing the TWI.
 * Pass both the slave address and the requrements for triggering on a general call in the
 * same byte. Use e.g. this notation when calling this function:
 * TWI_Slave_Initialise( (TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) );
 * The TWI module is configured to NACK on any requests. Use a TWI_Start_Transceiver function to 
 * start the TWI.
 * ---------------------------------------------------------------------------------------------- */
void TWI_Slave_Initialise( unsigned char TWI_ownAddress )
{
//	TWBR = TWI_TWBR;								  // Set bit rate register (Baudrate). Defined in header file. 
  TWAR = TWI_ownAddress;                            // Set own TWI slave address. Accept TWI General Calls.
  TWDR = 0xFF;                                      // Default content = SDA released.
  TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
         (0<<TWIE)|(0<<TWINT)|                      // Disable TWI Interupt.
         (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Do not ACK on any requests, yet.
         (0<<TWWC);                                 //
}    


/**
 * Call this function to test if the TWI_ISR is busy transmitting.
 * ---------------------------------------------------------------------------------------------- */
unsigned char TWI_Transceiver_Busy( void )
{
    // IF TWI interrupt is enabled then the Transceiver is busy
    return ( TWCR & (1<<TWIE) );
}


/**
 * Call this function to fetch the state information of the previous operation. The function will 
 * hold execution (loop) until the TWI_ISR has completed with the previous operation. If there was
 * an error, then the function will return the TWI State code.
 * ---------------------------------------------------------------------------------------------- */
unsigned char TWI_Get_State_Info( void )
{
 
  while ( TWI_Transceiver_Busy() );             // Wait until TWI has completed the transmission.
  return ( TWI_state );                         // Return error state. 
}


/**
 * Call this function to send a prepared message, or start the Transceiver for reception. Include
 * a pointer to the data to be sent if a SLA+W is received. The data will be copied to the TWI
 * buffer.  Also include how many bytes that should be sent. Note that unlike the similar Master
 * function, the Address byte is not included in the message buffers.
 * The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
 * then initialize the next operation and return.
 * ---------------------------------------------------------------------------------------------- */
void TWI_Start_Transceiver_With_Data( unsigned char *msg, unsigned char msgSize )
{
    unsigned char temp;
    
    
    // Wait until TWI is ready for next transmission.
    while ( TWI_Transceiver_Busy() );
    
    // Number of data to transmit.
    TWI_msgSize = msgSize;
    
    // Copy data that may be transmitted if the TWI Master requests data.
    for ( temp = 0; temp < msgSize; temp++ )
    {
        TWI_buf[ temp ] = msg[ temp ];
    }
    
    TWI_statusReg.all = 0;      
    TWI_state         = TWI_NO_STATE ;
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
           (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Prepare to ACK next time the Slave is addressed.
           (0<<TWWC);                             //
}


/**
 * Call this function to start the Transceiver without specifing new transmission data.
 * Usefull for restarting a transmission, or just starting the transceiver for reception.
 * The driver will reuse the data previously put in the transceiver buffers. The function will
 * hold execution (loop) until the TWI_ISR has completed with the  previous operation, then 
 * initialize the next operation and return.
* ----------------------------------------------------------------------------------------------- */
void TWI_Start_Transceiver( void )
{
   
    // Wait until TWI is ready for next transmission.
    while ( TWI_Transceiver_Busy() );
    
    TWI_statusReg.all = 0;      
    TWI_state         = TWI_NO_STATE ;
    TWCR = (1<<TWEN)|                             // TWI Interface enabled.
           (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
           (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|       // Prepare to ACK next time the Slave is addressed.
           (0<<TWWC);                             //
}


/**
 * Call this function to read out the received data from the TWI transceiver buffer. I.e. first
 * call TWI_Start_Transceiver to get the TWI Transceiver to fetch data. Then Run this function to
 * collect the data when they have arrived. Include a pointer to where to place the data and 
 * the number of bytes to fetch in the function call. The function will hold execution (loop)
 * until the TWI_ISR has completed with the previous operation, before reading out the data
 * and returning. If there was an error in the previous transmission the function will return
 * the TWI State code.
 * ---------------------------------------------------------------------------------------------- */
unsigned char TWI_Get_Data_From_Transceiver( unsigned char *msg, unsigned char msgSize )
{
    unsigned char i;
    

    // Wait until TWI is ready for next transmission.
    while ( TWI_Transceiver_Busy() );

    // Last transmission competed successfully.
    if( TWI_statusReg.lastTransOK )              
    {
        // Copy data from Transceiver buffer.
        for ( i=0; i<msgSize; i++ )
        {
            msg[i] = TWI_buf[i];
        }
        
        // Slave Receive data has been read from buffer.
        TWI_statusReg.RxDataInBuf = FALSE;
    }
    
    return TWI_statusReg.lastTransOK;                                   
}



/**
 * This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is
 * triggered; that is whenever a TWI event has occurred. This function should not be called
 * directly from the main application.
 * ---------------------------------------------------------------------------------------------- */
SIGNAL((TWI_vect))
{
 
 static unsigned char TWI_bufPtr;
 
 switch (TWSR)
 {
	 // Own SLA+R has been received; ACK has been returned
	 case TWI_STX_ADR_ACK:
		 // Set buffer pointer to first data location
		 TWI_bufPtr   = 0;
 
	 // Data byte in TWDR has been transmitted; ACK has been received
	 case TWI_STX_DATA_ACK:
		 TWDR = TWI_buf[TWI_bufPtr++];
		 
		 // TWI Interface enabled
		 // Enable TWI Interupt and clear the flag to send byte
		 TWCR = (1<<TWEN) | 
				 (1<<TWIE)|(1<<TWINT)| 
				 (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
				 (0<<TWWC);
		 break;
		 
	 // Data byte in TWDR has been transmitted; NACK has been received.
	 // I.e. this could be the end of the transmission.
	 case TWI_STX_DATA_NACK:  
		 // Have we transceived all expected data?
		 if (TWI_bufPtr == TWI_msgSize) 
		 {
			 // Set status bits to completed successfully.
			 TWI_statusReg.lastTransOK = TRUE; 
		 }
		 else
		 {
			 // Master has sent a NACK before all data where sent.
			 // Store TWI State as errormessage.
			 TWI_state = TWSR;		
		 }
		 
		 // Put TWI Transceiver in passive mode.
		 // Enable TWI-interface and release TWI pins
		 TWCR = (1<<TWEN)|
				(1<<TWIE)|(1<<TWINT)|				 // Disable Interupt
				(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|	 // Do not acknowledge on any new requests.
				(0<<TWWC);
		 //return the u1 intc
		 PORTD &=~BIT(0);
		break;	   
		
	 // General call address has been received; ACK has been returned
	 case TWI_SRX_GEN_ACK:
 
		 TWI_statusReg.genAddressCall = TRUE;
	 // Own SLA+W has been received ACK has been returned
	 case TWI_SRX_ADR_ACK:
 
		 // Dont need to clear TWI_S_statusRegister.generalAddressCall due to that it is the default state.
		 TWI_statusReg.RxDataInBuf = TRUE;		
		 
		 // Set buffer pointer to first data location
		 TWI_bufPtr   = 0;
		 
		 // Reset the TWI Interupt to wait for a new event.
		 
		 // TWI Interface enabled
		 // Enable TWI Interupt and clear the flag to send byte
		 // Expect ACK on this transmission 
            TWCR = (1<<TWEN)|
            		(1<<TWIE)|(1<<TWINT)| 
            		(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)| 
            		(0<<TWWC);                                 //    
        break;
        
    // Previously addressed with own SLA+W; data has been received; ACK has been returned
    // Previously addressed with general call; data has been received; ACK has been returned
    case TWI_SRX_ADR_DATA_ACK:

    case TWI_SRX_GEN_DATA_ACK:

        TWI_buf[TWI_bufPtr++] = TWDR;
        
        // Set flag transmission successfull.
        TWI_statusReg.lastTransOK = TRUE;
        
        // Reset the TWI Interupt to wait for a new event.
        TWCR = (1<<TWEN)|                          // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|               // Enable TWI Interupt and clear the flag to send byte
               (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // Send ACK after next reception
               (0<<TWWC);
		
        break;
        
    // A STOP condition or repeated START condition has been received while still addressed as Slave
    case TWI_SRX_STOP_RESTART:  
        // Put TWI Transceiver in passive mode.
        TWCR = (1<<TWEN)|                          // Enable TWI-interface and release TWI pins
               (1<<TWIE)|(1<<TWINT)|               // Disable Interupt
               (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // Do not acknowledge on any new requests.
               (0<<TWWC);		
		if(TWI_buf[0] == TWI_buf[2])
		{
				Configs.fetch_no = 3;		
		}	
		else if(TWI_buf[0] == TWI_buf[1])
		{
				Configs.fetch_no = TWI_buf[0];	
		}

        break;
        
    // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
    case TWI_SRX_ADR_DATA_NACK:

    // Previously addressed with general call; data has been received; NOT ACK has been returned
    case TWI_SRX_GEN_DATA_NACK:

    // Last data byte in TWDR has been transmitted (TWEA = ?0?); ACK has been received
    case TWI_STX_DATA_ACK_LAST_BYTE:

    // Bus error due to an illegal START or STOP condition
    case TWI_BUS_ERROR:

    default:
		TWI_state = TWSR;								  // Store TWI State as errormessage, operation also clears the Success bit.	  
//				TWSR = TWI_NO_STATE;
		TWCR = (1<<TWEN)|								  // Enable TWI-interface and release TWI pins
			   (1<<TWIE)|(1<<TWINT)|					  // Disable Interupt
			   (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)| 		  // Do not acknowledge on any new requests.
			   (0<<TWWC);

    }
}

//外中断初始化
void int_init(void)
{
//int0
//EIMSK |=1<<INT0;
//EICRA |=(1<<ISC00)|(0<<ISC01);//level change mode

//int1
//EIMSK |=1<<INT1;
//EICRA |=(1<<ISC10)|(0<<ISC11);//level change mode

//config PD2...PD7 to pin change interrput
//	PCICR |=(1<<PCIF2);
//	PCMSK2 =1<<PCINT20;//PCINT20
}


SIGNAL(INT0_vect)
{
//	PORTD ^= BIT(1);	
	if(Configs.flag==0)
	{
		START_T1;
		EICRA =(0<<ISC00)|(1<<ISC01);//level change mode
		EIMSK = 1<<INT0;
		Configs.flag++;
//		PORTD ^= BIT(1);		
	}else
	{
		STOP_T1;
		EIMSK = 0;
		Configs.flag++;
//		PORTD ^= BIT(1);	
	}
}

SIGNAL(INT1_vect)
{
	if(PIND & BIT(3))
	{
		START_T1;
		EICRA =(0<<ISC10)|(1<<ISC11);//level change mode
//		PORTD ^= BIT(1);
	}
	else if(TCCR1B!=0)
	{
		STOP_T1;
		EIMSK =0;
		Configs.flag=1;
//		PORTD ^= BIT(1);
	}
}

//pin int handler
SIGNAL(PCINT2_vect)
{
	if(PIND & BIT(4))
	{	
		START_T1;
//		PORTD ^= BIT(1);
	}
	else if(TCCR1B!=0)
	{
		STOP_T1;
		PCICR=0;
		Configs.flag=1;
//		PORTD ^= BIT(1);
	}
}


void init_devices(void)
{
	cli(); //禁止所有中断
	MCUCR  = 0x00;
//	MCUCSR = 0x80;//禁止JTAG
//	GICR   = 0x00;
	port_init();
	timer0_init();
	timer1_init();
	spi_init();
	watchdog_init();
//initial the i2c interface address : 0x4a
	TWI_Slave_Initialise(0x4a<<1);
//pin change interrput
//	int_init();
//adc 
	adc_init();

	sei();//开全局中断
}

long Read_phase(uchar p)
{
	int cnt = 0;
	//default the tick
	Configs.ticks=0;
	Configs.flag=0;
	//set the intc
	switch(p)
	{
		case 0:PCMSK2 =0;PCICR=0;EICRA =(1<<ISC00)|(1<<ISC01);EIFR = 1;EIMSK =1<<INT0;break;
		case 1:PCMSK2 =0;PCICR=0;EICRA =(1<<ISC10)|(1<<ISC11);EIFR = 2;EIMSK =1<<INT1;break;
		//PD4  PCINT20
		case 2:EIMSK =0;PCMSK2 =1<<PCINT20;PCIFR = 1<<PCIF2;PCICR =(1<<PCIE2);break;
	}
	//wait the result
	while(Configs.flag!=2 && cnt++!=20) _delay_ms(100);//break out when time out 2s
	STOP_T1;
	Configs.phase[p] = Configs.ticks;
	if(Configs.phase[p])
		switch(p)
		{
			case 0:if(PIND & BIT(5)) Configs.phase[p]|=((long)1<<31);break;
			case 1:if(PIND & BIT(6)) Configs.phase[p]|=((long)1<<31);break;
			//PD4  PCINT20
			case 2:if(PIND & BIT(7)) Configs.phase[p]|=((long)1<<31);break;
		}	
	//return the result
	return Configs.phase[p];
}

void set_vol_base(int v)
{
	char i;
	for(i=0;i<10;i++)
		write(v);
}

void test()
{
//	PCICR=0;EICRA =(1<<ISC00)|(0<<ISC01);EIMSK =1<<INT0;
//EICRA =(1<<ISC10)|(0<<ISC11);EIFR = 2;EIMSK =1<<INT1;
//PCMSK2 =1<<PCINT20;PCIFR = 1<<PCIF2;PCICR =(1<<PCIE2);

//	EIMSK =0;PCICR =(1<<PCIE2);
//START_T1;

//set_vol_base(0x8000);

unsigned int temp=adc_calc();
ml.l = temp;
TWI_buf[0] = ml.c.a1;
TWI_buf[1] = ml.c.a2;
TWI_buf[2] = ml.c.a3;
TWI_buf[3] = ml.c.a4;
PORTD|=BIT(0);
_delay_ms(1000);
}

//主函数

void calibration(uint v)
{
	uchar i;
	double dt;
	double v1;
	i = 0;
	dt = 0;
	while(i++ != 10)
	{
		dt += adc_calc();
		_delay_ms(1);
	}
//	dt=0;
	dt /=10.;
	v1 = dt;
	VREF = (int)(dt*16.38);
	set_vol_base(0x8000+VREF);
	i = 0;
	dt = 0;
	while(i++ != 10)
	{
		dt += adc_calc();
		_delay_ms(1);
	}
	dt /=10;
	if(dt > v1)
		VREF = -VREF;	
	set_vol_base(0x8000+VREF);
}

int main(void)
{
	init_devices();
	TWI_Start_Transceiver();
	set_vol_base(0x8000);
	//在这继续添加你的代码
	//auto calibration	
	_delay_ms(100);
	calibration(0);
//	test();
	while(1)
	{
//		if(PIND &BIT(2))
//			PORTD |=BIT(1);
//		else
//			PORTD &=~BIT(1);	
//		calibration(0);
//		set_vol_base(0x8000);

//		test();
		switch(Configs.fetch_no)
		{
			case 0:
			case 1:
			case 2:	Read_phase(Configs.fetch_no);
					ml.l = Configs.phase[Configs.fetch_no];
					TWI_buf[0] = ml.c.a1;
					TWI_buf[1] = ml.c.a2;
					TWI_buf[2] = ml.c.a3;
					TWI_buf[3] = ml.c.a4;
					Configs.fetch_no = 0xff;PORTD&=~BIT(0);_delay_ms(1);PORTD|=BIT(0);break;			
			case 3:	Configs.base_voltage = (TWI_buf[0]<<8)+TWI_buf[1];
					set_vol_base(0x8000+VREF+Configs.base_voltage);
					Configs.fetch_no = 0xff;
					break;//set voltage base
			default:break;
		}	
	}
	return 0;
}


