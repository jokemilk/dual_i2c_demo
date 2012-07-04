//��������ͷ�ļ�
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h> 
//#include "TWI_Master.h"
#include "twi_slave.h"

/*------�궨��------*/
#define uchar	unsigned char
#define uint	unsigned int
#define BIT(x)	(1<<(x))
#define NOP()	asm("nop")
#define WDR() 	asm("wdr")



//�˿ڳ�ʼ��
void port_init(void)
{
	PORTB = 0x00;
	DDRB  = 0x00|BIT(0)|BIT(2)|BIT(3);
	PORTC = 0x00;
	DDRC  = 0x00;
	PORTD = 0x00|BIT(1);
	DDRD  = 0x00;
}


//��ʱ��T0��ʼ��
void timer0_init(void)
{
/*
	TCCR0B  = 0x00;//ֹͣ��ʱ��
	TCNT0  = 0x00;//��ʼֵ
	OCR0A   = 200;//ƥ��ֵ
	TCCR0A = 0x00;
	TIMSK0 = 0x02; //timer 0 interrupt sources
	TCCR0B = 0x05;//�򿪼�ʱ��
	*/
	TCCR0B  = 0x00;//ֹͣ��ʱ��
	TIMSK0 = _BV(OCIE0A);  // Enable Interrupt TimerCounter0 Compare Match A (SIG_OUTPUT_COMPARE0A)
    TCCR0A = _BV(WGM01);  // Mode = CTC
    TCCR0B = _BV(CS02) | _BV(CS00);   // Clock/1024, 
    OCR0A = 149;          // 

}

//SPI��ʼ��
void spi_init(void)
{
	//spi��ʼ��
	SPCR = 0x54;
	SPSR = 0x01;
}

//����:ʹ��SPI����һ���ֽ�
void spi_write(uchar sData)
{
	SPDR = sData;
	while(!(SPSR & BIT(SPIF)));
	//sData=SPDR;//���ӻ�������������
}


//����:ʹ��SPI����һ���ֽ�
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
	//adcת����ʼ��
	ADCSRA	= 0x00;	//��ֹADת��
	ADMUX	= 0x00;
    DIDR0 &=~BIT(0);
	ACSR	= 0x80; //��ֹģ��Ƚ���
	ADCSRA	= 0x84;
}


unsigned int adc_calc(void)
{
	//����ʵ�ʵ�ѹ
	unsigned long value=0;
	unsigned int voltage=0; //��ѹ��λΪ(mV)
	ADCSRA|=_BV(ADSC);
	while(ADCSRA&_BV(ADSC));
	value=ADCL;		 //���ȶ���λ
	value|=(int)ADCH << 8; //Ȼ�����λ
	voltage=(value*2560)>>10;
	return voltage;
}

//T0�Ƚ��жϷ������
//#pragma interrupt_handler timer0_comp_isr:11
//void timer0_comp_isr(void)
SIGNAL(SIG_OUTPUT_COMPARE0A)
{
	static int i = 0;
	wdt_reset();
	if (i++ == 4)
	{
		i = 0;
		PORTB ^= BIT(0);
	}
}


//���Ź���ʼ��
void watchdog_init(void)
{
	WDR (); //this prevents a timeout on enabling
	WDTCSR |= (1<<WDCE) | (1<<WDE);/* 30-Oct-2006 Umesh*/  
	WDTCSR = 0x1F; //WATCHDOG ENABLED - dont forget to issue WDRs
}

static unsigned char TWI_buf[TWI_BUFFER_SIZE];     // Transceiver buffer. Set the size in the header file
static unsigned char TWI_msgSize  = 0;             // Number of bytes to be transmitted.
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
SIGNAL(SIG_2WIRE_SERIAL)
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
            TWCR = (1<<TWEN) | (1<<TWIE)|(1<<TWINT)| (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)| (0<<TWWC);
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
                   (0<<TWIE)|(0<<TWINT)|                // Disable Interupt
                   (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|     // Do not acknowledge on any new requests.
                   (0<<TWWC);
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
            TWCR = (1<<TWEN)| (1<<TWIE)|(1<<TWINT)| (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)| (0<<TWWC);                                 //      
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
               (0<<TWIE)|(0<<TWINT)|               // Disable Interupt
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // Do not acknowledge on any new requests.
               (0<<TWWC);
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

        // Store TWI State as errormessage, operation also clears the Success bit.
        //TWI_state = TWSR;  
        
        // transmit a stop condition. With this, the master gets in a valid state again
        // (because he may not get an bus error)
        TWCR = (1<<TWEN)|                          // Enable TWI-interface and release TWI pins
               (1<<TWIE)|(1<<TWINT)|               // Disable Interupt
               (1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|    // Do not acknowledge on any new requests.
               (0<<TWWC);
    }
}

//���жϳ�ʼ��
void int_init(void)
{
//config PD2...PD7 to pin change interrput
	PCICR |=(1<<PCIF2);
	PCMSK2 |=0xFC;
}


//pin int handler
SIGNAL(SIG_PIN_CHANGE2)
{

}

void init_devices(void)
{
	cli(); //��ֹ�����ж�
	MCUCR  = 0x00;
//	MCUCSR = 0x80;//��ֹJTAG
//	GICR   = 0x00;
	port_init();
	timer0_init();
	watchdog_init();
//initial the i2c interface address : 0x4a
	TWI_Slave_Initialise(0x4a<<1);
//pin change interrput
	int_init();
//adc 
	adc_init();
	sei();//��ȫ���ж�
}
//������
int main(void)
{
	init_devices();
	//������������Ĵ���
	while(1)
	{
	 NOP();
	}
	return 0;
}


