//��������ͷ�ļ�
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h> 



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
	DDRB  = 0x00|BIT(0);
	PORTC = 0x00;
	DDRC  = 0x00;
	PORTD = 0x00;
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


void init_devices(void)
{
	cli(); //��ֹ�����ж�
	MCUCR  = 0x00;
//	MCUCSR = 0x80;//��ֹJTAG
//	GICR   = 0x00;
	port_init();
	timer0_init();
	watchdog_init();
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


