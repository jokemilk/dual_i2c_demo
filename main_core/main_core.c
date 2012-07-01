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
	PORTA = 0x00;
	DDRA  = 0x00;
	PORTB = 0x00;
	DDRB  = 0x00;
	PORTC = 0x00;
	DDRC  = 0x00;
	PORTD = 0x00;
	DDRD  = 0x00;
	PORTG = 0x00;
	DDRG = 0x00 | BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4);
}


//��ʱ��T0��ʼ��
void timer0_init(void)
{
	TCCR0  = 0x00;//ֹͣ��ʱ��
	TCNT0  = 0x00;//��ʼֵ
	OCR0   = 149;//ƥ��ֵ
	TIMSK |= 0x02;//�ж�����
	TCCR0  = 0x0F;//������ʱ��
}


//T0�Ƚ��жϷ������
//#pragma interrupt_handler timer0_comp_isr:11
//void timer0_comp_isr(void)
SIGNAL(SIG_OUTPUT_COMPARE0)
{
	static int i = 0;
	wdt_reset();
	if (i++ == 4)
	{
		i = 0;
		PORTG ^= BIT(1);
	}
}


//���Ź���ʼ��
void watchdog_init(void)
{
	WDR();//ι�� 
	WDTCR = 0x0F;//ʹ�ܿ��Ź�
}


void init_devices(void)
{
	cli(); //��ֹ�����ж�
	MCUCR  = 0x00;
	MCUCSR = 0x80;//��ֹJTAG
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


