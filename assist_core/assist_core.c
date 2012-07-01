//��������ͷ�ļ�
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>




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
}


//��ʱ��T0��ʼ��
void timer0_init(void)
{

}


//T0�Ƚ��жϷ������
//#pragma interrupt_handler timer0_comp_isr:11
//void timer0_comp_isr(void)
SIGNAL(SIG_OUTPUT_COMPARE0)
{
	//�жϷ���ʱ����TCNT0=OCR0
}


//���Ź���ʼ��
void watchdog_init(void)
{

}


void init_devices(void)
{
	cli(); //��ֹ�����ж�
//	MCUCR  = 0x00;
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


