//包含所需头文件
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>




/*------宏定义------*/
#define uchar	unsigned char
#define uint	unsigned int
#define BIT(x)	(1<<(x))
#define NOP()	asm("nop")
#define WDR() 	asm("wdr")



//端口初始化
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


//定时器T0初始化
void timer0_init(void)
{

}


//T0比较中断服务程序
//#pragma interrupt_handler timer0_comp_isr:11
//void timer0_comp_isr(void)
SIGNAL(SIG_OUTPUT_COMPARE0)
{
	//中断发生时刻在TCNT0=OCR0
}


//看门狗初始化
void watchdog_init(void)
{

}


void init_devices(void)
{
	cli(); //禁止所有中断
//	MCUCR  = 0x00;
//	MCUCSR = 0x80;//禁止JTAG
//	GICR   = 0x00;
	port_init();
	timer0_init();
	watchdog_init();
	sei();//开全局中断
}
//主函数
int main(void)
{
	init_devices();
	//在这继续添加你的代码
	while(1)
	{
	 NOP();
	}
	return 0;
}


