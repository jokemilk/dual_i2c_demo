//包含所需头文件
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h> 


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
	PORTG = 0x00;
	DDRG = 0x00 | BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4);
}


//定时器T0初始化
void timer0_init(void)
{
	TCCR0  = 0x00;//停止定时器
	TCNT0  = 0x00;//初始值
	OCR0   = 149;//匹配值
	TIMSK |= 0x02;//中断允许
	TCCR0  = 0x0F;//启动定时器
}


//T0比较中断服务程序
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


//看门狗初始化
void watchdog_init(void)
{
	WDR();//喂狗 
	WDTCR = 0x0F;//使能看门狗
}


void init_devices(void)
{
	cli(); //禁止所有中断
	MCUCR  = 0x00;
	MCUCSR = 0x80;//禁止JTAG
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


