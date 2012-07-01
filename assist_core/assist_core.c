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
	PORTB = 0x00;
	DDRB  = 0x00|BIT(0);
	PORTC = 0x00;
	DDRC  = 0x00;
	PORTD = 0x00;
	DDRD  = 0x00;
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


//T0比较中断服务程序
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


//看门狗初始化
void watchdog_init(void)
{
	WDR (); //this prevents a timeout on enabling
	WDTCSR |= (1<<WDCE) | (1<<WDE);/* 30-Oct-2006 Umesh*/  
	WDTCSR = 0x1F; //WATCHDOG ENABLED - dont forget to issue WDRs
}


void init_devices(void)
{
	cli(); //禁止所有中断
	MCUCR  = 0x00;
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


