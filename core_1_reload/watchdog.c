#include "watchdog.h"

//定时器T0初始化
void timer0_init(void)
{
	TCCR0 = 0x00; //停止定时器
	TCNT0 = 0x00; //初始值
	OCR0 = 149; //匹配值
	TIMSK |= 0x02; //中断允许
	TCCR0 = 0x0F; //启动定时器
}

void watchdog_ini()
{
	timer0_init();
	wdt_enable(WDTO_2S);
	wdt_reset();
	//喂狗
}
