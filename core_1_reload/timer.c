#include "timer.h"
#include <avr/io.h>
#include "macro.h"

//定时T1初始化
void timer1_init(void)
{
	TCCR1B = 0x00; //停止定时器
	TIMSK |= 0x10; //中断允许
	TCNT1H = 0x00;
	TCNT1L = 0x00; //初始值
	OCR1A = 24000 - 1; //24000->100ms
	OCR1BH = 0x00;
	OCR1BL = 0x03; //匹配B值
	ICR1H = 0xFF;
	ICR1L = 0xFF; //输入捕捉匹配值
	TCCR1A = 0x00;
//	TCCR1B = 0x09;//启动定时器
}
