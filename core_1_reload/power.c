#include "power.h"
#include <avr/io.h>
#include "macro.h"
#include "USART.h"

extern void
xiezai(void);
extern volatile unsigned char commandPC[40];
extern volatile char flag_po; //电源指示

void my_power(void)
{
	if ((PINA & BIT(4)) == BIT(4))
	{
		commandPC[0] = head;
		commandPC[1] = 7;
		commandPC[2] = ~commandPC[1];
		commandPC[11] = tail;
		xiezai();
		puts_0((unsigned char *) commandPC, 12);
		flag_po = 1;
	}
}
