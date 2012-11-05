#include "io_conf.h"
#include <avr/io.h>
#include "macro.h"

void port_init(void)
{
	PORTA = 0x00;
	DDRA = 0x00 | BIT(3) | BIT(1) | BIT(0);
	PORTB = 0x00 | BIT(2) | BIT(3) | BIT(4);
	DDRB = 0xFF ^ BIT(6) ^ BIT(7) ^ BIT(5) ^ BIT(0);
	PORTC = 0x00;
	DDRC = 0x00;
	PORTD = 0x00;
	DDRD = 0x00 | BIT(0)|BIT(4);
	PORTE = 0x00; //|BIT(2)|BIT(3);
	DDRE = 0x00 | BIT(2) | BIT(3); //0x00 |BIT(4)|BIT(2)|BIT(3)|BIT(5)|BIT(6)|BIT(7);
	PORTF = 0x00 | BIT(0); //|BIT(0)|BIT(1);
	DDRF = 0x00 | BIT(0) | BIT(1) | BIT(2) | BIT(3);
	PORTG = 0x00;
	DDRG = 0x00 | BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4);
}
