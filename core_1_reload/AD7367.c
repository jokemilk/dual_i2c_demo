#include "AD7367.h"
#include <avr/io.h>
#include <util/delay.h>
#include "macro.h"

extern unsigned int Vw;
extern unsigned int Vr;
extern unsigned int Vi;
extern unsigned int Ad;

extern void
delay(int ms);

void AD_getdata(unsigned char num) // 0 vw 1 vr 2 vi 3 ad
{
	int i;
//	unsigned char CK;
	switch (num)
	{
	case 0:
	{
		Vw = 0;
		PORTG &= ~BIT(3);
		PORTB |= BIT(2) | BIT(4);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		_delay_us(10);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		while ((PINB & BIT(7)) != 0)
			;
		_delay_us(5);
		PORTB &= ~BIT(2);
		for (i = 0; i < 14; i++)
		{
			Vw = Vw << 1;
			_delay_us(10);
			PORTB &= ~BIT(4);
			_delay_us(5);
			if ((PINB & BIT(5)) != 0)
				Vw++;
			_delay_us(5);
			PORTB |= BIT(4);
		}
		_delay_us(5);
		PORTB |= BIT(2);
	}
		break;
	case 1:
	{
		Vr = 0;
		PORTG |= BIT(3);
		PORTB |= BIT(2) | BIT(4);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		_delay_us(10);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		while ((PINB & BIT(7)) != 0)
			;
		_delay_us(5);
		PORTB &= ~BIT(2);
		for (i = 0; i < 14; i++)
		{
			Vr = Vr << 1;
			_delay_us(10);
			PORTB &= ~BIT(4);
			_delay_us(5);
			if ((PINB & BIT(5)) != 0)
				Vr++;
			_delay_us(5);
			PORTB |= BIT(4);
		}
		_delay_us(5);
		PORTB |= BIT(2);
	}
		break;
	case 2:
	{
		Vi = 0;
		PORTG &= ~BIT(3);
		PORTB |= BIT(2) | BIT(4);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		_delay_us(10);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		while ((PINB & BIT(7)) != 0)
			;
		_delay_us(5);
		PORTB &= ~BIT(2);
		for (i = 0; i < 14; i++)
		{
			Vi = Vi << 1;
			_delay_us(10);
			PORTB &= ~BIT(4);
			_delay_us(5);
			if ((PINB & BIT(6)) != 0)
				Vi++;
			_delay_us(5);
			PORTB |= BIT(4);
		}
		_delay_us(5);
		PORTB |= BIT(2);
	}
		break;
	case 3:
	{
		Ad = 0;
		PORTG |= BIT(3);
		PORTB |= BIT(2) | BIT(4);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		_delay_us(10);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		while ((PINB & BIT(7)) != 0)
			;
		_delay_us(5);
		PORTB &= ~BIT(2);
		for (i = 0; i < 14; i++)
		{
			Ad = Ad << 1;
			_delay_us(10);
			PORTB &= ~BIT(4);
			_delay_us(5);
			if ((PINB & BIT(6)) != 0)
				Ad++;
			_delay_us(5);
			PORTB |= BIT(4);
		}
		_delay_us(5);
		PORTB |= BIT(2);
	}
		break;
	case 4:
	{
		Vw = 0;
		Vi = 0;
		PORTG &= ~BIT(3);
		PORTB |= BIT(2) | BIT(4);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		_delay_us(10);

		PORTG |= BIT(3);

		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		while ((PINB & BIT(7)) != 0)
			;
		_delay_us(5);
		PORTB &= ~BIT(2);
		for (i = 0; i < 14; i++)
		{
			Vw = Vw << 1;
			Vi = Vi << 1;
			_delay_us(10);
			PORTB &= ~BIT(4);
			_delay_us(5);
			if ((PINB & BIT(5)) != 0)
				Vw++;
			if ((PINB & BIT(6)) != 0)
				Vi++;
			_delay_us(5);
			PORTB |= BIT(4);
		}
		_delay_us(5);
		PORTB |= BIT(2);
		_delay_us(20);
/////////////////////////////////////////////////////////////////////////////////
		Vr = 0;
		Ad = 0;
		PORTB &= ~BIT(3);
		_delay_us(20);
		PORTB |= BIT(3);
		while ((PINB & BIT(7)) != 0)
			;
		_delay_us(5);
		PORTB &= ~BIT(2);
		for (i = 0; i < 14; i++)
		{
			Vr = Vr << 1;
			Ad = Ad << 1;
			_delay_us(10);
			PORTB &= ~BIT(4);
			_delay_us(5);
			if ((PINB & BIT(5)) != 0)
				Vr++;
			if ((PINB & BIT(6)) != 0)
				Ad++;
			_delay_us(5);
			PORTB |= BIT(4);
		}
		_delay_us(5);
		PORTB |= BIT(2);
	}
		break;
	default:
		break;
	}
//	PORTB =	PORTB|BIT(3)|BIT(4);
}

