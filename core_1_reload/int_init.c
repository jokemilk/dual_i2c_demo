#include "int_init.h"
#include <avr/io.h>
//���жϳ�ʼ��
void int_init(void)
{
	 EICRB = 0xFF; //extended ext ints rising edge for int 4 5 6 7
	 EIMSK |= 1<<INT7;	//ennable ints
}
