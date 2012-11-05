#ifndef _MACRO_
#define _MACRO_

/*------∫Í∂®“Â------*/
#define uchar	unsigned char
#define uint	unsigned int
#define BIT(x)	(1<<(x))
#define NOP()	asm("nop")
#define WDR() 	asm("wdr")

#define		head	0x3C
#define		tail	0x3E
#define		success	0x01
#define		fail	0x10
#define		req_set		0x20
#define		req_on		0x30
#endif
