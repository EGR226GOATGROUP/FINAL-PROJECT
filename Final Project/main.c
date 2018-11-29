#include "msp.h"

//Nolan Test from Web
//Nolan test from home
//Nolan Test from Laptop

/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

}
