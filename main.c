#include "msp.h"

 void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
    P1->SEL0 &= ~(BIT1|BIT4);  //initialize switches for speed up and slow down
    P1->SEL1 &= ~(BIT1|BIT4);
    P1->DIR  &= ~(BIT1|BIT4);
    P1->REN  |=   (BIT1|BIT4);
    P1->OUT |= (BIT4|BIT1);

    P1->SEL0 &= ~BIT0;
    P1->SEL1 &= ~BIT0;
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;


    while(1)
    {
        if(!(P1->IN & BIT4))
        {
            P1->OUT |= BIT0;
        }
        else
        {
            P1->OUT &= ~BIT0;
        }
    }

}
