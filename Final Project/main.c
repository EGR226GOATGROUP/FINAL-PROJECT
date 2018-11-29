#include "msp.h"
#include <stdio.h>
#include <string.h>


void configRTC(void);
void printRTC(void);


// global struct variable called now
struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} now;

uint8_t RTC_flag = 0, RTC_alarm;

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    __disable_irq();
    configRTC();
    NVIC_EnableIRQ(RTC_C_IRQn);
    __enable_irq();

    while(1)
    {
        if(RTC_flag)
        {
            printRTC();
            RTC_flag = 0;
        }
        if(RTC_alarm)
        {
            printf("ALARM\n");
            RTC_alarm = 0;
        }
    }
}

void configRTC(void)
{
    RTC_C->CTL0     =   0xA500;     //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;
    RTC_C->TIM0     = 00<<8 | 00;
    RTC_C->TIM1     = 2<<8 | 12;

    RTC_C->PS1CTL   = 0b11010;

    RTC_C->AMINHR   = 12<<8 | 31 | BIT(15) | BIT(7);

    RTC_C->CTL0     = ((0xA500) | BIT5);

}

void printRTC(void)
{
    printf("%02d:%02d:%02d\n",now.hour, now.min, now.sec);

}

void RTC_C_IRQHandler(void)
{
    if(RTC_C->CTL0 & BIT1) {
        RTC_alarm = 1;
        RTC_C->CTL0 = 0xA500;
    }
    if(RTC_C->PS1CTL & BIT0) {
        now.sec         =   RTC_C->TIM0>>0 & 0x00FF;
        now.min         =   RTC_C->TIM0>>8 & 0x00FF;
        now.hour        =   RTC_C->TIM1>>0 & 0x00FF;
        RTC_flag = 1;
        RTC_C->PS1CTL &= ~BIT0;
    }

}




