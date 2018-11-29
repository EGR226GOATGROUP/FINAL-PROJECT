#include "msp.h"
#include <stdio.h>
#include <string.h>


void configRTC(void);
void printRTC(void);


void LCD_init(void);
void commandWrite(uint8_t command);
void pushByte(uint8_t byte);
void pushNibble(uint8_t nibble);
void pulseEnablePin(void);
void sysTickDelay_ms(int ms);
void sysTickDelay_us(int microsec);
void dataWrite(uint8_t data);
void displayText(char text[], int lineNum);
// global struct variable called now
struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} now;

uint8_t RTC_flag = 0, RTC_alarm;


void SysTick_Init(void) //initializes systick timer
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF; //sets max value
    SysTick->VAL = 0; //sets min value
    SysTick->CTRL = 0x00000005; //enables timer
}

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    __disable_irq();
    configRTC();
    NVIC_EnableIRQ(RTC_C_IRQn);
    __enable_irq();
    SysTick_Init(); //initializes timer
    LCD_init(); //initializes LCD

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











void displayText(char text[], int lineNum) //function to display text on a given line
{

    int i;
    if(lineNum == 1)
    {
        for(i = 0; i<16; i++) //displays text on line one
        {
            dataWrite(text[i]);
        }

    }
    else if(lineNum == 2) //displays text on line 2 by starting at 0xA8
        {

            commandWrite(0xA8);
            for(i = 0; i<16; i++)
            {
                dataWrite(text[i]);
            }

        }
    else if(lineNum == 3)  //displays text on line 3 by starting at 0x90
        {

            commandWrite(0x90);
            for(i = 0; i<16; i++)
            {
                dataWrite(text[i]);
            }

        }
    else if(lineNum == 4) //displays text on line 4 by starting at 0xD0
        {

            commandWrite(0xD0);
            for(i = 0; i<16; i++)
            {
                dataWrite(text[i]);
            }

        }
}

void LCD_init(void) //initializes LCD
{
    P7->SEL0 &= ~0xFF; //port 4 GPIO
    P7->SEL1 &= ~0xFF;
    P7->DIR  |=  0xFF;

    sysTickDelay_us(10);

    commandWrite(0x03);  //reset sequence
    sysTickDelay_ms(100);
    commandWrite(0x03);
    sysTickDelay_us(200);
    commandWrite(0x03);
    sysTickDelay_ms(100);

    commandWrite(0x02); //4 bit mode to accept 2 nibbles rather than one byte
    sysTickDelay_us(100);
    commandWrite(0x02);
    sysTickDelay_us(100);





    commandWrite(0x8); //2 lines
    sysTickDelay_us(100);
    commandWrite(0x0F); //display on cursor on and blinking
    sysTickDelay_us(100);
    commandWrite(0x01); //clear display and move cursor home
    sysTickDelay_us(100);
    commandWrite(0x06); //increment cursor
    sysTickDelay_ms(10);

}

void commandWrite(uint8_t command) //writes a command to LCD using RS = 0
{
    P7->OUT &= ~BIT0; //rs=0
    sysTickDelay_ms(5);
    P7->OUT &= ~0xF0; //clears data pins
    sysTickDelay_us(5);
    pushByte(command);
    sysTickDelay_us(5);

}

void dataWrite(uint8_t data) //writes data to LCD using RS = 1
{
    P7->OUT |= BIT0; //rs=1
    sysTickDelay_ms(5);
    P7->OUT &= ~0xF0; //clears data pins
    sysTickDelay_us(5);
    pushByte(data);
    sysTickDelay_us(5);
}

void pushByte(uint8_t byte) //stores 4 bits into variable and sends nibble to push nibble twice for each half of the byte
{
    pushNibble((byte & 0xF0)); //sends first nibble
    sysTickDelay_ms(5);
    pushNibble((byte & 0x0F)<<4); //shifts second nibble and sends it through pins 4-7
}

void pushNibble(uint8_t nibble) //sets nibble up in data pins for pulse
{
    P7->OUT &= ~0xF0; //clearing data pins
    sysTickDelay_us(10);
    P7->OUT |= (nibble & 0xF0); //sends a nibble through pins 4-7
    sysTickDelay_us(5);
    pulseEnablePin();
}

void pulseEnablePin(void) //sends data to LCD by pulsing enable pin on and off
{
    P7->OUT &= ~BIT2; //enable low
    sysTickDelay_us(5);
    P7->OUT |= BIT2; //enable high
    sysTickDelay_us(5);
    P7->OUT &= ~BIT2; //enable low
    sysTickDelay_us(10);


}

void sysTickDelay_ms(int ms) //timer ms
{
    SysTick->LOAD = ((ms*3000)-1);
    SysTick->VAL = 0;
    while((SysTick->CTRL & BIT(16))==0);

}

void sysTickDelay_us(int microsec) //timer microseconds
{
    SysTick->LOAD = ((microsec*3)-1);
    SysTick->VAL = 0;
    while((SysTick->CTRL & BIT(16))==0);
}




