/*
 * Pin Out
 *
 * Temp Sensor
 * 5.5 -> Temp Sensor
 *
 * Buttons
 * 4.0 -> SetTime
 * 4.1 -> On/Off/Up
 * 4.2 -> AlarmSet
 * 4.3 -> Snozze/Down
 *
 * LCD
 * 7.0 -> RS
 * 7.2 -> E
 * 7.4 -> DB4
 * 7.5 -> DB5
 * 7.6 -> DB6
 * 7.7 -> DB7
 *
 */

//Nolan Comment
//Nolan Test

#include "msp.h"
#include <stdio.h>
#include <string.h>

#define ADC_CONVERSION_RATE 1500000
#define CLEAR 0x01
#define SETTIME BIT0
#define ALARM BIT1
#define UP BIT1
#define SETALARM BIT2
#define SNOOZE BIT3
#define DOWN BIT3

void configRTC(int hour, int min);
void ADC14init(void);
void tempT32interrupt(void);
void intButtons();


void displayText(char text[], int lineNum);
void displayAt(char text[], int place, int line);
void LCD_init(void);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);
void pushByte(uint8_t byte);
void pushNibble(uint8_t nibble);
void pulseEnablePin(void);
void sysTickDelay_ms(int ms);
void sysTickDelay_us(int microsec);
void SysTick_Init();

int timePresses=0, alarmPresses=0;
float temp=0, voltage = 0, raw = 0;
char time[2],tempAr[3];
int RTC_flag =0;

// global struct variable called now
struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} now;

//-----------------------------------------MAIN----------------------------------------------------

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    intButtons();
    SysTick_Init();                                 //initializes timer
    tempT32interrupt();
    LCD_init();                                     //initializes LCD
    ADC14init();
    __enable_interrupt();

    configRTC(12, 30);
    commandWrite(CLEAR);
    while(1)
    {

    }
}

//--------------------------------------------------Interrupts------------------------------------------------------------------------

void T32_INT1_IRQHandler()                          //Interrupt Handler for Timer 2
{
    TIMER32_1->INTCLR = 1;                          //Clear interrupt flag so it does not interrupt again immediately.
    ADC14->CTL0         |=  0b1;                    //Start ADC Conversion

}

void PORT4_IRQHandler()
{
    //SetTime Button Press
    if(P4->IFG & SETTIME)
    {
        displayAt("SETTIME    ", 4, 2);             //Debugging Display

        if(timePresses==0)                                  //First press -> Go into time Hours edit
        {
            //disable timer clock
        }
        else if(timePresses==3)                             //Third press -> Save time
        {
            //enable timer clock
            //save new time
        }

        timePresses++;
        P4->IFG &= ~SETTIME;
    }

    //Alarm toggle/Up Button Press
    if(P4->IFG & (ALARM|UP))
    {
        if(timePresses==1)                              //Incoment Hours
        {
            now.hour++;
            if(now.hour>12)                                 //hour Rolls Over
                now.hour=1;
        }
        else if(timePresses==2)                         //incoment Minutes
        {
            now.min++;
            if(now.min>59)
                now.min=0;                                  //Minutes roll over
        }

        displayAt("ALARM    ", 4, 2);               //Debugging Display
        P4->IFG &= ~(ALARM|UP);
    }

    //SetAlarm Button Press
    if(P4->IFG & SETALARM)                              //SetAlarm Button Press
    {
        displayAt("SETALARM   ", 4, 2);             //Debugging Display


        alarmPresses++;
        P4->IFG &= ~SETALARM;
    }

    //Snooze/Down Button Press
    if(P4->IFG & SNOOZE)
    {
        displayAt("SNOOZE    ", 4, 2);              //Debugging Display
        P4->IFG &= ~SNOOZE;
    }
}

void RTC_C_IRQHandler(void)
{
    if(RTC_C->PS1CTL & BIT0) {
        now.sec         =   RTC_C->TIM0>>0 & 0x00FF;
        now.min         =   RTC_C->TIM0>>8 & 0x00FF;
        now.hour        =   RTC_C->TIM1>>0 & 0x00FF;
        RTC_flag = 1;
        RTC_C->PS1CTL &= ~BIT0;
        sprintf(time,"%.2d",now.hour);
        displayAt(time,4,1);
        commandWrite(134);
        dataWrite(0b00111010);
        sprintf(time,"%.2d",now.min);
        displayAt(time,7,1);
        commandWrite(137);
        dataWrite(0b00111010);
        sprintf(time,"%.2d",now.sec);
        displayAt(time,10,1);
    }
}

void ADC14_IRQHandler(void)
{

    if(ADC14->IFGR0 & BIT0)
    {
            raw = ADC14->MEM[0];
            ADC14->CLRIFGR0     &=  ~BIT1;                  // Clear MEM1 interrupt flag
            voltage = raw*(3.3/16383);
            temp  = (1000*voltage - 500)/10;
            temp = ((temp*9.0)/5.0)+32.0;
            sprintf(tempAr,"%.1f",temp);
            displayAt(tempAr,4,4);
            commandWrite(216);
            dataWrite(0b11011111);
            commandWrite(217);
            dataWrite(0b01000110);

    }
    ADC14->CLRIFGR1     &=    ~0b1111110;                 // Clear all IFGR1 Interrupts (Bits 6-1.  These could trigger an interrupt and we are checking them for now.)
}

//------------------------------------------------------------Initilizations-----------------------------------------------------------------------
void intButtons()
{
    P4->SEL0 &= ~(BIT0|BIT1|BIT2|BIT3);
    P4->SEL1 &= ~(BIT0|BIT1|BIT2|BIT3);
    P4->DIR  &= ~(BIT0|BIT1|BIT2|BIT3);
    P4->REN  |=  (BIT0|BIT1|BIT2|BIT3);
    P4->OUT  |=  (BIT0|BIT1|BIT2|BIT3);
    P4->IE   |=  (BIT0|BIT1|BIT2|BIT3);
    P4->IES  |=  (BIT0|BIT1|BIT2|BIT3);

    NVIC_EnableIRQ(PORT4_IRQn);
}

void configRTC(int hour, int min)
{
    RTC_C->CTL0     =   0xA500;     //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;
    RTC_C->TIM0     = min<<8 | 00;
    RTC_C->TIM1     = hour;

    RTC_C->PS1CTL   = 0b11010;

    RTC_C->AMINHR   = 12<<8 | 31 | BIT(15) | BIT(7);

    RTC_C->CTL0     = ((0xA500) | BIT5);
    NVIC_EnableIRQ(RTC_C_IRQn);
}

void ADC14init(void)
{
    //For Analog Input 8
    P5->SEL0            |=   BIT5;                      // Select ADC Operation
    P5->SEL1            |=   BIT5;                      // SEL = 11 sets to ADC operation
    P5->DIR             &=  ~BIT5;
    ADC14->CTL0         =    0;                         // Disable ADC for setup

    // CTL0 Configuration
    // 31-30 = 10   to divide down input clock by 32X
    // 26    = 1    to sample based on the sample timer.  This enables the use of bits 11-8 below.
    // 21-19 = 100  for SMCLK
    // 18-17 = 00   one channel
    // 11-8  = 0011 for 32 clk sample and hold time
    // 7     = 1    for multiple ADC channels being read
    // 4     = 1    to turn on ADC
    ADC14->CTL0         =    0b10000100001000000000001100010000;

    ADC14->CTL1         =    (BIT5 | BIT4);         // Bits 5 and 4 = 11 to enable 14 bit conversion
                                                        // Bit 23 turns on Temperature Sensor
    ADC14->MCTL[0]      =    0;                         // A0 on P5.5
                                                        // BIT7 says to stop converting after this ADC.
    ADC14->IER0         |=   BIT0;            // Interrupt on conversion
//    REF_A->CTL0         |=   BIT0;                      // Turns on the Reference for the Temperature Sensor
//    REF_A->CTL0         &=   ~BIT3;                     // Turns off the Temperature Sensor Disable

    ADC14->CTL0         |=   0b10;                      // Enable Conversion
  NVIC_EnableIRQ(ADC14_IRQn);             // Turn on ADC Interrupts in NVIC.  Equivalent to "NVIC_EnableIRQ(ADC14_IRQn);"
}

void tempT32interrupt(void)
{
    TIMER32_1->LOAD       =   ADC_CONVERSION_RATE;        //Set interval for interrupt to occur at
    TIMER32_1->CONTROL       =   0b11100010;
    NVIC_EnableIRQ(T32_INT1_IRQn);
}

//---------------------------------------------------------------------------LCD Displaying functions----------------------------------------------

void displayAt(char text[], int place, int lineNum)
{
    int i;
    if(lineNum == 1)
    {
            commandWrite(place+128);
            for(i=0; i<strlen(text);i++)
            {
                dataWrite(text[i]);
            }
    }
    else if(lineNum == 2)
    {
            commandWrite(place+192);
            for(i=0; i<strlen(text);i++)
            {
                dataWrite(text[i]);
            }
    }
    else if(lineNum == 3)
    {
            commandWrite(place+144);
            for(i=0; i<strlen(text);i++)
            {
                dataWrite(text[i]);
            }
    }
    else if(lineNum == 4)
    {
            commandWrite(place+208);
            for(i=0; i<strlen(text);i++)
            {
                dataWrite(text[i]);
            }
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

//----------------------------------------------------------LCD Back end----------------------------------------------------------------------------

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
    commandWrite(CLEAR); //clear display and move cursor home
    sysTickDelay_us(100);
    commandWrite(0x06); //increment cursor
    sysTickDelay_ms(10);

    commandWrite(0x0C); //2 lines
    sysTickDelay_us(100);

}

void commandWrite(uint8_t command) //writes a command to LCD using RS = 0
{
    P7->OUT &= ~BIT0; //rs=0
    sysTickDelay_ms(5);
    P7->OUT &= ~0xF0; //clears data pins
    sysTickDelay_us(10);
    pushByte(command);
    sysTickDelay_us(10);
}

void dataWrite(uint8_t data) //writes data to LCD using RS = 1
{
    P7->OUT |= BIT0; //rs=1
    sysTickDelay_ms(5);
    P7->OUT &= ~0xF0; //clears data pins
    sysTickDelay_us(10);
    pushByte(data);
    sysTickDelay_us(10);
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
    sysTickDelay_ms(5);
    P7->OUT |= (nibble & 0xF0); //sends a nibble through pins 4-7
    sysTickDelay_us(10);
    pulseEnablePin();
}

void pulseEnablePin(void) //sends data to LCD by pulsing enable pin on and off
{
    P7->OUT &= ~BIT2; //enable low
    sysTickDelay_us(10);
    P7->OUT |= BIT2; //enable high
    sysTickDelay_us(10);
    P7->OUT &= ~BIT2; //enable low
    sysTickDelay_us(10);
}

void SysTick_Init(void) //initializes systick timer
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF; //sets max value
    SysTick->VAL = 0; //sets min value
    SysTick->CTRL = 0x00000005; //enables timer
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




