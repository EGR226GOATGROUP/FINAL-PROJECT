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
 * 4.3 -> Snooze/Down
 *
 * LCD
 * 7.0 -> RS
 * 7.2 -> E
 * 7.4 -> DB4
 * 7.5 -> DB5
 * 7.6 -> DB6
 * 7.7 -> DB7
 *
 * WAKE UP LIGHTS
 * P2.4 -> RED     TIMERA0.1
 * P2.5 -> GREEN   TIMERA0.2
 * P2.6 -> BLUE    TIMERA0.3
 *
 *
 *P2.7 -> PWM LCD BRIGHTNESS
 *P9.3 -> TIMERA3.4 BLINK TIMER DO NOT USE PIN
 */

/*
 * Process for Sending updates
 *
 * Copy from local -> main.c (Git Repository)
 * Save the main.c (Git Repository)
 * Commit and Push
 * If there is error -> Pull
 * Save the main.c (Git Repository)
 * Make change to save again
 * Commit and Push again
 *
 */

#define __SYSTEM_CLOCK 3000000

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
#define LOW 0
#define HIGH 1

void configRTC(int hour, int min);
void ADC14init(void);
void tempT32interrupt(void);
void intButtons();
void intAlarm();
void intSpeedButton();

void displayAMPM();
void toggleAMPM();
void displayHour();
void toggleAlarm();
void displayAlarm();
void toggleAMPM2();


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
void LED_init(void);
void wakeUpLights(void);
void LEDT32interrupt(void);
void intLCDBrightness(void);
void intBlinkTimerA(void);

float tempC=0,tempF = 0, voltage = 0, raw = 0,raw1 = 0,voltage1 = 0;
char time[2],tempAr[3];
int RTC_flag =0, timePresses=0, alarmFlag=0, alarmPresses=0, speed=0;
int AMPM = 1, AMPM2=1;                       //flag to determine AM or PM will be used more for UART functionality to convert 24 hr to 12 hr time
int lightsOn = 0;                   //flag to be used to check if the wake up lights should be turned on
int lightBrightness = 0;
float LCDbrightness = 50;
int blinkFlag = 0;

// global struct variable called now
struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} now;

struct
{
    uint8_t min;
    uint8_t hour;
} alarm;

//-------------------------------------------------MAIN---------------------------------------------------------------

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    //Initilizing Interupts
    __disable_interrupt();
    intButtons();
    SysTick_Init();                                 //initializes timer
    tempT32interrupt();
    LEDT32interrupt();
    intBlinkTimerA();
    LCD_init();                                     //initializes LCD
    ADC14init();
    LED_init();
    intLCDBrightness();
    intSpeedButton();

    commandWrite(CLEAR);

    intAlarm();
    P1->SEL0 &= ~BIT0;
    P1->SEL1 &= ~BIT0;
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;
    TIMER_A0->CCR[4] = 1000;
    __enable_interrupt();
    configRTC(5, 59);

    lightsOn = 1;


    while(1)
    {

    }
}

//--------------------------------------------------Non-Interrupt Functions-----------------------------------------------------------

void wakeUpLights(void)
{
    TIMER32_2->CONTROL |= BIT7;
}

void toggleAlarm()
{
    if(alarmFlag)
    {
        alarmFlag=0;
        displayAt("OFF",10,3);
    }

    else
    {
        alarmFlag=1;
        displayAt("ON ",10,3);
    }
}

void displayAlarm()
{
    if(alarm.hour<10)
    {
        sprintf(time," %d",alarm.hour);
        displayAt(time,3,2);
    }

    else
    {
        sprintf(time,"%.2d",alarm.hour);
        displayAt(time,3,2);
    }

    sprintf(time,"%.2d",alarm.min);
    displayAt(time,6,2);
}

void toggleAMPM2()
{
    if(AMPM2)
    {
        AMPM2=0;
        displayAt("PM",10,2);
    }
    else
    {
        AMPM2=1;
        displayAt("AM",10,2);
    }
}

void toggleAMPM()                                   //Toggle AMPM when hours roll over
{
    if(AMPM)
        AMPM=0;
    else
        AMPM=1;
}

void displayAMPM()
{
    if(AMPM)                                        //prints AM or PM based on flag variable
    {
        sprintf(time,"PM");
        displayAt(time,12,1);
    }
    else if(!AMPM)
    {
        sprintf(time,"AM");
        displayAt(time,12,1);
    }
}

void displayMin()                                   //Displays the minutes
{
    sprintf(time,"%.2d",now.min);
    displayAt(time,5,1);
}

void displayHour()                                  //Displays the hour with the correct number of leading zeros
{
    if(now.hour<10)
    {
        sprintf(time," %d",now.hour);
        displayAt(time,2,1);
    }
    else
    {
        sprintf(time,"%.2d",now.hour);
        displayAt(time,2,1);
    }
}



//--------------------------------------------------Interrupts------------------------------------------------------------------------

void T32_INT1_IRQHandler()                          //Interrupt Handler for Timer 2
{
    TIMER32_1->INTCLR = 1;                          //Clear interrupt flag so it does not interrupt again immediately.
    ADC14->CTL0         |=  0b1;                    //Start ADC Conversion

}

void T32_INT2_IRQHandler()                          //Interrupt Handler for Timer 2
{
    TIMER32_2->INTCLR = 1;                          //Clear interrupt flag so it does not interrupt again immediately.
    lightBrightness+= 10;                           //increase brightness by 1% every interrupt (3 seconds when enabled)
    TIMER_A0->CCR[1] = lightBrightness;
    sprintf(time,"%.2d",lightBrightness);
    if(lightBrightness == 1000)
    {
        TIMER_A0->CCR[1] = 0;
        TIMER32_2->CONTROL &= ~BIT7;
        lightsOn = 0;
    }
}

void PORT1_IRQHandler()
{
    if(P1->IFG & BIT1)
    {
        speed = HIGH;
        //P1->OUT |= BIT0;                //todo debug led
        P1->IFG &= ~BIT1;
    }
    else if(P1->IFG & BIT4)
    {
        speed = LOW;
        //P1->OUT &= ~BIT0;               //todo debug led
        P1->IFG &= ~BIT4;
    }
}

void PORT4_IRQHandler()
{
    //SetTime Button Press
    if(P4->IFG & SETTIME)
    {
        if((timePresses==0) & (alarmPresses==0))                                  //First press -> Go into time Hours edit
        {
            RTC_C->PS1CTL   = 0b00000;                      //disable timer clock
            displayAt("00",8,1);                            //clears seconds
            timePresses++;
        }
        else if((timePresses==1) & (alarmPresses==0))
            timePresses++;
        else if(timePresses==2)                             //Third press -> Save time
        {
            RTC_C->PS1CTL   = 0b11010;                      //Enable timer clock
            configRTC(now.hour, now.min);
            timePresses = 0;
        }

        P4->IFG &= ~SETTIME;
    }

    //Alarm toggle/Up Button Press
    if(P4->IFG & (ALARM|UP))
    {
        if(timePresses==1)                              //Incoment Hours
        {
            now.hour++;
            if(now.hour>12)                                //hour Rolls Over
            {
                now.hour=1;
                toggleAMPM();
                displayAMPM();
            }
            displayHour();
        }
        else if(timePresses==2)                         //incoment Minutes
        {
            now.min++;
            if(now.min>59)
                now.min=0;                                  //Minutes reset
            displayMin();
        }
        else if(alarmPresses==1)
        {
            alarm.hour++;
            if(alarm.hour>12)
            {
                alarm.hour=1;
                toggleAMPM2();
            }
            displayAlarm();
        }
        else if(alarmPresses==2)
        {
            alarm.min++;
            if(alarm.min>59)
            {
                alarm.min=0;
            }
            displayAlarm();
        }
        else                                            //toggles alarm output and alarmflag
        {
            toggleAlarm();
        }

        P4->IFG &= ~(ALARM|UP);
    }

    //SetAlarm Button Press
    if(P4->IFG & SETALARM)                              //SetAlarm Button Press
    {
        if(timePresses==0)
        {
            alarmPresses++;
        }
        if(alarmPresses==3)
            alarmPresses=0;
        P4->IFG &= ~SETALARM;
    }

    //Snooze/Down Button Press
    if(P4->IFG & SNOOZE)
    {
        if(timePresses==1)
        {
            now.hour--;
            if(now.hour<1)                                //hour Rolls Over
            {
                now.hour=12;
                toggleAMPM();
                displayAMPM();
            }

            displayHour();
        }
        else if(timePresses==2)                         //incoment Minutes
        {
            now.min--;
            if(now.min>59)
                now.min=59;                                  //Minutes reset
            displayMin();
        }
        else if(alarmPresses==1)
        {
            alarm.hour--;
            if(alarm.hour==0)
            {
                alarm.hour=12;
                toggleAMPM2();
            }
            displayAlarm();
        }
        else if(alarmPresses==2)
        {
            alarm.min--;
            if(alarm.min>100)
            {
                alarm.min=59;
            }
            displayAlarm();
        }

        P4->IFG &= ~SNOOZE;
    }
}

void RTC_C_IRQHandler(void)
{
    if(RTC_C->PS1CTL & BIT0)
    {
        if(speed != HIGH)
        {
            now.sec         =   RTC_C->TIM0>>0 & 0x00FF;
            now.min         =   RTC_C->TIM0>>8 & 0x00FF;
            now.hour        =   RTC_C->TIM1>>0 & 0x00FF;
        }
        else if(speed == HIGH)
        {
            now.sec =0;
            now.min++;
            if(now.min==60)
            {
                now.min = 0;
                now.hour++;
            }
            RTC_C->TIM0 = now.min<<8 | 00;
            RTC_C->TIM1  = now.hour;
        }

        if(now.hour > 12) //rolls over time for 12-hour time
        {
            now.hour = 1;
        }

        if((now.hour == 12) & (now.min == 0) & (now.sec == 0)) //toggles AM and PM flag for each roll over will be used more with UART to convert 24 hr to 12 hr
        {
            toggleAMPM();
        }

        displayAlarm();

        RTC_flag = 1;
        RTC_C->PS1CTL &= ~BIT0;
        displayHour();

        commandWrite(132);
        dataWrite(0b00111010);
        displayMin();
        commandWrite(135);
        dataWrite(0b00111010);
        sprintf(time,"%.2d",now.sec);
        displayAt(time,8,1);

        displayAMPM();

        if(RTC_C->CTL0 & BIT1) {
            P1->OUT |= BIT0;            //this is what happens when alarm goes off
            RTC_C->CTL0 = 0xA500;
        }
    }
}

void ADC14_IRQHandler(void)
{
    ADC14->CLRIFGR1     &=    ~0b1111110;                 // Clear all IFGR1 Interrupts (Bits 6-1.  These could trigger an interrupt and we are checking them for now.)
    if(ADC14->IFGR0 & BIT0)
    {
            raw = ADC14->MEM[0];
            ADC14->CLRIFGR0     &=  ~BIT1;                  // Clear MEM1 interrupt flag
            voltage = raw*(3.3/16383);
            tempC  = (1000*voltage - 500)/10;
            tempF = ((tempC*9.0)/5.0)+32.0;
            sprintf(tempAr,"%.1f",tempF);                   //displays temperature
            displayAt(tempAr,1,4);
            commandWrite(213);                              //moves cursor to degrees symbol spot
            dataWrite(0b11011111);                          //prints degrees symbol
            commandWrite(214);                              //moves cursor to fahrenheit spot
            dataWrite(0b01000110);                          //prints an f

            sprintf(tempAr,"%.1f",tempC);                   //displays temperature
            displayAt(tempAr,9,4);
            commandWrite(221);                              //moves cursor to degrees symbol spot
            dataWrite(0b11011111);                          //prints degrees symbol
            commandWrite(222);                              //moves cursor to fahrenheit spot
            dataWrite(0b01000011);                          //prints an c
    }
    if(ADC14->IFGR0 & BIT1)
    {
        raw1 = ADC14->MEM[1];
        voltage1 = raw1*(3.3/16383);
        TIMER_A0->CCR[4] = voltage1*303;
        ADC14->CLRIFGR0 &=  ~BIT1;                  // Clear MEM1 interrupt flag
    }
    ADC14->CLRIFGR1     &=    ~0b1111110;

}

void TA3_N_IRQHandler()
{
    if(TIMER_A3->CCTL[4] & BIT0)
    {
        TIMER_A3->CCTL[4] &= ~BIT0;
        TIMER_A3->CCTL[4] &= ~BIT1;
        if((timePresses == 1) & (blinkFlag == 0))
        {
            displayAt("  ",2,1);
            displayMin();
            blinkFlag = 1;
        }
        else if((timePresses == 1) & (blinkFlag == 1))
        {
            displayHour();
            displayMin();
            blinkFlag = 0;
        }
        else if((timePresses == 2) & (blinkFlag == 0))
        {
            displayHour();
            displayAt("  ",5,1);
            blinkFlag = 1;
        }
        else if((timePresses == 2) & (blinkFlag == 1))
        {
            displayHour();
            displayMin();
            blinkFlag = 0;
        }

        else if((alarmPresses == 1) & (blinkFlag == 0))
        {
            displayAlarm();
            displayAt("  ",3,2);
            blinkFlag = 1;
        }
        else if((alarmPresses == 1) & (blinkFlag == 1))
        {
            displayAlarm();
            blinkFlag = 0;
        }
        else if((alarmPresses == 2) & (blinkFlag == 0))
        {
            displayAlarm();
            displayAt("  ",6,2);
            blinkFlag = 1;
        }
        else if((alarmPresses == 2) & (blinkFlag == 1))
        {
            displayAlarm();
            blinkFlag = 0;
        }

        //ADD CODE TO BLINK TIME HERE
       //  use -> NVIC_DisableIRQ(TA3_N_IRQn); to turn off blinking
    }
}

//------------------------------------------------------------Initilizations-----------------------------------------------------------------------


void intBlinkTimerA(void)
{
    P9->SEL0 |= BIT3;
    P9->SEL1 &= BIT3;
    P9->DIR |= BIT3;
    P9->IE |= BIT3;
    P9->IES |= BIT3;
    TIMER_A3->EX0 = 0b0000000000000111;             // Bonus clock divider This register allows me to divide the clock by 1+EX0, in this case, a divider of 8
    TIMER_A3->CTL = 0b0000001011010100;             // SMCLK, Divide by 8, Count Up, Clear to start
    TIMER_A3->CCR[0] = 23437;                       // counts half a second
    TIMER_A3->CCTL[4] = 0b0000000011110100;         // CCR1 reset/set mode 7, with interrupt.
    TIMER_A3->CCR[4] = 11718;                       // Duty cycle to 50% to start (1/2 of CCR0).
    NVIC_EnableIRQ(TA3_N_IRQn);                     //USE THIS LINE TO TURN OFF INTERRUPT
}
void intLCDBrightness(void)
{
    P2->SEL0 |=  BIT7;
    P2->SEL1 &= ~BIT7;
    P2->DIR |=   BIT7;

    TIMER_A0->CCR[0] = 1000-1;
    TIMER_A0->CCTL[4] = 0b11100000;
    TIMER_A0->CTL = 0b1000010100;           //timer a pwm
}
void intAlarm()
{
    displayAt("6:00  AM",4,2);
    displayAt("Alarm: OFF",3,3);
    alarm.hour = 6;
    alarm.min = 0;
}

void intSpeedButton()
{
    P1->SEL0 &= ~(BIT1|BIT4);
    P1->SEL1 &= ~(BIT1|BIT4);
    P1->DIR  &= ~(BIT1|BIT4);
    P1->REN  |=  (BIT1|BIT4);
    P1->OUT  |=  (BIT1|BIT4);
    P1->IE   |=  (BIT1|BIT4);
    P1->IES  |=  (BIT1|BIT4);

    NVIC_EnableIRQ(PORT1_IRQn);
}

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

    RTC_C->AMINHR   = alarm.hour<<8 | alarm.min | BIT(15) | BIT(7);

    RTC_C->CTL0     = ((0xA500) | BIT5);
    NVIC_EnableIRQ(RTC_C_IRQn);
}

void ADC14init(void)
{
    //For Analog Input 8
    P5->SEL0            |=   (BIT5|BIT4);                      // Select ADC Operation
    P5->SEL1            |=   (BIT5|BIT4);                      // SEL = 11 sets to ADC operation
    ADC14->CTL0         =    0;                         // Disable ADC for setup

    // CTL0 Configuration
    // 31-30 = 10   to divide down input clock by 32X
    // 26    = 1    to sample based on the sample timer.  This enables the use of bits 11-8 below.
    // 21-19 = 100  for SMCLK
    // 18-17 = 00   one channel
    // 11-8  = 0011 for 32 clk sample and hold time
    // 7     = 1    for multiple ADC channels being read
    // 4     = 1    to turn on ADC
    ADC14->CTL0         =    0b10000100001000100000001110010000;

    ADC14->CTL1         =    (BIT5 | BIT4);         // Bits 5 and 4 = 11 to enable 14 bit conversion
                                                        // Bit 23 turns on Temperature Sensor
    ADC14->MCTL[0]      =    0;                         // A0 on P5.5
    ADC14->MCTL[1]      =    1| BIT7;
                                                        // BIT7 says to stop converting after this ADC.
    ADC14->IER0         |=   (BIT0 | BIT1);            // Interrupt on conversion
//    REF_A->CTL0         |=   BIT0;                      // Turns on the Reference for the Temperature Sensor
//    REF_A->CTL0         &=   ~BIT3;                     // Turns off the Temperature Sensor Disable

    ADC14->CTL0         |=   0b10;                      // Enable Conversion
    NVIC->ISER[0]       |=   1<<ADC14_IRQn;
}

void tempT32interrupt(void)
{
    TIMER32_1->LOAD       =   ADC_CONVERSION_RATE;        //Set interval for interrupt to occur at
    TIMER32_1->CONTROL       =   0b11100010;
    NVIC_EnableIRQ(T32_INT1_IRQn);
}

void LEDT32interrupt(void)
{
    TIMER32_2->LOAD       =   175781;        //Set interval for interrupt to occur at
    TIMER32_2->CONTROL       =  0b01101000; //use bit 7 to enable in wake up lights, interrupt enabled, divide by 256, 16 bit mode, wrapping mode
    NVIC_EnableIRQ(T32_INT2_IRQn);
}



void LED_init(void)
{
    P2->SEL0 |=  BIT4;
    P2->SEL1 &= ~BIT4;
    P2->DIR |=   BIT4;

    TIMER_A0->CCR[0] = 1000-1;
    TIMER_A0->CCTL[1] = 0b11100000;
    TIMER_A0->CTL = 0b1000010100;
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
    sysTickDelay_us(10);
    P7->OUT &= ~0xF0; //clears data pins
    sysTickDelay_us(10);
    pushByte(command);
    sysTickDelay_us(10);
}

void dataWrite(uint8_t data) //writes data to LCD using RS = 1
{
    P7->OUT |= BIT0; //rs=1
    sysTickDelay_us(10);
    P7->OUT &= ~0xF0; //clears data pins
    sysTickDelay_us(10);
    pushByte(data);
    sysTickDelay_us(10);
}

void pushByte(uint8_t byte) //stores 4 bits into variable and sends nibble to push nibble twice for each half of the byte
{
    pushNibble((byte & 0xF0)); //sends first nibble
    sysTickDelay_ms(2);
    pushNibble((byte & 0x0F)<<4); //shifts second nibble and sends it through pins 4-7
}

void pushNibble(uint8_t nibble) //sets nibble up in data pins for pulse
{
    P7->OUT &= ~0xF0; //clearing data pins
    sysTickDelay_ms(2);
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





