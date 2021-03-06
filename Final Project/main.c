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
 * SET UART TO
 * BAUD: 115200
 * PARITY: EVEN
 * STOP: 1 BIT
 *
 */

//todo when in 1 min of alarm change back to reg time

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

// Making a buffer of 100 characters for serial to store to incoming serial data
#define BUFFER_SIZE 100
char INPUT_BUFFER[BUFFER_SIZE];
// initializing the starting position of used buffer and read buffer
uint8_t storage_location = 0; // used in the interrupt to store new data
uint8_t read_location = 0; // used in the main application to read valid data that hasn't been read yet
int duty = 0;

void configRTC(int hour, int min,int sec);
void ADC14init(void);
void tempT32interrupt(void);
void intButtons();
void intAlarm();
void intSpeedButton();

void displayAMPM();
void toggleAMPM();
void displayHour();
void displayMin();
void toggleAlarm();
void displayAlarm();
void toggleAMPM2();
void wakeUpLights(void);
void LEDT32interrupt(void);
void intLCDBrightness(void);
void intBlinkTimerA(void);




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
void initTA2();

void setupSerial();
void writeOutput(char *string); // write output charactrs to the serial port
void readInput(char* string); // read input characters from INPUT_BUFFER that are valid
void extractTimeSerial(char string[]);


float tempC=0,tempF = 0, voltage = 0, raw = 0,raw1 = 0,voltage1 = 0;
char time[2],tempAr[3];
int RTC_flag =0, timePresses=0, alarmFlag=0, alarmSoundFlag=0, alarmPresses=0, speed=0, lightOn=0, resetNeeded=0,snoozeFlag=0;
int AMPM = 1, AMPM2=1;                       //flag to determine AM or PM will be used more for UART functionality to convert 24 hr to 12 hr time
int lightsOn = 0;                   //flag to be used to check if the wake up lights should be turned on
uint32_t lightBrightness = 0;
float LCDbrightness = 50;
int blinkFlag = 0;
int speakerFlag = 0,speakerFlag1 = 0,alarmSoundToggle = 1;

enum states{
    SETTIMESERIAL,
    SETALARMSERIAL,
    READTIME,
    READALARM,
    INVALID
};
enum states state = INVALID;

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

struct
{
    uint8_t min;
    uint8_t hour;
}master;

//todo alarm status
//todo stop 1 min before
//todo alarm snooze
//todo actualy toggle alarm
//todo snooze

//-------------------------------------------------MAIN---------------------------------------------------------------


void main(void)
 {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    //Initilizing Interupts
    __disable_interrupt();
    char string[BUFFER_SIZE]; // Creates local char array to store incoming serial commands
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
    setupSerial();
    initTA2();


    commandWrite(CLEAR);

    intAlarm();
    P1->SEL0 &= ~BIT0;
    P1->SEL1 &= ~BIT0;
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;
    TIMER_A0->CCR[4] = 1000;
    __enable_interrupt();
    configRTC(6, 59,55);

    lightsOn = 1;
int tempHour = 0,tempHourA;
timePresses=0;
    while(1)
    {

        tempHour = now.hour;
        tempHourA = alarm.hour;
        readInput(string); // Read the input up to \n, store in string.  This function doesn't return until \n is received
        if(string[0] != '\0'){ // if string is not empty, check the inputted data.

                if(!strncmp(string,"SETTIME ",8))
                {
                    state = SETTIMESERIAL;
                }
                else if(!strncmp(string,"SETALARM ",9))
                {
                    state = SETALARMSERIAL;
                }
                else if(!strncmp(string,"READTIME",9))
                {
                    state = READTIME;
                }
                else if(!strncmp(string,"READALARM",9))
                {
                    state = READALARM;
                }
                else
               {
                   state = INVALID;
               }

            switch(state) //state machine to account for each light being turned on
            {
            case SETTIMESERIAL:
                extractTimeSerial(string);
                break;
            case SETALARMSERIAL:
                extractTimeSerial(string);
                break;

            case READTIME:
                tempHour = now.hour;
                if(!AMPM)
                {
                   tempHour += 12;
                   sprintf(string,"%.2d:%.2d:%.2d",tempHour,now.min,now.sec);
                   writeOutput(string);
                }
                else if(AMPM)
                {
                    sprintf(string,"%.2d:%.2d:%.2d",tempHour,now.min,now.sec);
                    writeOutput(string);
                }
                break;
            case READALARM:
                tempHourA = alarm.hour;
                if(!AMPM2)
                {
                   tempHourA += 12;
                   sprintf(string,"%.2d:%.2d",tempHourA,alarm.min);
                   writeOutput(string);
                }
                else if(AMPM2)
                {
                    sprintf(string,"%.2d:%.2d",tempHourA,alarm.min);
                    writeOutput(string);
                }
                break;
            case INVALID:
                writeOutput("INVALID ");
                writeOutput(string);
                break;

            }

        }
    }
}

//--------------------------------------------------Non-Interrupt Functions-----------------------------------------------------------

void extractTimeSerial(char string[])
{   writeOutput("Command Sent: ");
    writeOutput(string);


            if(state == SETTIMESERIAL)
            {
                now.hour = ((int)string[8]-'0')*10+((int)string[9]-'0');
                now.min = ((int)string[11]-'0')*10+((int)string[12]-'0');
                now.sec = ((int)string[14]-'0')*10+((int)string[15]-'0');
                if(now.hour>12)
                {
                    now.hour-=12;
                    AMPM = 0;
                    displayAMPM();
                }
                else if(now.hour<=12)
                {
                    AMPM = 1;
                    displayAMPM();
                }
                configRTC(now.hour, now.min,now.sec);
            }
            else if(state == SETALARMSERIAL)
            {
                alarm.hour = ((int)string[9]-'0')*10+((int)string[10]-'0');
                alarm.min = ((int)string[12]-'0')*10+((int)string[13]-'0');
                if(alarm.hour>12)
                {
                    alarm.hour-=12;
                    AMPM2 = 0;
                    displayAlarm();
                    displayAt("PM",10,2);
                    RTC_C->AMINHR   = alarm.hour<<8 | alarm.min | BIT(15) | BIT(7);     //Sets Alarm Time
                }
                else if(alarm.hour<=12)
                {
                    AMPM2 = 1;
                    displayAlarm();
                    displayAt("AM",10,2);
                    RTC_C->AMINHR   = alarm.hour<<8 | alarm.min | BIT(15) | BIT(7);     //Sets Alarm Time
                }
            }
            else
            {
                writeOutput("INVALID ");
            }

}

void wakeUpLights(void)
{
    TIMER32_2->CONTROL |= BIT7;
}

void toggleAlarm()
{
    if(alarmFlag==1)
    {
        alarmFlag=0;                                                        //toggles the alarm falg
        displayAt("OFF",10,3);                                              //Displays Alarm: OFF
    }

    else
    {
        alarmFlag=1;                                                        //togles the alarm flag
        displayAt("ON ",10,3);                                              //Displays Alarm: On
        RTC_C->AMINHR   = alarm.hour<<8 | alarm.min | BIT(15) | BIT(7);     //Sets Alarm Time
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
    RTC_C->AMINHR   = alarm.hour<<8 | alarm.min | BIT(15) | BIT(7);
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
        sprintf(time,"AM");
        displayAt(time,12,1);
    }
    else if(!AMPM)
    {
        sprintf(time,"PM");
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
    if((alarmFlag==1)&(timePresses==0)&(alarmSoundFlag==0))//&(snoozeFlag==0))
    {
        lightOn = 1;
        uint32_t totalAlarm=0,totalTime=0;
        totalAlarm = (alarm.hour * 3600) + (alarm.min*60) + (0)+(!AMPM2 * 12 * 3600);
        totalTime  = (now.hour*3600) + (now.min*60) + (now.sec)+(!AMPM * 12 * 3600);

        lightBrightness=(1000-((totalAlarm-totalTime)*3.33333333333));

        //lightBrightness+= 10;                           //increase brightness by 1% every interrupt (3 seconds when enabled)
        TIMER_A0->CCR[1] = lightBrightness;
        if(lightBrightness > 1000)
        {
            TIMER_A0->CCR[1] = 0;                           //
            TIMER32_2->CONTROL &= ~BIT7;
            lightsOn = 0;
            lightOn=0;
        }
    }
    else if((alarmSoundFlag==1))
    {
        TIMER_A0->CCR[1] = 1000;
        lightOn=1;
    }
    else
    {
        TIMER_A0->CCR[1] = 0;
        lightOn=0;
    }

}

void PORT1_IRQHandler()
{
    if(P1->IFG & BIT1)
    {
        speed = HIGH;
        P1->IFG &= ~BIT1;
    }
    else if(P1->IFG & BIT4)
    {
        speed = LOW;
        P1->IFG &= ~BIT4;
    }
}

void PORT4_IRQHandler()
{
    //SetTime Button Press
    if(P4->IFG & SETTIME)
    {
        if(alarmSoundFlag == 0)
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
                configRTC(now.hour, now.min,now.sec);
                timePresses = 0;
            }
        }
        P4->IFG &= ~SETTIME;
    }

    //(alarmFlag==1)&(timePresses==0)&(alarmSoundFlag==0)
    //Alarm toggle/Up Button Press
    if(P4->IFG & (ALARM|UP))
    {
        if(lightOn & !alarmSoundFlag)               //light on sound off
        {
            resetNeeded=1;
            alarmFlag=0;
            alarm.hour = master.hour;
            alarm.min = master.min;
        }
        else if(!alarmSoundFlag)                    //sound off
        {
            if(timePresses==1)                              //Incoment Hours
            {
                now.hour++;
                if(now.hour>12)                                //hour Rolls Over
                    now.hour=1;
                else if(now.hour==12)
                {
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
                    alarm.hour=1;
                else if(alarm.hour==12)
                    toggleAMPM2();
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
        }
        else if(alarmSoundFlag)
        {
            TIMER_A0->CCR[1] = 0;
            alarmSoundFlag=0;
            lightOn=0;
            alarm.hour = master.hour;
            alarm.min = master.min;
            //turn of alarm for the day set alarm = master
        }
        snoozeFlag=0;

        P4->IFG &= ~(ALARM|UP);
    }

    //SetAlarm Button Press
    if(P4->IFG & SETALARM)                              //SetAlarm Button Press
    {
        if(alarmSoundFlag == 0)
        {
            if(timePresses==0)
            {
                alarmPresses++;
            }
            if(alarmPresses==3)
                alarmPresses=0;
        }
        P4->IFG &= ~SETALARM;
    }

    //Snooze/Down Button Press
    if(P4->IFG & SNOOZE)
    {
        if(!alarmSoundFlag)
        {
            if(timePresses==1)
            {
                now.hour--;
                if(now.hour<1)                                //hour Rolls Over
                    now.hour=12;
                if(now.hour==11)
                {
                    toggleAMPM();
                    displayAMPM();
                }

                displayHour();
            }
            else if((timePresses==2)&(!alarmSoundFlag))                         //incoment Minutes
            {
                now.min--;
                if(now.min>59)
                    now.min=59;                                  //Minutes reset
                displayMin();
            }
            else if((alarmPresses==1)&(!alarmSoundFlag))
            {
                alarm.hour--;
                master.hour = alarm.hour;
                if(alarm.hour==0)
                    alarm.hour=12;
                else if(alarm.hour==11)
                    toggleAMPM2();
                displayAlarm();
            }
            else if((alarmPresses==2)&(!alarmSoundFlag))
            {
                alarm.min--;
                master.min = alarm.min;
                if(alarm.min>100)
                {
                    alarm.min=59;
                }
                displayAlarm();
            }
        }
        else if(alarmSoundFlag)     //alarm is on and snooze is pressed
        {

            alarm.min = alarm.min + 10;
            if(alarm.min>59)
            {
                //todo
                alarm.min = alarm.min-59;
                displayAlarm();

                //if lights are on
            }
            alarmSoundFlag=0;
            snoozeFlag=1;
        }

        P4->IFG &= ~SNOOZE;
    }
}

void RTC_C_IRQHandler(void)
{
    //todo
    //printf("alarmFlag: %d\nalarmSoundFlag: %d\resetNeeded: %d\nlihtOn: %d\n\n",alarmFlag,alarmSoundFlag,resetNeeded,lightOn);
    if(RTC_C->PS1CTL & BIT0)
    {//     speed is low        alarm is off                middle of hour
        if((speed != HIGH) | (((alarm.hour==now.hour)&((alarm.min-now.min)==1)&(AMPM==AMPM2)) |
                (((alarm.hour-now.hour)==1)&(now.min==59)&(AMPM==AMPM2)&(alarm.min==0)) |
                (((now.hour==12)&(alarm.hour==1))&(AMPM!=AMPM2)&(now.min==59)))&(alarmFlag==1))
        {           //change of hour                                                changing from AMPM
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
            if((((alarm.hour==now.hour)&((alarm.min-now.min)==1)&(AMPM==AMPM2)) |
                    (((alarm.hour-now.hour)==1)&(now.min==59)&(AMPM==AMPM2)&(alarm.min==0)) |
                    (((now.hour==12)&(alarm.hour==1))&(AMPM!=AMPM2)&(now.min==59)))&(alarmFlag==1))
                speed=LOW;
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


        if(RTC_C->CTL0 & BIT1)              //todo
        {
            if((alarmFlag==1)&(AMPM==AMPM2))
            {
                P1->OUT ^= BIT0;            //this is what happens when alarm goes off
                alarmSoundFlag=1;

            }
            if(resetNeeded)
            {
                resetNeeded=0;
                alarmFlag=1;                //
            }
            RTC_C->CTL0 = 0xA500;
        }
        if(alarmSoundFlag)
        {
            if(alarmSoundToggle)
            {
                TIMER_A2->CCR[4] = 3000;
                alarmSoundToggle = 0;
            }
            else if(!alarmSoundToggle)
            {
                TIMER_A2->CCR[4] = 0;
                alarmSoundToggle = 1;
            }
        }
        else
        {
            TIMER_A2->CCR[4] = 0;
            alarmSoundToggle = 1;
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
        if(alarmSoundFlag)
        {
            TIMER_A0->CCR[4] = 1000;
        }
        else{
            raw1 = ADC14->MEM[1];
            voltage1 = raw1*(3.3/16383);
            TIMER_A0->CCR[4] = voltage1*303;
        }
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
void initTA2()
{
    P6->SEL0 |= BIT7;
    P6->SEL1 &= ~BIT7;
    P6->DIR |= BIT7;

    TIMER_A2->CCR[0] = 6000-1;
    TIMER_A2->CTL = 0b1000010100;
    TIMER_A2->CCTL[4] = 0b11100000;
    TIMER_A2->CCR[4] = 0;
}
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
    displayAt("7:00  AM",4,2);
    displayAt("Alarm: ON ",3,3);
    alarmFlag=1;
    alarm.hour = 7;
    alarm.min = 0;
    master.hour = alarm.hour;
    master.min = alarm.min;
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

void configRTC(int hour, int min, int sec)
{
    RTC_C->CTL0     =   0xA500;     //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;
    RTC_C->TIM0     = min<<8 | sec;
    RTC_C->TIM1     = hour;

    RTC_C->PS1CTL   = 0b11010;

    RTC_C->AMINHR   = alarm.hour<<8 | alarm.min | BIT(15) | BIT(7);


    //RTC_C->CTL0     = ((0xA500) | BIT5);                //enales the alarm
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
    TIMER32_2->LOAD       =  10000;        //Set interval for interrupt to occur at
    TIMER32_2->CONTROL    =  0b01101000; //use bit 7 to enable in wake up lights, interrupt enabled, divide by 256, 16 bit mode, wrapping mode
    TIMER32_2->CONTROL   |=  BIT7;
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
//---------------------------------------------------------------------------Serial Communication Code----------------------------------------------

void setupSerial()
{
    P1->SEL0 |=  (BIT2 | BIT3); // P1.2 and P1.3 are EUSCI_A0 RX
    P1->SEL1 &= ~(BIT2 | BIT3); // and TX respectively.

    EUSCI_A0->CTLW0  = BIT0; // Disables EUSCI. Default configuration is 8N1
    EUSCI_A0->CTLW0 |= BIT7; // Connects to SMCLK BIT[7:6] = 10, BIT14 = none
    EUSCI_A0->CTLW0 &=  ~(BIT(15)|BIT(14)|BIT(11));  //BIT15 = Parity, BIT11 = one Stop Bit
    // Baud Rate Configuration
    // 3000000/(16*9600) = 19.531  (3 MHz at 9600 bps is fast enough to turn on over sampling (UCOS = /16))
    // UCOS16 = 1 (0ver sampling, /16 turned on)
    // UCBR  = 19 (Whole portion of the divide)
    // UCBRF = .531 * 16 = 8 (0x08) (Remainder of the divide)
    // UCBRS = 3000000/9600 remainder=0.5 -> 0xAA (look up table 22-4)
    EUSCI_A0->BRW = 19;  // UCBR Value from above
    EUSCI_A0->MCTLW = 0xAA81; //UCBRS (Bits 15-8) & UCBRF (Bits 7-4) & UCOS16 (Bit 0)

    EUSCI_A0->CTLW0 &= ~BIT0;  // Enable EUSCI
    EUSCI_A0->IFG &= ~BIT0;    // Clear interrupt
    EUSCI_A0->IE |= BIT0;      // Enable interrupt
    NVIC_EnableIRQ(EUSCIA0_IRQn);
}

void writeOutput(char *string)
{
    int i = 0;  // Location in the char array "string" that is being written to
    while(string[i] != '\0') {
        EUSCI_A0->TXBUF = string[i];
        i++;
        while(!(EUSCI_A0->IFG & BIT1));
        EUSCI_A0->IFG &= ~BIT1;
    }
    EUSCI_A0->TXBUF = '\n';
}

void readInput(char *string)
{
    int i = 0;  // Location in the char array "string" that is being written to

    // One of the few do/while loops I've written, but need to read a character before checking to see if a \n has been read

    do
    {
        // If a new line hasn't been found yet, but we are caught up to what has been received, wait here for new data
        while(read_location == storage_location && INPUT_BUFFER[read_location] != '\n');
        string[i] = INPUT_BUFFER[read_location];  // Manual copy of valid character into "string"
        INPUT_BUFFER[read_location] = '\0';
        i++; // Increment the location in "string" for next piece of data
        read_location++; // Increment location in INPUT_BUFFER that has been read
        if(read_location == BUFFER_SIZE)  // If the end of INPUT_BUFFER has been reached, loop back to 0
        {
            read_location = 0;
        }
    }
    while(string[i-1] != '\n'); // If a \n was just read, break out of the while loop

    string[i-1] = '\0'; // Replace the \n with a \0 to end the string when returning this function

}

void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & BIT0)  // Interrupt on the receive line
    {
        INPUT_BUFFER[storage_location] = EUSCI_A0->RXBUF; // store the new piece of data at the present location in the buffer
        EUSCI_A0->IFG &= ~BIT0; // Clear the interrupt flag right away in case new data is ready
        storage_location++; // update to the next position in the buffer
        if(storage_location == BUFFER_SIZE) // if the end of the buffer was reached, loop back to the start
            storage_location = 0;
    }
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




//#include "msp.h"
//#include <stdio.h>
//#include <string.h>
//void initTASpeaker(void);
//
//void SysTick_Init(void) //initializes systick timer
//{
//    SysTick->CTRL = 0;
//    SysTick->LOAD = 0x00FFFFFF; //sets max value
//    SysTick->VAL = 0; //sets min value
//    SysTick->CTRL = 0x00000005; //enables timer
//}
//
//void sysTickDelay_ms(int ms) //timer ms
//{
//    SysTick->LOAD = ((ms*3000)-1);
//    SysTick->VAL = 0;
//    while((SysTick->CTRL & BIT(16))==0);
//}
//
//
//void main(void)
//{
//    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
//    initTASpeaker();
//    SysTick_Init();
//    while(1)
//    {
//        TIMER_A0->CCR[0] = 6000;  //Math in an interrupt is bad behavior, but shows how things are happening.  This takes our clock and divides by the frequency of this note to get the period.
//        TIMER_A0->CCR[2] = 3000;  //50% duty cycle
//        sysTickDelay_ms(1000);
//        TIMER_A0->CCR[2] = 0;
//        sysTickDelay_ms(1000);
//    }
//}
//
//void initTASpeaker(void)
//{
//    P2->SEL0 |= BIT5;
//    P2->SEL1 &= BIT5;
//    P2->DIR |= BIT5;
//    TIMER_A0->CCR[0] = 0;                           // Turn off timerA to start
//    TIMER_A0->CCTL[2] = 0b11100000;         // Setup Timer A0_2 Reset/Set, Interrupt, No Output
//    TIMER_A0->CCR[2] = 0;                           // Turn off timerA to start
//    TIMER_A0->CTL = 0b1000010100;             // Count Up mode using SMCLK, Clears, Clear Interrupt Flag
//}

