/* 
 * File:   Project 5 Main.c
 * Author: Alexander
 *
 * Created on October 28, 2019, 10:25 PM
 */


/*
 Agenda
 * finish the ending check function
 * double check to make sure it is turning correctly (might need to add extra cases idk yet)
 * Do Mario RGB for extra credit (maybe)
 
 
 
 
 */
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "ssd.h"
#include "lcd.h"
#include "led.h"
#include "utils.h"
#include "swt.h"
#include "srv.h"
#include <plib.h>
#include "adc.h"
/*
 * 
 */
#pragma config OSCIOFNC =	ON
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_4           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)









/* --------------------------Definitions------------------------------------ */
#define SYS_FREQ    (80000000L) // 80MHz system clock

// leds 
#define  led0  LATAbits.LATA0
#define  led1  LATAbits.LATA1
#define  led2  LATAbits.LATA2
#define  led3  LATAbits.LATA3
#define  led4  LATAbits.LATA4
#define  led5  LATAbits.LATA5
#define  led6  LATAbits.LATA6
#define  led7  LATAbits.LATA7
//IR sensors definitions
#define  LD1   PORTDbits.RD9
#define  LD2   PORTDbits.RD11
#define  LD3   PORTDbits.RD10
#define  LD4   PORTDbits.RD8

// switches






int PBCLK = 80000000;
int Time_preScl = 1;
float Time_Frequency = 10;
int mode =1; // used to change the states
int LCD_Lock = 0; // to debounce the lcd
int buttonLock = 0;
int timer2lock=0;
int recordCounter = 0;
char recordCounterChar[];
short sample[55000];
float timer2Counter=0.0;
int i=0;
int numpoints;
int samplepoints=0; // keeps track of sample points recorded
float sampleremaining=0.0; // used for timer in mode 3
char sampleChar[];
char sampleRemainingChar[];

int maxMicSignal=1000;// just debug variable
int clapDetect = 800;// signal for the mic to pick up the clap
int inBetweencClapDetect=650; // signal for the mic to pick up the inbetween silence between claps

//project 5 variables
int correctingSelf=0; //variable used to check of it is correcting it self back to the track

// previous states for the IR sensors
int pLD1=0;
int pLD2=0;
int pLD3=0;
int pLD4=0;
// debug section

char micCheck[50];

int frequency = 440;

float testcounter=0;

float scalingfactor=0; 
int FWDFast=300;
int FWDSlow=360;

int Neutral=380;

int BCKSlow=400;
int BCKFast=450;

int turning=1; // variable used in the ISR for turning states
int ISRCounter=0;
int clap1=0;
int inBetweenClaps=0;
int clap2=0;
int clapCounter=0;


int d1=0;
int d2=0;
int d3=0;
int d4=0;

int starting= 1; // variable to override the end case in the driving function
int pitstop=0;
// Demo Varibles
char RobotName='b';
int DisplayCounter=0;
int overRide=0;

float test=0;
int main() 
{
FWDFast+=scalingfactor;
BCKFast+=scalingfactor;
FWDSlow+=scalingfactor;
BCKSlow+=scalingfactor;
LCD_Init();
ADC_Init();
SSD_Init();
Timer3Setup();
CNConfig();
//Timer2Setup();
SRV_Init();
oc1Setup();
ANSELA = 0xFFFF;
TRISA = 0x0000;
DDPCONbits.JTAGEN = 0;      // Statement is required to use Pin RA0 as IO
TRISBbits.TRISB14 = 0;
ANSELBbits.ANSB14 = 0;
TRISFbits.TRISF3 = 0;

//switches
TRISFbits.TRISF3 = 1;
TRISFbits.TRISF5 =1;
TRISFbits.TRISF4=1;
TRISDbits.TRISD15=1;
TRISDbits.TRISD14=1;
        
TRISBbits.TRISB11=1;
ANSELBbits.ANSB11 =0;

TRISBbits.TRISB10=1;
ANSELBbits.ANSB10 =0;

TRISBbits.TRISB9=1;
ANSELBbits.ANSB9=0;

//buttons
ANSELBbits.ANSB8 = 0;


//mic
TRISBbits.TRISB4=1;
ANSELBbits.ANSB4=1;


// IR sensors 
TRISDbits.TRISD9  =1; //declaring IR sensors that use register D as input (LD1)
TRISDbits.TRISD11 =1; //declaring IR sensors that use register D as input (LD2)
TRISDbits.TRISD10 =1; //declaring IR sensors that use register D as input (LD3)
TRISDbits.TRISD8 =1; //declaring IR sensors that use register D as input (LD4)

//analog switches
ANSELBbits.ANSB11=0;
ANSELBbits.ANSB10=0;
ANSELBbits.ANSB9=0;

ANSELGbits.ANSG6=0;
RPB14R = 0xC;
LCD_DisplayClear();
LCD_WriteStringAtPos("Section", 1, 5);  // Debug
while(1)
{
    if(mode ==1)
    {
      IEC0bits.T1IE=1;
      IEC0bits.T2IE = 0; //delete this later so the servos will work  
      //MicCheck();  // debug
      led0=0; led1=0; led2=0; led3=0; led4=0; led5=0; led6=0; led7=0;
      LCD_WriteStringAtPos("Waiting to start", 0, 0);
      if(ADC_AnalogRead(4)>clapDetect) // threshhold to detect clap - Use this to calibrate the claps
      {
//        if(clapCounter==0)
//        {
//        LCD_WriteStringAtPos("Clap", 1, 0);  
//        }else if(clapCounter==1)
//        {
//        LCD_WriteStringAtPos("Clap", 1, 6);    
//        }
        clapCounter+=1;
        delay_ms(100);
      }
      
      if(clapCounter==2)
      {
          mode=2;
      }
      
    }else if(mode ==2)
    {
        if(DisplayCounter==0)
        {
        SSD_WriteDigits(d1,d2,32,32,0,1,0,0);
        }
        //timerDisplay();
        LCD_WriteStringAtPos("Team 13: Brayden",0,0);
        //sprintf(micCheck,"%d",pitstop);
        //LCD_WriteStringAtPos(micCheck,1,4);
        if(!LD1&&!LD2&&!LD3&&!LD4&&starting)  // end of track   //1
        {
            //LCD_WriteStringAtPos("Starting", 0, 5);
            //LCD_WriteStringAtPos("1", 1, 13);
            SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
            SRV_SetPulseMicroseconds1(BCKFast);  // left wheel  
        }
    }else if (mode == 3) //testing pwm in this mode. Delete later after adding it to mode 2
    {
     //IEC0bits.T2IE = 1;
    DemoCheckpoint();
    
        
    }
    
    
    
     
}
    
}


int MicCheck()
{
    if(ADC_AnalogRead(4)<maxMicSignal)
    {
      maxMicSignal=ADC_AnalogRead(4);  
        
    }
    sprintf(micCheck,"%d",maxMicSignal);
    LCD_WriteStringAtPos(micCheck,1,9);
    return ADC_AnalogRead(4);
    
}





void CNConfig()
{
    CNCONDbits.ON = 1;
    CNEND = 0xF00;
    CNPUD = 0x000;
    PORTD;
    IPC8bits.CNIP = 6;
    IPC8bits.CNIS = 3;
    
    IFS1bits.CNDIF = 0;
    IEC1bits.CNDIE = 1;
    INTEnableSystemMultiVectoredInt();
    
}

void drivingInstruction()
{
    if(mode==2)
    {
        if(overRide==0)
        {
            if((!LD1&&!LD2&&!LD3&&!LD4)&&!starting)
            {
                pitstop+=1;
            }
            if(pitstop==2)
            {
                overRide=1;
            }
             if((!LD1&&!LD2&&!LD3&&!LD4)&&(!starting||starting)&&pitstop<2) //2
            {

            LCD_WriteStringAtPos("Checkpoint", 0, 5);
            LCD_WriteStringAtPos("2.1", 1, 13);
            SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
            SRV_SetPulseMicroseconds1(BCKSlow);  // left wheel   
            starting=0;
            } if((!LD1&&!LD2&&!LD3&&!LD4)&&(!starting||starting)&&pitstop==2) //2
            {

            LCD_WriteStringAtPos("END", 0, 5);
            LCD_WriteStringAtPos("2", 1, 13);
            SRV_SetPulseMicroseconds0(Neutral);  //right wheel
            SRV_SetPulseMicroseconds1(Neutral);  // left wheel   
            starting=0;
            } else if((LD1&&!LD2&&!LD3&&!LD4)&&(!starting||starting)) //3
            {

            LCD_WriteStringAtPos("Sharp Left", 0, 5);
            LCD_WriteStringAtPos("3", 1, 13);
            SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
            SRV_SetPulseMicroseconds1(Neutral);  // left wheel   
            starting=0;
            }else if((LD1&&LD2&&!LD3&&!LD4)&&(!starting||starting))  //4
            {

            LCD_WriteStringAtPos("Slight Left", 0, 5);
            LCD_WriteStringAtPos("4", 1, 13);
            SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
            SRV_SetPulseMicroseconds1(BCKSlow);  // left wheel   
            starting=0;
            }else if((LD1&&LD2&&LD3&&!LD4)&&(!starting||starting))//5
            {

            LCD_WriteStringAtPos("Sharp Left", 0, 5);
            LCD_WriteStringAtPos("5", 1, 13);
            SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
            SRV_SetPulseMicroseconds1(Neutral);  // left wheel   
            starting=0;
            }else if((!LD1&&!LD2&&!LD3&&LD4)&&(!starting||starting)) //6
            {

            LCD_WriteStringAtPos("Sharp Right", 0, 5);
            LCD_WriteStringAtPos("6", 1, 13);
            SRV_SetPulseMicroseconds0(Neutral);  //right wheel
            SRV_SetPulseMicroseconds1(BCKFast);  // left wheel   
            starting=0;
            }else if((LD1&&!LD2&&!LD3&&LD4)&&(!starting||starting)) //7
            {

            LCD_WriteStringAtPos("Straight", 0, 5);
            LCD_WriteStringAtPos("7", 1, 13);
            SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
            SRV_SetPulseMicroseconds1(BCKFast);  // left wheel   
            starting=0;
            }else if((!LD1&&LD2&&LD3&&LD4)&&(!starting||starting)) //8
            {

            LCD_WriteStringAtPos("Sharp Right", 0, 5);
            LCD_WriteStringAtPos("8", 1, 13);
            SRV_SetPulseMicroseconds0(Neutral);  //right wheel
            SRV_SetPulseMicroseconds1(BCKFast);  // left wheel   
            starting=0;
            }else if((!LD1&&!LD2&&LD3&&LD4)&&(!starting||starting)) //9
            {

            LCD_WriteStringAtPos("Slight Right", 0, 5);
            LCD_WriteStringAtPos("9", 1, 13);
            SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
            SRV_SetPulseMicroseconds1(BCKFast);  // left wheel   
            starting=0;
            }else if((LD1&&LD2&&LD3&&LD4)&&(!starting||starting)) //10
            {

            LCD_WriteStringAtPos("Reverse", 0, 5);
            LCD_WriteStringAtPos("10", 1, 13);
            SRV_SetPulseMicroseconds0(BCKFast);  //right wheel
            SRV_SetPulseMicroseconds1(FWDFast);  // left wheel   
            starting=0;
            }
        }else
        {
            SRV_SetPulseMicroseconds0(Neutral);  //right wheel
            SRV_SetPulseMicroseconds1(Neutral);  // left wheel 
        }
    }
    
    
    
    
    
}

void drivingState()  // haven't included previous state checking yet
{
    if(!(!LD1&&!LD2&&!LD3&&!LD4)&&!starting&&(!pLD1&&!pLD2&&!pLD3&&!pLD4))
    {
        pitstop+=1;
    }
    if(!LD1&&!LD2&&!LD3&&!LD4&&starting)  // end of track
    {
        LCD_WriteStringAtPos("Starting", 0, 5);
        LCD_WriteStringAtPos("0", 1, 13);
        SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
        SRV_SetPulseMicroseconds1(BCKFast);  // left wheel  
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
    }else
    //section 1
    if((!LD1&&!LD2&&!LD3&&!LD4&&pitstop<2&&!starting||(!LD1&&!LD2&&!LD3&&!LD4&&!starting&&!(!pLD1&&!pLD2&&!pLD3&&pLD4))))  // end of track
    {
        LCD_WriteStringAtPos("End", 0, 5);
        LCD_WriteStringAtPos("1.1", 1, 13);
        SRV_SetPulseMicroseconds0(Neutral);  //right wheel
        SRV_SetPulseMicroseconds1(Neutral);  // left wheel
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else 
        
    // section 2
    if((!LD1&&!LD2&&!LD3&&LD4)&&(pLD1&&!pLD2&&!pLD3&&pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("Slight left", 0, 5);
    LCD_WriteStringAtPos("2.1", 1, 13);    
    SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
    SRV_SetPulseMicroseconds1(BCKFast);  // left wheel   
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((!LD1&&!LD2&&!LD3&&LD4)&&(!pLD1&&!pLD2&&!pLD3&&pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("Turn right", 0, 5);
    LCD_WriteStringAtPos("2.2", 1, 13);
    SRV_SetPulseMicroseconds0(BCKSlow);  //right wheel
    SRV_SetPulseMicroseconds1(BCKFast);  // left wheel   
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((!LD1&&!LD2&&!LD3&&LD4)&&(!pLD1&&!pLD2&&pLD3&&pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("Slight right", 0, 5);
    LCD_WriteStringAtPos("2.3", 1, 13);
    SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
    SRV_SetPulseMicroseconds1(BCKFast);  // left wheel
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else    
        
     //section 3
        
    if((!LD1&&!LD2&&LD3&&LD4)&&(pLD1&&!pLD2&&!pLD3&&pLD4)&&(!starting||starting))
    {
        LCD_WriteStringAtPos("Slight right", 0, 5);
        SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
        SRV_SetPulseMicroseconds1(BCKSlow);  // left wheel
        LCD_WriteStringAtPos("3.1", 1, 13);
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((!LD1&&!LD2&&LD3&&LD4)&&(!pLD1&&!pLD2&&pLD3&&pLD4)&&(!starting||starting))
    {
        LCD_WriteStringAtPos("turning right", 0, 5);
        LCD_WriteStringAtPos("3.2", 1, 13);
        SRV_SetPulseMicroseconds0(FWDSlow-2);  //right wheel
        SRV_SetPulseMicroseconds1(BCKFast);  // left wheel
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((!LD1&&!LD2&&LD3&&LD4)&&(!pLD1&&pLD2&&pLD3&&pLD4)&&(!starting||starting))
    {
        LCD_WriteStringAtPos("turning right", 0, 5);
        LCD_WriteStringAtPos("3.3", 1, 13);
        SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
        SRV_SetPulseMicroseconds1(BCKFast);  // left wheel        
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else 
        
    //section 4    
        
        
    if((!LD1&&LD2&&LD3&&LD4)&&(!pLD1&&!pLD2&&pLD3&&pLD4)&&(!starting||starting))
    {
        LCD_WriteStringAtPos("turning right", 0, 5);
        LCD_WriteStringAtPos("4.1", 1, 13);
        SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
        SRV_SetPulseMicroseconds1(BCKSlow);  // left wheel        (might need to be back fast)
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((!LD1&&LD2&&LD3&&LD4)&&(!pLD1&&pLD2&&pLD3&&pLD4)&&(!starting||starting))
    {
        LCD_WriteStringAtPos("sharp right", 0, 5);
        LCD_WriteStringAtPos("4.2", 1, 13);
        SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
        SRV_SetPulseMicroseconds1(BCKFast);  // left wheel        
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((!LD1&&LD2&&LD3&&LD4)&&(pLD1&&pLD2&&pLD3&&pLD4)&&(!starting||starting))
    {
        LCD_WriteStringAtPos("sharp right", 0, 5);
        LCD_WriteStringAtPos("4.3", 1, 13);
        SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
        SRV_SetPulseMicroseconds1(BCKFast);  // left wheel        
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else
      
    // section 5
        
    if((LD1&&!LD2&&!LD3&&!LD4)&&(pLD1&&!pLD2&&!pLD3&&pLD4)&&(!starting||starting))  // might need to change this back
    {
        LCD_WriteStringAtPos("turn left", 0, 5);
        LCD_WriteStringAtPos("5.1", 1, 13);
        SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
        SRV_SetPulseMicroseconds1(Neutral);  // left wheel        
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((LD1&&!LD2&&!LD3&&!LD4)&&(pLD1&&!pLD2&&!pLD3&&!pLD4)&&(!starting||starting))
    {
        LCD_WriteStringAtPos("Turning left", 0, 5);
        LCD_WriteStringAtPos("5.2", 1, 13);
        SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
        SRV_SetPulseMicroseconds1(Neutral);  // left wheel        
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((LD1&&!LD2&&!LD3&&!LD4)&&(pLD1&&pLD2&&!pLD3&&!pLD4)&&(!starting||starting))
    {
        LCD_WriteStringAtPos("slight left", 0, 5);
        LCD_WriteStringAtPos("5.3", 1, 13);
        SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
        SRV_SetPulseMicroseconds1(BCKSlow);  // left wheel                
        //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else  
     
    //Section 6
        
    if((LD1&&!LD2&&!LD3&&LD4)&&(pLD1&&!pLD2&&!pLD3&&pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("Straight", 0, 5); 
    LCD_WriteStringAtPos("6.1", 1, 13);
    SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
    SRV_SetPulseMicroseconds1(BCKFast);  // left wheel        
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((LD1&&!LD2&&!LD3&&LD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("Straight", 0, 5);
    LCD_WriteStringAtPos("6.2", 1, 13);
    SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
    SRV_SetPulseMicroseconds1(BCKSlow);  // left wheel            
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else
        
    // Section 7
        
    if((LD1&&!LD2&&!LD3&&LD4)&&(pLD1&&!pLD2&&!pLD3&&pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("Straight", 0, 5);
    LCD_WriteStringAtPos("7.1", 1, 13);
    SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
    SRV_SetPulseMicroseconds1(BCKFast);  // left wheel        
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((LD1&&LD2&&!LD3&&!LD4)&&(pLD1&&!pLD2&&!pLD3&&pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("turn Left", 0, 5);
    LCD_WriteStringAtPos("7.2", 1, 13);
    SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
    SRV_SetPulseMicroseconds1(Neutral);  // left wheel            
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((LD1&&LD2&&!LD3&&!LD4)&&(pLD1&&pLD2&&!pLD3&&!pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("move Left2", 0, 5);
    LCD_WriteStringAtPos("7.3", 1, 13);
    SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
    SRV_SetPulseMicroseconds1(Neutral);  // left wheel        
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((LD1&&LD2&&!LD3&&!LD4)&&(pLD1&&pLD2&&pLD3&&pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("move Left", 0, 5);
    LCD_WriteStringAtPos("7.4", 1, 13);
    SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
    SRV_SetPulseMicroseconds1(Neutral);  // left wheel 
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else 
        
    //section 8
        
    if((LD1&&LD2&&LD3&&!LD4)&&(pLD1&&pLD2&&!pLD3&&!pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("move  Left", 0, 5);
    LCD_WriteStringAtPos("8.1", 1, 13);
    SRV_SetPulseMicroseconds0(FWDSlow);  //right wheel
    SRV_SetPulseMicroseconds1(Neutral);  // left wheel     
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((LD1&&LD2&&LD3&&!LD4)&&(pLD1&&pLD2&&pLD3&&!pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("Sharp Left", 0, 5);
    LCD_WriteStringAtPos("8.2", 1, 13);
    SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
    SRV_SetPulseMicroseconds1(BCKSlow);  // left wheel     
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else if((LD1&&LD2&&LD3&&!LD4)&&(!pLD1&&!pLD2&&!pLD3&&!pLD4)&&(!starting||starting))
    {
        
    LCD_WriteStringAtPos("Sharp Left", 0, 5);
    LCD_WriteStringAtPos("8.3", 1, 13);
    SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
    SRV_SetPulseMicroseconds1(BCKSlow);  // left wheel      
    //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    }else 
        
    //section 9    
    if((LD1&&LD2&&LD3&&LD4)&&(!starting||starting)) // off of the track
    {
       LCD_WriteStringAtPos("Reversing", 0, 5); 
       LCD_WriteStringAtPos("9.1", 1, 13);
        SRV_SetPulseMicroseconds0(BCKSlow);  //right wheel
        SRV_SetPulseMicroseconds1(FWDSlow);  // left wheel  
       //last case assignments
        pLD1=LD1;
        pLD2=LD2;
        pLD3=LD3;
        pLD4=LD4;
        starting=0;
    } 
    /*
    else  // delete this at the end so you wont get bad readings
    {
        LCD_WriteStringAtPos("Not Yet", 0, 5); 
        
    }
   */
    
    
    
    
    
    
    
    
}


int endingcheck(int lD1,int lD2,int lD3,int lD4)
{
    pLD1=lD1;
    pLD1=lD2;
    pLD3=lD3;
    pLD4=lD4;
    
    //have it be able to slightly move forward or turn and check if the value has changed for the sensors, if changed return 0, if hasn't changed return 1
    
    return 1;
    
}


void RGB()// configure 
{
    int i =0;
    
    
    
    
}



void DemoCheckpoint()
{
    
     LCD_WriteStringAtPos("Team 13: Brayden",0,0);
     if(DisplayCounter==0)
     {
     SSD_WriteDigits(d1,d2,32,32,0,1,0,0);
     }
     if(mode ==3)
     {
     if(!prt_SWT_SWT0&&!prt_SWT_SWT1)
     {
         LCD_WriteStringAtPos("STP",1,13);
         SRV_SetPulseMicroseconds0(Neutral);  //right wheel
         led3=0; led2=0; led1=0; led0=0;   
     }else if(prt_SWT_SWT0&&!prt_SWT_SWT1)
     {
        LCD_WriteStringAtPos("FWD",1,13);
        SRV_SetPulseMicroseconds0(FWDFast);  //right wheel
        led3=1; led2=1; led1=0; led0=0;
     }else if(!prt_SWT_SWT0&&prt_SWT_SWT1)
     {
        LCD_WriteStringAtPos("REV",1,13);
        SRV_SetPulseMicroseconds0(BCKFast);  //right wheel
        led3=0; led2=0; led1=1; led0=1;
     }else if(prt_SWT_SWT0&&prt_SWT_SWT1)
     {
        LCD_WriteStringAtPos("STP",1,13);
        SRV_SetPulseMicroseconds0(Neutral);  //right wheel
        led3=0; led2=0; led1=0; led0=0;

     }else
     {
        led3=0; led2=0; led1=0; led0=0; 
     }
     
     
     /////////////////////////////////////////////////////////////
     if(!prt_SWT_SWT6&&!prt_SWT_SWT7)
     {
        LCD_WriteStringAtPos("STP",1,0);
        SRV_SetPulseMicroseconds1(Neutral);  //right wheel
        led7=0; led6=0; led5=0; led4=0;

     }else if(prt_SWT_SWT6&&!prt_SWT_SWT7)
     {
         LCD_WriteStringAtPos("FWD",1,0);
         SRV_SetPulseMicroseconds1(BCKFast);  //right wheel
         led7=1; led6=1; led5=0; led4=0;         
     }else if(!prt_SWT_SWT6&&prt_SWT_SWT7)
     {
        LCD_WriteStringAtPos("REV",1,0);
        SRV_SetPulseMicroseconds1(FWDFast);  //right wheel
        led7=0; led6=0; led5=1; led4=1;

     }else if(prt_SWT_SWT6&&prt_SWT_SWT7)
     {
        LCD_WriteStringAtPos("STP",1,0);
        SRV_SetPulseMicroseconds1(Neutral);  //right wheel
        led7=0; led6=0; led5=0; led4=0;

     }else
     {
         led7=0; led6=0; led5=0; led4=0;
         
     }
     }
     
     
     //ssd logic
     timerDisplay();
}

void timerDisplay()
{
    if((prt_SWT_SWT6&&prt_SWT_SWT7)||(prt_SWT_SWT0&&prt_SWT_SWT1))
     {
        d1=0;d2=0;d3=0;d4=0;DisplayCounter=0;  
     }else if(((prt_SWT_SWT6||prt_SWT_SWT7))||(prt_SWT_SWT0||prt_SWT_SWT1))
     {
        /// SSD

        if (DisplayCounter < 10) {
            SSD_WriteDigits(d1, d2, 32, 32, 0, 1, 0, 0);
        } else if (DisplayCounter < 100) {
            SSD_WriteDigits(d1, d2, d3, 32, 0, 1, 0, 0);
        } else if (DisplayCounter < 1000) {
            SSD_WriteDigits(d1, d2, d3, d4, 0, 1, 0, 0);
        } 
        incrementTime();
       // SSD_WriteDigits(d1,d2,d3,d4,0,1,0,0);
        //delay_ms(100);
     }else
     {
        d1=0;d2=0;d3=0;d4=0; DisplayCounter=0;
     }   
    
    
}



void incrementTime()
{
    if(d2== 9 && d1 ==9)
    {  
        if(d3<9)
        {
            d3++;
        }else
        {
            d3=0;
            d4++;
        }
        d1 = 0;
        d2 = 0;
        DisplayCounter+=1;
        
    }else if(d1<9)
    {
        d1=d1+1;

    }else
    {
        if(d2<9)
        {
            d1 =0;
            DisplayCounter+=1;
            d2 ++;
            
        }
    }
    
    
}

void IRcheck()  // IR sensor debug function
{
    if(LD1)
    {
        led0=1;
    }else if(!LD1)
    {
        led0=0;
    }
    
    if(LD2)
    {
        led1=1;
    }else if(!LD2)
    {
        led1=0;
    }
    
    if(LD3)
    {
        led2=1;
    }else if(!LD3)
    {
        led2=0;
    }
    
    if(LD4)
    {
        led3=1;
    }else if(!LD4)
    {
        led3=0;
    }
    
    
    
}




void oc1Setup()
{
    OC1CONbits.OC32 = 0;      //    use 16 bit compare
    OC1CONbits.OCM = 0b110;   //    PWM mode no fault detection
    OC1CONbits.OCTSEL = 0;    //    use timer2 as source
    OC1CONbits.ON = 1;        //    enable OC1 module
    IPC1bits.OC1IP = 6; // interrupt priority
    IPC1bits.OC1IS = 3; // interrupt sub priority
    IFS0bits.OC1IF = 0; // clear interrupt flag
    IEC0bits.OC1IE = 1; // enable interrupt for OC2 module
}
void Timer3Setup()
{
PR3 =((PBCLK/4)/(Time_Frequency*256));  
TMR3 = 0; // to clear timer 2
T3CONbits.TCKPS = 7; // changes the pre-scalar value 
T3CONbits.TGATE = 0; // used to disable the external clock
T3CONbits.TCS = 0;
T3CONbits.ON = 1; // to turn the timer back on
IPC3bits.T3IP = 7; // set the priority to 7
IPC3bits.T3IS = 3; // set the sub priority to 3
IFS0bits.T3IF = 0; // used to clear the flag of the interrupt
IEC0bits.T3IE = 1; // to turn the interrupt on 
macro_enable_interrupts(); // enable interrupts at CPU
}



void __ISR(_TIMER_3_VECTOR) timer2(void)
{
    IEC0bits.T3IE = 0;
    if(mode==2&&pitstop<2)
    {
        if (DisplayCounter < 10) {
            SSD_WriteDigits(d1, d2, 32, 32, 0, 1, 0, 0);
        } else if (DisplayCounter < 100) {
            SSD_WriteDigits(d1, d2, d3, 32, 0, 1, 0, 0);
        } else if (DisplayCounter < 1000) {
            SSD_WriteDigits(d1, d2, d3, d4, 0, 1, 0, 0);
        } 
        incrementTime();
    }
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
}


void __ISR(_CHANGE_NOTICE_VECTOR) CN_Handler(void)   // ISR to check for the IR sensors
{
    IEC1bits.CNDIE= 0;
    drivingInstruction(); 
    PORTD;
    IFS1bits.CNDIF = 0;
    IEC1bits.CNDIE = 1;
}

void delay_ms(int ms){
	int		i,counter;
	for (counter=0; counter<ms; counter++)
    {
        for(i=0;i<1426;i++)
        {
        }   //software delay 1 millisec
    }
}