// PWMtest.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include <stdint.h>
#include "PLL.h"
#include "PWM.h"
#include "ADCSWTrigger.h"

// Function prototypes
void PortF_Init(void);
void PortB_Init(void);
void delay(unsigned long int time);
void Change_B_Polarity(void);
void Display_Info(unsigned long potentiometer, unsigned long sensor1, unsigned long sensor2 );


// Global Variables
unsigned int speed = 0;
unsigned int speedValues[] = {0, 25, 50, 75, 100};
char status = 'R'; //Initialize to red status
char lastStatus = 'G'; //Initialize to red status

// SENSOR VARIABLES
float dist1, dist2;
unsigned long ain1, ain2, ain3, dutyCycle;
char sample=0;
int adcTable[] = {4095, 3050, 1980, 1370, 950, 830, 730, 650, 570, 530, 460, 390, 330, 300, 0};
int distTable[] = {0, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 999};
float distance_ADC = 0;  //  <---- THIS USE TO BE CALLed distance but is now changed to distance_ADC so be aware
float calibration = 0;
float a = 0;
float b = 0;
int ia = 0;
int ib = 0;
float m = 0;
float l = 0;
float lm;
int i;
int f;

unsigned int getPercent(unsigned long ADCvalue){
	unsigned pct = ADCvalue/40;
	if (pct >= 100)	pct = 100;
	return pct;
}

unsigned int getCm(unsigned long ADCvalue){
	return 2.1955322 + 38701.8148/ADCvalue;
}

void delay(unsigned long int time)    // This function provides delay in terms of seconds
{
		//Roughly 1 second delay on 16MHz TM4C
    unsigned char i,j,k,l;
 
    for(i = 0; i < time; i++){
        for(j=0; j<250; j++){
					for(k=0; k< 250; k++){
						for (l=0; l< 60; l++){
						}
					}
				}
		}
}

unsigned int getAbs(int n) 
{ 
  int const mask = n >> (sizeof(int) * 8 - 1); 
  return ((n + mask) ^ mask); 
} 

unsigned int getLookup(unsigned long ADCvalue){
	//innaccurate after 45cm
	int adcOutput[15] = {2930, 2144, 1640, 1350, 1150,  928,  900, 880, 840, 790, 650, 630, 530};
	int distance[15] =  {  10,   15,   20,   25,   30,   35,   40,  45,  50,  55,  60,  65,  70};
	unsigned int closest = getAbs(ADCvalue-adcOutput[0]);
	unsigned int val = distance[0];
	unsigned int i = 0;
	
	for (i = 0; i < 15; i++){
		if (  (getAbs(adcOutput[i]-ADCvalue)) < closest ){
				closest = getAbs(ADCvalue-adcOutput[i]);
				val = distance[i];
		}
	}
	return val;
}


int main(void){
	unsigned long potentiometer, sensor1, sensor2, percent;
	ADC_Init298();
	PortB_Init();
	PortF_Init();
	Nokia5110_Init();
	PWM0A_Init(40000);         // initialize PWM0, 1000 Hz
  PWM0B_Init(40000);         // initialize PWM0, 1000 Hz
	PLL_Init();           // bus clock at 80 MHz
	
	// Initialize speeds based on potentiometer
	signed int leftSpeed = getPercent(potentiometer);
	signed int rightSpeed = getPercent(potentiometer);
	
  while(1){
		ADC_In298(&ain1, &ain2, &ain3); // Ensure sampler works
		// ADC PART OF LOOP	
			
			//Update Sensors
			// Find distance
		for(i = 0; i < 15; i = i + 1){
			if(ain1 > adcTable[i]){
				break;
			}
			else{
				a = adcTable[i+1];
				ia = i+1;
			}
		}
		
		for(f = 0; f < 15; f = f + 1){
			if(ain1 < adcTable[f]){
				b = adcTable[f];
				ib = f;
			}
			else {
				break;
			}
		}
		 m = b - a;
		 l = b - ain1;
		lm = l / m ;
		
		dist1 = distTable[ib] + (lm * 5);
		// Find distance
		for(i = 0; i < 15; i = i + 1){
			if(ain2 > adcTable[i]){
				break;
			}
			else{
				a = adcTable[i+1];
				ia = i+1;
			}
		}
		
		for(f = 0; f < 15; f = f + 1){
			if(ain2 < adcTable[f]){
				b = adcTable[f];
				ib = f;
			}
			else {
				break;
			}
		}
		 m = b - a;
		 l = b - ain2;
		lm = l / m ;
		
		dist2 = distTable[ib] + (lm * 5);
		// END ADC PART OF LOOP
		
		unsigned int leftDistance = getCm(sensor1);
		unsigned int rightDistance = getCm(sensor2);
		
		Display_Info(potentiometer,dist1,dist2);
		
		PWM0A_Duty(0);
		PWM0B_Duty(0);
  }
}

void PortF_Init(void){    
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R |= 0x1F;           // allow changes to PF4-0       
	GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4(sw2), PF0(sw1) inputs (built-in button)
  GPIO_PORTF_DIR_R |=  0x0E;    //  make PF3,PF2,PF1 output+++
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4 - PF0  
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
	GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
  NVIC_PRI7_R |= (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void PortB_Init(void){
	unsigned int delay;
  SYSCTL_RCGC2_R |= 0x00000002;     // 1) B clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R = 0x0F;          // 5) PB2-PB0 output 
  GPIO_PORTB_AFSEL_R = 0x00;        // 6) no alterna=te function
  GPIO_PORTB_DEN_R = 0x0F;          // 7) enable digital pins PB2-PB0    
	GPIO_PORTB_DATA_R = 0x08;          // Initialize PB0 high, PB1 kept low
}



void Display_Info(unsigned long potentiometer, unsigned long sensor1, unsigned long sensor2){
	unsigned int percent = getPercent(potentiometer);
	Nokia5110_Clear();
  Nokia5110_OutString("L:");
	Nokia5110_OutUDec((sensor1));
	
	Nokia5110_SetCursor(0, 2);
	Nokia5110_OutString("R:");
	Nokia5110_OutUDec((sensor2));
	
	Nokia5110_SetCursor(0, 4);
	Nokia5110_OutString("PWM");
	Nokia5110_OutUDec(percent);
	delay(1);
	
}


// Unused functions for future use
void Change_B_Polarity(void){
	GPIO_PORTB_DATA_R ^= 0x0C;
}