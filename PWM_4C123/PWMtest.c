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

// Function prototypes
void PortF_Init(void);
void PortB_Init(void);
void delay(unsigned int seconds);
void Change_LED(char color);
void Change_B_Polarity(void);
void Display_Info(char status, int speed);


// Global Variables
unsigned int speed = 0;
unsigned int speedValues[] = {0, 25, 50, 75, 100};
char status = 'R'; //Initialize to red status
char lastStatus = 'G'; //Initialize to red status

int main(void){
	PortB_Init();
	PortF_Init();
	Nokia5110_Init();
	PWM0A_Init(40000);         // initialize PWM0, 1000 Hz, 75% duty
  PWM0B_Init(40000);         // initialize PWM0, 1000 Hz, 25% duty
	Display_Info(status, speed);
  while(1){
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
	Change_LED(status); // Initialize status led
}

void PortB_Init(void){
	unsigned int delay;
  SYSCTL_RCGC2_R |= 0x00000002;     // 1) B clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R = 0x0F;          // 5) PB2-output 
  GPIO_PORTB_AFSEL_R = 0x00;        // 6) no alterna=te function
  GPIO_PORTB_DEN_R = 0x0F;          // 7) enable digital pins PB2-PB0    
	GPIO_PORTB_DATA_R = 0x05;          // Initialize PB0, PB2 high
}

void GPIOPortF_Handler(void){
// Dual push button ISR
// RED LED indicates no robot motion
// BLUE LED indicates backward direction
// GREEN LED indicates forward direction
	
	//Motor speeds: 0,60,70,85,100
  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch (Speed)
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		if      (speed ==  speedValues[0]){
			speed = speedValues[1];
			status = lastStatus;
		}
		else if (speed == speedValues[1]) speed = speedValues[2];
		else if (speed == speedValues[2]) speed = speedValues[3];
		else if (speed == speedValues[3]) speed = speedValues[4];
		else if (speed ==speedValues[4]){
			speed = speedValues[0];
			lastStatus = status;
			status = 'R';
		}
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch (Direction)
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		if      (status == 'G'){
			status = 'B';
			Change_B_Polarity();
		}
		else if (status == 'B'){
			status = 'G';
			Change_B_Polarity();
		}
	}
	PWM0A_Duty(speed);
	PWM0B_Duty(speed);
	Change_LED(status);
	Display_Info(status, speed);
}

void Change_LED(char color){
	// Port F Onboard LED Color Codes
	// Color    LED(s) PortF
	// dark     ---    0
	// red      R--    0x02
	// blue     --B    0x04
	// green    -G-    0x08
	// yellow   RG-    0x0A
	// sky blue -GB    0x0C	
	// white    RGB    0x0E
	// pink     R-B    0x06
	if (color == 'R') GPIO_PORTF_DATA_R = 0x02;
	if (color == 'B') GPIO_PORTF_DATA_R = 0x04;
	if (color == 'G') GPIO_PORTF_DATA_R = 0x08;
	if (color == 'X') GPIO_PORTF_DATA_R = 0x00;
}

void Change_B_Polarity(void){
	GPIO_PORTB_DATA_R = ~GPIO_PORTB_DATA_R;
}

void Display_Info(char status, int speed){
	Nokia5110_Clear();
  Nokia5110_OutString("DIR    ");
	
	if      (status == 'R') Nokia5110_OutChar('S');
	else if (status == 'G') Nokia5110_OutChar('F');
	else if (status == 'B') Nokia5110_OutChar('B');
	else    								Nokia5110_OutChar('?');
	
	Nokia5110_SetCursor(0,2);
	Nokia5110_OutString("PWM");
	Nokia5110_OutUDec(speed);
}
