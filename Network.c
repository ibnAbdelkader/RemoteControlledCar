// Network.c
// Runs on LM4F120
// This test file demonstrates the UART.  Download this
// program to two different Launch Pads and connect their
// UART1s and grounds together.  Connect PC4 of the first
// board to PC5 of the second board and PC4 of the second
// board to PC5 of the first board.  Connect the grounds
// together.  When powered, both multi-colored LEDs will
// come on.  Press SW1 to cycle through the color wheel.
// Press SW2 to send the current color to the other board.
// Daniel Valvano
// August 1, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// U1Rx (PC4) connected to U1Tx (PC5) of other microcontroller
// U1Tx (PC5) connected to U1Rx (PC4) of other microcontroller
// Ground connected ground of other microcontroller

// SW2 (send color) connected to PF0
// Red LED connected to PF1
// Blue LED connected to PF2
// Green LED connected to PF3
// SW1 (step color) connected to PF4

//#include "PLL.h"
#include "SysTick.h"
#include "UART.h"

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
	
/*******************************************************************************/

#define GPIO_PORTD_DATA_BITS_R  ((volatile unsigned long *)0x40007000)
#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_PUR_R        (*((volatile unsigned long *)0x40007510))
#define GPIO_PORTD_PDR_R        (*((volatile unsigned long *)0x40007514))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_LOCK_R       (*((volatile unsigned long *)0x40007520))
#define GPIO_PORTD_CR_R         (*((volatile unsigned long *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))



/********************************************************************************/
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	
void Delay(void);
void go_forward(void);
void go_back(void);
void turn_left(void);
void turn_right(void);
void forward_left(void);
void forward_right(void);
void back_left(void);
void back_right(void);

void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x20;         // 1) activate Port F
  delay = SYSCTL_RCGC2_R;         // allow time for clock to stabilize
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // 2) unlock Port F lock
  GPIO_PORTF_CR_R = 0x1F;         //    enable commit (allow configuration changes) on PF4-0
  GPIO_PORTF_AMSEL_R = 0x00;      // 3) disable analog functionality on PF4-0
  GPIO_PORTF_PCTL_R = 0x00000000; // 4) configure PF4-0 as GPIO
  GPIO_PORTF_DIR_R = 0x0E;        // 5) PF4 and PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;      // 6) disable alt funct on PF4-0
  GPIO_PORTF_DEN_R = 0x1F;        // 7) enable digital I/O on PF4-0
  GPIO_PORTF_PUR_R = 0x11;        //    enable pull-up on PF4 and PF0
}
/*********************************************************/

/**************************************************************/
void PortD_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x08;         // 1) activate Port D
  delay = SYSCTL_RCGC2_R;         // allow time for clock to stabilize
  GPIO_PORTD_LOCK_R = 0x4C4F434B; // 2) unlock Port D lock
  GPIO_PORTD_CR_R = 0x0F;         //    enable commit (allow configuration changes) on PD0-3
  GPIO_PORTD_AMSEL_R = 0x00;      // 3) disable analog functionality on PF4-0
  GPIO_PORTD_PCTL_R = 0x00000000; // 4) configure PD3-0 as GPIO
  GPIO_PORTD_DIR_R = 0x0F;        // 5) out PD0-3
  GPIO_PORTD_AFSEL_R = 0x00;      // 6) disable alt funct on PD3-0
  GPIO_PORTD_DEN_R = 0x0F;        // 7) enable digital I/O on PD3-0
  //GPIO_PORTD_PUR_R = 0x11;        //    enable pull-up on D4 and PF0
}

//  red, yellow, green, light blue, blue, purple,  white,  dark
const long ColorWheel[8] = {0x02,0x0A,0x08,0x0C,0x04,0x06,0x0E,0x00};
int main(void){ unsigned long SW1,SW2;
  long prevSW1 = 0;        // previous value of SW1
  long prevSW2 = 0;        // previous value of SW2
  unsigned char test;   // color value from other microcontroller
	unsigned char test2;
  unsigned char color = 0; // this microcontroller's color value
 // PLL_Init();              // set system clock to 80 MHz
  SysTick_Init();          // initialize SysTick
  UART_Init();             // initialize UART
  PortF_Init(); 
  PortD_Init();	// initialize buttons and LEDs on Port F
  while(1){
		//GPIO_PORTF_DATA_R=ColorWheel[2];
    test = UART_InChar();
		
		/********************************************************************************/
		//forward
		if (test=='F')
		{ 
			
			go_forward();
			if (GPIO_PORTD_DATA_R & 0x01)
			{ 
				GPIO_PORTF_DATA_R= ColorWheel[3];
				
			}
		}
		/*************************************************************************************/
		//back
		if (test=='B')
		{ 
					go_back();
		}
		/*********************************************************************************************/
		//left
		if (test=='L')
		{ 
			turn_left();
			
		}
		/*********************************************************************************************/
		//right
		if (test=='R')
		{ 
			turn_right();
			
			
		}
		/*********************************************************************************************/
		
		//forward left
		if (test=='G')
		{ 
			
			forward_left();
			
		}
		
		
		
		//forward right
		if (test=='I')
		{ 
			
			forward_right();
			
		}
		/*********************************************************************************************/
		
		
		
		//back left
		if (test=='H')
		{ 
			
			back_left();
			
		}
		
		
		//back right
		if (test=='J')
		{ 
			
			back_right();
			
		}
		
		if (test=='S')
		{ 
			GPIO_PORTD_DATA_R=0x00;
		}
		
	}
}
void Delay(void){unsigned long volatile time;
  time = 145448;  // 0.1sec
  while(time){
		time--;
  }
}
void go_forward(void){
	
	GPIO_PORTD_DATA_R=0x01;
}
void go_back(void){
	GPIO_PORTD_DATA_R=0x02;
	
}
void turn_left(void){
	GPIO_PORTD_DATA_R=0x08;
}
void turn_right(void){
	
	GPIO_PORTD_DATA_R=0x04;
}
void forward_left(void){
	GPIO_PORTD_DATA_R=0x09;
}
void forward_right(void){
	GPIO_PORTD_DATA_R=0x05;
}
void back_left(void){
	GPIO_PORTD_DATA_R=0x0A;
}

void back_right(void){
	GPIO_PORTD_DATA_R=0x06;
	
}