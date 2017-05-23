/*
* Partner(s) Name & E-mail: Andrew Apostol aapos001@ucr.edu, Marlo Zeroth mzero001@ucr.edu
* Lab Section:22
* Assignment: Lab # 10 Exercise # 3
* Exercise Description: [PWM]
*
* I acknowledge all content contained herein, excluding template or example
* code, is my own original work.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "bit.h" // For setting getting bit
#include "timer.h" // For general concurrent synchronous state machine timing
#include "queue.h" 
#include "uart.h" // For communicating using usart
#include "pwm.h"
#include "adc.h"
#include <stdio.h>
#include <avr/eeprom.h> // For reading/writing to eeprom

//--------Find GCD function --------------------------------------------------
unsigned long int findGCD( unsigned long int a, unsigned long int b)
{
	unsigned long int c;
	while (1){
		c = a%b;
		if (c==0){ return b;}
		a = b;
		b = c;
	}
	return 0;
}

//--------End find GCD function ----------------------------------------------
//--------Task scheduler data structure---------------------------------------
// Struct for Tasks represent a running process in our simple real-time operating system.
typedef struct _task {
	/*Tasks should have members that include: state, period,
	a measurement of elapsed time, and a function pointer.*/
	signed char state; //Task's current state
	unsigned long int period; //Task period
	unsigned long int elapsedTime; //Time elapsed since last task tick
	int (*TickFct)( int ); //Task tick function
} task;

//--------End Task scheduler data structure-----------------------------------
//--------Shared Variables----------------------------------------------------
unsigned char system_on = 0; // Turns on/off the system
unsigned short temperature = 0; // Holds the current temperature
unsigned char distance = 0; // Holds the current distance
unsigned char temperature_on_light = 5; // Controls light for sensing temperature
unsigned char system_on_light = 4; // Pin location for the system on light
unsigned char transmit_light = 6; // Pin location for the transmit light on
char output_buffer[100];
unsigned short bufferNumberCharWritten; // Number of characters written to output_buffer

const unsigned char usart0 = 0; // USART0
const unsigned char usart1 = 1; // USART1
const unsigned char MODpin = 5; // Pin number for the bluetooth module MOD pin
unsigned char MODpinState = 0; // State of the pin 0 for ground 1 for high
const unsigned char CTSpin = 4; // Pin number for the bluetooth module CTS pin
unsigned char CTSpinState = 0; // State of the pin 0 for ground 1 for high
const unsigned char DFUpin = 7; // Pin number for the bluetooth module DFU pin
unsigned char DFUpinState = 0; // State of the pin 0 for ground 1 for high

//--------End Shared Variables------------------------------------------------

//--------User defined FSMs---------------------------------------------------

// Enumeration of states.
enum SM1_States {SM1_Wait, SM1_ButtonPress, SM1_ButtonRelease};

// Mealy machine
int SMTick1(int state){

	unsigned char press;
	press = ~PINB & 0x01;
	switch(state){
		
		case SM1_Wait:
			if(press){
				state = SM1_ButtonPress;
			}
			break;
			
		case SM1_ButtonPress:
			state = SM1_ButtonRelease;
			break;

		case SM1_ButtonRelease:
			if(!press){
				state = SM1_Wait;
			}
			break;	
			
		default:
			state = SM1_Wait;
			break;
	}
	
	switch(state){
		case SM1_Wait:
			break;
			
		case SM1_ButtonPress:
			system_on = (system_on == 0)? 1 : 0; // Toggle the system
			system_on_light = (system_on == 0)? 0x00 : 0x10;
			break;
		
		case SM1_ButtonRelease:
			break;
			
		default:
			break;
	}
	return state;	
}

//Enumeration of states.
enum SM2_States {SM2_sense};

int SMTick2(int state){
	// Local variables
	static unsigned short registerADC;	
	switch(state){
		case SM2_sense:
			state = SM2_sense;
			break;
		
		default:
			state = SM2_sense;
			break;
	}
	
	switch(state){
		case SM2_sense:			
			// Uncomment to use single conversion mode on ADC
			//char mux = 0;
			//registerADC = sensor_read(mux); // Read the value from the ADC register
			
			// Use free running conversion mode
			// Get sensor current value
			registerADC = ADC;	
			temperature = ( (registerADC*(5000/1024)) - 500)/10;
			bufferNumberCharWritten = sprintf(output_buffer,"%d C", temperature);			
			break;
			
		default:
			break;
	}
	return state;
}

//Enumeration of states.
enum SM3_States { SM3_Off, SM3_transmit};

int SMTick3(int state){
	// Local variables
	
	switch(state){
		case SM3_Off:
			if(system_on){
				state = SM3_transmit;
				//PORTD = PIND & ~(1 << MODpin); // Enable data mode for the bluetooth module by clearing the MOD pin
			}
			break;
		
		case SM3_transmit:
			if(!system_on){
				state = SM3_Off;
			}
			break;
		
		default:
			state = SM3_Off;
			break;
	}
	
	switch(state){
		case SM3_Off:
			break;
		
		case SM3_transmit:
		    {
				PORTD &= ~(1 << CTSpin); // Enable transmit for the bluetooth module by clearing the CTS pin
				unsigned short i = 0;
				unsigned short bufferSize = sizeof (output_buffer)/ sizeof (char);
				for(i = 0; i <= bufferNumberCharWritten; i++){
					if(USART_IsSendReady(usart0)){
						USART_Send(output_buffer[i],usart0);
						USART_Flush(usart0);// flush the system
					}
				}			
			}
			break;
		
		default:
			break;
	}
	return state;
}


//Enumeration of states.
enum SM4_States { SM4_display };
	
// Displays the current buffer onto the LCD
int SMTick4 ( int state) {
	// Local Variables
	//State machine transitions
	
	switch (state) {
		
		case SM4_display: 
			state = SM4_display;
			break;
		default:
			state = SM4_display ;
			break;
	}
	//State machine actions
	switch (state) {
		case SM4_display : 
			// write shared outputs
			// to local variables
			LCD_DisplayString(1,output_buffer);
			//LCD_DisplayString(1,"Hello World");
			PORTB = (system_on_light | temperature_on_light) | 0x0F ;
			break ;
			
		default: 
			break ;
	}
	
	//PORTB = output; // Write combined, shared output variables to PORTB
	return state;
}


//Enumeration of states.
enum SM5_States { SM5_Off, SM5_PWMon};
const double frequency = 329.63;
int SMTick5(int state){
	// Local variables
	
	switch(state){
		case SM5_Off:
			if(temperature > 27){
				state = SM5_PWMon;
				PWM_on();
			}
			break;
		
		case SM5_PWMon:
			if( temperature < 27 ){
				state = SM5_Off;
			}
			break;
		
		default:
			state = SM5_Off;
			break;
	}
	
	switch(state){
		case SM5_Off:
			PWM_off();
			break;
		
		case SM5_PWMon:
		    {
				set_PWM(frequency);	
			}
			break;
		
		default:
			break;
	}
	return state;
}



// --------END User defined FSMs-----------------------------------------------
// Implement scheduler code from PES.
int main()
{
	// Set Data Direction Registers
	// Buttons PORTA[0-7], set AVR PORTA to pull down logic
	DDRA = 0x00; PORTA = 0xFF; // Analog Sensor input
	DDRB = 0xF0; PORTB = 0x0F; // P0-3 input, p4-7: 3 LEDs & Bluetooth DFU module control line
	DDRC = 0xFF; PORTC = 0x00; // LCD control lines
	DDRD = 0xFA; PORTD = 0x05; // P0-5 Bluetooth module control lines (UART), P6-7 LCD control lines
	// . . . etc

	// Period for the tasks
	unsigned long int SMTick1_calc = 50;
	unsigned long int SMTick2_calc = 5000;
	unsigned long int SMTick3_calc = 5000;
	unsigned long int SMTick4_calc = 5000;
	unsigned long int SMTick5_calc = 1;

	//Calculating GCD
	unsigned long int tmpGCD = 1;
	tmpGCD = findGCD(SMTick1_calc, SMTick2_calc);
	tmpGCD = findGCD(tmpGCD, SMTick3_calc);
	tmpGCD = findGCD(tmpGCD, SMTick4_calc);
	tmpGCD = findGCD(tmpGCD, SMTick5_calc);

	//Greatest common divisor for all tasks or smallest time unit for tasks.
	unsigned long int GCD = tmpGCD;

	//Recalculate GCD periods for scheduler
	unsigned long int SMTick1_period = SMTick1_calc/GCD;
	unsigned long int SMTick2_period = SMTick2_calc/GCD;
	unsigned long int SMTick3_period = SMTick3_calc/GCD;
	unsigned long int SMTick4_period = SMTick4_calc/GCD;
	unsigned long int SMTick5_period = SMTick5_calc/GCD;

	//Declare an array of tasks
	static task task1, task2, task3, task4, task5;
	task *tasks[] = { &task1, &task2, &task3, &task4, &task5 };
	const unsigned short numTasks = sizeof (tasks)/ sizeof (task*);

	// Task 1
	task1.state = -1; //Task initial state.
	task1.period = SMTick1_period; //Task Period.
	task1.elapsedTime = SMTick1_period; //Task current elapsed time.
	task1.TickFct = &SMTick1; //Function pointer for the tick.

	// Task 2
	task2.state = -1; //Task initial state.
	task2.period = SMTick2_period; //Task Period.
	task2.elapsedTime = SMTick2_period; //Task current elapsed time.
	task2.TickFct = &SMTick2; //Function pointer for the tick.

	// Task 3
	task3.state = -1; //Task initial state.
	task3.period = SMTick3_period; //Task Period.
	task3.elapsedTime = SMTick3_period; //Task current elapsed time.
	task3.TickFct = &SMTick3; //Function pointer for the tick.

	// Task 4
	task4.state = -1; //Task initial state.
	task4.period = SMTick4_period; //Task Period.
	task4.elapsedTime = SMTick4_period; // Task current elasped time.
	task4.TickFct = &SMTick4; // Function pointer for the tick.
	
	// Task 5
	task5.state = -1; //Task initial state.
	task5.period = SMTick5_period; //Task Period.
	task5.elapsedTime = SMTick5_period; // Task current elasped time.
	task5.TickFct = &SMTick5; // Function pointer for the tick.


	// Set the timer and turn it on
	TimerSet(GCD);
	TimerOn();
	unsigned short i; // Scheduler for-loop iterator
	ADC_init(); // Initialize ADC register
	LCD_init();
	initUSART(usart0); // We use on D0 & D1
	

	while (1) {
		// Scheduler code
		for ( i = 0; i < numTasks; i++ ) {
			// Task is ready to tick
			if ( tasks[i]->elapsedTime == tasks[i]->period ) {
				// Setting next state for task
				tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
				// Reset the elapsed time for next tick.
				tasks[i]->elapsedTime = 0;
			}
			tasks[i]->elapsedTime += 1;
		}
		while (!TimerFlag);
		TimerFlag = 0;
	}
	// Error: Program should not exit!
	return 0;
}