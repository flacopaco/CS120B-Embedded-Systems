#include "timer.h"
#include "io.c"
#include <avr/eeprom.h>

enum States {Start, Measure, DisplayMoisturePerc, DisplayWater, SetTemp } state;	//state machine states
enum States2 {Start1, NextState} state2;											//determines next state transition from Measure

// 0.954 hz is lowest frequency possible with this function,
// based on settings in PWM_on()
// Passing in 0 as the frequency will stop the speaker from generating sound
void set_PWM(double frequency) {
	static double current_frequency; // Keeps track of the currently set frequency
	// Will only update the registers when the frequency changes, otherwise allows
	// music to play uninterrupted.
	if (frequency != current_frequency) {
		if (!frequency) { TCCR0B &= 0x08; } //stops timer/counter
		else { TCCR0B |= 0x03; } // resumes/continues timer/counter
		
		// prevents OCR3A from overflowing, using prescaler 64
		// 0.954 is smallest frequency that will not result in overflow
		if (frequency < 0.954) { OCR0A = 0xFFFF; }
		
		// prevents OCR0A from underflowing, using prescaler 64                    // 31250 is largest frequency that will not result in underflow
		else if (frequency > 31250) { OCR0A = 0x0000; }
		
		// set OCR3A based on desired frequency
		else { OCR0A = (short)(8000000 / (128 * frequency)) - 1; }

		TCNT0 = 0; // resets counter
		current_frequency = frequency; // Updates the current frequency
	}
}

void PWM_on() {
	TCCR0A = (1 << COM0A0) | (1 << WGM00);
	// COM3A0: Toggle PB3 on compare match between counter and OCR0A
	TCCR0B = (1 << WGM02) | (1 << CS01) | (1 << CS00);
	// WGM02: When counter (TCNT0) matches OCR0A, reset counter
	// CS01 & CS30: Set a prescaler of 64
	set_PWM(0);
}

void PWM_off() {
	TCCR0A = 0x00;
	TCCR0B = 0x00;
}

//Read the ADC value
short readADC(uint8_t ch){
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	
	// start single conversion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes ’0? again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}
//initialize ADC and enable conversion
void ADC_init() {
	// AREF = AVcc
	ADMUX = (1<<REFS0);
	// ADC Enable and prescaler of 12
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}




//for printing moisture level
char buffer[16];

void startMoist(short moisture){
	LCD_ClearScreen();
	sprintf(buffer,"Last Moisture   Level: %d%%       ",moisture);
	LCD_DisplayString(1, buffer);
}
//displays soil moisture percentage
void dispMoist(short moisture){
	LCD_ClearScreen();
	sprintf(buffer,"Moisture Level:       %d%%        ", moisture);
	LCD_DisplayString(1, buffer);
}
//displays whether it is dry or muddy
void DryOrMuddy(short moisture){
	LCD_ClearScreen();
	if(moisture <= 50){
		LCD_DisplayString(1, "Soil  Moisture:       DRY       ");
		}else{
		LCD_DisplayString(1, "Soil  Moisture:       MUDDY    ");
	}
}
void tempDisp(short x){
	LCD_ClearScreen();
	sprintf(buffer,"Temperature:          TBA     ", x);
	LCD_DisplayString(1, buffer);
}

void dispMoistureMat(unsigned char moisture){
	if(moisture < 10){
		PORTC = 0x00;
		set_PWM(400);
		}else if(moisture < 20){
		PORTC = 0x01;
		set_PWM(450);
		}else if(moisture < 30){
		PORTC = 0x03;
		set_PWM(500);
		}else if(moisture < 40){
		PORTC = 0x07;
		set_PWM(550);
		}else if(moisture < 50){
		PORTC = 0x0F;
		set_PWM(600);
		}else if(moisture < 60){
		PORTC = 0x1F;
		set_PWM(650);
		}else if(moisture < 70){
		PORTC = 0x3F;
		set_PWM(700);
		}else if(moisture < 80){
		PORTC = 0x7F;
		set_PWM(750);
		}else if(moisture > 90){
		PORTC = 0xFF;
		set_PWM(800);
	}
}

uint8_t lastPerc;
uint8_t ByteOfData;
short moistureP = 0;
short ud = 0;
short lr = 0;
short temp = 0;

unsigned char stateChange = DisplayMoisturePerc;

void Tick(){
	switch(state){
		case Start:
		lastPerc = eeprom_read_byte((uint8_t*)30);
		temp = lastPerc;
		ByteOfData = eeprom_read_byte((uint8_t*)46);
		moistureP = ByteOfData;
		startMoist(moistureP);
		state = Measure;
		break;
		
		case Measure:
		state = stateChange;
		break;
		
		case DisplayMoisturePerc:
		ud = readADC(1);
		lr = readADC(2);
		if(ud >= 50){
			stateChange = DisplayWater;
			}else if(lr >= 50){
			stateChange = SetTemp;
		}
		
		state = Measure;
		break;
		
		case DisplayWater:
		ud = readADC(1);
		if(ud < 20){
			stateChange = DisplayMoisturePerc;
		}
		
		state = Measure;
		break;
		
		case SetTemp:
		lr = readADC(2);
		if(lr < 20){
			stateChange = DisplayMoisturePerc;
		}
		state = Measure;
		break;
		default:
		state = Start;
		break;
	}
	
	switch(state){
		case Start:
		break;
		
		case Measure:
		moistureP = readADC(0) / 7;
		ByteOfData = moistureP;
		eeprom_update_byte((uint8_t*)46, ByteOfData);
		break;
		
		case DisplayMoisturePerc:
		
		dispMoistureMat(moistureP);
		dispMoist(moistureP);
		break;
		
		case DisplayWater:

		dispMoistureMat(moistureP);
		DryOrMuddy(moistureP);
		break;
		
		case SetTemp:
		
		temp = readADC(3) / 8;
		lastPerc = temp;
		eeprom_update_byte((uint8_t*)30, lastPerc);
		tempDisp(temp);
		
		break;
		
		default:
		break;
	}
}

void Tick2(){
	switch(state2){
		case Start1:
		state2 = NextState;
		break;
		
		case NextState:
		
		if(state == DisplayMoisturePerc){
			
			} else if (state == DisplayWater){

			} else if (state == SetTemp){

		}
		state2 = NextState;
		
		break;
		default:
		state2 = Start;
		break;
	}
}

int main(){
	state = Start;
	state2 = Start1;
	DDRA = 0xFF; PORTA = 0x00;	//initialize ports
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	
	PWM_on();
	TimerSet(125);				//initialize timer, ADC, and LCD
	TimerOn();
	ADC_init();
	LCD_init();
	LCD_ClearScreen();
	

	while (1){
		Tick();
		Tick2();
		while (!TimerFlag);
		TimerFlag = 0;
	}
	return 0;
}

