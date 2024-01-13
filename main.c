////////////////////////////////////////////////////
// weller_driver_v09.c
////////////////////////////////////////////////////
// Copyright 2015 - 2017 Jaakko Kairus
// jaakko@kair.us
//
// You may freely use and modify this software for personal use.
// Commercial use of this software or parts of it is prohibited
// without written permission from author.
// DISCLAIMER: This software comes with ABSOLUTELY NO WARRANTY
//
// For more information about this project see project web page at
// http://kair.us/projects/weller/
//
// Compiled with CCS compiler version 5.054
// Total compiled length 5298 bytes
///////////////////////////////////////////////////
// Version history
// v0.1 30.5.2015
//      Initial version. For modified kuivanapitolämmitin board.
// v0.2 4.6.2015
//      Reduce ADC averaging from 512 to 16 to speed up heater control response
//      Temperature reading is now 16 bit and not 21 bit anymore
// v0.3 3.11.2015
//		First FW version for dedicated Weller driver v1 board
//		Added menus to adjust setback temp and delay, poweroff delay, temperature
//		units and step size. Diagnostics menu to show cold point compensation temp,
//		both thermocouple temperatures separately, identified iron type, reed status
//		and correcting for reference tolerance. Moved 7-segment character defines
//		to separate header file. Changed saa_1064 function to allow displaying of
//		temperature, numeric values without decimals and reference with decimal.
//		Added a lot of messages to support the new menu structure.
// v0.4 22.12.2015
//		Cleanup for release
// v0.5 27.12.2015
//		Fixed saving temperature setting (broken because of re-organize of EEPROM addresses)
//		Remove display addressing of show_7seg_num function because only one display used
// v0.6 10.1.2017
//		Changed back to using resistor to recognize tip type to reduce buzz with WMRT.
//		Added FW version display to diagnostics menu.
//		Added menu setting 'poor'; when enabled, assumes plain WMRP tip connected w/o
//		any resistors. Useful if only used for WMRP connected to a 3.5 mm socket.
//		Added mains frequency filtering to temperature readout. Changed temperature readout
//		to happen at fixed intervals (10 ms for 50 Hz mains, 8.32 ms for 60 Hz mains).
//		Two consecutive temperature readings are averaged which filters away the mains
//		frequency. Added new menu setting to choose 50 Hz or 60 Hz. Shortening the interval
//		required different optimizations to still achieve long enough heating cycle.
//		Added filtering to KTY readout (slower response, more averaging)
//		Splitted old menuMachine() function to separate updateDispay() and menuMachine()
//		functions, now slow updateDisplay() is only called once during the heating cycle,
//		menuMachine is called in interrupt, after encoder readout. Ensures no encoder 
//		events are missed.
// v0.7	31.1.2017
//		Reduced maximum heating duty cycle back to about 69% (63% at 60Hz) . Was about 86%
//		with FW 0.6 which may be too high and stress the tip. Added some sanity checks to
//		EEPROM reads and writes.
// v0.8	5.2.2017
//		Fixed time delays to use the 832 us interrupt period. Better button debounce.
//		Fixed tip detection when plugging in iron which is on stand (reed closed)
// v0.9 28.7.2017
//		Added check for all parameters which are stored to EEPROM. If any of parameters
//		are out of valid range, all of them are set to default values. This fixes problem
//		when using MPLAB IPE to program the PIC, and EEPROM is not erased or programmed.
//		Added clearing of display anodes before parameter save to avoid bright flash of
//		currently active segments.
// v0.901 28.7.2017
//		Fixed setback and poweroff incase delay set to 0 minutes. Added some fixes to
//		tip type recognition at startup to prevent errorneous WMRT detection. Added
//		workaround to allow returning from setback for five seconds even if setback
//		delay set to zero. Allows e.g. to put the iron to standby from setback with two
//		consecutive button presses.

#define FW_VERSION 901
#define CC_DISPLAY	// Actually we use common anode display but as we mux
					// segments and not digits we must use inverted patterns!

#include <16F1788.h>
#device ADC=12;
#include "7seg_chars.h"
#include <stdlib.h>
#include <stdio.h>
#include <internal_eeprom.c>	// for reading / writing int16
#fuses NOWDT,NOFCMEN,NOIESO,BROWNOUT,BORV25,NOPROTECT,NOLVP,NODEBUG,PUT,NOMCLR
#use delay(int=16MHz, clock=32MHz)
#use FAST_IO(a)
#use FAST_IO(b)
#use FAST_IO(c)
#use FAST_IO(e)

// Some defaut parameter values to EEPROM. Not necessary since all EEPROM values are verified to be
// in range at startup.
// #rom 0xf000 = {0x7c,0x01,0xfa,0x00,0x05,0x00,0x1e,0x00}
// #rom 0xf008 = {0x00,0x00,0x00,0x08,0x05,0x00,0x00,0x00}
// #rom 0xf010 = {0x32}

#define REED_PULLUP PIN_B3
#define HEATER_1 PIN_B4
#define HEATER_2 PIN_B5

// Assign some 'factory default' values to use when run for the first time
// These are also used if any of the parameters stored in eeprom are outside
// of valid range
#define DEFAULT_SETPOINT 380
#define DEFAULT_SETBACK 250
#define DEFAULT_SETBACK_DELAY 5
#define DEFAULT_POWEROFF_DELAY 30
#define DEFAULT_TEMPERATURE_OFFSET 0
#define DEFAULT_REFERENCE 2048
#define DEFAULT_STEPSIZE 5
#define DEFAULT_TEMPERATURE_UNIT DISP_C
#define DEFAULT_POORMODE 0
#define DEFAULT_MAINSFREQUENCY 50

// Assign sane / safe limits to the above parameters
#define MAX_SETPOINT 450				// these are also used
#define MIN_SETPOINT 100				// for setback range
#define MAX_SETBACK_DELAY 30
#define MAX_POWEROFF_DELAY 120
#define MAX_TEMPERATURE_OFFSET 40
#define MAX_REFERENCE_TOLERANCE 103		// max. tolerance is 5% of 2.048

// Message examples
byte const  cold[5] = {_C,_O,_L,_D,_SPACE}; //  CoLd
byte const  stby[5] = {_S,_T,_B,_Y,_SPACE}; //  Stby
byte const  setb[5] = {_S,_E,_T,_B,_SPACE}; //  SEtb
byte const  bacc[5] = {_B,_A,_C,_C,_SPACE}; //  bAcc
byte const  dela[5] = {_D,_E,_L,_A,_SPACE}; //  dELA
byte const   off[5] = {_SPACE,_O,_F,_F,_SPACE}; //  OFF
byte const  poff[5] = {_P,_O,_F,_F,_SPACE}; //  POFF
byte const  ofse[5] = {_O,_F,_S,_E,_SPACE}; //  oFSE
byte const  unit[5] = {_U,_N,_I,_T,_SPACE}; //  Unit
byte const  step[5] = {_S,_T,_E,_P,_SPACE}; //  StEP
byte const  diag[5] = {_D,_I,_A,_G,_SPACE}; //  diAG
byte const   ref[5] = {_SPACE,_R,_E,_F,_SPACE}; //  rEF
byte const  type[5] = {_T,_Y,_P,_E,_SPACE}; //  tyPE
byte const  wmrp[5] = {_W,_M,_R,_P,_SPACE}; //  WMrP
byte const  wmrt[5] = {_W,_M,_R,_T,_SPACE}; //  WMrt
byte const    nc[5] = {_SPACE,_SPACE,_N,_C,_SPACE}; //  nC
byte const  reed[5] = {_R,_E,_E,_D,_SPACE}; //  rEEd
byte const  open[5] = {_O,_P,_E,_N,_SPACE}; //  oPEn
byte const  clos[5] = {_C,_L,_O,_S,_SPACE}; //  CLoS
byte const  tc_1[5] = {_T,_C,_SPACE,_1,_SPACE}; //  tC 1
byte const  tc_2[5] = {_T,_C,_SPACE,_2,_SPACE}; //  tC 2
byte const  poor[5] = {_P,_O,_O,_R,_SPACE}; //  Poor
byte const    on[5] = {_SPACE,_SPACE,_O,_N,_SPACE}; // on
byte const  freq[5] = {_F,_R,_E,_Q,_SPACE}; //	FrEQ
byte const  vers[5] = {_V,_E,_R,_S,_SPACE}; //  vErS

int8 const hex_table[16]= {_0,_1,_2,_3,_4,_5,_6,_7,_8,_9,_A,_B,_C,_D,_E,_F};

typedef enum {
        ST_MAIN,
        ST_SETBACK,
        ST_STANDBY,
        ST_MENU_MAIN,
        ST_MENU_SETBACK,
        ST_ADJ_SETBACK,
        ST_MENU_DELAY,
        ST_ADJ_DELAY,
        ST_MENU_POWEROFF,
        ST_ADJ_POWEROFF,
        ST_MENU_OFFSET,
        ST_ADJ_OFFSET,
        ST_MENU_UNIT,
        ST_ADJ_UNIT,
        ST_MENU_STEP_SIZE,
        ST_ADJ_STEP_SIZE,
        ST_MENU_DIAGNOSTICS,
        ST_MENU_BACK_FROM_DIAGNOSTICS,
        ST_MENU_COLD_COMPENSATION,
        ST_SHOW_COLD_COMPENSATION,
        ST_MENU_REFERENCE,
        ST_ADJ_REFERENCE,
        ST_MENU_TIP_TYPE,
        ST_SHOW_TIP_TYPE,
        ST_MENU_REED_STATE,
        ST_SHOW_REED_STATE,
        ST_MENU_TC_1_READING,
        ST_SHOW_TC_1_READING,
        ST_MENU_TC_2_READING,
        ST_SHOW_TC_2_READING,
        ST_MENU_POOR,
        ST_ADJ_POOR,
		ST_MENU_FREQUENCY,
		ST_ADJ_FREQUENCY,
        ST_MENU_FW_VERSION,
        ST_SHOW_FW_VERSION,
} STATES;
STATES state = ST_MAIN;

typedef enum {
        EVT_LEFT,
        EVT_RIGHT,
        EVT_BUTTON,
        EVT_LONG_PRESS,
        EVT_NONE
} EVENTS;
EVENTS event = EVT_NONE;

typedef enum {
		TYPE_WMRP,
		TYPE_WMRT,
		TYPE_NC
} TIP_TYPES;
TIP_TYPES tiptype = TYPE_WMRP;		

typedef enum {
		REED_CLOSED,
		REED_OPEN
} REED_STATES;
REED_STATES reed_status = REED_OPEN;
REED_STATES prev_reed_status = REED_OPEN;	

typedef enum {
		DISP_C,
		DISP_F,
		DISP_NUM,
		DISP_REF
} DISP_MODES;

// D-type thermocouple 0..500 C difference, op amp gain 241, reference 2.048V
unsigned int16 ROM tc_lookup[51] = {0,756,1535,2352,3200,4072,4967,5900,6848,7828,
									8830,9856,10905,11984,13080,14190,15331,16481,17660,18856,
									20067,21293,22542,23807,25087,26375,27686,29005,30347,31696,
									33054,34426,35815,37210,38622,40041,41467,42902,44352,45809,
									47267,48740,50221,51709,53197,54701,56205,57717,59236,60763,
									62290};

// KTY82/110 temperature sensor connected between GND and 1k pullup to reference. -50...+150 in 10 degree steps
// ADC REF = 2.048V
unsigned int16 ROM kty_lookup[21] = {22278,23713,25181,26619,28023,29428,30787,32116,33411,34652,
									35868,37030,38138,39216,40242,41227,42172,43054,43857,44558,45126};

unsigned int8 DISPLAY[5];			// Display memory, last byte is clock colons / degree sign
unsigned int8 CURRENT_SEG = 0;

unsigned int8 oldenc = 0;

signed int32 temp1=0;
signed int32 temp2=0;
unsigned int32 ylempi=0;
unsigned int32 alempi=0;
unsigned int16 adc_sum = 0;
signed int16 right_buf=0;
signed int16 left_buf=0;
signed int16 kty_buf=25;
signed int16 new_temp=0;

// Saveable parameters. These will be downloaded from EEPROM and set to default
// if any of them are out of range, therefore no need to give initial values.
signed int16 setpoint;
signed int16 setback;
signed int16 setback_delay;
signed int16 poweroff_delay;
signed int16 temperature_offset;
signed int16 reference;
signed int16 stepsize;
DISP_MODES temperature_unit;
unsigned int1 poorMode;
unsigned int8 mainsFrequency;

signed int16 normal_setpoint=0;
int8 i=0;
int16 j=0;
unsigned int16 buttimer=0;
unsigned int8 releasetimer=0;
unsigned int8 heaterStatus=0xff;
unsigned int8 heater2Status=0xff;
unsigned int32 milliseconds=0;
unsigned int8 idleminutes=0;
unsigned int16 setpointdelay=1000;
signed int16 filteredtemp=0;
unsigned int8 savesettings=0;
unsigned int8 mainsCycles=0;

void saveParms();															// saves current parameters to EEPROM
void menuMachine();															// updates menu state based on encoder event
void updateDisplay();														// updates display memory based on current menu state
void show_7seg_num(DISP_MODES mode, signed int16 num);						// converts integers to 7 segment numbers
void readAdc(int16* adc_r, unsigned int8 channel);							// reads specified ADC input 16 times to get a virtual 16 bit reading
void tc_lookup_32bit(unsigned int32* adc_sum, signed int16* temperature);	// converts thermocouple voltage to temperature
void kty_lookup_16bit(unsigned int16* adc_sum, signed int16* temperature);	// converts cold junction compensation sensor voltage to temperature
void recognizeTypeInStand()	;												// recognizes tip type even when in stand

#int_timer2			// timer2 is set to 832 us
void timer2isr(void) {
	// output_a(_8); // for testing interrupt length, currently 20-25 us when menuMachine call moved to interrupt
	mainsCycles++;
	if (mainsFrequency==50) {
		if (mainsCycles>=24)	// 50 Hz
			mainsCycles=0;
	}
	else {
		if (mainsCycles>=20)	// 60 Hz
			mainsCycles=0;
	}
	
	output_a(_SPACE); // clear anodes
	switch(CURRENT_SEG) {
		case 0:	output_c(0b11111110);
				output_a(((DISPLAY[4]&0x01)<<7)|((DISPLAY[3]&0x01)<<6)|((DISPLAY[2]&0x01)<<5)|((DISPLAY[1]&0x01)<<4)|((DISPLAY[0]&0x01)<<3));
				break;
		case 1:	output_c(0b11111101);
				output_a(((DISPLAY[4]&0x02)<<6)|((DISPLAY[3]&0x02)<<5)|((DISPLAY[2]&0x02)<<4)|((DISPLAY[1]&0x02)<<3)|((DISPLAY[0]&0x02)<<2));
				break;
		case 2:	output_c(0b11111011);
				output_a(((DISPLAY[4]&0x04)<<5)|((DISPLAY[3]&0x04)<<4)|((DISPLAY[2]&0x04)<<3)|((DISPLAY[1]&0x04)<<2)|((DISPLAY[0]&0x04)<<1));
				break;
		case 3:	output_c(0b11110111);
				output_a(((DISPLAY[3]&0x08)<<3)|((DISPLAY[2]&0x08)<<2)|((DISPLAY[1]&0x08)<<1)|(DISPLAY[0]&0x08));
				break;
		case 4:	output_c(0b11101111);
				output_a(((DISPLAY[3]&0x10)<<2)|((DISPLAY[2]&0x10)<<1)|((DISPLAY[1]&0x10))|((DISPLAY[0]&0x10)>>1));
				break;
		case 5:	output_c(0b11011111);
				output_a(((DISPLAY[3]&0x20)<<1)|((DISPLAY[2]&0x20))|((DISPLAY[1]&0x20)>>1)|((DISPLAY[0]&0x20)>>2));
				break;
		case 6:	output_c(0b10111111);
				output_a(((DISPLAY[3]&0x40))|((DISPLAY[2]&0x40)>>1)|((DISPLAY[1]&0x40)>>2)|((DISPLAY[0]&0x40)>>3));
				break;
		case 7:	output_c(0b01111111);
				output_a((((DISPLAY[3]&0x80) _ADD_HEATER_STATUS)>>1)|(((DISPLAY[2]&0x80) _ADD_HEATER_2_STATUS)>>2)|((DISPLAY[1]&0x80)>>3)|((DISPLAY[0]&0x80)>>4));
				break;
		default:output_c(0b11111111);
				
	}
	
	CURRENT_SEG++;
	if (CURRENT_SEG==8 && state!=ST_STANDBY)
		CURRENT_SEG=0;
	if (CURRENT_SEG==16)
		CURRENT_SEG=0;


	// ENCODER READOUT
	if (!input(PIN_B7))
	{
		if (!input(PIN_B6) && (oldenc & 0x40))
			event = EVT_RIGHT;
		if (input(PIN_B6) && !(oldenc & 0x40))
			event = EVT_LEFT;
	}	
	oldenc = input_b();

	if (!input(PIN_E3)||(buttimer>0&&buttimer<72))	// 60 ms debounce on press
		buttimer++;
	else
		buttimer=0;
	
	if (input(PIN_E3)&&(releasetimer>0))			// 30 ms debounce on release
		releasetimer--;
	else if (!input(PIN_E3)&&buttimer>1)
		releasetimer=36;
			
	if (buttimer==1 && releasetimer==0)
		event = EVT_BUTTON;
	else if (buttimer>1023)
		event = EVT_LONG_PRESS;
	// End reading encoder
	
	menuMachine();	// menuMachine() call moved here to prevent missing events if interrupt happens during menuMachine call
		
	if (reed_status == REED_CLOSED && (state == ST_MAIN || state == ST_SETBACK))
		milliseconds++;
	else {
		milliseconds=0;
		idleminutes=0;
	}	
	
	if (milliseconds==72191)	// 72115*832 us = 60 s. But 72191 is faster to compare
	{
		milliseconds=0;
		if (idleminutes < 255)	// don't let idleminutes overflow
			idleminutes++;
//		if (idleminutes == setback_delay && setback_delay != 31)
//			state = ST_SETBACK;
//		if (idleminutes == poweroff_delay && poweroff_delay != 130)
//			state = ST_STANDBY;		
	}

	if (milliseconds == 1 && idleminutes == setback_delay && setback_delay != 31 && state == ST_MAIN)
		state = ST_SETBACK;
	if (milliseconds == 1 && idleminutes == poweroff_delay && poweroff_delay != 130 && (state == ST_MAIN || state == ST_SETBACK))
		state = ST_STANDBY;		


	if (setpointdelay>0)
		setpointdelay--;

	if (setpointdelay==1)
		savesettings=1;
	// output_a(_SPACE); // to test interrupt length
}

void show_7seg_num(DISP_MODES mode, signed int16 num)
{
	signed int8 j;                       // Digit index
	signed int16 temp16;

	switch(mode) {
		case DISP_C:
			DISPLAY[3] = _CELSIUS;
			DISPLAY[4] = _DEGREE;
			j = 2;
			break;
		case DISP_F:
			DISPLAY[3] = _F;
			DISPLAY[4] = _DEGREE;
			num = num*9/5;
			if (state != ST_ADJ_OFFSET && state != ST_ADJ_STEP_SIZE)
				num = num+32;
			if (num > 999)
				num = 999;
			j = 2;
			break;
		case DISP_NUM:
		case DISP_REF:
			DISPLAY[4] = _SPACE;
			j = 3;
	}		

	temp16 = labs(num); 

	DISPLAY[j] = hex_table[temp16 %10];
	while (j > 0)
	{
		j--;
		if (j==0 && mode == DISP_REF)
			DISPLAY[j] = hex_table[(temp16/10) %10] _ADDPOINT;
		else if ((temp16/10) == 0 && num>=0)
			DISPLAY[j] = _SPACE;
		else if ((temp16/10) == 0 && temp16 != 0 && num < 0)
			DISPLAY[j] = _MINUS;
		else if (temp16 == 0 && num < 0)
			DISPLAY[j] = _SPACE;
		else
			DISPLAY[j] = hex_table[(temp16/10) %10];
		temp16 /= 10;
	}
}

void readAdc(int16* adc_r, unsigned int8 channel) {
	set_adc_channel(channel);
	delay_us(5);
	i=0;
	adc_sum = 0;
	while (i<16) { // sample 16 times to get a virtual 16 bit reading..
		adc_sum += read_adc();
		i++;
	}
	*adc_r = adc_sum;
}

void tc_lookup_32bit(unsigned int32* adc_sum, signed int16* temperature)
{
	unsigned int32 sum32;
	// sum32 = (unsigned int32)*adc_sum * (unsigned int32)reference / 2048;	// compensate for reference inaccuracy
	sum32 = *adc_sum * (unsigned int32)reference / 2048;	// compensate for reference inaccuracy
	for (j=0;j<50;j++)
	{
		if (sum32<tc_lookup[j])
			break;
	}
	
	ylempi = tc_lookup[j];
	alempi = tc_lookup[j-1];
	temp1=10*(alempi-sum32);
	temp2=(alempi-ylempi);
	new_temp = temp1/temp2+ 10*(j-1)+kty_buf;		// -1 because tc starts from zero.
	new_temp = new_temp - temperature_offset;		// subtract offset setting
	
	*temperature = new_temp;
}

void kty_lookup_16bit(unsigned int16* adc_sum, signed int16* temperature)
{
	for (j=0;j<21;j++)
	{
		if (*adc_sum<kty_lookup[j])
			break;
	}
	
	ylempi = kty_lookup[j];
	alempi = kty_lookup[j-1];
	temp1=10*(alempi-*adc_sum);
	temp2=(alempi-ylempi);
	new_temp = temp1/temp2+ 10*(j-6); // -6 because kty_lookup begins from -50
	*temperature = new_temp;
}

void menuMachine()		// Takes now 3..4us when display updates moved to separate function
{
	switch(state)
	{
		case ST_MAIN:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_STANDBY;
					break;
				case EVT_LONG_PRESS:
					state = ST_MENU_MAIN;
					break;
				case EVT_LEFT:
					setpoint-=stepsize;
					if (setpoint < MIN_SETPOINT)
						setpoint = MIN_SETPOINT;
					normal_setpoint = setpoint;
					setpointdelay=1202;
					break;
				case EVT_RIGHT:
					setpoint+=stepsize;
					if (setpoint > MAX_SETPOINT)
						setpoint = MAX_SETPOINT;
					normal_setpoint = setpoint;
					setpointdelay=1202;
					break;
			}
			break;
		case ST_STANDBY:
			setpoint=0;
			if (reed_status == REED_OPEN && prev_reed_status == REED_CLOSED)
				event = EVT_BUTTON;
			prev_reed_status = reed_status;
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MAIN;
					setpoint = normal_setpoint;
					milliseconds = 0;
					idleminutes = 0;
					break;
				case EVT_LONG_PRESS:
					state = ST_MENU_MAIN;
					setpoint = normal_setpoint;
					break;
			}
			break;	
		case ST_SETBACK:
			if (setback < setpoint)
				setpoint = setback;
			if (reed_status == REED_OPEN)
				event = EVT_BUTTON;
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MAIN;
					setpoint = normal_setpoint;
					milliseconds = 0;
					idleminutes = 0;
					if (setback_delay == 0)			// workaround to briefly activate ST_MAIN if setback is zero
						milliseconds = 4294961286;	// this number will overflow in 5 seconds
					break;
				case EVT_LONG_PRESS:
					state = ST_MENU_MAIN;
					setpoint = normal_setpoint;
					break;
			}
			break;	
		case ST_MENU_MAIN:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MAIN;
					break;
				case EVT_LEFT:
					state = ST_MENU_DIAGNOSTICS;
					break;
				case EVT_RIGHT:
					state = ST_MENU_SETBACK;
					break;
			}
			break;
		case ST_MENU_SETBACK:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_SETBACK;
					break;
				case EVT_LEFT:
					state = ST_MENU_MAIN;
					break;
				case EVT_RIGHT:
					state = ST_MENU_DELAY;
					break;
			}
			break;
		case ST_ADJ_SETBACK:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_SETBACK;
					break;
				case EVT_LEFT:
					if(setback > MIN_SETPOINT)
						setback-=10;
					break;
				case EVT_RIGHT:
					if(setback < MAX_SETPOINT)
						setback+=10;
					break;
			}
			break;
		case ST_MENU_DELAY:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_DELAY;
					break;
				case EVT_LEFT:
					state = ST_MENU_SETBACK;
					break;
				case EVT_RIGHT:
					state = ST_MENU_POWEROFF;
					break;
			}
			break;
		case ST_ADJ_DELAY:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_DELAY;
					break;
				case EVT_LEFT:
					if(setback_delay>0)
						setback_delay--;
					break;
				case EVT_RIGHT:
					if(setback_delay < MAX_SETBACK_DELAY+1)
						setback_delay++;
				break;
			}
			break;
		case ST_MENU_POWEROFF:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_POWEROFF;
					break;
				case EVT_LEFT:
					state = ST_MENU_DELAY;
					break;
				case EVT_RIGHT:
					state = ST_MENU_OFFSET;
					break;
			}
			break;
		case ST_ADJ_POWEROFF:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_POWEROFF;
					break;
				case EVT_LEFT:
					poweroff_delay-=10;
					if (poweroff_delay<0)
						poweroff_delay=0;
					break;
				case EVT_RIGHT:
					poweroff_delay+=10;
					if (poweroff_delay > MAX_POWEROFF_DELAY)
						poweroff_delay = MAX_POWEROFF_DELAY+10;
					break;
			}
			break;
		case ST_MENU_OFFSET:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_OFFSET;
					break;
				case EVT_LEFT:
					state = ST_MENU_POWEROFF;
					break;
				case EVT_RIGHT:
					state = ST_MENU_UNIT;
					break;
			}
			break;
		case ST_ADJ_OFFSET:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_OFFSET;
					break;
				case EVT_LEFT:
					if(temperature_offset > -(MAX_TEMPERATURE_OFFSET))
						temperature_offset--;
					break;
				case EVT_RIGHT:
					if(temperature_offset < MAX_TEMPERATURE_OFFSET)
						temperature_offset++;
					break;
			}
			break;
		case ST_MENU_UNIT:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_UNIT;
					break;
				case EVT_LEFT:
					state = ST_MENU_OFFSET;
					break;
				case EVT_RIGHT:
					state = ST_MENU_STEP_SIZE;
					break;
			}
			break;
		case ST_ADJ_UNIT:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_UNIT;
					break;
				case EVT_LEFT:
				case EVT_RIGHT:
					if(temperature_unit == DISP_C)
						temperature_unit = DISP_F;
					else
						temperature_unit = DISP_C; 
					break;
			}
			break;
		case ST_MENU_STEP_SIZE:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_STEP_SIZE;
					break;
				case EVT_LEFT:
					state = ST_MENU_UNIT;
					break;
				case EVT_RIGHT:
					state = ST_MENU_DIAGNOSTICS;
					break;
			}
			break;
		case ST_ADJ_STEP_SIZE:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_STEP_SIZE;
					break;
				case EVT_LEFT:
				case EVT_RIGHT:
					if(stepsize == DEFAULT_STEPSIZE)
						stepsize = 1;
					else
						stepsize = DEFAULT_STEPSIZE; 
					break;
			}
			break;
		case ST_MENU_DIAGNOSTICS:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_BACK_FROM_DIAGNOSTICS;
					break;
				case EVT_LEFT:
					state = ST_MENU_STEP_SIZE;
					break;
				case EVT_RIGHT:
					state = ST_MENU_MAIN;
					break;
			}
			break;
		case ST_MENU_BACK_FROM_DIAGNOSTICS:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_DIAGNOSTICS;
					break;
				case EVT_LEFT:
					state = ST_MENU_FW_VERSION;
					break;
				case EVT_RIGHT:
					state = ST_MENU_COLD_COMPENSATION;
					break;
			}
			break;
		case ST_MENU_COLD_COMPENSATION:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_SHOW_COLD_COMPENSATION;
					break;
				case EVT_LEFT:
					state = ST_MENU_BACK_FROM_DIAGNOSTICS;
					break;
				case EVT_RIGHT:
					state = ST_MENU_REFERENCE;
					break;
			}
			break;
		case ST_SHOW_COLD_COMPENSATION:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_COLD_COMPENSATION;
					break;
			}
			break;
		case ST_MENU_REFERENCE:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_REFERENCE;
					break;
				case EVT_LEFT:
					state = ST_MENU_COLD_COMPENSATION;
					break;
				case EVT_RIGHT:
					state = ST_MENU_TIP_TYPE;
					break;
			}
			break;
		case ST_ADJ_REFERENCE:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_REFERENCE;
					break;
				case EVT_LEFT:
					if(reference > 2048 - MAX_REFERENCE_TOLERANCE)
						reference--;
					break;
				case EVT_RIGHT:
					if(reference < 2048 + MAX_REFERENCE_TOLERANCE)
						reference++;
					break;
			}
			break;
		case ST_MENU_TIP_TYPE:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_SHOW_TIP_TYPE;
					break;
				case EVT_LEFT:
					state = ST_MENU_REFERENCE;
					break;
				case EVT_RIGHT:
					state = ST_MENU_REED_STATE;
					break;
			}
			break;
		case ST_SHOW_TIP_TYPE:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_TIP_TYPE;
					break;
			}
			break;
		case ST_MENU_REED_STATE:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_SHOW_REED_STATE;
					break;
				case EVT_LEFT:
					state = ST_MENU_TIP_TYPE;
					break;
				case EVT_RIGHT:
					state = ST_MENU_TC_1_READING;
					break;
			}
			break;
		case ST_SHOW_REED_STATE:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_REED_STATE;
					break;
			}
			break;
		case ST_MENU_TC_1_READING:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_SHOW_TC_1_READING;
					break;
				case EVT_LEFT:
					state = ST_MENU_REED_STATE;
					break;
				case EVT_RIGHT:
					state = ST_MENU_TC_2_READING;
					break;
			}
			break;
		case ST_SHOW_TC_1_READING:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_TC_1_READING;
					break;
			}
			break;
		case ST_MENU_TC_2_READING:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_SHOW_TC_2_READING;
					break;
				case EVT_LEFT:
					state = ST_MENU_TC_1_READING;
					break;
				case EVT_RIGHT:
					state = ST_MENU_POOR;
					break;
			}
			break;
		case ST_SHOW_TC_2_READING:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_TC_2_READING;
					break;
			}
			break;
		case ST_MENU_POOR:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_POOR;
					break;
				case EVT_LEFT:
					state = ST_MENU_TC_2_READING;
					break;
				case EVT_RIGHT:
					state = ST_MENU_FREQUENCY;
					break;
			}
			break;
		case ST_ADJ_POOR:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_POOR;
					break;
				case EVT_LEFT:
				case EVT_RIGHT:
					poorMode = !poorMode;
					break;
			}
			break;
		case ST_MENU_FREQUENCY:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_ADJ_FREQUENCY;
					break;
				case EVT_LEFT:
					state = ST_MENU_POOR;
					break;
				case EVT_RIGHT:
					state = ST_MENU_FW_VERSION;
					break;
			}
			break;
		case ST_ADJ_FREQUENCY:
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_FREQUENCY;
					break;
				case EVT_LEFT:
				case EVT_RIGHT:
					if (mainsFrequency==50)
						mainsFrequency=60;
					else
						mainsFrequency=50;
					break;
			}
			break;
		case ST_MENU_FW_VERSION:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_SHOW_FW_VERSION;
					break;
				case EVT_LEFT:
					state = ST_MENU_FREQUENCY;
					break;
				case EVT_RIGHT:
					state = ST_MENU_BACK_FROM_DIAGNOSTICS;
					break;
			}
			break;
		case ST_SHOW_FW_VERSION:
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_FW_VERSION;
					break;
			}
			break;
	} // end menu case
	event = EVT_NONE;
}		

void updateDisplay()	// takes 520 us when showing iron temp, 400us when KTY temp, 680us when reference, 500us when number
{
	switch(state)
	{
		case ST_MAIN:
			if (tiptype == TYPE_NC)
				memcpy (DISPLAY, nc, sizeof (DISPLAY));
			else if (setpointdelay)
				show_7seg_num(temperature_unit, setpoint);
			else
				show_7seg_num(temperature_unit,filteredtemp);
			break;
		case ST_STANDBY:
			memcpy (DISPLAY, stby, sizeof (DISPLAY));
			break;	
		case ST_SETBACK:
			show_7seg_num(temperature_unit, filteredtemp);
			break;	
		case ST_MENU_MAIN:
			memcpy (DISPLAY, bacc, sizeof (DISPLAY));
			break;
		case ST_MENU_SETBACK:
			memcpy (DISPLAY, setb, sizeof (DISPLAY));
			break;
		case ST_ADJ_SETBACK:
			show_7seg_num(temperature_unit, setback);
			break;
		case ST_MENU_DELAY:
			memcpy (DISPLAY, dela, sizeof (DISPLAY));
			break;
		case ST_ADJ_DELAY:
			if (setback_delay==31)
				memcpy (DISPLAY, off, sizeof (DISPLAY));
			else
				show_7seg_num(DISP_NUM, setback_delay);
			break;
		case ST_MENU_POWEROFF:
			memcpy (DISPLAY, poff, sizeof (DISPLAY));
			break;
		case ST_ADJ_POWEROFF:
			if (poweroff_delay > 120)
				memcpy (DISPLAY, off, sizeof (DISPLAY));
			else
				show_7seg_num(DISP_NUM, poweroff_delay);
			break;
		case ST_MENU_OFFSET:
			memcpy (DISPLAY, ofse, sizeof (DISPLAY));
			break;
		case ST_ADJ_OFFSET:
			show_7seg_num(temperature_unit, temperature_offset);
			break;
		case ST_MENU_UNIT:
			memcpy (DISPLAY, unit, sizeof (DISPLAY));
			break;
		case ST_ADJ_UNIT:
			show_7seg_num(temperature_unit, 0);
			break;
		case ST_MENU_STEP_SIZE:
			memcpy (DISPLAY, step, sizeof (DISPLAY));
			break;
		case ST_ADJ_STEP_SIZE:
			show_7seg_num(temperature_unit, stepsize);
			break;
		case ST_MENU_DIAGNOSTICS:
			memcpy (DISPLAY, diag, sizeof (DISPLAY));
			break;
		case ST_MENU_BACK_FROM_DIAGNOSTICS:
			memcpy (DISPLAY, bacc, sizeof (DISPLAY));
			break;
		case ST_MENU_COLD_COMPENSATION:
			memcpy (DISPLAY, cold, sizeof (DISPLAY));
			break;
		case ST_SHOW_COLD_COMPENSATION:
			show_7seg_num(temperature_unit, kty_buf);
			break;
		case ST_MENU_REFERENCE:
			memcpy (DISPLAY, ref, sizeof (DISPLAY));
			break;
		case ST_ADJ_REFERENCE:
			show_7seg_num(DISP_REF, reference);
			break;
		case ST_MENU_TIP_TYPE:
			memcpy (DISPLAY, type, sizeof (DISPLAY));
			break;
		case ST_SHOW_TIP_TYPE:
			switch(tiptype)
			{
				case TYPE_WMRP:
					memcpy (DISPLAY, wmrp, sizeof(DISPLAY));
					break;
				case TYPE_WMRT:
					memcpy (DISPLAY, wmrt, sizeof(DISPLAY));
					break;
				case TYPE_NC:
					memcpy (DISPLAY, nc, sizeof(DISPLAY));
					break;
			}
			break;		
		case ST_MENU_REED_STATE:
			memcpy (DISPLAY, reed, sizeof (DISPLAY));
			break;
		case ST_SHOW_REED_STATE:
			switch(reed_status)
			{
				case REED_OPEN:
					memcpy (DISPLAY, open, sizeof(DISPLAY));
					break;
				case REED_CLOSED:
					memcpy (DISPLAY, clos, sizeof(DISPLAY));
					break;
			}			
			break;
		case ST_MENU_TC_1_READING:
			memcpy (DISPLAY, tc_1, sizeof (DISPLAY));
			break;
		case ST_SHOW_TC_1_READING:
			show_7seg_num(temperature_unit, right_buf);
			break;
		case ST_MENU_TC_2_READING:
			memcpy (DISPLAY, tc_2, sizeof (DISPLAY));
			break;
		case ST_SHOW_TC_2_READING:
			show_7seg_num(temperature_unit, left_buf);
			break;
		case ST_MENU_POOR:
			memcpy (DISPLAY, poor, sizeof (DISPLAY));
			break;
		case ST_ADJ_POOR:
			if (poorMode)
				memcpy (DISPLAY, on, sizeof (DISPLAY));
			else
				memcpy (DISPLAY, off, sizeof (DISPLAY));
			break;
		case ST_MENU_FREQUENCY:
			memcpy (DISPLAY, freq, sizeof (DISPLAY));
			break;
		case ST_ADJ_FREQUENCY:
			show_7seg_num(DISP_NUM, mainsFrequency);
			break;
		case ST_MENU_FW_VERSION:
			memcpy (DISPLAY, vers, sizeof (DISPLAY));
			break;
		case ST_SHOW_FW_VERSION:
			show_7seg_num(DISP_REF, (signed int16)FW_VERSION);
			break;
	} // end display case
}		

void recognizeTypeInStand()
{
	// Tip type recognision by pulsing current to heater2 = wmrt left heater. If drives TC2 to rail, must be WMRT
	// If the iron is on stand, cannot recognize based on resistor in parallel with reed because reed is closed.
	// Since FW 0.8 this is only used when necessary to recognize if WMRT is connected. Older FW used this method
	// continuously which caused continuous buzz noise from WMRT tip. The buzz comes from the heating elements of
	// WMRT when power is switched on and off rapidly. Original Weller stations don't make the noise because they
	// drive the elements with AC

	set_adc_channel(1);
	output_high(HEATER_2);
	delay_us(20);
	unsigned int16 temp_reading = read_adc();
	output_low(HEATER_2);
	if (temp_reading > 4000)
		tiptype = TYPE_WMRT;
}

void saveParms()
{
	output_low(HEATER_1);	// Turn heaters off before saving parameters - interrupts are disabled
	output_low(HEATER_2);	// during EEPROM write and we don't want something bad to happen if write jams
	output_a(_SPACE); 		// clear anodes to eliminate brigt flash of currently active segments
	write_int16_eeprom(0, setpoint);
	write_int16_eeprom(2, setback);
	write_int16_eeprom(4, setback_delay);
	write_int16_eeprom(6, poweroff_delay);
	write_int16_eeprom(8, temperature_offset);
	write_int16_eeprom(10, reference);
	write_int16_eeprom(12, stepsize);
	write_eeprom(14, temperature_unit);
	write_eeprom(15, poorMode);
	write_eeprom(16, mainsFrequency);
}	


void main()
{
	unsigned int16 temp_reading=0;
	unsigned int32 right_sum=0;
	unsigned int32 left_sum=0;
	unsigned int32 right_sum_2=0;
	unsigned int32 left_sum_2=0;
	unsigned int16 kty_sum=0;
	signed int16 kty_filtering_sum=0;
	signed int16 signed_temp_reading=0;
	unsigned int16 reed_sum=0;
	unsigned int8 kty_cycles=0;
	unsigned int8 kty_filt_cycles=0;
	unsigned int1 calculateTemp=0;
	unsigned int1 checkTip=0;
	unsigned int8 paramsOutOfRange=0;
	setup_timer_2(T2_DIV_BY_16,208,2); // set timer2 to 832 µs (prescaler 16 and postscaler 2, period 208)
	enable_interrupts(INT_TIMER2);
	enable_interrupts(GLOBAL);
	set_tris_a(0b00000111);
	set_tris_b(0b11001111);
	set_tris_c(0x00);
	set_tris_e(0x11111000);
	port_a_pullups(0b00000000);
	port_b_pullups(0b11000000);
	port_e_pullups(0b00001000);
	output_low(HEATER_1);	// Heater 1 (right) off
	output_low(HEATER_2);	// Heater 2 (left) off

	DISPLAY[4]=_SPACE;
		
	setup_adc_ports( sAN0|sAN1|sAN2|sAN9|sAN12,VSS_FVR);	// AN9 = reed pullup
	setup_vref(VREF_ON | VREF_ADC_2v048 | VREF_COMP_DAC_2v048);
	// setup_opamp2(OPAMP_ENABLED | OPAMP_HIGH_GBWP_MODE | OPAMP_NI_TO_FVR);		// To enable KTY pullup
	setup_opamp2(OPAMP_DISABLED);		// To disable KTY pullup
	setup_dac(DAC_OFF);

	setup_adc(ADC_CLOCK_DIV_32);	// Gives 1µs conversion clock cycle time (fastest recommended)

	setpoint = read_int16_eeprom(0);
	setback = read_int16_eeprom(2);			// must use even addresses because one int16 takes two bytes
	setback_delay = read_int16_eeprom(4);
	poweroff_delay = read_int16_eeprom(6);
	temperature_offset = read_int16_eeprom(8);
	reference = read_int16_eeprom(10);
	stepsize = read_int16_eeprom(12);
	temperature_unit = read_eeprom(14);
	poorMode = read_eeprom(15);
	mainsFrequency = read_eeprom(16);

	if (setpoint < MIN_SETPOINT || setpoint > MAX_SETPOINT)		// Ensure safe temperature setpoints in case of corrupt EEPROM data
		paramsOutOfRange=TRUE;
	if (setback < MIN_SETPOINT || setback > MAX_SETPOINT)
		paramsOutOfRange=TRUE;
	if (setback_delay < 0 || setback_delay > MAX_SETBACK_DELAY+1)
		paramsOutOfRange=TRUE;
	if (poweroff_delay < 0 || poweroff_delay > MAX_POWEROFF_DELAY+10)
		paramsOutOfRange=TRUE;
	if (temperature_offset < -(MAX_TEMPERATURE_OFFSET) || temperature_offset > MAX_TEMPERATURE_OFFSET)
		paramsOutOfRange=TRUE;
	if (reference < 2048 - MAX_REFERENCE_TOLERANCE || reference > 2048 + MAX_REFERENCE_TOLERANCE)
		paramsOutOfRange=TRUE;
	if (stepsize != 1 && stepsize != 5)
		paramsOutOfRange=TRUE;
	if (temperature_unit != DISP_C && temperature_unit != DISP_F)
		paramsOutOfRange=TRUE;
	if (mainsFrequency != 50 && mainsFrequency != 60)
		paramsOutOfRange=TRUE;
	
	if (paramsOutOfRange) {
		setpoint = DEFAULT_SETPOINT;
		setback = DEFAULT_SETBACK;
		setback_delay = DEFAULT_SETBACK_DELAY;
		poweroff_delay = DEFAULT_POWEROFF_DELAY;
		temperature_offset = DEFAULT_TEMPERATURE_OFFSET;
		reference = DEFAULT_REFERENCE;
		stepsize = DEFAULT_STEPSIZE;
		temperature_unit = DEFAULT_TEMPERATURE_UNIT;
		poorMode = DEFAULT_POORMODE;
		mainsFrequency = DEFAULT_MAINSFREQUENCY;
	}	

	normal_setpoint = setpoint;

	recognizeTypeInStand();
	recognizeTypeInStand();
	delay_ms(10);	// let auto-zero op-amps stabilize before tip type recognition
	tiptype = TYPE_WMRP;
	recognizeTypeInStand();

	/* Infinite loop */
	while(1)
	{
		output_low(HEATER_1);	// Heater 1 (right) off
		output_low(HEATER_2);	// Heater 2 (left) off
		delay_us(10);

		// READ REED STATUS - Reed pullup is on for 35 us (measured 21.1.2017 / FW 0.6)
		setup_adc_ports( sAN0|sAN1|sAN2|sAN12,VSS_VDD);	// AN9 = reed pullup
		set_tris_b(0b11000111);
		output_high(REED_PULLUP);
		set_adc_channel(12);
		delay_us(5);
		reed_sum = read_adc();		
		output_low(REED_PULLUP);
		set_tris_b(0b11001111);
		setup_adc_ports( sAN0|sAN1|sAN2|sAN9|sAN12,VSS_FVR);	// AN9 = reed pullup

		// KTY READOUT - KTY pullup is on for 34 us (measured 21.1.2017 / FW 0.6)
		setup_opamp2(OPAMP_ENABLED | OPAMP_HIGH_GBWP_MODE | OPAMP_NI_TO_FVR);		// To enable KTY pullup
		set_adc_channel(2);
		delay_us(5);
		kty_sum += read_adc();
		if (state != ST_ADJ_REFERENCE)			// Keep KTY pullup on (reference on) when adjusting reference compensation
			setup_opamp2(OPAMP_DISABLED);		// To disable KTY pullup

		// delay_us(300);		// takes about 200-300 µs for opamp output to stabilize after pullups turned off - delay not
								// needed anymore because wait for mainsCycles below gives more than enough time.
		
		if (mainsFrequency==50)										// added 2*832us waiting to reduce heating duty cycle in FW 0.7
			while (mainsCycles != 3 && mainsCycles != 15) {	}		// wait here for measurement cycle start (50 Hz)
		else
			while (mainsCycles != 3 && mainsCycles != 13) {	}		// wait here for measurement cycle start (60 Hz)

		if (mainsCycles == 3) {
			readAdc(&left_sum, 1);	// this function should take about 0.3 ms (ADC_CLOCK_DIV32 @ 32 MHz)
			readAdc(&right_sum, 0);
		}
		else {
			readAdc(&left_sum_2, 1);
			readAdc(&right_sum_2, 0);
			calculateTemp = 1;
		}

		if (checkTip) {				// seems that new iron is connected with reed closed - lets check the type!
			tiptype = TYPE_WMRP;	// defalut to WMRP
			recognizeTypeInStand();	// check if it is WMRT
			checkTip=0;
		}	
		
		if (tiptype == TYPE_NC) {
			right_buf=555;
		}

		if (right_buf<setpoint) {
			heaterStatus = 0x7f;
			output_high(HEATER_1);
		}
		else {
			heaterStatus = 0xff;
		}	

		if (left_buf<setpoint && tiptype == TYPE_WMRT) {
			heater2Status = 0x7f;
			output_high(HEATER_2);
		}
		else {
			heater2Status = 0xff;
		}	

		kty_cycles++;
		if (kty_cycles==16) {
			kty_lookup_16bit(&kty_sum, &signed_temp_reading);
			kty_sum=0;
			if (signed_temp_reading > 130)	// No KTY sensor or bad contact
				signed_temp_reading=30;		// use some default value
			kty_filtering_sum += signed_temp_reading;
			kty_filt_cycles++;
			if (kty_filt_cycles==16) {
				kty_filt_cycles=0;
				kty_buf = kty_filtering_sum / 16;
				kty_filtering_sum=0;
			}
			kty_cycles=0;
		}

		if (calculateTemp==1) {				// takes about 1.5ms
			left_sum = (left_sum + left_sum_2)/2;
			right_sum = (right_sum + right_sum_2)/2;
			tc_lookup_32bit(&left_sum, &left_buf);
			tc_lookup_32bit(&right_sum, &right_buf);
			calculateTemp=0;
		}

		if (savesettings && state == ST_MAIN)	// do not save setpoint if already gone to stby
		{
			write_int16_eeprom(0, setpoint);
			savesettings=0;
		}	
		
		// some filtering to displayed temperature to reduce flickering
		temp_reading = filteredtemp*30;
		temp_reading = temp_reading + right_buf*10;
		filteredtemp = temp_reading / 40;

		if ((filteredtemp-setpoint < 3) && (setpoint-filteredtemp < 3))
			filteredtemp = setpoint;

		if (reed_sum<312) {
			if (poorMode) {
				tiptype = TYPE_WMRP;
				reed_status = REED_OPEN;
			}
			else if (tiptype == TYPE_NC) {
				checkTip=1;
				reed_status = REED_CLOSED;
			}	
			else
				reed_status = REED_CLOSED;
		}
		else if (reed_sum>1798 && reed_sum<2298) {
			tiptype = TYPE_WMRP;
			reed_status = REED_OPEN;
		}
		else if (reed_sum>2481 && reed_sum<2981) {
			tiptype = TYPE_WMRT;
			reed_status = REED_OPEN;
		}
		else if (reed_sum>3750) {
			tiptype = TYPE_NC;
			reed_status = REED_OPEN;
		}

		updateDisplay();		// updateDisplay call takes about 520 µs when in 'normal' mode (displaying temp)

		if (mainsFrequency==50)
			while (mainsCycles != 0 && mainsCycles != 12) {	}		// wait here for heating cycle to stop (50 Hz)
		else
			while (mainsCycles != 0 && mainsCycles != 10) {	}		// wait here for heating cycle to stop (60 Hz)
	}	// end while
}