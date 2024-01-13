////////////////////////////////////////////////////
// weller_driver_v05.c
////////////////////////////////////////////////////
// Copyright 2015 Jaakko Kairus
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
// Compiled with CCS compiler version 5.048
// Total compiled length 4503 bytes
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

#define REED_PULLUP PIN_B3
#define HEATER_1 PIN_B4
#define HEATER_2 PIN_B5


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
DISP_MODES temperature_unit = DISP_C;

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
signed int16 kty_buf=0;
signed int16 new_temp=0;
signed int16 setback_delay=5;
signed int16 setback=250;
signed int16 setpoint=380;
signed int16 normal_setpoint=0;
signed int16 temperature_offset=0;
signed int16 poweroff_delay=30;
int8 i=0;
int16 j=0;
unsigned int16 buttimer=0;
unsigned int8 heaterStatus=0xff;
unsigned int8 heater2Status=0xff;
unsigned int16 milliseconds=0;
unsigned int16 idleminutes=0;
unsigned int16 setpointdelay=1000;
signed int16 filteredtemp=0;
unsigned int8 savesettings=0;
signed int16 stepsize=5;
signed int16 reference=2048;

void saveParms();

#int_timer2
void timer2isr(void) {
	
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

	if (!input(PIN_E3)||(buttimer>1&&buttimer<10))	// 10 ms debounce
		buttimer++;
	else
		buttimer=0;
		
	if (buttimer==1)
		event = EVT_BUTTON;
	else if (buttimer>1000)
		event = EVT_LONG_PRESS;
	// End reading encoder
	
		
	if (reed_status == REED_CLOSED && (state == ST_MAIN || state == ST_SETBACK))
		milliseconds++;
	else {
		milliseconds=0;
		idleminutes=0;
	}	
	
	if (milliseconds==60000)
	{
		milliseconds=0;
		idleminutes++;
		if (idleminutes == setback_delay && setback_delay != 31)
			state = ST_SETBACK;
		if (idleminutes == poweroff_delay && poweroff_delay != 130)
			state = ST_STANDBY;		
	}

	if (setpointdelay>0)
		setpointdelay--;

	if (setpointdelay==1)
		savesettings=1;
	
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
		if ((temp16/10) == 0 && num>=0)
			DISPLAY[j] = _SPACE;
		else if ((temp16/10) == 0 && temp16 != 0 && num < 0)
			DISPLAY[j] = _MINUS;
		else if (temp16 == 0 && num < 0)
			DISPLAY[j] = _SPACE;
		else if (j==0 && mode == DISP_REF)
			DISPLAY[j] = hex_table[(temp16/10) %10] _ADDPOINT;
		else
			DISPLAY[j] = hex_table[(temp16/10) %10];
		temp16 /= 10;
	}
}

void readAdc(int16* adc_r, unsigned int8 channel) {
	set_adc_channel(channel);
	i=0;
	adc_sum = 0;
	while (i<16) { // sample 16 times to get a virtual 16 bit reading..
		delay_us(5);
		adc_sum += read_adc();
//		adc_tulos = read_adc();
//		adc_sum = adc_sum + adc_tulos;
		i++;
	}
	*adc_r = adc_sum;
}

void tc_lookup_32bit(unsigned int16* adc_sum, signed int16* temperature)
{
	unsigned int32 sum32;
	sum32 = (unsigned int32)*adc_sum * (unsigned int32)reference / 2048;	// compensate for reference inaccuracy
	for (j=0;j<50;j++)
	{
		if (sum32<tc_lookup[j])
			break;
	}
	
	ylempi = tc_lookup[j];
	alempi = tc_lookup[j-1];
	temp1=10*(alempi-sum32);
	temp2=(alempi-ylempi);
	new_temp = temp1/temp2+ 10*(j-1)+kty_buf; // -1 because tc starts from zero.
	new_temp = new_temp - temperature_offset;			// subtract offset setting
	
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

void menuMachine()
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
					if (setpoint < 100)
						setpoint = 100;
					normal_setpoint = setpoint;
					setpointdelay=1000;
					break;
				case EVT_RIGHT:
					setpoint+=stepsize;
					if (setpoint > 450)
						setpoint = 450;
					normal_setpoint = setpoint;
					setpointdelay=1000;
					break;
			}
			break;
		case ST_STANDBY:
			memcpy (DISPLAY, stby, sizeof (DISPLAY));
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
			show_7seg_num(temperature_unit, filteredtemp);
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
					break;
			}
			break;	
		case ST_MENU_MAIN:
			memcpy (DISPLAY, bacc, sizeof (DISPLAY));
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
			memcpy (DISPLAY, setb, sizeof (DISPLAY));
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
			show_7seg_num(temperature_unit, setback);
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_SETBACK;
					break;
				case EVT_LEFT:
					if(setback>100)
						setback-=10;
					break;
				case EVT_RIGHT:
					if(setback<450)
						setback+=10;
					break;
			}
			break;
		case ST_MENU_DELAY:
			memcpy (DISPLAY, dela, sizeof (DISPLAY));
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
			if (setback_delay==31)
				memcpy (DISPLAY, off, sizeof (DISPLAY));
			else
				show_7seg_num(DISP_NUM, setback_delay);
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
					if(setback_delay<31)
						setback_delay++;
				break;
			}
			break;
		case ST_MENU_POWEROFF:
			memcpy (DISPLAY, poff, sizeof (DISPLAY));
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
			if (poweroff_delay > 120)
				memcpy (DISPLAY, off, sizeof (DISPLAY));
			else
				show_7seg_num(DISP_NUM, poweroff_delay);
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
					if (poweroff_delay>120)
						poweroff_delay=130;
					break;
			}
			break;
		case ST_MENU_OFFSET:
			memcpy (DISPLAY, ofse, sizeof (DISPLAY));
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
			show_7seg_num(temperature_unit, temperature_offset);
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_OFFSET;
					break;
				case EVT_LEFT:
					if(temperature_offset>-40)
						temperature_offset--;
					break;
				case EVT_RIGHT:
					if(temperature_offset<40)
						temperature_offset++;
					break;
			}
			break;
		case ST_MENU_UNIT:
			memcpy (DISPLAY, unit, sizeof (DISPLAY));
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
			show_7seg_num(temperature_unit, 0);
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
			memcpy (DISPLAY, step, sizeof (DISPLAY));
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
			show_7seg_num(temperature_unit, stepsize);
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_STEP_SIZE;
					break;
				case EVT_LEFT:
				case EVT_RIGHT:
					if(stepsize == 5)
						stepsize = 1;
					else
						stepsize = 5; 
					break;
			}
			break;
		case ST_MENU_DIAGNOSTICS:
			memcpy (DISPLAY, diag, sizeof (DISPLAY));
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
			memcpy (DISPLAY, bacc, sizeof (DISPLAY));
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_DIAGNOSTICS;
					break;
				case EVT_LEFT:
					state = ST_MENU_TC_2_READING;
					break;
				case EVT_RIGHT:
					state = ST_MENU_COLD_COMPENSATION;
					break;
			}
			break;
		case ST_MENU_COLD_COMPENSATION:
			memcpy (DISPLAY, cold, sizeof (DISPLAY));
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
			show_7seg_num(temperature_unit, kty_buf);
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_COLD_COMPENSATION;
					break;
			}
			break;
		case ST_MENU_REFERENCE:
			memcpy (DISPLAY, ref, sizeof (DISPLAY));
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
			show_7seg_num(DISP_REF, reference);
			switch(event)
			{
				case EVT_BUTTON:
					saveParms();
					state = ST_MENU_REFERENCE;
					break;
				case EVT_LEFT:
					if(reference>1945)
						reference--;
					break;
				case EVT_RIGHT:
					if(reference<2151)
						reference++;
					break;
			}
			break;
		case ST_MENU_TIP_TYPE:
			memcpy (DISPLAY, type, sizeof (DISPLAY));
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
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_TIP_TYPE;
					break;
			}
			break;
		case ST_MENU_REED_STATE:
			memcpy (DISPLAY, reed, sizeof (DISPLAY));
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
			switch(reed_status)
			{
				case REED_OPEN:
					memcpy (DISPLAY, open, sizeof(DISPLAY));
					break;
				case REED_CLOSED:
					memcpy (DISPLAY, clos, sizeof(DISPLAY));
					break;
			}			
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_REED_STATE;
					break;
			}
			break;
		case ST_MENU_TC_1_READING:
			memcpy (DISPLAY, tc_1, sizeof (DISPLAY));
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
			show_7seg_num(temperature_unit, right_buf);
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_TC_1_READING;
					break;
			}
			break;
		case ST_MENU_TC_2_READING:
			memcpy (DISPLAY, tc_2, sizeof (DISPLAY));
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_SHOW_TC_2_READING;
					break;
				case EVT_LEFT:
					state = ST_MENU_TC_1_READING;
					break;
				case EVT_RIGHT:
					state = ST_MENU_BACK_FROM_DIAGNOSTICS;
					break;
			}
			break;
		case ST_SHOW_TC_2_READING:
			show_7seg_num(temperature_unit, left_buf);
			switch(event)
			{
				case EVT_BUTTON:
					state = ST_MENU_TC_2_READING;
					break;
			}
			break;
	} // end menu case
	event = EVT_NONE;
}		


void saveParms()
{
	write_int16_eeprom(0, setpoint);
	write_int16_eeprom(2, setback);
	write_int16_eeprom(4, setback_delay);
	write_int16_eeprom(6, poweroff_delay);
	write_int16_eeprom(8, temperature_offset);
	write_int16_eeprom(10, reference);
	write_int16_eeprom(12, stepsize);
	write_eeprom(14, temperature_unit);
}	


void main()
{
	unsigned int16 temp_reading=0;
	unsigned int32 right_sum=0;
	unsigned int32 left_sum=0;
	unsigned int16 kty_sum=0;
	unsigned int16 reed_sum=0;
	setup_timer_2(T2_DIV_BY_16,250,2); // set timer2 to 1ms (prescaler 16 and postscaler 2, period 250)
	enable_interrupts(INT_TIMER2);
	enable_interrupts(GLOBAL);
	set_tris_a(0b00000111);
	set_tris_b(0b11001111);
	set_tris_c(0x00);
	set_tris_e(0x11111000);
	port_a_pullups(0b00000000);
	port_b_pullups(0b11000000);

	DISPLAY[4]=_SPACE;
		
	setup_adc_ports( sAN0|sAN1|sAN2|sAN9|sAN12,VSS_FVR);	// AN9 = reed pullup
	setup_vref(VREF_ON | VREF_ADC_2v048 | VREF_COMP_DAC_2v048);
	setup_opamp2(OPAMP_ENABLED | OPAMP_HIGH_GBWP_MODE | OPAMP_NI_TO_FVR);		// To enable KTY pullup
	setup_dac(DAC_OFF);

	setup_adc(ADC_CLOCK_DIV_64);

	if (read_eeprom(0)==0xFF && read_eeprom(1)==0xFF)	// Reprogramming has erased EEPROM or first time run
		saveParms();
	setpoint = read_int16_eeprom(0);
	setback = read_int16_eeprom(2);			// must use even addresses because one int16 takes two bytes
	setback_delay = read_int16_eeprom(4);
	poweroff_delay = read_int16_eeprom(6);
	temperature_offset = read_int16_eeprom(8);
	reference = read_int16_eeprom(10);
	stepsize = read_int16_eeprom(12);
	temperature_unit = read_int16_eeprom(14);

	normal_setpoint = setpoint;

	/* Infinite loop */
	while(1)
	{
		menuMachine();			// menuMachnie call takes about 520 µs when in 'normal' mode (displaying temp)
		output_low(HEATER_1);	// Heater 1 (right) off
		output_low(HEATER_2);	// Heater 2 (left) off
		delay_us(10);

		/* READ REED STATUS */	/*Measured 21.12.2015: pullup is on for 580µs ~ readAdc time*/
		setup_adc_ports( sAN0|sAN1|sAN2|sAN12,VSS_VDD);	// AN9 = reed pullup
		set_tris_b(0b11000111);
		output_high(REED_PULLUP);
		readAdc(&reed_sum, 12);
		output_low(REED_PULLUP);
		set_tris_b(0b11001111);
		setup_adc_ports( sAN0|sAN1|sAN2|sAN9|sAN12,VSS_FVR);	// AN9 = reed pullup
		
		if (reed_sum<5000)
			reed_status = REED_CLOSED;
		else
			reed_status = REED_OPEN;

// Commented block below to recognize tip type from resistor in parallel with reed sw (works only when not in stand)	
					
/*		else if (temp_reading<37000)
			tiptype = TYPE_WMRP;
		else
			tiptype = TYPE_WMRT;
*/		
		/* KTY pullup is on for 560µs (measured 21.12.2015 */
		setup_opamp2(OPAMP_ENABLED | OPAMP_HIGH_GBWP_MODE | OPAMP_NI_TO_FVR);		// To enable KTY pullup
		// delay_us(5);	// takes only about 1 us to stabilize and readAdc function already has 5us (measured 21.12.2015)
		readAdc(&kty_sum, 2);
		if (state != ST_ADJ_REFERENCE)			// Keep KTY pullup on (reference on) when adjusting reference compensation
			setup_opamp2(OPAMP_DISABLED);		// To disable KTY pullup
		kty_lookup_16bit(&kty_sum, &kty_buf);
		if (kty_buf > 130)	// No KTY sensor or bad contact
			kty_buf=30;		// use some default value

		if (savesettings && state == ST_MAIN)	// do not save setpoint if already gone to stby
		{
			write_int16_eeprom(0, setpoint);
			savesettings=0;
		}	
		
		/* some filtering to displayed temperature to reduce flickering */
		temp_reading = filteredtemp*30;
		temp_reading = temp_reading + right_buf*10;
		filteredtemp = temp_reading / 40;

		if ((filteredtemp-setpoint < 3) && (setpoint-filteredtemp < 3))
			filteredtemp = setpoint;
		
		// delay_us(300);		// takes about 200-300 µs for opamp output to stabilize after pullups turned off - menuMachine call below should be enough
		delay_us(100);
		menuMachine();
		
		readAdc(&left_sum, 1);	// read temperature ja right_sum summaus vie n. 0.6 ms (5 us viive ja ADC_CLOCK_DIV64 @ 32 MHz)
		tc_lookup_32bit(&left_sum, &left_buf);

		readAdc(&right_sum, 0);	// read temperature ja right_sum summaus vie n. 0.6 ms (5 us viive ja ADC_CLOCK_DIV64 @ 32 MHz)
		tc_lookup_32bit(&right_sum, &right_buf);

		/* Tip type recognision by pulsing current to heater2 = wmrt left heater. If drives TC2 to rail, must be WMRT */
		set_adc_channel(1);
		output_high(HEATER_2);
		delay_us(10);
		temp_reading = read_adc();
		output_low(HEATER_2);
			
		if (reed_sum>50000)
			tiptype = TYPE_NC;
		else if (temp_reading > 4090)
			tiptype = TYPE_WMRT;
		else
			tiptype = TYPE_WMRP;
	

		if (tiptype == TYPE_NC) {
			right_buf=555;
		}

		if (right_buf<setpoint) {
			heaterStatus = 0x7f;
			output_high(HEATER_1);
		}
		else {
			heaterStatus = 0xff;
			output_low(HEATER_1);
		}	

		if (left_buf<setpoint && tiptype == TYPE_WMRT) {
			heater2Status = 0x7f;
			output_high(HEATER_2);
		}
		else {
			heater2Status = 0xff;
			output_low(HEATER_2);
		}	

		/* Heating cycle. Call menuMachine periodically to not miss events */
		/* 4*2ms delays + 4 calls to menuMachine are 10.5ms in reality */
		menuMachine();
		delay_ms(2);
		menuMachine();
		delay_ms(2);
		menuMachine();
		delay_ms(2);
		menuMachine();
		delay_ms(2);
		/* Complete cycle about 15 ms (measured 21.12.2015)*/
	}	// end while
}