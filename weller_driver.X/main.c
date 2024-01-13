#define FW_VERSION 920
// #define WELLER_CC_DISPLAY	// Uncomment this line if you use common cathode display
// #define BUZZ_MOD             // Uncomment this line if you applied the buzzer mod
// #define DEBUG                // Uncomment to enable some debugging functions
#define POLY_LOOKUP          // Uncomment to enable polynomial temperature lookups

#ifdef WELLER_CC_DISPLAY
#define SEGMENT_A { LATC = 0b00000001; };
#define SEGMENT_B { LATC = 0b00000010; };
#define SEGMENT_C { LATC = 0b00000100; };
#define SEGMENT_D { LATC = 0b00001000; };
#define SEGMENT_E { LATC = 0b00010000; };
#define SEGMENT_F { LATC = 0b00100000; };
#define SEGMENT_G { LATC = 0b01000000; };
#define SEGMENT_DP { LATC = 0b10000000; };
#define SEGMENT_NONE { LATC = 0b00000000; };
#else
#define SEGMENT_A { LATC = 0b11111110; };
#define SEGMENT_B { LATC = 0b11111101; };
#define SEGMENT_C { LATC = 0b11111011; };
#define SEGMENT_D { LATC = 0b11110111; };
#define SEGMENT_E { LATC = 0b11101111; };
#define SEGMENT_F { LATC = 0b11011111; };
#define SEGMENT_G { LATC = 0b10111111; };
#define SEGMENT_DP { LATC = 0b01111111; };
#define SEGMENT_NONE { LATC = 0b11111111; };
#define CC_DISPLAY	// Actually we use common anode display but as we mux
// segments and not digits we must use inverted patterns!
#endif

#include "xc8_header.h"
#include "7seg_chars.h"

uint_fast8_t EepromRead8(uint_fast16_t address);
uint_fast16_t EepromRead16(uint_fast16_t address);
void EepromWrite8(uint_fast16_t address, uint_fast8_t data);
void EepromWrite16(uint_fast16_t address, uint_fast16_t data);
inline int_fast16_t AdcRead(uint_fast8_t channel);

// Some default parameter values to EEPROM. Not necessary since all EEPROM values are verified to be
// in range at startup.
// __EEPROM_DATA(0x7c, 0x01, 0xfa, 0x00, 0x05, 0x1e, 0x00, 0x00);
// __EEPROM_DATA(0x80, 0x05, 0x00, 0x00, 0x32, 0x05, 0x00);

#define REED_PULLUP LATBbits.LATB3
#define HEATER_R LATBbits.LATB4
#define HEATER_L LATBbits.LATB5
#ifdef BUZZ_MOD
#define BUZZER LATBbits.LATB1
#endif

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
#define DEFAULT_MAXDUTY 5				// 58% duty cycle on 50 Hz, 50% on 60 Hz
#define DEFAULT_IDLEDUTY 0				// Idle detection from duty cycle off as default

// Assign sane / safe limits to the above parameters
#define MAX_SETPOINT 450				// these are also used
#define MIN_SETPOINT 100				// for setback range
#define MAX_SETBACK_DELAY 30
#define MAX_POWEROFF_DELAY 120
#define MAX_TEMPERATURE_OFFSET 40
#define MAX_REFERENCE_TOLERANCE 103		// max. tolerance is 5% of 2.048
#define MAX_MAXDUTY 9
#define MIN_MAXDUTY 3					// This means actually max setting, which is 75% for 50 Hz and 70% for 60 Hz
#define MAX_IDLEDUTY 75

// Message examples
uint_fast8_t const cold[5] = {_C, _O, _L, _D, _SPACE}; //  CoLd
uint_fast8_t const stby[5] = {_S, _T, _B, _Y, _SPACE}; //  Stby
uint_fast8_t const setb[5] = {_S, _E, _T, _B, _SPACE}; //  SEtb
uint_fast8_t const bacc[5] = {_B, _A, _C, _C, _SPACE}; //  bAcc
uint_fast8_t const dela[5] = {_D, _E, _L, _A, _SPACE}; //  dELA
uint_fast8_t const off[5] = {_SPACE, _O, _F, _F, _SPACE}; //  oFF
uint_fast8_t const poff[5] = {_P, _O, _F, _F, _SPACE}; //  PoFF
uint_fast8_t const ofse[5] = {_O, _F, _S, _E, _SPACE}; //  oFSE
uint_fast8_t const unit[5] = {_U, _N, _I, _T, _SPACE}; //  Unit
uint_fast8_t const step[5] = {_S, _T, _E, _P, _SPACE}; //  StEP
uint_fast8_t const diag[5] = {_D, _I, _A, _G, _SPACE}; //  diAG
uint_fast8_t const ref[5] = {_SPACE, _R, _E, _F, _SPACE}; //  rEF
uint_fast8_t const type[5] = {_T, _Y, _P, _E, _SPACE}; //  tyPE
uint_fast8_t const wmrp[5] = {_W, _M, _R, _P, _SPACE}; //  WMrP
uint_fast8_t const wmrt[5] = {_W, _M, _R, _T, _SPACE}; //  WMrt
uint_fast8_t const nc[5] = {_SPACE, _SPACE, _N, _C, _SPACE}; //  nC
uint_fast8_t const reed[5] = {_R, _E, _E, _D, _SPACE}; //  rEEd
uint_fast8_t const open[5] = {_O, _P, _E, _N, _SPACE}; //  oPEn
uint_fast8_t const clos[5] = {_C, _L, _O, _S, _SPACE}; //  CLoS
uint_fast8_t const tc_R[5] = {_T, _C, _SPACE, _R, _SPACE}; //  tC r
uint_fast8_t const tc_L[5] = {_T, _C, _SPACE, _L, _SPACE}; //  tC L
uint_fast8_t const pwmR[5] = {_D, _C, _SPACE, _R, _SPACE}; //  dC r
uint_fast8_t const pwmL[5] = {_D, _C, _SPACE, _L, _SPACE}; //  dC L
uint_fast8_t const idle[5] = {_I, _D, _L, _E, _SPACE}; //  idLE
uint_fast8_t const dcli[5] = {_D, _C, _L, _I, _SPACE}; //  dCLi
uint_fast8_t const poor[5] = {_P, _O, _O, _R, _SPACE}; //  Poor
uint_fast8_t const on[5] = {_SPACE, _SPACE, _O, _N, _SPACE}; // on
uint_fast8_t const freq[5] = {_F, _R, _E, _Q, _SPACE}; //	FrEq
uint_fast8_t const vers[5] = {_V, _E, _R, _S, _SPACE}; //  vErS

uint_fast8_t const hexTable[16] = {_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _A, _B, _C, _D, _E, _F};

typedef enum {
    ST_MENU_MAIN,
    ST_MENU_SETBACK,
    ST_MENU_DELAY,
    ST_MENU_POWEROFF,
    ST_MENU_OFFSET,
    ST_MENU_UNIT,
    ST_MENU_STEP_SIZE,
    ST_MENU_DIAGNOSTICS,
    ST_ADJ_SETBACK,
    ST_ADJ_DELAY,
    ST_ADJ_POWEROFF,
    ST_ADJ_OFFSET,
    ST_ADJ_UNIT,
    ST_ADJ_STEP_SIZE,
    ST_MENU_BACK_FROM_DIAGNOSTICS,
    ST_MENU_COLD_COMPENSATION,
    ST_MENU_REFERENCE,
    ST_MENU_TIP_TYPE,
    ST_MENU_REED_STATE,
    ST_MENU_TC_R_READING,
    ST_MENU_TC_L_READING,
    ST_MENU_PWM_R_READING,
    ST_MENU_PWM_L_READING,
    ST_MENU_IDLE_DC,
    ST_MENU_DC_LIMIT,
    ST_MENU_POOR,
    ST_MENU_FREQUENCY,
    ST_MENU_FW_VERSION,
    ST_SHOW_COLD_COMPENSATION,
    ST_ADJ_REFERENCE,
    ST_SHOW_TIP_TYPE,
    ST_SHOW_REED_STATE,
    ST_SHOW_TC_R_READING,
    ST_SHOW_TC_L_READING,
    ST_SHOW_PWM_R_READING,
    ST_SHOW_PWM_L_READING,
    ST_ADJ_IDLE_DC,
    ST_ADJ_DC_LIMIT,
    ST_ADJ_POOR,
    ST_ADJ_FREQUENCY,
    ST_SHOW_FW_VERSION,
    ST_MAIN,
    ST_SETBACK,
    ST_STANDBY,
} STATES;
STATES State = ST_MAIN;

typedef enum {
    EVT_LEFT,
    EVT_RIGHT,
    EVT_BUTTON_SHORT,
    EVT_BUTTON_LONG,
    EVT_NONE
} EVENTS;
EVENTS Event = EVT_NONE;

typedef enum {
    TYPE_WMRP,
    TYPE_WMRT,
    TYPE_NC
} TIP_TYPES;
TIP_TYPES TipType = TYPE_WMRP;

typedef enum {
    REED_CLOSED,
    REED_OPEN
} REED_STATES;
REED_STATES ReedStatus = REED_OPEN;
REED_STATES PreviousReedStatus = REED_OPEN;

typedef enum {
    DISP_C,
    DISP_F,
    DISP_NUM,
    DISP_REF
#ifdef DEBUG
    , DISP_HEX
#endif
} DISP_MODES;

// Custom thermocouple 0..500 C difference, op amp gain 241, reference 2.048V
// after second calibration round
#ifndef POLY_LOOKUP
uint_fast16_t const TcLookupData[51] = {0, 1003, 2028, 3073, 4138, 5223, 6327, 7449, 8588, 9743,
    10915, 12101, 13302, 14517, 15745, 16986, 18238, 19501, 20774, 22057,
    23349, 24649, 25956, 27271, 28591, 29916, 31247, 32581, 33918, 35258,
    36600, 37943, 39286, 40629, 41971, 43311, 44649, 45984, 47315, 48642,
    49963, 51279, 52588, 53889, 55183, 56468, 57743, 59008, 60263, 61506,
    62736};
#else
// f(x) = 1.02451488165452 + 0.00959004175229577 * x + -5.66340700498518E-08 * x^2 + 4.88503811305787E-13 * x^3
#define TC_POLY_COEFF_0    1024515
#define TC_POLY_COEFF_1  959004175
#define TC_POLY_COEFF_2 -566340700
#define TC_POLY_COEFF_3       4885
#endif

// Here is table after first calibration round
//#ifndef POLY_LOOKUP
//uint_fast16_t const tcLookupData[51] = {0,944,1912,2903,3917,4954,6011,7089,8188,9305,
//									10441,11595,12767,13954,15158,16376,17609,18855,20115,21387,
//									22670,23964,25268,26582,27904,29235,30572,31917,33267,34623,
//									35983,37347,38714,40084,41455,42827,44200,45572,46943,48313,
//									49679,51043,52403,53758,55108,56452,57789,59119,60441,61753,
//									63057};
//#else
//#define TC_POLY_COEFF_0    1547710
//#define TC_POLY_COEFF_1  999655845
//#define TC_POLY_COEFF_2 -659027776
//#define TC_POLY_COEFF_3       5221
//#endif

// KTY82/110 temperature sensor connected between GND and 1k pullup to reference. -50...+150 in 10 degree steps
// ADC REF = 2.048V
#ifndef POLY_LOOKUP 

uint_fast16_t const KtyLookupData[21] =
#ifndef BUZZ_MOD
{22278, 23713, 25181, 26619, 28023, 29428, 30787, 32116, 33411, 34652,
    35868, 37030, 38138, 39216, 40242, 41227, 42172, 43054, 43857, 44558, 45126};
#else
        // The digital output doesn't reach Vdd fully, according to the datasheet it's typically down by ~96% at 2.5mA
{
    21387, 22765, 24174, 25554, 26902, 28251, 29556, 30832, 32074, 33266,
    34434, 35549, 36612, 37648, 38633, 39578, 40485, 41332, 42103, 42775, 43321
};
#endif

#else

#ifndef BUZZ_MOD
// f(x) = 69.2214564312176 + -0.0300199006192531 * x + 1.86463281201501E-06 * x^2 + -4.20621130087746E-11 * x^3 + 3.6218784771061E-16 * x^4
#define KTY_POLY_COEFF_0   69221456
#define KTY_POLY_COEFF_1 -300199006
#define KTY_POLY_COEFF_2 1864632812
#define KTY_POLY_COEFF_3 -420621130
#define KTY_POLY_COEFF_4       3622
#else
#define KTY_POLY_COEFF_0   69221445
#define KTY_POLY_COEFF_1 -312707283
#define KTY_POLY_COEFF_2 2023256016
#define KTY_POLY_COEFF_3 -475419918
#define KTY_POLY_COEFF_4       4264
#endif

#endif
uint_fast8_t Display[5]; // Display memory, last char is clock colons / degree sign

// Saveable parameters. These will be downloaded from EEPROM and set to default
// if any of them are out of range, therefore no need to give initial values.
volatile int_fast16_t SolderSetpoint;
volatile int_fast16_t SetbackSetpoint;
volatile uint_fast8_t SetbackDelay;
volatile uint_fast8_t PoweroffDelay;
volatile int_fast8_t TemperatureOffset;
volatile int_fast16_t Reference;
volatile uint_fast8_t Stepsize;
volatile DISP_MODES TemperatureUnit;
volatile __bit PoorMode;
volatile uint_fast8_t MainsFrequency;
volatile uint_fast8_t MaxDuty;
volatile uint_fast8_t IdleDuty;

volatile int_fast16_t ActiveSetpoint = 0;
volatile uint_fast8_t HeaterStatusR = 0xff;
volatile uint_fast8_t HeaterStatusL = 0xff;
volatile uint_fast32_t IdleMilliseconds = 0;
volatile uint_fast8_t IdleMinutes = 0;
volatile uint_fast16_t SetpointDelay = 1000;
volatile int_fast16_t DisplayTemperature = 0;
volatile uint_fast8_t HeaterDutyR = 0;
volatile uint_fast8_t HeaterDutyL = 0;

volatile int_fast16_t HeaterTemperatureR = 0;
volatile int_fast16_t HeaterTemperatureL = 0;
volatile int_fast16_t KtyTemperature = 25;

volatile __bit DutyCycleOn = 0;
volatile __bit SaveAllSettings = 0;
volatile __bit SaveSetpoint = 0;
#ifdef BUZZ_MOD
volatile uint_fast16_t BuzzerCycles = 0;
#endif

void showSevenSegNum(DISP_MODES mode, int_fast32_t num); // converts integers to 7 segment numbers
inline void readAdc(int_fast32_t* adc, uint_fast8_t channel); // reads specified ADC input 16 times to get a virtual 16 bit reading
int_fast16_t tcLookup(int_fast32_t* adcSum); // converts thermocouple voltage to temperature
int_fast16_t ktyLookup(int_fast32_t* adcSum); // converts cold junction compensation sensor voltage to temperature
inline void menuMachine(void); // updates menu state based on encoder event
inline void updateDisplay(void); // updates display memory based on current menu state
void recognizeTypeInStand(void); // recognizes tip type even when in stand
void saveParms(void); // saves current parameters to EEPROM
inline void init(void); // initialize MCU

inline void timer2isr(void) {
    static uint_fast8_t OldEncoderValue = 0;
    static uint_fast8_t CurrentSegment = 0;
    static uint_fast16_t ButtonTimer = 0;
    static uint_fast8_t ButtonReleaseTimer = 0;
    static uint_fast16_t HeaterCyclesR = 0;
    static uint_fast16_t HeaterCyclesL = 0;
    static uint_fast16_t HeaterCyclesDutyCounter = 0;
    static __bit DegreeDimmingFlag = 0;
    static uint_fast8_t DigitFour = 0;
    static uint_fast8_t MainsCyclesLimit = 0;
    static uint_fast8_t MainsCycles = 0;

    // LATA = _8; // for testing interrupt length, currently 20-25 us when menuMachine call moved to interrupt

    // Limit for: 100 Hz counter at 50 Hz mains frequency
    //            120 Hz counter at 60 Hz mains frequency
    MainsCyclesLimit = (MainsFrequency == 50) ? 12 : 10;
    
    // Update mainsCycles. It is used to trigger next temperature readout, somewhat synced to AC phase
    
    MainsCycles++;
    if (MainsCycles >= MainsCyclesLimit) {
        MainsCycles = 0;
        DutyCycleOn = 0;
    }
    else if (MainsCycles >= MaxDuty) {
        DutyCycleOn = 1;
    }
    
    // Dim decimal point if reed closed, i.e. iron on stand
    if (DegreeDimmingFlag == 1 && ReedStatus == REED_CLOSED)
        DigitFour = _SPACE;
    else
        DigitFour = Display[4];

    // Seven segment display multiplexing
	LATA = _SPACE; // clear anodes
    switch (CurrentSegment) {
        case 0: SEGMENT_A
            LATA = (uint_fast8_t) (((DigitFour & 0x01) << 7) | ((Display[3]&0x01) << 6) | ((Display[2]&0x01) << 5) | ((Display[1]&0x01) << 4) | ((Display[0]&0x01) << 3));
            break;
        case 1: SEGMENT_B
            LATA = (uint_fast8_t) (((DigitFour & 0x02) << 6) | ((Display[3]&0x02) << 5) | ((Display[2]&0x02) << 4) | ((Display[1]&0x02) << 3) | ((Display[0]&0x02) << 2));
            break;
        case 2: SEGMENT_C
            LATA = (uint_fast8_t) (((DigitFour & 0x04) << 5) | ((Display[3]&0x04) << 4) | ((Display[2]&0x04) << 3) | ((Display[1]&0x04) << 2) | ((Display[0]&0x04) << 1));
            break;
        case 3: SEGMENT_D
            LATA = (uint_fast8_t) (((Display[3]&0x08) << 3) | ((Display[2]&0x08) << 2) | ((Display[1]&0x08) << 1) | (Display[0]&0x08));
            break;
        case 4: SEGMENT_E
            LATA = (uint_fast8_t) (((Display[3]&0x10) << 2) | ((Display[2]&0x10) << 1) | ((Display[1]&0x10)) | ((Display[0]&0x10) >> 1));
            break;
        case 5: SEGMENT_F
            LATA = (uint_fast8_t) (((Display[3]&0x20) << 1) | ((Display[2]&0x20)) | ((Display[1]&0x20) >> 1) | ((Display[0]&0x20) >> 2));
            break;
        case 6: SEGMENT_G
            LATA = (uint_fast8_t) (((Display[3]&0x40)) | ((Display[2]&0x40) >> 1) | ((Display[1]&0x40) >> 2) | ((Display[0]&0x40) >> 3));
            break;
        case 7: SEGMENT_DP
            LATA = (uint_fast8_t) ((((Display[3]&0x80) _ADD_HEATER_STATUS) >> 1) | (((Display[2]&0x80) _ADD_HEATER_2_STATUS) >> 2) | ((Display[1]&0x80) >> 3) | ((Display[0]&0x80) >> 4));
            break;
        default: SEGMENT_NONE
            LATA = _SPACE; // clear anodes
            break;
    }

    // Dim the display if on standby
    CurrentSegment++;
    if (CurrentSegment == 8 && State != ST_STANDBY) {
        CurrentSegment = 0;
        DegreeDimmingFlag = !DegreeDimmingFlag; // Toggle degreeDimmer bit at each multiplexing round
    }
    if (CurrentSegment == 16)
        CurrentSegment = 0;

    // ENCODER READOUT
    if (!PORTBbits.RB7) {
        if (!PORTBbits.RB6 && (OldEncoderValue & 0x40))
            Event = EVT_RIGHT;
        if (PORTBbits.RB6 && !(OldEncoderValue & 0x40))
            Event = EVT_LEFT;
    }
    
    OldEncoderValue = PORTB;

    // Encoder button readout
    if (!(PORTEbits.RE3) || (ButtonTimer > 0 && ButtonTimer < 72)) // 60 ms debounce on press
        ButtonTimer++;
    else
        ButtonTimer = 0;

    if ((PORTEbits.RE3) && (ButtonReleaseTimer > 0)) // 30 ms debounce on release
        ButtonReleaseTimer--;
    else if (!(PORTEbits.RE3) && ButtonTimer > 1)
        ButtonReleaseTimer = 36;

    if (ButtonTimer == 1 && ButtonReleaseTimer == 0)
        Event = EVT_BUTTON_SHORT;
    else if (ButtonTimer > 1023) {
        Event = EVT_BUTTON_LONG;
        ButtonTimer = 1024;
    }
    // End reading encoder

    menuMachine(); // menuMachine() call moved here to prevent missing events if interrupt happens during menuMachine call

    // Count time when iron is on stand or heater duty cycles are below idle duty.
    if ((((HeaterDutyR < IdleDuty) && (HeaterDutyL < IdleDuty)) || (ReedStatus == REED_CLOSED)) && (State == ST_MAIN || State == ST_SETBACK))
        IdleMilliseconds++;
    else {
        IdleMilliseconds = 0;
        IdleMinutes = 0;
    }

    if (IdleMilliseconds == 72191) // 72115*832 us = 60 s. But 72191 is faster to compare
    {
        IdleMilliseconds = 0;
        if (IdleMinutes < 255) // don't let idleminutes overflow
            IdleMinutes++;
    }

    // The time counted above is needed to determine when the setback or standby modes should be activated.
    if (IdleMilliseconds == 1 && IdleMinutes == SetbackDelay && SetbackDelay != 31 && State == ST_MAIN)
        State = ST_SETBACK;
    if (IdleMilliseconds == 1 && IdleMinutes == PoweroffDelay && PoweroffDelay != 130 && (State == ST_MAIN || State == ST_SETBACK))
        State = ST_STANDBY;

    // Setpointdelay means time setpoint is shown after adjusting, before display reverts back to show actual temperature
    if (SetpointDelay > 0)
        SetpointDelay--;

    // The setpoint is saved to EEPROM after the setpoint delay, to save EERPOM from wearing out
    if (SetpointDelay == 1)
        SaveSetpoint = 1;

    // Calculate the average duty cycle driven to heaters
    if (HeaterStatusR == 0x7f)
        HeaterCyclesR++;

    if (HeaterStatusL == 0x7f)
        HeaterCyclesL++;

    HeaterCyclesDutyCounter++;
    if (HeaterCyclesDutyCounter == 800) {
        HeaterCyclesDutyCounter = 0;
        HeaterDutyR = (uint_fast8_t) (HeaterCyclesR >> 3);
        HeaterDutyL = (uint_fast8_t) (HeaterCyclesL >> 3);
        HeaterCyclesR = 0;
        HeaterCyclesL = 0;
    }

#ifdef BUZZ_MOD
    if (BuzzerCycles > 0)
        BuzzerCycles--;
#endif

    // LATA = _SPACE; // to test interrupt length
}

void __interrupt() isrMain(void) {
    // All interrupts land here
    
    STATUS = 0;
    
    // Timer 2 ISR
    if (PIR1bits.TMR2IF) {
        // #int_timer2			// timer2 is set to 832 us
        timer2isr();

        PIR1bits.TMR2IF = 0;
    }
    else {
        // Unkown interrupt. No good way to handle, reset self.
        
        HEATER_R = 0;
        HEATER_L = 0;
#ifdef BUZZ_MOD
        BUZZER = 0;
#endif
        RESET();
    }
}

void main(void) {
    static uint_fast8_t KtyCycles = 0;
    static uint_fast8_t KtyFilteringCycles = 0;
    static int_fast16_t ReedSum = 0;
    static int_fast32_t AdcSumR = 0;
    static int_fast32_t AdcSumL = 0;
    static int_fast32_t AdcSumR2 = 0;
    static int_fast32_t AdcSumL2 = 0;
    static int_fast32_t KtySum = 0;
    static int_fast16_t KtyFilteringSum = 0;
    static int_fast16_t KtyReading = 0;
    static __bit CalculateTemperature = 0;
    static __bit ReadTip = 0;
    static __bit AdcReadSwitch = 0;
#ifdef BUZZ_MOD
    static __bit WaitingForTemperature = 0;
#endif

    init();

    Display[4] = _SPACE;

    SolderSetpoint = (int_fast16_t) EepromRead16(0);
    SetbackSetpoint = (int_fast16_t) EepromRead16(2); // must use even addresses because one int_fast16_t takes two uint_fast8_ts
    SetbackDelay = EepromRead8(4);
    PoweroffDelay = EepromRead8(5);
    TemperatureOffset = (int_fast8_t) EepromRead8(6);
    Reference = (int_fast16_t) EepromRead16(7);
    Stepsize = EepromRead8(9);
    TemperatureUnit = EepromRead8(10);
    PoorMode = (__bit) EepromRead8(11);
    MainsFrequency = EepromRead8(12);
    MaxDuty = EepromRead8(13);
    IdleDuty = EepromRead8(14);

    if ((SolderSetpoint < MIN_SETPOINT || SolderSetpoint > MAX_SETPOINT) || // Ensure safe temperature setpoints in case of corrupt EEPROM data
            (SetbackSetpoint < MIN_SETPOINT || SetbackSetpoint > MAX_SETPOINT) ||
            (SetbackDelay > MAX_SETBACK_DELAY + 1) ||
            (PoweroffDelay > MAX_POWEROFF_DELAY + 10) ||
            (TemperatureOffset < -(MAX_TEMPERATURE_OFFSET) || TemperatureOffset > MAX_TEMPERATURE_OFFSET) ||
            (Reference < 2048 - MAX_REFERENCE_TOLERANCE || Reference > 2048 + MAX_REFERENCE_TOLERANCE) ||
            (Stepsize != 1 && Stepsize != 5) ||
            (TemperatureUnit != DISP_C && TemperatureUnit != DISP_F) ||
            (MainsFrequency != 50 && MainsFrequency != 60) ||
            (MaxDuty < MIN_MAXDUTY || MaxDuty > MAX_MAXDUTY) ||
            (IdleDuty > MAX_IDLEDUTY)) {

        SolderSetpoint = DEFAULT_SETPOINT;
        SetbackSetpoint = DEFAULT_SETBACK;
        SetbackDelay = DEFAULT_SETBACK_DELAY;
        PoweroffDelay = DEFAULT_POWEROFF_DELAY;
        TemperatureOffset = DEFAULT_TEMPERATURE_OFFSET;
        Reference = DEFAULT_REFERENCE;
        Stepsize = DEFAULT_STEPSIZE;
        TemperatureUnit = DEFAULT_TEMPERATURE_UNIT;
        PoorMode = DEFAULT_POORMODE;
        MainsFrequency = DEFAULT_MAINSFREQUENCY;
        MaxDuty = DEFAULT_MAXDUTY;
        IdleDuty = DEFAULT_IDLEDUTY;
        
        saveParms();
    }

    DisplayTemperature = 0;
    ActiveSetpoint = SolderSetpoint;

    recognizeTypeInStand();
    recognizeTypeInStand();

    __delay_ms(10); // let auto-zero op-amps stabilize before tip type recognition

    TipType = TYPE_WMRP;
    recognizeTypeInStand();

    /* Infinite loop */
    while (1) {
        HEATER_R = 0; // Heater 1 (right) off
        HEATER_L = 0; // Heater 2 (left) off
        
        HeaterStatusR = 0xff;
        HeaterStatusL = 0xff;

        __delay_us(10);

        // READ REED STATUS - Reed pullup is on for 35 us (measured 21.1.2017 / FW 0.6)

        // setup_adc_ports( sAN0|sAN1|sAN2|sAN12,VSS_VDD);	// AN9 = reed pullup
        ADCON1bits.ADPREF = 0b00;
        ADCON1bits.ADNREF = 0;
        ANSELBbits.ANSB3 = 0;

        TRISBbits.TRISB3 = 0;

        REED_PULLUP = 1;

        ReedSum = AdcRead(12);

        REED_PULLUP = 0;

        TRISBbits.TRISB3 = 1;

        // setup_adc_ports( sAN0|sAN1|sAN2|sAN9|sAN12,VSS_FVR);	// AN9 = reed pullup
#ifndef BUZZ_MOD
        ADCON1bits.ADPREF = 0b11;
        ADCON1bits.ADNREF = 0;
#endif
        ANSELBbits.ANSB3 = 1;

        // KTY READOUT - KTY pullup is on for 34 us (measured 21.1.2017 / FW 0.6)

#ifndef BUZZ_MOD
        //setup_opamp2(OPAMP_ENABLED | OPAMP_HIGH_GBWP_MODE | OPAMP_NI_TO_FVR);		// To enable KTY pullup
        OPA2CON = 0b11000011;
#else
        LATBbits.LATB2 = 1;
        TRISBbits.TRISB2 = 0;
#endif

        KtySum += AdcRead(2);

        if (State != ST_ADJ_REFERENCE) // Keep KTY pullup on (reference on) when adjusting reference compensation
        {
#ifndef BUZZ_MOD
            // setup_opamp2(OPAMP_DISABLED);		// To disable KTY pullup
            OPA2CON = 0;
#else
            TRISBbits.TRISB2 = 1;
            LATBbits.LATB2 = 0;
#endif
        }

#ifdef BUZZ_MOD        
        ADCON1bits.ADPREF = 0b11;
#endif   
        // delay_us(300);		// takes about 200-300 �s for opamp output to stabilize after pullups turned off - delay not
        // needed anymore because wait for mainsCycles below gives more than enough time.

        // added 2*832us waiting to reduce heating duty cycle in FW 0.7
        // wait here for measurement cycle start
        while(!DutyCycleOn);
        
        readAdc(&AdcSumL, 1);
        readAdc(&AdcSumR, 0);
        
        if (!AdcReadSwitch) {
            readAdc(&AdcSumL, 1); // this function should take about 0.3 ms (ADC_CLOCK_DIV32 @ 32 MHz)
            readAdc(&AdcSumR, 0);
            AdcReadSwitch = 1;
        } else {
            readAdc(&AdcSumL2, 1);
            readAdc(&AdcSumR2, 0);
            AdcReadSwitch = 0;
            CalculateTemperature = 1;
        }

        if (ReadTip) { // seems that new iron is connected with reed closed - lets check the type!
            TipType = TYPE_WMRP; // default to WMRP
            recognizeTypeInStand(); // check if it is WMRT
            ReadTip = 0;
        }
        
        if (TipType != TYPE_NC && HeaterTemperatureR < ActiveSetpoint) {
            HeaterStatusR = 0x7f;
            HEATER_R = 1;
        } else {
            HeaterStatusR = 0xff;
            HEATER_R = 0;
        }

        if (TipType == TYPE_WMRT && HeaterTemperatureL < ActiveSetpoint) {
            HeaterStatusL = 0x7f;
            HEATER_L = 1;
        } else {
            HeaterStatusL = 0xff;
            HEATER_L = 0;
        }

        KtyCycles++;
        if (KtyCycles == 16) {
            KtyReading = ktyLookup(&KtySum);
            KtySum = 0;
            if (KtyReading > 130) // No KTY sensor or bad contact
                KtyReading = 30; // use some default value
            KtyFilteringSum += KtyReading;
            KtyFilteringCycles++;
            if (KtyFilteringCycles == 16) {
                KtyFilteringCycles = 0;
                KtyTemperature = KtyFilteringSum / 16;
                KtyFilteringSum = 0;
            }
            KtyCycles = 0;
        }

        if (CalculateTemperature) { // takes about 1.5ms
            AdcSumL = (AdcSumL + AdcSumL2) / 2;
            AdcSumR = (AdcSumR + AdcSumR2) / 2;
            HeaterTemperatureL = tcLookup(&AdcSumL);
            HeaterTemperatureR = tcLookup(&AdcSumR);
            CalculateTemperature = 0;
        }

        if (State == ST_MAIN && (SaveSetpoint || SaveAllSettings)) // do not save setpoint if already gone to stby
        {
            if (SaveAllSettings) {
                saveParms();
                SaveAllSettings = 0;
            } else {
                EepromWrite16(0, (uint_fast16_t) SolderSetpoint);
            }

            SaveSetpoint = 0;
        }

        // some filtering to displayed temperature to reduce flickering
        // old temperature is 75% of new one
        if (TipType == TYPE_WMRT) {
            DisplayTemperature *= 6;
            DisplayTemperature += HeaterTemperatureL;
            DisplayTemperature += HeaterTemperatureR;
            DisplayTemperature /= 8;
        }
        else {
            DisplayTemperature *= 3;
            DisplayTemperature += HeaterTemperatureR;
            DisplayTemperature /= 4;
        }
        
        // If temperature is within 3 degrees from target, just show target temperature
        if ((DisplayTemperature - ActiveSetpoint < 3) && (ActiveSetpoint - DisplayTemperature < 3)) {
            DisplayTemperature = ActiveSetpoint;
            
#ifdef BUZZ_MOD
            if (WaitingForTemperature) {
                WaitingForTemperature = 0;
                BuzzerCycles = 300; // 250ms
                BUZZER = 1;
            }
#endif
        }
#ifdef BUZZ_MOD 
        else {
            if (SetpointDelay > 0 && SetpointDelay <= 600)
                WaitingForTemperature = 1;
        }

        if (BuzzerCycles <= 0) {
            BUZZER = 0;
        }
#endif

        if (ReedSum < 312) {
            if (PoorMode) {
                TipType = TYPE_WMRP;
                ReedStatus = REED_OPEN;
            } else if (TipType == TYPE_NC) {
                ReadTip = 1;
                ReedStatus = REED_CLOSED;
            } else
                ReedStatus = REED_CLOSED;
        } else if (ReedSum > 1798 && ReedSum < 2298) {
            TipType = TYPE_WMRP;
            ReedStatus = REED_OPEN;
        } else if (ReedSum > 2481 && ReedSum < 2981) {
            TipType = TYPE_WMRT;
            ReedStatus = REED_OPEN;
        } else if (ReedSum > 3750) {
            TipType = TYPE_NC;
            ReedStatus = REED_OPEN;
        }

        updateDisplay(); // updateDisplay call takes about 520 �s when in 'normal' mode (displaying temp)

        // wait here for heating cycle to stop
        while(DutyCycleOn);
    } // end while
}

void showSevenSegNum(DISP_MODES mode, int_fast32_t num) {
    static int_fast8_t j; // Digit index
    static uint_fast16_t tmp16;

#ifdef DEBUG
    if (mode == DISP_HEX) {
        Display[0] = hexTable[(num >> 12) & 0x0F];
        Display[1] = hexTable[(num >> 8) & 0x0F];
        Display[2] = hexTable[(num >> 4) & 0x0F];
        Display[3] = hexTable[num & 0x0F];
        Display[4] = _SPACE;
        return;
    }
#endif

    switch (mode) {
        case DISP_C:
            Display[3] = _CELSIUS;
            Display[4] = _DEGREE;
            j = 2;
            break;
        case DISP_F:
            Display[3] = _F;
            Display[4] = _DEGREE;
            num = num * 9 / 5;
            if (State != ST_ADJ_OFFSET && State != ST_ADJ_STEP_SIZE)
                num = num + 32;
            if (num > 999)
                num = 999;
            j = 2;
            break;
        case DISP_NUM:
        case DISP_REF:
            Display[4] = _SPACE;
            j = 3;
            break;
        default:
            return;
    }

    tmp16 = (uint_fast16_t) labs(num);

    Display[j] = hexTable[tmp16 % 10];
    while (j > 0) {
        j--;
        if (j == 0 && mode == DISP_REF)
            Display[j] = (uint_fast8_t) (hexTable[(tmp16 / 10) % 10] _ADDPOINT);
        else if ((tmp16 / 10) == 0 && num >= 0)
            Display[j] = _SPACE;
        else if ((tmp16 / 10) == 0 && tmp16 != 0 && num < 0)
            Display[j] = _MINUS;
        else if (tmp16 == 0 && num < 0)
            Display[j] = _SPACE;
        else
            Display[j] = hexTable[(tmp16 / 10) % 10];
        tmp16 /= 10;
    }
}

inline void readAdc(int_fast32_t* adc, uint_fast8_t channel) {
    static int_fast32_t adcSum = 0;
    static int_fast8_t i = 0;
    
    adcSum = 0;
    
    ADCON0bits.CHS = channel;
    
    __delay_us(5);
    
    for (i = 0; i < 16; i++) { // sample 16 times to get a virtual 16 bit reading..
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO_nDONE);
        
        adcSum += ((((int_fast16_t)ADRESH) << 8) | (int_fast16_t)ADRESL);
    }

    *adc = adcSum;
}

// <editor-fold defaultstate="collapsed" desc="Lookups">

int_fast32_t temp1 = 0;
#ifndef POLY_LOOKUP
int_fast32_t temp2 = 0;
int_fast32_t ylempi = 0;
int_fast32_t alempi = 0;

int_fast8_t j = 0;
#endif

int_fast16_t tcLookup(int_fast32_t* adcSum) {
    static int_fast32_t sum32 = 0;
    // sum32 = (uint_fast32_t)*adcSum * (uint_fast32_t)reference / 2048;	// compensate for reference inaccuracy
    sum32 = (*adcSum * Reference) / 2048; // compensate for reference inaccuracy
#ifndef POLY_LOOKUP
    for (j = 0; j < 50; j++) {
        if (sum32 < TcLookupData[j])
            break;
    }
    ylempi = TcLookupData[j];
    alempi = TcLookupData[j - 1];
    temp1 = 10 * (alempi - sum32);
    temp2 = (alempi - ylempi);

    return ((int_fast16_t) (temp1 / temp2 + 10 * (j - 1))) + KtyTemperature - TemperatureOffset; // -1 because tc starts from zero. subtract offset setting
#else
    if (sum32 <= 0) {
        temp1 = 0;
    } else if (sum32 > 65535) {
        temp1 = 525;
    } else {
        // Polynomial of 3rd degree. Result calculated with Horner method.
        // Fewer multiplications and better control over number ranges
        
        temp1 = (TC_POLY_COEFF_3 * sum32) + TC_POLY_COEFF_2;
        temp1 = ((temp1 - 5000) / 10000); // division by 10000 with rounding, number is always negative
        temp1 = temp1 * sum32;
        temp1 = ((temp1 - 5) / 10); // division by 10 with rounding, number is always negative
        temp1 = temp1 + TC_POLY_COEFF_1;
        temp1 = ((temp1 + 50000) / 100000); // division by 100000 with rounding, number is always positive
        temp1 = (temp1 * sum32) + TC_POLY_COEFF_0;
        temp1 = ((temp1 + 500000) / 1000000); // division by 1000000 with rounding, number is always positive
    }

    return ((int_fast16_t) temp1) + KtyTemperature - TemperatureOffset;
#endif
}

int_fast16_t ktyLookup(int_fast32_t* adcSum) {
#ifndef POLY_LOOKUP
    for (j = 0; j < 21; j++) {
        if (*adcSum < KtyLookupData[j])
            break;
    }

    ylempi = KtyLookupData[j];
    alempi = KtyLookupData[j - 1];
    temp1 = 10 * (alempi - *adcSum);
    temp2 = (alempi - ylempi);
    return (int_fast16_t) (temp1 / temp2 + 10 * (j - 6)); // -6 because kty_lookup begins from -50
#else
    if ((*adcSum) > 43800)
        return 155;
    else if ((*adcSum) < 19800)
        return -60;

    // Polynomial of 4th degree. Result calculated with Horner method.
    // Fewer multiplications and better control over number ranges

    temp1 = (KTY_POLY_COEFF_4 * *adcSum) + KTY_POLY_COEFF_3;
    temp1 = ((temp1 - 5000) / 10000); // division by 10000 with rounding, number is always negative
    temp1 = (temp1 * *adcSum) + KTY_POLY_COEFF_2;
    temp1 = ((temp1 + 50000) / 100000); // division by 100000 with rounding, number is always positive
    temp1 = (temp1 * *adcSum) + KTY_POLY_COEFF_1;
    temp1 = ((temp1 - (temp1 > 0 ? 5000 : -5000)) / 10000); // division by 10000 with rounding, number can be positive or negative
    temp1 = (temp1 * *adcSum) + KTY_POLY_COEFF_0;
    temp1 = ((temp1 - (temp1 > 0 ? 500000 : -500000)) / 1000000); // division by 1000000 with rounding, number can be positive or negative

    return (int_fast16_t) temp1;
#endif
}

// </editor-fold>

inline void menuMachine(void) // Takes now 3..4us when display updates moved to separate function
{
    if (State == ST_SETBACK) {
        if (SetbackSetpoint < SolderSetpoint)
            ActiveSetpoint = SetbackSetpoint;
        //			if (ReedStatus == REED_OPEN && PreviousReedStatus == REED_CLOSED)
        // Auto wake when reed contact opens or when duty cycle larger then expected (soldering attempt)
        if ((IdleDuty != 0 && (HeaterDutyR > IdleDuty || HeaterDutyL > IdleDuty)) || (ReedStatus == REED_OPEN && PreviousReedStatus == REED_CLOSED))
            Event = EVT_BUTTON_SHORT;
        PreviousReedStatus = ReedStatus;
    } else if (State == ST_STANDBY) {
        ActiveSetpoint = 0;
        // Auto wake when reed contact opens 
        if (ReedStatus == REED_OPEN && PreviousReedStatus == REED_CLOSED)
            //			if (HeaterDutyR > IdleDuty || HeaterDutyL > IdleDuty || (ReedStatus == REED_OPEN && PreviousReedStatus == REED_CLOSED))
            Event = EVT_BUTTON_SHORT;
        PreviousReedStatus = ReedStatus;
    }

    if (Event == EVT_LEFT || Event == EVT_RIGHT) {
        // <editor-fold defaultstate="collapsed" desc="Encoder left/right">

        if (State >= ST_MENU_MAIN && State <= ST_MENU_DIAGNOSTICS) {
            // Main menu cycle
            if (Event == EVT_LEFT) {
                if (State >= ST_MENU_DIAGNOSTICS)
                    State = ST_MENU_MAIN;
                else
                    State++;
            } else {
                if (State <= ST_MENU_MAIN)
                    State = ST_MENU_DIAGNOSTICS;
                else
                    State--;
            }
        } else if (State >= ST_MENU_BACK_FROM_DIAGNOSTICS && State <= ST_MENU_FW_VERSION) {
            // Diagnostic menu cycle
            if (Event == EVT_LEFT) {
                if (State >= ST_MENU_FW_VERSION)
                    State = ST_MENU_BACK_FROM_DIAGNOSTICS;
                else
                    State++;
            } else {
                if (State <= ST_MENU_BACK_FROM_DIAGNOSTICS)
                    State = ST_MENU_FW_VERSION;
                else
                    State--;
            }
        } else {
            // Adjust values
            switch (State) {
                case ST_MAIN:
                    if (Event == EVT_LEFT) {
                        SolderSetpoint -= Stepsize;
                        if (SolderSetpoint < MIN_SETPOINT)
                            SolderSetpoint = MIN_SETPOINT;
                    } else {
                        SolderSetpoint += Stepsize;
                        if (SolderSetpoint > MAX_SETPOINT)
                            SolderSetpoint = MAX_SETPOINT;
                    }
                    ActiveSetpoint = SolderSetpoint;
                    SetpointDelay = 1202;
                    break;
                case ST_ADJ_SETBACK:
                    if (Event == EVT_LEFT) {
                        if (SetbackSetpoint > MIN_SETPOINT)
                            SetbackSetpoint -= 10;
                    } else {
                        if (SetbackSetpoint < MAX_SETPOINT)
                            SetbackSetpoint += 10;
                    }
                    break;
                case ST_ADJ_DELAY:
                    if (Event == EVT_LEFT) {
                        if (SetbackDelay > 0)
                            SetbackDelay--;
                    } else {
                        if (SetbackDelay < MAX_SETBACK_DELAY + 1)
                            SetbackDelay++;
                    }
                    break;
                case ST_ADJ_POWEROFF:
                    if (Event == EVT_LEFT) {
                        if (PoweroffDelay > 0)
                            PoweroffDelay -= 10;
                    } else {
                        PoweroffDelay += 10;
                        if (PoweroffDelay > MAX_POWEROFF_DELAY)
                            PoweroffDelay = MAX_POWEROFF_DELAY + 10;
                    }
                    break;
                case ST_ADJ_OFFSET:
                    if (Event == EVT_LEFT) {
                        if (TemperatureOffset > -(MAX_TEMPERATURE_OFFSET))
                            TemperatureOffset--;
                    } else {
                        if (TemperatureOffset < MAX_TEMPERATURE_OFFSET)
                            TemperatureOffset++;
                    }
                    break;
                case ST_ADJ_UNIT:
                    if (TemperatureUnit == DISP_C)
                        TemperatureUnit = DISP_F;
                    else
                        TemperatureUnit = DISP_C;
                    break;
                case ST_ADJ_STEP_SIZE:
                    if (Stepsize == DEFAULT_STEPSIZE)
                        Stepsize = 1;
                    else
                        Stepsize = DEFAULT_STEPSIZE;
                    break;
                case ST_ADJ_REFERENCE:
                    if (Event == EVT_LEFT) {
                        if (Reference > 2048 - MAX_REFERENCE_TOLERANCE)
                            Reference--;
                    } else {
                        if (Reference < 2048 + MAX_REFERENCE_TOLERANCE)
                            Reference++;
                    }
                    break;
                case ST_ADJ_IDLE_DC:
                    if (Event == EVT_LEFT) {
                        if (IdleDuty > 0)
                            IdleDuty--;
                    } else {
                        if (IdleDuty < MAX_IDLEDUTY)
                            IdleDuty++;
                    }
                    break;
                case ST_ADJ_DC_LIMIT:
                    if (Event == EVT_LEFT) {
                        if (MaxDuty < MAX_MAXDUTY)
                            MaxDuty++;
                    } else {
                        if (MaxDuty > MIN_MAXDUTY)
                            MaxDuty--;
                    }
                    break;
                case ST_ADJ_POOR:
                    PoorMode = !PoorMode;
                    break;
                case ST_ADJ_FREQUENCY:
                    if (MainsFrequency == 50)
                        MainsFrequency = 60;
                    else
                        MainsFrequency = 50;
                    break;
                default:
                    break;
            }
        }

        // </editor-fold>
    } else if (Event == EVT_BUTTON_SHORT) {
        // <editor-fold defaultstate="collapsed" desc="Short button press">

        if (State >= ST_MENU_SETBACK && State <= ST_MENU_DIAGNOSTICS) {
            // Enter adjust/sub menu
            State = ST_ADJ_SETBACK + (State - ST_MENU_SETBACK);
        } else if (State >= ST_MENU_COLD_COMPENSATION && State <= ST_MENU_FW_VERSION) {
            // Enter diagnostic adjust/sub menu
            State = ST_SHOW_COLD_COMPENSATION + (State - ST_MENU_COLD_COMPENSATION);
        } else if (State >= ST_ADJ_SETBACK && State <= ST_MENU_BACK_FROM_DIAGNOSTICS) {
            // Return from adjust/sub menu
            if (State != ST_MENU_BACK_FROM_DIAGNOSTICS)
                SaveAllSettings = 1;

            State = ST_MENU_SETBACK + (State - ST_ADJ_SETBACK);
        } else if (State >= ST_SHOW_COLD_COMPENSATION && State <= ST_SHOW_FW_VERSION) {
            // Return from diagnostic adjust/sub menu
            if (State == ST_ADJ_REFERENCE || (State >= ST_ADJ_IDLE_DC && State <= ST_ADJ_FREQUENCY))
                SaveAllSettings = 1;

            State = ST_MENU_COLD_COMPENSATION + (State - ST_SHOW_COLD_COMPENSATION);
        } else if (State == ST_MENU_MAIN) {
            State = ST_MAIN;
        } else if (State == ST_MAIN) {
            State = ST_STANDBY;
        } else if (State == ST_STANDBY || State == ST_SETBACK) {
            State = ST_MAIN;
            ActiveSetpoint = SolderSetpoint;
            IdleMilliseconds = 0;
            IdleMinutes = 0;
            if (State == ST_SETBACK && SetbackDelay == 0) // workaround to briefly activate ST_MAIN if setback is zero
                IdleMilliseconds = 4294961286; // this number will overflow in 5 seconds
        }

        // </editor-fold>
    } else if (Event == EVT_BUTTON_LONG) {
        // <editor-fold defaultstate="collapsed" desc="Long button press">

        if (State == ST_MAIN) {
            State = ST_MENU_MAIN;
#ifdef BUZZ_MOD
            BuzzerCycles = 120; // 100ms
            BUZZER = 1;
#endif
        } else if (State == ST_STANDBY || State == ST_SETBACK) {
            State = ST_MENU_MAIN;
            ActiveSetpoint = SolderSetpoint;
#ifdef BUZZ_MOD
            BuzzerCycles = 120; // 100ms
            BUZZER = 1;
#endif
        }

        // </editor-fold>
    }

    Event = EVT_NONE;
}

inline void updateDisplay(void) // takes 520 us when showing iron temp, 400us when KTY temp, 680us when reference, 500us when number
{
    switch (State) {
        case ST_MAIN:
            if (TipType == TYPE_NC)
                memcpy(Display, nc, sizeof (Display));
            else if (SetpointDelay)
                showSevenSegNum(TemperatureUnit, SolderSetpoint);
            else
                showSevenSegNum(TemperatureUnit, DisplayTemperature);
            break;
        case ST_STANDBY:
            memcpy(Display, stby, sizeof (Display));
            break;
        case ST_SETBACK:
            if (TipType == TYPE_NC)
                memcpy(Display, nc, sizeof (Display));
            else
                showSevenSegNum(TemperatureUnit, DisplayTemperature);
            break;
        case ST_MENU_MAIN:
            memcpy(Display, bacc, sizeof (Display));
            break;
        case ST_MENU_SETBACK:
            memcpy(Display, setb, sizeof (Display));
            break;
        case ST_ADJ_SETBACK:
            showSevenSegNum(TemperatureUnit, SetbackSetpoint);
            break;
        case ST_MENU_DELAY:
            memcpy(Display, dela, sizeof (Display));
            break;
        case ST_ADJ_DELAY:
            if (SetbackDelay == 31)
                memcpy(Display, off, sizeof (Display));
            else
                showSevenSegNum(DISP_NUM, SetbackDelay);
            break;
        case ST_MENU_POWEROFF:
            memcpy(Display, poff, sizeof (Display));
            break;
        case ST_ADJ_POWEROFF:
            if (PoweroffDelay > 120)
                memcpy(Display, off, sizeof (Display));
            else
                showSevenSegNum(DISP_NUM, PoweroffDelay);
            break;
        case ST_MENU_OFFSET:
            memcpy(Display, ofse, sizeof (Display));
            break;
        case ST_ADJ_OFFSET:
            showSevenSegNum(TemperatureUnit, TemperatureOffset);
            break;
        case ST_MENU_UNIT:
            memcpy(Display, unit, sizeof (Display));
            break;
        case ST_ADJ_UNIT:
            showSevenSegNum(TemperatureUnit, 0);
            break;
        case ST_MENU_STEP_SIZE:
            memcpy(Display, step, sizeof (Display));
            break;
        case ST_ADJ_STEP_SIZE:
            showSevenSegNum(TemperatureUnit, Stepsize);
            break;
        case ST_MENU_DIAGNOSTICS:
            memcpy(Display, diag, sizeof (Display));
            break;
        case ST_MENU_BACK_FROM_DIAGNOSTICS:
            memcpy(Display, bacc, sizeof (Display));
            break;
        case ST_MENU_COLD_COMPENSATION:
            memcpy(Display, cold, sizeof (Display));
            break;
        case ST_SHOW_COLD_COMPENSATION:
            showSevenSegNum(TemperatureUnit, KtyTemperature);
            break;
        case ST_MENU_REFERENCE:
            memcpy(Display, ref, sizeof (Display));
            break;
        case ST_ADJ_REFERENCE:
            showSevenSegNum(DISP_REF, Reference);
            break;
        case ST_MENU_TIP_TYPE:
            memcpy(Display, type, sizeof (Display));
            break;
        case ST_SHOW_TIP_TYPE:
            switch (TipType) {
                case TYPE_WMRP:
                    memcpy(Display, wmrp, sizeof (Display));
                    break;
                case TYPE_WMRT:
                    memcpy(Display, wmrt, sizeof (Display));
                    break;
                default:
                    memcpy(Display, nc, sizeof (Display));
                    break;
            }
            break;
        case ST_MENU_REED_STATE:
            memcpy(Display, reed, sizeof (Display));
            break;
        case ST_SHOW_REED_STATE:
            switch (ReedStatus) {
                case REED_OPEN:
                    memcpy(Display, open, sizeof (Display));
                    break;
                case REED_CLOSED:
                    memcpy(Display, clos, sizeof (Display));
                    break;
            }
            break;
        case ST_MENU_TC_R_READING:
            memcpy(Display, tc_R, sizeof (Display));
            break;
        case ST_SHOW_TC_R_READING:
            showSevenSegNum(TemperatureUnit, HeaterTemperatureR);
            break;
        case ST_MENU_TC_L_READING:
            memcpy(Display, tc_L, sizeof (Display));
            break;
        case ST_SHOW_TC_L_READING:
            showSevenSegNum(TemperatureUnit, HeaterTemperatureL);
            break;
        case ST_MENU_PWM_R_READING:
            memcpy(Display, pwmR, sizeof (Display));
            break;
        case ST_SHOW_PWM_R_READING:
            showSevenSegNum(DISP_NUM, HeaterDutyR);
            break;
        case ST_MENU_PWM_L_READING:
            memcpy(Display, pwmL, sizeof (Display));
            break;
        case ST_SHOW_PWM_L_READING:
            showSevenSegNum(DISP_NUM, HeaterDutyL);
            break;
        case ST_MENU_IDLE_DC:
            memcpy(Display, idle, sizeof (Display));
            break;
        case ST_ADJ_IDLE_DC:
            if (IdleDuty == 0)
                memcpy(Display, off, sizeof (Display));
            else
                showSevenSegNum(DISP_NUM, IdleDuty);
            break;
        case ST_MENU_DC_LIMIT:
            memcpy(Display, dcli, sizeof (Display));
            break;
        case ST_ADJ_DC_LIMIT:
            if (MainsFrequency == 50)
                //				show_7seg_num(DISP_NUM, 100-maxDuty*8);		// approximation with int_fast8_t arithmetics
                showSevenSegNum(DISP_NUM, (1200 - (uint_fast16_t) (MaxDuty)*100) / 12);
            else
                showSevenSegNum(DISP_NUM, 100 - MaxDuty * 10);
            break;
        case ST_MENU_POOR:
            memcpy(Display, poor, sizeof (Display));
            break;
        case ST_ADJ_POOR:
            if (PoorMode)
                memcpy(Display, on, sizeof (Display));
            else
                memcpy(Display, off, sizeof (Display));
            break;
        case ST_MENU_FREQUENCY:
            memcpy(Display, freq, sizeof (Display));
            break;
        case ST_ADJ_FREQUENCY:
            showSevenSegNum(DISP_NUM, MainsFrequency);
            break;
        case ST_MENU_FW_VERSION:
            memcpy(Display, vers, sizeof (Display));
            break;
        case ST_SHOW_FW_VERSION:
            showSevenSegNum(DISP_REF, (int_fast16_t) FW_VERSION);
            break;
    } // end display case
}

void recognizeTypeInStand(void) {
    static int_fast16_t reading = 0;
    
    // Tip type recognition by pulsing current to heater2 = WMRT left heater. If drives TC2 to rail, must be WMRT
    // If the iron is on stand, cannot recognize based on resistor in parallel with reed because reed is closed.
    // Since FW 0.8 this is only used when necessary to recognize if WMRT is connected. Older FW used this method
    // continuously which caused continuous buzz noise from WMRT tip. The buzz comes from the heating elements of
    // WMRT when power is switched on and off rapidly. Original Weller stations don't make the noise because they
    // drive the elements with AC
    
    ADCON0bits.CHS = 1;
    HEATER_L = 1;
    
    __delay_us(15);

    reading = AdcRead(1);
    
    HEATER_L = 0;
    
    if (reading > 4000)
        TipType = TYPE_WMRT;
}

void saveParms(void) {
    HEATER_R = 0; // Turn heaters off before saving parameters - interrupts are disabled
    HEATER_L = 0; // during EEPROM write and we don't want something bad to happen if write jams
    LATA = _SPACE; // clear anodes to eliminate bright flash of currently active segments
    
    EepromWrite16(0, (uint_fast16_t) SolderSetpoint);
    EepromWrite16(2, (uint_fast16_t) SetbackSetpoint);
    EepromWrite8(4, SetbackDelay);
    EepromWrite8(5, PoweroffDelay);
    EepromWrite8(6, (uint_fast8_t) TemperatureOffset);
    EepromWrite16(7, (uint_fast16_t) Reference);
    EepromWrite8(9, Stepsize);
    EepromWrite8(10, TemperatureUnit);
    EepromWrite8(11, PoorMode);
    EepromWrite8(12, MainsFrequency);
    EepromWrite8(13, MaxDuty);
    EepromWrite8(14, IdleDuty);
}

// <editor-fold defaultstate="collapsed" desc="EEPROM R/W">

uint_fast8_t EepromRead8(uint_fast16_t address) {
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;

    EEADRL = address & 0xFF;
    EEADRH = (address << 8) & 0xFF;

    EECON1bits.RD = 1;

    return EEDATA;
}

uint_fast16_t EepromRead16(uint_fast16_t address) {
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;

    EEADRL = address & 0xFF;
    EEADRH = (address >> 8) & 0xFF;

    EECON1bits.RD = 1;

    uint_fast16_t lsb = EEDATA;
    address++;

    EEADRL = address & 0xFF;
    EEADRH = (address >> 8) & 0xFF;

    EECON1bits.RD = 1;

    return lsb | (uint_fast16_t) (EEDATA << 8);
}

void EepromWrite8(uint_fast16_t address, uint_fast8_t data) {
    INTCONbits.GIE = 0;

    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;

    EEADRL = address & 0xFF;
    EEADRH = (address >> 8) & 0xFF;

    EEDATA = data;

    EECON1bits.WREN = 1;

    EECON2 = 0x55;
    EECON2 = 0xAA;

    EECON1bits.WR = 1;

    while (EECON1bits.WR == 1);
    EECON1bits.WREN = 0;

    INTCONbits.GIE = 1;
}

void EepromWrite16(uint_fast16_t address, uint_fast16_t data) {
    INTCONbits.GIE = 0;

    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;

    EEADRL = address & 0xFF;
    EEADRH = (address >> 8) & 0xFF;

    EEDATA = data & 0xFF;

    EECON1bits.WREN = 1;

    EECON2 = 0x55;
    EECON2 = 0xAA;

    EECON1bits.WR = 1;

    while (EECON1bits.WR == 1);
    EECON1bits.WREN = 0;

    address++;

    EEADRL = address & 0xFF;
    EEADRH = (address >> 8) & 0xFF;

    EEDATA = (data >> 8) & 0xFF;

    EECON1bits.WREN = 1;

    EECON2 = 0x55;
    EECON2 = 0xAA;

    EECON1bits.WR = 1;

    while (EECON1bits.WR == 1);
    EECON1bits.WREN = 0;

    INTCONbits.GIE = 1;
}

// </editor-fold>

inline int_fast16_t AdcRead(uint_fast8_t channel) { 
    ADCON0bits.CHS = channel;
    ADCON2bits.CHSN = 0b1111;
    
    __delay_us(5);
    
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO_nDONE);

    return (((int_fast16_t)ADRESH) << 8) | (int_fast16_t)ADRESL;
}

inline void init(void) {
    // Some things reverse engineered from CCS compiled binary
    
    // #use delay(int=16MHz, clock=32MHz)
    OSCCON = 0b11110000;
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    
    CM1CON1 = 0;
    CM1CON0 = 0;
    
    // setup_timer_2(T2_DIV_BY_16,208,2); // set timer2 to 832 �s (prescaler 16 and postscaler 2, period 208)
    T2CON = 0b00001110;
    PR2 = 208;
    
    // enable_interrupts(INT_TIMER2);
    PIE1bits.TMR2IE = 1;
    
    // enable_interrupts(GLOBAL);
    INTCON = 0b11000000;
    
    TRISA = 0b00000111;
#ifndef BUZZ_MOD
    TRISB = 0b11001111;
#else
    TRISB = 0b11001101;
#endif
    TRISC = 0b00000000;
    TRISEbits.TRISE3 = 1;

    PORTA = 0;
    PORTB = 0b11000000;
    
    WPUA = 0b00000000;
    WPUB = 0b11000000;
    WPUEbits.WPUE3 = 1;

    OPTION_REGbits.nWPUEN = 0;
 
    HEATER_R = 0; // Heater 1 (right) off
    HEATER_L = 0; // Heater 2 (left) off
#ifdef BUZZ_MOD
    BUZZER = 0;
#endif
    
    // setup_adc_ports( sAN0|sAN1|sAN2|sAN9|sAN12,VSS_FVR);	// AN9 = reed pullup
    // setup_adc(ADC_CLOCK_DIV_32);	// Gives 1�s conversion clock cycle time (fastest recommended)
    // #device ADC=12;
    
    ADCON1bits.ADPREF = 0b11;
    ADCON1bits.ADNREF = 0;
    
    ANSELA = 0b00000111;
    ANSELB = 0b00001001;
    
    //setup_vref(VREF_ON | VREF_ADC_2v048 | VREF_COMP_DAC_2v048);
    FVRCON = 0b10001010;

#ifndef BUZZ_MOD
    // setup_opamp2(OPAMP_DISABLED);		// To disable KTY pullup
    OPA2CON = 0;
#endif
    
    // setup_dac(DAC_OFF);
    DAC1EN = 0;
    
    ADCON1bits.ADCS = 0b010;
    ADCON1bits.ADFM = 1;
    
    ADCON0bits.ADON = 1;
    
    ADCON1 = 0b10100011;
    
    ADCON1bits.ADCS = 0b010;
}