# DIY-Weller-WMRP-WMRT
DIY soldering station firmware for WMRP and WMRT handles. Based on firmware - and build for the PCB - of Jaakko Kairus: http://kair.us/projects/weller/index.html

## Beeper/Buzzer modification
The beep/buzzer is used to indicate that the set temperature has been reached.
To use is it the PCB has the be modified. The trace between the MCU pin 22 and 23 has to be cut. Idealy this would be done before soldering the MCU, if the MCU is already soldered it has to be desolder to gain access to the trace. When the trace is cut the MCU can be soldered as usual. Afterwards the MCU pin 22 is used to drive a high active buzzer/beeper circuit (A simple 5V buzzer/beeper with a NPN/N-MOSFET). Once modified the PCB is no longer compatible with Jaakko Kairus firmware releases. The firmware provided here with the BUZZ_MOD flag activated has to be used.

## Polynomial temperature lookups
Uses a polynomial formular instead of using a lookup table to determine the thermocouple and cold junction temperatures.
Has constant runtime and should also deliver slightly better results. Is activated with the POLY_LOOKUP flag.