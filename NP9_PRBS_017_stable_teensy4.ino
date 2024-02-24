/*

          Rudi's Electronics Lab - NP9 generator"

This projects generates clean PRBS NP9 codes, as well as NP9 codes with predetermined) errors. It is intended to be generate input signals for BER testing like on the Anritsu MG3700A. The NP9 codes are identical to those made by the Rohde & Schwarz SMIQ vector signal generator. 

This sketch  is written to run on an Teensy 4.0 microcontroller. It may also run on various Arduino boards. Note however that: 
(1) Arduino boards are (currently) much, much slower than the Teensy 4.0 (which as an ARM Cortelx-M7 at 700 MHz). So, for Arduno boards, the max clock speed will be much lower. 
(2) Because of the twodimensional arrays, the sketch requires approx. 90kB of  dynamic memory (SRAM). An Arduino UNO R3, for instance, has only 2kB SRAM so does not have enough memory, and an Arduino UNO R4 WiFi has 32kB SRAM which is 
also too little to run this code. 
(3) For Arduino's, the INPUT_PULLUP needs to be replaced by "INPUT" and a fysical pullup resistors will need to be added. 

Other hardware requited: 
- An SSD1306 OLED display, pin connections shown below 
- An 74LS273 octal D-type flipflop for clocking the final signal out
- A two-pole momentary switches (or who single momentary switches) for the UI, connected to 3.3V (NOT 5V)  
- A clock generator providing a clock of up to 1.2MHz at TTL level
- A LED with a appropriate series resitor to be driven by a Teensy output pin. Make sure the LED does not soncume more than
the pin can deliver. I use a Broadcom HLMP-4700, a red 5mm led that is very bright and only draws 2mA at 1.7V. This LED requires a 560 Ohm series resistor 
to be used with the Teensy 3.3V outputs. 

The DEBUG mode (scenario 17) activates additional debug lines to see the wait loops and write confirmation. This is useful to 
visualize the working of the sketch on an osciloscope (with enough logic channels). Note however that the additional pin
writes in the DEBUG mode take extra time, and the maximum input clock speed reduces from 1.3MHz to 800 kHz. 

Firmware revisions: 
FW 0.17 DEBUG scenario now has Sequence 4 data
FW 0.16 Inverses all outputs so they can drive a HEX inverter (74LS04). 
FW 0.15 Uses subroutine for display update, has SO SYNC warning.
FW 0.12 PN code table replaced with SMIQ values. It now works on Anritsu! 
FW 0.11: Stable version. Now repaces HEX inverter (74LS04) by clocked output via 74LS273. Many other improvements. Needs external clock. Turned out NOT to work on Anritsu
FW 0.10: Now written for Teensy 4.0. Implements a toggle switch as I found out that analogread slowed down the sketch a lot.   
FW 0.05: Inverses all outputs so they can drive a HEX inverter (74LS04)
FW 0.04: First fully working version, for Arduino RP2040.  

*/

char FirmwareVersion[] = "0.17"; // this is later used to show in display

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

int OUTPIN_NP9 = 2;        // Selects pin for CORRECT NP9 out 
int OUTPIN_MOD = 3;        // Selects pin for MODIFIED NP9 out
int OUTPIN_FRAME_OUT = 4;  // Selects pin for Frame out
int OUTPIN_ERROR = 6;      // Selects pin for ERR out

int OUTPIN_DEBUG_LOOP1 = 16;      // Selects pin for debug wait loop 1
int OUTPIN_DEBUG_LOOP2 = 17;      // elects pin for debug wait loop 2
int OUTPIN_DEBUG_WRITEFINISHED = 12;  // Selects pin for debug write finished flag
int OUTPIN_ERROR_LED = 11;        // Selects pin for ERROR LED 

int INPIN_CLKIN = 8;              // Selects the digital input pin for external clock (must be 3.3V logic)
int INPIN_BUTTONUP = 9;    // Select digital input pin for the UP button 
int INPIN_BUTTONDOWN = 10;  // Select digital input pin for the DOWN button

int inVert = 1;             // when set to 1, all data outputs are reversed (used when fysical inverting port is used)

int ButtonUp = 1;
int ButtonDown = 1;

int Scenario = 4; // Initial value for scenario = running mode
int PrevScenario = 4; 

int LoopCount = 0; // If LoopCount gets to high, there is apparently no incoming SYC signal

// 128x64 SSD1306 OLED PIN CONNECTIONS: SCL=A5, SDA=A4 
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

/* The below array has the folowing bit squences (created by "PLanning PN sequences for Arduino 02.XLS")
00. Clean MP9 (from Rohde & Schwarz SMIQ03B)
01. NP9 with 1 error
02. NP9 with 2 errors
03. NP9 with 3 errors
04 NP9 with 5 errors
05. NP9 with 6 errors
06. NP9 with 7 errors
07. NP8 with 8 errors within 64 bits
08. NP9 with 9 errors
09. NP9 with 12 errors
20. NP9 with 25 errors
11. NP9 with 50 errors
12. All 1
13. All 0
14. 1010101
15. Frame sync (100000)
*/ 

void printArray ( const int [][ 3 ] ); // prototype
const int rows = 16;
const int columns = 511;
const  bool  np9[ rows ][ columns ]  PROGMEM  = { 
{0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{0,0,1,1,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,1,0,1,0},
{0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,0,0,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,1,1,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,0,0,1,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{0,0,1,0,0,0,0,0,0,0,0,1,0,1,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,1,0,1,1,0,1,1,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,1,0,1,1,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,1,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,1,1,0,1,1,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,1,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,0,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,0,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,1,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,1,1,0,0,1,1,1,0,1,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{1,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,1,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,0,0,1,1,0,0,1,0,1,1,0,0,1,1,1,0,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,1,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,0,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,0,1,0,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,1,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,0,0,1,0,0,1,1,1,1,1,1,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,0,0,1,0,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,0,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,1,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,1,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,1,1,0,1,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,0,1,1,1,1,1,0,0,0,1,0,1,1,1,1,0,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,0,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,0,0,1,0,1,1,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,0,0,1,0},
{0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0,1,0,0,1,1,1,1,0,1,0,1,0,1,0,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,0,0,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,1,1,0,1,0,0,1,1,1,1,1,0,0,1,1,0,0,1,0,0,1,0,0,1,0,1,1,0,1,1,1,1,1,1,0,0,1,1,0,1,1,0,1,0,1,0,0,1,1,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,0,1,0,0,0,1,0,1,1,0,0,0,0,1,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1,1,0,0,0,1,1,1,0,1,0,1,1,1,0,1,0,1,1,0,0,1,1,1,1,1,0,0,1,1,1,1,1,0,1,1,1,0,1,0,0,0,0,0,1,1,1,1,0,1,1,0,1,1,0,1,1,1,1,1,1,0,0,0,0,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,1,0,0,1,0,1,0,1,1,0,0,0,1,0,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0,1,0,1,0,1,0,0,1,0,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,0,1,0,0,1,0,0,1,1,0,1,0,0,0,0,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,1,0,1,1,1,1,0,1,1,1,1,1,0,0,1,1,0,1,1,1,0,1,1,1,0,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,0,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,1,1,0,1,1,1,0,0,1,1,0,0,1,1,0,0,0,1,0},
{0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0,1,0,1,0,1,0,1,1,0,0,0,0,1,1,0,0,1,1,1,0,1,0,1,1,1,0,1,1,1,0,0,1,0,0,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,1,1,0,0,1,0,0,1,0,0,1,0,0,1,0,1,0,0,1,1,1,1,1,1,0,0,1,0,0,1,1,0,1,0,1,0,0,1,0,1,0,1,1,0,0,0,0,0,0,0,1,1,0,0,1,1,1,0,0,1,0,0,0,0,0,1,1,1,1,0,0,1,0,0,1,1,1,0,1,1,0,1,0,1,0,1,1,1,1,0,1,0,1,1,1,0,1,1,1,1,0,0,1,0,1,1,0,1,1,0,1,1,0,0,1,1,1,0,1,1,0,1,1,1,0,1,0,0,0,0,1,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,1,1,1,1,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,1,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,1,0,1,0,0,1,1,0,0,1,0,0,1,0,0,1,1,1,1,0,1,0,0,0,1,0,1,1,1,0,0,0,1,0,0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,0,1,1,0,1,1,0,1,0,1,1,0,1,1,0,1,1,1,0,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,0,1,1,1,0,1,0,0,0,1,0,1,1,1,0,0,1,1,0,0,1,0,0,0,0,1,1,0,1,1,0,1,0,0,1,1,1,0,1,1,0,1,0,0,0,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,0,1,1,0,0,0,1,0,1,0,1,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,1,0,1,0,1,0,1,1,1,0,0,0,1,0,0,1,1,0,1,1,1,0},
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0},
{1,0}
};

// The below array has the information for the four subsequent frames for each 17 scenarios

const int frames[ 18 ][ 511 ] PROGMEM  = { 
{0, 0, 0, 0},         // scenario 1 NP9 without error
{1, 0, 0, 0},         // scenario 2 BER 0,049% with 1|0|0|0 errors
{1, 0, 1, 0},         // scenario 3 BER 0,098% with 1|0 errors
{1, 1, 1, 1},         // scenario 4 BER 0,196% with 1 error
{1, 1, 3, 1},         // scenario 5 BER 0,294% with 1|1|3|1 errors
{2, 2, 2, 2},         // scenario 6 BER 0,391% with 2 errors
{3, 1, 4, 2},         // scenario 7 BER 0,538% with 3|1|5|2 errors
{4, 4, 4, 4},         // scenario 8 BER 0,978% with 5 errors
{8, 1, 4, 5},         // scenario 9 BER 1,027% with 9|1|5|6 errors
{9, 9, 9, 9},         // scenarion 10 BER 2,348% with 12 errors
{10, 10, 10, 10},     // scenarion 11 BER 4,892% with 25 errors
{11, 11, 11, 11},     // scenarion 12 BER 9,785% with 50 errors
{7, 0, 0, 0},         // scenarion 13 BER 0,391 with 8 errors within 64 bits
{12, 12, 12, 12},     // scenarion 14 All 1 sequence
{13, 13, 13, 13},     // scenarion 15 All 0 sequence
{14, 14, 14, 14},     // scenarion 16 010101 sequence 
{1, 1, 1, 1},         // scenario 17 DEBUG (as Scenario 4)
{1, 1, 1, 1}          // scenarion (unnumbered 18) Info & Firmware Screen
};

void setup () {

 Serial.begin(115200);
 
 pinMode(OUTPIN_NP9, OUTPUT);
 pinMode(OUTPIN_MOD, OUTPUT);
 pinMode(OUTPIN_FRAME_OUT, OUTPUT);
 pinMode(OUTPIN_ERROR, OUTPUT);

 pinMode(OUTPIN_ERROR_LED, OUTPUT);   

 pinMode(OUTPIN_DEBUG_LOOP1, OUTPUT);  
 pinMode(OUTPIN_DEBUG_LOOP2, OUTPUT); 
 pinMode(OUTPIN_DEBUG_WRITEFINISHED, OUTPUT);  

 pinMode(INPIN_CLKIN, INPUT);      
 pinMode(INPIN_BUTTONUP, INPUT_PULLUP);   
 pinMode(INPIN_BUTTONDOWN, INPUT_PULLUP);   

 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, LOW);   // turn the board LED of at the start of sketch

 u8g2.begin(); // initializes display unit

WriteDisplay(18); // write welcome screen and firmware version
delay(2000);
Scenario = 4;
WriteDisplay(Scenario); // start device with Scenario 4

} // end setup loop

void loop () {

for ( int i = 0; i < 4; ++i ) {


int ModOut= (frames[Scenario-1] [i]);

  for ( int j = 0; j < columns; ++j ) {
      
      
      LoopCount = 0;

        while (digitalRead(INPIN_CLKIN)==0) { 
          if (Scenario==17) {  // IF only satisfied in scenario 17 DEBUG mode
            digitalWrite(OUTPIN_DEBUG_LOOP1,1); 
            digitalWrite(OUTPIN_DEBUG_LOOP1,0); 
           } // end scenario 17 loop
            
              ++ LoopCount; 
                  if (LoopCount==1000) {  // happens if there is no incoming SYNC, or a sync lower than 10kHz
                  WriteDisplay(19);
                  delay (1000);
                  while (digitalRead(INPIN_CLKIN)==0) { } // wait for at least one fresh clock cycle
                  while (digitalRead(INPIN_CLKIN)==1) { } 
                WriteDisplay(Scenario); // after error message, show regular scenario agian 
              } // end loop LoopCount==1000)                  
        } // wait until external clock is back to 1

        // Serial.println(LoopCount);

        while (digitalRead(INPIN_CLKIN)==1) { 
          if (Scenario==17) {  // IF only satisfied in scenario 17 DEBUG mode
          digitalWrite(OUTPIN_DEBUG_LOOP2,1); 
          digitalWrite(OUTPIN_DEBUG_LOOP2,0); 
        } // end scenario 17 loop
        } // wait until external clock is back to 0

      digitalWrite(OUTPIN_NP9, inVert^ (np9[ 0 ][ j ] ));       // writes correct NP9 to output OUTPIN_NP9
      digitalWrite(OUTPIN_MOD, inVert^ (np9[ ModOut ][ j ] ));  // writes Modified NP9 to output OUTPIN_MOD
      digitalWrite(OUTPIN_FRAME_OUT, inVert^ (np9[ 15 ][ j ] ));       // writes Frame Out to output OUTPIN_FRAME_OUT
      digitalWrite (OUTPIN_ERROR, (inVert^ np9[ 0 ][ j ] ) ^ (np9[ ModOut ][ j ] ) ); // writes Error Flag to output OUTPIN_ERROR
     

if (Scenario==17) { // IF only satisfied in scenario 17 DEBUG mode
  digitalWrite(OUTPIN_DEBUG_WRITEFINISHED,0); 
  digitalWrite(OUTPIN_DEBUG_WRITEFINISHED,1);
}

if (digitalRead(INPIN_CLKIN)==1){ //  ERROR LED loop
  digitalWrite(OUTPIN_ERROR_LED,1); // turn LED on in case of sync error 
  delay(1);
  digitalWrite(OUTPIN_ERROR_LED,0);

} // end ERROR LED loop
  
  } // end of j loop

} // end of i loop

ButtonUp=digitalRead(INPIN_BUTTONUP);
ButtonDown=digitalRead(INPIN_BUTTONDOWN);

if (ButtonUp==0) {
  ++Scenario;
  if (Scenario==19) {
    Scenario=18;
  }
} // end of the ButtonUp loop

if (ButtonDown==0) {
  --Scenario;
  if (Scenario==0) {
    Scenario=1;
  }
} // end of the ButtonDown loop

if (Scenario != PrevScenario) {

digitalWrite(LED_BUILTIN, HIGH); // this onboard LED indicates a user UI action is processed. 

WriteDisplay(Scenario); // write new selected scenario on display 

PrevScenario = Scenario;

// below code makes sure the UP switch is released before continuing
while (ButtonUp==0) { ButtonUp=digitalRead(INPIN_BUTTONUP); } 

// below code makes sure the DOWN switch is released before continuing
while (ButtonDown==0){ ButtonDown=digitalRead(INPIN_BUTTONDOWN); } 

digitalWrite(LED_BUILTIN, LOW);

} // end if (Scenario != PrevScenario) 

} // end void loop ()

void WriteDisplay(int ShowScreen) { // Subroutine that writes the screen 

u8g2.clear();

if (ShowScreen==1)  { 
  u8g2.firstPage();
  do {
   // u8g2.setFont(u8g2_font_6x10_tf);
   u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "1" );
   u8g2.drawStr(25,17,"BER 0.000%");
   u8g2.drawStr(5,45," Clean NP9");
   u8g2.drawStr(5,60," sequence");
   } while ( u8g2.nextPage() );
}

if (ShowScreen==2) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "2" );
    u8g2.drawStr(25,17,"BER 0.049%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame   1   2   3   4");
    u8g2.drawStr(0,57,"Errors  1   .   .   .");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); u8g2.drawLine(60, 30, 60, 60); u8g2.drawLine(84, 30, 84, 60); u8g2.drawLine(107, 30, 107, 60); // vertical lines 
  } while ( u8g2.nextPage() );
}

if (ShowScreen==3) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "3" );
    u8g2.drawStr(25,17,"BER 0.098%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame     1       2 ");
    u8g2.drawStr(0,57,"Errors    1       .");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    
    u8g2.drawLine(36, 30, 36, 60); u8g2.drawLine(84, 30, 84, 60);  // vertical lines 
  } while ( u8g2.nextPage() );
}

  if (ShowScreen==4) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "4" );
    u8g2.drawStr(25,17,"BER 0.196%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame         1");
    u8g2.drawStr(0,57,"Errors        1");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); // vertical lines 
  } while ( u8g2.nextPage() );
}

 if (ShowScreen==5) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "5" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(25,17,"BER 0.294%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame   1   2   3   4");
    u8g2.drawStr(0,57,"Errors  1   1   3   1");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); u8g2.drawLine(60, 30, 60, 60); u8g2.drawLine(84, 30, 84, 60); u8g2.drawLine(107, 30, 107, 60); // vertical lines 
  } while ( u8g2.nextPage() );
} 

 if (ShowScreen==6) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "6" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(25,17,"BER 0.391%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame         1");
    u8g2.drawStr(0,57,"Errors        2");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); // vertical lines 
  } while ( u8g2.nextPage() );
}

 if (ShowScreen==7) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "7" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(25,17,"BER 0.538%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame   1   2   3   4");
    u8g2.drawStr(0,57,"Errors  3   1   5   2");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); u8g2.drawLine(60, 30, 60, 60); u8g2.drawLine(84, 30, 84, 60); u8g2.drawLine(107, 30, 107, 60); // vertical lines 
  } while ( u8g2.nextPage() );
}

 if (ShowScreen==8) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "8" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(25,17,"BER 0.978%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame         1");
    u8g2.drawStr(0,57,"Errors        5");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); // vertical lines   } while ( u8g2.nextPage() );
  } while ( u8g2.nextPage() );
}

if (ShowScreen==9) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "9" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(25,17,"BER 1.027%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame   1   2   3   4");
    u8g2.drawStr(0,57,"Errors  9   1   5   6");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); u8g2.drawLine(60, 30, 60, 60); u8g2.drawLine(84, 30, 84, 60); u8g2.drawLine(107, 30, 107, 60); // vertical lines 
  } while ( u8g2.nextPage() );
}

if (ShowScreen==10) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "10" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(30,17,"BER 2.348%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame         1");
    u8g2.drawStr(0,57,"Errors        12");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); // vertical lines   } while ( u8g2.nextPage() );
  } while ( u8g2.nextPage() );
  }

if (ShowScreen==11) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "11" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(30,17,"BER 4.892%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame         1");
    u8g2.drawStr(0,57,"Errors        25");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); // vertical lines   } while ( u8g2.nextPage() );
  } while ( u8g2.nextPage() );
}

if (ShowScreen==12) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "12" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(30,17,"BER 9.785%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame         1");
    u8g2.drawStr(0,57,"Errors        50");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); // vertical lines   } while ( u8g2.nextPage() );
  } while ( u8g2.nextPage() );
}

if (ShowScreen==13) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "13" );
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawStr(30,17,"BER 0.391%");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,42,"Frame         1");
    u8g2.drawStr(0,57,"Errors  8 within 64 bits   ");
    u8g2.drawLine(0, 30, 128, 30); u8g2.drawLine(0, 45, 128, 45); u8g2.drawLine(0, 60, 128, 60); // horizontal lines
    u8g2.drawLine(36, 30, 36, 60); // vertical lines   } while ( u8g2.nextPage() );
  } while ( u8g2.nextPage() );
}

 if (ShowScreen==14) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "14" );
    u8g2.drawStr(30,17,"SEQUENCE");
    u8g2.drawStr(5,20,"     ");
   u8g2.drawStr(0,45," All 1 ");
   u8g2.drawStr(0,60," on MOD out");
  } while ( u8g2.nextPage() );
}

 if (ShowScreen==15) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "15" );
    u8g2.drawStr(30,17,"SEQUENCE");
    u8g2.drawStr(5,20,"     ");
   u8g2.drawStr(0,45," All 0 ");
   u8g2.drawStr(0,60," on MOD out");
  } while ( u8g2.nextPage() );
}

if (ShowScreen==16) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "16" );
    u8g2.drawStr(30,17,"SEQUENCE");
    u8g2.drawStr(5,20,"     ");
   u8g2.drawStr(0,45," 010101 pattern");
   u8g2.drawStr(0,60," on MOD out");
  } while ( u8g2.nextPage() );
}

 if (ShowScreen==17) { u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB12_tf);
    u8g2.drawButtonUTF8(5, 15, U8G2_BTN_INV, 0,  4,  3, "17" );
    u8g2.drawStr(5,20,"     ");
   u8g2.drawStr(30,17,"DEBUG");
u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0,40,"As Scenario 4,");
    u8g2.drawStr(0,50,"debug lines active,");
      u8g2.drawStr(0,60,"max clk. reduced");
  } while ( u8g2.nextPage() );
}

if (ShowScreen==18)  { 
  u8g2.firstPage();
  do {
   u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, " Rudi's Elect. Lab");  
        u8g2.setFont(u8g2_font_helvB12_tf);
        u8g2.drawStr(0, 30, "  PRBS NP9");  
        u8g2.drawStr(0, 45, "  generator");  
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 60, " Firmware");
        u8g2.drawStr(60, 60, FirmwareVersion);


   } while ( u8g2.nextPage() );
}

if (ShowScreen==19)  { 
  u8g2.firstPage();
  do {
              u8g2.setFont(u8g2_font_helvB12_tf);
              u8g2.drawStr(0, 20, "NO CLOCK");  
              u8g2.setFont(u8g2_font_6x10_tf);
              u8g2.drawStr(0, 30, "Please input clock");  
              u8g2.drawStr(0, 40, "signal between 10kHz "); 
              u8g2.drawStr(0, 50, "and 1.3 MHz."); 
  } while (u8g2.nextPage());
}

if (ShowScreen==20)  { 
  u8g2.firstPage();
  do {
              u8g2.setFont(u8g2_font_helvB12_tf);
              u8g2.drawStr(0, 20, "CLOCK OVL");  
              u8g2.setFont(u8g2_font_6x10_tf);
              u8g2.drawStr(0, 30, "Clock overlead!");  
              u8g2.drawStr(0, 40, "Please input clock "); 
              u8g2.drawStr(0, 50, "lower than 1.3 MHz."); 
  } while (u8g2.nextPage());
}

} // end of void WriteDisplay(int ShowScreen)
