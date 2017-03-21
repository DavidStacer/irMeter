/* Copyright (c) 2017  David Stacer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 
 */

/*
 * File:   main.c
 * Author: David Stacer
 *
 * Created on February 2, 2017, 5:50 AM
 * Code converted to MPLAB X from the original source by Dennis Vollrath
 * https://www.rcgroups.com/forums/showthread.php?2809231-Battery-IR-Meter-Circuitboard%21
 */

// PIC18F25K80 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#pragma config PLLCFG = OFF     // PLL x4 Enable bit (Disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = SWDTDIS  // Watchdog Timer (WDT enabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN Mux bit (ECAN TX and RX pins are located on RB2 and RB3, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-01FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 02000-03FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 04000-05FFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 06000-07FFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-01FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 02000-03FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 04000-05FFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 06000-07FFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-01FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 02000-03FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 04000-05FFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 06000-07FFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)

#include "main.h"
#include <xc.h>
#include "eeprom.h"
#include <math.h>
#include <stdlib.h>

/*
BATTERY ir METER TEST UNIT RELAY SYSTEM VERSION II
RCGroups - vollrathd


This project applies two different resistor loads to the
battery under test.  Current is calculated by voltage drop
across two precision power resistors.

Project set up to run with the PIC18F25K80 PicChip

 *****************************************************************************
 * CAUTION! IN ORDER TO USE RC0 AND RC1, SET UP OSCILLATOR CONFIGURATION FOR *
 ***** DIGITAL (SCLK1) MODE IN CONFIGURATION AND SETUP FILES *****************
 **DEFAULT PORT A AND B ARE ANALOG INPUTS.  MUST ASSIGN DIGITAL IN A/D SETUP**
 * PicChip clock speed set to 16 Mhz with mikroP in order for LCD to keep up *
 *************** PicChip programmed to use internal clock ********************
 ****DO NOT PROGRAM PICCHIP WITH BATTERY UNDER TEST PLUGGED IN!***************
 ****THE PROGRAMMER WILL TURN ON THE POWER RESISTORS, OVERHEATING THEM********
 *****************************************************************************
Operation modes
1.  Power up, display voltages on all seven cells plus total voltage
2.  Watch for key input for next command
3.  Low volts is 1.428 Ohm load   High Volts is 5 Ohm load
4.  Apply 5 Ohm load for 1.5 seconds to equalize cells
5.  Measure all cell voltages and store to array
6.  Apply both the 2 and 5 Ohms for 0.5 seconds
7.  Measure all cell voltages and store to array
8.  Calculate the  internal resistance
9.  Display internal resistance for all cells
10. Calculate maximum safe current for battery pack
11. Add up all cell internal resistance for each cell and show total
12. Loop back to the start
    Following are the connections to the electronic relays
     cell_1 {porta=0b00001000; portc=0b00000000;}
     cell_2 {porta=0b00100000; portc=0b00000000;}
     cell_3 {porta=0b01000000; portc=0b00000000;}
     cell_4 {porta=0b00000000; portc=0b00000001;}
     cell_5 {porta=0b00000000; portc=0b00000010;}
     cell_6 {porta=0b00000000; portc=0b00000100;}
     cell_7 {porta=0b00000000; portc=0b00001000;}
 * 
 * cell_1 a3
 * cell_2 a5
 * cell_3 a6
 * cell_4 c0
 * cell_5 c1
 * cell_6 c2
 * cell_7 c3 
 */
#define RS RB3              // LCD defines
#define EN RB2
#define D4 RC4
#define D5 RC5
#define D6 RC6
#define D7 RC7
#include "lcd.h"

//set up definitions
unsigned char lcd_loc;
unsigned long mi_volt, milli_read, counter, milli_readavg, Max_ir, Tot_ir;
float Test_Amps, Batt_ir, Test_LoAmp, Test_HiAmp;
unsigned long CellCnt, BattNL, BattV5ohm, BattV2ohm, BattV14ohm, Batt_Mah, calc;
unsigned long ch;
long ch_signed;
unsigned char Mah_1000, Mah_100; //Batt Mah numbers
unsigned long tlong;
unsigned char counts, LCD1, LCD2;
unsigned int ArrayV [7];
unsigned int Array14ohm [7];
unsigned int Array2ohm [7];
unsigned int Array5ohm [7];
unsigned long Array_ir [7];
unsigned long sampleV [32];
unsigned long sampleL, sampleH;
//   Look Up tables
unsigned char LCDRow[] = {1, 2, 3, 4, 1, 2, 3, 4}; //LCD Rows
unsigned char LCDCol[] = {4, 4, 4, 4, 15, 15, 15, 15}; //LCD Columns
unsigned char LCDirCol[] = {6, 6, 6, 6, 17, 17, 17, 17}; //LCD Columns
unsigned char LCDirColir[] = {4,4,4,4,15,15,15,15} ;      //irLCD Columns
unsigned char ADChan [] = {8, 10, 0, 1, 2, 3, 4}; //AD Channel
// these are the channel assignments to turn on proper relay for each cell
unsigned char porta_bat[] = {0x08, 0x20, 0x40, 0x00, 0x00, 0x00, 0x00};
unsigned char portc_bat[] = {0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08};

// Variables for time keeping
unsigned long bres = 0;
unsigned long seconds = 0;
unsigned long milliseconds = 0;
unsigned long milliX1000 = 0;

unsigned long start_test, stop_test, end_test, test_duration;

// The PIC ADC relies on the LM7805 for the reference voltage.
// adjust is used to fine tune to a specific LM7805.
int adjust = -4;       // unique adjustment for each meter.
// Meter 1 = -14
// Meter 2 = -12
// Meter 3 = -2
// Meter 4 = -4
// Meter 5 = -1
// Meter 6 = -18
// Meter 7 = -8
// Meter 8 = -4
// Meter 9 = -9
// Meter 10 = -8
// Meter 11 = 1

// Test duration constants.  The ADC measurement will happen at the end of 
// the test time.  
#define ADCTime  861            //measured milliseconds for ADC call 
#define TestTime5ohm 1500       // milliseconds for 5 ohm test
#define TestTime2ohm 1500       // milliseconds for 2 ohm test
#define TestTime1428ohm 500     // milliseconds for 1.42857 ohm test

#define switch PORTBbits.RB1   
#define Button_Pushed PORTBbits.RB1 == 0b00000001
#define Button_NOT_Pushed PORTBbits.RB1 != 0b00000001
#define blink_delay 500
#define R2ohm PORTBbits.RB4
#define R5ohm PORTBbits.RB5
#define on 1
#define off 0

void ADC_Init() {
    //Configure A/D converter
    ANCON0 = 0X01; //configures channel 0 as A/D
    ANCON1 = 0X00; //configure port B as all
    ADCON1 = 0x00; // digital default A/D!

    ADCON0 = 0x01; //Turn ON ADC and Clock Selection
    ADCON1 = 0x00; //All pins as Analog Input and setting Reference Voltages
    ADCON2 = 0xB6; // Right justify result
}

unsigned int Adc_Read(unsigned char channel) {
    if (channel > 7) //Channel range is 0 ~ 7
        return 0;

    ADCON0 &= 0x01; //Clearing channel selection bits
    ADCON0 |= channel << 2; //Setting channel selection bits
    __delay_ms(2); //Acquisition time to charge hold capacitor
    GO_nDONE = 1; //Initializes A/D conversion
    while (GO_nDONE); //Waiting for conversion to complete

    int result = (ADRESH << 8);
    result = result + ADRESL;
    result = result + adjust;
    if (result < 0) result = 0; // don't let it go negative
    return (result);
}
// Display message vertical

void vdisplay(const char *message) {
    for (int i = 0; i < 4; i++)
        Lcd_Chr(i + 1, 10, message[i]);
    __delay_ms(blink_delay);
    for (int i = 0; i < 4; i++)
        Lcd_Chr(i + 1, 10, ' ');
    __delay_ms(blink_delay);

}

void ADCFunction(int *CellVolts) {
    CellCnt = 0;
    for (int cell = 0; cell < 7; cell++) {
        CellVolts[cell] = 0;
        PORTA = (porta_bat[CellCnt]);
        __delay_us(2);
        PORTC = (portc_bat[CellCnt]);
        __delay_ms(40); //wait for warm up
        milli_readavg = 0; //clear milli read
        for (counter = 0; counter < 32; counter++) //adjust calibration  32
        {
            milli_read = Adc_Read(0); //read channel zero
            if (CellCnt == 0) sampleV[counter] = milli_read;
            milli_readavg = (milli_readavg + milli_read); //add up all readings
        }
        mi_volt = (milli_readavg * 10 / 256); //calibrate range here

        if (mi_volt < 100) //If zero volts, skip
        {
            CellVolts[CellCnt] = 0x00; //enter zero count
        } else {
            CellVolts[CellCnt] = mi_volt; //Store cell volts to array
            CellCnt = CellCnt + 1;
        }
    }
    //if first cell is zero volts, clear display to all zeros
    if (CellVolts[0] < 100) //Test first Array for zero
    {
        for (int cell = 0; cell < 7; cell++)
            CellVolts[cell] = 0x00; //If zero, clear full Array
    }

    PORTB = 0x00; //turn off power resistors
    PORTA = 0x00; //turn everything off
    PORTC = 0x00;
}

void LCDFunction(int *CellVolts) {
    {
        for (counts = 0; counts < (CellCnt); counts++) {
            tlong = CellVolts [counts]; //show volts
            Lcd_Chr(LCDRow[counts], LCDCol[counts], 0x30 + ch); //4th digit
            ch = tlong / 1000;
            Lcd_Chr(LCDRow[counts], LCDCol[counts], 0x30 + ch); //4th digit

            Lcd_Chr_Cp('.'); // decimal

            ch = (tlong / 100) % 10;
            Lcd_Chr_Cp(0x30 + ch); //3rd digit

            ch = (tlong / 10) % 10; //2nd digit
            Lcd_Chr_Cp(0x30 + ch);
            //1st digit
            ch = (tlong / 1) % 10;
            Lcd_Chr_Cp(0x30 + ch);
        }
        __delay_ms(50);
    }
}
//*****************************************************************************
//*****Battery Voltage Function with tlong input, LCD1,2 screen location*******
//*****************************************************************************

void Display_Data(char row, char column, long voltage) {
    unsigned long temp;

    temp = voltage / 10000;
    Lcd_Chr(row, column, 0x30 + temp); //4th digit
    temp = (voltage / 1000) % 10;
    Lcd_Chr_Cp(0x30 + temp); //3rd digit
    Lcd_Chr_Cp('.'); // decimal
    temp = (voltage / 100) % 10; //2nd digit
    Lcd_Chr_Cp(0x30 + temp);
    temp = (voltage / 10) % 10; //1st digit
    Lcd_Chr_Cp(0x30 + temp); 
   // temp = (voltage / 1) % 10;
   // Lcd_Chr_Cp(0x30 + temp);
}

void set_pack_mAh() {
    //Clear pack mAh preset
    Mah_1000 = 0;
    Mah_100 = 0;
    EEPROM_Write(0x00, Mah_1000); //write both to EEProm
    EEPROM_Write(0x01, Mah_100);
    //show info on display
    Lcd_Out(1, 1, (char *) "Battery ir Meter    ");
    Lcd_Out(2, 1, (char *) "Release Button      ");
 
    Lcd_Out(4, 1, (char *) "Batt mAh  Cap       ");
    Lcd_Chr(4, 15, 0x30 + Mah_1000);
    Lcd_Chr(4, 16, 0x30 + Mah_100);
    Lcd_Chr(4, 17, 0x30);
    Lcd_Chr(4, 18, 0x30);

    while (Button_Pushed) {
        Lcd_Out(2, 1, (char *) "Release Button      ");
        __delay_ms(blink_delay);
        Lcd_Out(2, 1, (char *) "                    ");
        __delay_ms(blink_delay);
    }
    PORTB = 0; //verify load is off
    
    Lcd_Out(2, 1, (char *) "Hold/Release Button ");
    Lcd_Out(3, 1, (char *) "Do 1000 mAh Batt    ");
    // wait for button to be pushed again. Start of mAh setting
    while (Button_NOT_Pushed);

    //adjust 1000's mAh
    Mah_1000 = -1;
    while (Button_Pushed) {
        if (Mah_1000 == 0x09) //if above 9, set to zero
            Mah_1000 = 0;
        else
            Mah_1000 = Mah_1000 + 1;

        Lcd_Chr(4, 15, 0x30 + Mah_1000);
        __delay_ms(700);
    }
    //adjust 100's mAh

    __delay_ms(500);
    Lcd_Out(3, 1, (char *) "Do 100 mAh Batt     ");
    Lcd_Out(2, 1, (char *) "Hold/Release Button ");
    while (Button_NOT_Pushed); // spin and wait
    Mah_100 = -1;
    while (Button_Pushed) {
        //if above 9, set to zero
        if (Mah_100 == 0x09)
            Mah_100 = 0;
        else
            Mah_100 = Mah_100 + 1;
        Lcd_Chr(4, 16, 0x30 + Mah_100);
        __delay_ms(700);
    }
    EEPROM_Write(0x00, Mah_1000); //write both to EEProm 
    EEPROM_Write(0x01, Mah_100);
}

void startup_splash() {
    unsigned long NineVBat;
    Lcd_Out(1, 1, (char *) "Battery ir Meter    ");
    Lcd_Out(2, 1, (char *) "9 Volt Batt         ");
    Lcd_Out(3, 1, (char *) "Batt Cap      00 mAh");
    Lcd_Out(4, 1, (char *) "Built 03-19-2017 DJS");
    Mah_1000 = EEPROM_Read(0x00);
    Mah_100 = EEPROM_Read(0x01);
    if ( Mah_1000 == 0xFF) {  //prevents garbage display upon firmware load
        Mah_1000 = 0;
        Mah_100 = 0;
    }
    Batt_Mah = (Mah_1000 * 10) + Mah_100;
    Lcd_Chr(3, 13, 0x30 + Mah_1000);
    Lcd_Chr(3, 14, 0x30 + Mah_100);
    //get 9 Volt battery value
    NineVBat = Adc_Read(1);
    NineVBat = (NineVBat * 10000) / 2640;
    Display_Data(2, 14, NineVBat);
    __delay_ms(3000);
    //set off low battery warning if below 6.50 Volts DC
    while (NineVBat < 6700) {
        Lcd_Clear(); 
        Lcd_Out(3, 1, (char *) "Change 9 VDC Battery");
        __delay_ms(3000);
    }

}

void timer_init() {

    T0CONbits.TMR0ON = 1; // Enables Timer
    T0CONbits.T08BIT = 1; // 8 vs 16-bit Timer
    T0CONbits.T0CS = 0; // Internal instruction cycle clock
    T0CONbits.T0SE = 0; // increments on low to high (doesn't matter)
    T0CONbits.PSA = 0; // prescaler not assigned
    T0CONbits.T0PS = 001; // prescaler select  1:4
    INTCONbits.TMR0IE = 1; // set to interrupt on Timer 0 overflow
    INTCONbits.GIE = 1;
    RCONbits.IPEN = 1; // enable multiple priorities of interrupts
}

void interrupt low_priority timer(void) {
    if (TMR0IE && TMR0IF) { //TMR0IE &&
        TMR0IF = 0; // is set when overflow 
        bres += 256; // add 256 ticks to bresenham total
        if (bres >= 1000) // if reached 1 millisecond
        {
            bres -= 1000; // subtract 1 second, retain error
            milliseconds++;
            milliX1000++;
            if (milliX1000 > 1000) {
                seconds = seconds + 1;
                milliX1000 = 0;
            }
            // do_1sec_event(); // update clock, etc
        }
    }
}

unsigned long millis(void) {
    return milliseconds;      // emulates the Arduino function
}
void init() {
    OSCCON = 0x7C; // set clock to 16MHz
    lcd_loc = 0x00;
    TRISA = 0b00000011; //port a, bit 0,1 are analog inputs
    TRISB = 0b00000011; //make port b, bits 0, 1 inputs
    TRISC = 0b00000000;
    PORTA = 0; //shut everything off
    PORTB = 0; //turn off loads
    PORTC = 0;
    Lcd_Init();
    Lcd_Clear();
    ADC_Init();
    R2ohm = off;
    __delay_ms(10);
    R5ohm = off; //turn off power resistors
}

//*****************************************************************************
//***********This is the main program ****************************t************
//*****************************************************************************

void main() {
    init();
    timer_init();
    if (Button_Pushed) set_pack_mAh(); // set battery capacity
    startup_splash(); // check 9v battery
    //this routine reads the individual cell voltages and displays them in a loop
    while (1) { // endless loop
        Lcd_Clear();
        //individual LCD data inputs
        do {
            Lcd_Out(1, 1, (char *) "#1");
            Lcd_Out(2, 1, (char *) "#2");
            Lcd_Out(3, 1, (char *) "#3");
            Lcd_Out(4, 1, (char *) "#4");
            Lcd_Out(1, 9, (char *) "V  #5");
            Lcd_Out(2, 9, (char *) "V  #6");
            Lcd_Out(3, 9, (char *) "V  #7");
            Lcd_Out(4, 9, (char *) "V  #7");
            Lcd_Out(1, 20, (char *) "V");
            Lcd_Out(2, 20, (char *) "V");
            Lcd_Out(3, 20, (char *) "V");
            Lcd_Out(4, 9, (char *) "V Bat");
            Lcd_Out(4, 20, (char *) "V");
            PORTB = 0b00000000; //no load battery measurement
            __delay_ms(30); //  wait for relays to stabilize

            do {
                ADCFunction((int *) ArrayV); //get 7 channel volts
                if (ArrayV[0] < 500) {//Test first Array Volts
                    //test no battery connected
                    Lcd_Clear();
                    Lcd_Out(3, 1, (char *) "No Battery Connected"); //Clear Display to blanks
                    __delay_ms(1000);
                    Lcd_Out(3, 1, (char *) "                    "); //Clear Display to blanks
                    __delay_ms(300); //stay here
                }
            } while (ArrayV[0] < 500);
            //do program
            //*********** Setup display for constants, do not change readouts
            LCDFunction((int *) ArrayV); //display 7 channel volts
            tlong = ArrayV[0] + ArrayV[1] + ArrayV[2]
                    + ArrayV[3] + ArrayV[4] + ArrayV[5] + ArrayV[6]; //add up all cell volts
            Display_Data(4, 15, tlong); //print LiPo batt volts
            //Test for switch pushed for doing load tests //hold here till button pushed
        } while (Button_NOT_Pushed);

        Lcd_Out(4, 1, (char *) "For Fast Hold Switch"); //show how to fast forward
        __delay_ms(1000);
        Lcd_Clear();
        
         //Routine applies 5 Ohm load, reads all cell voltages and stores to Array5ohm
        Lcd_Out(1, 1, (char *) "Test in Progress    ");
        Lcd_Out(2, 1, (char *) "5   Ohm Load is ON  ");
        start_test = millis();
        PORTB = 0b00100000;      // turn on 5 ohm load
        end_test = start_test + TestTime5ohm; 
        while (end_test > millis());  // sit and spin until time expires
        ADCFunction((int *) Array5ohm);
        stop_test = millis();
        PORTB = 0x00;           //turn off 5 ohm load
        test_duration = stop_test - start_test;
        Lcd_Out(2, 1, (char *) "5   Ohm Load is OFF ");
        // Record Volts for Low Current Test Amps
        BattV5ohm = Array5ohm[0] + Array5ohm[1] + Array5ohm[2] //add up all voltages
                  + Array5ohm[3] + Array5ohm[4] + Array5ohm[5] + Array5ohm[6];

        //Routine applies 2 Ohm load, reads all cell voltages and stores to Array2ohm
        Lcd_Out(3, 1, (char *) "2   Ohm Load is ON  ");
        start_test = millis();
        PORTB = 0b00010000;     // turn on 2 ohm load
        end_test = start_test + TestTime2ohm; 
        while (end_test > millis());  // sit and spin until time expire
        ADCFunction((int *) Array2ohm);
        stop_test = millis();
        PORTB = 0x00;           //turn off 2 ohms
        test_duration = stop_test - start_test;
        Lcd_Out(3, 1, (char *) "2   Ohm Load is OFF ");
        BattV2ohm = Array2ohm[0] + Array2ohm[1] + Array2ohm[2] //add up all voltages
                  + Array2ohm[3] + Array2ohm[4] + Array2ohm[5] + Array2ohm[6];
        //Routine applies 1.428 Ohm Load, reads all cell voltages stores to Array14ohm
        Lcd_Out(4, 1, (char *) "1.4 Ohm Load is ON ");
        start_test = millis();
        PORTB = 0b00110000;         //turn on 5 and 2 ohm resistors
        end_test = start_test + TestTime1428ohm; 
        while (end_test > millis());  // sit and spin until time expires
        ADCFunction((int *) Array14ohm);
        stop_test = millis();
        PORTB = 0x00;           //turn off 5 and 2 ohm resistors
        test_duration = stop_test - start_test;
        Lcd_Out(4, 1, (char *) "1.4 Ohm Load is OFF ");
        BattV14ohm = Array14ohm[0] + Array14ohm[1] + Array14ohm[2]
                + Array14ohm[3] + Array14ohm[4] + Array14ohm[5] + Array14ohm[6];
        // check to see that pack was connected
        if ( abs((BattV5ohm - BattV14ohm)) < 10 ) {
            Lcd_Clear();
            Lcd_Out(2, 1, (char *) "   Turn off meter   ");
            Lcd_Out(3, 1, (char *) "Connect power leads ");
            while (Button_NOT_Pushed);
        }
        //*****************************************************************************
        //if button is closed at this time, skip all readings and go to ir readouts
        //******************************************************************************
        if (Button_NOT_Pushed) {

            //else, show results volts and amps at 5, 2 and 1.4 Ohms
            Lcd_Clear();
            Lcd_Out(1, 1, (char *) "Results             ");
            Lcd_Out(2, 1, (char *) "LoAmps      V      A");
            Lcd_Out(3, 1, (char *) "MdAmps      V      A");
            Lcd_Out(4, 1, (char *) "HiAmps      V      A");
            //display pack voltage at low Amps, mid Amps and Hi Amps
            Display_Data(2, 8, BattV5ohm);
            Display_Data(3, 8, BattV2ohm);
            Display_Data(4, 8, BattV14ohm);
            Display_Data(2, 15, BattV5ohm / 5);
            Display_Data(3, 15, BattV2ohm / 2);
            Display_Data(4, 15, (long) (BattV14ohm / 1.42857));
            //hold for key push here
            do {
                Lcd_Out(1, 9, (char *) "Push Button ");
                __delay_ms(blink_delay);
                Lcd_Out(1, 8, (char *) "            ");
                __delay_ms(blink_delay);
            } while (Button_NOT_Pushed);

            //show battery voltages under load
            Lcd_Out(1, 1, (char *) "#1      V  #5      V");
            Lcd_Out(2, 1, (char *) "#2      V  #6      V");
            Lcd_Out(3, 1, (char *) "#3      V  #7      V");
            Lcd_Out(4, 1, (char *) "#4      V  Low Amp  ");

            LCDFunction((int *) Array5ohm);
            do {
                vdisplay("Push");
            } while (Button_NOT_Pushed);

            //show results at 2 ohms
            Lcd_Out(4, 1, (char *) "#4      V  Mid Amp  ");

            LCDFunction((int *) Array2ohm);
            do {
                vdisplay("Push");
            } while (Button_NOT_Pushed);

            Lcd_Out(4, 1, (char *) "#4      V  High Amp ");

            LCDFunction((int *) Array14ohm);
            do {
                vdisplay("Push");
            } while (Button_NOT_Pushed);
        }

        // Display ir results
        Lcd_Out(1, 1, (char *) "1ir        5ir      ");
        Lcd_Out(2, 1, (char *) "2ir        6ir      ");
        Lcd_Out(3, 1, (char *) "3ir        7ir      ");
        Lcd_Out(4, 1, (char *) "4ir       milliohms ");
        //calculate ir where ir = (Vlow-Vhi)/(AmpsHi-AmpsLo)
        Array_ir [0] = 0; //clear array for less than 7 cells
        Array_ir [1] = 0;
        Array_ir [2] = 0;
        Array_ir [3] = 0;
        Array_ir [4] = 0;
        Array_ir [5] = 0;
        Array_ir [6] = 0;
        Tot_ir = 0; //clear total ir value
        //do ir calculations 
        Test_Amps = (BattV14ohm / 1.42857) - (BattV5ohm / 5);
        for (counts = 0; counts < (CellCnt); counts++) {
            tlong = (Array5ohm[counts] - Array14ohm[counts]);
            tlong = tlong * 100000; //adjust ranges on calculation
            tlong = tlong / (long ) Test_Amps;
            Array_ir [counts] = tlong;
            Tot_ir = Tot_ir + tlong; //add up total IR's
            ch = tlong / 10000; //9xx.xx
            Lcd_Chr(LCDRow[counts], LCDirColir[counts], 0x30 + ch); //4th digit
            ch = (tlong / 1000) % 10;
            Lcd_Chr_Cp(0X30 + ch);
            ch = (tlong / 100) % 10;
            Lcd_Chr_Cp(0x30 + ch); //3rd digit
            Lcd_Chr_Cp('.'); // decimal
            ch = (tlong / 10) % 10;
            Lcd_Chr_Cp(0x30 + ch); //2nd digit
            ch = (tlong / 1) % 10;
            Lcd_Chr_Cp(0x30 + ch); //1st digit
        }
        __delay_ms(1000);

        do {
            vdisplay("Rls ");
        } while (Button_Pushed);

        Lcd_Out(4, 11, (char *) "T-IR     ");
        tlong = Tot_ir;

        ch = tlong / 10000; //adjust full range
        Lcd_Chr(4, 16, 0x30 + ch); //4th digit
        ch = (tlong / 1000) % 10;
        Lcd_Chr_Cp(0x30 + ch); //3rd digit

        ch = (tlong / 100) % 10; //2nd digit
        Lcd_Chr_Cp(0x30 + ch);
        Lcd_Chr_Cp('.'); // decimal                                               //1st digit
        ch = (tlong / 10) % 10;
        Lcd_Chr_Cp(0x30 + ch);

        do {
            vdisplay("Push");
        } while (Button_NOT_Pushed);

        Lcd_Out(1, 1, (char *) "Max Current is     A");
        Lcd_Out(2, 1, (char *) "Reference RCGroups  ");
        Lcd_Out(3, 1, (char *) "Forsyth,Julian,Giles");
        Lcd_Out(4, 1, (char *) "                    ");
        //this routine finds the maximum ir value in the ir Array
        for (counts = 0; counts < 7; counts++) {
            if (Array_ir[0] < Array_ir[counts])
                Array_ir[0] = Array_ir[counts];
        }
        Max_ir = Array_ir[0]; //calculate ir for cell
        calc = 60000 * Batt_Mah;
        tlong = (long) sqrt(calc / Max_ir); //do square root on results
        ch = tlong / 100;
        Lcd_Chr(1, 16, 0x30 + ch); //4th digit
        ch = (tlong / 10) % 10;
        Lcd_Chr_Cp(0x30 + ch); //3rd digit

        ch = (tlong / 1) % 10; //2nd digit
        Lcd_Chr_Cp(0x30 + ch);

        do {
            Lcd_Out(4, 1, (char *) "Push Button to Reset");
            __delay_ms(1000);
            Lcd_Out(4, 1, (char *) "                    ");
            __delay_ms(500);

        } while (Button_NOT_Pushed);
    }
}



