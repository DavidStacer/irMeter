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


void EEPROM_Write(unsigned char address, unsigned char databyte)
{ // writes the "databyte" value to EEPROM at the address given
 // location in "address".
 EECON1bits.EEPGD = 0; // Set to access EEPROM memory
 EECON1bits.CFGS = 0; // Do not access Config registers
 EEDATA = databyte; // Load EEDATA with byte to be written
 EEADR = address; // Load EEADR with address of location to write.
 EECON1bits.WREN = 1; // Enable writing

 INTCONbits.GIE = 0; // Disable interrupts
 EECON2 = 0x55; // Begin Write sequence
 EECON2 = 0xAA;
 EECON1bits.WR = 1; // Set WR bit to begin EEPROM write
 INTCONbits.GIE = 1; // re-enable interrupts

 while (EECON1bits.WR == 1)
 { // wait for write to complete.
 };
 EECON1bits.WREN = 0; // Disable writing as a precaution.
} 

unsigned char EEPROM_Read(unsigned char address)
{ // reads and returns the EEPROM byte value at the address given
 // given in "address".
 EECON1bits.EEPGD = 0; // Set to access EEPROM memory
 EECON1bits.CFGS = 0; // Do not access Config registers
 EEADR = address; // Load EEADR with address of location to write.
 // execute the read
 EECON1bits.RD = 1; // Set the RD bit to execute the EEPROM read
 // The value read is ready the next instruction cycle in EEDATA. No wait is
 // needed.
 return EEDATA;
}