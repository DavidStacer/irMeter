

//LCD Functions Developed by electroSome

#define _LCD_FIRST_ROW          0x80 // Move cursor to the 1st row
#define _LCD_SECOND_ROW         0xC0 // Move cursor to the 2nd row
#define _LCD_THIRD_ROW          0x94 // Move cursor to the 3rd row
#define _LCD_FOURTH_ROW         0xD4 //	Move cursor to the 4th row
#define _LCD_CLEAR              0x01 // Clear display
#define _LCD_RETURN_HOME        0x02 //	Return cursor to home position, returns a shifted display to its original position. Display data RAM is unaffected.
#define _LCD_CURSOR_OFF         0x0C // Turn off cursor
#define _LCD_UNDERLINE_ON       0x0E // Underline cursor on
#define _LCD_BLINK_CURSOR_ON	0x0F // Blink cursor on
#define _LCD_MOVE_CURSOR_LEFT   0x10 //	Move cursor left without changing display data RAM
#define _LCD_MOVE_CURSOR_RIGHT  0x14 //	Move cursor right without changing display data RAM
#define _LCD_TURN_ON            0x0C //	Turn Lcd display on
#define _LCD_TURN_OFF           0x08 // Turn Lcd display off
#define _LCD_SHIFT_LEFT         0x18 // Shift display left without changing display data RAM
#define _LCD_SHIFT_RIGHT        0x1C //	Shift display right without changing display data RAM

void Lcd_Port(char a)
{
	if(a & 1)
		D4 = 1;
	else
		D4 = 0;

	if(a & 2)
		D5 = 1;
	else
		D5 = 0;

	if(a & 4)
		D6 = 1;
	else
		D6 = 0;

	if(a & 8)
		D7 = 1;
	else
		D7 = 0;
}
void Lcd_Cmd(char a)
{   //Register Select (RS). RS=0: Command, RS=1: Data
	RS = 0;             // => RS = 0
	Lcd_Port(a);
	EN  = 1;             // => E = 1
    __delay_ms(4);
    EN  = 0;             // => E = 0
}

void Lcd_Clear()    // Clear the LCD Screen
{
    Lcd_Cmd(0);
    Lcd_Cmd(_LCD_CLEAR);
}

void Lcd_Set_Cursor(char row, char column)
{
	char temp,z,y;
    switch (row){
        case 1:
            temp = _LCD_FIRST_ROW  + column - 1;
            break;
        case 2:
            temp = _LCD_SECOND_ROW + column - 1;
            break;
        case 3:
            temp = _LCD_THIRD_ROW + column - 1;
            break;
        case 4:
            temp = _LCD_FOURTH_ROW  + column - 1;
            break;            
    }
    z = temp>>4;
    y = temp & 0x0F;
    Lcd_Cmd(z);
    Lcd_Cmd(y);
}

void Lcd_Init()
{
  Lcd_Port(0x00);
   __delay_ms(20);
  Lcd_Cmd(0x03);
	__delay_ms(5);
  Lcd_Cmd(0x03);
	__delay_ms(11);
  Lcd_Cmd(0x03);
  Lcd_Cmd(0x02);
  Lcd_Cmd(0x02);
  Lcd_Cmd(0x08);
  Lcd_Cmd(0x00);
  Lcd_Cmd(0x0C);
  Lcd_Cmd(0x00);
  Lcd_Cmd(0x06);
}
// write a character at the current cursor location
void Lcd_Chr_Cp(char a)
{
   char temp,y;
   temp = a&0x0F;
   y = a&0xF0;
   RS = 1;             // => RS = 1
   Lcd_Port(y>>4);             //Data transfer
   EN = 1;
   __delay_us(40);
   EN = 0;
   Lcd_Port(temp);
   EN = 1;
   __delay_us(40);
   EN = 0;
}

void Lcd_Write_String(char *a)
{
	int i;
	for(i=0;a[i]!='\0';i++)
	   Lcd_Chr_Cp(a[i]);
}

// Write character at row, column 
void Lcd_Chr(char row, char column, char out_char)
{
    Lcd_Set_Cursor(row, column);
    Lcd_Chr_Cp(out_char);
}

// Write a string at row, column 
void Lcd_Out(char row, char column, char *out_string)
{
    Lcd_Set_Cursor(row, column);
    Lcd_Write_String(out_string);
}