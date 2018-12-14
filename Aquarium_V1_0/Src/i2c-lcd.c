
/** Put this in the src folder **/

#include "i2c-lcd.h"
#include <string.h>
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
{
	lcd_send_cmd (0x02);
	lcd_send_cmd (0x28);
	lcd_send_cmd (0x0c);
	lcd_send_cmd (0x80);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

// Spring mit Cursor zur Reihe und Spalte
void cursor_jumpto_r_c (uint8_t row, uint8_t column)
{
	uint8_t mycmd;
	switch(row){
		case 1: mycmd = 0x80;break;		// MSB=1, Bits 6-0 = AC: 1-000 0000

		case 2: mycmd = 0x94;break;		// 1-001 0100

		case 3: mycmd = 0xA8;break;		// 1-010 1000

		case 4: mycmd = 0xBC;break;		// 1-011 1100
	}
	mycmd+=column;
	lcd_send_cmd (mycmd);
}

// Shift cursor einmal nach links
void cursor_shift_left(void)
{
	lcd_send_cmd (0x10);
}

// Shift cursor einmal nach rechts
void cursor_shift_right(void)
{
	lcd_send_cmd (0x14);
}

//Shift cursor n-mal nach links
void cursor_shift_left_ntime(uint8_t number)
{
	for(int i = 0 ; i < number ; i++)
		{
			cursor_shift_left();
		}
}

//Shift cursor n-mal nach rechts
void cursor_shift_right_ntime(uint8_t number)
{
	for(int i = 0 ; i < number ; i++)
		{
			cursor_shift_right();
		}
}

//Lösche die ganze Zeile
void delete_row (uint8_t row)
{
	char *delete_me[20];
	memset(delete_me,0,20);				//Lösche alles im Array

	cursor_jumpto_r_c ( row, 0);
	lcd_send_string (&delete_me);		// überschreibe mit leerem String
	cursor_jumpto_r_c ( row, 0);		// Springe zur ersten Stelle der Zeile zurück
}

//Lösche das aktuelle Zeichen
void delete_current_char(void)
{
	char delete_me= '\0';
	cursor_shift_left();
	lcd_send_data (delete_me);			// überschreibe mit leerem Char
	cursor_shift_left();
}

void delete_some_chars (uint8_t number)
{
	char *delete_me[number];
	memset(delete_me,0,number);

	cursor_shift_left_ntime(number);
	lcd_send_string (&delete_me);		// überschreibe mit leerem String
	cursor_shift_left_ntime(number);	// laufe mit dem Cursor zurück
}
