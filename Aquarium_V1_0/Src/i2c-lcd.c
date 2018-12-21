
#include "i2c-lcd.h"
#include <string.h>
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // I2C Module adress

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
	uint8_t i=0;
	HAL_Delay(100);
	for(i=0;i<3;i++)  //sending 3 times: select 4-bit mode
	{
		lcd_send_cmd(0x03);
		HAL_Delay(45);
	}
	lcd_send_cmd (0x02);
	HAL_Delay(100);
	lcd_send_cmd (0x28);
	HAL_Delay(1);
	lcd_send_cmd (0x0c);
	HAL_Delay(1);
	lcd_send_cmd (0x80);
	HAL_Delay(1);
	lcd_send_cmd (0x01);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

// Spring mit Cursor zur Reihe und Spalte
void cursor_jumpto_r_c (uint8_t row, uint8_t column)
{
	uint8_t mycmd = 0;
	switch(row){
		case 1: mycmd = 0x80;break;		// MSB=1, Bits 6-0 = AC: 1-000 0000

		case 2: mycmd = 0xC0;break;		// 1-001 0100

		case 3: mycmd = 0x94;break;		// 1-010 1000

		case 4: mycmd = 0xD4;break;		// 1-011 1100
	}
	mycmd += (column-1);
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

//L�sche die ganze Zeile
void delete_row (uint8_t row)
{

	cursor_jumpto_r_c ( row, 0);
	lcd_send_string ("                    ");		// Ueberschreibe mit leerem String
	cursor_jumpto_r_c ( row, 0);		// Springe zur ersten Stelle der Zeile zurueck
}

//L�sche das aktuelle Zeichen
void delete_current_char(void)
{
	char delete_me= ' ';
	cursor_shift_left();
	lcd_send_data (delete_me);			// Ueberschreibe mit leerem Char
	cursor_shift_left();
}

void delete_some_chars (uint8_t number)
{
	char delete_me[number];

	for (int i=0; i<number; i++){
		delete_me[i]=' ';
	}
	lcd_send_string (&delete_me);		// Ueberschreibe mit leerem String
	cursor_shift_left_ntime(number);	// laufe mit dem Cursor zurueck
}

//Blinken des Cursor ON

void blink_cursor_ON (void)
{
	lcd_send_cmd(0x0D);
}

//Blinken des Cursor OFF

void blink_cursor_off(void)
{
	lcd_send_cmd(0x0C);
}

// Turn ON display

void display_turn_on (void)
{
	lcd_send_cmd(0x0C);
}

// Turn OFF display

void display_turn_off (void)
{
	lcd_send_cmd(0x08);
}
