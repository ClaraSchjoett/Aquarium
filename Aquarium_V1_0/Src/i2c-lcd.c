
#include "i2c-lcd.h"
#include <string.h>
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // I2C Module adress

/*
 * @brief In dieser Datei werden die Grundfunktionen des Display implementiert,
 * die wir später im main gebrauchen können.
 * Sie bestehen au:
 * 		-lcd_send_cmd
 * 		-lcd_send_data
 * 		-lcd_init
 * 		-lcd_send_string
 * 		-cursor_jump_r_c
 * 		-cursor_shift_left
 * 		-cursor_shift_right
 * 		-cursor_shift_left_ntime
 * 		-cursor_shift_right_ntime
 * 		-delete_row
 * 		-delete_current_char
 * 		-delete_some_chars
 * 		-blink_cursor_ON
 * 		-blink_cursor_OFF
 * 		-display_turn_on
 * 		-display_turn_off
 *
 */

/*
 * @brief Mit der Funktion lcd_send_cmd koennen
 * Befehle ans Display übergeben werden. Die verschiedenen
 * Befehle koennen aus dem Datenblatt entnommen werden.
 * @param Befehl (Hex)
 * @retval None
 */

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

/*
 * @brief Mit dieser Funktion kann ein Buchstaben uebertragen werden.
 * @param 1 Char
 * @retval None
 */

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

/*
 * @brief Diese Funktion wird am Anfang des Programm gebraucht um das Dispalay
 * beim Aufstarten zu inizialisieren.
 * Dies wurde wie im Datenblatt gefordert gemacht.
 * @param None
 * @retval None
 */

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

/*
 * @brief Dieser Funktion kann man einen String uebergeben und sie
 * gibt den Text auf dem Display aus, indem sie für jeden
 * Character die Funktion lcd_send_data aufruft.
 * @param Text
 * @retval None
 */

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

/*
 * @brief Dieser Funktion kann man den Ort (Reihe und Spalte)
 * angeben wo man hinspringen möchte und sie versetzt den
 * Cursor an den richtigen Ort.
 * @param Zeile, Spalte
 * @retval None
 */

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

/*
 * @brief Diese Funktion verschiebt den Cursor um einen Character
 * nach links. Die Funktion wird bei löschen gebraucht.
 * @param None
 * @retval None
 */

void cursor_shift_left(void)
{
	lcd_send_cmd (0x10);
}

/*
 * @brief Diese Funktion verschiebt den Cursor um einen Character
 * nach rechts.
 * @param None
 * @retval None
 */

void cursor_shift_right(void)
{
	lcd_send_cmd (0x14);
}

/*
 * @brief Diese Funktion kann den Cursor um eine gewuenschte
 * Anzahl Character nach links verschieben. Sie wir beim Loeschen graucht.
 * @param Anzahl Sprünge
 * @retval None
 */

void cursor_shift_left_ntime(uint8_t number)
{
	for(int i = 0 ; i < number ; i++)
		{
			cursor_shift_left();
		}
}

/*
 * @brief Diese Funktion kann den Cursor um eine gewuenschte
 * Anzahl Character nach rechts verschieben.
 * @param Anzahl Sprünge
 * @retval None
 */

void cursor_shift_right_ntime(uint8_t number)
{
	for(int i = 0 ; i < number ; i++)
		{
			cursor_shift_right();
		}
}

/*
 * @brief Diese kann eine ganze Zeile loeschen.
 * Ihr muss die Zeile die geloescht werden soll
 * übergeben werden.
 * Sie "ueberschreibt" die Zeile mit Leerzeichen.
 * @param Zeilennummer
 * @retval None
 */

void delete_row (uint8_t row)
{

	cursor_jumpto_r_c ( row, 0);
	lcd_send_string ("                    ");
	cursor_jumpto_r_c ( row, 0);
}

/*
 * @brief Die Funktion soll den aktuellen Character loeschen.
 * @param None
 * @retval None
 */

void delete_current_char(void)
{
	char delete_me= ' ';
	cursor_shift_left();
	lcd_send_data (delete_me);
	cursor_shift_left();
}

/*
 * @brief Die Funktion Loescht eine gewuenschte Anzahl an Zeichen.
 * @param Anzahl Buchstaben
 * @retval None
 */

void delete_some_chars (uint8_t number)
{
	char delete_me[number];

	for (int i=0; i<number; i++){
		delete_me[i]=' ';
	}
	lcd_send_string (&delete_me);		// Ueberschreibe mit leerem String
	cursor_shift_left_ntime(number);	// laufe mit dem Cursor zurueck
}

/*
 * @brief Funktion um den Cursor blinken zu lassen
 * (wird nicht gebraucht)
 * @param None
 * @retval None
 */

void blink_cursor_ON (void)
{
	lcd_send_cmd(0x0D);
}

/*
 * @brief Funktion um das Blinken des Cursors
 * wieder auszuschalten.
 * (wird nicht gebraucht)
 * @param None
 * @retval None
 */

void blink_cursor_off(void)
{
	lcd_send_cmd(0x0C);
}

/*
 * @brief Funktion, die das Display einschalten soll.
 * Funktioniert wird aber nicht gebraucht.
 * @param None
 * @retval None
 */

void display_turn_on (void)
{
	lcd_send_cmd(0x0C);
}

/*
 * @brief Funktion, die das Display ausschalten soll.
 * Es verschwindet der Text vom Display, Hintergrundbeleuchtung
 * bleibt.
 * (wird nicht gebraucht)
 * @param None
 * @retval None
 */

void display_turn_off (void)
{
	lcd_send_cmd(0x08);
}
