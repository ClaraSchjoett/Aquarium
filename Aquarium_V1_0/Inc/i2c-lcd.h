#include "stm32f4xx_hal.h"

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void cursor_jumpto_r_c (uint8_t row, uint8_t column);	// jump with cursor to row/column

void cursor_shift_left(void);	//shift cursor once left

void cursor_shift_right(void);	//shift cursor once right

void cursor_shift_left_ntime(uint8_t number);	// shift cursor n-time left

void cursor_shift_right_ntime(uint8_t number);	// shift cursor n-time right

void delete_row (uint8_t row);		// delete the whole row

void delete_current_char(void);		// delete the current char

void delete_some_chars (uint8_t number);	// delete some chars or n-time
