#include "stm32f4xx_hal.h"

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void cursor_jumpto_r_c (uint8_t row, uint8_t column);

void cursor_shift_left(void);

void cursor_shift_right(void);
