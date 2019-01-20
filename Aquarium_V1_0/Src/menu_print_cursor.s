.global menu_print_cursor
.extern	cursor_jumpto_r_c			// This function moves the cursor to row x and column y
.extern delete_some_char			//	Delete x chars
.extern lcd_send_string				// Print text on LCD

.thumb
.syntax unified

.data
my_arrow:	.asciz	"->"


.text
.align

/* User Code begin here
 * Brief	display the arrow on the chosen row
 * data		r0 contains the chosen row
*/
menu_print_cursor:
	PUSH	{lr}
	cmp		r0, #1					//Switch case 1,2,3,4
	BEQ		case_1					// Compare r0 with the rows and jump if it's equal
	cmp		r0, #2
	BEQ		case_2
	cmp		r0, #3
	BEQ		case_3
	cmp		r0, #4
	BEQ		case_4

case_1:								// The row 1 is chosen.
	MOV		r0, #2					// Be to sure, delete on every row the arrow "->", which
	MOV		r1, #2					// won't be displayed. Then rewrite the arrow
	BL		cursor_jumpto_r_c		// on the chosen row.
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #3
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #4
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #1				//move the cursor to row 1
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, my_arrow		//display the arrow
	BL		lcd_send_string

	BL end

case_2:							// The row 2 is chosen.
	MOV		r0, #1
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #3
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #4
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #2					//move the cursor to row 2
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, my_arrow			//display the arrow
	BL		lcd_send_string

	BL end

case_3:								// The row 3 is chosen.
	MOV		r0, #1
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #2
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #4
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #3					//move the cursor to row 3
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, my_arrow			//display the arrow
	BL		lcd_send_string

	BL end

case_4:								// The row 4 is chosen.
	MOV		r0, #1
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #2
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #3
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, #2
	BL		delete_some_chars

	MOV		r0, #4					//move the cursor to row 4
	MOV		r1, #2
	BL		cursor_jumpto_r_c
	MOV		r0, my_arrow			//display the arrow
	BL		lcd_send_string


	BL end

end:
.align
	POP	{pc}

