/* Keyboard example for Teensy USB Development Board
* http://www.pjrc.com/teensy/usb_keyboard.html
* Copyright (c) 2008 PJRC.COM, LLC
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_keyboard.h"

#define LED_CONFIG		(DDRC  |=  0x80)
#define LED_ON				(PORTC &= ~0x80)
#define LED_OFF			(PORTC |=  0x80)
#define CPU_PRESCALE(n)	(CLKPR  = 0x80, CLKPR = (n))
#define BMASK				0b00011111
#define FMASK				0b01110011
#define REPEAT_RATE     2

uint8_t keymap[2][3][10] =
	{{{KEY_Q, KEY_W, KEY_E,      KEY_R,         KEY_T, KEY_Y, KEY_U,     KEY_I,     KEY_O,      KEY_P},
     {KEY_A, KEY_S, KEY_D,      KEY_F,         KEY_G, KEY_H, KEY_J,     KEY_K,     KEY_L,      KEY_FUNCTION},
	  {KEY_Z, KEY_X, KEY_C,      KEY_V,         KEY_B, KEY_N, KEY_M,     KEY_COMMA, KEY_PERIOD, KEY_SHIFT}},

	 {{KEY_1,    KEY_2,     KEY_3,     KEY_4,         KEY_5,          KEY_6,           KEY_7,         KEY_8,  KEY_9,     KEY_0},
     {KEY_ESC,  KEY_MINUS, KEY_EQUAL, KEY_SEMICOLON, KEY_QUOTE,      KEY_LEFT,        KEY_DOWN,      KEY_UP, KEY_RIGHT, KEY_FUNCTION},
     {KEY_CTRL, KEY_ALT,   KEY_GUI,   KEY_TILDE,     KEY_LEFT_BRACE, KEY_RIGHT_BRACE, KEY_BACKSLASH, 0,      KEY_SLASH, KEY_SHIFT}}};

uint8_t b_combo_mask[3] = {0x07,      0x0E,          0x0E};
uint8_t b_combo[3]      = {KEY_TAB,   KEY_BACKSPACE, KEY_DELETE};

uint8_t f_combo_mask[3] = {0x32,      0x32,          0xFF};
uint8_t f_combo[3]      = {KEY_ENTER, KEY_SPACE,     0};


uint8_t b_mask[5] = {0x01,       // B0
                     0x02,       // B1
							0x04,       // B2
							0x08,       // B3
							0x10};      // B4

uint8_t f_mask[5] = {0x01,			// F0
                     0x02,       // F1
							0x10,       // F4
							0x20,       // F5
							0x40};      // F6

uint16_t idle_count = 0;

int main(void) {
	uint8_t b, f,
			  rowCount, colCount,
			  function = 0, mod = 0,
           run,
			  prevKey = 0, curKey = 0,
           hold_count = 0;
	uint8_t b_prev[3] = {0xFF, 0xFF, 0xFF};
	uint8_t f_prev[3] = {0xFF, 0xFF, 0xFF};

	// set for 16 MHz clock
	CPU_PRESCALE(0);

	// Set all input and output ports correctly
	LED_CONFIG;
	LED_OFF;
	DDRD  = 0x47;					// Rows 0 thru 2 & the Teensy's LED
	PORTD = 0x00;
	DDRB  = 0x00;  				// Cols 0 thru 4
	PORTB = 0x00;
	DDRF  = 0x00;              // Cols 5 thru 9
	PORTF = 0x00;

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured())
		;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	// Configure timer 0 to generate a timer overflow interrupt every
	// 256*1024 clock cycles, or approx 61 Hz when using 16 MHz clock
	// This demonstrates how to use interrupts to implement a simple
	// inactivity timeout.
	// TCCR0A = 0x00;
	// TCCR0B = 0x05;
	// TIMSK0 = (1<<TOIE0);

	PORTD = 0x40;  				//  Teensy LED On
	LED_CONFIG;

	while (1) {
		if (mod & ~KEY_SHIFT)
			LED_ON;
		else
			LED_OFF;

		for (rowCount = 0, run = 1, curKey = 0; rowCount < 3 && run; ++rowCount) {
			PORTD |= 1 << rowCount;
			_delay_ms(5);
			b = PINB;
			f = PINF;

			// Handle combo key entry
			if ((b & b_combo_mask[rowCount]) == b_combo_mask[rowCount] &&
			 (b & BMASK) != (b_prev[rowCount] & BMASK)) {
				curKey = b_combo[rowCount];
				run = 0;
			}
			if ((f & f_combo_mask[rowCount])== f_combo_mask[rowCount] &&
			 (f & FMASK) != (f_prev[rowCount] & FMASK)) {
				curKey = f_combo[rowCount];
				run = 0;
			}

			for (colCount = 0; colCount < 5 && run; ++colCount) {
				// Handle the Mod and Function Keys
				if(keymap[0][rowCount][colCount + 5] == KEY_FUNCTION) {
					function = f & f_mask[colCount] ? 1 : 0;
				}

				if(keymap[0][rowCount][colCount + 5] == KEY_SHIFT) {			// This should be a switch statement
					if (f & f_mask[colCount])
						mod |= KEY_SHIFT;
					else
						mod &= ~(KEY_SHIFT);
				}
				if(keymap[function][rowCount][colCount] == KEY_CTRL) {
					if (b & b_mask[colCount])
						mod |= KEY_CTRL;
					// else
					// 	mod &= ~(KEY_CTRL);
				}
				if(keymap[function][rowCount][colCount] == KEY_ALT) {
					if (b & b_mask[colCount])
						mod |= KEY_ALT;
					// else
					// 	mod &= ~(KEY_ALT);
				}
				if(keymap[function][rowCount][colCount] == KEY_GUI) {
					if (b & b_mask[colCount])
						mod |= KEY_GUI;
					// else
					// 	mod &= ~(KEY_GUI);
				}

				// Handle traditional key entry
				if ((b & b_mask[colCount]) &&
			    !(b_prev[rowCount] & b_mask[colCount])) {
					curKey = keymap[function][rowCount][colCount];
				}
				if ((f & f_mask[colCount]) &&
			    !(f_prev[rowCount] & f_mask[colCount])) {
					curKey = keymap[function][rowCount][colCount + 5];
				}

			}

			// if any keypresses were detected, reset the idle counter
			// if (reset_idle) {
				// variables shared with interrupt routines must be
				// accessed carefully so the interrupt routine doesn't
				// try to use the variable in the middle of our access
				// cli();
				// idle_count = 0;
				// sei();
			// }

			// now the current pins will be the previous, and
			// wait a short delay so we're not highly sensitive
			// to mechanical "bounce".

			_delay_ms(10);
			PORTD &= ~(1 << rowCount);

			// if (keyCount > 2)
			// 	curKey = KEY_SPACE;
			// else {
			b_prev[rowCount] = b;
			f_prev[rowCount] = f;
			// }
		}
		if (curKey && prevKey != curKey) {
			usb_keyboard_press(curKey, mod & KEY_META_MASK);
			mod = 0;
			hold_count = 0;
		} else if (++hold_count == REPEAT_RATE) {
		   for (rowCount = 0; rowCount < 3; ++rowCount) {
				b_prev[rowCount] = 0x00;
				f_prev[rowCount] = 0x00;
			}
			hold_count = 0;
      }

		prevKey = curKey;
	}
}

// This interrupt routine is run approx 61 times per second.
// A very simple inactivity timeout is implemented, where we
// will send a space character.
// ISR(TIMER0_OVF_vect)
// {
// 	idle_count++;
// 	if (idle_count > 61 * 8) {
// 		idle_count = 0;
// 		usb_keyboard_press(KEY_SPACE, 0);
// 	}
// }
