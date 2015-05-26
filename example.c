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

uint8_t number_keys[10] =
	{KEY_0,KEY_1,KEY_2,KEY_3,KEY_4,KEY_5,KEY_6,KEY_7,KEY_8,KEY_9};

uint8_t keymap[2][3][10] =
	{{{KEY_Q, KEY_W, KEY_E,      KEY_R,         KEY_T, KEY_Y, KEY_U,     KEY_I,     KEY_O,      KEY_P},
     {KEY_A, KEY_S, KEY_D,      KEY_F,         KEY_G, KEY_H, KEY_J,     KEY_K,     KEY_L,      KEY_FUNCTION},
	  {KEY_Z, KEY_X, KEY_C,      KEY_V,         KEY_B, KEY_N, KEY_M,     KEY_COMMA, KEY_PERIOD, KEY_SHIFT}},

	 {{KEY_1, KEY_2, KEY_3,      KEY_4,         KEY_5,      KEY_6,     KEY_7,     KEY_8, KEY_9, KEY_0},
     {0,     0,     0,          KEY_BACKSPACE, KEY_DELETE, KEY_ENTER, KEY_SPACE, 0,     0,     KEY_FUNCTION},
     {0,     0,     0,          0,             0,          0,         0,         0,     0,     KEY_SHIFT}}};

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
			  shift, function,
			  reset_idle;
	uint8_t b_prev[3] = {0xFF, 0xFF, 0xFF};
	uint8_t f_prev[3] = {0xFF, 0xFF, 0xFF};

	// set for 16 MHz clock
	CPU_PRESCALE(0);

	// Set all input and output ports correctly
	LED_CONFIG;
	LED_OFF;
	DDRD  = 0x47;					// Rows 0 thru 2 & the LEDs
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
		// check if any pins are high, but were low previously
		// reset_idle = 0;

		if (function)
			LED_ON;
		else
			LED_OFF;

		for (rowCount = 0; rowCount < 3; ++rowCount) {
			PORTD |= 1 << rowCount;
			_delay_ms(1);
			b = PINB;
			f = PINF;

			for (colCount = 0; colCount < 5; ++colCount) {
			// Handle the Shift and Function Keys
				if(keymap[0][rowCount][colCount + 5] == KEY_SHIFT)
					shift = f & f_mask[colCount] ? 1 : 0;
				if(keymap[0][rowCount][colCount + 5] == KEY_FUNCTION)
					function = f & f_mask[colCount] ? 1 : 0;

			// Handle the left side
				if ((b & b_mask[colCount]) &&
				     !(b_prev[rowCount] & b_mask[colCount])) {
					usb_keyboard_press(keymap[function][rowCount][colCount],
					                   shift ? KEY_SHIFT : 0);
					reset_idle = 1;
				}
			// Handle the right side
				if ((f & f_mask[colCount]) &&
				     !(f_prev[rowCount] & f_mask[colCount])) {
					usb_keyboard_press(keymap[function][rowCount][colCount + 5],
					                   shift ? KEY_SHIFT : 0);
					reset_idle = 1;
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

			b_prev[rowCount] = b;
			f_prev[rowCount] = f;

			_delay_ms(1);
			PORTD &= ~(1 << rowCount);
		}
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
