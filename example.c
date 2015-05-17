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

#define LED_CONFIG		(DDRD |= (1<<8))
#define LED_ON				(PORTD &= ~(1<<8))
#define LED_OFF			(PORTD |= (1<<8))
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

uint8_t number_keys[10] =
	{KEY_0,KEY_1,KEY_2,KEY_3,KEY_4,KEY_5,KEY_6,KEY_7,KEY_8,KEY_9};

uint8_t layer0_keys[3][10] =
	{{KEY_Q, KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I,     KEY_O,      KEY_P},
    {KEY_A, KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_I,     KEY_L,      0},  		     // Need to figure out Function Layer thing.
	 {KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA, KEY_PERIOD, KEY_SHIFT}};

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

int main(void)
{
	uint8_t b,
	        f,
			  rowCount,
		     colCount,
			  reset_idle;
	uint8_t b_prev = 0xFF,
	        f_prev = 0xFF;

	// set for 16 MHz clock
	CPU_PRESCALE(0);

	// Set all input and output ports correctly
	DDRD  = 0x07;					// Rows 0 thru 2
	PORTD = 0x00;
	DDRB  = 0x00;  				// Cols 0 thru 4
	DDRF  = 0x00;              // Cols 5 thru 9

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	// Configure timer 0 to generate a timer overflow interrupt every
	// 256*1024 clock cycles, or approx 61 Hz when using 16 MHz clock
	// This demonstrates how to use interrupts to implement a simple
	// inactivity timeout.
	TCCR0A = 0x00;
	TCCR0B = 0x05;
	TIMSK0 = (1<<TOIE0);

	while (1) {

		// now we need a for loop, then we look at all three rows
		b = 0x00;
		f = 0x00;

		// read all port B and port F pins
		PORTD = 0x01;
		b |= PINB;
		f |= PINF;

		// check if any pins are low, but were high previously
		reset_idle = 0;

		for (colCount = 0; colCount < 5; colCount++) {
			if ((b & b_mask[colCount]) == 0  &&
			     b_prev & b_mask[colCount] != 0) {
				usb_keyboard_press(KEY_B, KEY_SHIFT);
				usb_keyboard_press(layer0_keys[0][colCount], 0);
				reset_idle = 1;
			}
			if ((f & f_mask[colCount]) == 0 &&
			     (f_prev & f_mask[colCount]) != 0) {
				usb_keyboard_press(KEY_F, KEY_SHIFT);
				usb_keyboard_press(layer0_keys[0][colCount], 0);
				reset_idle = 1;
			}
		}
		// if any keypresses were detected, reset the idle counter
		if (reset_idle) {
			// variables shared with interrupt routines must be
			// accessed carefully so the interrupt routine doesn't
			// try to use the variable in the middle of our access
			cli();
			idle_count = 0;
			sei();
		}
		// now the current pins will be the previous, and
		// wait a short delay so we're not highly sensitive
		// to mechanical "bounce".
		b_prev = b;
		f_prev = f;
		_delay_ms(2);
	}
}

// This interrupt routine is run approx 61 times per second.
// A very simple inactivity timeout is implemented, where we
// will send a space character.
ISR(TIMER0_OVF_vect)
{
	idle_count++;
	if (idle_count > 61 * 8) {
		idle_count = 0;
		usb_keyboard_press(KEY_SPACE, 0);
	}
}
