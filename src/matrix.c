/*
Copyright 2014 Kai Ryu <kai1103@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * scan matrix
 */
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"
#ifdef UART_RGB_ENABLE
#include "uart_rgb.h"
#endif

#ifndef DEBOUNCE
#   define DEBOUNCE 5
#endif
static uint8_t debouncing = DEBOUNCE;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t matrix_debouncing[MATRIX_ROWS];

static matrix_row_t read_cols(void);
static void init_cols(void);
static void init_rows(void);
static void unselect_rows(void);
static void select_row(uint8_t row);

inline
uint8_t matrix_rows(void)
{
    return MATRIX_ROWS;
}

inline
uint8_t matrix_cols(void)
{
    return MATRIX_COLS;
}

void matrix_init(void)
{

#ifdef UART_RGB_ENABLE
uart_rgb_init();
#endif
    // disable JTAG
    MCUCR = _BV(JTD);
    MCUCR = _BV(JTD);

    // 85 REST
    DDRD |= _BV(PD7);
    PORTD |= _BV(PD7);

    // initialize row and col
    init_rows();
    init_cols();

    // initialize matrix state: all keys off
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
        matrix_debouncing[i] = 0;
    }
}

uint8_t matrix_scan(void)
{
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        select_row(i);
        _delay_us(30);  // without this wait read unstable value.
        matrix_row_t cols = read_cols();
        if (matrix_debouncing[i] != cols) {
            matrix_debouncing[i] = cols;
            if (debouncing) {
                debug("bounce!: "); debug_hex(debouncing); debug("\n");
            }
            debouncing = DEBOUNCE;
        }
        unselect_rows();
    }

    if (debouncing) {
        if (--debouncing) {
            _delay_ms(1);
        } else {
            for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
                matrix[i] = matrix_debouncing[i];
            }
        }
    }

    return 1;
}

bool matrix_is_modified(void)
{
    if (debouncing) return false;
    return true;
}

inline
bool matrix_is_on(uint8_t row, uint8_t col)
{
    return (matrix[row] & ((matrix_row_t)1<<col));
}

inline
matrix_row_t matrix_get_row(uint8_t row)
{
    return matrix[row];
}

void matrix_print(void)
{
    print("\nr/c 0123456789ABCDEF\n");
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        phex(row); print(": ");
        pbin_reverse16(matrix_get_row(row));
        print("\n");
    }
}

uint8_t matrix_key_count(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        count += bitpop16(matrix[i]);
    }
    return count;
}

/* Column pin configuration
 * col: 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17
 * pin: C6  C7  A7  A6  A5  A3  A4  A2  A1  A0  F7  F6  F5  F4  F3  F2  F1  F0
 * */
static void  init_cols(void)
{
    // Input with pull-up(DDR:0, PORT:1)
    DDRA  = 0b00000000;
    PORTA = 0b11111111;
    DDRF  = 0b00000000;
    PORTF = 0b11111111;
    DDRC  &= ~(_BV(PC6) | _BV(PC7));
    PORTC |= (_BV(PC6) | _BV(PC7));

}

static matrix_row_t read_cols(void)
{

    return (PINC&_BV(PC6) ? 0 : (1UL<<0)) |
           (PINC&_BV(PC7) ? 0 : (1UL<<1)) |
           (PINA&_BV(PA7) ? 0 : (1UL<<2)) |
           (PINA&_BV(PA6) ? 0 : (1UL<<3)) |
           (PINA&_BV(PA5) ? 0 : (1UL<<4)) |
           (PINA&_BV(PA3) ? 0 : (1UL<<5)) |
           (PINA&_BV(PA4) ? 0 : (1UL<<6)) |
           (PINA&_BV(PA2) ? 0 : (1UL<<7)) |
           (PINA&_BV(PA1) ? 0 : (1UL<<8)) |
           (PINA&_BV(PA0) ? 0 : (1UL<<9)) |
           (PINF&_BV(PF7) ? 0 : (1UL<<10)) |
           (PINF&_BV(PF6) ? 0 : (1UL<<11)) |
           (PINF&_BV(PF5) ? 0 : (1UL<<12)) |
           (PINF&_BV(PF4) ? 0 : (1UL<<13)) |
           (PINF&_BV(PF3) ? 0 : (1UL<<14)) |
           (PINF&_BV(PF2) ? 0 : (1UL<<15)) |
           (PINF&_BV(PF1) ? 0 : (1UL<<16)) |
           (PINF&_BV(PF0) ? 0 : (1UL<<17));
}

/* Row pin configuration
 * row: 0   1   2   3   4  5
 * pin: C0  C1  C2  C3  C4 C5
 */
static void init_rows(void)
{
    unselect_rows();
}

static void unselect_rows(void)
{
    // Hi-Z(DDR:0, PORT:0) to unselect
    DDRC  &= ~0b00111111;
    PORTC &= ~0b00111111;
}

static void select_row(uint8_t row)
{
    // Output low(DDR:1, PORT:0) to select
    DDRC |= (1<<(5-row));
    PORTC &= ~(1<<(5-row));
}
