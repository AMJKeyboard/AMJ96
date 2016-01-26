#ifdef UART_RGB_ENABLE
#include "uart_rgb.h"
#include "print.h"

static uint8_t mode_id  = 1;
const uint8_t MODE_MAX = 12;
static uint8_t color_id  = 1;
const uint8_t COLOR_MAX = 9;
static uint8_t reduce_id  = 1;
const uint8_t REDUCE_MAX = 10;

void uart_rgb_init(void) {

    // 16M 9600
    DDRD  |= _BV(PD3);
    UBRR1H = 0;
    UBRR1L = 103;

    UCSR1A &= ~_BV(U2X1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
    UCSR1B |= _BV(TXEN1);
    print("\n");
}

bool uart_rgb_send_is_ready(void) {
    return ((UCSR1A & (1 << UDRE1)) ? true : false);
}

void uart_rgb_send_byte(uint8_t byte_data) {
    print_hex8(byte_data);
    print("\n");
    while (!(uart_rgb_send_is_ready()));
    UDR1 = byte_data;
}

void uart_rgb_on(void){
    print("UART RGN ON(16): ");
    uart_rgb_send_byte(0x16);
}

void uart_rgb_off(void) {
    print("UART RGB OFF(26): ");
    uart_rgb_send_byte(0x26);
}

void uart_rgb_toggle(void) {
    print("UART RGB REDUCE: ");
    if (reduce_id == REDUCE_MAX) {
        reduce_id = 1;
    }
    else {
        reduce_id ++;
    }

    uint8_t cmd = (reduce_id << 4) + 4;
    uart_rgb_send_byte(cmd);
}


void uart_rgb_level_increase(void){
    print("UART MODE INC: ");
    if ( mode_id < MODE_MAX){
        mode_id ++;
    }
    uint8_t cmd = (mode_id << 4) + 3;
    uart_rgb_send_byte(cmd);
}

void uart_rgb_level_decrease(void){
    print("UART MODE DEC: ");
    if ( mode_id > 1){
        mode_id --;
    }
    uint8_t cmd = (mode_id << 4) + 3;
    uart_rgb_send_byte(cmd);
}

void uart_rgb_colorset_increase(void){
    print("UART COLORSET INC: ");
    if ( color_id < COLOR_MAX){
        color_id ++;
    }
    uint8_t cmd = (color_id << 4) + 2;
    uart_rgb_send_byte(cmd);
}

void uart_rgb_colorset_decrease(void){
    print("UART COLORSET DEC: ");
    if ( color_id > 1){
        color_id --;
    }
    uint8_t cmd = (color_id << 4) + 2;
    uart_rgb_send_byte(cmd);
}
#endif
