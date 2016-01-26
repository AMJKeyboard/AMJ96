
#ifdef UART_RGB_ENABLE
#include <LUFA/Drivers/Peripheral/Serial.h>

#define UART_RGB_ON 0x00F1
#define UART_RGB_OFF 0x00F2
#define UART_RGB_TOGGLE 0x00F3
#define UART_RGB_LEVEL_INCREASE 0x00F4
#define UART_RGB_LEVEL_DECREASE 0x00F5
#define UART_RGB_COLORSET_INCREASE 0x00F6
#define UART_RGB_COLORSET_DECREASE 0x00F7

void uart_rgb_init(void);
bool uart_rgb_send_is_ready(void);
void uart_rgb_send_byte(uint8_t byte_data);

void uart_rgb_on(void);
void uart_rgb_off(void);
void uart_rgb_toggle(void);
void uart_rgb_level_increase(void);
void uart_rgb_level_decrease(void);
void uart_rgb_colorset_increase(void);
void uart_rgb_colorset_decrease(void);

#endif
