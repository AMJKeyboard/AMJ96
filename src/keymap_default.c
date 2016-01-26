#include "keymap_common.h"

#ifdef UART_RGB_ENABLE
#include "uart_rgb.h"
#endif



// Default
#ifdef KEYMAP_SECTION_ENABLE
const uint8_t keymaps[KEYMAPS_COUNT][MATRIX_ROWS][MATRIX_COLS] __attribute__ ((section (".keymap.keymaps"))) = {
#else
const uint8_t keymaps[][MATRIX_ROWS][MATRIX_COLS] PROGMEM = {
#endif
    /* Keymap 0: Default Layer
     * ,----------------------------------------------------------------------------.
     * |Esc| F1| F2| F3| F4| F5| F6| F7| F8| F9|F10|F11|F12|Psc|Pau||Ins|Del|PgU|PgD|
     * |-----------------------------------------------------------||---+---+---+---|
     * |  `|  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|Backsp ||Num|  /|  *| - |
     * |-----------------------------------------------------------||---+---+---+---|
     * |Tab  |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ]|    \||  7|  8|  9| + |
     * |-----------------------------------------------------------||---+---+---|---|
     * |Caps  |  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|Return  ||  4|  5|  6| + |
     * |-----------------------------------------------------------||---+---+---+---|
     * |Shift   |  Z|  X|  C|  V|  B|  N|  M|  ,|  .|  /|Shift |Up ||  1|  2|  3|   |
     * |-----------------------------------------------------------||---+---+---|Ent|
     * |Ctrl|Gui |Alt |         Space          |Alt |Fn0 |Left|Down||Rig| 0 |  .|   |
     * `----------------------------------------------------------------------------'
     */
    KEYMAP(
        ESC, FN1,  FN2,  FN3,  FN4,  FN5,  FN6,  FN7,  FN8,  FN9,  FN10, F11, F12, PSCR,PAUS,  INS, DEL, PGUP,PGDN, \
        GRV, 1,   2,   3,   4,   5,   6,   7,   8,   9,   0,   MINS,EQL, BSPC,NO,    NLCK,PSLS,PAST,PMNS, \
        TAB, Q,   W,   E,   R,   T,   Y,   U,   I,   O,   P,   LBRC,RBRC,     BSLS,  P7,  P8,  P9,  PPLS, \
        CAPS,A,   S,   D,   F,   G,   H,   J,   K,   L,   SCLN,QUOT,          ENT,   P4,  P5,  P6,  PPLS, \
        LSFT,NO, Z,   X,   C,   V,   B,   N,   M,   COMM,DOT, SLSH,      RSFT,UP,    P1,  P2,  P3,  PENT,\
        LCTL,LGUI,LALT,               SPC,                RALT,FN0,      LEFT,DOWN,  RGHT,P0, PDOT, PENT),
    /* Keymap 1: Fn Layer
     * ,----------------------------------------------------------------------------.
     * |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   ||   |   |   |   |
     * |-----------------------------------------------------------||---+---+---+---|
     * |   |   |   |   |   |   |   |   |   |   |   |   |   |       ||   |   |   |   |
     * |-----------------------------------------------------------||---+---+---+---|
     * |     |   |Up |   |   |   |Cal|   |Ins|   |Psc|Slk|Pau|     ||   |   |   |   |
     * |-----------------------------------------------------------||---+---+---|   |
     * |      |Lef|Dow|Rig|   |   |   |   |   |   |Hom|PgU|        ||   |   |   |   |
     * |-----------------------------------------------------------||---+---+---+---|
     * |        |   |App|Fn1|Fn2|Fn3|VoD|VoU|Mut|End|PgD|      |   ||   |   |   |   |
     * |-----------------------------------------------------------||---+---+---|   |
     * |    |    |    |                        |    |    |    |    ||   |   |   |   |
     * `----------------------------------------------------------------------------'
     */
    KEYMAP(
        TRNS,FN1,FN2,FN3,FN4,FN5,FN6,FN7,FN8,FN9,TRNS,TRNS,TRNS,TRNS,TRNS,  TRNS,TRNS,HOME,END,\
        TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,NO,    TRNS,TRNS,TRNS,TRNS,\
        TRNS,TRNS,UP,  TRNS,TRNS,TRNS,CALC,TRNS,INS, TRNS,PSCR,SLCK,PAUS,     TRNS,  TRNS,TRNS,TRNS,TRNS,\
        TRNS,LEFT,DOWN,RGHT,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,HOME,PGUP,          TRNS,  TRNS,TRNS,TRNS,TRNS,\
        TRNS,NO,  TRNS,APP, FN1, FN2, FN3, VOLD,VOLU,MUTE,END, PGDN,     TRNS,TRNS,  TRNS,TRNS,TRNS,TRNS,\
        TRNS,TRNS,TRNS,               TRNS,               TRNS,TRNS,     TRNS,TRNS,  TRNS,TRNS,TRNS,TRNS)
};

/*
 * Fn action definition
 */
#ifdef KEYMAP_SECTION_ENABLE
const uint16_t fn_actions[FN_ACTIONS_COUNT] __attribute__ ((section (".keymap.fn_actions"))) = {
#else
const uint16_t fn_actions[] PROGMEM = {
#endif
    [0] = ACTION_LAYER_MOMENTARY(1),
    [1] = ACTION_BACKLIGHT_DECREASE(),
    [2] = ACTION_BACKLIGHT_TOGGLE(),
    [3] = ACTION_BACKLIGHT_INCREASE(),
#ifdef UART_RGB_ENABLE
    [4] = ACTION_FUNCTION(UART_RGB_ON),
    [5] = ACTION_FUNCTION(UART_RGB_OFF),
    [6] = ACTION_FUNCTION(UART_RGB_LEVEL_DECREASE),
    [7] = ACTION_FUNCTION(UART_RGB_LEVEL_INCREASE),
    [8] = ACTION_FUNCTION(UART_RGB_COLORSET_DECREASE),
    [9] = ACTION_FUNCTION(UART_RGB_COLORSET_INCREASE),
    [10] = ACTION_FUNCTION(UART_RGB_TOGGLE),
#endif
};

#ifdef KEYMAP_IN_EEPROM_ENABLE
uint16_t keys_count(void) {
    return sizeof(keymaps) / sizeof(keymaps[0]) * MATRIX_ROWS * MATRIX_COLS;
}

uint16_t fn_actions_count(void) {
    return sizeof(fn_actions) / sizeof(fn_actions[0]);
}
#endif

#ifdef UART_RGB_ENABLE
void action_function(keyrecord_t *record, uint8_t id, uint8_t opt)
{
    if (record->event.pressed) {
        switch (id) {
            case UART_RGB_ON:
                uart_rgb_on();
                break;
            case UART_RGB_OFF:
                uart_rgb_off();
                break;
            case UART_RGB_TOGGLE:
                uart_rgb_toggle();
                break;
            case UART_RGB_LEVEL_INCREASE:
                uart_rgb_level_increase();
                break;
            case UART_RGB_LEVEL_DECREASE:
                uart_rgb_level_decrease();
                break;
            case UART_RGB_COLORSET_INCREASE:
                uart_rgb_colorset_increase();
                break;
            case UART_RGB_COLORSET_DECREASE:
                uart_rgb_colorset_decrease();
                break;
        }
    }
}
#endif

/*
 * Action macro definition
 */
enum macro_id {
    KEYPAD_00 = 0,
};

const macro_t *action_get_macro(keyrecord_t *record, uint8_t id, uint8_t opt)
{
    switch (id) {
        case KEYPAD_00:
            return (record->event.pressed ?
                    MACRO( T(P0), T(P0), END ) :
                    MACRO_NONE );
    }
    return MACRO_NONE;
}
