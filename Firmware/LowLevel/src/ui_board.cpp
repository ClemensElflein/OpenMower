#include "ui_board.h"

void setLed(struct msg_set_leds &msg, int led, uint8_t state) {
    // mask the current led state
    uint64_t mask = ~(((uint64_t)0b111) << (led*3));
    msg.leds &= mask;
    msg.leds |= ((uint64_t)(state&0b111)) << (led*3);
}

void setBars7(struct msg_set_leds &msg, double value) {
    int on_leds = round(value * 7.0);
    for(int i = 0; i < 7; i++) {
        setLed(msg, LED11-i, i < on_leds ? LED_on : LED_off);
    }
}

void setBars4(struct msg_set_leds &msg, double value) {
    if(value < 0) {
        for(int i = 0; i < 4; i++) {
            setLed(msg, i+LED15, LED_blink_fast);
        }
    } else {
        int on_leds = round(value * 4.0);
        for(int i = 0; i < 4; i++) {
            setLed(msg, LED18-i, i < on_leds ? LED_on : LED_off);
        }
    }
}