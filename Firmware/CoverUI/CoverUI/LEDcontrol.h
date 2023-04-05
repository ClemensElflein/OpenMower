#ifndef _LEDcontrol_HEADER_FILE_
#define _LEDcontrol_HEADER_FILE_


#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "BttnCtl.h"



/***********************************
 * LEDcontrol.h
 * rev 0.0 - El 2022-04-04
 *
 * *********************************/

// define the Ports
#define out_LED_SIN         10
#define out_LED_BLANK       11
#define out_LED_LAT         12
#define out_LED_CLK         13

extern uint64_t LED_activity;




void init_LED_activity(void);               // Init LED array all LEDs off
void LED_animation(PIO pioBlock, int statemachine);
void LEDs_refresh(PIO pioBlock, int statemachine);  // refresh pio state machine
    void Blink_LED(PIO pioBlock, int statemachine, int led);
    void Force_LED_off(int led, bool force);
    void Force_LED_on(int led, bool force);


#endif // _LEDcontrol_HEADER_FILE_
