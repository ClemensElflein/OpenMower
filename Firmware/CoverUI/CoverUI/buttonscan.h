#ifndef _buttonscan_HEADER_FILE_
#define _buttonscan_HEADER_FILE_


#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/pio.h"

//include the state machine programs
#include "status_LED.pio.h" 
#include "LED_mux.pio.h" 
#include "buzzer.pio.h" 




/***********************************
 * buttonscan.h
 * rev 0.0 - El 2022-04-04
 * 
 * *********************************/



// assign output and input to hardware ports
#define out_buttonRow1      15
#define out_buttonRow2      16
#define out_buttonRow3      17
#define out_buttonRow4      18

#define in_buttonColumn1    19
#define in_buttonColumn2    20
#define in_buttonColumn3    21
#define in_buttonColumn4    22

#define HIGH                1
#define LOW                 0



    
void init_button_scan();        // Init Hardwareports

unsigned int bit_getbutton(uint32_t press_timeout, bool &still_pressed);   // scan for a pressed button, retrun button nr or 0.

unsigned int keypressed();      // Check for key pressed




#endif // _buttonscan_HEADER_FILE_
