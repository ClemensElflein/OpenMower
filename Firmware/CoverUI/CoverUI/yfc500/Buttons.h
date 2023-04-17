/**
 * @file Buttons.h
 * @author Apehaenger (joerg@ebeling.ws)
 * @brief YardForce Classic 500 CoverUI Buttons class for OpenMower https://github.com/ClemensElflein/OpenMower
 * @version 0.1
 * @date 2023-04-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef YFC500_BUTTONS_H
#define YFC500_BUTTONS_H

#include <stdint.h>
#include "stm32cube/inc/gpio.h"
#include "ButtonDebouncer.h"

// #include "../BttnCtl.h"

#define NUM_GPIO_PORTS 4
#define NUM_BUTTONS 14

class Buttons
{
private:
    // Somehow static initialization, but's not expected that the PCB will change anymore
    const GPIO_TypeDef *_gpio_ports[NUM_GPIO_PORTS] = {GPIOA, GPIOB, GPIOC, GPIOF}; // All ports with a button. Get continious scanned for filter/debounce by timer
    ButtonDebouncer *_debouncers[NUM_GPIO_PORTS] = {                          // Debouncer obj for each port in the same order as *_gpio_ports
        new ButtonDebouncer(), new ButtonDebouncer(),
        new ButtonDebouncer(), new ButtonDebouncer()};

    struct FYC500_Button_Def // Definition of a YFC500 button
    {
        uint8_t debouncer_index; // Debouncer index as defined in _debouncers array
        uint16_t button_pin;     // Mask which identifies a button
    };

    // Map OM button number to YFC500 Button-definiton (but starting at 0 and not 1)
    const FYC500_Button_Def button_nrs[NUM_BUTTONS] = {
        {3, 0b0000000000010000}, //  0 = BTN_CLK
        {0, 0b0001000000000000}, //  1 = BTN_HOME
        {0, 0b0000100000000000}, //  2 = BTN_PLAY
        {1, 0b0000000000000100}, //  3 = BTN_S1
        {1, 0b0000010000000000}, //  4 = BTN_S2
        {1, 0b0000100000000000}, //  5 = BTN_LOCK
        {3, 0b0000000000100000}, //  6 = BTN_OK
        {1, 0b0001000000000000}, //  7 = BTN_MON
        {1, 0b0010000000000000}, //  8 = BTN_TUE
        {1, 0b0100000000000000}, //  9 = BTN_WED
        {1, 0b1000000000000000}, // 10 = BTN_THU
        {2, 0b0000000001000000}, // 11 = BTN_FRI
        {2, 0b0000000010000000}, // 12 = BTN_SAT
        {2, 0b0000000100000000}  // 13 = BTN_SUN
    };

public:
    Buttons();

    void process_states();                           // Has to get called regulary i.e. by timer (2.5ms)
    uint16_t get_status(uint8_t gpio_index);         // Get status of all pins on the given GPIO index (as declared in *_gpio_ports)
    bool is_pressed_by_button_nr(uint8_t button_nr); // Return boolean if the given button number is pressed
};

#endif /* YFC500_BUTTONS_H */
