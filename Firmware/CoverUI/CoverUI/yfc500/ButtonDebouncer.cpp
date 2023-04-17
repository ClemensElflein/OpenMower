/**
 * @file ButtonDebouncer.cpp
 * @author Apehaenger (joerg@ebeling.ws)
 * @brief YardForce Classic 500 CoverUI Button-Debouncer class for OpenMower https://github.com/ClemensElflein/OpenMower
 * @version 0.1
 * @date 2023-04-13
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <stdio.h>
#include "ButtonDebouncer.h"
#include "stm32cube/inc/gpio.h"
// #include "../BttnCtl.h" // LED_state is defined in BttnCtl.h

ButtonDebouncer::ButtonDebouncer() {}

void ButtonDebouncer::process_state(const GPIO_TypeDef *gpio_port)
{
    uint8_t i;
    uint16_t last_state_debounced = _state_debounced;
    _states[_state_index] = (uint16_t)gpio_port->IDR ^ 0xFFFF; // XOR changes for pull-up _states

    // Debounce
    for (i = 0, _state_debounced = 0xFFFF; i < NUM_BUTTON_STATES; i++)
        _state_debounced &= _states[i];

    // Circular buffer index
    _state_index++;
    if (_state_index >= NUM_BUTTON_STATES)
        _state_index = 0;

    // Save what changed
    _state_changed = _state_debounced ^ last_state_debounced;
}

uint16_t ButtonDebouncer::get_pressed()
{
    return (_state_changed & _state_debounced);
}

uint16_t ButtonDebouncer::get_status()
{
    return _state_debounced;
}