/**
 * @file Buttons.cpp
 * @author Apehaenger (joerg@ebeling.ws)
 * @brief YardForce Classic 500 CoverUI Buttons class for OpenMower https://github.com/ClemensElflein/OpenMower
 * @version 0.1
 * @date 2023-04-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "Buttons.h"
#include "ButtonDebouncer.h"

Buttons::Buttons() {}

void Buttons::process_states()
{
    for (uint8_t i = 0; i < NUM_GPIO_PORTS; i++)
        _debouncers[i]->process_state(_gpio_ports[i]);
}

/**
 * @brief Get (debounced) status of the given GPIO index.
 * See Buttons.h: _gpio_ports for the GPIO indexes.
 *
 * @param gpio_index
 * @return uint16_t
 */
uint16_t Buttons::get_status(uint8_t gpio_index)
{
    return _debouncers[gpio_index]->get_status();
}

/**
 * @brief Return boolean if the given button number is pressed.
 *        Take into notice that the returned state is already debounced.
 *
 * @param button_nr
 * @return true
 * @return false
 */
bool Buttons::is_pressed_by_button_nr(uint8_t button_nr){
    return get_status(button_nrs[button_nr].debouncer_index) & button_nrs[button_nr].button_pin;
};