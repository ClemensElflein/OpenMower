/**
 * @file LEDcontrol.cpp
 * @author Apehaenger (joerg@ebeling.ws)
 * @brief YardForce Classic 500 CoverUI LED driver for OpenMower https://github.com/ClemensElflein/OpenMower
 * @version 0.1
 * @date 2023-04-10
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <map>
#include "LEDcontrol.h"
#include "stm32cube/inc/gpio.h"
#include "../BttnCtl.h" // LED_state is defined in BttnCtl.h

LEDcontrol::LEDcontrol() {}

/**
 * @brief Set any of known LED_state states for the given LED num.
 *
 * @param led_num
 * @param state
 * @param change_state Indicate if the state get written to _led_states_bin buffer.
 *      This got implemented for synchronous blinking LEDs as well as Force_LED_on|off() replacement.
 */
void LEDcontrol::set(uint8_t led_num, LED_state state, bool change_state)
{
    switch (state)
    {
    case LED_state::LED_on:
        HAL_GPIO_WritePin(_leds[led_num].port, _leds[led_num].pin, GPIO_PIN_SET);
        break;
    case LED_state::LED_off:
        HAL_GPIO_WritePin(_leds[led_num].port, _leds[led_num].pin, GPIO_PIN_RESET);
        break;
    case LED_state::LED_blink_slow:
    case LED_state::LED_blink_fast:
        // Get handled by timer
        break;
    }
    if (change_state)
        _change_led_states(led_num, state);
}

void LEDcontrol::set(uint64_t all_state)
{
    for (uint8_t led = 0; led < NUM_LEDS; led++)
    {
        uint8_t led_state = (all_state >> (3 * led)) & 0b111;
        set(led, static_cast<LED_state>(led_state));
    }
}

LED_state LEDcontrol::get(uint8_t led_num)
{
    return (LED_state)((_led_states_bin >> (3 * led_num)) & 0b111);
}

void LEDcontrol::_force(uint8_t led_num, bool force, uint32_t *_force_type)
{
    uint32_t led_bin = 1 << led_num;

    if (force)
        *_force_type |= led_bin;
    else
        *_force_type &= ~led_bin;
}

void LEDcontrol::force_off(uint8_t led_num, bool force)
{
    _force(led_num, force, &_force_led_off);
    if (force)
        set(led_num, LED_state::LED_off, false); // Directly set without changing state
    else
        set(led_num, get(led_num), false); // Restore state
}

void LEDcontrol::force_on(uint8_t led_num, bool force)
{
    _force(led_num, force, &_force_led_on);
    if (force)
        set(led_num, LED_state::LED_on, false); // Directly set without changing state
    else
        set(led_num, get(led_num), false); // Restore state
}

void LEDcontrol::toggle(uint8_t led_num)
{
    HAL_GPIO_TogglePin(_leds[led_num].port, _leds[led_num].pin);
};

void LEDcontrol::_change_led_states(uint8_t led_num, LED_state state)
{
    _led_states_bin &= ~((uint64_t)(0b111) << (3 * led_num)); // Be safe for future LED_state changes and mask out the whole led
    _led_states_bin |= (uint64_t)(state) << (3 * led_num);    // Set new state
}

bool LEDcontrol::has_state(uint8_t led_num, LED_state state)
{
    return (_led_states_bin >> (3 * led_num) & (uint64_t)(0b111)) == (uint64_t)(state);
}

void LEDcontrol::blink_timer_elapsed(LED_state blink_state)
{
    // Sync blink vars are only for cosmetic nature, probably only interesting for a nice looking CoverUITest
    static std::map<LED_state, LED_state> sync_blink_map = {
        {LED_state::LED_blink_slow, LED_state::LED_on},
        {LED_state::LED_blink_fast, LED_state::LED_on}};

    if (blink_state != LED_state::LED_blink_fast && blink_state != LED_state::LED_blink_slow) // Ensure that this method only get called for blinking LED states
        return;

    for (uint8_t led_num = 0; led_num < NUM_LEDS; led_num++) // FIXME: Find some more efficient instead of looping through all NUM_LEDS
    {
        if (has_state(led_num, blink_state) && !(_force_led_off & (1 << led_num)))
        {
            // toggle() might blink in push-pull/reverse than other LEDs with the same blink-rate
            // toggle(led_num);
            // Directly set LED without changing state
            set(led_num, sync_blink_map[blink_state], false);
        }
    }
    // Synchronous toggle
    if (sync_blink_map[blink_state] == LED_state::LED_on)
        sync_blink_map[blink_state] = LED_state::LED_off;
    else
        sync_blink_map[blink_state] = LED_state::LED_on;
}

void LEDcontrol::animate()
{
    for (int led = 0; led < NUM_LEDS; led++)
    {
        set(led, LED_state::LED_on);
        HAL_Delay(15);
    }

    for (int led = 0; led < NUM_LEDS; led++)
    {
        set(led, LED_state::LED_off);
        HAL_Delay(15);
    }
}

/**
 * @brief Identify LED by short blink code
 * 
 * @param led_num 
 */
void LEDcontrol::identify(uint8_t led_num)
{
    force_off(led_num, false);
    force_on(led_num, true);
    HAL_Delay(100);

    force_off(led_num, true);
    force_on(led_num, false);
    HAL_Delay(100);

    // stop with forced off
    force_off(led_num, true); // FIXME: This doesn't stop force off!
    force_on(led_num, false);
}
