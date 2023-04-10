/**
 * @file LEDcontrol.cpp
 * @author Apehaenger (joerg@ebeling.ws)
 * @brief YardForce Classic 500 CoverUI driver for OpenMower https://github.com/ClemensElflein/OpenMower
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

void LEDcontrol::set(uint8_t led_num, LED_state state)
{
    switch (state)
    {
    case LED_state::LED_on:
        HAL_GPIO_WritePin(_leds[led_num].port, _leds[led_num].pin, GPIO_PIN_SET);
        _change_led_states(led_num, state); // On state not used here atm. here, but follow original code
        break;
    case LED_state::LED_off:
        HAL_GPIO_WritePin(_leds[led_num].port, _leds[led_num].pin, GPIO_PIN_RESET);
        _change_led_states(led_num, state); // Off state not used here atm. here, but follow original code
        break;
    case LED_state::LED_blink_slow:
    case LED_state::LED_blink_fast:
        _change_led_states(led_num, state); // used by TIM_BLINK_* timer
        break;
    }
}

void LEDcontrol::set(uint64_t all_state)
{
    for (uint8_t led = 0; led < NUM_LEDS; led++)
    {
        uint8_t led_state = (all_state >> (3 * led)) & 0b111;
        set(led, static_cast<LED_state>(led_state));
    }
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

bool LEDcontrol::is_led_state(uint8_t led_num, LED_state state)
{
    return (_led_states_bin >> (3 * led_num) & (uint64_t)(0b111)) == (uint64_t)(state);
}

void LEDcontrol::blink_timer_elapsed(LED_state blink_state)
{
    // Sync blink vars are only for cosmetic nature, probably only interesting for a nice looking CoverUITest
    static std::map<LED_state, GPIO_PinState> sync_blink_map = {
        {LED_state::LED_blink_slow, GPIO_PIN_SET},
        {LED_state::LED_blink_fast, GPIO_PIN_SET}};

    if (blink_state != LED_state::LED_blink_fast && blink_state != LED_state::LED_blink_slow) // Ensure that this method only get called for blinking LED states
        return;

    for (uint8_t led_num = 0; led_num < NUM_LEDS; led_num++) // FIXME: Find some more efficient instead of looping through all NUM_LEDS
    {
        if (is_led_state(led_num, blink_state))
        {
            // toggle() may blink in push-pull
            // toggle(led_num);
            // Directly set LED without going over set() which would change the LED_state
            HAL_GPIO_WritePin(_leds[led_num].port, _leds[led_num].pin, sync_blink_map[blink_state]);
        }
    }
    // Synchronous toggle
    if (sync_blink_map[blink_state] == GPIO_PIN_SET)
        sync_blink_map[blink_state] = GPIO_PIN_RESET;
    else
        sync_blink_map[blink_state] = GPIO_PIN_SET;
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