#include "LEDcontrol.h"
#include "stm32cube/gpio.h"
#include "../BttnCtl.h" // LED_state is defined in BttnCtl.h

LEDcontrol::LEDcontrol() {}

void LEDcontrol::set(uint8_t led_num, LED_state state)
{
    switch (state)
    {
    case LED_state::LED_on:
        _leds[led_num].on();
        _change_led_states(led_num, state); // On state not used here atm. here, but follow original code
        break;
    case LED_state::LED_off:
        _leds[led_num].off();
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
    _leds[led_num].toggle();
};

void LEDcontrol::_change_led_states(uint8_t led_num, LED_state state)
{
    _led_states_bin &= ~((uint64_t)(0b111) << (3 * led_num)); // Be safe for future LED_state changes and mask out the led
    _led_states_bin |= (uint64_t)(state) << (3 * led_num);    // Set state
}

bool LEDcontrol::is_led_state(uint8_t led_num, LED_state state)
{
    return (_led_states_bin >> (3 * led_num) & (uint64_t)(0b111)) == (uint64_t)(state);
}

void LEDcontrol::handle_blink_timer(LED_state state)
{
    if (state != LED_state::LED_blink_fast && state != LED_state::LED_blink_slow)
        return;
    for (uint8_t led_num = 0; led_num < NUM_LEDS; led_num++) // FIXME: Find some more efficient
    {
        if (is_led_state(led_num, state))
        {
            toggle(led_num);
        }
    }
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