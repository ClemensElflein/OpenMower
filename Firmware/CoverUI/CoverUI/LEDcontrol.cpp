/****************************************
 * LEDcontrol.c
 * rev 0.0 El 2022-04-04
 * deals with the LEDs on the mowerboard
 * **************************************/

#include "LEDcontrol.h"

#define HIGH 1
#define LOW 0

// each LED gets three bits for the current state (see BtnCtrl.h)
uint64_t LED_activity = 0;
uint32_t force_led_on = 0;
uint32_t force_led_off = 0;

/*********************************************
 *
 * init LED status array
 *
 **********************************************/

void init_LED_activity()
{
    // Set all LEDs to off
    LED_activity = 0;
    force_led_on = 0;
    force_led_off = 0;
}

void LED_animation(PIO pioBlock, int statemachine)
{
    LED_activity = 0;
    int on_leds = 4;

    for (int led = 0; led < 18; led++)
    {
        LED_activity ^= (uint64_t)(0b111) << (3 * led);
        LEDs_refresh(pioBlock, statemachine);
        sleep_ms(15);
    }

    for (int led = 0; led < 18; led++)
    {
        LED_activity ^= (uint64_t)(0b111) << (3 * led);
        LEDs_refresh(pioBlock, statemachine);
        sleep_ms(15);
    }

    LED_activity = 0;
    LEDs_refresh(pioBlock, statemachine);
}

/**
 * @brief Transmits the current state of the LEDs to the PIO output. This currently only supports ON, OFF and the blinking state, no dimming
 *
 * @param fast true, if we're in a "fast" blinking time slot
 * @param slow true, if we're in a "slow" blinking time slot
 * @param pioBlock the pio block for the state machine
 * @param statemachine the statemachine inside the block
 */
void LEDs_refresh(PIO pioBlock, int statemachine)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    bool fast = (now / 100) & 1;
    bool slow = (now / 500) & 1;

    // Bitmask for the PIO
    uint32_t LED_mirror = 0;
    for (uint led = 0; led < 18; led++)
    {
        bool force_on = (force_led_on >> led) & 1;
        bool force_off = (force_led_off >> led) & 1;
        // fetch the current state of the LED
        uint8_t led_state = (LED_activity >> (3 * led)) & 0b111;
        if (!force_off)
        {
            if (led_state == LED_on || force_on)
                LED_mirror = LED_mirror | 0b00000000000001000000000000000000;
            else if ((led_state == LED_blink_fast) && fast)
                LED_mirror = LED_mirror | 0b00000000000001000000000000000000;
            else if ((led_state == LED_blink_slow) && slow)
                LED_mirror = LED_mirror | 0b00000000000001000000000000000000;
        }
        LED_mirror = LED_mirror >> 1;
    }

    // Write the current state to the state machine
    pioBlock->txf[statemachine] = LED_mirror;
}

void Blink_LED(PIO pioBlock, int statemachine, int led)
{

    printf("flashing led %d\n", led);
    Force_LED_off(led, false);
    Force_LED_on(led, true);
    for (int i = 0; i < 10; i++)
    {
        LEDs_refresh(pioBlock, statemachine);
        sleep_ms(10);
    }
    Force_LED_off(led, true);
    Force_LED_on(led, false);
    for (int i = 0; i < 10; i++)
    {
        LEDs_refresh(pioBlock, statemachine);
        sleep_ms(10);
    }
    // stop with forced off
    Force_LED_off(led, true);
    Force_LED_on(led, false);
}

void Force_LED_off(int led, bool force)
{
    // TODO might allow more than one LED also, but we don't need it
    if (force)
    {
        force_led_off = 1 << led;
    }
    else
    {
        force_led_off = 0;
    }
}
void Force_LED_on(int led, bool force)
{
    // TODO might allow more than one LED also, but we don't need it
    if (force)
    {
        force_led_on = 1 << led;
    }
    else
    {
        force_led_on = 0;
    }
}