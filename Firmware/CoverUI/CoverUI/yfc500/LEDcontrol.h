#ifndef YFC500_LEDCONTROL_H
#define YFC500_LEDCONTROL_H

#include <stdint.h>
#include "stm32cube/inc/gpio.h"
#include "LED.h"

#define NUM_LEDS 19
#define LED_NUM_LIFTED 14
#define LED_NUM_WIRE 15
#define LED_NUM_BAT 16
#define LED_NUM_CHARGE 17

class LEDcontrol
{
private:
    LED _leds[NUM_LEDS] = {
        LED(LED_2HR_GPIO_Port, LED_2HR_Pin), // 1st (4 piece) row
        LED(LED_4HR_GPIO_Port, LED_4HR_Pin),
        LED(LED_6HR_GPIO_Port, LED_6HR_Pin),
        LED(LED_8HR_GPIO_Port, LED_8HR_Pin),
        LED(LED_S1_GPIO_Port, LED_S1_Pin), // 2nd row
        LED(LED_S2_GPIO_Port, LED_S2_Pin),
        LED(LED_LOCK_GPIO_Port, LED_LOCK_Pin),
        LED(LED_MON_GPIO_Port, LED_MON_Pin), // 3rd (7 piece) row
        LED(LED_TUE_GPIO_Port, LED_TUE_Pin),
        LED(LED_WED_GPIO_Port, LED_WED_Pin),
        LED(LED_THU_GPIO_Port, LED_THU_Pin),
        LED(LED_FRI_GPIO_Port, LED_FRI_Pin),
        LED(LED_SAT_GPIO_Port, LED_SAT_Pin),
        LED(LED_SUN_GPIO_Port, LED_SUN_Pin),
        LED(LED_LIFTED_GPIO_Port, LED_LIFTED_Pin), // 4th row
        LED(LED_WIRE_GPIO_Port, LED_WIRE_Pin),
        LED(LED_BAT_GPIO_Port, LED_BAT_Pin),
        LED(LED_CHARGE_GPIO_Port, LED_CHARGE_Pin),
        LED(LED_SMD_GPIO_Port, LED_SMD_Pin) // LED 19 = SMD LED which seem not to exist on OM-CoverUI
    };

    uint64_t _led_states_bin = 0; // each LED gets three bits for the current state (see BtnCtrl.h)
    void _change_led_states(uint8_t led_num, LED_state state);

public:
    LEDcontrol();

    void animate();                           // A short LED Animation
    void handle_blink_timer(LED_state state); // Get called by responsible blink timer
    bool is_led_state(uint8_t led_num, LED_state state);
    void set(uint8_t led_num, LED_state state = LED_state::LED_off); // Set any of known LED_state states for a specific LED
    void set(uint64_t all_state);                                    // Set any of known LED_state states for all LEDs by binary state value
    void toggle(uint8_t led_num);                                    // Toggle on->off or off->on
};

#endif /* YFC500_LEDCONTROL_H */
