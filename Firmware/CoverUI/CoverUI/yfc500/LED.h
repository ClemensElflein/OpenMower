#ifndef YFC500_LED_H
#define YFC500_LED_H

#include <stdint.h>
#include "stm32cube/inc/gpio.h"
#include "../BttnCtl.h" // Why is the LED_state defintion within Bttn?!

class LED
{
protected:
    GPIO_TypeDef *port;
    uint16_t pin;
    LED_state state;

public:
    LED(GPIO_TypeDef *port, uint16_t pin, LED_state state = LED_state::LED_off);
    
    void on();
    void off();
    void toggle(); // Toggle on->off or off->on
};

#endif /* YFC500_LED_H */
