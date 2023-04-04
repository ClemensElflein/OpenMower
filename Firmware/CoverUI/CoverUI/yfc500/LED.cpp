#include "LED.h"
#include "stm32cube/gpio.h"

LED::LED(GPIO_TypeDef *setPort, uint16_t setPin, LED_state setState)
{
    port = setPort;
    pin = setPin;
    state = setState;
}

void LED::set(bool on)
{
    if (on)
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void LED::toggle()
{
    HAL_GPIO_TogglePin(port, pin);
}