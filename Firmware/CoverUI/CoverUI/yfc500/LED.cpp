#include "LED.h"
#include "stm32cube/gpio.h"

LED::LED(GPIO_TypeDef *set_port, uint16_t set_pin, LED_state set_state)
{
    port = set_port;
    pin = set_pin;
    state = set_state;
}

void LED::on()
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void LED::off()
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void LED::toggle()
{
    HAL_GPIO_TogglePin(port, pin);
}