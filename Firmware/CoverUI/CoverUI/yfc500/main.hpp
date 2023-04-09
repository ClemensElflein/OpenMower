#ifndef __YFC500_MAIN_H
#define __YFC500_MAIN_H

// STM32CubeIDE code is only used for HW definition and initialization.
// There's no user defined code within STM32Cube specific files except some handy #define's.
// By this it should be easily possible to change/enhance HW definitions via STM32CubeIDE.
// Simply copy STM32CubeIDE files (except main.c) to the stm32cube directory  without further modifications.
#include "stm32cube/inc/main.h"
#include "yfc500/stm32cube/inc/dma.h"
#include "yfc500/stm32cube/inc/tim.h"
#include "yfc500/stm32cube/inc/usart.h"
#include "yfc500/stm32cube/inc/gpio.h"
#include "yfc500/sysclock.hpp"
#include "yfc500/stm32cube/inc/stm32f0xx_it.h"
#include "yfc500/LEDcontrol.h"
#include "yfc500/ring_buffer.hpp"

#ifdef DEBUG_SEMIHOSTING
extern "C" void initialise_monitor_handles(void);
#endif

// Current STM32F030 implementation is single core without threads.
// Send mutex calls to nirvana
#define auto_init_mutex(name)
#define mutex_enter_blocking(ptr)
#define mutex_exit(ptr)

void initMCU()
{
    HAL_Init();           // Reset of all peripherals, Initializes the Flash interface and the Systick
    SystemClock_Config(); // Configure the system clock

    // Initialize required peripherals
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_TIM16_Init(); // TIM_BLINK_SLOW
    HAL_TIM_Base_Start_IT(&htim16);
    MX_TIM17_Init(); // TIM_BLINK_FAST
    HAL_TIM_Base_Start_IT(&htim17);

#ifdef DEBUG_SEMIHOSTING
    initialise_monitor_handles(); // Semihosting
#endif
}

#endif /* __YFC500_MAIN_H */
