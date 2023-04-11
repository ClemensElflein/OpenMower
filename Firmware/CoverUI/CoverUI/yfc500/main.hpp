/**
 * @file main.hpp
 * @author Apehaenger (joerg@ebeling.ws)
 * @brief YardForce Classic 500 CoverUI main header for OpenMower https://github.com/ClemensElflein/OpenMower
 * @version 0.1
 * @date 2023-04-11
 *
 * @copyright Copyright (c) 2023
 *
 */
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
#include "yfc500/stm32cube/inc/stm32f0xx_it.h"

#include "yfc500/sysclock.hpp"
#include "yfc500/LEDcontrol.h"

#ifdef DEBUG_SEMIHOSTING
extern "C" void initialise_monitor_handles(void);
#endif

// Current STM32F030 implementation is single core without threads.
// Send mutex calls to nirvana. Dangerous?
#define auto_init_mutex(name)
#define mutex_enter_blocking(ptr)
#define mutex_exit(ptr)

// UART-LL specific settings
#define UART_LL_DMA_BUFFSIZE 50
uint8_t uart_ll_dma_buffer[UART_LL_DMA_BUFFSIZE]; // DMA buffer size has to be at least one COBS cmd length
extern DMA_HandleTypeDef HDMA_UART_LL_RX;
extern void getDataFromBuffer(const uint8_t *data, uint16_t size); // defined in main.cpp

// Main LED controller object
LEDcontrol LedControl;

/**
 * @brief Init all HW relevant stuff like GPIO, DMA, U(S)ARTs, Timer and Semihosting
 *
 */
void initMCU()
{
    HAL_Init();           // Reset of all peripherals, Initializes the Flash interface and the Systick
    SystemClock_Config(); // Configure the system clock

    // Initialize required peripherals
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init(); // to LL-Pico via J6
    MX_USART1_UART_Init(); // to onBoard U4
    MX_TIM16_Init();       // TIM_BLINK_SLOW
    HAL_TIM_Base_Start_IT(&HTIM_BLINK_SLOW);
    MX_TIM17_Init(); // TIM_BLINK_FAST
    HAL_TIM_Base_Start_IT(&HTIM_BLINK_FAST);

#ifdef DEBUG_SEMIHOSTING
    initialise_monitor_handles(); // Semihosting
#endif
}

/**
 * @brief Timer callbacks. Currently only for LED blink- slow and fast.
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM_BLINK_SLOW)
        LedControl.blink_timer_elapsed(LED_state::LED_blink_slow);
    else if (htim->Instance == TIM_BLINK_FAST)
        LedControl.blink_timer_elapsed(LED_state::LED_blink_fast);
}

/**
 * @brief Start UART-LL (LL-Pico) receive (Idle-DMA), which in turn will trigger HAL_UARTEx_RxEventCallback() on receive
 *
 */
void start_uart_LL_DMA_receive()
{
    if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&HUART_LL, uart_ll_dma_buffer, UART_LL_DMA_BUFFSIZE))
    {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(&HDMA_UART_LL_RX, DMA_IT_HT); // Disable "Half Transfer" interrupt
}

/**
 * @brief RX idle callback which has to process the data within the circular DMA buffer.
 *        Take attention when enabling the printf() debugging. Semihosting will burn to much CPU for reliable (>1-2 circled) buffer processing!!!
 *
 * @param huart UART handle
 * @param pos Position of last rx data within buffer
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos)
{
    static uint16_t old_ll_pos;
    uint16_t size;

    if (huart->Instance == UART_LL)
    {
        if (pos == old_ll_pos) // no new data
            return;

        if (pos > old_ll_pos) // buffer simply increased without going over end of buffer
        {
            size = pos - old_ll_pos;
            // printf("\nDMA(incr) old_ll_pos %u, pos %u = size %u", old_ll_pos, pos, size);
            getDataFromBuffer(&uart_ll_dma_buffer[old_ll_pos], size);
        }
        else // pos < old_ll_pos => buffer pos circled
        {
            // Remaining end of circular buffer
            size = UART_LL_DMA_BUFFSIZE - old_ll_pos;
            // printf("\nDMA(rest) old_ll_pos %u, pos %u = size %u", old_ll_pos, pos, size);
            if (size > 0) // Not really necessary, could also call getDataFromBuffer() with a size of 0
            {
                getDataFromBuffer(&uart_ll_dma_buffer[old_ll_pos], size);
            }
            // Start of circular buffer
            // printf("\nDMA(incr) old_ll_pos %u, pos %u = size %u", old_ll_pos, pos, pos);
            if (pos > 0) // Not really necessary, could also call getDataFromBuffer() with a size of 0
            {
                getDataFromBuffer(&uart_ll_dma_buffer[0], pos);
            }
        }
        old_ll_pos = pos;
    }
}

#endif /* __YFC500_MAIN_H */
