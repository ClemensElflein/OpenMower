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

/* --- PRINTF_BYTE_TO_BINARY macro's --- */
#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i) (((i)&0x80ll) ? '1' : '0'), (((i)&0x40ll) ? '1' : '0'), (((i)&0x20ll) ? '1' : '0'), (((i)&0x10ll) ? '1' : '0'), \
                                      (((i)&0x08ll) ? '1' : '0'), (((i)&0x04ll) ? '1' : '0'), (((i)&0x02ll) ? '1' : '0'), (((i)&0x01ll) ? '1' : '0')
#define PRINTF_BINARY_PATTERN_INT16 PRINTF_BINARY_PATTERN_INT8 PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BYTE_TO_BINARY_INT16(i) PRINTF_BYTE_TO_BINARY_INT8((i) >> 8), PRINTF_BYTE_TO_BINARY_INT8(i)
/* --- end PRINTF_BYTE_TO_BINARY macros --- */

// STM32CubeIDE code is only used for HW definition and initialization.
// There's no user defined code within STM32Cube specific files except some handy #define's.
// By this it should be easily possible to change/enhance HW definitions via STM32CubeIDE.
// Simply copy STM32CubeIDE files (except main.c) to the stm32cube directory  without further modifications.
#include "stm32cube/inc/main.h"
#include "stm32cube/inc/dma.h"
#include "stm32cube/inc/tim.h"
#include "stm32cube/inc/usart.h"
#include "stm32cube/inc/gpio.h"

#include "sysclock.hpp"
#include "LEDcontrol.h"
#include "Buttons.h"

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

// Some dummy Pico-SDK definitions. Not used but by this we'll NOT pollution original code to much
#define pio0 NULL
#define pio1 NULL
typedef bool *PIO;
#define buzzer_SM_CYCLE 10800

// Main LED controller object
LEDcontrol LedControl;
Buttons Btns;

/**
 * @brief Init all HW relevant stuff like GPIO, DMA, U(S)ARTs, Timer and Semihosting
 *
 */
void init_mcu()
{
    HAL_Init();           // Reset of all peripherals, Initializes the Flash interface and the Systick
    SystemClock_Config(); // Configure the system clock

    // Initialize required peripherals
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init(); // to LL-Pico via J6
    MX_TIM16_Init();       // Blink-slow timer
    MX_TIM17_Init();       // Blink-fast timer
    MX_TIM6_Init();        // Button debounce timer

#ifdef DEBUG_SEMIHOSTING
    initialise_monitor_handles(); // Semihosting
#endif
}

void start_peripherals()
{
    // Start all timer
    HAL_TIM_Base_Start_IT(&HTIM_BLINK_SLOW);
    HAL_TIM_Base_Start_IT(&HTIM_BLINK_FAST);
    HAL_TIM_Base_Start_IT(&HTIM_BTN);

    // Start UART-LL DMA receive
    if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&HUART_LL, uart_ll_dma_buffer, UART_LL_DMA_BUFFSIZE))
    {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(&HDMA_UART_LL_RX, DMA_IT_HT); // Disable "Half Transfer" interrupt
}

/**
 * @brief OM wrapper
 *
 * @param led_num
 * @param force
 */
void Force_LED_off(uint8_t led_num, bool force)
{
    LedControl.force_off(led_num, force);               // This only effect blink states
}

/**
 * @brief OM wrapper
 *
 * @param pioBlock
 * @param statemachine
 * @param led
 */
void Blink_LED(PIO dummy, int dummy2, int led_num)
{
    LedControl.identify(led_num);
}

/**
 * @brief OM wrapper
 * 
 * @param anz 
 * @param timeON 
 * @param timeOFF 
 */
static inline void buzzer_program_put_words(PIO pio, uint sm, uint32_t repeat, uint32_t duration, uint32_t gap)
{
    // YFC500 doesn't has a buzzer on CoverUI
}

/**
 * @brief OM wrapper for original function call to STM's Buttons class implementation
 *
 * @param press_timeout
 * @param still_pressed
 * @return unsigned int
 */
uint8_t bit_getbutton(uint32_t press_timeout, bool &still_pressed)
{
    still_pressed = false;

    // As it's not clear ATM how the implementation will become,
    // let's scan the buttons in the same order as original OM FW does
    const uint8_t scan_order[] = {0, 1, 2, 3, 4, 5, 6, 13, 7, 8, 9, 10, 11, 12}; // Attention: Ours = OM's -1. See Buttons.h: FYC500_Button_Def for index number
    for (uint8_t i : scan_order)
    {
        uint32_t start = HAL_GetTick(); // start press_timeout measurement
        if (Btns.is_pressed_by_button_nr(i))
        {
            // wait for button released
            while (Btns.is_pressed_by_button_nr(i) && (press_timeout == 0 || (HAL_GetTick() - start) < press_timeout))
                ;
            if (Btns.is_pressed_by_button_nr(i))
                still_pressed = true;
            else
                still_pressed = false;
            return (i + 1); // OM's button numbering seem to start at n > 0
        }
    }
    return 0;
}

/**
 * @brief Timer callbacks.
 *  TIM_BLINK_SLOW (LED blink- slow) is configure at 500ms
 *  TIM_BLINK_FAST (LED blink- fast) is configured at 100ms
 *  TIM_BTN (button debouncing) is configured at 2.5ms
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM_BLINK_SLOW)
    {
        LedControl.blink_timer_elapsed(LED_state::LED_blink_slow);
#ifdef _serial_debug_
        printf("Button status " PRINTF_BINARY_PATTERN_INT16 " " PRINTF_BINARY_PATTERN_INT16 " " PRINTF_BINARY_PATTERN_INT16 " " PRINTF_BINARY_PATTERN_INT16 "\n",
               PRINTF_BYTE_TO_BINARY_INT16(Btns.get_status(0)), PRINTF_BYTE_TO_BINARY_INT16(Btns.get_status(1)), PRINTF_BYTE_TO_BINARY_INT16(Btns.get_status(2)), PRINTF_BYTE_TO_BINARY_INT16(Btns.get_status(3)));
#endif
    }
    else if (htim->Instance == TIM_BLINK_FAST)
        LedControl.blink_timer_elapsed(LED_state::LED_blink_fast);
    else if (htim->Instance == TIM_BTN)
        Btns.process_states(); // Read all defined gpio-port debouncer
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
