// control to print out serial information in debug state via usb serial
#define _serial_debug_

#include <stdio.h>

#ifdef HW_YFC500

// STM32CubeIDE files are only used for HW definition and initialization.
// There's no user defined code within STM32Cube specific files except "#include yfc500/main.h" ??? FIXME
// within "yfc500/stm32cube/main.h" which act as main.h for CoverUI's main.c
// By this it's easily possible to change STM32Cube for HW definition,
// copy STM32Cube files to this repo's yfc500/stm32cube dir without further modifications.
// This is mainy to hold this repos's main.cpp and other files clean and untouched (as much as possible)
#include "yfc500/stm32cube/main.h"
#include "yfc500/stm32cube/dma.h"
#include "yfc500/stm32cube/tim.h"
#include "yfc500/stm32cube/usart.h"
#include "yfc500/stm32cube/gpio.h"
#include "yfc500/stm32cube/error.hpp"
#include "yfc500/stm32cube/sysclock.hpp"
#include "yfc500/stm32cube/stm32f0xx_it.h"
#include "yfc500/LED.h"
#ifdef DEBUG_SEMIHOSTING
extern "C" void initialise_monitor_handles(void);
#endif
extern DMA_HandleTypeDef hdma_usart2_rx; // UART_LL
#define RX_BUFFER_SIZE 1000              // FIXME: This look huge. See "#define bufflen 1000"
uint8_t uart_ll_rx_buffer[RX_BUFFER_SIZE];
// Misc Pico-SDK stuff
#define auto_init_mutex(name) // Don't have threads

#else // HW Pico

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

// 2nd UART
#define UART_1 uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#endif

#include <cstdio>
#include "COBS.h"
#ifdef CRC // i.e. by STM32Cube
#undef CRC
#endif
// FIXME STM32: should use the STM32 included crc unit
#include "CRC.h"

#include <cstring>

#include "LEDcontrol.h"
#include "statemachine.h"
#include "buttonscan.h"
#include "BttnCtl.h"

#define bufflen 1000

#define FIRMWARE_VERSION 200
// V 2.00 v. 11.10.2022 new protocol implementation for less messages on the bus

// buzzer ontime in ms
#define shortbeep 50

// write & read pointer serial interrupt buffer
static uint write = 0;

// serial buffer
static uint8_t buffer_serial[bufflen];
// count values in COBS-Buffer
static uint8_t decoded_buffer[bufflen];
// buffer for encoded output messages
static uint8_t out_buf[bufflen];

COBS cobs;

// defintion PIO-block and uses statemachines
/* FIXME
PIO pio_Block1 = pio0;
PIO pio_Block2 = pio1;
int sm_blink;  // Statemachine onboard LED blinking
int sm_LEDmux; // Statemachine control LEDSs
int sm_buzz;   // Statemachine control Buzzer

auto_init_mutex(mx1); */

/****************************************************************************************************
 *
 * interface to the statemachine, control buzzer
 *
 *****************************************************************************************************/

void Buzzer_set(uint32_t anz, uint32_t timeON, uint32_t timeOFF)
{
#ifndef HW_YFC500
  mutex_enter_blocking(&mx1);
  buzzer_program_put_words(pio_Block2, sm_buzz, anz, timeON * buzzer_SM_CYCLE / 4000, timeOFF);
  mutex_exit(&mx1);
#endif
}

/****************************************************************************************************
 *
 * sendMessage send the command strucutre via COBS and serial Port and wait for reply
 *
 *****************************************************************************************************/

void sendMessage(void *message, size_t size)
{
  // FIXME: mutex_enter_blocking(&mx1);

  // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
  if (size < 4)
  {
    // FIXME: mutex_exit(&mx1);
    return;
  }

  uint8_t *data_pointer = (uint8_t *)message;
  uint16_t *crc_pointer = (uint16_t *)(data_pointer + (size - 2));

  // calculate the CRC
  *crc_pointer = CRC::Calculate(message, size - 2, CRC::CRC_16_CCITTFALSE());
  // structure is filled and CRC calculated, so print out, what should be encoded

#ifdef _serial_debug_
  printf("\nprint struct before encoding %d byte : ", (int)size);
  uint8_t *temp = data_pointer;
  for (int i = 0; i < size; i++)
  {
    printf("0x%02x , ", *temp);
    temp++;
  }
#endif

  // encode message

  size_t encoded_size = cobs.encode((uint8_t *)message, size, out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

#ifdef _serial_debug_
  printf("\nencoded message              %d byte : ", (int)encoded_size);
  for (uint i = 0; i < encoded_size; i++)
  {
    printf("0x%02x , ", out_buf[i]);
  }
#endif

#ifdef HW_YFC500
  HAL_UART_Transmit_DMA(&huart2, out_buf, encoded_size);
#else // HW Pico
  for (uint i = 0; i < encoded_size; i++)
  {
    uart_putc(UART_1, out_buf[i]);
  }
  mutex_exit(&mx1);
#endif
}

/****************************************************************************************************
 *
 * PacketReceived() there is a serial stream in the buffer
 * decode it with cobs, calc crc and decode into the messagestruct
 *****************************************************************************************************/

void PacketReceived()
{
  size_t data_size = cobs.decode(buffer_serial, write - 1, (uint8_t *)decoded_buffer);

  // calculate the CRC only if we have at least three bytes (two CRC, one data)
  if (data_size < 3)
    return;

  uint16_t calc_crc = CRC::Calculate(decoded_buffer, data_size - 2, CRC::CRC_16_CCITTFALSE());

  // struct mower_com *struct_CRC = (struct mower_com *) encoded_buffer;

  if (decoded_buffer[0] == Get_Version && data_size == sizeof(struct msg_get_version))
  {
    struct msg_get_version *message = (struct msg_get_version *)decoded_buffer;
    if (message->crc == calc_crc)
    {
      // valid get_version request, send reply
      struct msg_get_version reply;
      reply.type = Get_Version;
      reply.version = FIRMWARE_VERSION;
      sendMessage(&reply, sizeof(reply));
    }
  }
  else if (decoded_buffer[0] == Set_Buzzer && data_size == sizeof(struct msg_set_buzzer))
  {
    struct msg_set_buzzer *message = (struct msg_set_buzzer *)decoded_buffer;
    if (message->crc == calc_crc)
    {
      // valid set_buzzer request
      Buzzer_set(message->repeat, message->on_time, message->off_time);
    }
  }
  else if (decoded_buffer[0] == Set_LEDs && data_size == sizeof(struct msg_set_leds))
  {
    struct msg_set_leds *message = (struct msg_set_leds *)decoded_buffer;
    if (message->crc == calc_crc)
    {
      // valid set_leds request
      printf("Got valid setled call\n");
      // FIXME: mutex_enter_blocking(&mx1);
      LED_activity = message->leds;
      // FIXME: LEDs_refresh(pio_Block1, sm_LEDmux);
      // FIXME: mutex_exit(&mx1);
    }
    else
    {
      printf("Got setled call with crc error\n");
    }
  }
  else
  {
    printf("some invalid packet\n");
  }

#ifdef _serial_debug_
  printf("packet received with %d bytes : ", (int)data_size);
  uint8_t *temp = decoded_buffer;
  for (int i = 0; i < data_size; i++)
  {
    printf("0x%02x , ", *temp);
    temp++;
  }
  printf("\n");
#endif
}

/****************************************************************************************************
 *
 * getDataFromBuffer() read bytes from serial interrupt controlled buffer and couple a string with endcode "00"
 * out to buffer_serial
 *****************************************************************************************************/

#ifdef HW_YFC500
void getDataFromBuffer(uint16_t size)
{
  for (uint16_t i = 0; i < size; i++)
  {
    u_int8_t readbyte = uart_ll_rx_buffer[i];
#else // HW Pico
void getDataFromBuffer()
{
  while (uart_is_readable(UART_1))
  {
    u_int8_t readbyte = uart_getc(UART_1);
#endif
    buffer_serial[write] = readbyte;
    write++;
    if (write >= bufflen)
    {
      // buffer is full, but no separator. Reset
      write = 0;
      return;
    }
    if (readbyte == 0)
    {
      // we have found the packet marker, notify the other core
      PacketReceived();
      write = 0;
      return;
    }
  }
}

int getLedForButton(int button)
{
  if (button >= 4 && button <= 6)
    return 13 - (button - 4);
  if (button >= 8 && button <= 14)
    return 10 - (button - 8);
  return 0;
}

void core1()
{
  printf("Core 1 started\n");
  while (true)
  {
    // Scan Buttons
    bool pressed = false;
    unsigned int button = 0;
    int pressed_count = 0;
    bool allow_hold;

    button = bit_getbutton(1, pressed);

    // Buttons where we allow the hold. The reason for this is, that we have an indicator LED for these buttons.
    allow_hold = (button >= 4 && button <= 6) || (button >= 8 && button <= 14);

    /* FIXME
        if (button && allow_hold)
        {
          int led = getLedForButton(button);
          // force led off
          mutex_enter_blocking(&mx1);
          Force_LED_off(led, true);
          mutex_exit(&mx1);
          do
          {
            // allow hold, wait for user to hold
            button = bit_getbutton(1000, pressed);
            if (pressed)
            {
              // user indeed held the button
              pressed_count++;
              // if button is still held, we flash and rety
              for (int i = 0; i < pressed_count; i++)
              {
                mutex_enter_blocking(&mx1);
                Blink_LED(pio_Block1, sm_LEDmux, led);
                mutex_exit(&mx1);
              }
            }
          } while (pressed && button != 0 && pressed_count < 2);

          // yes that's a duplicate but we want to wait before releasing the LED, so that's intended.
          if (button && pressed)
          {
            // we're still holding, wait for the button to release. We don't care about the result here.
            bool tmp;
            bit_getbutton(0, tmp);
          }

          mutex_enter_blocking(&mx1);
          Force_LED_off(led, false);
          mutex_exit(&mx1);
        }
        else
        {
          if (button && pressed)
          {
            // we're still holding, wait for the button to release. We don't care about the result here.
            bool tmp;
            bit_getbutton(0, tmp);
          }
        }

        if (button > 0)
        {

          struct msg_event_button button_msg;
          button_msg.type = Get_Button;
          button_msg.button_id = button;
          button_msg.press_duration = pressed_count;

          sendMessage(&button_msg, sizeof(button_msg));

          // confirm pressed button with buzzer.
          // TODO: on rapid button presses, the FIFO gets full and therefore this call is blocking for a long time.
          mutex_enter_blocking(&mx1);
          buzzer_program_put_words(pio_Block2, sm_buzz, 1, shortbeep * buzzer_SM_CYCLE / 4000, 40);
          mutex_exit(&mx1);

          printf("\n\rsend Button Nr.: %d with count %d", button, pressed_count);
        }*/
  }
}

int main(void)
{
  // uint32_t last_led_update = 0;

#ifdef HW_YFC500
#ifdef DEBUG_SEMIHOSTING
  initialise_monitor_handles(); // Semihosting
#endif
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

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_ll_rx_buffer, RX_BUFFER_SIZE); // UART_LL DMA buffer
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);                         // Disable "Half Transfer" interrupt

  printf("Main started\n");

  int blink = 0;
  uint8_t cnt = 0;

  LED TestLed = LED(LED_S1_GPIO_Port, LED_S1_Pin);
#else

  stdio_init_all();

  // setup the second UART 1
  uart_init(UART_1, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  // no flow control
  uart_set_hw_flow(UART_1, false, false);
  uart_set_fifo_enabled(UART_1, true);

#endif

  /* FIXME
  init_button_scan(); // Init hardware for button matix
  // init_LED_driver();
  */

  float ver = (float)FIRMWARE_VERSION / 100.0;
  printf("\n\n\n\rMower Button-LED-Control Version %2.2f\n", ver);

  /* FIXME
  // initialise state machines
  sm_blink = init_run_StateMachine_blink(pio_Block1);    // on board led alive blink
  sm_LEDmux = init_run_StateMachine_LED_mux(pio_Block1); // LED multiplexer
  sm_buzz = init_run_StateMachine_buzzer(pio_Block2);    // Beeper control

  //FIXME: mutex_enter_blocking(&mx1);
  init_LED_activity();                 // init LED statusmatrix all LEDs off
  LEDs_refresh(pio_Block1, sm_LEDmux); // LED-state to Hardware
  mutex_exit(&mx1);
  sleep_ms(1000);

  // LED blink to say it's alive
  LED_animation(pio_Block1, sm_LEDmux);

  // buzzer say hello
  Buzzer_set(1, 50, 40);

  // Check for button ins permanently pressed e.g. while mounting

  u_int32_t n = keypressed();
  while (n > 0)
  {
    mutex_enter_blocking(&mx1);
    LED_activity = -1;
    LEDs_refresh(pio_Block1, sm_LEDmux);
    mutex_exit(&mx1);
    Buzzer_set(n, 250, 100);

    sleep_ms(10000);
    n = keypressed();
  }

  mutex_enter_blocking(&mx1);
  LED_activity = 0;
  LEDs_refresh(pio_Block1, sm_LEDmux);
  mutex_exit(&mx1);

  // enable other core for button detection
  multicore_reset_core1();
  multicore_launch_core1(core1);

  printf("\n\n waiting for commands or button press");
*/

  printf("On\n");
  TestLed.set(true);
  HAL_Delay(1000);
  printf("Off\n");
  TestLed.set(false);

  while (true)
  {
#ifdef HW_YFC500
  // STM UART get read by DMA+INT
#else // HW Pico
    getDataFromBuffer();
#endif

    /* FIXME
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Update the LEDs and their blinking states
    if (now - last_led_update > 10)
    {
      mutex_enter_blocking(&mx1);
      LEDs_refresh(pio_Block1, sm_LEDmux);
      mutex_exit(&mx1);
      last_led_update = now;
    }*/
  }
}

#ifdef HW_YFC500

/**
 * UART_LL DMA Callback once data is ready/idle
 *
 * @param huart
 * @param Size
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == UART_LL)
  {
    getDataFromBuffer(Size);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_ll_rx_buffer, RX_BUFFER_SIZE); // Start UART DMA again
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);                         // Disable "Half Transfer" interrupt
  }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  // printf("UART TX Half callback\n");
  //  FIXME: Probably need to ensure that there's no TX overflow
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  // printf("UART TX full callback\n");
  //  FIXME: Probably need to ensure that there's no TX overflow
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM_BLINK_SLOW)
  {
    HAL_GPIO_TogglePin(LED_2HR_GPIO_Port, LED_2HR_Pin);
  }
  else if (htim->Instance == TIM_BLINK_FAST)
  {
    HAL_GPIO_TogglePin(LED_4HR_GPIO_Port, LED_4HR_Pin);
  }
}

#endif