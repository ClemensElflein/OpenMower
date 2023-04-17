#ifndef __YFC500_ERROR_H
#define __YFC500_ERROR_H

#include "main.h"
#include "LEDcontrol.h"

extern LEDcontrol LedControl;

/**
 * @brief  This function is executed in case of error occurrence.
 *         TODO: Add some kind of morse code if there will be errors
 * @retval None
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Can't use timers anymore as we're in faulty state. Dummy delay loop
        for (float i = 0; i < 0xFFF; i++) // something < 100ms
            ;
        LedControl.toggle(LED_NUM_REAR);
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#endif /* __YFC500_ERROR_H */