/****************************************
 * buttonscan.c
 * rev 0.0 El 2022-04-04
 * get button pressed
 * **************************************/


#include "buttonscan.h"



/******************************************************************
*
* init and Hardware, define input, output ans set output to init values
*
*******************************************************************/


void init_button_scan()
        {


            // Init gpios
       
            // Button output row
            /* FIXME
                        gpio_init(out_buttonRow1);
                        gpio_init(out_buttonRow2);
                        gpio_init(out_buttonRow3);
                        gpio_init(out_buttonRow4);
                        // Button output column
                        gpio_init(in_buttonColumn1);
                        gpio_init(in_buttonColumn2);
                        gpio_init(in_buttonColumn3);
                        gpio_init(in_buttonColumn4);

                        // set port direction
                        // Button Matrix out
                        gpio_set_dir(out_buttonRow1, GPIO_OUT);
                        gpio_set_dir(out_buttonRow2, GPIO_OUT);
                        gpio_set_dir(out_buttonRow3, GPIO_OUT);
                        gpio_set_dir(out_buttonRow4, GPIO_OUT);

                        // Button Matrix in
                        gpio_set_dir(in_buttonColumn1, GPIO_IN);
                        gpio_set_dir(in_buttonColumn2, GPIO_IN);
                        gpio_set_dir(in_buttonColumn3, GPIO_IN);
                        gpio_set_dir(in_buttonColumn4, GPIO_IN);


                        // pull down inputs
                        gpio_pull_down(in_buttonColumn1);
                        gpio_pull_down(in_buttonColumn2);
                        gpio_pull_down(in_buttonColumn3);
                        gpio_pull_down(in_buttonColumn4);



                        // Init output row level to Low
                        gpio_put(out_buttonRow1, LOW);
                        gpio_put(out_buttonRow2, LOW);
                        gpio_put(out_buttonRow3, LOW);
                        gpio_put(out_buttonRow4, LOW);
*/
                }







            /******************************************************************
            *
            * set the output, read input an check whether button is pressed, return buttonnr
            *
            *******************************************************************/

            unsigned int bit_getbutton(uint32_t press_timeout, bool &still_pressed)
            {
                still_pressed = false;
                unsigned int retval = 0;
                // converts button nr to linear order
                // cross reference Hardware position to logical position in one column
                unsigned int buttonNr[15] = {0, 1, 2, 3, 4, 5, 6, 7, 14, 8, 9, 10, 11, 12, 13};

                for (int i = out_buttonRow1; i <= out_buttonRow4; i++)
                {
                    retval = 0;
                    /* FIXME
                    gpio_put(i, HIGH);
                    sleep_ms(1);
                    for (int j = in_buttonColumn1; j <= in_buttonColumn4; j++)
                    {
                        if (gpio_get(j))
                        {
                            // start time measurement
                            uint32_t start = to_ms_since_boot(get_absolute_time());
                            // wait because button oszillation
                            sleep_ms(1);
                            if (gpio_get(j))
                            {
                                retval = buttonNr[(i - 15) * 4 + (j - 18)];
                                // wait for button released
                                while (gpio_get(j) && (press_timeout == 0 || (to_ms_since_boot(get_absolute_time()) - start) < press_timeout))
                                    ;
                                if (gpio_get(j))
                                {
                                    still_pressed = true;
                                }
                                else
                                {
                                    still_pressed = false;
                                }
                                gpio_put(i, LOW);
                                return (retval);
                            }
                        }
                    }

                    gpio_put(i, LOW);
                    sleep_ms(1);*/
                }
                return (0);

                // todo reactivate keypressed

    }





    // check for key pressed, don wait until released returns  number of the first key seen pressed
    unsigned int keypressed()
    {
        unsigned int retval = 0; 

        // converts button nr to linear order
        // cross reference Hardware position to logical position in one column
        unsigned int buttonNr[15]= {0 , 1, 2, 3, 4, 5, 6, 7, 14, 8, 9, 10, 11, 12, 13};

       
       for (int i = out_buttonRow1; i <= out_buttonRow4; i++)
       {
           retval = 0;
           /* FIXME
gpio_put(i, HIGH);
           sleep_ms(5);
           for (int j = in_buttonColumn1; j <= in_buttonColumn4; j++)
           {
               if (gpio_get(j))
               {
                  // wait 10 ms because button oszillation
                  sleep_ms(10);
                  if (gpio_get(j)) 
                  {
                    retval = buttonNr[(i-15)*4 + (j-18)];
                    gpio_put(i, LOW);
                    return (retval);
                  }  
                  
               }
               gpio_put(i, LOW);
           }*/
           
          
          
       }
       
       return(0);

    }

