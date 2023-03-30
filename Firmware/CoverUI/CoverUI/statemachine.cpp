/****************************************
 * statemachine.c
 * rev 0.0 El 2022-04-04
 * interface to the state machines
 * **************************************/

#define _serial_debug_


#include "statemachine.h"




/******************************************************************
*
* init and run statemachine flashing on boar LED asl alive signal
*
*******************************************************************/

/* FIXME

int  init_run_StateMachine_blink(PIO pio_Block)
        {
            int retval = -1;
            u_int freq = 1; // Blinkrate 1 Hz

            #ifdef _serial_debug_
            printf("\n\nStarting statemachine Blink");
            #endif

            // check whether programm can be loaded
            if (pio_can_add_program(pio_Block, &blink_program))
            {
                #ifdef _serial_debug_
                printf("\nPIO Program can be loadet");
                #endif
            }
            else
              {
                  #ifdef _serial_debug_
                  printf("\nPIO *** ERROR *** ont enought space to load Program ");
                  #endif
                  return(retval);
              }

            // get the offset = startadress in 32 bit Area for assembler code
            uint offset = pio_add_program(pio_Block, &blink_program);

            // find a free state machine
            uint sm  = pio_claim_unused_sm(pio_Block, true);
            #ifdef _serial_debug_
            printf("\nfound free Statemachine : %d  Startadress Programm : %d",sm,offset);
            #endif

            // run helperfunktion to load program into memory
            blink_program_init(pio_Block, sm, offset, LED_PIN, 125000);

            // run statemachine
            pio_sm_set_enabled(pio_Block, sm, true);

            // Blink forever
            #ifdef _serial_debug_
            printf("\nStatemachine blink running on pin: %d with %d  Hz", LED_PIN, freq);
            #endif



            // push frequency to statmachine
            pio_Block->txf[sm] = clock_get_hz(clk_sys) / (freq);
            pio_Block->txf[sm] = clock_get_hz(clk_sys) / (256 * freq);

            return (sm);
        }

*/

/******************************************************************
*
* init and run statemachine to control the buzzer
*
*******************************************************************/

/* FIXME
int init_run_StateMachine_buzzer(PIO pio_Block)
{
    int retval = -1;

#ifdef _serial_debug_
    printf("\n\nStarting statemachine buzzer");
#endif
    // check whether programm can bel loaded
    if (pio_can_add_program(pio_Block, &buzzer_program))
    {
#ifdef _serial_debug_
        printf("\nPIO Program can be loadet");
#endif
    }

    else
    {
#ifdef _serial_debug_
        printf("\nPIO *** ERROR *** ont enought space to load Program ");
#endif
        return (retval);
    }

    // get the offset = startadress in 32 bit Area for assembler code
    uint offset = pio_add_program(pio_Block, &buzzer_program);

    // find a free state machine
    uint sm = pio_claim_unused_sm(pio_Block, true);
#ifdef _serial_debug_
    printf("\nfound free Statemachine : %d  Startadress Programm : %d", sm, offset);
#endif

    // run helperfunktion to load program into memory
    buzzer_program_init(pio_Block, sm, offset, buzzer_Out_PIN, buzzer_SM_CYCLE);

    // run statemachine
    pio_sm_set_enabled(pio_Block, sm, true);

// Blink forever
#ifdef _serial_debug_
    printf("\nStatemachine buzzer running on pin: %d with %d  Hz", buzzer_Out_PIN, buzzer_SM_CYCLE);
#endif

    return (sm);
        }
*/



/******************************************************************
*
* init and run statemachine LED Mux
*
*******************************************************************/

/* FIXME
int init_run_StateMachine_LED_mux(PIO pio_Block)
        {
            
            int retval = -1;
            #ifdef _serial_debug_
            printf("\n\nStarting statemachine LED-MUX"); 
            #endif
            // check whether programm can bel loaded
            if (pio_can_add_program(pio_Block, &LED_mux_program)) 
            {
                #ifdef _serial_debug_
                printf("\nPIO Program can be loadet"); 
                #endif
            }
                
                else 
                {
                      #ifdef _serial_debug_                      
                      printf("\nPIO *** ERROR *** ont enought space to load Program ");
                      #endif
                      return(retval);
                }
            
            // get the offset = startadress in 32 bit Area for assembler code
            uint offset = pio_add_program(pio_Block, &LED_mux_program);

            // find a free state machine
            uint sm = pio_claim_unused_sm(pio_Block, true);

            #ifdef _serial_debug_
            printf("\nfound free Statemachine : %d  Startadress Programm : %d",sm,offset);

            printf("\nStatemachine LED-mux running with %d Hz",PIN_SIN, SM_CYCLE);
            #endif
            
            // init and start statemachine
            LED_mux_program_init(pio_Block, sm, offset, PIN_SIN, SM_CYCLE);
            return(sm);

        }
*/

