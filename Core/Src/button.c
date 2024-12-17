/* Includes ------------------------------------------------------------------*/
#include "button.h"

/**
 * @brief Button debounce.
 *
 * Function eliminates the influence of contact flickering upon pressing a button (debouncing). 
 * Usage:
 * @code
 * uint8_t oldstate;  // Old state flag
 * void main() {
 *	  
 *
 *  do {
 *    if(Button(GPIOB, GPIO_PIN_3, 1, 0)) { // Detect logical one
 *      oldstate = 1;                              // Update flag
 *    }
 *    if (oldstate && !Button(GPIOB, GPIO_PIN_4, 0)) {   // Detect one-to-zero transition
 *      //"PB4 was pressed";
 *      oldstate = 0;                              // Update flag
 *    }
 *  } while(1);                                    // Endless loop
 * }
 * @endcode
 * @param ui32Port GPIO where button is connected, using the macro from ST (GPIOX)
 * @param ui32Pin  Pin number on designated port from the gpio.h pin definitions (GPIO\_PIN\_X) 
 * @param active_state can be either 0 or 1, and it determines if the button is active upon logical zero or logical one
 * @return 255 if the pin was in the active state for given period, 0 otherwise
 */
uint16_t button(GPIO_TypeDef* ui32Port, uint16_t ui16Pin, uint16_t active_state)
{
    //lê o botão 'tries' vezes durante 'time' milissegundos
    //o botão deve passar pelo menos a metade final do período em active_state
    uint32_t i,tries = 5;
    uint8_t pressed = 0, unpressed = 0;
    uint16_t ret = 0;
    
    
    for(i=0;i<tries;i++)
    {
        
        if(HAL_GPIO_ReadPin(ui32Port,ui16Pin))
        {
            pressed++;
            unpressed=0;
        }
        else
        {
            unpressed++;
            pressed = 0;
        }
        HAL_Delay(1);
    }
    if(active_state)
    {
        if (pressed>(tries/2))
        {
            ret = 0xFF;
        }
    }
    else
    {
        if (unpressed>(tries/2))
        {
            ret = 0xFF;
        }
    }    
    return ret;
}

/* Includes ------------------------------------------------------------------*/
#include "button.h"

/**
 * @brief Button debounce and release detect.
 *
 * Function eliminates the influence of contact flickering upon pressing a button (debouncing). 
 * When a button press is detected, the function only returns when the button is released.
 *  
 * Usage:
 * @code
 * uint8_t oldstate;  // Old state flag
 * void main() {
 *	  
 *
 *  do {
 *    if(button_release(GPIOB, GPIO_PIN_3, 1, 0)) { 
 *          //button was pressed and released
 *    }
 *  } while(1);                                    // Endless loop
 * }
 * @endcode
 * @param ui32Port GPIO where button is connected, using the macro from ST (GPIOX)
 * @param ui32Pin  Pin number on designated port from the gpio.h pin definitions (GPIO\_PIN\_X) 
 * @param active_state can be either 0 or 1, and it determines if the button is active upon logical zero or logical one
 * @return 255 if the pin was in the active state for given period and then released, 0 otherwise
 */
uint16_t button_release(GPIO_TypeDef* ui32Port, uint16_t ui16Pin, uint16_t active_state)
{
    if(button(ui32Port,ui16Pin,active_state))
    {
        if(active_state)
        {
            while (HAL_GPIO_ReadPin(ui32Port, ui16Pin))
                ;
        }
        else
        {
            while (!HAL_GPIO_ReadPin(ui32Port, ui16Pin))
                ;
        }
        return 0xFF;
    }
    else
    {
        return 0;
    }    
}

