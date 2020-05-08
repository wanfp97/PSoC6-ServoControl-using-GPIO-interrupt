/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cycfg.h"
#include "cyhal.h"

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void button_intr_handler(void *handler_arg, cyhal_gpio_event_t event);

/*******************************************************************************
* Global Variables
********************************************************************************/
volatile bool button_intr_flag = false;

int main(void)
{
    /* Initialize the device and board peripherals */

    __enable_irq(); // enable interrupt

    init_cycfg_all(); // initialize all the configuration done in Device Configurator
	Cy_TCPWM_PWM_Init(tcpwm1_HW, tcpwm1_NUM, &tcpwm1_config); // initialize tcpwm1
    Cy_TCPWM_PWM_Enable(tcpwm1_HW, tcpwm1_NUM); // enable tcpwm1
    Cy_TCPWM_TriggerStart(tcpwm1_HW, tcpwm1_MASK); //Start the tcpwm1 (initially 0.4ms high per cycle)(right)
    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(button_HAL_PORT_PIN,
                                 button_intr_handler, NULL); // assigning isr handler
    cyhal_gpio_enable_event(button_HAL_PORT_PIN, CYHAL_GPIO_IRQ_BOTH,
                                 1u, true); // interrupt on both rising and falling edge of the button
    for (;;)
    {
    	/* Check the interrupt status */
		if (true == button_intr_flag) // if interrupt happens (button pressed or released)
		{
			button_intr_flag = false; // clear interrupt flag
			if(1UL == Cy_GPIO_Read(button_PORT, button_NUM)) // if button is pressed
			{
				Cy_GPIO_Write(green_PORT, green_NUM, 1UL); // green LED on
				Cy_GPIO_Write(red_PORT, red_NUM, 0UL);	// red LED off
				Cy_TCPWM_TriggerCaptureOrSwap(tcpwm1_HW, 1UL); // tcpwm1 swap to 2.4ms(left) high per cycle
			}
			else
			{
				Cy_GPIO_Write(green_PORT, green_NUM, 0UL); // green LED off
				Cy_GPIO_Write(red_PORT, red_NUM, 1UL);	// red LED on
				Cy_TCPWM_TriggerCaptureOrSwap(tcpwm1_HW, 1UL); // tcpwm1 swap to 0.4ms(right) high per cycle
			}

		}
    }
}

/*******************************************************************************
* Function Name: button_intr_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
*******************************************************************************/
static void button_intr_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    button_intr_flag = true;
}
/* [] END OF FILE */
