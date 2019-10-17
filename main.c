/******************************************************************************
* File Name: main.c
*
* Description: This code example demonstrates the use of GPIO configured as an
*              input pin to generate interrupts in PSoC 6 MCU.
*
* Related Document: README.md
*
******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
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
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
********************************************************************************/


/******************************************************************************
* Header files 
******************************************************************************/
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include "cyhal.h"
#include "cybsp.h"


/******************************************************************************
 * Global constants
 *****************************************************************************/
#define DELAY_SHORT             (250)   /* milliseconds */
#define DELAY_LONG              (500)   /* milliseconds */
#define LED_BLINK_COUNT         (4)
#define GPIO_INTERRUPT_PRIORITY (7u)


/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);


/*******************************************************************************
* Global Variables
********************************************************************************/
bool gpio_intr_flag = false;


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function configures and initializes the GPIO
*  interrupt, update the delay on every GPIO interrupt, blinks the LED and enter
*  in deepsleep mode.
*
* Return: int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t res;
    uint32_t count = 0;
    uint32_t delay_ms = DELAY_LONG;

    /* initialize the resources that are located on the board and any sources
     * setup via configurator
     */
    res = cybsp_init();
    if (res != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0u);
    }

    /* Initialize retargeting standard IO to the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
                        CY_RETARGET_IO_BAUDRATE);

    /* Initialize the user LED */
    cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize the user button */
    cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_BTN1, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback((cyhal_gpio_t)CYBSP_USER_BTN1, 
                                 gpio_interrupt_handler, NULL);
    cyhal_gpio_enable_event((cyhal_gpio_t)CYBSP_USER_BTN1, CYHAL_GPIO_IRQ_FALL, 
                                 GPIO_INTERRUPT_PRIORITY, true);

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("**************** PSoC 6 MCU: GPIO Interrupt *****************\r\n");

    while(1)
    {
        /* Check the interrupt status */
        if(true == gpio_intr_flag)
        {
            gpio_intr_flag = false;

            /* Update LED toggle delay */
            if(DELAY_LONG == delay_ms)
            {
                delay_ms = DELAY_SHORT;
            }
            else
            {
                delay_ms = DELAY_LONG;
            }
        }

        /* Blink LED four times */
        for (count = 0; count < LED_BLINK_COUNT; count++)
        {
            cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_ON);
            Cy_SysLib_Delay(delay_ms);
            cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_OFF);
            Cy_SysLib_Delay(delay_ms);
        }

        /* Enter deep sleep mode */
        cyhal_system_deepsleep();
    }
}


/*******************************************************************************
* Function Name: gpio_interrupt_handler()
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
*******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    gpio_intr_flag = true;
}


/* [] END OF FILE */
