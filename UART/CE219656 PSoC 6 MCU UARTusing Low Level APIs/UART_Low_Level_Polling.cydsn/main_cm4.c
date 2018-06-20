/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.20
*
* Description: This example demonstrates the UART transmit and receive 
*              operation using low level polling APIs.
*
* Related Document: CE219656.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer kit
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/

#include <project.h>
#include <stdio.h>

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
void handle_error(void);

/*******************************************************************************
*            Constants
*******************************************************************************/
#define LED_ON              (0x00u)
#define LED_OFF             (!LED_ON)

/*******************************************************************************
*            Global variables
*******************************************************************************/


/***************************************************************************//**
* Function Name: handle_error
********************************************************************************
*
* Summary:
* This function processes unrecoverable errors such as any component 
* initialization errors etc. In case of such error the system will switch on 
* RED_LED_ERROR and stay in the infinite loop of this function.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void handle_error(void)
{   
     /* Disable all interrupts */
    __disable_irq();
    
    /* Switch on error LED */
    Cy_GPIO_Write(RED_LED_ERROR_0_PORT, RED_LED_ERROR_0_NUM, LED_ON);
    while(1u) {}
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Sets up UART component.
*   2. If initialization of UART component fails then switch on RED_LED_ERROR.
*   3. UART sends text header into the serial terminal.
*   4. Re-transmits whatever the user types on the console.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
int main(void)
{
    uint32_t read_data;
    cy_en_scb_uart_status_t initstatus;    
    
    Cy_GPIO_Write(RED_LED_ERROR_0_PORT, RED_LED_ERROR_0_NUM, LED_OFF);
    
    /* Start UART operation. */
    initstatus = Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    if(initstatus!=CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
        
    Cy_SCB_UART_Enable(UART_HW);   

    /* Transmit header to the terminal. */
    Cy_SCB_UART_PutString(UART_HW, "\r\n**********************************************************************************\r\n");
    Cy_SCB_UART_PutString(UART_HW, "This is UART example, which uses low level polling APIs to demonstrate UART operation\r\n");
    Cy_SCB_UART_PutString(UART_HW, "If you are able to read this text the terminal connection is configured correctly.\r\n");
    Cy_SCB_UART_PutString(UART_HW, "Start transmitting the characters to see an echo in the terminal.\r\n");
    Cy_SCB_UART_PutString(UART_HW, "\r\n");

    for (;;)
    {
        /* Check if there is a received character from user console */
        if (0UL != Cy_SCB_UART_GetNumInRxFifo(UART_HW))
        {
            /* Re-transmit whatever the user types on the console */
            read_data = Cy_SCB_UART_Get(UART_HW);
            while (0UL == Cy_SCB_UART_Put(UART_HW,read_data))
            {
            }
        }
    }
}

/* [] END OF FILE */

