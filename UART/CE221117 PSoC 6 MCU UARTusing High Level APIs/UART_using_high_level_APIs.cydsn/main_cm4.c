/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This example demonstrates the Serial Communication Block (SCB) 
* based UART transmit and receive operation in PSoC 6 MCU using high level 
* APIs found in Peripheral Driver Library (PDL).
*
* Related Document: CE221117_PSoC6MCU_UARTusingHighLevelAPIs.pdf
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
#define LED_ON               (0x00u)
#define LED_OFF              (!LED_ON)

/* UART receive ring buffer size */
#define RING_BUFFER_SIZE     (32u)

/* The number of characters to be received from terminal before echoing */
#define CHARACTER_BUNCH_SIZE (10u)

/*******************************************************************************
*            Global variables
*******************************************************************************/

/***************************************************************************//**
* Function Name: handle_error
********************************************************************************
*
* This function processes unrecoverable errors such as any component 
* initialization errors etc. In case of such error the system will switch on 
* ERROR_RED_LED and stay in the infinite loop of this function.
*
*******************************************************************************/
void handle_error(void)
{   
     /* Disable all interrupts */
    __disable_irq();
    
    /* Switch on error LED */
    Cy_GPIO_Write(ERROR_RED_LED_0_PORT, ERROR_RED_LED_0_NUM, LED_ON);
    while(1u) {}
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
*  The main function performs the following actions:
*   1. Sets up UART component.
*   2. If initialization of UART component fails then switch on ERROR_RED_LED.
*   3. UART sends text header into the serial terminal.
*   4. Start the UART receive operation from the terminal with size = 
*      CHARACTER_BUNCH_SIZE
*   5. Wait for "CHARACTER_BUNCH_SIZE" number of characters to receive
*   6. Start the UART transmit operation with size = CHARACTER_BUNCH_SIZE
*   7. Wait for "CHARACTER_BUNCH_SIZE" number of characters to transmit
*   8. Repeat steps 4 to 7
*
*******************************************************************************/
int main(void)
{
    cy_en_scb_uart_status_t uart_status;
    cy_en_sysint_status_t sysint_status;
    uint8_t uartBuffer[RING_BUFFER_SIZE+1];
    
    /* Turn off ERROR_RED_LED */
    Cy_GPIO_Write(ERROR_RED_LED_0_PORT, ERROR_RED_LED_0_NUM, LED_OFF);
    
    /* Initialize the UART operation */
    uart_status = Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    if(uart_status != CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    
    /* Initialize the UART Interrupt */
    sysint_status = Cy_SysInt_Init(&UART_SCB_IRQ_cfg, &UART_Interrupt);
    if(sysint_status != CY_SYSINT_SUCCESS)
    {
        handle_error();
    }
    
    /* Enable the interrupt */
    NVIC_EnableIRQ((IRQn_Type) UART_SCB_IRQ_cfg.intrSrc);
    
    /* Start the UART receive ring buffer */
    Cy_SCB_UART_StartRingBuffer(UART_HW, uartBuffer, RING_BUFFER_SIZE, &UART_context);
    
    /* Enable the UART */
    Cy_SCB_UART_Enable(UART_HW);
    
    /* Enable global interrupts. */
    __enable_irq();

    /* Transmit header to the terminal. */
    Cy_SCB_UART_PutString(UART_HW, "\r\n**********************************************************************************\r\n");
    Cy_SCB_UART_PutString(UART_HW, "This is UART example, which uses high level APIs to demonstrate UART operation\r\n");
    Cy_SCB_UART_PutString(UART_HW, "If you are able to read this text the terminal connection is configured correctly.\r\n");
    Cy_SCB_UART_PutString(UART_HW, "Start typing the characters to see an echo in the terminal.\r\n");
    Cy_SCB_UART_PutString(UART_HW, "\r\n");

    /* Wait for completion of header transmit */
    while (Cy_SCB_UART_IsTxComplete(UART_HW) == false);
    
    for (;;)
    {
        /* Start the UART receive operation from the terminal with size = CHARACTER_BUNCH_SIZE */
        uart_status = Cy_SCB_UART_Receive(UART_HW, uartBuffer, CHARACTER_BUNCH_SIZE, &UART_context);
        if(uart_status != CY_SCB_UART_SUCCESS)
        {
            handle_error();
        }
        
        /* Make sure that "CHARACTER_BUNCH_SIZE" number of characters are received from the terminal */
        while (0UL != (CY_SCB_UART_RECEIVE_ACTIVE  & Cy_SCB_UART_GetReceiveStatus (UART_HW, &UART_context)));
        
        /* Start the UART transmit operation from the terminal with size = CHARACTER_BUNCH_SIZE */
        uart_status = Cy_SCB_UART_Transmit(UART_HW, uartBuffer, CHARACTER_BUNCH_SIZE, &UART_context);
        if(uart_status != CY_SCB_UART_SUCCESS)
        {
            handle_error();
        }
        
        /* Make sure that "CHARACTER_BUNCH_SIZE" number of characters are transmitted to the terminal */
        while (0UL != (CY_SCB_UART_TRANSMIT_ACTIVE & Cy_SCB_UART_GetTransmitStatus(UART_HW, &UART_context)));
    }
}

/* [] END OF FILE */

