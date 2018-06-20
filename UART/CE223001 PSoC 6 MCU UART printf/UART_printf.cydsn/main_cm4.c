/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This example shows how to re-direct the printf function to 
* use UART API to send data to external terminal.
*
* Related Document: CE223001_PSoC6MCU_UART_printf.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer kit
*
******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation.
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
#include "project.h"
#include <stdio.h>

#define MAX_COUNT 65535u

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
static void HandleError(void);
static void SwInterruptIsr(void);


/*******************************************************************************
*        Global variables
*******************************************************************************/
int count = 0u;
uint32_t printCount = false;

/*******************************************************************************
* Function Name: SwInterruptIsr
********************************************************************************
* Summary: 
* This function is executed when GPIO interrupt is triggered.
* This ISR 
*        i. Clears the interrupt source.
*        ii. Increments the count and sets printCount flag.
*       
* \Param None
*
* \return
* void
*
*******************************************************************************/
static void SwInterruptIsr(void)
{   
    Cy_GPIO_ClearInterrupt(Pin_Switch_0_PORT, Pin_Switch_0_NUM);
        
    count++;
    /* If count reaches MAX_COUNT reset the count variable. */
    if(count == MAX_COUNT)
    {
        count = 0u;   
    }
    printCount = true;
    
	NVIC_ClearPendingIRQ(SysInt_SW_cfg.intrSrc);
}

/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
* Summary:
*  The main function performs the following actions:
*   1. Sets up UART component.
*   2. If initialization of UART component fails then stay in an infinite loop.
*   3. Continuously check whether printCount flag is set.
*   4. If printCount flag is set, reset this flag and print the count using
*      printf function.
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
     /* UART initialization status */
    cy_en_scb_uart_status_t uart_status ;
    
     /* Initialize UART operation. Config and Context structure is copied from Generated source. */
    uart_status  = Cy_SCB_UART_Init(Uart_Printf_HW, &Uart_Printf_config, &Uart_Printf_context);
    if(uart_status != CY_SCB_UART_SUCCESS)
    {
        HandleError();
    }	
    Cy_SCB_UART_Enable(Uart_Printf_HW);
    
    /* Initialize and enable GPIO interrupt assigned to CM4+ */
    /* SysInt_SW_cfg structure is defined in cyfitter_sysint_cfg.c based on
       parameters entered in the Interrupts tab of CYDWR. */
    Cy_SysInt_Init(&SysInt_SW_cfg, SwInterruptIsr); 
    NVIC_ClearPendingIRQ(SysInt_SW_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)SysInt_SW_cfg.intrSrc);

     __enable_irq(); /* Enable global interrupts. */

    for(;;)
    {
        if(printCount)
        {
            printCount = false;   
            printf("Number of times switch SW2 has been pressed = %d\r\n", count);  
        } 
    }
}


/*******************************************************************************
* Function Name: HandleError
********************************************************************************
* Summary:
* This function processes unrecoverable errors such as UART component 
* initialization error. In case of such error the system will stay in an
* infinite loop of this function.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void HandleError(void)
{
     /* Disable all interrupts */
    __disable_irq();

    while(1u) 
    {
        
    }
}


/*******************************************************************************
* When printf function is called it is redirected to the following functions
* depending on compiler used.
*******************************************************************************/
#if defined (__GNUC__)
/*******************************************************************************
* Function Name: _write
********************************************************************************
* Summary: 
* NewLib C library is used to retarget printf to _write. printf is redirected to 
* this function when GCC compiler is used to print data to terminal using UART. 
*
* \param file
* This variable is not used.
* \param *ptr
* Pointer to the data which will be transfered through UART.
* \param len
* Length of the data to be transfered through UART.
*
* \return
* returns the number of characters transferred using UART.
* \ref int
*******************************************************************************/
   int _write(int file, char *ptr, int len)
    {
        int nChars = 0;

        /* Suppress the compiler warning about an unused variable. */
        if (0 != file)
        {
        }
                
        nChars = Cy_SCB_UART_PutArray(Uart_Printf_HW, ptr, len);
           
        return (nChars);
    }
#elif defined(__ARMCC_VERSION)
    
/*******************************************************************************
* Function Name: fputc
********************************************************************************
* Summary: 
* printf is redirected to this function when MDK compiler is used to print data
* to terminal using UART.
*
* \param ch
* Character to be printed through UART.
*
* \param *file
* pointer to a FILE object that identifies the stream where the character is to be
* written.
*
* \return
* This function returns the character that is written in case of successful
* write operation else in case of error EOF is returned.
* \ref int
*******************************************************************************/
    struct __FILE
    {
        int handle;
    };
    
    enum
    {
        STDIN_HANDLE,
        STDOUT_HANDLE,
        STDERR_HANDLE
    };
    
    FILE __stdin = {STDIN_HANDLE};
    FILE __stdout = {STDOUT_HANDLE};
    FILE __stderr = {STDERR_HANDLE};
    
    int fputc(int ch, FILE *file)
    {
        int ret = EOF;
        switch(file->handle)
        {
            case STDOUT_HANDLE:
                while (0UL == Cy_SCB_UART_Put(Uart_Printf_HW, ch))
                {
                }
                ret = ch;
            break;
                
            case STDERR_HANDLE:
                ret = ch;
            break;
                
            default:
                file = file;
            break;
        }
        return(ret);
    }
    

#endif /* (__GNUC__) */


/* [] END OF FILE */



