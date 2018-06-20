/*****************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00  
*
* Description:This code example demonstrates how to use a TCPWM Component to
* measure frequency, by counting the number of rising edges in a known time
* period.
*
* Related Document: CE220692.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE Kit
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#include "project.h"
#include<stdio.h>

#define BUFFER_SIZE         (128u)
#define SET                 (1u)
#define RESET               (0u)
#define MAXCOUNT            (65536u)    /* 16 bit Counter */ 

uint8_t intFlag = 0;
uint32_t counterOverflow = 0;
char buffer[BUFFER_SIZE];

/*******************************************************************************
* Function Name: IsrCounter()
********************************************************************************
* Summary: Interrupt service routine for Counter interrupt. This function is 
* executed when a capture or an overflow event happens.
* 
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void IsrCounter(void)
{
    /* Get which event triggered the interrupt */
    uint32_t intSource = Cy_TCPWM_GetInterruptStatus(Counter_HW, Counter_TCPWM__CNT_IDX);
    
    /* Clear interrupt */
    Cy_TCPWM_ClearInterrupt(Counter_HW, Counter_TCPWM__CNT_IDX, CY_TCPWM_INT_ON_CC_OR_TC);
    NVIC_ClearPendingIRQ(SysInt_Counter_cfg.intrSrc);
    
    /* If the interrupt is triggered by capture event then set the flag for frequency 
    calculation */
    if(intSource == CY_TCPWM_INT_ON_CC)
    {
        /* Set interrupt flag */ 
        intFlag = SET;
    }
    
    /* If the interrupt is triggered by an overflow event, then counting how 
    many times counter overflow happened in one second */ 
    if(intSource == CY_TCPWM_INT_ON_TC)
    {
        counterOverflow++;
    }
} 

/******************************************************************************
* Function Name: main()
*******************************************************************************
*
* Summary: This is the system entrance point for Cortex-M4.
* This function initializes the PSoC Components to measure the frequency and
* display the result accordingly.
*
* Parameters:
*  None
*
* Return:
*  int
*
* Side Effects:
*  None  
*
******************************************************************************/
int main(void)
{
    uint32_t freq, captureVal;
    /* Enable global interrupts. */
    __enable_irq(); 
        
    /* This project uses 3 TCPWM Component. One is set up as a Counter which is
    used to count the rising edges. A second is set up as a PWM which generates
    1 sec time window. The third is also set up as a PWM for generating different
    frequency signal and based on the Period and Compare value it generates a 
    signal with a particular frequency. */
    
    /* Initialize with config set in component and enable the Counter */
    Cy_TCPWM_Counter_Init(Counter_HW, Counter_CNT_NUM, &Counter_config);
    Cy_TCPWM_Enable_Multiple(Counter_HW, Counter_CNT_MASK);
    
    /* Hookup and enable interrupt */
    Cy_SysInt_Init(&SysInt_Counter_cfg, IsrCounter);
    NVIC_EnableIRQ((IRQn_Type)SysInt_Counter_cfg.intrSrc);
    
    /* Start the Counter */
    Cy_TCPWM_TriggerStart(Counter_HW, Counter_CNT_MASK);
        
    /* Initilaize with config set in componentand and start the PWM for generating 
     one second time window */
    Cy_TCPWM_PWM_Init(OneSecTimer_HW, OneSecTimer_CNT_NUM, &OneSecTimer_config);
    Cy_TCPWM_Enable_Multiple(OneSecTimer_HW, OneSecTimer_CNT_MASK);
    Cy_TCPWM_TriggerStart(OneSecTimer_HW, OneSecTimer_CNT_MASK);
    
    /* Initialize with config set in component and start the PWM for generating
     signal of desired frequency */
    Cy_TCPWM_PWM_Init(PWM_MeasFreq_HW, PWM_MeasFreq_CNT_NUM, &PWM_MeasFreq_config);
    Cy_TCPWM_Enable_Multiple(PWM_MeasFreq_HW, PWM_MeasFreq_CNT_MASK);
    Cy_TCPWM_TriggerStart(PWM_MeasFreq_HW, PWM_MeasFreq_CNT_MASK);
        
    /* Initialize with config set in component and enable the UART to display the result*/
    Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    Cy_SCB_UART_Enable(UART_HW);
    
    /* Display code example title */
    Cy_SCB_UART_PutString(UART_HW, "CE220692 - PSoC 6 MCU - Frequency Measurement Using TCPWM\n\r");
    
    for(;;)
    {
        static uint32_t capturePrev = 0;
        if (intFlag)    /* Interrupt was trigerred, read the captured counter value now */
        {
            /* Get the Counter value */
            captureVal = Cy_TCPWM_Counter_GetCapture(Counter_HW, Counter_TCPWM__CNT_IDX);  
            
            /* Calculate freaquency */
            freq = counterOverflow * MAXCOUNT + captureVal - capturePrev;
            counterOverflow = RESET;
            
            capturePrev = captureVal;
            /* Clear the interrupt flag */
            intFlag = RESET;
            
            /* Display frequency */
            sprintf(buffer, "Frequency = %lu Hz \n\r", freq);
            Cy_SCB_UART_PutString(UART_HW, buffer);
        }
    }
}
/* [] END OF FILE */
