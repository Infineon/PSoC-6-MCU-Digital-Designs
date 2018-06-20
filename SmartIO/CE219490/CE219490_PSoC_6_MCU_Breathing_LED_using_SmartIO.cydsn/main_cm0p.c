/*****************************************************************************
* File Name: main_cm0p.c
*
* Version: 1.10
*
* Description: Demonstrates a hardware-based breathing LED using a TCPWM
* component and SmartIO component.
* This design does not use the CPU beyond initialization.
*
* Related Document: Code example CE219490.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE Kit
*
*******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
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
*******************************************************************************/
#include "project.h"
#include "TCPWM_Config.h"
/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  Main function.
*
* Parameters:  
*  None
*
* Return: 
*  int
*
*******************************************************************************/

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
    /* Start the PWM component */
    Cy_TCPWM_PWM_Init(TCPWM0, PWM_TCPWM__CNT_IDX, &TCPWM_config);
    Cy_TCPWM_PWM_Enable(TCPWM0, PWM_TCPWM__CNT_IDX);
    Cy_TCPWM_TriggerStart(TCPWM0, PWM_CNT_MASK);

    /* Start the Smart IO component */
    SmartIO_Start();
    for(;;)
    {
        
    }
}

/* [] END OF FILE */
