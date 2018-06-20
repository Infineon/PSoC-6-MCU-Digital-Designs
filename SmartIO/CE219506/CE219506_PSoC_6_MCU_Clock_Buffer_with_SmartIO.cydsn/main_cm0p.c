/*****************************************************************************
* File Name: main_cm0p.c
*
* Version: 1.10
*
* Description: "Clock Buffering with SmartIO" code example source code. The 
* chip is put into deep-sleep mode periodically. The chip wakes up and toggles
* the StatusPin upon a GPIO interrupt from Trigger pin. 
*
* Related Document: Code example CE219506.pdf
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
#include "config.h"

/* Macros for configuring project functionality */
#define HIGH 1
#define LOW 0

/*
* status returned by GPIO and MCDWT Interrupt service routines .
*/

typedef enum
{ 
    GPIOFLAG,  /*status returned by GPIO Interrupt service routine*/
    WDTFLAG  /*status returned by MCDWT Interrupt service routine*/
    
} FLAG;


FLAG state;

/*******************************************************************************
* Function Name: MCWDT_1_Interrupt
********************************************************************************
*
* Summary:
*  Interrupt service routine for the Watchdog timer interrupt
*    
*  Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/

void MCWDT_1_Interrupt(void)
{
    uint32 mcwdtIsrMask;
    mcwdtIsrMask=Cy_MCWDT_GetInterruptStatus(MCWDT_STRUCT0);
    if(0u!=(CY_MCWDT_CTR0 & mcwdtIsrMask))
    {
        Cy_MCWDT_ClearInterrupt(MCWDT_STRUCT0,CY_MCWDT_CTR0);
        NVIC_ClearPendingIRQ(isr_MCWDT_cfg.intrSrc);  
        state=WDTFLAG;
     }
   
    
}
        

/*******************************************************************************
* Function Name: GPIO_interrupt
********************************************************************************
*
* Summary:
*  Interrupt service routine for the port interrupt triggered from IntrPin.
*    
*  Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void GPIO_interrupt()
{
    state=GPIOFLAG;
    Cy_GPIO_ClearInterrupt(Trigger_0_PORT,Trigger_0_NUM);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);  
 
}

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
    /* Initialize and enable GPIO interrupt assigned to CM0+ */
    Cy_SysInt_Init(&isr_gpio_cfg,GPIO_interrupt);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_gpio_cfg.intrSrc);
    
    /*Initialize and enable MCWDT interrupt assigned to CM0+*/
    Cy_SysInt_Init(&isr_MCWDT_cfg,MCWDT_1_Interrupt);
    NVIC_ClearPendingIRQ(isr_MCWDT_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_MCWDT_cfg.intrSrc);
            
      
    /*Start MCDWT counter C0*/
    Cy_MCWDT_Init(MCWDT_STRUCT0, &MCWDT_config);
    /*Enable the interrupts for MCDWT counter C0 only*/
    Cy_MCWDT_SetInterruptMask(MCWDT_STRUCT0, CY_MCWDT_CTR0);
    
    __enable_irq(); /* Enable global interrupts. */
    
    SmartIO_Start();/*Start the Smart IO Component*/
    SmartIO_HoldOverride(SmartIO_OVCTRL_ENABLE);
    Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    SmartIO_HoldOverride(SmartIO_OVCTRL_DISABLE);
   
    
    for(;;)
    {     
        
        if(state==GPIOFLAG)
        {

            /*Start MCDWT counter C0*/
            Cy_MCWDT_Enable(MCWDT_STRUCT0, MCWDT_1_ENABLED_CTRS_MASK,0u);
             /*Turn ON Green LED to indicate CM0p wakeup*/
            Cy_GPIO_Write(StatusPin_0_PORT,StatusPin_0_NUM,LOW);
                       
        }
        else if (state==WDTFLAG)
        {

            /*Stop MCDWT counter C0*/
            Cy_MCWDT_Disable(MCWDT_STRUCT0,CY_MCWDT_CTR_Msk, MCWDT_1_TWO_LF_CLK_CYCLES_DELAY);
            
            /*Turn OFF Green LED to indicate CM0p Deep Sleep */
            Cy_GPIO_Write(StatusPin_0_PORT,StatusPin_0_NUM,HIGH);
            
            /* Hold override functionality must be enabled before entering Deep-Sleep */
            SmartIO_HoldOverride(SmartIO_OVCTRL_ENABLE);
            /*Put CM0p core to Deep Sleep*/
            Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
            SmartIO_HoldOverride(SmartIO_OVCTRL_DISABLE);
            
        }
       

         
        
    }
}

/* [] END OF FILE */
