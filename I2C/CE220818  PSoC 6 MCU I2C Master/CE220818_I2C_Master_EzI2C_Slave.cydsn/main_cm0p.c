/******************************************************************************
* File Name: main_cm0p.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the I2C master (SCB 
* mode) component. The I2C master sends the packet with a command to the EzI2C slave to control
* the RGB LED color. The packet with a status is read back.
*
* Related Document: CE220818.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit 
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
#include "project.h"

#define EZI2C_BUFFER_SIZE      (0x08UL)

/* Start and end of packet markers */
#define PACKET_SOP             (0x01UL)
#define PACKET_EOP             (0x17UL)

/* Command valid status */
#define STS_CMD_DONE           (0x00UL)
#define STS_CMD_FAIL           (0xFFUL)

/*Color data position in buffer */
#define PACKET_SOP_POS         (0x00UL)
#define COLOR_RED_POS          (0x01UL)
#define COLOR_GREEN_POS        (0x02UL)
#define COLOR_BLUE_POS         (0x03UL)
#define PACKET_EOP_POS         (0x04UL)
#define PACKET_RPLY_SOP_POS    (0x05UL)
#define PACKET_RPLY_STS_POS    (0x06UL)
#define PACKET_RPLY_EOP_POS    (0x07UL)

#define ZERO               (0UL)

static void SEzI2C_InterruptHandler(void);
static void CheckEzI2Cbuffer( void );
static void HandleError(void);

uint8_t buffer[EZI2C_BUFFER_SIZE] ;


/*******************************************************************************
* Function Name: SEzI2C_InterruptHandler
****************************************************************************//**
*
* This function executes interrupt service routine.
*
* \param None
*
* \return
*  None
* Side Effects:
*  None  
*
*******************************************************************************/
static void SEzI2C_InterruptHandler(void)
{
    /* ISR implementation for EZI2C. */
    Cy_SCB_EZI2C_Interrupt(sEzI2C_HW, &sEzI2C_context);
}


/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
*  The main function performs the following actions:
*   1. Sets up EzI2C component to acts as slave.
*   2. If initialization of EzI2C component fails, system will be in infinite loop.
*   3. Initializes PWM components  to control RGB LED. If initialization of PWM components fails, system will be in infinite loop.
*   4. EzI2C slave component receives packets from master and configured the PWM to drive RGB LED.
*   5. Slave replys with the acknowledgment packet .
*
*******************************************************************************/
int main(void)
{
    cy_en_tcpwm_status_t initPWMstatus;
    cy_en_scb_ezi2c_status_t initEzI2Cstatus;
    cy_en_sysint_status_t sysEzI2Cstatus;
   /* Enable global interrupts. */
    __enable_irq(); 
    
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 
      
    /* Initialize and enables TCPWM to control red LED. */
    initPWMstatus = Cy_TCPWM_PWM_Init(RedPWM_HW, RedPWM_CNT_NUM, &RedPWM_config);
    if(initPWMstatus != CY_TCPWM_SUCCESS)
    {
        HandleError();
    }
    Cy_TCPWM_Enable_Multiple(RedPWM_HW, RedPWM_CNT_MASK);
    Cy_TCPWM_TriggerStart(RedPWM_HW, RedPWM_CNT_MASK);
    
    /* Initialize and enables TCPWM to control green LED. */
    initPWMstatus = Cy_TCPWM_PWM_Init(GreenPWM_HW, GreenPWM_CNT_NUM, &GreenPWM_config);
    if(initPWMstatus!=CY_TCPWM_SUCCESS)
    {
        HandleError();
    }
    Cy_TCPWM_Enable_Multiple(GreenPWM_HW, GreenPWM_CNT_MASK);
    Cy_TCPWM_TriggerStart(GreenPWM_HW, GreenPWM_CNT_MASK);
    
    /* Initialize and enables TCPWM to control blue LED. */
    initPWMstatus = Cy_TCPWM_PWM_Init(BluePWM_HW, BluePWM_CNT_NUM, &BluePWM_config);
    if(initPWMstatus != CY_TCPWM_SUCCESS)
    {
        HandleError();
    }
    Cy_TCPWM_Enable_Multiple(BluePWM_HW, BluePWM_CNT_MASK);
    Cy_TCPWM_TriggerStart(BluePWM_HW, BluePWM_CNT_MASK);
    
    /*Initialize EzI2C slave.*/
    initEzI2Cstatus = Cy_SCB_EZI2C_Init(sEzI2C_HW, &sEzI2C_config, &sEzI2C_context);
    if(initEzI2Cstatus!=CY_SCB_EZI2C_SUCCESS)
    {
        HandleError();
    }
    /* Hook interrupt service routine and enable interrupt. */
    sysEzI2Cstatus = Cy_SysInt_Init(&sEzI2C_SCB_IRQ_cfg, &SEzI2C_InterruptHandler);
    if(sysEzI2Cstatus!=CY_SYSINT_SUCCESS)
    {
        HandleError();
    }
    NVIC_EnableIRQ((IRQn_Type) sEzI2C_SCB_IRQ_cfg.intrSrc);
    
    /* Configure buffer for communication with master. */
    Cy_SCB_EZI2C_SetBuffer1(sEzI2C_HW, buffer, EZI2C_BUFFER_SIZE, EZI2C_BUFFER_SIZE, &sEzI2C_context);
    
    /* Enable SCB for the EZI2C operation. */
    Cy_SCB_EZI2C_Enable(sEzI2C_HW);
       
    for(;;)
    {
        /* Continuously check for packets written in buffer. */
        CheckEzI2Cbuffer( );
    }
}


/*******************************************************************************
* Function Name: CheckEzI2Cbuffer
****************************************************************************//**
*
*  This function continuously reads the contents of EzI2C buffer. Packets are compared and appropriate
*actions are taken to set the PWM comapre value.
*
* \param None
*
* \return
*  None
*
*******************************************************************************/
static void CheckEzI2Cbuffer( void )
{
    uint32_t ezi2cState;
 
    /* Disable the EZI2C interrupts so that ISR is not serviced while
           checking for EZI2C status. */
    NVIC_DisableIRQ(sEzI2C_SCB_IRQ_cfg.intrSrc);
        
    /* Read the EZi2C status */        
    ezi2cState = Cy_SCB_EZI2C_GetActivity(sEzI2C_HW, &sEzI2C_context);
        
    /* Write complete without errors: parse packets, otherwise ignore. */
    if((0u != (ezi2cState & CY_SCB_EZI2C_STATUS_WRITE1)) && (0u == (ezi2cState & CY_SCB_EZI2C_STATUS_ERR)))
    {
        /*Check buffer content to know any new packets are written from master. */
        if( ( buffer[PACKET_SOP_POS] ==PACKET_SOP) && ( buffer[PACKET_EOP_POS] ==PACKET_EOP) )
        {
            /*Sets the compare value to control the color of RGB LED. */
            Cy_TCPWM_PWM_SetCompare0(RedPWM_HW, RedPWM_CNT_NUM, buffer[COLOR_RED_POS]);
            Cy_TCPWM_PWM_SetCompare0(GreenPWM_HW, GreenPWM_CNT_NUM, buffer[COLOR_GREEN_POS]);
            Cy_TCPWM_PWM_SetCompare0(BluePWM_HW, BluePWM_CNT_NUM, buffer[COLOR_BLUE_POS]);
            
            /* Clear the location so that any new packets written to buffer will be known. */
            buffer[PACKET_SOP_POS]      = ZERO;              
            buffer[PACKET_EOP_POS]      = ZERO;   
            
            /* Write to buffer the data related to status. */
            buffer[PACKET_RPLY_SOP_POS] = PACKET_SOP;
            buffer[PACKET_RPLY_STS_POS] = STS_CMD_DONE;
            buffer[PACKET_RPLY_EOP_POS] = PACKET_EOP; 
        }   
        else
        {
            /* write to buffer the data related to status. */
            buffer[PACKET_RPLY_SOP_POS] = PACKET_SOP;
            buffer[PACKET_RPLY_STS_POS] = STS_CMD_FAIL;
            buffer[PACKET_RPLY_EOP_POS] = PACKET_EOP; 
        }
    }
     /* Enable interrupts for servicing ISR. */
    NVIC_EnableIRQ(sEzI2C_SCB_IRQ_cfg.intrSrc);
}


/*******************************************************************************
* Function Name: HandleError
****************************************************************************//**
*
* This function processes unrecoverable errors such as any component 
* initialization errors etc. In case of such error the system will 
* stay in the infinite loop of this function..
*
*
* \note
* * If error ocuurs interrupts are disabled.
*
*******************************************************************************/
static void HandleError(void)
{   
     /* Disable all interrupts. */
    __disable_irq();
    
    /* Infinite loop. */
    while(1u) {}
}
/* [] END OF FILE */




