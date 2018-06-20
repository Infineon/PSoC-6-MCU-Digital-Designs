/******************************************************************************
* File Name: main_cm0p.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the I2C master (SCB 
* mode) component using low level PDL function. The I2C master sends the packet with a command
* to the I2C slave to control the RGB LED color. The packet with a status is read back. CM0 acts 
* as a slave and accepts various command packets from master and changes the color of RGB LED.
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

/* Buffer and packet size */
#define PACKET_SIZE          (5UL)

/* Buffer and packet size in the slave*/
#define SL_RD_BUFFER_SIZE    (03UL)
#define SL_WR_BUFFER_SIZE    (PACKET_SIZE)

/* Start and end of packet markers */
#define PACKET_SOP           (0x01UL)
#define PACKET_EOP           (0x17UL)

/* Command valid status */
#define STS_CMD_DONE         (0x00UL)
#define STS_CMD_FAIL         (0xFFUL)

/* Packet positions */
#define PACKET_SOP_POS       (0UL)
#define PACKET_STS_POS       (1UL)
#define PACKET_RED_POS       (1UL)
#define PACKET_GREEN_POS     (2UL)
#define PACKET_BLUE_POS      (3UL)
#define PACKET_EOP_POS       (4UL)
#define COLOR_RED_POS        (0x01UL)
#define COLOR_GREEN_POS      (0x02UL)
#define COLOR_BLUE_POS       (0x03UL)

#define ZERO               (0UL)

/*******************************************************************************
* Function prototypes
*******************************************************************************/
static void SlaveExecuteCommand(void);
static void HandleEventsSlave(uint32 event);
void HandleError(void);

/* I2C slave read and write buffers. */
uint8_t i2cReadBuffer [SL_RD_BUFFER_SIZE] = {PACKET_SOP, STS_CMD_FAIL, PACKET_EOP};
uint8_t i2cWriteBuffer[SL_WR_BUFFER_SIZE] ;


/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
*  The main function performs the following actions:
*   1. Sets up I2C component to acts as slave.
*   2. If initialization of I2C component fails, system will be in infinite loop.
*   3. Initializes PWM components to control RGB LED. If initialization of PWM component fails, system will be in infinite loop.
*   4. I2C slave component receives packets from master and configured the PWM to drive RGB LED.
*   5. Slave replys with the acknowledgment packet .
*
*******************************************************************************/
int main(void)
{
    cy_en_tcpwm_status_t initPWMstatus;
    cy_en_scb_i2c_status_t initI2Cstatus;
    cy_en_sysint_status_t sysI2Cstatus;
    
    /* Enable global interrupts. */
    __enable_irq(); 
    
    /* Initialize the reply status packet. */
    i2cReadBuffer[PACKET_STS_POS] = STS_CMD_FAIL;
    
    /* Enable CM4. CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
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
    if(initPWMstatus != CY_TCPWM_SUCCESS)
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
    
    /* Initialize and enable I2C component in slave mode. */
    initI2Cstatus = Cy_SCB_I2C_Init(sI2C_HW, &sI2C_config, &sI2C_context);
    if(initI2Cstatus != CY_SCB_I2C_SUCCESS)
    {
        HandleError();
    }
    sysI2Cstatus = Cy_SysInt_Init(&sI2C_SCB_IRQ_cfg, &sI2C_Interrupt);
    if(sysI2Cstatus != CY_SYSINT_SUCCESS)
    {
        HandleError();
    }
    Cy_SCB_I2C_SlaveConfigReadBuf(sI2C_HW, i2cReadBuffer, SL_RD_BUFFER_SIZE, &sI2C_context);
    Cy_SCB_I2C_SlaveConfigWriteBuf(sI2C_HW, i2cWriteBuffer, SL_WR_BUFFER_SIZE, &sI2C_context);
    Cy_SCB_I2C_RegisterEventCallback(sI2C_HW, (cy_cb_scb_i2c_handle_events_t) HandleEventsSlave, &sI2C_context);
    NVIC_EnableIRQ((IRQn_Type) sI2C_SCB_IRQ_cfg.intrSrc);
    Cy_SCB_I2C_Enable(sI2C_HW);
   
    for(;;)
    {
        /* Continuous looping. Data reception by I2C slave is handled by interrupt. */
        
    }
}


/*******************************************************************************
* Function Name: HandleEventsSlave
****************************************************************************//**
*
* Handles slave events write and read completion events. 
*
* \param event
* reports slave events.
* ref uint32_t
*
* \return
*  None
*
*******************************************************************************/
static void HandleEventsSlave(uint32_t event)
{
    /* Check write complete event. */
    if (0UL != (CY_SCB_I2C_SLAVE_WR_CMPLT_EVENT & event))
    {
        
        /* Check for errors */
        if (0UL == (CY_SCB_I2C_SLAVE_ERR_EVENT & event))
        {
            /* Check packet length */
            if (PACKET_SIZE ==  Cy_SCB_I2C_SlaveGetWriteTransferCount(sI2C_HW, &sI2C_context))            
            {
                /* Check start and end of packet markers. */
                if ((i2cWriteBuffer[PACKET_SOP_POS] == PACKET_SOP) &&
                    (i2cWriteBuffer[PACKET_EOP_POS] == PACKET_EOP))
                {
                    SlaveExecuteCommand( );
                }
            }
        }
        
        /* Update status of received commend. */
        i2cReadBuffer[PACKET_STS_POS] = STS_CMD_DONE;
        
        /* Configure write buffer for the next write. */
        i2cWriteBuffer[PACKET_SOP_POS] = ZERO;
        i2cWriteBuffer[PACKET_EOP_POS] = ZERO;
        Cy_SCB_I2C_SlaveConfigWriteBuf(sI2C_HW, i2cWriteBuffer, SL_WR_BUFFER_SIZE, &sI2C_context);
    }
    
    /* Check write complete event. */
    if (0UL != (CY_SCB_I2C_SLAVE_RD_CMPLT_EVENT & event))
    {
        /* Configure read buffer for the next read. */
        i2cReadBuffer[PACKET_STS_POS] = STS_CMD_FAIL;
        Cy_SCB_I2C_SlaveConfigReadBuf(sI2C_HW, i2cReadBuffer, SL_RD_BUFFER_SIZE, &sI2C_context);
    }
}


/*******************************************************************************
* Function Name: SlaveExecuteCommand
****************************************************************************//**
*
*  Executes received command to control the LED state and returns status.
*  If the command is unknown, the LED state is not changed.
*
* \param None
*
* \return
*  None
*
*******************************************************************************/
static void SlaveExecuteCommand( void )
{   
    /* Sets the compare value to control the color of RGB LED. */
    Cy_TCPWM_PWM_SetCompare0(RedPWM_HW, RedPWM_CNT_NUM, i2cWriteBuffer[COLOR_RED_POS]);
    Cy_TCPWM_PWM_SetCompare0(GreenPWM_HW, GreenPWM_CNT_NUM, i2cWriteBuffer[COLOR_GREEN_POS]);
    Cy_TCPWM_PWM_SetCompare0(BluePWM_HW, BluePWM_CNT_NUM, i2cWriteBuffer[COLOR_BLUE_POS]);
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
void HandleError(void)
{   
     /* Disable all interrupts. */
    __disable_irq();
    
    /* Infinite loop. */
    while(1u) {}
}
/* [] END OF FILE */



