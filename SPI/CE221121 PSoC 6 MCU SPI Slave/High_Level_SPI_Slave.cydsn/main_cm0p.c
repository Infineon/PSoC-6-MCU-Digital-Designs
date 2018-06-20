/******************************************************************************
* File Name: main_cm0.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the SPI(SCB 
* mode) component in slave mode using high level PDL function. The SPI master sends  
* the packet with a command to the SPI slave to control the RGB LED color. The packet 
* with a status is read back by master. CM0 acts as a slave. CM4 acts as a master and 
* sends various command packets to change the color of RGB LED on the slave 
* (Note : Master on CM4 is developed using low level PDL functions).
*
* Related Document: CE221121_PSoC6MCU_SPI_Slave.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit 
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

/***************************************
*         Function Prototypes
****************************************/
static void InitializeEnablePWM(void);
static uint32_t SPIS_WaitForCommandAndExecute(void);
static void SPIS_WriteDummyPacket(void);
static void HandleError(void);
static void SPIS_UpdateStatus(uint32_t status);
static void SlaveExecuteCommand( void );

/* Buffer and packet size */
#define PACKET_SIZE          (05UL)

/* Buffer and packet size in the slave */
#define SL_RD_BUFFER_SIZE    (PACKET_SIZE)
#define SL_WR_BUFFER_SIZE    (PACKET_SIZE)

/* Start and end of packet markers */
#define PACKET_SOP           (0x01UL)
#define PACKET_EOP           (0x17UL)

/* Dummy value to be written to Tx FIFO */
#define DUMMY_VALUE           (0xF0UL)

/* Command valid status */
#define STS_CMD_DONE         (0x00UL)
#define STS_CMD_FAIL         (0x1FUL)

/* Status to indicate master has read status packet */
#define STS_READ_CMPLT        (0x1FUL)

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

/*******************************************************************************
* Global variables
*******************************************************************************/
/* Receive Buffer */
uint8_t sRxBuffer[PACKET_SIZE];

/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
*  The main function performs the following actions:
*   1. Sets up SPI component to acts as Slave.
*   2. If initialization of SPI component fails, system will be in infinite loop.
*   3. Initializes PWM components to control RGB LED. If initialization of PWM component fails, system will be in infinite loop.
*   4. SPI slave component receives packets from master and configure the PWM to drive RGB LED.
*   5. Slave replys with the acknowledgment packet .
*
*******************************************************************************/
int main(void)
{
    cy_en_scb_spi_status_t initStatus;
    cy_en_sysint_status_t  sysStatus;
    uint32_t status = 0;
    
    /* Initialize and enable TCPWM Components */
    InitializeEnablePWM();    
    
     /* Initialize and enable SPI Component in slave mode. If initialization fails process error */
    initStatus = Cy_SCB_SPI_Init(sSPI_HW, &sSPI_config, &sSPI_context);
    if(initStatus != CY_SCB_SPI_SUCCESS)
    {
        HandleError();
    }
    
    /* Set active slave select to line 0 */
     Cy_SCB_SPI_SetActiveSlaveSelect(sSPI_HW, sSPI_SPI_SLAVE_SELECT0);
    
     /* Hook interrupt service routine */
    sysStatus = Cy_SysInt_Init(&sSPI_SCB_IRQ_cfg, &sSPI_Interrupt);
    if(sysStatus != CY_SYSINT_SUCCESS)
    {
        HandleError();
    }
    
    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ((IRQn_Type) sSPI_SCB_IRQ_cfg.intrSrc);
    
    Cy_SCB_SPI_Enable(sSPI_HW);
    
     /* Writes dummy values to tx buffer */   
    SPIS_WriteDummyPacket();
   
    __enable_irq(); /* Enable global interrupts. */
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 
   
    for(;;)
    {
        /* Checks if any data is written into SLave RX FIFO. If written, data packets are checked and 
           if it's correct TCPWM compare value is set according to received data. Status is written
           by slave to its's TX FIFO. After Master reads status, FIFO is cleared and slave waits for 
           next packet to receive.*/
        status = SPIS_WaitForCommandAndExecute();
        if(status != STS_READ_CMPLT)
        {
            SPIS_UpdateStatus(status);
        } 
       
    }
}

/*******************************************************************************
* Function Name: InitializeEnablePWM
****************************************************************************//**
*
* This function initializes and enables the TCPWM Componet for controllling RGB LED.
*
* \param None
* \note
*
* \return
*  None
*
*******************************************************************************/
static void InitializeEnablePWM(void)
{
    cy_en_tcpwm_status_t initPWMstatus;
    
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
 
}

/*******************************************************************************
* Function Name: SPIS_UpdateStatus
****************************************************************************//**
*
* This function updates the status by writing status value to SPI TX buffer.
*
* \param status
* \note: This is the parameter which will be updated to Tx buffer with header and footer data.
*
* \return
*  None
*
*******************************************************************************/
static void SPIS_UpdateStatus(uint32_t status)
{  
    /* Transmit Buffer */
    uint8_t sTxBuffer[PACKET_SIZE] = {PACKET_SOP, STS_CMD_FAIL, PACKET_EOP, DUMMY_VALUE, DUMMY_VALUE};
    
    sTxBuffer[PACKET_STS_POS] = (uint8_t)status;
    Cy_SCB_SPI_Transfer(sSPI_HW, sTxBuffer, sRxBuffer, PACKET_SIZE, &sSPI_context);
}

/*******************************************************************************
* Function Name: SPIS_WriteDummyPacket
****************************************************************************//**
*
* This function writes the dummy values to TX FIFO. When master sends data to slave this
* data will be read by master.
*
* \param None
* \note
*
* \return
*  None
*
*******************************************************************************/
static void SPIS_WriteDummyPacket(void)
{     
   Cy_SCB_SPI_Transfer(sSPI_HW, NULL, sRxBuffer, PACKET_SIZE, &sSPI_context); 
}

/*******************************************************************************
* Function Name: SPIS_WaitForCommandAndExecute
****************************************************************************//**
*
* This function waits for command from master by checking SPI slave RX FIFO. If packets are received
* it is checked and respective data is set for TCPWM compare value. If status read command is received 
* packets are not checked.
*
* \param None
* \note
*
* \return
* Checks the command received and returns the status after execution of received command.
* ref uint32_t
*
*******************************************************************************/
static uint32_t SPIS_WaitForCommandAndExecute(void)
{
    uint32_t slaveStatus;
    uint32_t status = STS_CMD_FAIL;
    
    do
    {
        slaveStatus  = Cy_SCB_SPI_GetTransferStatus(sSPI_HW, &sSPI_context); 
        
    } while ((0UL != (slaveStatus & CY_SCB_SPI_TRANSFER_ACTIVE)) );
   
    /* Check start and end of packet markers. If correct set the received TCPWM compare value. If packet 
       consists of PACKET_SOP and PACKET_EOP then the SPI master has sent command to set the RGB value. Otherwise SPI
       Master has sent command to read status for previously sent command. */
    if ((sRxBuffer[PACKET_SOP_POS] == PACKET_SOP) &&
        (sRxBuffer[PACKET_EOP_POS] == PACKET_EOP))
    {
        SlaveExecuteCommand( );
        status = STS_CMD_DONE;
    }else
    {
        status = STS_READ_CMPLT;
        
        /* Write dummy packets to Tx FIFO to receive next command from master. */
        SPIS_WriteDummyPacket();
    } 
  
    return status;
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
    /*Sets the compare value to control the color of RGB LED. */
    Cy_TCPWM_PWM_SetCompare0(RedPWM_HW, RedPWM_CNT_NUM, sRxBuffer[COLOR_RED_POS]);
    Cy_TCPWM_PWM_SetCompare0(GreenPWM_HW, GreenPWM_CNT_NUM, sRxBuffer[COLOR_GREEN_POS]);
    Cy_TCPWM_PWM_SetCompare0(BluePWM_HW, BluePWM_CNT_NUM, sRxBuffer[COLOR_BLUE_POS]);
}

/*******************************************************************************
* Function Name: HandleError
****************************************************************************//**
*
* This function processes unrecoverable errors such as any component 
* initialization errors etc. In case of such error the system will 
* stay in the infinite loop of this function.
*
*
* \note
* * If error occurs interrupts are disabled.
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



