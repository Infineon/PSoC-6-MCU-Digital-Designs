/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the SPI(SCB 
* mode) slave component using DMA and low level PDL function. The SPI master sends the packet 
* with a command to the SPI slave to control the RGB LED color. The packet with a status
* is read back by master. CM0 acts as a slave. CM4 acts as a master and sends various command 
* packets to change the color of RGB LED on the slave (Note : Master on CM4 is developed using
* low level PDL functions).
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
static void ConfigureRxDma(void);
static void ConfigureTxDma(void);
static void HandleError(void);
static void RxDmaComplete(void);
static void TxDmaComplete(void);
static void SPIS_WriteDummyPacket(void);
static uint32_t SPIS_CheckExecuteRecvdCmnd(void);
static void SPIS_UpdateStatus(uint32_t status);
static void SlaveExecuteCommand(void);

/* Buffer and packet size */
#define PACKET_SIZE          (05UL)

/* Buffer and packet size in the slave */
#define SL_RD_BUFFER_SIZE    (PACKET_SIZE)
#define SL_WR_BUFFER_SIZE    (PACKET_SIZE)

/* Start and end of packet markers */
#define PACKET_SOP           (0x01UL)
#define PACKET_EOP           (0x17UL)

/* Command valid status */
#define STS_CMD_DONE         (0x00UL)
#define STS_CMD_FAIL         (0x1FUL)

/* Dummy value to be written to Tx FIFO */
#define DUMMY_VALUE           (0xF0UL)

/* Packet positions */
#define PACKET_SOP_POS       (0UL)
#define PACKET_RED_POS       (1UL)
#define PACKET_GREEN_POS     (2UL)
#define PACKET_BLUE_POS      (3UL)
#define PACKET_EOP_POS       (4UL)
#define COLOR_RED_POS        (0x01UL)
#define COLOR_GREEN_POS      (0x02UL)
#define COLOR_BLUE_POS       (0x03UL)

/* Status packet positions */
#define STS_SOP_POS         (PACKET_SOP_POS)
#define STS_STS_POS         (1UL)
#define STS_EOP_POS         (2UL)

/* Status to indicate master has read status packet */
#define STS_READ_CMPLT        (0x1FUL)

/* Status Flags  */
#define WAIT_FOR_CMND_PCKT    (0x01UL)
#define CMND_RECEIVED         (0x02UL)

/*******************************************************************************
* Global variables
*******************************************************************************/
uint8_t sRxBuffer[PACKET_SIZE];
uint8_t sTxBuffer[PACKET_SIZE];                 
uint32_t cmndFlag = WAIT_FOR_CMND_PCKT;
uint32_t rxDmaCmplt = false;

/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
*  The main function performs the following actions:
*   1. Sets up SPI component to acts as Slave.
*   2. If initialization of SPI component fails, system will be in infinite loop.
*   3. Configures Tx and Rx DMA to handle data transfer and receive from SPI slave.
*   3. Initializes PWM components to control RGB LED. If initialization of PWM component fails, system will be in infinite loop.
*   4. SPI slave component receives packets from master and configures the PWM to drive RGB LED.
*   5. Slave replys with the acknowledgment packet .
*
*******************************************************************************/
int main(void)
{
    cy_en_scb_spi_status_t initStatus;
    uint32_t status;
    
    /* Initialize and enable TCPWM Components */
    InitializeEnablePWM();   
    
     /* Configure component */
    initStatus = Cy_SCB_SPI_Init(sSPI_HW, &sSPI_config, &sSPI_context);
    if(initStatus != CY_SCB_SPI_SUCCESS)
    {
        HandleError();
    }
    
    /* Set active slave select to line 0 */
    Cy_SCB_SPI_SetActiveSlaveSelect(sSPI_HW, sSPI_SPI_SLAVE_SELECT0);
            
    /* Enable SPI master hardware. */
    Cy_SCB_SPI_Enable(sSPI_HW);
    
    /* Configure DMA Rx and Tx channels for operation */
    ConfigureTxDma();
    ConfigureRxDma();
    
    /* Writes dummy values to tx buffer */
    SPIS_WriteDummyPacket();
    
    __enable_irq(); /* Enable global interrupts. */
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 

    for(;;)
    {
        /* Status is checked to know whether Rx DMA has written any command to receive buffer   */
        if(cmndFlag == CMND_RECEIVED && rxDmaCmplt == true)
        {
            rxDmaCmplt = false;
            status = SPIS_CheckExecuteRecvdCmnd();
            if(status != STS_READ_CMPLT)
            {
                SPIS_UpdateStatus(status);  
            }   
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
* Function Name: ConfigureRxDma
********************************************************************************
*
* Summary:
* Configures DMA Rx channel for operation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void ConfigureRxDma(void)
{
    cy_en_dma_status_t dma_init_status;
	cy_stc_dma_channel_config_t channelConfig;	
    
    /* Initialize descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&rxDma_Descriptor_1, &rxDma_Descriptor_1_config);
    if(dma_init_status!=CY_DMA_SUCCESS)
    {
        HandleError();
    }
    
    /* Initialize the DMA channel */
    channelConfig.descriptor  = &rxDma_Descriptor_1;
    channelConfig.preemptable = rxDma_PREEMPTABLE;
    channelConfig.priority    = rxDma_PRIORITY;
    channelConfig.enable      = false;

    dma_init_status = Cy_DMA_Channel_Init(rxDma_HW, rxDma_DW_CHANNEL, &channelConfig);
    if(dma_init_status!=CY_DMA_SUCCESS)
    {
        HandleError();
    }
    
     /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&rxDma_Descriptor_1, (void *)&sSPI_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&rxDma_Descriptor_1, (uint8_t *)sRxBuffer);
    
    /* Initialize and enable interrupt from RxDma */ 
    Cy_SysInt_Init  (&intRxDma_cfg, &RxDmaComplete);
    NVIC_EnableIRQ(intRxDma_cfg.intrSrc);
    
     /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(rxDma_HW, rxDma_DW_CHANNEL, rxDma_INTR_MASK);
    
     Cy_DMA_Enable(rxDma_HW);     
}

/*******************************************************************************
* Function Name: ConfigureTxDma
********************************************************************************
*
* Summary:
* Configures DMA Tx channel for operation.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void ConfigureTxDma(void)
{
    cy_en_dma_status_t dma_init_status;
	cy_stc_dma_channel_config_t channelConfig;	
    
    /* Initialize descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&txDma_Descriptor_1, &txDma_Descriptor_1_config);
    if(dma_init_status!=CY_DMA_SUCCESS)
    {
        HandleError();
    }
    
     /* Initialize the DMA channel */
    channelConfig.descriptor  = &txDma_Descriptor_1;
    channelConfig.preemptable = txDma_PREEMPTABLE;
    channelConfig.priority    = txDma_PRIORITY;
    channelConfig.enable      = false;
    
    dma_init_status = Cy_DMA_Channel_Init(txDma_HW, txDma_DW_CHANNEL, &channelConfig);
    if(dma_init_status!=CY_DMA_SUCCESS)
    {
        HandleError();
    }
    
    /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&txDma_Descriptor_1, (uint8_t *)sTxBuffer);
    Cy_DMA_Descriptor_SetDstAddress(&txDma_Descriptor_1, (void *)&sSPI_HW->TX_FIFO_WR);
    
     /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init(&intTxDma_cfg, &TxDmaComplete);
    NVIC_EnableIRQ(intTxDma_cfg.intrSrc);
    
    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(txDma_HW, txDma_DW_CHANNEL, txDma_INTR_MASK);       
}

/*******************************************************************************
* Function Name: SPIS_UpdateStatus
****************************************************************************//**
*
* This function updates the status by writing status value to TX buffer.
* After writing status txDma channel is enabled to transfer data from buffer to SPI Tx FIFO.
*
* \param status
* \note
*
* \return
*  None
*
*******************************************************************************/
static void SPIS_UpdateStatus(uint32_t status)
{
    /* Data to be sent to SPI slave Tx FIFO. */
    sTxBuffer[STS_SOP_POS] = PACKET_SOP;
    sTxBuffer[STS_STS_POS] = (uint8_t)status;
    sTxBuffer[STS_EOP_POS] = PACKET_EOP;
    sTxBuffer[3] = DUMMY_VALUE;
    sTxBuffer[4] = DUMMY_VALUE;
    
    /* Enable txDma channel to transfer data from buffer to SPI slave Tx FIFO. */ 
    Cy_DMA_Channel_Enable(txDma_HW, txDma_DW_CHANNEL);
    
    cmndFlag = WAIT_FOR_CMND_PCKT;
}

/*******************************************************************************
* Function Name: SPIS_WriteDummyPacket
****************************************************************************//**
*
* This function writes dummy values to TX buffer. After writing dummy values txDma 
* channel is enabled to transfer data from buffer to SPI Tx FIFO.
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
    uint32_t i;
    
    /* Data to be sent to SPI slave Tx FIFO. */
    for(i=0;i<PACKET_SIZE;i++)
    {
        sTxBuffer[i] = DUMMY_VALUE;   
    }
     
    /* Enable txDma channel to transfer data from buffer to SPI slave Tx FIFO. */ 
    Cy_DMA_Channel_Enable(txDma_HW, txDma_DW_CHANNEL);
   
    cmndFlag = WAIT_FOR_CMND_PCKT;
}

/*******************************************************************************
* Function Name: SPIS_CheckExecuteRecvdCmnd
****************************************************************************//**
*
* This function waits for command from master by checking SPI slave RX FIFO. If packets are received
* it is checked and respective data is set for TCPWM compare value.
*
* \param None
* \note
*
* \return
*  None
*
*******************************************************************************/
static uint32_t SPIS_CheckExecuteRecvdCmnd(void)
{
    uint32_t status = STS_CMD_FAIL;
   
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
        
        /* Write dummy packets to Tx buffer to receive next command from master. */
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
static void SlaveExecuteCommand(void)
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

/*******************************************************************************
* Function Name: RxDmaComplete
********************************************************************************
*
* Summary:
*  Handles Rx Dma descriptor completion interrupt source: Clears Interrupt and sets
*  rxDmaCmplt flag. cmndFlag is set with value CMND_RECEIVED for further processing of received data.
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void RxDmaComplete(void)
{ 
    /* Clear rx DMA interrupt */
    Cy_DMA_Channel_ClearInterrupt(rxDma_HW, rxDma_DW_CHANNEL);
    
    /* Check interrupt cause to capture errors. */
    if((CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(rxDma_HW, rxDma_DW_CHANNEL))&&
       (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(rxDma_HW, rxDma_DW_CHANNEL)))
    {
        /* DMA error occured while RX operations */
        HandleError();
    }     
     
    /* Set the status check flags. */
    rxDmaCmplt = true;
    
    if(cmndFlag == WAIT_FOR_CMND_PCKT)
    {
        cmndFlag = CMND_RECEIVED;
    }
}

/*******************************************************************************
* Function Name: TxDmaComplete
********************************************************************************
*
* Summary:
*  Handles Tx Dma descriptor completion interrupt source: only used for
*  indication.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void TxDmaComplete(void)
{
    /* Check tx DMA status */
    if((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_DW_CHANNEL)) &&
       (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_DW_CHANNEL)))
    {
        /* DMA error occured while TX operations */
        HandleError();
    }
    
    /* Clear tx DMA interrupt */
    Cy_DMA_Channel_ClearInterrupt(txDma_HW, txDma_DW_CHANNEL);   
    
    /* Enable rxDma channel */
    Cy_DMA_Channel_Enable(rxDma_HW, rxDma_DW_CHANNEL);
}

/* [] END OF FILE */




