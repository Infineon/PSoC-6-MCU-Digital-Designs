/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the SPI(SCB 
* mode) component in master mode using DMA and low level PDL function. The SPI master sends the packet 
* with a command to the SPI slave to control the RGB LED color. The packet with a status
* is read back by master. CM4 acts as a master and sends various command packets to change 
* the color of RGB LED on the slave. CM0 acts as a slave (Note: Slave on CM0 is developed
* using low level PDL functions).
*
* Related Document: CE221120.pdf
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
static void CreateCommandPacketBuffer(uint32_t cmd);
static void HandleError(void);
static void ConfigureRxDma(void);
static void ConfigureTxDma(void);
static void RxDmaComplete(void);
static void TxDmaComplete(void);
static void SPIIntHandler(void);
static void WriteCommandPacket(uint32_t cmd);
static void ReadStatusPacket(void);
static uint32_t checkStatusPacket(void);

/***************************************
*            Constants
****************************************/

/* Buffer and packet size */
#define TX_PACKET_SIZE        (5UL)
#define RX_PACKET_SIZE        (5UL)
#define STS_PACKET_SIZE       (3UL)

/* Start and end of packet markers */
#define PACKET_SOP            (0x01UL)
#define PACKET_EOP            (0x17UL)

/* Command valid status */
#define TRANSFER_CMPLT        (0x00UL)
#define TRANSFER_ERROR        (0xFFUL)
#define READ_CMPLT            (TRANSFER_CMPLT)
#define READ_ERROR            (TRANSFER_ERROR)

#define TRANSFER_CMPLT              (0x00UL)
#define TRANSFER_ERROR              (0xFFUL)
#define TRANSFER_IN_PROGRESS        (0x10UL)
#define STATUS_READ                 (0x11UL)
#define STATUS_READ_IN_PROGRESS     (0x12UL)
#define STAUS_READ_CMPLT            (0x13UL)
#define STATUS_READ_ERROR           (0x14UL)

/* Colour Code  */
#define COLOR_RED       (0x00UL)
#define COLOR_GREEN     (0x01UL)
#define COLOR_BLUE      (0x02UL)
#define COLOR_CYAN      (0x03UL)
#define COLOR_PURPLE    (0x04UL)
#define COLOR_YELLOW    (0x05UL)
#define COLOR_WHITE     (0x06UL)
#define DUMMY_VALUES    (0x07UL)

/* Command valid status */
#define STS_CMD_DONE    (0x00UL)
#define STS_CMD_FAIL    (0x1FUL)

/* Packet positions */
#define PACKET_SOP_POS        (0UL)
#define PACKET_CMD_1_POS      (1UL)
#define PACKET_CMD_2_POS      (2UL)
#define PACKET_CMD_3_POS      (3UL)
#define PACKET_EOP_POS        (4UL)
#define PACKET_STS_POS        (1UL)
#define RX_PACKET_SOP_POS     (0UL)
#define RX_PACKET_STS_POS     (1UL)
#define RX_PACKET_EOP_POS     (2UL)

/* Delays in milliseconds */
#define CMD_TO_CMD_DELAY      (2000UL)
#define STATUS_READ_DELAY     (1UL)

/*******************************************************************************
* Global variables
*******************************************************************************/
/* Transmit buffer */
uint8_t  txBuffer[TX_PACKET_SIZE];
/* Receive buffer */
uint8_t  rxBuffer[TX_PACKET_SIZE];
/* Staus Data Buffer */
uint8_t statusBuffer[STS_PACKET_SIZE];
/* SPI Master status flag */
uint32_t masterTranStatus = TRANSFER_CMPLT;
/* Rx DMA complete status flag */
uint32_t rxDmaCmplt = false;
/* Count for number of bytes received. */
uint32_t count = 0;

/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
*  The main function performs the following actions:
*   1. Sets up SPI component to acts as master.
*   2. If initialization of SPI component system will be in infinite loop.
*   3. Configures Tx and Rx DMA to handle data transfer and receive from SPI master.
*   4. SPI master sends commands packets every two seconds to SPI slave using DMA and low level PDL function 
*      to change to color of RGB LED on the slave.
*   5. Master reads the reply from the slave to know whether commands are received properly.
*
*******************************************************************************/
int main(void)
{
    uint32_t cmd = COLOR_RED;
    cy_en_scb_spi_status_t initStatus;
    cy_en_sysint_status_t sysSpistatus;
    /* Timeout 1 sec */
    uint32_t timeOut = 1000000UL; 
   
     /* Configure component */
    initStatus = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &mSPI_context);
    if(initStatus != CY_SCB_SPI_SUCCESS)
    {
        HandleError();
    }
    
    /* Set active slave select to line 0 */
    Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, mSPI_SPI_SLAVE_SELECT0);
    
     /* Unmasking only the spi done interrupt bit */
    mSPI_HW->INTR_M_MASK = SCB_INTR_M_SPI_DONE_Msk;
    
     /* Hook interrupt service routine */
    sysSpistatus = Cy_SysInt_Init(&intSpi_cfg, &SPIIntHandler);
    if(sysSpistatus != CY_SYSINT_SUCCESS)
    {
        HandleError();
    }
    
    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ((IRQn_Type) intSpi_cfg.intrSrc);
       
    /* Enable SPI master hardware. */
    Cy_SCB_SPI_Enable(mSPI_HW);
   
    /* Configure DMA Rx and Tx channels for operation */
    ConfigureTxDma();
    ConfigureRxDma();
    
    __enable_irq(); /* Enable global interrupts. */
      
    for(;;)
    {
        /* Write command to DMA Tx buffer and Enable DMA. Initial value of masterTranStatus is TRANSFER_CMPLT. */
        if((masterTranStatus == TRANSFER_CMPLT) && (rxDmaCmplt == false))
        {
            WriteCommandPacket(cmd);         
            
        }
        else if((masterTranStatus == STATUS_READ) && (rxDmaCmplt == true))
        {
            /* Checks SPI master done status and Rx DMA complete flag status and writes command to Tx buffer 
               to read status */
            
            /* Reset flag */
            rxDmaCmplt = false;
                       
            /* Update Tx buffer with status command to get status  */
            ReadStatusPacket();    
            
        }
        else if((masterTranStatus == STAUS_READ_CMPLT) && (rxDmaCmplt == true))
        {
            /*Checks masterTranStatus status set by SPI master done interrupt handler and Rx DMA complete flag */ 
                       
            /* Check status packet received to know SPI slave as executed previously sent command */
            if(TRANSFER_CMPLT == checkStatusPacket())
            {
                /* Reset flag */
                rxDmaCmplt = false;
                timeOut = 1000000UL; 
                count = 0UL;
                masterTranStatus = TRANSFER_CMPLT;
                
                 /* Next command to be written. */
                cmd++;
                if(cmd > COLOR_WHITE)
                {
                    cmd = COLOR_RED;
                }    
                
                /* Give 2 Second delay between commands. */
                Cy_SysLib_Delay(CMD_TO_CMD_DELAY);  
            
            }
            else if ((timeOut > 0) && (count < STS_PACKET_SIZE))
            {
                masterTranStatus = STATUS_READ;
                   
            }                   
        } 
        else if((masterTranStatus == TRANSFER_ERROR) &&  
                  (masterTranStatus == STATUS_READ_ERROR))
        {
            /* If any error occurs send same command to slave. */ 
            HandleError(); 
        }
        
        if(masterTranStatus == STATUS_READ_IN_PROGRESS)
        {
            Cy_SysLib_DelayUs(CY_SCB_WAIT_1_UNIT);
            timeOut--;
            /* If status is not received in one second or  handle the error. Also if count equals to STS_PACKET_SIZE
               error is handled because wrong or corrupt data is received. */
            if((timeOut == 0UL || count == STS_PACKET_SIZE))
            {
                HandleError();
            }   
        }
    }
}

/*******************************************************************************
* Function Name: CreateCommandPacketBuffer
****************************************************************************//**
*
*  This function initializes the buffer with data related to color.
*
* \param cmd
* Depending on this value packet is created for different colors and sent to slave.
*   - COLOR_RED:    set red color of the LED.
*   - COLOR_GREEN:  set green color of the LED.
*   - COLOR_BLUE:   set blue color of the LED.
*   - COLOR_CYAN:   set cyan color of the LED.
*   - COLOR_PURPLE: set purple color of the LED.
*   - COLOR_YELLOW: set yellow color of the LED.
*   - COLOR_WHITE:  set white color of the LED.
*   - DUMMY_VALUES: dummy Values to read status. 
* \ref uint32_t
*
* \return
*  None
*******************************************************************************/
static void CreateCommandPacketBuffer(uint32_t cmd)
{ 
    /* Initialize buffer with commands to be sent. */
    txBuffer[PACKET_SOP_POS] = PACKET_SOP;
    txBuffer[PACKET_EOP_POS] = PACKET_EOP;
    switch(cmd)
    {
        case COLOR_RED:
            txBuffer[PACKET_CMD_1_POS] = 0xFF;
            txBuffer[PACKET_CMD_2_POS] = 0x00;
            txBuffer[PACKET_CMD_3_POS] = 0x00;
            break;
        
        case COLOR_GREEN:
            txBuffer[PACKET_CMD_1_POS] = 0x00;
            txBuffer[PACKET_CMD_2_POS] = 0xFF;
            txBuffer[PACKET_CMD_3_POS] = 0x00;
            break;
        
        case COLOR_BLUE:
            txBuffer[PACKET_CMD_1_POS] = 0x00;
            txBuffer[PACKET_CMD_2_POS] = 0x00;
            txBuffer[PACKET_CMD_3_POS] = 0xFF;
            break;
        
        case COLOR_CYAN:
            txBuffer[PACKET_CMD_1_POS] = 0x00;
            txBuffer[PACKET_CMD_2_POS] = 0xFF;
            txBuffer[PACKET_CMD_3_POS] = 0xFF;
            break;
        
        case COLOR_PURPLE:
            txBuffer[PACKET_CMD_1_POS] = 0x7F;
            txBuffer[PACKET_CMD_2_POS] = 0x00;
            txBuffer[PACKET_CMD_3_POS] = 0x7F;
            break;
        
        case COLOR_YELLOW:
            txBuffer[PACKET_CMD_1_POS] = 0xFF;
            txBuffer[PACKET_CMD_2_POS] = 0xFF;
            txBuffer[PACKET_CMD_3_POS] = 0x00;
            break;
        
        case COLOR_WHITE:
            txBuffer[PACKET_CMD_1_POS] = 0xFF;
            txBuffer[PACKET_CMD_2_POS] = 0xFF;
            txBuffer[PACKET_CMD_3_POS] = 0xFF;
            break;
            
        case DUMMY_VALUES:
            txBuffer[PACKET_SOP_POS]   = 0xF0;
            txBuffer[PACKET_EOP_POS]   = 0xF0;
            txBuffer[PACKET_CMD_1_POS] = 0xF0;
            txBuffer[PACKET_CMD_2_POS] = 0xF0;
            txBuffer[PACKET_CMD_3_POS] = 0xF0;
         
        default:
            break;  
    } 
}

/*******************************************************************************
* Function Name: WriteCommandPacket
****************************************************************************//**
*
* Buffer is assigned with data to be sent to slave. After buffer is assigned with data
* DMA channel is enbaled for transfer of data from Tx buffer to SPI Tx FIFO and Rx FIFO to.
* Rx buffer.
*
* \param cmd
* Depending on this value packet is created for different colors and sent to slave.
*   - COLOR_RED:    set red color of the LED.
*   - COLOR_GREEN:  set green color of the LED.
*   - COLOR_BLUE:   set blue color of the LED.
*   - COLOR_CYAN:   set cyan color of the LED.
*   - COLOR_PURPLE: set purple color of the LED.
*   - COLOR_YELLOW: set yellow color of the LED.
*   - COLOR_WHITE:  set white color of the LED.
* \ref uint32_t
*
* \return
*  None
*
*******************************************************************************/
static void WriteCommandPacket(uint32_t cmd)
{      
    /* Create packet to be sent to slave. */
    CreateCommandPacketBuffer(cmd);   
    
    /* Enable tx and rx DMA channel. */
    Cy_DMA_Channel_Enable(rxDma_HW, rxDma_DW_CHANNEL);  
    Cy_DMA_Channel_Enable(txDma_HW, txDma_DW_CHANNEL);
    
    /* Set the flag for status checking. */
    masterTranStatus = TRANSFER_IN_PROGRESS; 
}

/*******************************************************************************
* Function Name: ReadStatusPacket
****************************************************************************//**
*
*  Master initiates to read status packet from the slave. It assigns dummy values for 
* tx buffer to read status packet from SPI slave. DMA channels are enables to send and receive
* data.
*
* \param None
*
* \return
*  None
*
*******************************************************************************/
static void ReadStatusPacket(void)
{  
    /* Create packet to be sent to slave to read status. */
    CreateCommandPacketBuffer(DUMMY_VALUES);
    
    /* Enable tx and rx DMA channel. */
    Cy_DMA_Channel_Enable(rxDma_HW, rxDma_DW_CHANNEL);
    Cy_DMA_Channel_Enable(txDma_HW, txDma_DW_CHANNEL);
     
    /* Set the flag for status checking. */
    masterTranStatus = STATUS_READ_IN_PROGRESS;
}


/*******************************************************************************
* Function Name: checkStatusPacket
****************************************************************************//**
*
* This function checks whether staus packet read by master is correct.
*  
* \return
* Checks the status packet and returns the status.
* ref uint32_t
*
* \param None
* \note
*
* \note
* * If the staus packect read is correct function returns TRANSFER_CMPLT anf
*   if staus packet is incorrect function returns TRANSFER_ERROR.
*
*******************************************************************************/
static uint32_t checkStatusPacket(void)
{
    uint32_t status = TRANSFER_ERROR;
    uint32_t i;
        
    for(i=0;i<5;i++)
    {
        if( (rxBuffer[i] != 0xFF) && (rxBuffer[i] != 0xF0) )
        {
            statusBuffer[count++] = rxBuffer[i];    
        }
    }
    
    if((PACKET_SOP   == statusBuffer[RX_PACKET_SOP_POS]) &&
       (PACKET_EOP   == statusBuffer[RX_PACKET_EOP_POS]) &&
       (STS_CMD_DONE == statusBuffer[RX_PACKET_STS_POS]))
    {
        status = TRANSFER_CMPLT;
    }
    
    return status;  
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
    Cy_DMA_Descriptor_SetSrcAddress(&rxDma_Descriptor_1, (void *)&mSPI_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&rxDma_Descriptor_1, (uint8_t *)rxBuffer);
    
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
    Cy_DMA_Descriptor_SetSrcAddress(&txDma_Descriptor_1, (uint8_t *)txBuffer);
    Cy_DMA_Descriptor_SetDstAddress(&txDma_Descriptor_1, (void *)&mSPI_HW->TX_FIFO_WR);
    
     /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init(&intTxDma_cfg, &TxDmaComplete);
    NVIC_EnableIRQ(intTxDma_cfg.intrSrc);
    
     /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(txDma_HW, txDma_DW_CHANNEL, txDma_INTR_MASK);
    
    Cy_DMA_Enable(txDma_HW);    
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
*  Handles Rx Dma descriptor completion interrupt source. Clears Interrupt and sets
*  the rxDmaCmplt flag.
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void RxDmaComplete(void)
{ 
    /* Clear rx DMA interrupt. */
    Cy_DMA_Channel_ClearInterrupt(rxDma_HW, rxDma_DW_CHANNEL);
    
    /* Check interrupt cause to capture errors. */
    if (CY_DMA_INTR_CAUSE_COMPLETION != Cy_DMA_Channel_GetStatus(rxDma_HW, rxDma_DW_CHANNEL)&&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(rxDma_HW, rxDma_DW_CHANNEL)))
    {
        /* DMA error occured while RX operations */
        HandleError();
    }     
   
    /* Set rxDmaCmplt flag. */   
    rxDmaCmplt = true;
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
    if ((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_DW_CHANNEL)) &&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_DW_CHANNEL)))
    {
        /* DMA error occured while TX operations */
        HandleError();
    }
    
    /* Clear tx DMA interrupt */
    Cy_DMA_Channel_ClearInterrupt(txDma_HW, txDma_DW_CHANNEL);   
    
}


/*******************************************************************************
* Function Name: SPIIntHandler
********************************************************************************
*
* Summary:
* Handles SPI Done Inteerupt.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void SPIIntHandler(void)
{
     uint32_t masterStatus;
    
    /* Check the status of master */
    masterStatus  = Cy_SCB_SPI_GetSlaveMasterStatus(mSPI_HW);
    
    if(masterStatus == CY_SCB_SPI_MASTER_DONE)
    {
        /* set the status depending on the command sent by master */
        if(masterTranStatus == TRANSFER_IN_PROGRESS)
        {
            masterTranStatus = STATUS_READ;
            
        }
        else if(masterTranStatus == STATUS_READ_IN_PROGRESS)
        {
            masterTranStatus = STAUS_READ_CMPLT;
        }
    
    }
    else
    {
        /* If error occurs set the relevant status */
        if(masterTranStatus == TRANSFER_IN_PROGRESS)
        {
            masterTranStatus = TRANSFER_ERROR;
            
        }
        else if(masterTranStatus == STATUS_READ_IN_PROGRESS)
        {
            masterTranStatus = STATUS_READ_ERROR;
        }        
    }
    
    Cy_SCB_SPI_ClearSlaveMasterStatus(mSPI_HW, masterStatus);
}
/* [] END OF FILE */



