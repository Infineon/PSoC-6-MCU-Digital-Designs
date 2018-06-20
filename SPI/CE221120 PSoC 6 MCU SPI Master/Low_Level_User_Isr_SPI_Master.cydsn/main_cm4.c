/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the SPI 
* (SCB mode) component in master mode using low level PDL function and user enabled interrupt. 
* This example uses user enabled interrupt method to complete the SPI transfers. 
* The SPI master sends the packet with a command to the SPI slave to control the 
* RGB LED color. The packet with a status is read back by master. CM4 acts as a master
* and sends various command packets to change the color of RGB LED on the slave.
* CM0 acts as a slave (Note: Slave on CM0 is developed using low level PDL functions) .
*
* Related Document: CE221120_PSoC6MCU_SPI_Master.pdf
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

/* Function prototypes */
static void WriteCommandPacket(uint32_t cmd);
static void CreateCommandPacketBuffer(uint32_t cmd);
static uint32_t checkStatusPacket(void);
static void ReadStatusPacket(void);
static void HandleError(void);
void ISR_SPI(void);

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

/* Dummy value to be written to Tx FIFO */
#define DUMMY_VALUE           (0xF0UL)

/* Command valid status */
#define TRANSFER_CMPLT              (0x00UL)
#define TRANSFER_IN_PROGRESS        (0x01UL)
#define TRANSFER_ERROR              (0x0FUL)
#define STATUS_READ                 (0x10UL)
#define STATUS_READ_IN_PROGRESS     (0x11UL)
#define STAUS_READ_CMPLT            (0x12UL)
#define STATUS_READ_ERROR           (0x1FUL)

/* Colour Code  */
#define COLOR_RED       (0x00UL)
#define COLOR_GREEN     (0x01UL)
#define COLOR_BLUE      (0x02UL)
#define COLOR_CYAN      (0x03UL)
#define COLOR_PURPLE    (0x04UL)
#define COLOR_YELLOW    (0x05UL)
#define COLOR_WHITE     (0x06UL)

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
#define STATUS_READ_DELAY     (10UL)

/*******************************************************************************
* Global variables
*******************************************************************************/
uint8_t  txBuffer[TX_PACKET_SIZE];
uint32_t masterTranStatus = TRANSFER_CMPLT;


/***************************************************************************//**
* Function Name: ISR_SPI
********************************************************************************
*
* Summary:
*  This function is registered to be called when SPI interrupt occurs
*  (Note that only SPI Master Done interrupt is enabled). 
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void ISR_SPI(void)
{
    uint32_t masterStatus;
    
    /* Check the status of master */
    masterStatus  = Cy_SCB_SPI_GetSlaveMasterStatus(mSPI_HW);
    if(masterStatus == CY_SCB_SPI_MASTER_DONE)
    {   
        /*Check number of elements in RX FIFO */
        if(TX_PACKET_SIZE == Cy_SCB_SPI_GetNumInRxFifo(mSPI_HW))
        {
            if(masterTranStatus == TRANSFER_IN_PROGRESS)
            {
                /* If master has sent command for RGB LED.*/
                masterTranStatus = STATUS_READ;  
                Cy_SCB_SPI_ClearRxFifoStatus(mSPI_HW, CY_SCB_SPI_RX_INTR_MASK);
                Cy_SCB_SPI_ClearRxFifo(mSPI_HW);
            }
            else
            {
                 /* If master has sent command for status read.*/
                masterTranStatus = STAUS_READ_CMPLT;
            }           
        }
        else
        {
            /*If number of elements is less in RX FIFO*/
            if(masterTranStatus == TRANSFER_IN_PROGRESS)
            {
                 masterTranStatus = TRANSFER_ERROR;   
            }
            else
            {
                masterTranStatus = STATUS_READ_ERROR;
            }  
        }           
    }
    /*Clear status and TX FIFO */
    Cy_SCB_SPI_ClearSlaveMasterStatus(mSPI_HW, masterStatus);
    Cy_SCB_SPI_ClearTxFifoStatus(mSPI_HW, CY_SCB_SPI_TX_INTR_MASK );
    Cy_SCB_SPI_ClearTxFifo(mSPI_HW);
}

/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
*  The main function performs the following actions:
*   1. Sets up SPI component to acts as master.
*   2. If initialization of SPI component system will be in infinite loop.
*   3. SPI master sends commands packets every two seconds to SPI slave using user enabled interrupt and
*   low level PDL function to change to color of RGB LED on the slave.
*   4. Master reads the reply from the slave to know whether commands are received properly.
*
*******************************************************************************/
int main(void)
{
    uint32_t cmd = COLOR_RED;
    cy_en_scb_spi_status_t initStatus;
            
    /* Configure component */
    initStatus = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &mSPI_context);
    if(initStatus != CY_SCB_SPI_SUCCESS)
    {
        HandleError();
    }
    
    /* Set active slave select to line 0 */
    Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, mSPI_SPI_SLAVE_SELECT0);
    
    //Cy_SysInt_Init(&mSPI_SCB_IRQ_cfg, &ISR_SPI);
    
    /* Unmasking only the spi done interrupt bit */
    mSPI_HW->INTR_M_MASK = SCB_INTR_M_SPI_DONE_Msk;

    /* Configure User ISR */
    Cy_SysInt_Init(&ISR_SPI_cfg, &ISR_SPI);
    
    /* Enable the interrupt */
    NVIC_EnableIRQ(ISR_SPI_cfg.intrSrc);
   
    /* Enable SPI master hardware. */
    Cy_SCB_SPI_Enable(mSPI_HW);
    
    __enable_irq(); /* Enable global interrupts. */
    
    for(;;)
    {   
         /* Write command to Tx FIFO of SPI master. Initial value of masterTranStatus is TRANSFER_CMPLT. */
        if(masterTranStatus == TRANSFER_CMPLT)
        {
            WriteCommandPacket(cmd);    
            
        }
        else if(masterTranStatus == STATUS_READ)
        {
            /* Send command to read status from slave for previously sent command for setting color of RGB LED  */
            ReadStatusPacket();  
            
        }
        else if(masterTranStatus == STAUS_READ_CMPLT)
        {
            /* After receiving status packet check whether status is correct */
            if(TRANSFER_CMPLT == checkStatusPacket())
            {
                masterTranStatus = TRANSFER_CMPLT;
                 /* Next command to be written. */
                cmd++;
                if(cmd > COLOR_WHITE)
                {
                    cmd = COLOR_RED;
                }    
            }
            else
            {
                /* If status read is incorrect handle error */
                HandleError();   
            }
             /* Give 2 Second delay between commands. */
            Cy_SysLib_Delay(CMD_TO_CMD_DELAY);  
            
        } 
        else if((masterTranStatus == TRANSFER_ERROR) &&  
                (masterTranStatus == STATUS_READ_ERROR))
        {
            /* If any error occurs handle the error  */         
            HandleError(); 
        }
    } 
}

/*******************************************************************************
* Function Name: WriteCommandPacket
****************************************************************************//**
*
* Buffer is assigned with data to be sent to slave.
* Low level PDL libray function is used to control SPI SCB to send data.
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
* Return:
*  None
*
*******************************************************************************/
static void WriteCommandPacket(uint32_t cmd)
{    
    /* Create packet to be sent to slave. */
    CreateCommandPacketBuffer(cmd);
    /* Send packet to SPI slave  */
    Cy_SCB_SPI_WriteArrayBlocking(mSPI_HW, txBuffer, TX_PACKET_SIZE);
    masterTranStatus = TRANSFER_IN_PROGRESS;      
}

/*******************************************************************************
* Function Name: CreateCommandPacketBuffer
****************************************************************************//**
*
*  This function initializes the buffer with data related to color.
*  The status of the transfer is returned.
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
        
        default:
            break;  
    } 
}

/*******************************************************************************
* Function Name: ReadStatusPacket
****************************************************************************//**
*
*  Master initiates to read status packet from the slave.
*  Master sends dummy packet to read status packet.
*
* \param None
*
* \return
*  None
*******************************************************************************/
static void ReadStatusPacket(void)
{
    uint8_t dummyTxBuf[TX_PACKET_SIZE] = {DUMMY_VALUE, DUMMY_VALUE, DUMMY_VALUE, DUMMY_VALUE, DUMMY_VALUE};
   
    /* Send dummy packet to slave to read status */
    Cy_SCB_SPI_WriteArrayBlocking(mSPI_HW, dummyTxBuf, TX_PACKET_SIZE);
    
    /* set masterTranStatus flag with value STATUS_READ_IN_PROGRESS */
    masterTranStatus = STATUS_READ_IN_PROGRESS;
}

/*******************************************************************************
* Function Name: checkStatusPacket
****************************************************************************//**
*
*  This function checks whether staus packet read by master is correct.
* 
* \param None
*
* \return
* Checks the status packet and returns the status.
* ref uint32_t
*
* \note
* * If the staus packect read is correct function returns TRANSFER_CMPLT and
*   if staus packet is incorrect function returns TRANSFER_ERROR.
*
*******************************************************************************/
static uint32_t checkStatusPacket(void)
{
    uint32_t status = TRANSFER_ERROR;
    uint8_t statusRxBuf[TX_PACKET_SIZE];
    uint8_t stsPacket[STS_PACKET_SIZE];
    uint32_t i;
    uint32_t j = 0;
   
    Cy_SCB_SPI_ReadArray(mSPI_HW, statusRxBuf, TX_PACKET_SIZE);
    Cy_SCB_SPI_ClearRxFifoStatus(mSPI_HW, CY_SCB_SPI_RX_INTR_MASK);
    Cy_SCB_SPI_ClearRxFifo(mSPI_HW);
    
    for(i=0; i<TX_PACKET_SIZE; i++)
    {
        if((statusRxBuf[i] != (0xFF)) && (statusRxBuf[i] != (DUMMY_VALUE)))
        {
            stsPacket[j] =  statusRxBuf[i]; 
            j++;
        }
    }
    
    if((PACKET_SOP   == stsPacket[RX_PACKET_SOP_POS]) &&
       (PACKET_EOP   == stsPacket[RX_PACKET_EOP_POS]) &&
       (STS_CMD_DONE == stsPacket[RX_PACKET_STS_POS]) )
    {
        status = TRANSFER_CMPLT;
    }
    
    return status;  
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

/* [] END OF FILE */




