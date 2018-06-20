/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the SPI(SCB 
* mode) component in master mode using high level PDL function. The SPI master sends the packet 
* with a command to the SPI slave to control the RGB LED color. The packet with a status
* is read back by master. CM4 acts as a master and sends various command packets to change 
* the color of RGB LED on the slave. CM0 acts as a slave (Note: Slave on CM0 is developed
* using low level PDL functions).
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
static void HandleError(void);
static uint32_t WriteCommandPacket(uint32_t cmd);
static void CreateCommandPacketBuffer(uint32_t cmd);
static uint32_t ReadStatusPacket(void);

/* Combine master error statuses in single mask */
#define MASTER_ERROR_MASK  (CY_SCB_SPI_SLAVE_TRANSFER_ERR  | CY_SCB_SPI_TRANSFER_OVERFLOW    | \
                            CY_SCB_SPI_TRANSFER_UNDERFLOW)  
/***************************************
*            Constants
****************************************/

/* Buffer and packet size */
#define TX_PACKET_SIZE        (5UL)
#define RX_PACKET_SIZE        (5UL)
#define ONE_PCKT_SIZE         (1UL)

/* Start and end of packet markers */
#define PACKET_SOP            (0x01UL)
#define PACKET_EOP            (0x17UL)

/* Command valid status */
#define TRANSFER_CMPLT        (0x00UL)
#define TRANSFER_ERROR        (0xFFUL)
#define READ_CMPLT            (TRANSFER_CMPLT)
#define READ_ERROR            (TRANSFER_ERROR)

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
#define STATUS_READ_DELAY     (1UL)
#define ONE_MS_DELAY          (1UL)

/*******************************************************************************
* Global variables
*******************************************************************************/
/* Transmit buffer */
uint8_t  txBuffer[TX_PACKET_SIZE];


/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
*  The main function performs the following actions:
*   1. Sets up SPI component to acts as master.
*   2. If initialization of SPI component system will be in infinite loop.
*   3. SPI master sends commands packets every two seconds to SPI slave using high level PDL function 
*   to change to color of RGB LED on the slave.
*   4. Master reads the reply from the slave to know whether commands are received properly.
*
*******************************************************************************/
int main(void)
{
    uint32_t cmd = COLOR_RED;
    cy_en_scb_spi_status_t initStatus;
    cy_en_sysint_status_t sysSpistatus;
        
    /* Configure component */
    initStatus = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &mSPI_context);
    if(initStatus != CY_SCB_SPI_SUCCESS)
    {
        HandleError();
    }
    
    /* Set active slave select to line 0 */
    Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, mSPI_SPI_SLAVE_SELECT0);
    
    /* Hook interrupt service routine */
    sysSpistatus = Cy_SysInt_Init(&mSPI_SCB_IRQ_cfg, &mSPI_Interrupt);
    if(sysSpistatus != CY_SYSINT_SUCCESS)
    {
        HandleError();
    }
    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ((IRQn_Type) mSPI_SCB_IRQ_cfg.intrSrc);
       
    /* Enable SPI master hardware. */
    Cy_SCB_SPI_Enable(mSPI_HW);
      
    __enable_irq(); /* Enable global interrupts. */

    for(;;)
    {
        /* Write command to TX FIFO to be sent to SPI slave. */
        if (TRANSFER_CMPLT == WriteCommandPacket(cmd))
        { 
            /* Read response packet from the slave. */
            if (READ_CMPLT == ReadStatusPacket())
            {
                /* Next command to be written. */
                cmd++;
                if(cmd > COLOR_WHITE)
                {
                    cmd = COLOR_RED;
                }      
            }
            /* Give 2 Second delay between commands. */
            Cy_SysLib_Delay(CMD_TO_CMD_DELAY);
        }
    }
}

/*******************************************************************************
* Function Name: WriteCommandPacket
****************************************************************************//**
*
* Buffer is assigned with data to be sent to slave.
* High level PDL libray function is used to control SPI SCB to send data.
* Errors are handled depend on the return value from the appropriate function.
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
* returns the status after command is written to slave.
* TRANSFER_ERROR is returned if any error occurs.
* TRANSFER_CMPLT is returned if write is successfull.
* \ref uint32_t
*
*******************************************************************************/
static uint32_t WriteCommandPacket(uint32_t cmd)
{
    uint32_t status = TRANSFER_ERROR;
    cy_en_scb_spi_status_t errorStatus;
    uint8_t statusRxBuf[TX_PACKET_SIZE];
        
    /* Create packet to be sent to slave. */
    CreateCommandPacketBuffer(cmd);
    
    /* Initiate SPI Master write and read transaction. */
    errorStatus = Cy_SCB_SPI_Transfer(mSPI_HW, txBuffer, statusRxBuf, TX_PACKET_SIZE, &mSPI_context);
    
    /* If no error wait till master sends data in Tx FIFO */
    if(errorStatus == CY_SCB_SPI_SUCCESS)
    {
        uint32_t masterStatus;        
        /* Timeout 1 sec */
        uint32_t timeOut = 1000UL;
        
        /* Wait until master complete read transfer or time out has occured */
        do
        {
            masterStatus  = Cy_SCB_SPI_GetTransferStatus(mSPI_HW, &mSPI_context);
            Cy_SysLib_Delay(CY_SCB_WAIT_1_UNIT);
            timeOut--;
            
        } while ((0UL != (masterStatus & CY_SCB_SPI_TRANSFER_ACTIVE)) && (timeOut > 0UL));
        
        
        if ((0UL == (MASTER_ERROR_MASK & masterStatus)) &&
            (TX_PACKET_SIZE == Cy_SCB_SPI_GetNumTransfered(mSPI_HW, &mSPI_context)))
        {
            status = TRANSFER_CMPLT;
            
        }
        else 
        {
            HandleError();
        } 
    }
    
    return (status);
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
*  The status of the previous command transfer is returned.
*
* \param None
*
* \return
* Checks the status packet and returns the status.
* ref uint32_t
*
* \note
*  If the staus packect read is correct function returns TRANSFER_CMPLT and
*  if staus packet is incorrect function returns TRANSFER_ERROR.
*
*******************************************************************************/
static uint32_t ReadStatusPacket(void)
{
    uint32_t count = 0;
    uint32_t tempVar;
    uint32_t timeOutStatusCheck = 1000UL; /* Timeout 1 sec */
    uint8_t statusRxBuf[RX_PACKET_SIZE];
    cy_en_scb_spi_status_t errorStatus;
    uint32_t status = TRANSFER_ERROR;
    
    /* Initiate SPI Master write and read transaction. Master will send one byte and receives one byte.
       Received byte is checked whether valid status byte is received. Till valid 5 bytes are received or timeout
       occurs, master will be sending dummy byte to slave. Valid bytes received is checked and status is set. */
    do
    {
        errorStatus = Cy_SCB_SPI_Transfer(mSPI_HW, NULL, &tempVar, ONE_PCKT_SIZE, &mSPI_context);
         
        if(errorStatus == CY_SCB_SPI_SUCCESS)
        {
            uint32_t masterStatus;         
            /* Timeout 1 sec */
            uint32_t timeOut = 1000UL;
            
            /* Wait until master complete read transfer or time out has occured */
            do
            {
                masterStatus  = Cy_SCB_SPI_GetTransferStatus(mSPI_HW, &mSPI_context);
                Cy_SysLib_Delay(CY_SCB_WAIT_1_UNIT);
                timeOut--;
                
            } while ((0UL != (masterStatus & CY_SCB_SPI_TRANSFER_ACTIVE)) && (timeOut > 0UL) );
            
            if ((0UL == (MASTER_ERROR_MASK & masterStatus)) && (tempVar != 0xFF) )
            {  
                statusRxBuf[count] = (uint8_t)tempVar;
                count++;
         
                if(count == RX_PACKET_SIZE)
                {
                    /* Check packet structure and set status */
                    if((PACKET_SOP   == statusRxBuf[RX_PACKET_SOP_POS]) &&
                        (PACKET_EOP   == statusRxBuf[RX_PACKET_EOP_POS]) &&
                        (STS_CMD_DONE == statusRxBuf[RX_PACKET_STS_POS]) )
                    {
                        status = TRANSFER_CMPLT;
                    }       
                }
            }
            else 
            {
                HandleError();
            }
        }     
        
        Cy_SysLib_Delay(CY_SCB_WAIT_1_UNIT);
        timeOutStatusCheck--;
        
    }while((count < RX_PACKET_SIZE) && (timeOutStatusCheck > 0));
    
    if(timeOutStatusCheck == (0UL))
    {
        HandleError();       
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
* \param None
* \note
*
* \return
*  None
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




