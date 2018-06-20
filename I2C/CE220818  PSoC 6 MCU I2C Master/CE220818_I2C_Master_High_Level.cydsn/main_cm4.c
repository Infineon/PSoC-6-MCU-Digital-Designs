/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.0
*
* Description: This example project demonstrates the basic operation of the I2C master (SCB 
* mode) component using high level PDL function. The I2C master sends the packet with a command 
* to the I2C slave to control the RGB LED color. The packet with a status is read back. CM4 acts as a master and
* sends various command packets to change the color of RGB LED on the slave.
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

/***************************************
*         Function Prototypes
****************************************/

/* Function prototypes */
static uint8_t WriteCommandPacket(uint8_t cmd);
static void CreateCommandPacketBuffer(uint8_t cmd);
static uint8_t ReadStatusPacket(void);
static void HandleError(void);
static bool WaitOneUnit(uint32_t *timeout);

/* Combine master error statuses in single mask */
#define MASTER_ERROR_MASK  (CY_SCB_I2C_MASTER_DATA_NAK | CY_SCB_I2C_MASTER_ADDR_NAK    | \
                            CY_SCB_I2C_MASTER_ARB_LOST | CY_SCB_I2C_MASTER_ABORT_START | \
                            CY_SCB_I2C_MASTER_BUS_ERR)    
    
/***************************************
*            Constants
****************************************/

/* Colour Code  */
#define COLOR_RED       (0x00UL)
#define COLOR_GREEN     (0x01UL)
#define COLOR_BLUE      (0x02UL)
#define COLOR_CYAN      (0x03UL)
#define COLOR_PURPLE    (0x04UL)
#define COLOR_YELLOW    (0x05UL)
#define COLOR_WHITE     (0x06UL)

/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR  (0x24UL)

/* Buffer and packet size */
#define TX_PACKET_SIZE  (5UL)
#define RX_PACKET_SIZE  (3UL)

/* Start and end of packet markers */
#define PACKET_SOP      (0x01UL)
#define PACKET_EOP      (0x17UL)

/* Command valid status */
#define STS_CMD_DONE    (0x00UL)
#define STS_CMD_FAIL    (0xFFUL)

/* Command valid status */
#define TRANSFER_CMPLT    (0x00UL)
#define TRANSFER_ERROR    (0xFFUL)
#define READ_CMPLT        (TRANSFER_CMPLT)
#define READ_ERROR        (TRANSFER_ERROR)

/* Timeout */
#define LOOP_FOREVER        (0UL)

/* Packet positions */
#define PACKET_SOP_POS      (0UL)
#define PACKET_CMD_1_POS    (1UL)
#define PACKET_CMD_2_POS    (2UL)
#define PACKET_CMD_3_POS    (3UL)
#define PACKET_EOP_POS      (4UL)
#define PACKET_STS_POS      (1UL)
#define RX_PACKET_SOP_POS   (0UL)
#define RX_PACKET_STS_POS   (1UL)
#define RX_PACKET_EOP_POS   (2UL)

/* Delays in milliseconds */
#define CMD_TO_CMD_DELAY    (2000UL)

/*******************************************************************************
* Global variables
*******************************************************************************/
/* Structure for master transfer configuration */
cy_stc_scb_i2c_master_xfer_config_t masterTransferCfg =
{
    .slaveAddress = I2C_SLAVE_ADDR,
    .buffer       = NULL,
    .bufferSize   = 0U,
    .xferPending  = false
};

uint8_t  buffer[TX_PACKET_SIZE];

/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
*  The main function performs the following actions:
*   1. Sets up I2C component to acts as master.
*   2. If initialization of I2C component system will be in infinite loop.
*   3. I2C master sends commands packets every two seconds using high level PDL function 
*   to change to color of RGB LED on the slave.
*   4. Master reads the reply from the slave to know whether commands are received properly.
*
*******************************************************************************/
int main(void)
{
    cy_en_scb_i2c_status_t initStatus;
    cy_en_sysint_status_t sysStatus;
    uint32_t dataRateSet;
    uint8_t cmd = COLOR_RED;
    
    /* Initilaize and enable the I2C in master mode. */  
     initStatus = Cy_SCB_I2C_Init(mI2C_HW, &mI2C_config, &mI2C_context);
    if(initStatus != CY_SCB_I2C_SUCCESS)
    {
        HandleError();
    }
    /* Configure desired data rate. */
    dataRateSet = Cy_SCB_I2C_SetDataRate(mI2C_HW, mI2C_DATA_RATE_HZ, mI2C_CLK_FREQ_HZ);
    
    /* check whether data rate set is not greather then required reate. */
    if( dataRateSet > mI2C_DATA_RATE_HZ )
    {
        HandleError();
    }
    
    /* Hook interrupt service routine. */
    sysStatus = Cy_SysInt_Init(&mI2C_SCB_IRQ_cfg, &mI2C_Interrupt);
    if(sysStatus != CY_SYSINT_SUCCESS)
    {
        HandleError();
    }
    
    /* Enable interrupt in NVIC. */
    NVIC_EnableIRQ((IRQn_Type) mI2C_SCB_IRQ_cfg.intrSrc);
    
    /* Enable I2C master hardware. */
    Cy_SCB_I2C_Enable(mI2C_HW);
    
    /* Enable global interrupts. */
    __enable_irq(); 
        
    /***************************************************************************
    * Main polling loop
    ***************************************************************************/
    for(;;)
    {
        /* Place your application code here. */
        
        /* Send packet with command to the slave. */
        if (TRANSFER_CMPLT == WriteCommandPacket(cmd))
        {
            /* Read response packet from the slave. */
            if (TRANSFER_CMPLT == ReadStatusPacket())
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
* high level PDL libray function is used to control I2C SCB to send data.
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
* \ref uint8_t
*
* \return
* returns the status after command is written to slave.
* TRANSFER_ERROR is returned if any error occurs.
* TRANSFER_CMPLT is returned if write is successfull.
* \ref uint32_t
*
*******************************************************************************/
static uint8_t WriteCommandPacket(uint8_t cmd)
{
    uint8_t status = TRANSFER_ERROR;
    cy_en_scb_i2c_status_t  errorStatus;
  
    /* create packet to be sent to slave.  */
    CreateCommandPacketBuffer(cmd);
      
    /* Setup transfer specific parameters. */
    masterTransferCfg.buffer     = buffer;
    masterTransferCfg.bufferSize = TX_PACKET_SIZE;

    /* Initiate write transaction. */
    errorStatus = Cy_SCB_I2C_MasterWrite(mI2C_HW, &masterTransferCfg, &mI2C_context);
    if(errorStatus == CY_SCB_I2C_SUCCESS)
    {
        uint32_t masterStatus;
        bool     timeOutStatus;
        
        /* Timeout 1 sec (one unit is us) */
        uint32_t timeout = 1000000UL;
        /* Wait until master complete read transfer or time out has occured */
        do
        {
            masterStatus  = Cy_SCB_I2C_MasterGetStatus(mI2C_HW, &mI2C_context);
            timeOutStatus = WaitOneUnit(&timeout);
            
        } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (timeOutStatus == false));
        
        if (timeOutStatus) 
        {
            /* Timeout recovery */
            Cy_SCB_I2C_Disable(mI2C_HW, &mI2C_context);
            Cy_SCB_I2C_Enable (mI2C_HW);
        }
        else
        {
            if ((0u == (MASTER_ERROR_MASK & masterStatus)) &&
                (TX_PACKET_SIZE == Cy_SCB_I2C_MasterGetTransferCount(mI2C_HW, &mI2C_context)))
            {
                status = TRANSFER_CMPLT;
            }
        } 
    }
    
    return (status);
}


/*******************************************************************************
* Function Name: CreateCommandPacketBuffer
****************************************************************************//**
*
*  This function initializes the buffer with data related to color.
*  The status of the transfer is returned.
*
* \param *cmndBuffer
* Adrress of the buffer to which data will be assigned.
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
* \ref uint8_t
*
*******************************************************************************/
static void CreateCommandPacketBuffer(uint8_t cmd)
{ 
    /* Initialize buffer with commands to be sent. */
    buffer[PACKET_SOP_POS] = PACKET_SOP;
    buffer[PACKET_EOP_POS] = PACKET_EOP;
    switch(cmd)
    {
        case COLOR_RED:
            buffer[PACKET_CMD_1_POS] = 0xFF;
            buffer[PACKET_CMD_2_POS] = 0x00;
            buffer[PACKET_CMD_3_POS] = 0x00;
            break;
        
        case COLOR_GREEN:
            buffer[PACKET_CMD_1_POS] = 0x00;
            buffer[PACKET_CMD_2_POS] = 0xFF;
            buffer[PACKET_CMD_3_POS] = 0x00;
            break;
        
        case COLOR_BLUE:
            buffer[PACKET_CMD_1_POS] = 0x00;
            buffer[PACKET_CMD_2_POS] = 0x00;
            buffer[PACKET_CMD_3_POS] = 0xFF;
            break;
        
        case COLOR_CYAN:
            buffer[PACKET_CMD_1_POS] = 0x00;
            buffer[PACKET_CMD_2_POS] = 0xFF;
            buffer[PACKET_CMD_3_POS] = 0xFF;
            break;
        
        case COLOR_PURPLE:
            buffer[PACKET_CMD_1_POS] = 0x7F;
            buffer[PACKET_CMD_2_POS] = 0x00;
            buffer[PACKET_CMD_3_POS] = 0x7F;
            break;
        
        case COLOR_YELLOW:
            buffer[PACKET_CMD_1_POS] = 0xFF;
            buffer[PACKET_CMD_2_POS] = 0xFF;
            buffer[PACKET_CMD_3_POS] = 0x00;
            break;
        
        case COLOR_WHITE:
            buffer[PACKET_CMD_1_POS] = 0xFF;
            buffer[PACKET_CMD_2_POS] = 0xFF;
            buffer[PACKET_CMD_3_POS] = 0xFF;
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
*  The status of the transfer is returned.
*
* \return
* Checks the status packet and returns the status.
* ref uint32_t
*
* \note
* * If the staus packect read is correct function returns TRANSFER_CMPLT anf
*   if staus packet is incorrect function returns TRANSFER_ERROR.
*
*******************************************************************************/
static uint8_t ReadStatusPacket(void)
{      
    uint8_t status = TRANSFER_ERROR;
    cy_en_scb_i2c_status_t errorStatus;
    uint8_t  rxBuffer[RX_PACKET_SIZE];
  
    /* Setup transfer specific parameters */
    masterTransferCfg.buffer     = rxBuffer;
    masterTransferCfg.bufferSize = RX_PACKET_SIZE;

    /* Initiate read transaction */
    errorStatus = Cy_SCB_I2C_MasterRead(mI2C_HW, &masterTransferCfg, &mI2C_context);
    if(errorStatus == CY_SCB_I2C_SUCCESS)
    {
        uint32_t masterStatus;
        bool     timeOutStatus;
        
        /* Timeout 1 sec (one unit is us) */
        uint32_t timeout = 1000000UL;
        
        /* Wait until master complete read transfer or time out has occured */
        do
        {
            masterStatus  = Cy_SCB_I2C_MasterGetStatus(mI2C_HW, &mI2C_context);
            timeOutStatus = WaitOneUnit(&timeout);
            
        } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (timeOutStatus == false));
        
       if (timeOutStatus) 
        {
            /* Timeout recovery */
            Cy_SCB_I2C_Disable(mI2C_HW, &mI2C_context);
            Cy_SCB_I2C_Enable (mI2C_HW);
        }
        else
        {
            /* Check transfer status */
            if (0u == (MASTER_ERROR_MASK & masterStatus))
            {
                /* Check packet structure and status */
                if((PACKET_SOP   == rxBuffer[RX_PACKET_SOP_POS]) &&
                    (PACKET_EOP   == rxBuffer[RX_PACKET_EOP_POS]) &&
                    (STS_CMD_DONE == rxBuffer[RX_PACKET_STS_POS]) )
                {
                    status = TRANSFER_CMPLT;
                }
            }
        }
    }
    return (status);
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


/******************************************************************************
* Function Name: WaitOneUnit
****************************************************************************//**
*
* Waits for one unit before unblock code execution.
* Note If a timeout value is 0, this function does nothing and returns 0.
*
* \param timeout
* The pointer to a timeout value.
*
* \return
* Returns 0 if a timeout does not expire or the timeout mask.
*
*******************************************************************************/
static bool WaitOneUnit(uint32_t *timeout)
{
    uint32_t status = 0UL;

    /* If the timeout equal to 0. Ignore the timeout. */
    if (*timeout > 0UL)
    {
        Cy_SysLib_DelayUs(CY_SCB_WAIT_1_UNIT);
        --(*timeout);

        if (0UL == *timeout)
        {
            status = true;
        }
    }

    return (status);
}
/* [] END OF FILE */



