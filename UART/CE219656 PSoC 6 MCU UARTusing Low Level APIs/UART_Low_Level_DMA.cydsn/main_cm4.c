/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.20
*
* Description: This example demonstrates the UART transmit and receive 
*              operation using DMA.
*
* Related Document: CE219656.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer kit
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
*******************************************************************************/

#include <project.h>
#include <stdio.h>

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
void ConfigureTxDma(void);
void ConfigureRxDma(void);
void ConfigureDmaIntr(void);
void RxDmaComplete(void);
void TxDmaComplete(void);
void UartErrors(void);
void handle_error(void);

/*******************************************************************************
*            Constants
*******************************************************************************/
#define DMA_DESCR0       (0u)
#define DMA_DESCR1       (1u)
#define BUFFER_SIZE      (1u)
#define LED_ON           (0x00u)
#define LED_OFF          (!LED_ON)

/*******************************************************************************
*            Global variables
*******************************************************************************/
uint8 RxDmaUartBufferA[BUFFER_SIZE];
uint8 RxDmaUartBufferB[BUFFER_SIZE];
uint32_t activeDescr; /* flag to control which descriptor to use */
uint8 rx_dma_error;   /* RxDma error flag */
uint8 tx_dma_error;   /* TxDma error flag */
uint8 uart_error;     /* UART error flag */
uint8 rx_dma_done;    /* RxDma done flag */

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
*
* Summary:
* This function processes unrecoverable errors such as UART component 
* initialization error or DMA error etc. In case of such error the system will
* Turn on RED_LED_ERROR and stay in the infinite loop of this function.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void handle_error(void)
{    
     /* Disable all interrupts */
    __disable_irq();
	
    /* Turn on error LED */
    Cy_GPIO_Write(RED_LED_ERROR_0_PORT, RED_LED_ERROR_0_NUM, LED_ON);
    while(1u) {}
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
* The main function performs the following actions:
*  1. Configures RX and TX DMAs to handle UART RX+TX direction.
*  2. Configures UART component.
*  3. If initialization of UART component fails then Turn on RED_LED_ERROR.
*  4. Sends text header to the UART serial terminal.
*  5. Waits in an infinite loop (for DMA or UART error interrupt)
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main()
{
    cy_en_scb_uart_status_t initstatus;
    
    /* Configure DMA Rx and Tx channels for operation */
    ConfigureRxDma();
    ConfigureTxDma();

    /* Initialize and enable interrupt from UART. The UART interrupt sources
    *  are enabled in the Component GUI */
    Cy_SysInt_Init(&UART_INT_cfg, &UartErrors);
    NVIC_EnableIRQ(UART_INT_cfg.intrSrc);
    
    /* Turn off the RED_LED_ERROR */
    Cy_GPIO_Write(RED_LED_ERROR_0_PORT, RED_LED_ERROR_0_NUM, LED_OFF);
	
    /* Start UART operation */
    initstatus = Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    if(initstatus!=CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    Cy_SCB_UART_Enable(UART_HW);  

    /* Transmit header to the terminal */
    Cy_SCB_UART_PutString(UART_HW, "\r\n**********************************************************************************\r\n");
    Cy_SCB_UART_PutString(UART_HW, "This is UART example, which uses DMA to demonstrate UART operation\r\n");
    Cy_SCB_UART_PutString(UART_HW, "If you are able to read this text the terminal connection is configured correctly.\r\n");
    Cy_SCB_UART_PutString(UART_HW, "Start transmitting the characters to see an echo in the terminal.\r\n");
    Cy_SCB_UART_PutString(UART_HW, "\r\n");
	
	/* Initialize flags */
    rx_dma_error = 0;
    tx_dma_error = 0;
    uart_error = 0;
    rx_dma_done = 0;
    
     /* Enable global interrupts */
    __enable_irq();
	
    /* UART communication is managed by interrupts and DMA. */
    for(;;)
    {
		/* Indicate status if RxDma error or TxDma error or UART error occurs */
		if((rx_dma_error==1) | (tx_dma_error==1) | (uart_error==1))
		{
			handle_error();
		}
		
		/* Handle RxDma complete */
		if(rx_dma_done==1)
		{
			rx_dma_done = 0;
			
			/* Ping Pong between RxDmaUartBufferA and RxDmaUartBufferB */
			/* Ping Pong buffers give firmware time to pull the data out of one or the other buffer */
			if (DMA_DESCR0 == activeDescr)
			{
				/* Set source to be RX Buffer A */
				Cy_DMA_Descriptor_SetSrcAddress(&TxDma_Descriptor_1, (uint32_t *) RxDmaUartBufferA);
			}
			else
			{
				/* Set source to be RX Buffer B */
				Cy_DMA_Descriptor_SetSrcAddress(&TxDma_Descriptor_1, (uint32_t *) RxDmaUartBufferB);
			}
		
			Cy_DMA_Channel_SetDescriptor(TxDma_HW, TxDma_DW_CHANNEL, &TxDma_Descriptor_1);
			Cy_DMA_Channel_Enable(TxDma_HW, TxDma_DW_CHANNEL);
		
			/* User alternate active descriptor */
			activeDescr ^= DMA_DESCR1;			
		} 
    }
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
void ConfigureRxDma(void)
{
	cy_en_dma_status_t dma_init_status;
	cy_stc_dma_channel_config_t channelConfig;	

    /* Initialize descriptor 1 */
    dma_init_status = Cy_DMA_Descriptor_Init(&RxDma_Descriptor_1, &RxDma_Descriptor_1_config);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }
	
    /* Initialize descriptor 2 */
    dma_init_status = Cy_DMA_Descriptor_Init(&RxDma_Descriptor_2, &RxDma_Descriptor_2_config);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }
    
    /* Initialize the DMA channel */
    channelConfig.descriptor = &RxDma_Descriptor_1;
    channelConfig.preemptable = RxDma_PREEMPTABLE;
    channelConfig.priority    = RxDma_PRIORITY;
    channelConfig.enable      = 0u;
    dma_init_status = Cy_DMA_Channel_Init(RxDma_HW, RxDma_DW_CHANNEL, &channelConfig);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }

    /* Set source and destination address for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&RxDma_Descriptor_1, (uint32_t *) &UART_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&RxDma_Descriptor_1, (uint32_t *) RxDmaUartBufferA);
    activeDescr = DMA_DESCR0;

    /* Set source and destination address for descriptor 2 */
    Cy_DMA_Descriptor_SetSrcAddress(&RxDma_Descriptor_2, (uint32_t *) &UART_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&RxDma_Descriptor_2, (uint32_t *) RxDmaUartBufferB);
    
    /* Initialize and enable interrupt from RxDma */
    Cy_SysInt_Init  (&RX_DMA_INT_cfg, &RxDmaComplete);
    NVIC_EnableIRQ(RX_DMA_INT_cfg.intrSrc);

    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(RxDma_HW, RxDma_DW_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable channel and DMA block to start descriptor execution process */
    Cy_DMA_Channel_Enable(RxDma_HW, RxDma_DW_CHANNEL);
    Cy_DMA_Enable(RxDma_HW);
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
void ConfigureTxDma(void)
{
    cy_en_dma_status_t dma_init_status;
    cy_stc_dma_channel_config_t channelConfig;

    /* Init descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&TxDma_Descriptor_1, &TxDma_Descriptor_1_config);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }
    
    /* Initialize the DMA channel */
    channelConfig.descriptor = &TxDma_Descriptor_1;
    channelConfig.preemptable = TxDma_PREEMPTABLE;
    channelConfig.priority    = TxDma_PRIORITY;
    channelConfig.enable      = 0u;
    dma_init_status = Cy_DMA_Channel_Init(TxDma_HW, TxDma_DW_CHANNEL, &channelConfig);
	if(dma_init_status!=CY_DMA_SUCCESS)
    {
        handle_error();
    }    

    /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&TxDma_Descriptor_1, (uint32_t *) RxDmaUartBufferA);
    Cy_DMA_Descriptor_SetDstAddress(&TxDma_Descriptor_1, (uint32_t *) &UART_HW->TX_FIFO_WR);

    /* Set next descriptor to NULL to stop the chain execution after descriptor 1
    *  is completed.
    */
    Cy_DMA_Descriptor_SetNextDescriptor(Cy_DMA_Channel_GetCurrentDescriptor(TxDma_HW, TxDma_DW_CHANNEL), NULL);

    /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init  (&TX_DMA_INT_cfg, &TxDmaComplete);
    NVIC_EnableIRQ(TX_DMA_INT_cfg.intrSrc);

    /* Enable DMA interrupt source */
    Cy_DMA_Channel_SetInterruptMask(TxDma_HW, TxDma_DW_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable Data Write block but keep channel disabled to not trigger
    *  descriptor execution because TX FIFO is empty and SCB keeps active level
    *  for DMA.
    */
    Cy_DMA_Enable(TxDma_HW);
}


/*******************************************************************************
* Function Name: RxDmaComplete
********************************************************************************
*
* Summary:
*  Handles Rx Dma descriptor completion interrupt source: triggers Tx Dma to
*  transfer back data received by the Rx Dma descriptor.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void RxDmaComplete(void)
{
    Cy_DMA_Channel_ClearInterrupt(RxDma_HW, RxDma_DW_CHANNEL);

    /* Check interrupt cause to capture errors. */
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(RxDma_HW, RxDma_DW_CHANNEL))
    {
		rx_dma_done = 1;
	}
	else
	{
		/* DMA error occurred while RX operations */
		rx_dma_error = 1;
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
void TxDmaComplete(void)
{
    /* Check interrupt cause to capture errors.
    *  Note that next descriptor is NULL to stop descriptor execution */
    if ((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus(TxDma_HW, TxDma_DW_CHANNEL)) &&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(TxDma_HW, TxDma_DW_CHANNEL)))
    {
        /* DMA error occurred while TX operations */
        tx_dma_error = 1;
    }
	else
	{
		Cy_DMA_Channel_ClearInterrupt(TxDma_HW, TxDma_DW_CHANNEL);
	}
}


/*******************************************************************************
* Function Name: UartErrors
********************************************************************************
*
* Summary:
* Handles UART Rx underflow and overflow conditions. This conditions must never
* occur.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void UartErrors(void)
{
    uint32 intrSrcRx;
    uint32 intrSrcTx;

    /* Get RX interrupt sources */
    intrSrcRx = Cy_SCB_UART_GetRxFifoStatus(UART_HW);
    Cy_SCB_UART_ClearRxFifoStatus(UART_HW, intrSrcRx);

    /* Get TX interrupt sources */
    intrSrcTx = Cy_SCB_UART_GetTxFifoStatus(UART_HW);
    Cy_SCB_UART_ClearTxFifoStatus(UART_HW, intrSrcTx);

    /* RX overflow or RX underflow or RX overflow occured */
    uart_error = 1;
}

/* [] END OF FILE */

