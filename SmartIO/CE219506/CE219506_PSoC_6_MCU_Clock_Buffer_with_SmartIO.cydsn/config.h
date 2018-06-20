/*******************************************************************************
* File Name: config.h
* Version 1.10
*
* Description:
*  This file provides constants and parameter values for the MCWDT
*  component.
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

#include "cyfitter.h"
#include "mcwdt/cy_mcwdt.h"

/******************************************************************************
*           MCWDT configuration macros
*******************************************************************************/
#define MCWDT_C0_CLEAR_ON_MATCH  (1U)
#define MCWDT_C1_CLEAR_ON_MATCH  (0U)
#define MCWDT_CASCADE_C0C1       (0U)
#define MCWDT_CASCADE_C1C2       (0U)
#define MCWDT_C0_MATCH           (32768U)
#define MCWDT_C0_MODE            (1U)
#define MCWDT_C1_MATCH           (32768U)
#define MCWDT_C1_MODE            (0U)
#define MCWDT_C2_PERIOD          (16U)
#define MCWDT_C2_MODE            (0U)

/*******************************************************************************
* MCWDT configuration structure
*******************************************************************************/

const cy_stc_mcwdt_config_t MCWDT_config =
{
    .c0Match     = MCWDT_C0_MATCH,
    .c1Match     = MCWDT_C1_MATCH,
    .c0Mode      = MCWDT_C0_MODE,
    .c1Mode      = MCWDT_C1_MODE,
    .c2ToggleBit = MCWDT_C2_PERIOD,
    .c2Mode      = MCWDT_C2_MODE,
    .c0ClearOnMatch = (bool)MCWDT_C0_CLEAR_ON_MATCH,
    .c1ClearOnMatch = (bool)MCWDT_C1_CLEAR_ON_MATCH,
    .c0c1Cascade = (bool)MCWDT_CASCADE_C0C1,
    .c1c2Cascade = (bool)MCWDT_CASCADE_C1C2
};
