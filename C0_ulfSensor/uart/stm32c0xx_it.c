/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Communication_Tx_IT_Init/Src/stm32c0xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32c0xx_it.h"
#include "stm32c0xx_ll_usart.h"

/* Private includes ----------------------------------------------------------*/

/* external data -------------------------------------------------------------*/
extern volatile uint8_t    sBuffer[32];
extern volatile uint8_t    sBufIndex;
extern volatile uint8_t    sBufChars;

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/* handle Non maskable interrupt
 */
void NMI_Handler(void)
{
}

/* handle Hard fault interrupt
 */
void HardFault_Handler(void)
{
  while (1);
}

/* handle System service call via SWI instruction
 */
void SVC_Handler(void)
{
}

/* handle Pending request for system service
 */
void PendSV_Handler(void)
{
}


/******************************************************************************/
/* STM32C0xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/
#define USART_ERRORS           (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF)
#define USART_CR1_TXEIE_MASK   (0x1UL << USART_CR1_TXEIE_TXFNFIE_Pos)

/* handle USART1 interrupt.
 */
void USART1_IRQHandler(void)
{
    /* Tx handling code */
    if (USART1->ISR & USART_ISR_TXE_TXFNF)
    {
        USART1->TDR = sBuffer[sBufIndex++];  /* next char; write clears the TXE flag */
        sBufChars--;

        /* if last character, disable the interrupt */
        //if (sBuffer[sBufIndex] == '\0')
        if (sBufChars <= 1)
            USART1->CR1 &= ~USART_CR1_TXEIE_MASK;
    }
    else // clear errors
    {
        USART1->ICR |= USART_ERRORS;
    }
}
