/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    Examples_LL/GPIO/GPIO_InfiniteLedToggling_Init/Src/stm32c0xx_it.c
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdio.h>
#include "memory.h"
#include "stm32c011xx.h"
#include "stm32c0xx_ll_i2c.h"
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

volatile uint16_t reg_address = 0; // The "register" the master wants
typedef enum {
  I2C_STATE_IDLE = 0,
  I2C_STATE_ADDR_1,
  I2C_STATE_ADDR_2,
  I2C_STATE_READ,
  I2C_STATE_WRITE,
  I2C_STATE_WAITING_FOR_RESET,
  I2C_STATE_RESET_TO_DISCOVERY_ON_STOP,
  I2C_STATE_WAITING_TO_RESET_SYSTEM
} I2C_State_t;

volatile I2C_State_t state = I2C_STATE_IDLE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32C0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32c0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles I2C1 interrupt (combined with EXTI 23).
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */
  if (LL_I2C_IsActiveFlag_ADDR(I2C1))
  {
    if ((uint8_t)LL_I2C_GetAddressMatchCode(I2C1) == memory_get_i2c_address() << 1)
    {
      DBG_PRINT("Got hit on real address (0x%02X)\n", LL_I2C_GetAddressMatchCode(I2C1) >> 1);
      // For every transaction on the bus this is the first thing which happens.
      // Both when reading and writing the first action is to write the 16-bit
      // register address.
      if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE)
      {
        state = I2C_STATE_ADDR_1;
      }
      else
      {
        state = I2C_STATE_READ;

        LL_I2C_EnableIT_TX(I2C1);
      }
    }
    else if (LL_I2C_GetAddressMatchCode(I2C1) == I2C_ADDR_RESTART_DISCOVERY) {
      DBG_PRINT("Read on restart discovery address, reset to discovery address on STOP\n");
      state = I2C_STATE_RESET_TO_DISCOVERY_ON_STOP;
      LL_I2C_EnableIT_TX(I2C1);
    }
    else if (LL_I2C_GetAddressMatchCode(I2C1) == I2C_ADDR_RESET) {
      DBG_PRINT("Read on reset address, reset to discovery address on STOP\n");
      state = I2C_STATE_WAITING_TO_RESET_SYSTEM;
      LL_I2C_EnableIT_TX(I2C1);
      memory_set_i2c_address(I2C_ADDR_DISCOVERY >> 1); // Reset the address to discovery
    }
    else
    {
      DBG_PRINT("Not my address: 0x%02X\n", LL_I2C_GetAddressMatchCode(I2C1) >> 1);
    }
    LL_I2C_ClearFlag_ADDR(I2C1);
  } else if (LL_I2C_IsActiveFlag_RXNE(I2C1)) {
    uint8_t data = LL_I2C_ReceiveData8(I2C1);

    switch (state) {
      case I2C_STATE_ADDR_1:
        reg_address = (data << 8);
        state = I2C_STATE_ADDR_2;
        break;
      case I2C_STATE_ADDR_2:
        reg_address |= data;
        // The default action after address is writing, if we read we will
        // do a new start and then continue to read.
        state = I2C_STATE_WRITE;
        break;
      case I2C_STATE_WRITE:
        memory_write(reg_address++, data);
        break;
      case I2C_STATE_RESET_TO_DISCOVERY_ON_STOP:
        // Do nothing here. This is when we are sent
        // bytes when using the reset address
        DBG_PRINT("Received data when in reset state: 0x%02X\n", data);
        break;
      default:
        DBG_PRINT("Unexpected RXNE state: %d\n", state);
        break;
    }
  } else if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {
    if (state == I2C_STATE_RESET_TO_DISCOVERY_ON_STOP) {
      // We are in the reset state, we should not send any real data and all slaves should
      // send the same data to avoid bus arbitration errors.
      DBG_PRINT("TXIS in reset state, ignoring\n");
      LL_I2C_TransmitData8(I2C1, 0x00);
    } else {
      uint8_t data = memory_read(reg_address++);
      LL_I2C_TransmitData8(I2C1, data);
    }
  }

  if (LL_I2C_IsActiveFlag_ARLO(I2C1)) {
    DBG_PRINT("Arbitration lost, jump to RESET address\n");
    state = I2C_STATE_WAITING_FOR_RESET;
    LL_I2C_ClearFlag_ARLO(I2C1);

    LL_I2C_DisableOwnAddress1(I2C1);
    LL_I2C_SetOwnAddress1(I2C1, I2C_ADDR_RESTART_DISCOVERY, LL_I2C_OWNADDRESS1_7BIT);
    LL_I2C_EnableOwnAddress1(I2C1);
  }


  /* Check NACK flag value in ISR register */
  if (LL_I2C_IsActiveFlag_NACK(I2C1))
  {
    /* End of Transfer */
    LL_I2C_ClearFlag_NACK(I2C1);
  }
  /* Check STOP flag value in ISR register */
  if (LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    /* Clear STOP flag value in ISR register */
    LL_I2C_ClearFlag_STOP(I2C1);

    /* Check TXE flag value in ISR register */
    if (!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
      /* Flush the TXDR register */
      LL_I2C_ClearFlag_TXE(I2C1);
    }

    if (state == I2C_STATE_RESET_TO_DISCOVERY_ON_STOP) {
      // We got a stop on the reset address, we should now reset the I2C address
      // to the discovery address.
      DBG_PRINT("Resetting I2C address to discovery address\n");
      state = I2C_STATE_IDLE;
      LL_I2C_DisableOwnAddress1(I2C1);
      LL_I2C_SetOwnAddress1(I2C1, I2C_ADDR_DISCOVERY, LL_I2C_OWNADDRESS1_7BIT);
      LL_I2C_EnableOwnAddress1(I2C1);
    } else if (I2C_STATE_WAITING_TO_RESET_SYSTEM == state) {
      // We got a stop on the reset address, we should now reset the system
      DBG_PRINT("Resetting system\n");
      NVIC_SystemReset();
    } else if (state != I2C_STATE_WAITING_FOR_RESET) {
      if (memory_check_and_reset_i2c_address_update()) {
        DBG_PRINT("Updating I2C address to: 0x%02X\n", memory_get_i2c_address());
        uint8_t new_addr = memory_get_i2c_address();
        LL_I2C_DisableOwnAddress1(I2C1);
        LL_I2C_SetOwnAddress1(I2C1, new_addr << 1, LL_I2C_OWNADDRESS1_7BIT);
        LL_I2C_EnableOwnAddress1(I2C1);
      }
    }

  }

  /* USER CODE END I2C1_IRQn 0 */
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
