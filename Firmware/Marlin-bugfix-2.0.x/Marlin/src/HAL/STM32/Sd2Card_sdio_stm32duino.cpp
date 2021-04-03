/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../inc/MarlinConfig.h"

#if ENABLED(SDIO_SUPPORT)

#include <stdint.h>
#include <stdbool.h>

#if NONE(STM32F103xE, STM32F103xG, STM32F4xx, STM32F7xx)
  #error "ERROR - Only STM32F103xE, STM32F103xG, STM32F4xx or STM32F7xx CPUs supported"
#endif

#if HAS_SD_HOST_DRIVE

  // use USB drivers

  extern "C" { int8_t SD_MSC_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
               int8_t SD_MSC_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
               extern SD_HandleTypeDef hsd;
  }

  bool SDIO_Init() {
    return hsd.State == HAL_SD_STATE_READY;  // return pass/fail status
  }

  bool SDIO_ReadBlock(uint32_t block, uint8_t *src) {
    int8_t status = SD_MSC_Read(0, (uint8_t*)src, block, 1); // read one 512 byte block
    return (bool) status;
  }

  bool SDIO_WriteBlock(uint32_t block, const uint8_t *src) {
    int8_t status = SD_MSC_Write(0, (uint8_t*)src, block, 1); // write one 512 byte block
    return (bool) status;
  }

#else // !USBD_USE_CDC_COMPOSITE

  // use local drivers
  #if defined(STM32F103xE) || defined(STM32F103xG)
    #include <stm32f1xx_hal_rcc_ex.h>
    #include <stm32f1xx_hal_sd.h>
  #elif defined(STM32F4xx)
    #include <stm32f4xx_hal_rcc.h>
    #include <stm32f4xx_hal_dma.h>
    #include <stm32f4xx_hal_gpio.h>
    #include <stm32f4xx_hal_sd.h>
  #elif defined(STM32F7xx)
    #include <stm32f7xx_hal_rcc.h>
    #include <stm32f7xx_hal_dma.h>
    #include <stm32f7xx_hal_gpio.h>
    #include <stm32f7xx_hal_sd.h>
  #else
    #error "ERROR - Only STM32F103xE, STM32F103xG, STM32F4xx or STM32F7xx CPUs supported"
  #endif

  SD_HandleTypeDef  hsd;  // create SDIO structure
  DMA_HandleTypeDef dmaRx, dmaTx;

  /*
    SDIO_INIT_CLK_DIV is 118
    SDIO clock frequency is 48MHz / (TRANSFER_CLOCK_DIV + 2)
    SDIO init clock frequency should not exceed 400KHz = 48MHz / (118 + 2)

    Default TRANSFER_CLOCK_DIV is 2 (118 / 40)
    Default SDIO clock frequency is 48MHz / (2 + 2) = 12 MHz
    This might be too fast for stable SDIO operations

    MKS Robin board seems to have stable SDIO with BusWide 1bit and ClockDiv 8 i.e. 4.8MHz SDIO clock frequency
    Additional testing is required as there are clearly some 4bit initialization problems
  */

  #ifndef USBD_OK
    #define USBD_OK 0
  #endif

  // Target Clock, configurable. Default is 18MHz, from STM32F1
  #ifndef SDIO_CLOCK
    #define SDIO_CLOCK                         18000000       /* 18 MHz */
  #endif

  // SDIO retries, configurable. Default is 3, from STM32F1
  #ifndef SDIO_READ_RETRIES
    #define SDIO_READ_RETRIES                  3
  #endif

  #ifndef SDIO_TIMEOUT_MS
    #define SDIO_TIMEOUT_MS                    10  // 10 milliseconds
  #endif

  // SDIO Max Clock (naming from STM Manual, don't change)
  #define SDIOCLK 48000000

  static uint32_t clock_to_divider(uint32_t clk) {
    // limit the SDIO master clock to 8/3 of PCLK2. See STM32 Manuals
    // Also limited to no more than 48Mhz (SDIOCLK).
    const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    clk = min(clk, (uint32_t)(pclk2 * 8 / 3));
    clk = min(clk, (uint32_t)SDIOCLK);
    // Round up divider, so we don't run the card over the speed supported,
    // and subtract by 2, because STM32 will add 2, as written in the manual:
    // SDIO_CK frequency = SDIOCLK / [CLKDIV + 2]
    return pclk2 / clk + (pclk2 % clk != 0) - 2;
  }

  extern "C" void SDMMC1_IRQHandler(void) {
    HAL_SD_IRQHandler(&hsd);
  }

  extern "C" void DMA2_Stream6_IRQHandler(void) {
    HAL_DMA_IRQHandler(hsd.hdmatx);
  }

  extern "C" void DMA2_Stream3_IRQHandler(void) {
    HAL_DMA_IRQHandler(hsd.hdmarx);
  }

  void HAL_InitDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef *inst, uint32_t chan, IRQn_Type irq, uint32_t dir, uint32_t minc) {
    hdma.Instance                 = inst;
    hdma.Init.Channel             = chan;
    hdma.Init.Direction           = dir;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma.Init.MemInc              = minc;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma.Init.Mode                = DMA_PFCTRL;
    hdma.Init.Priority            = DMA_PRIORITY_LOW;
    hdma.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma.Init.MemBurst            = DMA_MBURST_INC4;
    hdma.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_DeInit(&hdma);
    HAL_DMA_Init(&hdma);
    HAL_NVIC_EnableIRQ(irq);
  }

  bool SDIO_Init() {

    uint8_t SD_Error;
    __HAL_RCC_SDIO_FORCE_RESET();
    delay(2);
    __HAL_RCC_SDIO_RELEASE_RESET();
    delay(2);
    __HAL_RCC_SDIO_CLK_ENABLE();

    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = clock_to_divider(SDIO_CLOCK);

    pinmap_pinout(PC_8, PinMap_SD);
    pinmap_pinout(PC_12, PinMap_SD);
    pinmap_pinout(PD_2, PinMap_SD);
    #if PINS_EXIST(SDIO_D1, SDIO_D2, SDIO_D3) // go to 4 bit wide mode if pins are defined
      pinmap_pinout(PC_9, PinMap_SD);
      pinmap_pinout(PC_10, PinMap_SD);
      pinmap_pinout(PC_11, PinMap_SD);
    #endif

    __HAL_RCC_DMA2_FORCE_RESET();
    delay(2);
    __HAL_RCC_DMA2_RELEASE_RESET();
    delay(2);
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_InitDmaStream(dmaRx, DMA2_Stream3, DMA_CHANNEL_4, DMA2_Stream3_IRQn, DMA_PERIPH_TO_MEMORY, DMA_MINC_ENABLE);
    __HAL_LINKDMA(&hsd, hdmarx, dmaRx);
    HAL_InitDmaStream(dmaTx, DMA2_Stream6, DMA_CHANNEL_4, DMA2_Stream6_IRQn, DMA_MEMORY_TO_PERIPH, DMA_MINC_ENABLE);
    __HAL_LINKDMA(&hsd, hdmatx, dmaTx);

    SD_Error = HAL_SD_Init(&hsd);
    if(SD_Error != HAL_OK) return false;

    #if PINS_EXIST(SDIO_D1, SDIO_D2, SDIO_D3) // go to 4 bit wide mode if pins are defined
      SD_Error = HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
      if(SD_Error != HAL_OK) return false;
    #endif

    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    return true;
  }

  bool SDIO_ReadBlock(uint32_t block, uint8_t *dst) {

    if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)dst, block, 1) != HAL_OK) {
      return false;
    }

    uint32_t start = millis() + SDIO_TIMEOUT_MS;
    while(HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
      if (millis() > start) {
        return false;
      }
    }

    return true;
  }

  bool SDIO_WriteBlock(uint32_t block, const uint8_t *src) {

    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)src, block, 1) != HAL_OK) {
      return false;
    }

    uint32_t start = millis() + SDIO_TIMEOUT_MS;
    while(HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
      if (millis() > start) {
        return false;
      }
    }

    return true;
  }

#endif // !USBD_USE_CDC_COMPOSITE
#endif // SDIO_SUPPORT
#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
