/*!
 * \file      smtc_hal_spi.c
 *
 * \brief     SPI Hardware Abstraction Layer implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "em_gpio.h"
#include "smtc_hal.h"
#include "smtc_hal_gpio.h"
#include "modem_pinout.h"
#include "spidrv.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */


/*!
 *  @brief SPI structure
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

SPIDRV_HandleData_t handleData;
SPIDRV_Handle_t handle = &handleData;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void hal_spi_init( const uint32_t id, const hal_gpio_pin_names_t mosi, const hal_gpio_pin_names_t miso,
                   const hal_gpio_pin_names_t sclk )
{

  // Configure TX pin as an output
    GPIO_PinModeSet(hal_get_gpio_port(mosi), hal_get_gpio_pin_num(mosi), gpioModePushPull, 0);

    // Configure RX pin as an input
    GPIO_PinModeSet(hal_get_gpio_port(miso), hal_get_gpio_pin_num(miso), gpioModeInput, 0);

    // Configure CLK pin as an output low (CPOL = 0)
    GPIO_PinModeSet(hal_get_gpio_port(sclk), hal_get_gpio_pin_num(sclk), gpioModePushPull, 0);

    GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1);//Murata:NSS

     /*
      * Route USART0 RX, TX, and CLK to the specified pins.  Note that CS is
      * not controlled by USART0 so there is no write to the corresponding
      * USARTROUTE register to do this.
      */
    if(id == 0){
        // Default asynchronous initializer (main mode, 1 Mbps, 8-bit data)
        USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

        init.msbf = true;   // MSB first transmission for SPI compatibility
        CMU_ClockEnable(cmuClock_USART0, true);
        GPIO->USARTROUTE[0].TXROUTE = (hal_get_gpio_port(mosi) << _GPIO_USART_TXROUTE_PORT_SHIFT)
             | (hal_get_gpio_pin_num(mosi) << _GPIO_USART_TXROUTE_PIN_SHIFT);
        GPIO->USARTROUTE[0].RXROUTE = (hal_get_gpio_port(miso) << _GPIO_USART_RXROUTE_PORT_SHIFT)
             | (hal_get_gpio_pin_num(miso) << _GPIO_USART_RXROUTE_PIN_SHIFT);
        GPIO->USARTROUTE[0].CLKROUTE = (hal_get_gpio_port(sclk) << _GPIO_USART_CLKROUTE_PORT_SHIFT)
             | (hal_get_gpio_pin_num(sclk) << _GPIO_USART_CLKROUTE_PIN_SHIFT);

        // Enable USART interface pins
        GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN |    // MISO
            GPIO_USART_ROUTEEN_TXPEN |    // MOSI
            GPIO_USART_ROUTEEN_CLKPEN;

        // Configure and enable USART0
        USART_InitSync(USART0, &init);
    }
    if((id == 1) || (id == 2)){
        uint8_t eusart_id;
        if(id == 1){
            CMU_ClockEnable(cmuClock_EUSART0, true);
            eusart_id = 0;
        }
        else{
            CMU_ClockEnable(cmuClock_EUSART1, true);
            eusart_id = 1;
        }

        // SPI advanced configuration (part of the initializer)
        EUSART_SpiAdvancedInit_TypeDef adv = EUSART_SPI_ADVANCED_INIT_DEFAULT;
        EUSART_SpiInit_TypeDef init = EUSART_SPI_MASTER_INIT_DEFAULT_HF;

        adv.msbFirst = true;        // SPI standard MSB first
        // Default asynchronous initializer (main/master mode and 8-bit data)
        init.bitRate = 1000000;        // 1 MHz shift clock
        init.advancedSettings = &adv;  // Advanced settings structure

        /*
         * Route EUSART1 MOSI, MISO, and SCLK to the specified pins.  CS is
         * not controlled by EUSART1 so there is no write to the corresponding
         * EUSARTROUTE register to do this.
         */



        GPIO->EUSARTROUTE[eusart_id].TXROUTE = (hal_get_gpio_port(mosi) << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
                       | (hal_get_gpio_pin_num(mosi) << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
        GPIO->EUSARTROUTE[eusart_id].RXROUTE = (hal_get_gpio_port(miso) << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
                       | (hal_get_gpio_pin_num(miso) << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
        GPIO->EUSARTROUTE[eusart_id].SCLKROUTE = (hal_get_gpio_port(sclk) << _GPIO_EUSART_SCLKROUTE_PORT_SHIFT)
                       | (hal_get_gpio_pin_num(sclk) << _GPIO_EUSART_SCLKROUTE_PIN_SHIFT);

        // Enable EUSART interface pins
        GPIO->EUSARTROUTE[eusart_id].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN |    // MISO
            GPIO_EUSART_ROUTEEN_TXPEN |    // MOSI
            GPIO_EUSART_ROUTEEN_SCLKPEN;


        if(id == 1){
            // Configure and enable EUSART0
            EUSART_SpiInit(EUSART0, &init);
        }
        else{
            // Configure and enable EUSART1
            EUSART_SpiInit(EUSART1, &init);
        }
    }
}

void hal_spi_deinit( const uint32_t id )
{
  if(id == 0){
      USART_Reset(USART0);
  }
  if(id == 1){
      EUSART_Reset(EUSART0);
  }
  if(id == 2){
      EUSART_Reset(EUSART1);
  }
}

uint16_t hal_spi_in_out( const uint32_t id, const uint16_t out_data )
{
  uint8_t rxValue = 0;


  if(id == 0){
      rxValue = USART_SpiTransfer(USART0, out_data);
  }
  if(id == 1){
      rxValue = EUSART_Spi_TxRx(EUSART0, out_data);
  }
  if(id == 2){
      rxValue = EUSART_Spi_TxRx(EUSART1, out_data);
  }

  return rxValue;
}



/* --- EOF ------------------------------------------------------------------ */
