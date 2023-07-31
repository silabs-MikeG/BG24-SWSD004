/*!
 * @file      smtc_hal_uart.c
 *
 * @brief     Board specific package UART API implementation.
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
#include "smtc_hal_gpio_pin_names.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_uart.h"
#include "smtc_hal_mcu.h"
#include "em_usart.h"
#include "em_eusart.h"
#include "uartdrv.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define SL_UARTDRV_USART_INST_BAUDRATE        115200

#define SL_UARTDRV_USART_INST_PARITY          usartNoParity

#define SL_UARTDRV_USART_INST_STOP_BITS       usartStopbits1

#define SL_UARTDRV_USART_INST_FLOW_CONTROL_TYPE uartdrvFlowControlNone

#define SL_UARTDRV_USART_INST_OVERSAMPLING      usartOVS4

#define SL_UARTDRV_USART_INST_MVDIS             false

#define SL_UARTDRV_USART_INST_RX_BUFFER_SIZE  2

#define SL_UARTDRV_USART_INST_TX_BUFFER_SIZE 2



#define SL_UARTDRV_EUSART_INST_LF_MODE         false



#define SL_UARTDRV_EUSART_INST_OVERSAMPLING      eusartOVS16

// <o SL_UARTDRV_EUSART_INST_MVDIS> Majority vote disable for 16x, 8x and 6x oversampling modes
// <eusartMajorityVoteEnable=> False
// <eusartMajorityVoteDisable=> True
// <i> Default: eusartMajorityVoteEnable
#define SL_UARTDRV_EUSART_INST_MVDIS             eusartMajorityVoteEnable

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

UARTDRV_HandleData_t sl_uartdrv_usart_vcom_handle_data;
UARTDRV_Handle_t sl_uartdrv_usart_vcom_handle = &sl_uartdrv_usart_vcom_handle_data;

UARTDRV_HandleData_t sl_uartdrv_eusart_vcom_handle_data;
UARTDRV_Handle_t sl_uartdrv_eusart_vcom_handle = &sl_uartdrv_eusart_vcom_handle_data;



static UARTDRV_Handle_t sli_uartdrv_default_handle = NULL;


/* Define RX and TX buffer queues */
DEFINE_BUF_QUEUE(SL_UARTDRV_USART_INST_RX_BUFFER_SIZE, sl_uartdrv_usart_inst_rx_buffer);
DEFINE_BUF_QUEUE(SL_UARTDRV_USART_INST_TX_BUFFER_SIZE, sl_uartdrv_usart_inst_tx_buffer);


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

//void USART1_IRQHandler( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_uart_init( const uint32_t id, const hal_gpio_pin_names_t uart_tx, const hal_gpio_pin_names_t uart_rx )
{


  // Configure the USART TX pin to the board controller as an output
  GPIO_PinModeSet(hal_get_gpio_port(uart_tx), hal_get_gpio_pin_num(uart_tx), gpioModePushPull, 0);

  // Configure the USART RX pin to the board controller as an input
  GPIO_PinModeSet(hal_get_gpio_port(uart_rx), hal_get_gpio_pin_num(uart_rx), gpioModeInput, 0);

  if(id == 0){
      CMU_ClockEnable(cmuClock_USART0, true);
      // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
      USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
      init.baudrate = 115200;

      // Route USART0 TX and RX to the board controller TX and RX pins
      GPIO->USARTROUTE[0].TXROUTE = (hal_get_gpio_port(uart_tx) << _GPIO_USART_TXROUTE_PORT_SHIFT)
                              | (hal_get_gpio_pin_num(uart_tx) << _GPIO_USART_TXROUTE_PIN_SHIFT);
      GPIO->USARTROUTE[0].RXROUTE = (hal_get_gpio_port(uart_rx) << _GPIO_USART_RXROUTE_PORT_SHIFT)
                              | (hal_get_gpio_pin_num(uart_rx) << _GPIO_USART_RXROUTE_PIN_SHIFT);

      // Enable RX and TX signals now that they have been routed
      GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_TXPEN;

      // Configure and enable USART0
      USART_InitAsync(USART0, &init);
  }
  if((id == 1) || (id == 2)){
      uint8_t eusart_id;
      // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
      EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
      if(id==1){
          CMU_ClockEnable(cmuClock_EUSART0, true);
          eusart_id = 0;
      }
      else{
          CMU_ClockEnable(cmuClock_EUSART1, true);
          eusart_id = 1;
      }



      // Route EUSART1 TX and RX to the board controller TX and RX pins
      GPIO->EUSARTROUTE[eusart_id].TXROUTE = (hal_get_gpio_port(uart_tx) << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
                    | (hal_get_gpio_pin_num(uart_tx) << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
      GPIO->EUSARTROUTE[eusart_id].RXROUTE = (hal_get_gpio_port(uart_rx) << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
                    | (hal_get_gpio_pin_num(uart_rx) << _GPIO_EUSART_RXROUTE_PIN_SHIFT);

      // Enable RX and TX signals now that they have been routed
      GPIO->EUSARTROUTE[eusart_id].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN;


      if(id==1){
          // Configure and enable EUSART1 for high-frequency (EM0/1) operation
          EUSART_UartInitHf(EUSART0, &init);
      }
      else{
          // Configure and enable EUSART1 for high-frequency (EM0/1) operation
          EUSART_UartInitHf(EUSART1, &init);
      }


  }
}

//void hal_uart_init( const uint32_t id, const hal_gpio_pin_names_t uart_tx, const hal_gpio_pin_names_t uart_rx )
//{
//
//    if((id == 1) || (id == 2))
//    {
//      /* Create uartdrv initialization structs */
//      UARTDRV_InitEuart_t sl_uartdrv_eusart_init_vcom;
//        if(id == 1)
//          sl_uartdrv_eusart_init_vcom.port = HAL_USART1;
//        else if(id == 2)
//          sl_uartdrv_eusart_init_vcom.port = HAL_USART2;
//
//
//        sl_uartdrv_eusart_init_vcom.useLowFrequencyMode = SL_UARTDRV_EUSART_INST_LF_MODE;
//        sl_uartdrv_eusart_init_vcom.baudRate = SL_UARTDRV_USART_INST_BAUDRATE;
//        sl_uartdrv_eusart_init_vcom.txPort = hal_get_gpio_port(uart_tx);
//        sl_uartdrv_eusart_init_vcom.rxPort = hal_get_gpio_port(uart_rx);
//        sl_uartdrv_eusart_init_vcom.txPin = hal_get_gpio_pin_num(uart_tx);
//        sl_uartdrv_eusart_init_vcom.rxPin = hal_get_gpio_pin_num(uart_rx);
//        sl_uartdrv_eusart_init_vcom.stopBits = SL_UARTDRV_USART_INST_STOP_BITS;
//        sl_uartdrv_eusart_init_vcom.parity = SL_UARTDRV_USART_INST_PARITY;
//        sl_uartdrv_eusart_init_vcom.oversampling = SL_UARTDRV_EUSART_INST_OVERSAMPLING;
//        sl_uartdrv_eusart_init_vcom.mvdis = SL_UARTDRV_EUSART_INST_MVDIS;
//        sl_uartdrv_eusart_init_vcom.fcType = SL_UARTDRV_USART_INST_FLOW_CONTROL_TYPE;
//        sl_uartdrv_eusart_init_vcom.ctsPort = 0;
//        sl_uartdrv_eusart_init_vcom.ctsPin = 0;
//        sl_uartdrv_eusart_init_vcom.rtsPort = 0;
//        sl_uartdrv_eusart_init_vcom.rtsPin = 0;
//        sl_uartdrv_eusart_init_vcom.rxQueue = (UARTDRV_Buffer_FifoQueue_t *)&sl_uartdrv_usart_inst_rx_buffer;
//        sl_uartdrv_eusart_init_vcom.txQueue = (UARTDRV_Buffer_FifoQueue_t *)&sl_uartdrv_usart_inst_tx_buffer;
//
//        if((id == 1)){
//        sl_uartdrv_eusart_init_vcom.uartNum = 0;
//        }
//        else if(id == 2){
//            sl_uartdrv_eusart_init_vcom.uartNum = 1;
//        }
//
//        UARTDRV_InitEuart(sl_uartdrv_eusart_vcom_handle, &sl_uartdrv_eusart_init_vcom);
//        sli_uartdrv_default_handle = sl_uartdrv_eusart_vcom_handle;
//    }
//  else if(id == 0)
//    {
//      /* Create uartdrv initialization structs */
//      UARTDRV_InitUart_t sl_uartdrv_usart_init_vcom;
//      sl_uartdrv_usart_init_vcom.port = HAL_USART0;
//      sl_uartdrv_usart_init_vcom.baudRate = SL_UARTDRV_USART_INST_BAUDRATE;
//      sl_uartdrv_usart_init_vcom.txPort = hal_get_gpio_port(uart_tx);;
//      sl_uartdrv_usart_init_vcom.rxPort = hal_get_gpio_port(uart_rx);;
//      sl_uartdrv_usart_init_vcom.txPin = hal_get_gpio_pin_num(uart_tx);;
//      sl_uartdrv_usart_init_vcom.rxPin = hal_get_gpio_pin_num(uart_rx);;
//      sl_uartdrv_usart_init_vcom.uartNum = 0;
//      sl_uartdrv_usart_init_vcom.stopBits = SL_UARTDRV_USART_INST_STOP_BITS;
//      sl_uartdrv_usart_init_vcom.parity = SL_UARTDRV_USART_INST_PARITY;
//      sl_uartdrv_usart_init_vcom.oversampling = SL_UARTDRV_USART_INST_OVERSAMPLING;
//      sl_uartdrv_usart_init_vcom.fcType = SL_UARTDRV_USART_INST_FLOW_CONTROL_TYPE;
//      sl_uartdrv_usart_init_vcom.ctsPort = 0;
//      sl_uartdrv_usart_init_vcom.rtsPort = 0;
//      sl_uartdrv_usart_init_vcom.ctsPin = 0;
//      sl_uartdrv_usart_init_vcom.rtsPin = 0;
//      sl_uartdrv_usart_init_vcom.rxQueue = (UARTDRV_Buffer_FifoQueue_t *)&sl_uartdrv_usart_inst_rx_buffer;
//      sl_uartdrv_usart_init_vcom.txQueue = (UARTDRV_Buffer_FifoQueue_t *)&sl_uartdrv_usart_inst_tx_buffer;
//
//      UARTDRV_InitUart(sl_uartdrv_usart_vcom_handle, &sl_uartdrv_usart_init_vcom);
//      sli_uartdrv_default_handle = sl_uartdrv_usart_vcom_handle;
//
//    }
//
//
//}

void hal_uart_deinit( const uint32_t id )
{
  //  UARTDRV_DeInit (sli_uartdrv_default_handle);
}

void hal_uart_tx( const uint32_t id, uint8_t* buff, uint16_t len )
{


  for(uint32_t j = 0; j < len; j++){
      switch(id){
        case 0:
          USART_Tx(USART0, buff[j]);
          break;
        case 1:
          EUSART_Tx(EUSART0, buff[j]);
          break;
        case 2:
          EUSART_Tx(EUSART1, buff[j]);
          break;
        default:
          break;
      }

  }

  //  UARTDRV_TransmitB ( sli_uartdrv_default_handle, buff,len);
}

void hal_uart_rx( const uint32_t id, uint8_t* rx_buffer, uint16_t len )
{
  for(uint32_t j = 0; j < len; j++){
      switch(id){
        case 0:
          rx_buffer[j] = USART_Rx(USART0);
          break;
        case 1:
          rx_buffer[j] = EUSART_Rx(EUSART0);
          break;
        case 2:
          rx_buffer[j] = EUSART_Rx(EUSART1);
          break;
        default:
          break;
      }

  }
  //  UARTDRV_ReceiveB ( sli_uartdrv_default_handle, rx_buffer, len);
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

///**
// * @brief  This function handles USART1 interrupt request.
// */
//void USART1_IRQHandler( void ) { HAL_UART_IRQHandler( &hal_uart[0].handle ); }
//
///**
// * @brief  Rx Transfer completed callback
// * @param  UartHandle: UART handle
// * @note   This example shows a simple way to report end of DMA Rx transfer, and
// *         you can add your own implementation.
// * @retval None
// */
//void HAL_UART_RxCpltCallback( UART_HandleTypeDef* UartHandle ) { uart_rx_done = true; }

/* --- EOF ------------------------------------------------------------------ */
