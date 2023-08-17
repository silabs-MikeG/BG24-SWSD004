/*!
 * @file      smtc_hal_gpio.c
 *
 * @brief     Implements the gpio HAL functions
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

//#define DEBUG_HAL_GPIO
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal.h"
#include "em_gpio.h"
#include "smtc_hal_gpio.h"
#include "gpiointerrupt.h"
#ifdef DEBUG_HAL_GPIO
#include "smtc_modem_hal_dbg_trace.h"
#endif

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
 * GPIO setup data structure
 */
typedef struct
{
  hal_gpio_pin_names_t pin;
  GPIO_Port_TypeDef     port;
  uint8_t               pin_num;
  uint32_t             mode;
  uint32_t             pull;
  bool                irq_risingEdge;
  bool                irq_fallingEdge;
  bool                irq_en;
} hal_gpio_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Array holding attached IRQ gpio data context
 */
static hal_gpio_irq_t const* gpio_irq[16];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
void HAL_GPIO_EXTI_Callback( uint8_t gpio_pin );
/*!
 * Generic gpio initialization
 *
 * @param [in/out] gpio  Holds MCU gpio parameters
 * @param [in]     value Initial MCU pin value
 * @param [in/out] irq   Pointer to IRQ data context.
 *                         NULL when setting gpio as output
 */
static void hal_gpio_init( const hal_gpio_t* gpio, const hal_gpio_state_t value, const hal_gpio_irq_t* irq );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
uint8_t hal_get_gpio_pin_num(hal_gpio_pin_names_t pin)
{
  int pin_num;

  pin_num = pin & 0x0F;

  return pin_num;
}

GPIO_Port_TypeDef hal_get_gpio_port(hal_gpio_pin_names_t pin)
{
  GPIO_Port_TypeDef port = 0;
  uint8_t temp_port;

  temp_port = (pin & 0xF0) >> 4;
  switch(temp_port){
    case 0x00:
      port = gpioPortA;
      break;
    case 0x01:
      port = gpioPortB;
      break;
    case 0x02:
      port = gpioPortC;
      break;
    case 0x03:
      port = gpioPortD;
      break;
      //    case 0x04:
      //      port = gpioPortE;
      //      break;
    default:
      port = 0x00;
      break;
  }
  return port;
}

void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const hal_gpio_state_t value )
{
  hal_gpio_t gpio = {
      .pin = pin, .port = hal_get_gpio_port(pin), .pin_num = hal_get_gpio_pin_num(pin), .mode = gpioModePushPull, .pull = 0
  };

  GPIO_PinModeSet(gpio.port, gpio.pin_num, gpio.mode, value);
}

void hal_gpio_deinit( const hal_gpio_pin_names_t pin )
{
  hal_gpio_t gpio = {
      .pin = pin,.port = hal_get_gpio_port(pin), .pin_num = hal_get_gpio_pin_num(pin), .mode = gpioModeDisabled, .pull = 0
  };
  GPIO_PinModeSet(gpio.port, gpio.pin_num, gpio.mode, 0);
}

/**
 * @brief MCU input pin Handling
 */

void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
                       const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t* irq )
{
  //  const uint32_t modes[] = { GPIO_MODE_INPUT, GPIO_MODE_IT_RISING, GPIO_MODE_IT_FALLING,
  //      GPIO_MODE_IT_RISING_FALLING };
  //  const uint32_t pulls[] = { GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN };

  bool rising_edge, falling_edge, enable_irq;
  hal_gpio_t gpio = {
      .pin = pin, .port = hal_get_gpio_port(pin), .pin_num = hal_get_gpio_pin_num(pin), .mode = NULL, .pull = NULL
  };

  gpio.mode = (pull_mode == HAL_GPIO_PULL_MODE_NONE) ? gpioModeInput : gpioModeInputPull;

  gpio.pull = (pull_mode == HAL_GPIO_PULL_MODE_DOWN) ?  0 : 1;
  GPIO_PinModeSet(gpio.port, gpio.pin_num, gpio.mode, gpio.pull);
  if( irq != NULL )
    {
#ifdef DEBUG_HAL_GPIO
//      SMTC_MODEM_HAL_TRACE_WARNING( "GPIO IRQ init, ping 0x%x\n", pin);
#endif
      irq->pin = pin;
      hal_gpio_irq_attach(irq);
    }

  if(irq_mode != HAL_GPIO_IRQ_MODE_OFF){
#ifdef DEBUG_HAL_GPIO
//      SMTC_MODEM_HAL_TRACE_WARNING( "GPIO IRQ init, pin 0x%x\n", pin);
#endif
      switch(irq_mode){
        case HAL_GPIO_IRQ_MODE_OFF:
          gpio.irq_risingEdge = false;
          gpio.irq_fallingEdge = false;
          gpio.irq_en = false;
          break;
        case HAL_GPIO_IRQ_MODE_RISING:
          gpio.irq_risingEdge = true;
          gpio.irq_fallingEdge = false;
          gpio.irq_en = true;
          break;
        case HAL_GPIO_IRQ_MODE_FALLING:
          gpio.irq_risingEdge = false;
          gpio.irq_fallingEdge = true;
          gpio.irq_en = true;
          break;
        case HAL_GPIO_IRQ_MODE_RISING_FALLING:
          gpio.irq_risingEdge = true;
          gpio.irq_fallingEdge = true;
          gpio.irq_en = true;
          break;
      }

      GPIO_ExtIntConfig(gpio.port,
                        gpio.pin_num,
                        gpio.pin_num, //Murata: TODO should be intno.
                        gpio.irq_risingEdge,
                        gpio.irq_fallingEdge,
                        gpio.irq_en);

      // Register callback functions and enable interrupts
      GPIOINT_CallbackRegister(gpio.pin_num, HAL_GPIO_EXTI_Callback);
      GPIO_IntEnable(1<<gpio.pin_num);
  }
}

void hal_gpio_irq_attach( const hal_gpio_irq_t* irq )
{
#ifdef DEBUG_HAL_GPIO
  SMTC_MODEM_HAL_TRACE_WARNING( "hal_gpio_irq_attach, pin 0x%x\n", irq->pin);
#endif
  if( ( irq != NULL ) && ( irq->callback != NULL ) )
    {
      gpio_irq[( irq->pin ) & 0x0F] = irq;
    }
}

void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq )
{
  if( irq != NULL )
    {
      gpio_irq[( irq->pin ) & 0x0F] = NULL;
    }
}

static uint32_t gpio_interrupt_state;
void hal_gpio_irq_enable( void )
{
#ifdef DEBUG_HAL_GPIO
  gpio_interrupt_state = GPIO_EnabledIntGet();
  SMTC_MODEM_HAL_TRACE_WARNING( "GPIO IRQ enabled, state 0x%x\n", gpio_interrupt_state);
#endif

//  GPIO_IntEnable(gpio_interrupt_state);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

}

void hal_gpio_irq_disable( void )
{
#ifdef DEBUG_HAL_GPIO
  gpio_interrupt_state = GPIO_EnabledIntGet();
  SMTC_MODEM_HAL_TRACE_WARNING( "GPIO IRQ disabled, state 0x%x\n", gpio_interrupt_state);
#endif

//  GPIO_IntDisable(0xFFFFFFFF);
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
  NVIC_DisableIRQ(GPIO_ODD_IRQn);
}

/**
 * @brief MCU pin state control
 */

void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const hal_gpio_state_t value )
{
  if(value == HAL_GPIO_SET)
    {
      GPIO_PinOutSet (hal_get_gpio_port(pin), hal_get_gpio_pin_num(pin));
    }
  else
    {
      GPIO_PinOutClear (hal_get_gpio_port(pin), hal_get_gpio_pin_num(pin));
    }

}

void hal_gpio_toggle( const hal_gpio_pin_names_t pin )
{
  GPIO_PinOutToggle (hal_get_gpio_port(pin), hal_get_gpio_pin_num(pin));
}

uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin )
{


  return ( GPIO_PinInGet (hal_get_gpio_port(pin), hal_get_gpio_pin_num(pin)));
}

void hal_gpio_clear_pending_irq( const hal_gpio_pin_names_t pin )
{
  //uint8_t temp_pin;

  //temp_pin = hal_get_gpio_pin_num(pin);
  //GPIO_IntClear ((1 << temp_pin));
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void hal_gpio_init( const hal_gpio_t* gpio, const hal_gpio_state_t value, const hal_gpio_irq_t* irq )
{
  //  GPIO_InitTypeDef gpio_local;
  //  GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( gpio->pin & 0xF0 ) << 6 ) );
  //
  //  gpio_local.Pin       = ( 1 << ( gpio->pin & 0x0F ) );
  //  gpio_local.Mode      = gpio->mode;
  //  gpio_local.Pull      = gpio->pull;
  //  gpio_local.Speed     = gpio->speed;
  //  gpio_local.Alternate = gpio->alternate;
  //
  //  if( gpio_port == GPIOA )
  //    {
  //      __HAL_RCC_GPIOA_CLK_ENABLE( );
  //    }
  //  else if( gpio_port == GPIOB )
  //    {
  //      __HAL_RCC_GPIOB_CLK_ENABLE( );
  //    }
  //  else if( gpio_port == GPIOC )
  //    {
  //      __HAL_RCC_GPIOC_CLK_ENABLE( );
  //    }
  //  else if( gpio_port == GPIOD )
  //    {
  //      __HAL_RCC_GPIOD_CLK_ENABLE( );
  //    }
  //  else if( gpio_port == GPIOE )
  //    {
  //      __HAL_RCC_GPIOE_CLK_ENABLE( );
  //    }
  //  else if( gpio_port == GPIOH )
  //    {
  //      __HAL_RCC_GPIOH_CLK_ENABLE( );
  //    }
  //
  //  HAL_GPIO_WritePin( gpio_port, gpio_local.Pin, ( GPIO_PinState ) value );
  //  HAL_GPIO_Init( gpio_port, &gpio_local );
  //
  //  if( ( gpio->mode == GPIO_MODE_IT_RISING ) || ( gpio->mode == GPIO_MODE_IT_FALLING ) ||
  //      ( gpio->mode == GPIO_MODE_IT_RISING_FALLING ) )
  //    {
  //      hal_gpio_irq_attach( irq );
  //      switch( gpio->pin & 0x0F )
  //      {
  //        case 0:
  //          HAL_NVIC_SetPriority( EXTI0_IRQn, 0, 0 );
  //          HAL_NVIC_EnableIRQ( EXTI0_IRQn );
  //          break;
  //        case 1:
  //          HAL_NVIC_SetPriority( EXTI1_IRQn, 0, 0 );
  //          HAL_NVIC_EnableIRQ( EXTI1_IRQn );
  //          break;
  //        case 2:
  //          HAL_NVIC_SetPriority( EXTI2_IRQn, 0, 0 );
  //          HAL_NVIC_EnableIRQ( EXTI2_IRQn );
  //          break;
  //        case 3:
  //          HAL_NVIC_SetPriority( EXTI3_IRQn, 0, 0 );
  //          HAL_NVIC_EnableIRQ( EXTI3_IRQn );
  //          break;
  //        case 4:
  //          HAL_NVIC_SetPriority( EXTI4_IRQn, 0, 0 );
  //          HAL_NVIC_EnableIRQ( EXTI4_IRQn );
  //          break;
  //        case 5:
  //        case 6:
  //        case 7:
  //        case 8:
  //        case 9:
  //          HAL_NVIC_SetPriority( EXTI9_5_IRQn, 0, 0 );
  //          HAL_NVIC_EnableIRQ( EXTI9_5_IRQn );
  //          break;
  //        default:
  //          HAL_NVIC_SetPriority( EXTI15_10_IRQn, 0, 0 );
  //          HAL_NVIC_EnableIRQ( EXTI15_10_IRQn );
  //          break;
  //      }
  //    }
}


uint32_t gpio_irq_flags = 0;

void hal_gpio_check_irq_flag(void){
  uint8_t callback_index = 0;

  if( gpio_irq_flags > 0 )
    {
      for(callback_index = 0; callback_index < 16; callback_index++){

          if((1<<callback_index) & gpio_irq_flags){
              if( ( gpio_irq[callback_index] != NULL ) && ( gpio_irq[callback_index]->callback != NULL ) )
                {
#ifdef DEBUG_HAL_GPIO
                  SMTC_MODEM_HAL_TRACE_WARNING( "GPIO IRQ callback start\n");
#endif
                  gpio_irq[callback_index]->callback( gpio_irq[callback_index]->context );
#ifdef DEBUG_HAL_GPIO
                  SMTC_MODEM_HAL_TRACE_WARNING( "GPIO IRQ callback end\n");
#endif
                }
          }
      }
    }
}

void HAL_GPIO_EXTI_Callback( uint8_t gpio_pin )
{
  uint8_t callback_index = 0;
#ifdef DEBUG_HAL_GPIO
  SMTC_MODEM_HAL_TRACE_WARNING( "****HD:HAL_GPIO_EXTI_Callback,gpio_pin: 0x%x\n",gpio_pin);
#endif
  gpio_irq_flags |= (1<<gpio_pin);
  //if( gpio_pin > 0 )
    //{
      callback_index = gpio_pin;
      if( ( gpio_irq[callback_index] != NULL ) && ( gpio_irq[callback_index]->callback != NULL ) )
        {
          gpio_irq[callback_index]->callback( gpio_irq[callback_index]->context );
        }
    //}
}

/* --- EOF ------------------------------------------------------------------ */
