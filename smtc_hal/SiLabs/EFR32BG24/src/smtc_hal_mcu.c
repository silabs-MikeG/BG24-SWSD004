/*!
 * \file      smtc_hal_mcu.c
 *
 * \brief     MCU Hardware Abstraction Layer implementation
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

#include "smtc_hal_mcu.h"
#include "smtc_hal_mcu_ex.h"
#include "modem_pinout.h"

#include "smtc_board.h"



#include "smtc_hal.h"
#include "em_cmu.h"
#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * Watchdog counter reload value during sleep
 *
 * \remark The period must be lower than MCU watchdog period
 */
#define WATCHDOG_RELOAD_PERIOD_SECONDS 20
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Low Power options
 */
typedef enum low_power_mode_e
{
    LOW_POWER_ENABLE,
    LOW_POWER_DISABLE,
    LOW_POWER_DISABLE_ONCE
} low_power_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile bool             exit_wait            = false;
static volatile low_power_mode_t lp_current_mode      = LOW_POWER_ENABLE;
static bool                      partial_sleep_enable = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief init the MCU clock tree
 */
static void hal_mcu_system_clock_config( void );

/*!
 * @brief init the GPIO
 */
static void hal_mcu_gpio_init( void );

/*!
 * @brief init the power voltage detector
 */
static void hal_mcu_pvd_config( void );

/*!
 * @brief reinit the MCU clock tree after a stop mode
 */
static void hal_mcu_system_clock_re_config_after_stop( void );

/*!
 * @brief Deinit the MCU
 */
static void hal_mcu_lpm_mcu_deinit( void );

/*!
 * @brief MCU enter in low power stop mode
 */
static void hal_mcu_lpm_enter_stop_mode( void );

/*!
 * @brief MCU exit in low power stop mode
 */
static void hal_mcu_lpm_exit_stop_mode( void );

/*!
 * @brief Function runing the low power mode handler
 */
static void hal_mcu_lpm_handler( void );

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
/*!
 * @brief printf
 */
static void vprint( const char* fmt, va_list argp );
#endif

/*!
 * @brief Configure the STM32WB SMPS
 */
static void hal_mcu_smps_config( void );

/*!
 * @brief Activate the forward of the LSE on RCO pin
 *
 * @param enable Enable or disable the LSCO
 */
static void hal_mcu_system_clock_forward_LSE( bool enable );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void hal_mcu_critical_section_end( uint32_t* mask ) { __set_PRIMASK( *mask ); }

void hal_mcu_disable_irq( void ) { __disable_irq( ); }

void hal_mcu_enable_irq( void ) { __enable_irq( ); }

void hal_mcu_delay_ms( uint32_t delay_ms ) { sl_sleeptimer_delay_millisecond( delay_ms ); }

uint32_t hal_mcu_get_tick( void ) { return HAL_GetTick( ); }

void hal_mcu_init( void )
{

    /* Initialize clocks */
    hal_mcu_system_clock_config( );

    // Initialize watchdog
#if( HAL_USE_WATCHDOG == HAL_FEATURE_ON )
    hal_watchdog_init( );
#endif  // HAL_USE_WATCHDOG == HAL_FEATURE_ON

    /* Initialize GPIOs */
    hal_mcu_gpio_init( );

//    /* Initialize I2C */
//    hal_i2c_init( HAL_I2C_ID, SMTC_I2C_SDA, SMTC_I2C_SCL );

    /* Initialize UART */
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_init( HAL_PRINTF_UART_ID, UART_TX, UART_RX );
#endif

    /* Initialize low power timer */
    hal_lp_timer_init( );

    /* Initialize the user flash */
    hal_flash_init( );

    /* Init power voltage voltage detector */
    hal_mcu_pvd_config( );

    /* Initialize SPI */
    hal_spi_init( HAL_RADIO_SPI_ID, SMTC_RADIO_SPI_MOSI, SMTC_RADIO_SPI_MISO, SMTC_RADIO_SPI_SCLK );

    /* Initialize RTC */
    hal_rtc_init( );

    /* Initialize ADC */
    hal_adc_init( );
}

void hal_mcu_reset( void )
{
    __disable_irq( );
    NVIC_SystemReset( );  // Restart system
}

void __attribute__( ( optimize( "O0" ) ) ) hal_mcu_wait_us( const int32_t microseconds )
{
    const uint32_t nb_nop = microseconds * 1000 / 363;
    for( uint32_t i = 0; i < nb_nop; i++ )
    {
        __NOP( );
    }
}

void hal_mcu_set_sleep_for_ms( const int32_t milliseconds )
{
    bool last_sleep_loop = false;

    if( milliseconds <= 0 )
    {
        return;
    }

    int32_t time_counter = milliseconds;

    hal_watchdog_reload( );

    if( lp_current_mode == LOW_POWER_ENABLE )
    {
        do
        {
            if( ( time_counter > ( WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 ) ) )
            {
                time_counter -= WATCHDOG_RELOAD_PERIOD_SECONDS * 1000;
                hal_rtc_wakeup_timer_set_ms( WATCHDOG_RELOAD_PERIOD_SECONDS * 1000 );
            }
            else
            {
                hal_rtc_wakeup_timer_set_ms( time_counter );
                // if the sleep time is less than the wdog reload period, this is the last sleep loop
                last_sleep_loop = true;
            }
            hal_mcu_lpm_handler( );
            hal_watchdog_reload( );
        } while( ( hal_rtc_has_wut_irq_happened( ) == true ) && ( last_sleep_loop == false ) );
        if( ( last_sleep_loop == false ) || ( lp_current_mode == LOW_POWER_DISABLE_ONCE ) )
        {
            // in case sleep mode is interrupted by an other irq than the wake up timer, stop it and exit
            hal_rtc_wakeup_timer_stop( );
        }
    }
}

uint16_t hal_mcu_get_vref_level( void ) { return hal_adc_get_vref_int( ); }

int16_t hal_mcu_get_temperature( void ) { return hal_adc_get_temp( ); }

void hal_mcu_disable_low_power_wait( void )
{
    exit_wait       = true;
    lp_current_mode = LOW_POWER_DISABLE;
}

void hal_mcu_enable_low_power_wait( void )
{
    exit_wait       = false;
    lp_current_mode = LOW_POWER_ENABLE;
}

void hal_mcu_trace_print( const char* fmt, ... )
{
#if HAL_DBG_TRACE == HAL_FEATURE_ON
    va_list argp;
    va_start( argp, fmt );
    vprint( fmt, argp );
    va_end( argp );
#endif
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line
 * number where the assert_param error has occurred. Input          : - file:
 * pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    // User can add his own implementation to report the file name and line
    // number,
    // ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line)

    SMTC_HAL_TRACE_PRINTF( "Wrong parameters value: file %s on line %lu\r\n", ( const char* ) file, line );
    // Infinite loop
    while( 1 )
    {
    }
}
#endif

//
//void HAL_MspInit( void )
//{
//    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );
//
//    /* HSEM Clock enable */
//    __HAL_RCC_HSEM_CLK_ENABLE( );
//
//    /* System interrupt init*/
//    /* MemoryManagement_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority( MemoryManagement_IRQn, 0, 0 );
//    /* BusFault_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority( BusFault_IRQn, 0, 0 );
//    /* UsageFault_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority( UsageFault_IRQn, 0, 0 );
//    /* SVCall_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority( SVCall_IRQn, 0, 0 );
//    /* DebugMonitor_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority( DebugMonitor_IRQn, 0, 0 );
//    /* PendSV_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority( PendSV_IRQn, 0, 0 );
//    /* SysTick_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
//}

void hal_mcu_partial_sleep_enable( bool enable ) { partial_sleep_enable = enable; }

void hal_mcu_smps_enable( bool enable )
{
    if( enable )
    {
        LL_PWR_SMPS_Enable( );
    }
    else
    {
        LL_PWR_SMPS_Disable( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void hal_mcu_system_clock_config( void )
{

}

static void hal_mcu_smps_config( void )
{

}

static void hal_mcu_pvd_config( void )
{

}

static void hal_mcu_gpio_init( void )
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    hal_gpio_init_out( SMTC_RADIO_NSS, 1 );
    hal_gpio_init_in( SMTC_RADIO_BUSY, HAL_GPIO_PULL_MODE_DOWN, HAL_GPIO_IRQ_MODE_OFF, NULL );
    // Here init only the pin as an exti rising and the callback will be attached later
    hal_gpio_init_in( SMTC_RADIO_DIOX, HAL_GPIO_PULL_MODE_DOWN, HAL_GPIO_IRQ_MODE_RISING, NULL );
    hal_gpio_init_out( SMTC_RADIO_NRST, 1 );

    hal_gpio_init_out( SMTC_LED_RX, 0 );
    hal_gpio_init_out( SMTC_LED_TX, 0 );
}

/**
 * @brief Enters Low Power Stop Mode
 *
 * @note ARM exits the function when waking up
 *
 */
static void hal_mcu_lpm_enter_stop_mode( void )
{

}

/**
 * @brief Exists Low Power Stop Mode
 *
 */
static void hal_mcu_lpm_exit_stop_mode( void )
{

}

/**
 * @brief Low power handler
 *
 */
static void hal_mcu_lpm_handler( void )
{
  sl_power_manager_sleep();
}

/**
 * @brief De-init periph begore going in sleep mode
 *
 */
static void hal_mcu_lpm_mcu_deinit( void )
{
    hal_spi_deinit( HAL_RADIO_SPI_ID );

    /* Disable I2C */
    hal_i2c_deinit( HAL_I2C_ID );
    /* Disable UART */
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_deinit( HAL_PRINTF_UART_ID );
#endif
}

/**
 * @brief Re-init MCU clock after a wait in stop mode 2
 *
 */
void hal_mcu_lpm_mcu_reinit( void )
{
    /* Reconfig needed OSC and PLL */
    hal_mcu_system_clock_re_config_after_stop( );

    /* Initialize UART */
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_init( HAL_PRINTF_UART_ID, UART_TX, UART_RX );
#endif
    /* Initialize I2C */
    hal_i2c_init( HAL_I2C_ID, SMTC_I2C_SDA, SMTC_I2C_SCL );

    /* Initialize SPI */
    hal_spi_init( HAL_RADIO_SPI_ID, SMTC_RADIO_SPI_MOSI, SMTC_RADIO_SPI_MISO, SMTC_RADIO_SPI_SCLK );
}

static void hal_mcu_system_clock_re_config_after_stop( void )
{

}

void hal_mcu_system_clock_forward_LSE( bool enable )
{

}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler( void )
{
    HAL_DBG_TRACE_ERROR( "HARDFAULT_Handler\n" );

    /* reset the board */
    hal_mcu_reset( );
}

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
static void vprint( const char* fmt, va_list argp )
{
    char string[HAL_PRINT_BUFFER_SIZE];
    if( 0 < vsprintf( string, fmt, argp ) )  // build string
    {
        hal_uart_tx( HAL_PRINTF_UART_ID, ( uint8_t* ) string, strlen( string ) );
    }
}
#endif

/* --- EOF ------------------------------------------------------------------ */
