/*!
 * \file      smtc_hal_lp_timer.c
 *
 * \brief     Implements Low Power Timer utilities functions.
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
//#define DEBUG_HAL_LP_TIMER
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal_lp_timer.h"
#include "sl_sleeptimer.h"
#include "smtc_hal_mcu.h"
#ifdef DEBUG_HAL_LP_TIMER
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */


static hal_lp_timer_irq_t lptim_tmr_irq = { .context = NULL, .callback = NULL };

sl_sleeptimer_timer_handle_t handleLpLoraTimer;

sl_sleeptimer_timer_callback_t lpTimerCallback(sl_sleeptimer_timer_handle_t *handle, void *data);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init( void )
{
  /* Using SiLabs Sleep Timer service for low power timer functions and IRQ  */
}

void hal_lp_timer_start( const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    uint32_t delay_ms_2_tick = 0;

    // Remark LSE_VALUE / LPTIM_PRESCALER_DIV16
    delay_ms_2_tick = ( uint32_t )( ( ( uint64_t ) milliseconds * ( 32768 ) ) / 1000 );

    // check if delay_ms_2_tick is not greater than 0xFFFF and clamp it if it is the case
    if( delay_ms_2_tick > 0xFFFFFF )
    {
        delay_ms_2_tick = 0xFFFFFF;
    }


    lptim_tmr_irq = *tmr_irq;
#ifdef DEBUG_HAL_LP_TIMER
    SMTC_MODEM_HAL_TRACE_WARNING( "hal_lp_timer_start %d milliseconds\n", milliseconds );
#endif

    sl_sleeptimer_start_timer (&handleLpLoraTimer, delay_ms_2_tick, lpTimerCallback, (void*)0,  0, 0 );

}

void hal_lp_timer_stop( void ) { sl_sleeptimer_stop_timer (&handleLpLoraTimer); }

void hal_lp_timer_irq_enable( void ) { NVIC_EnableIRQ( SYSRTC_APP_IRQn ); }

void hal_lp_timer_irq_disable( void ) { NVIC_DisableIRQ( SYSRTC_APP_IRQn ); }

sl_sleeptimer_timer_callback_t lpTimerCallback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
    sl_sleeptimer_stop_timer (&handleLpLoraTimer);
#ifdef DEBUG_HAL_LP_TIMER
    SMTC_MODEM_HAL_TRACE_WARNING( "lpTimerCallback\n" );
#endif
    if( lptim_tmr_irq.callback != NULL )
    {
        lptim_tmr_irq.callback( lptim_tmr_irq.context );
    }
}


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
