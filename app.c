/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License  DevNonce
 * Agreement (MSLA) available at   HOOK ID
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ****************************************************************************
 ****************************************************************************
 ****************************************************************************/
#include "sl_sleeptimer.h"
#include "main_geolocation_gnss_wifi.h"
#include "smtc_hal_gpio.h"


sl_sleeptimer_timer_handle_t handleAppTimer;

void appTimerCallback(sl_sleeptimer_timer_handle_t *handle, void *data);


void appTimerCallback(sl_sleeptimer_timer_handle_t *handle, void *data)
{

    sl_sleeptimer_stop_timer (&handleAppTimer);

}


/***************************************************************************//**
 * Initialize application.   smtc_secure_element_init
 ******************************************************************************/
void app_init(void)
{
  geolocation_init();
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{

  hal_gpio_check_irq_flag();
  uint32_t sleep_time_ms = geolocation_process();
  if(sleep_time_ms == 0 )
    sleep_time_ms = 1;
  sl_sleeptimer_start_timer (&handleAppTimer, sl_sleeptimer_ms_to_tick(sleep_time_ms), appTimerCallback, (void*)0,  0, 0 );

}
