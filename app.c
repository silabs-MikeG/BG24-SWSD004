/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This  startup_time_in_tick
 * software is distributed to you in Source Code format and is governed by the internal Wi-Fi scan start
 * sections of the MSLA applicable to Source Code.
 *
 ****************************************************************************
 ****************************************************************************
 ****************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "sl_sleeptimer.h"
#include "main_geolocation_gnss_wifi.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_dbg_trace.h"


#define EM_GPIO1_PORT   gpioPortA
#define EM_GPIO2_PORT   gpioPortA

#define EM_GPIO1_PIN    6
#define EM_GPIO2_PIN    7

sl_sleeptimer_timer_handle_t handleAppTimer;
bool appLoraModemSleep;
bool appLoraTimerWake;



// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

void appTimerCallback(sl_sleeptimer_timer_handle_t *handle, void *data);


void appTimerCallback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  appLoraTimerWake = true;
}


/***************************************************************************//**
 * Function called by power manager to ensure that system is ok to sleep.
 ******************************************************************************/
bool app_is_ok_to_sleep()
{
  return appLoraModemSleep;
}



/***************************************************************************//**
 * Initialize application.   smtc_secure_element_init
 ******************************************************************************/
void app_init(void)
{
  geolocation_init();
  appLoraModemSleep = true;
  appLoraTimerWake = false;
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  sl_status_t error;

  if(appLoraTimerWake)
    {
      appLoraTimerWake = false;
      //      HAL_DBG_TRACE_ERROR("LoRa Timer expired\n");
    }
  uint32_t sleep_time_ms = geolocation_process();
  if(sleep_time_ms <= 20 ){
      //      sleep_time_ms = 10;
      if(appLoraModemSleep){
          appLoraModemSleep = false;
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
      }
  }
  else{
      if(!appLoraModemSleep){
          appLoraModemSleep = true;
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
      }
  }
  error = sl_sleeptimer_restart_timer (&handleAppTimer, sl_sleeptimer_ms_to_tick(sleep_time_ms), appTimerCallback, (void*)0,  0, 0 );

  if(error != SL_STATUS_OK)
    {
      HAL_DBG_TRACE_ERROR("Sleep Timer err %X\r\n", error);
    }
}



/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      //Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
          advertising_set_handle,
          1600, // min. adv. interval (milliseconds * 1.6)
          1600, // max. adv. interval (milliseconds * 1.6)
          0,   // adv. duration
          0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

      // -------------------------------
      // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      printf("Connection Opened\r\n");
      break;

      // -------------------------------
      // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      printf("Connection Closed\r\n");
      break;

      ///////////////////////////////////////////////////////////////////////////
      // Add additional event handlers here as your application requires!      //
      ///////////////////////////////////////////////////////////////////////////

      // -------------------------------
      // Default event handler.
    default:
      break;
  }
}

