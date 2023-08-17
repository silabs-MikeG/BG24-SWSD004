/*!
 * @ingroup   apps_geolocation
 * @file      main_geolocation_gnss_wifi.c
 *
 * @brief     LoRa Basics Modem LR11XX Geolocation GNSS and Wi-Fi example.
 *
 * @copyright
 * @parblock
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
 * @endparblock
 */

/*!
 * @addtogroup apps_geolocation
 * LoRa Basics Modem LR11XX Geolocation example
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "main_geolocation_gnss_wifi.h"
#include "lorawan_key_config.h"
#include "smtc_board.h"
#include "smtc_hal.h"
#include "apps_utilities.h"
#include "apps_modem_common.h"
#include "apps_modem_event.h"

#include "gnss_middleware.h"
#include "wifi_middleware.h"
#include "lr11xx_system.h"

#include "smtc_modem_api.h"
#include "smtc_modem_test_api.h"
#include "smtc_modem_utilities.h"
#include "smtc_board_ralf.h"

#ifdef PORTING_TEST
#include "smtc_modem_hal.h"

#include "smtc_hal_dbg_trace.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"

#include "ralf_lr11xx.h"
#endif

#include <stdio.h>
#include <string.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
/**
 * @brief LR11XX radio firmware
 */
#define LR1110_FW_VERSION 0x0307
#define LR1110_FW_TYPE 0x01
#define LR1120_FW_VERSION 0x0101
#define LR1120_FW_TYPE 0x02

#ifdef PORTING_TEST
// !! SHOULD BE DEFINED BY USER !!
#define ENABLE_TEST_FLASH 0  // Enable flash porting test BUT disable other porting tests

// Delay introduced by the start of lptim timer in stm32
#define COMPENSATION_IN_MS_STM32 0

#define NB_LOOP_TEST_SPI 2
#define NB_LOOP_TEST_CONFIG_RADIO 2

#define LR11XX_FW_VERSION 0x0307

#define FREQ_NO_RADIO 868300000
#define SYNC_WORD_NO_RADIO 0x21

#define MARGIN_GET_TIME_IN_MS 1
#define MARGIN_TIMER_IRQ_IN_MS 2
#define MARGIN_TIME_CONFIG_RADIO_IN_MS 8
#define MARGIN_SLEEP_IN_MS 2
#endif

/**
 * @brief Duration in second after last ALC sync response received to consider the local clock time invalid
 *
 * Set time valid for 1 day (to be fine tuned depending on board properties)
 */
#define APP_ALC_TIMING_INVALID ( 3600 * 24 )

/**
 * @brief Interval in second between two consecutive ALC sync requests
 *
 * 3 time sync requests per day
 */
#define APP_ALC_TIMING_INTERVAL ( APP_ALC_TIMING_INVALID / 3 )

/**
 * @brief Duration in second after Join to start the application keep alive sequence
 */
#define APP_TIMER_START ( 10 )

/**
 * @brief Duration in second for keep alive alarm sequence
 */
#define APP_TIMER_KEEP_ALIVE ( 30 )

#ifdef PORTING_TEST
#define PORTING_TEST_MSG_OK( )                                \
    do                                                        \
    {                                                         \
        HAL_DBG_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_GREEN );   \
        HAL_DBG_TRACE_PRINTF( " OK \n" );                    \
        HAL_DBG_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT ); \
    } while( 0 );

#define PORTING_TEST_MSG_WARN( ... )                          \
    do                                                        \
    {                                                         \
        HAL_DBG_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_YELLOW );  \
        HAL_DBG_TRACE_PRINTF( __VA_ARGS__ );                 \
        HAL_DBG_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT ); \
    } while( 0 );

#define PORTING_TEST_MSG_NOK( ... )                           \
    do                                                        \
    {                                                         \
        HAL_DBG_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_RED );     \
        HAL_DBG_TRACE_PRINTF( "\n NOK:" );                   \
        HAL_DBG_TRACE_PRINTF( __VA_ARGS__ );                 \
        HAL_DBG_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT ); \
    } while( 0 );

#if defined( SX128X )
const ralf_t modem_radio = RALF_SX128X_INSTANTIATE( NULL );
#elif defined( SX126X )
const ralf_t modem_radio = RALF_SX126X_INSTANTIATE( NULL );
#elif defined( LR11XX )
//const ralf_t modem_radio = RALF_LR11XX_INSTANTIATE( NULL );
#else
#error "Please select radio board.."
#endif
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

#ifdef PORTING_TEST
/**
 * @brief Return test enumeration
 */
typedef enum return_code_test_e
{
    RC_PORTING_TEST_OK       = 0x00,  // Test OK
    RC_PORTING_TEST_NOK      = 0x01,  // Test NOK
    RC_PORTING_TEST_RELAUNCH = 0x02,  // Relaunch test
} return_code_test_t;
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Stack identifier
 */
static uint8_t stack_id = 0;

#ifdef PORTING_TEST
static volatile bool     radio_irq_raised      = false;
static volatile bool     irq_rx_timeout_raised = false;
static volatile bool     timer_irq_raised      = false;
static volatile uint32_t irq_time_ms           = 0;
static volatile uint32_t irq_time_s            = 0;

// LoRa configurations TO NOT receive or transmit
static ralf_params_lora_t rx_lora_param = { .sync_word                       = SYNC_WORD_NO_RADIO,
                                            .symb_nb_timeout                 = 0,
                                            .rf_freq_in_hz                   = FREQ_NO_RADIO,
                                            .mod_params.cr                   = RAL_LORA_CR_4_5,
                                            .mod_params.sf                   = RAL_LORA_SF12,
                                            .mod_params.bw                   = RAL_LORA_BW_125_KHZ,
                                            .mod_params.ldro                 = 0,
                                            .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
                                            .pkt_params.pld_len_in_bytes     = 255,
                                            .pkt_params.crc_is_on            = false,
                                            .pkt_params.invert_iq_is_on      = true,
                                            .pkt_params.preamble_len_in_symb = 8 };

static ralf_params_lora_t tx_lora_param = { .sync_word                       = SYNC_WORD_NO_RADIO,
                                            .symb_nb_timeout                 = 0,
                                            .rf_freq_in_hz                   = FREQ_NO_RADIO,
                                            .output_pwr_in_dbm               = 14,
                                            .mod_params.cr                   = RAL_LORA_CR_4_5,
                                            .mod_params.sf                   = RAL_LORA_SF12,
                                            .mod_params.bw                   = RAL_LORA_BW_125_KHZ,
                                            .mod_params.ldro                 = 0,
                                            .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
                                            .pkt_params.pld_len_in_bytes     = 50,
                                            .pkt_params.crc_is_on            = true,
                                            .pkt_params.invert_iq_is_on      = false,
                                            .pkt_params.preamble_len_in_symb = 8 };

static const char* name_context_type[] = { "MODEM", "LR1MAC", "DEVNONCE", "SECURE_ELEMENT" };
#endif

/*!
 * @brief Modem radio
 */
ralf_t* modem_radio;

/*!
 * @brief Wi-Fi output results
 */
static wifi_mw_event_data_scan_done_t wifi_results;

/*!
 * @brief First time sync status for application startup
 */
static bool is_first_time_sync = true;

/*!
 * @brief ADR custom list and retransmission definition for EU868 / IN865 / RU864 / AU915 / CN470 /AS923 / KR920 regions
 */
static const uint8_t adr_custom_list_dr5_dr3[16] = ADR_CUSTOM_LIST_DR5_DR3;
static const uint8_t custom_nb_trans_dr5_dr3     = CUSTOM_NB_TRANS_DR5_DR3;

/*!
 * @brief ADR custom list and retransmission definition for US915 region
 */
static const uint8_t adr_custom_list_us915[16] = ADR_CUSTOM_LIST_US915;
static const uint8_t custom_nb_trans_us915     = CUSTOM_NB_TRANS_US915;

/*!
 * @brief ADR custom list and retransmission definition for WW2G4 region
 */
static const uint8_t adr_custom_list_ww2g4[16] = ADR_CUSTOM_LIST_WW2G4;
static const uint8_t custom_nb_trans_ww2g4     = CUSTOM_NB_TRANS_WW2G4;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Helper function that configure the custom ADR configuration for geolocation scan & send, based on the region
 * already configured in the stack.
 *
 * Prior using this function, the region must have been set already in the stack.
 */
static void configure_adr( void );

/*!
 * @addtogroup basics_modem_evt_callback
 * LoRa Basics Modem event callbacks
 * @{
 */

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void on_modem_reset( uint16_t reset_count );

/*!
 * @brief Network Joined event callback
 */
static void on_modem_network_joined( void );

/*!
 * @brief Clock synchronisation event callback
 */
static void on_modem_clk_synch( smtc_modem_event_time_status_t time_status );

/*!
 * @brief Downlink data event callback.
 *
 * @param [in] rssi       RSSI in signed value in dBm + 64
 * @param [in] snr        SNR signed value in 0.25 dB steps
 * @param [in] rx_window  RX window
 * @param [in] port       LoRaWAN port
 * @param [in] payload    Received buffer pointer
 * @param [in] size       Received buffer size
 */
static void on_modem_down_data( int8_t rssi, int8_t snr, smtc_modem_event_downdata_window_t rx_window, uint8_t port,
                                const uint8_t* payload, uint8_t size );

/*!
 * @brief GNSS Almanac update event callback
 */
static void on_modem_almanac_update( smtc_modem_event_almanac_update_status_t status );

/*!
 * @brief GNSS middleware event callback
 */
static void on_middleware_gnss_event( uint8_t pending_events );

/*!
 * @brief Wi-Fi middleware event callback
 */
static void on_middleware_wifi_event( uint8_t pending_events );

#ifdef PORTING_TEST
static void radio_tx_irq_callback( void* obj );
static void radio_rx_irq_callback( void* obj );
static void radio_irq_callback_get_time_in_s( void* obj );
static void timer_irq_callback( void* obj );

static bool               reset_init_radio( void );
static return_code_test_t test_get_time_in_s( void );
static return_code_test_t test_get_time_in_ms( void );
static bool               test_context_store_restore( modem_context_type_t context_type );
static bool               test_crashlog_store_restore( void );
static bool               test_crashlog_status( void );

static bool porting_test_spi( void );
static bool porting_test_radio_irq( void );
static bool porting_test_get_time( void );
static bool porting_test_timer_irq( void );
static bool porting_test_stop_timer( void );
static bool porting_test_disable_enable_irq( void );
static bool porting_test_random( void );
static bool porting_test_config_rx_radio( void );
static bool porting_test_config_tx_radio( void );
static bool porting_test_sleep_ms( void );
static void porting_test_timer_irq_low_power_wake( void );

static bool porting_test_flash( void );
static bool porting_test_crashlog( void );
#endif
/*!
 * @}
 */

/*  smtc_event_callback
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
apps_modem_event_callback_t smtc_event_callback = {
        .adr_mobile_to_static  = NULL,
        .alarm                 = NULL,
        .almanac_update        = on_modem_almanac_update,
        .down_data             = on_modem_down_data,
        .join_fail             = NULL,
        .joined                = on_modem_network_joined,
        .link_status           = NULL,
        .mute                  = NULL,
        .new_link_adr          = NULL,
        .reset                 = on_modem_reset,
        .set_conf              = NULL,
        .stream_done           = NULL,
        .time_updated_alc_sync = on_modem_clk_synch,
        .tx_done               = NULL,
        .upload_done           = NULL,
        .user_radio_access     = NULL,
        .middleware_1          = on_middleware_gnss_event,
        .middleware_2          = on_middleware_wifi_event,
    };
/**
 * @brief Main application entry point.
 */


int geolocation_init( void )
{
    lr11xx_system_version_t            lr11xx_fw_version;
    lr11xx_status_t                    status;
#ifdef PORTING_TEST	
    bool ret = true;
#endif


    /* Initialise the ralf_t object corresponding to the board */
    modem_radio = smtc_board_initialise_and_get_ralf( );

    /* Disable IRQ to avoid unwanted behaviour during init */
    hal_mcu_disable_irq( );

    /* Init board and peripherals */
    hal_mcu_init( );
    smtc_board_init_periph( );

    /* Init the Lora Basics Modem event callbacks */
    apps_modem_event_init( &smtc_event_callback );

    /* Init the modem and use apps_modem_event_process as event callback, please note that the callback will be called
     * immediately after the first call to modem_run_engine because of the reset detection */
    smtc_modem_init( modem_radio, &apps_modem_event_process );

    /* Re-enable IRQ */
    hal_mcu_enable_irq( );

#ifdef PORTING_TEST
    // Tests
    HAL_DBG_TRACE_MSG( "\n\n\nPORTING_TEST example is starting \n\n" );

#if( ENABLE_TEST_FLASH == 0 )

    ret = porting_test_spi( );
    if( ret == false )
        return;

    ret = porting_test_radio_irq( );
    if( ret == false )
        return;

    ret = porting_test_get_time( );

    ret = porting_test_timer_irq( );
    if( ret == false )
        return;

    porting_test_stop_timer( );

    porting_test_disable_enable_irq( );

    hal_watchdog_reload( );

    porting_test_random( );

    porting_test_config_rx_radio( );

    porting_test_config_tx_radio( );

    porting_test_sleep_ms( );

    porting_test_timer_irq_low_power_wake( );

#else

    ret = porting_test_flash( );
    if( ret == false )
        return ret;

    ret = porting_test_crashlog( );
    if( ret == false )
        return ret;

    HAL_DBG_TRACE_MSG_COLOR( "\n MCU RESET => relaunch tests and check if read after reset = write before reset \n\n",
                              HAL_DBG_TRACE_COLOR_BLUE );

    hal_mcu_set_sleep_for_ms( 2000 );

    hal_mcu_reset( );

#endif

    HAL_DBG_TRACE_MSG( "----------------------------------------\nEND \n\n" );
#endif
    //smtc_modem_hal_start_timer(3000, on_modem_timer, NULL);

    /* Notify user that the board is initialized */
    smtc_board_leds_blink( smtc_board_get_led_all_mask( ), 100, 2 );

    HAL_DBG_TRACE_MSG( "\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem GNSS/Wi-Fi Geolocation example ==== ######\n\n" );
    apps_modem_common_display_lbm_version( );

    /* Check LR11XX Firmware version */
    ASSERT_SMTC_MODEM_RC( smtc_modem_suspend_before_user_radio_access( ) ); /* protect from radio access conflicts */
    status = lr11xx_system_get_version( modem_radio->ral.context, &lr11xx_fw_version );
    ASSERT_SMTC_MODEM_RC( smtc_modem_resume_after_user_radio_access( ) );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get LR11XX firmware version\n" );
        mcu_panic( );
    }
    if( ( ( lr11xx_fw_version.fw != LR1110_FW_VERSION ) && ( lr11xx_fw_version.type = LR1110_FW_TYPE ) ) &&
        ( ( lr11xx_fw_version.fw != LR1120_FW_VERSION ) && ( lr11xx_fw_version.type = LR1120_FW_TYPE ) ) )
    {
        HAL_DBG_TRACE_ERROR( "Wrong LR11XX firmware version, expected 0x%04X, got 0x%04X\n", LR1110_FW_VERSION,
                             lr11xx_fw_version.fw );
        mcu_panic( );
    }
    HAL_DBG_TRACE_INFO( "LR11XX FW   : 0x%04X\n", lr11xx_fw_version.fw );

    HAL_DBG_TRACE_INFO( "Geolocation init Complete\n" );
    return 0;
}

uint32_t geolocation_process( void )
{
     /* Execute modem runtime, this function must be called again in sleep_time_ms milliseconds or sooner. */
         uint32_t sleep_time_ms = smtc_modem_run_engine( );
//         HAL_DBG_TRACE_INFO( "geolocation process, sleep for %s ms\n", sleep_time_ms );
         /* go in low power */
//         hal_mcu_set_sleep_for_ms( sleep_time_ms );
         return sleep_time_ms;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*!
 * @brief LoRa Basics Modem event callbacks called by smtc_event_process function
 */
#ifdef PORTING_TEST
/**
 * @brief Test SPI
 *
 * @remark
 * Prerequisite:
 * Radio reset should be implemented:
 * - drive of gpio (hal_gpio_set_value)
 * - mcu wait us (hal_mcu_wait_us)
 *
 * Test processing:
 * - Reset radio
 * - Read data through SPI
 * - Check if data is coherent
 *
 * Ported functions:
 * lr11xx_hal_read
 *      lr11xx_hal_check_device_ready
 *          lr11xx_hal_wait_on_busy
 *              hal_gpio_get_value
 *          hal_gpio_set_value
 *          hal_spi_in_out
 *
 * @return bool True if test is successful
 */
static bool porting_test_spi( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_spi : " );

    uint16_t counter_nok = 0;

    // Reset radio (prerequisite)
    ral_reset( &( modem_radio->ral ) );

    for( uint16_t i = 0; i < NB_LOOP_TEST_SPI; i++ )
    {
#if defined( LR11XX )
        lr11xx_system_version_t version;
        lr11xx_status_t         status;

        status = lr11xx_system_get_version( modem_radio->ral.context, &version );

        if( status == LR11XX_STATUS_OK )
        {
            if( version.fw != LR11XX_FW_VERSION )
            {
                PORTING_TEST_MSG_NOK( " Wrong LR11XX firmware version: expected 0x%04X / get 0x%04X \n",
                                      LR11XX_FW_VERSION, version.fw );
                counter_nok++;
            }
        }
        else
        {
            PORTING_TEST_MSG_NOK( " Failed to get LR11XX firmware version \n" );
            counter_nok++;
        }

#elif defined( SX126X )
        sx126x_chip_status_t chip_status;
        sx126x_status_t      status;

        status = sx126x_get_status( NULL, &chip_status );

        if( status == SX126X_STATUS_OK )
        {
            if( chip_status.chip_mode == SX126X_CHIP_MODE_UNUSED )
            {
                PORTING_TEST_MSG_NOK( " Wrong SX126X chip mode, get SX126X_CHIP_MODE_UNUSED \n" );
                counter_nok++;
            }
        }
        else
        {
            PORTING_TEST_MSG_NOK( " Failed to get SX126X status \n" );
            counter_nok++;
        }
#else
        PORTING_TEST_MSG_NOK( " Radio is not supported \n" );
        return false;
#endif
    }

    if( counter_nok == 0 )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_WARN( " Failed test = %u / %u \n", counter_nok, NB_LOOP_TEST_SPI );
        return false;
    }

    return true;
}

/**
 * @brief Reset and init radio
 *
 * @remark
 * Test processing:
 * - Reset radio
 * - Init radio
 * - Set radio in sleep mode
 *
 * Ported functions:
 * ral_reset:
 * lr11xx_hal_reset
 *     hal_gpio_set_value
 *     hal_mcu_wait_us
 * ral_init:
 * ral_lr11xx_init
 *     ral_lr11xx_bsp_get_crc_state
 *     ral_lr11xx_bsp_get_reg_mode
 *     ral_lr11xx_bsp_get_rf_switch_cfg
 *     ral_lr11xx_bsp_get_xosc_cfg
 *     lr11xx_system_set_tcxo_mode
 *         lr11xx_hal_write
 *     lr11xx_system_calibrate
 *         lr11xx_hal_write
 * ral_set_sleep:
 * ral_lr11xx_set_sleep
 *     lr11xx_system_set_sleep
 *         lr11xx_hal_write
 *
 * @return bool True if test is successful
 */
static bool reset_init_radio( void )
{
    ral_status_t status = RAL_STATUS_ERROR;

    // Reset, init radio and put it in sleep mode
    ral_reset( &( modem_radio->ral ) );

    status = ral_init( &( modem_radio->ral ) );
    if( status != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_init() function failed \n" );
        return false;
    }

    status = ral_set_sleep( &( modem_radio->ral ), true );
    if( status != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_sleep() function failed \n" );
        return false;
    }

    return true;
}

/**
 * @brief Test radio irq
 *
 * @remark
 * Test processing:
 * - Reset and init radio
 * - Configure radio irq
 * - Configure radio with bad parameters to receive a rx timeout irq
 * - Configure radio in reception mode with a timeout
 * - Wait
 * - Check if rx timeout irq is raised
 *
 * Ported functions:
 * smtc_modem_hal_irq_config_radio_irq:
 *     hal_gpio_irq_attach
 *
 * lr11xx_hal_write
 *     lr11xx_hal_check_device_ready
 *         lr11xx_hal_wait_on_busy
 *         hal_gpio_get_value
 *     hal_gpio_set_value
 *     hal_spi_in_out
 *     hal_gpio_set_value
 *     hal_mcu_wait_us
 *
 * @return bool True if test is successful
 */
static bool porting_test_radio_irq( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_radio_irq : " );

    bool     ret              = true;
    uint32_t rx_timeout_in_ms = 500;
    radio_irq_raised          = false;

    // Reset, init radio and put it in sleep mode
    ret = reset_init_radio( );
    if( ret == false )
        return ret;

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq( radio_rx_irq_callback, NULL);
    smtc_modem_hal_start_radio_tcxo( );
    if( ralf_setup_lora( modem_radio, &rx_lora_param ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
        return false;
    }
    if( ral_set_dio_irq_params( &( modem_radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                          RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
        return false;
    }
    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio->ral ), rx_timeout_in_ms ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        return false;
    }

    // Wait 2 * timeout
    hal_mcu_wait_us( ( rx_timeout_in_ms * 2 ) * 1000 );

    if( radio_irq_raised == true )
    {
        PORTING_TEST_MSG_OK( );
        smtc_modem_hal_irq_clear_radio_irq();
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timeout, radio irq not received \n" );
        return false;
    }
    return true;
}

/**
 * @brief Test get time in s
 *
 *
 * @remark
 *  Test processing:
 * - Reset, init and configure radio
 * - Configure radio in reception mode with a timeout
 * - Get start time
 * - Wait radio irq (get stop time in irq callback)
 * - Check if time is coherent with the configured timeout radio irq
 * Note: if radio irq received different of rx timeout irq -> relaunch test
 *
 * Ported functions:
 * smtc_modem_hal_get_time_in_s
 *      hal_rtc_get_time_s
 *
 * @return return_code_test_t   RC_PORTING_TEST_OK
 *                              RC_PORTING_TEST_NOK
 *                              RC_PORTING_TEST_RELAUNCH
 */
static return_code_test_t test_get_time_in_s( void )
{
    HAL_DBG_TRACE_MSG( " * Get time in second: " );

    bool     ret              = true;
    uint32_t rx_timeout_in_ms = 5000;

    radio_irq_raised      = false;
    irq_rx_timeout_raised = false;

    rx_lora_param.symb_nb_timeout = 0;

    // Reset, init radio and put it in sleep mode
    ret = reset_init_radio( );
    if( ret == false )
        return RC_PORTING_TEST_NOK;

    //smtc_modem_hal_radio_irq_clear_pending();
    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq( radio_irq_callback_get_time_in_s, NULL );

    smtc_modem_hal_start_radio_tcxo( );
    if( ralf_setup_lora( modem_radio, &rx_lora_param ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }
    if( ral_set_dio_irq_params( &( modem_radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                          RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio->ral ), rx_timeout_in_ms ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }
    uint32_t start_time_s = smtc_modem_hal_get_time_in_s( );

    while( radio_irq_raised == false )
    {
        // Do nothing
    }

    if( irq_rx_timeout_raised == false )
    {
        PORTING_TEST_MSG_WARN( "\n Radio irq received but not RAL_IRQ_RX_TIMEOUT -> relaunched test \n " );
        return RC_PORTING_TEST_RELAUNCH;
    }

    uint32_t time = irq_time_s - start_time_s;
    if( time == ( rx_timeout_in_ms / 1000 ) )
    {
        PORTING_TEST_MSG_OK( );
        smtc_modem_hal_irq_clear_radio_irq();
        HAL_DBG_TRACE_PRINTF( " Time expected %us / get %us (no margin) \n", ( rx_timeout_in_ms / 1000 ), time );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Time is not coherent: expected %us / get %us (no margin) \n",
                              ( rx_timeout_in_ms / 1000 ), time );
        return RC_PORTING_TEST_NOK;
    }

    return RC_PORTING_TEST_OK;
}

/**
 * @brief Test get time in ms
 *
 *
 * @remark
 *  Test processing:
 * - Reset, init and configure radio (with a timeout symbol number)
 * - Get start time
 * - Configure radio in reception mode
 * - Wait radio irq (get stop time in irq callback)
 * - Check if time is coherent with the configured timeout symbol number
 * Note: if radio irq received different of rx timeout irq -> relaunch test
 *
 * Ported functions:
 * smtc_modem_hal_get_time_in_ms
 *      hal_rtc_get_time_ms
 *
 * @return return_code_test_t   RC_PORTING_TEST_OK
 *                              RC_PORTING_TEST_NOK
 *                              RC_PORTING_TEST_RELAUNCH
 */
static return_code_test_t test_get_time_in_ms( void )
{
    HAL_DBG_TRACE_MSG( " * Get time in millisecond: " );

    bool ret              = true;
    radio_irq_raised      = false;
    irq_rx_timeout_raised = false;
    uint8_t wait_start_ms = 5;

    // To avoid misalignment between symb timeout and real timeout for all radio, a number of symbols smaller than 63 is
    // to be used.
    rx_lora_param.symb_nb_timeout = 30;
    rx_lora_param.mod_params.sf   = RAL_LORA_SF12;
    rx_lora_param.mod_params.bw   = RAL_LORA_BW_125_KHZ;

    // Warning: to be updated if previous parameters (SF and BW) are changed
    uint32_t symb_time_ms =
        ( uint32_t )( rx_lora_param.symb_nb_timeout * ( ( 1 << 12 ) / 125.0 ) );  // 2^(SF) / BW * symb_nb_timeout

    // Reset, init radio and put it in sleep mode
    ret = reset_init_radio( );
    if( ret == false )
        return RC_PORTING_TEST_NOK;

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq( radio_rx_irq_callback, NULL );

    smtc_modem_hal_start_radio_tcxo( );
    if( ralf_setup_lora( modem_radio, &rx_lora_param ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }
    if( ral_set_dio_irq_params( &( modem_radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                          RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio->ral ), 0 ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    while( radio_irq_raised == false )
    {
        // Do nothing
    }

    if( irq_rx_timeout_raised == false )
    {
        PORTING_TEST_MSG_WARN( "\n Radio irq received but not RAL_IRQ_RX_TIMEOUT -> relaunched test \n" );
        return RC_PORTING_TEST_RELAUNCH;
    }

    uint32_t time = irq_time_ms - start_time_ms - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    if( abs( time - symb_time_ms ) <= MARGIN_GET_TIME_IN_MS )
    {
        PORTING_TEST_MSG_OK( );
        smtc_modem_hal_irq_clear_radio_irq();
        HAL_DBG_TRACE_PRINTF( " Time expected %ums / get %ums (margin +/-%ums) \n", ( uint32_t ) symb_time_ms, time,
                               MARGIN_GET_TIME_IN_MS );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Time is not coherent with radio irq : expected %ums / get %ums (margin +/-%ums) \n",
                              ( uint32_t ) symb_time_ms, time, MARGIN_GET_TIME_IN_MS );
        smtc_modem_hal_irq_clear_radio_irq();
        return RC_PORTING_TEST_NOK;
    }

    return RC_PORTING_TEST_OK;
}

/**
 * @brief Test time (Get time in s and in ms)
 *
 * @remark See test_get_time_in_s() and test_get_time_in_ms() functions
 *
 * @return bool True if test is successful
 */
static bool porting_test_get_time( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_get_time : \n" );

    return_code_test_t ret = RC_PORTING_TEST_OK;

    do
    {
        ret = test_get_time_in_s( );
        if( ret == RC_PORTING_TEST_NOK )
            return false;
    } while( ret == RC_PORTING_TEST_RELAUNCH );

    do
    {
        ret = test_get_time_in_ms( );
        if( ret == RC_PORTING_TEST_NOK )
            return false;
    } while( ret == RC_PORTING_TEST_RELAUNCH );

    return true;
}

/**
 * @brief Test timer IRQ
 *
 * @warning smtc_modem_hal_start_timer() function takes ~4ms for STM32 (see HAL_LPTIM_TimeOut_Start_IT())
 *
 * @remark
 * Test processing:
 * - Get start time
 * - Configure and start timer
 * - Wait timer irq (get stop time in irq callback)
 * - Check the time elapsed between timer start and timer IRQ reception
 *
 * Ported functions:
 * smtc_modem_hal_start_timer
 *      hal_lp_timer_start
 *
 * @return bool True if test is successful
 */
static bool porting_test_timer_irq( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_timer_irq : " );

    uint32_t timer_ms      = 3000;
    uint8_t  wait_start_ms = 5;
    uint16_t timeout_ms    = 2000;
    timer_irq_raised       = false;

    smtc_modem_hal_stop_timer( );

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;

    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback,
                                NULL );  // Warning this function takes ~3,69 ms for STM32

    // Timeout if irq not raised
    while( ( timer_irq_raised == false ) &&
           ( ( smtc_modem_hal_get_time_in_ms( ) - start_time_ms ) < ( timer_ms + timeout_ms ) ) )
    {
        // Do nothing
    }

    if( timer_irq_raised == false )
    {
        PORTING_TEST_MSG_NOK( " Timeout: timer irq not received \n" );
        return false;
    }

    uint32_t time = irq_time_ms - start_time_ms - COMPENSATION_IN_MS_STM32;
    // to compensate delay introduced by smtc_modem_hal_start_timer for STM32

    if( ( time >= timer_ms ) && ( time <= timer_ms + MARGIN_TIMER_IRQ_IN_MS ) )
    {
        PORTING_TEST_MSG_OK( );
        HAL_DBG_TRACE_PRINTF( " Timer irq configured with %ums / get %ums (margin +%ums) \n", timer_ms, time,
                               MARGIN_TIMER_IRQ_IN_MS );
        //PORTING_TEST_MSG_WARN(
        //    " Note: smtc_modem_hal_start_timer takes ~4ms for STM32 (see HAL_LPTIM_TimeOut_Start_IT) \n" );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timer irq delay is not coherent: expected %ums / get %ums (margin +%ums) \n", timer_ms,
                              time, MARGIN_TIMER_IRQ_IN_MS );
        PORTING_TEST_MSG_WARN(
            " Note: smtc_modem_hal_start_timer takes ~4ms for STM32 (see HAL_LPTIM_TimeOut_Start_IT) \n" );
        return false;
    }
    return true;
}

/**
 * @brief Test stop timer
 *
 * @remark
 * Test processing:
 * - Configure and start timer
 * - Wait
 * - Stop timer
 * - Wait the end of timer
 * - Check if timer IRQ is not received
 *
 * Ported functions:
 * smtc_modem_hal_stop_timer
 *      hal_lp_timer_stop
 *
 * @return bool True if test is successful
 */
static bool porting_test_stop_timer( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_stop_timer : " );

    uint32_t timer_ms = 1000;
    timer_irq_raised  = false;

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback, NULL );

    // Wait half of timer
    uint32_t time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms / 2 ) )
    {
        // Do nothing
    }

    smtc_modem_hal_stop_timer( );

    // Wait a little more than the end of timer
    time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms + 500 ) )
    {
        // Do nothing
    }

    if( timer_irq_raised == false )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timer irq raised while timer is stopped \n" );
        return false;
    }
    return true;
}

/**
 * @brief Test enable/disable irq
 *
 * @remark
 * Test processing:
 * - Disable irq
 * - Start timer with irq
 * - Wait the end of timer
 * - Check if timer irq is not raised
 * - Enable irq
 * - Check if timer irq is raised
 *
 * Ported functions:
 * smtc_modem_hal_disable_modem_irq
 * smtc_modem_hal_enable_modem_irq
 *
 * @return bool True if test is successful
 */
static bool porting_test_disable_enable_irq( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_disable_enable_irq : " );

    uint32_t timer_ms = 3000;
    timer_irq_raised  = false;

    smtc_modem_hal_disable_modem_irq( );

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback, NULL );

    // Wait a little more than the end of timer
    uint32_t time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms + 1000 ) )
    {
        // Do nothing
    }

    if( timer_irq_raised == true )
    {
        PORTING_TEST_MSG_NOK( " Timer irq raised while irq is disabled\n" );
        return false;
    }

    smtc_modem_hal_enable_modem_irq( );

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback, NULL );

    // Wait a little more than the end of timer
    time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms + 1000 ) )
    {
        // Do nothing
    }

    if( timer_irq_raised == true )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timer irq not received while irq is reenabled \n" );
        return false;
    }

    return true;
}

/**
 * @brief Test get random numbers
 *
 * @remark
 * Test processing:
 * 1) - Get 2 random numbers
 * - Check if numbers are not equals to 0 and are different
 * 2) - Get 2 random numbers in a defined range
 * - Check if numbers are different and in the defined range
 * 3) - Get random draw of numbers between in a defined range
 * - Check if draw of each value is equivalent
 *
 * Ported functions:
 * smtc_modem_hal_get_random_nb
 *      hal_rng_get_random
 * smtc_modem_hal_get_random_nb_in_range
 *      hal_rng_get_random_in_range
 *
 * @return bool True if test is successful
 */
static bool porting_test_random( void )
{
    bool ret = true;

    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_random : \n" );

    HAL_DBG_TRACE_MSG( " * Get random nb : " );
    uint32_t rdom1 = smtc_modem_hal_get_random_nb( );
    uint32_t rdom2 = smtc_modem_hal_get_random_nb( );

    if( ( rdom1 != 0 ) && ( rdom2 != 0 ) && ( rdom1 != rdom2 ) )
    {
        PORTING_TEST_MSG_OK( );
        HAL_DBG_TRACE_PRINTF( " random1 = %u, random2 = %u\n", rdom1, rdom2 );
    }
    else
    {
        PORTING_TEST_MSG_WARN( "\n => random1 = %u, random2 = %u\n", rdom1, rdom2 );
        ret = false;
    }

    HAL_DBG_TRACE_MSG( " * Get random nb in range : " );
    uint32_t range_min = 1;
    uint32_t range_max = 42;

    rdom1 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );
    rdom2 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );

    if( ( rdom1 >= range_min ) && ( rdom1 <= range_max ) && ( rdom2 >= range_min ) && ( rdom2 <= range_max ) &&
        ( rdom1 != rdom2 ) )
    {
        PORTING_TEST_MSG_OK( );
        HAL_DBG_TRACE_PRINTF( " random1 = %u, random2 = %u in range [%u;%u]\n", rdom1, rdom2, range_min, range_max );
    }
    else
    {
        PORTING_TEST_MSG_WARN( "\n => random1 = %u, random2 = %u, expected range [%u;%u]\n", rdom1, rdom2, range_min,
                               range_max );
        ret = false;
    }

    HAL_DBG_TRACE_MSG( " * Get random draw : " );
    range_min                       = 1;
    range_max                       = 10;
    uint32_t tab_counter_random[10] = { 0 };
    uint32_t nb_draw                = 20000;
    uint32_t probability_draw       = nb_draw / ( range_max - range_min + 1 );
    int16_t  margin                 = ( probability_draw * 5 ) / 100;  // error margin = 5% of probability_draw

    for( uint16_t i = 0; i < nb_draw; i++ )
    {
        rdom1 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );
        tab_counter_random[rdom1 - 1]++;
    }

    uint8_t tab_size = sizeof( tab_counter_random ) / sizeof( uint32_t );
    for( uint16_t i = 0; i < tab_size; i++ )
    {
        if( abs( probability_draw - tab_counter_random[i] ) > margin )
        {
            PORTING_TEST_MSG_WARN( "\n => The number %u has been drawned %u times, Expected [%u;%u] times \n",
                                   ( i + 1 ), tab_counter_random[i], ( probability_draw - margin ),
                                   ( probability_draw + margin ) );
            ret = false;
        }
    }

    if( ret == true )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_WARN( " Warning smtc_modem_hal_get_random_nb_in_range error margin > 5%% \n" );
    }

    HAL_DBG_TRACE_PRINTF( " Random draw of %u numbers between [%u;%u] range \n", nb_draw, range_min, range_max );

    return ret;
}

/**
 * @brief Test time to configure rx radio
 *
 * @remark
 * Test processing:
 * - Init radio
 * - Configure radio irq
 * - Get start time
 * - Configure rx radio
 * - Get stop time
 * - Check configuration time
 *
 * @return bool True if test is successful
 */
static bool porting_test_config_rx_radio( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_config_rx_radio :" );

    bool ret = true;
    // uint32_t rx_timeout_in_ms = 500;
    uint16_t counter_nok = 0;
    radio_irq_raised     = false;

    // Reset, init and put it in sleep mode radio
    // Setup radio and relative irq
    ret = reset_init_radio( );
    if( ret == false )
        return ret;

    smtc_modem_hal_irq_config_radio_irq( radio_rx_irq_callback, NULL );

    for( uint16_t i = 0; i < NB_LOOP_TEST_CONFIG_RADIO; i++ )
    {
        radio_irq_raised = false;

        uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( );
        // Setup radio and relative irq
        smtc_modem_hal_start_radio_tcxo( );
        if( ralf_setup_lora( modem_radio, &rx_lora_param ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
            return false;
        }
        if( ral_set_dio_irq_params( &( modem_radio->ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                              RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
            return false;
        }

        // Configure radio in reception mode
        // if( ral_set_rx( &( modem_radio->ral ), rx_timeout_in_ms ) !=
        //     RAL_STATUS_OK )
        // {
        //     PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        //     return false;
        // }

        uint32_t time = smtc_modem_hal_get_time_in_ms( ) - start_time_ms;
        if( time >= MARGIN_TIME_CONFIG_RADIO_IN_MS )
        {
            PORTING_TEST_MSG_NOK( " Configuration of rx radio is too long: %ums (margin +%ums) \n", time,
                                  MARGIN_TIME_CONFIG_RADIO_IN_MS );
            counter_nok++;
        }
        // else
        // {
        //     HAL_DBG_TRACE_PRINTF( " Configuration of rx radio is: %ums  \n", time );
        // }

        smtc_modem_hal_stop_radio_tcxo( );
    }

    if( counter_nok == 0 )
    {
        PORTING_TEST_MSG_OK( );
        smtc_modem_hal_irq_clear_radio_irq();
    }
    else
    {
        PORTING_TEST_MSG_WARN( " => Failed test = %u / %u \n", counter_nok, NB_LOOP_TEST_CONFIG_RADIO );
    }

    return true;
}

/**
 * @brief Test time to configure tx radio
 *
 * @remark
 * Test processing:
 * - Init radio
 * - Configure radio irq
 * - Get start time
 * - Configure tx radio
 * - Get stop time
 * - Check configuration time
 *
 * @return bool True if test is successful
 */
static bool porting_test_config_tx_radio( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_config_tx_radio :" );

    uint16_t payload_size = 50;
    uint8_t  payload[50]  = { 0 };
    uint16_t counter_nok  = 0;
    radio_irq_raised      = false;

    // Reset, init and put it in sleep mode radio
    bool ret = reset_init_radio( );
    if( ret == false )
        return ret;

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq( radio_tx_irq_callback, NULL );

    for( uint16_t i = 0; i < NB_LOOP_TEST_CONFIG_RADIO; i++ )
    {
        //radio_irq_raised = false;

        //uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( );

        smtc_modem_hal_start_radio_tcxo( );
        if( ralf_setup_lora( modem_radio, &tx_lora_param ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
            return false;
        }
        if( ral_set_dio_irq_params( &( modem_radio->ral ), RAL_IRQ_TX_DONE ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
            return false;
        }

        if( ral_set_pkt_payload( &( modem_radio->ral ), payload, payload_size ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ral_set_pkt_payload() function failed \n" );
            return false;
        }

        if( ral_set_tx( &( modem_radio->ral ) ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ral_set_tx() function failed \n" );
            return false;
        }

        //uint32_t time = smtc_modem_hal_get_time_in_ms( ) - start_time_ms;
        //if( time >= MARGIN_TIME_CONFIG_RADIO_IN_MS )
        //{
        //    PORTING_TEST_MSG_NOK( " Configuration of tx radio is too long: %ums (margin +%ums) \n", time,
        //                          MARGIN_TIME_CONFIG_RADIO_IN_MS );
        //    counter_nok++;
        //}
        // else
        // {
        //     HAL_DBG_TRACE_PRINTF( " Configuration of tx radio is: %ums  \n", time );
        // }

        smtc_modem_hal_stop_radio_tcxo( );
    }
#if 0
    if( counter_nok == 0 )
    {
        PORTING_TEST_MSG_OK( );
        smtc_modem_hal_irq_clear_radio_irq();
    }
    else
    {
        PORTING_TEST_MSG_WARN( " => Failed test = %u / %u \n", counter_nok, NB_LOOP_TEST_CONFIG_RADIO );
    }
#endif
    // Wait 2 * timeout
    do
    {

    } while( radio_irq_raised == false );

    if( radio_irq_raised == true )
    {
        PORTING_TEST_MSG_OK( );
        smtc_modem_hal_irq_clear_radio_irq();
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timeout, radio irq not received \n" );
        return false;
    }
    return true;
}

/**
 * @brief Test sleep time
 *
 * @remark
 * Test processing:
 * - Get start time
 * - Set sleep for ms
 * - Get stop time
 * - Check sleep time
 *
 * Ported functions:
 * hal_mcu_set_sleep_for_ms
 *      hal_watchdog_reload
 *      hal_rtc_wakeup_timer_set_ms
 *      lpm_handler
 *      hal_rtc_wakeup_timer_stop
 *
 * @return bool True if test is successful
 */
static bool porting_test_sleep_ms( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_sleep_ms :" );

    bool    ret           = true;
    int32_t sleep_ms      = 2000;
    uint8_t wait_start_ms = 5;

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    hal_mcu_set_sleep_for_ms( sleep_ms );

    uint32_t stop_time_ms = smtc_modem_hal_get_time_in_ms( );
    uint32_t time         = stop_time_ms - start_time_ms;

    if( abs( time - sleep_ms ) <= MARGIN_SLEEP_IN_MS )
    {
        PORTING_TEST_MSG_OK( );
        HAL_DBG_TRACE_PRINTF( " Sleep time expected %ums / get %ums (margin +/-%ums) \n", sleep_ms, time,
                               MARGIN_SLEEP_IN_MS );
    }
    else
    {
        PORTING_TEST_MSG_WARN( "\n => Sleep time is not coherent: expected %ums / get %ums (margin +/-%ums) \n",
                               sleep_ms, time, MARGIN_SLEEP_IN_MS );
    }
    return ret;
}

/**
 * @brief Start the timer and wait in low power then show the wake up time
 *
 * @warning smtc_modem_hal_start_timer() function takes ~4ms for STM32 (see HAL_LPTIM_TimeOut_Start_IT())
 *
 * @remark
 * Test processing:
 * - Get start time
 * - Configure and start timer
 * - Wait timer irq
 * - Get stop time
 * - Check the time elapsed between timer start and timer IRQ reception
 *
 * Ported functions:
 * smtc_modem_hal_start_timer
 *      hal_lp_timer_start
 *
 */
static void porting_test_timer_irq_low_power_wake( void )
{
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_timer_irq_low_power_wake : " );

    uint32_t timer_ms      = 3000;
    int32_t  sleep_ms      = timer_ms + 5000;
    uint8_t  wait_start_ms = 5;
    timer_irq_raised       = false;

    smtc_modem_hal_stop_timer( );

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback,
                                NULL );  // Warning this function takes ~3,69 ms for STM32L4 and L0

    hal_mcu_set_sleep_for_ms( sleep_ms );

    uint32_t time = irq_time_ms - start_time_ms - COMPENSATION_IN_MS_STM32;
    // to compensate delay introduced by smtc_modem_hal_start_timer for STM32

    HAL_DBG_TRACE_PRINTF( " Timer irq configured with %ums / get %ums \n", timer_ms, time );
    HAL_DBG_TRACE_PRINTF( " Conclusion: Wake up delay after low power is %u ms \n", time - timer_ms );
}

/*
 * -----------------------------------------------------------------------------
 * --- FLASH PORTING TESTS -----------------------------------------------------
 */

/**
 * @brief Test read/write context in flash
 *
 * @remark
 * Test processing:
 * - Read context in flash
 * - Write a different context in flash
 * - Read context in flash
 * - Check if read context is equal to written context
 *
 * Ported functions:
 * smtc_modem_hal_context_restore
 *     hal_flash_read_buffer
 * smtc_modem_hal_context_store
 *     hal_flash_write_buffer
 *
 * @param [in]  context_type   The context type
 *
 * @return bool True if test is successful
 */
static bool test_context_store_restore( modem_context_type_t context_type )
{
    bool    ret             = true;
    uint8_t read_buffer[8]  = { 0 };
    uint8_t write_buffer[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    bool    cmp             = true;

    HAL_DBG_TRACE_PRINTF( "\n * Context %s : \n", name_context_type[context_type] );

    smtc_modem_hal_context_restore( context_type, read_buffer, sizeof( read_buffer ) );

    HAL_DBG_TRACE_MSG( " Read:  { " );
    for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%u", read_buffer[i] );
        if( i != ( sizeof( read_buffer ) - 1 ) )
            HAL_DBG_TRACE_MSG( ", " );
    }
    HAL_DBG_TRACE_MSG( " }\n" );

    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        if( read_buffer[i] == write_buffer[i] )
        {
            write_buffer[i] = ( read_buffer[i] + 1 ) % 256;
        }
    }

    HAL_DBG_TRACE_MSG( " Write: { " );
    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%u", write_buffer[i] );
        if( i != ( sizeof( write_buffer ) - 1 ) )
            HAL_DBG_TRACE_MSG( ", " );
    }
    HAL_DBG_TRACE_MSG( " }\n" );

    smtc_modem_hal_context_store( context_type, write_buffer, sizeof( write_buffer ) );

    memset( read_buffer, 0, sizeof( read_buffer ) );
    smtc_modem_hal_context_restore( context_type, read_buffer, sizeof( read_buffer ) );

    HAL_DBG_TRACE_MSG( " Read:  { " );
    for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%u", read_buffer[i] );
        if( i != ( sizeof( read_buffer ) - 1 ) )
            HAL_DBG_TRACE_MSG( ", " );
    }
    HAL_DBG_TRACE_MSG( " }\n" );

    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        if( read_buffer[i] != write_buffer[i] )
        {
            cmp = false;
        }
    }
    if( cmp == true )
    {
        HAL_DBG_TRACE_MSG( " Store/restore without MCU reset :" );
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Store or restore context failed (without MCU reset) \n\n" );
        return false;
    }

    return ret;
}

/**
 * @brief Test read/write context in flash
 *
 * @remark
 * Test processing:
 * - See test_context_store_restore() function
 * - Reset MCU
 * - RELAUNCH this test after mcu reset
 * - Check if read after reset = write before reset
 *
 * @return bool True if test is successful
 */
static bool porting_test_flash( void )
{
    bool ret = true;
    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_flash : \n" );
    HAL_DBG_TRACE_MSG_COLOR( " !! TEST TO BE LAUNCH TWICE !! To check writing after MCU reset \n",
                              HAL_DBG_TRACE_COLOR_BLUE );

    /* LR1MAC */
    ret = test_context_store_restore( CONTEXT_LR1MAC );
    if( ret == false )
        return ret;

    /* MODEM */
    ret = test_context_store_restore( CONTEXT_MODEM );
    if( ret == false )
        return ret;

    /* SECURE ELEMENT */
    ret = test_context_store_restore( CONTEXT_SECURE_ELEMENT );
    if( ret == false )
        return ret;

    /* DEVNONCE */
    ret = test_context_store_restore( CONTEXT_DEVNONCE );
    if( ret == false )
        return ret;

    return ret;
}

/**
 * @brief Test read/write crashlog data
 *
 * @remark
 * Test processing:
 * - Read crashlog data
 * - Write different crashlog data
 * - Read crashlog data
 * - Check if crashlog data is equal to written crashlog data
 *
 * Ported functions:
 * smtc_modem_hal_restore_crashlog
 * smtc_modem_hal_store_crashlog
 *
 * @return bool True if test is successful
 */
static bool test_crashlog_store_restore( void )
{
    bool    ret                            = true;
    uint8_t read_crashlog[CRASH_LOG_SIZE]  = { 0 };
    uint8_t write_crashlog[CRASH_LOG_SIZE] = { 0 };
    bool    cmp                            = true;

    /* Store/Restore crashlog */
    HAL_DBG_TRACE_MSG( "\n * Store/Restore crashlog : \n" );
    smtc_modem_hal_restore_crashlog( read_crashlog );

    HAL_DBG_TRACE_MSG( " Read:  { " );
    for( uint8_t i = 0; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%u", read_crashlog[i] );
        if( i != 7 )
            HAL_DBG_TRACE_MSG( ", " );
    }
    HAL_DBG_TRACE_MSG( " }\n" );

    for( uint8_t i = 0; i < 8; i++ )
    {
        write_crashlog[i] = i + 1;
        if( read_crashlog[i] == write_crashlog[i] )
        {
            write_crashlog[i] = ( read_crashlog[i] + 1 ) % 256;
        }
    }

    HAL_DBG_TRACE_MSG( " Write: { " );
    for( uint8_t i = 0; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%u", write_crashlog[i] );
        if( i != 7 )
            HAL_DBG_TRACE_MSG( ", " );
    }
    HAL_DBG_TRACE_MSG( " }\n" );

    smtc_modem_hal_store_crashlog( write_crashlog );

    memset( read_crashlog, 0, sizeof( read_crashlog ) );
    smtc_modem_hal_restore_crashlog( read_crashlog );

    HAL_DBG_TRACE_MSG( " Read:  { " );
    for( uint8_t i = 0; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%u", read_crashlog[i] );
        if( i != 7 )
            HAL_DBG_TRACE_MSG( ", " );
    }
    HAL_DBG_TRACE_MSG( " }\n" );

    for( uint8_t i = 0; i < sizeof( write_crashlog ); i++ )
    {
        if( read_crashlog[i] != write_crashlog[i] )
        {
            cmp = false;
        }
    }
    if( cmp == true )
    {
        HAL_DBG_TRACE_MSG( " Store/restore crashlog without MCU reset :" );
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Store or restore crashlog failed \n\n" );
        ret = false;
    }

    return ret;
}

/**
 * @brief Test read/write crashlog status
 *
 * @remark
 * Test processing:
 * - Read crashlog status
 * - Write different crashlog status
 * - Read crashlog status
 * - Check if crashlog status is equal to writen crashlog status
 *
 * Ported functions:
 * smtc_modem_hal_get_crashlog_status
 * smtc_modem_hal_set_crashlog_status
 *
 * @return bool True if test is successful
 */
static bool test_crashlog_status( void )
{
    bool ret = true;
    bool read_status;
    bool write_status;
    HAL_DBG_TRACE_MSG( "\n * Get/Set crashlog status : \n" );

    read_status = smtc_modem_hal_get_crashlog_status( );
    HAL_DBG_TRACE_PRINTF( " Get: %u \n", read_status );

    write_status = !read_status;
    HAL_DBG_TRACE_PRINTF( " Set: %u \n", write_status );
    smtc_modem_hal_set_crashlog_status( write_status );

    read_status = smtc_modem_hal_get_crashlog_status( );
    HAL_DBG_TRACE_PRINTF( " Get: %u \n", read_status );

    if( write_status == read_status )
    {
        HAL_DBG_TRACE_MSG( " Set/get crashlog status without MCU reset :" );
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Set or get crashlog status failed \n\n" );
        ret = false;
    }

    return ret;
}

/**
 * @brief Test read/write crashlog data and status
 *
 * @remark
 * Test processing:
 * - See test_crashlog_store_restore() and test_crashlog_status() functions
 * - Reset MCU
 * - RELAUNCH this test after mcu reset
 * - Check if read after reset = write before reset
 *
 * @return bool True if test is successful
 */
static bool porting_test_crashlog( void )
{
    bool ret = true;

    HAL_DBG_TRACE_MSG( "----------------------------------------\n porting_test_crashlog : \n" );
    HAL_DBG_TRACE_MSG_COLOR( " !! TEST TO BE LAUNCH TWICE !! To check writing after MCU reset \n",
                              HAL_DBG_TRACE_COLOR_BLUE );

    /* Store/Restore crashlog */
    ret = test_crashlog_store_restore( );
    if( ret == false )
        return ret;

    /* Get/Set crashlog status */
    ret = test_crashlog_status( );
    if( ret == false )
        return ret;

    return ret;
}

/*
 * -----------------------------------------------------------------------------
 * --- IRQ CALLBACK DEFINITIONS ---------------------------------------------------------
 */

/**
 * @brief Radio tx irq callback
 */
static void radio_tx_irq_callback( void* obj )
{
    HAL_DBG_TRACE_MSG( "HD: radio_tx_irq_callback \n" );
    UNUSED( obj );

    ral_irq_t radio_irq = 0;
    irq_time_ms = smtc_modem_hal_get_time_in_ms( );
    radio_irq_raised = true;

    if( ral_get_irq_status( &( modem_radio->ral ), &radio_irq ) != RAL_STATUS_OK )
    {
        HAL_DBG_TRACE_MSG_COLOR( "NOK\n ral_get_irq_status() function failed \n", HAL_DBG_TRACE_COLOR_RED );
    }
    HAL_DBG_TRACE_PRINTF( " RP: IRQ source - 0x%04x\n", radio_irq );

    if( ( radio_irq & RAL_IRQ_TX_DONE ) == RAL_IRQ_TX_DONE )
    {
        HAL_DBG_TRACE_PRINTF( " HD: Receive RAL_IRQ_TX_DONE.\n");
    }

    if( ral_clear_irq_status( &( modem_radio->ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_clear_irq_status() function failed \n" );
    }
}

/**
 * @brief Radio rx irq callback (get time in ms)
 */
static void radio_rx_irq_callback( void* obj )
{
    HAL_DBG_TRACE_MSG( "HD: radio_rx_irq_callback \n" );

    UNUSED( obj );

    ral_irq_t radio_irq = 0;
    irq_time_ms         = smtc_modem_hal_get_time_in_ms( );
    radio_irq_raised    = true;

    if( ral_get_irq_status( &( modem_radio->ral ), &radio_irq ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( "ral_get_irq_status() function failed \n" );
    }
    HAL_DBG_TRACE_PRINTF( " RP: IRQ source - 0x%04x\n", radio_irq );

    if( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT )
    {
        irq_rx_timeout_raised = true;
        HAL_DBG_TRACE_PRINTF( " HD: Receive RAL_IRQ_RX_TIMEOUT.\n");
    }

    if( ral_clear_irq_status( &( modem_radio->ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_clear_irq_status() function failed \n" );
    }

    // Shut Down the TCXO
    smtc_modem_hal_stop_radio_tcxo( );
}

/**
 * @brief Radio irq callback (get time in s)
 */
static void radio_irq_callback_get_time_in_s( void* obj )
{
    HAL_DBG_TRACE_MSG( "HD: radio_irq_callback_get_time_in_s \n" );
    UNUSED( obj );
    ral_irq_t radio_irq = 0;
    irq_time_s          = smtc_modem_hal_get_time_in_s( );
    radio_irq_raised    = true;

    if( ral_get_irq_status( &( modem_radio->ral ), &radio_irq ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_get_irq_status() function failed \n" );
    }

    if( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT )
    {
        irq_rx_timeout_raised = true;
    }

    if( ral_clear_irq_status( &( modem_radio->ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_clear_irq_status() function failed \n" );
    }

    // Shut Down the TCXO
    smtc_modem_hal_stop_radio_tcxo( );
}

/**
 * @brief Timer irq callback
 */
static void timer_irq_callback( void* obj )
{
    UNUSED( obj );
    irq_time_ms      = smtc_modem_hal_get_time_in_ms( );
    timer_irq_raised = true;
}
#endif

static void on_modem_reset( uint16_t reset_count )
{
    apps_modem_common_configure_lorawan_params( stack_id );

    /* Configure modem DM status for regular almanac status update */
    smtc_modem_dm_info_interval_format_t format   = SMTC_MODEM_DM_INFO_INTERVAL_IN_DAY;
    uint8_t                              interval = 1;
    ASSERT_SMTC_MODEM_RC( smtc_modem_dm_set_info_interval( format, interval ) );

    /* Active almanac update OTA - WARNING: will remove all other DM message */
    uint8_t info_field = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
    ASSERT_SMTC_MODEM_RC( smtc_modem_dm_set_info_fields( &info_field, 1 ) );

    /* Start the Join process */
    ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( stack_id ) );

    HAL_DBG_TRACE_INFO( "###### ===== JOINING ==== ######\n\n" );

    /* Notify user with leds */
    smtc_board_led_set( smtc_board_get_led_tx_mask( ), true );
}

static void on_modem_network_joined( void )
{
    /* Notify user with leds */
    smtc_board_led_set( smtc_board_get_led_tx_mask( ), false );
    smtc_board_led_set( smtc_board_get_led_rx_mask( ), true );

    /* Start time sync (ALC sync), necessary for GNSS scan:
    The interval_s indicates how often the LBM will request a time sync from the DAS.
    If no time sync downlink has been received from the DAS after the invalid_delay_s is elapsed,
    the LBM will report SMTC_MODEM_RC_NO_TIME on smtc_modem_get_time() call. */
    /* -- */
    ASSERT_SMTC_MODEM_RC( smtc_modem_time_set_sync_interval_s( APP_ALC_TIMING_INTERVAL ) );     /* keep call order */
    ASSERT_SMTC_MODEM_RC( smtc_modem_time_set_sync_invalid_delay_s( APP_ALC_TIMING_INVALID ) ); /* keep call order */
    /* Start the service */
    ASSERT_SMTC_MODEM_RC( smtc_modem_time_start_sync_service( stack_id, SMTC_MODEM_TIME_ALC_SYNC ) );
}

static void on_modem_clk_synch( smtc_modem_event_time_status_t time_status )
{
    mw_return_code_t gnss_rc;
    mw_version_t     mw_version;

    if( time_status != SMTC_MODEM_EVENT_TIME_NOT_VALID )
    {
        /* Notify user with leds (ready for GNSS scan) */
        smtc_board_led_pulse( smtc_board_get_led_rx_mask( ), true, LED_PERIOD_MS );

        if( is_first_time_sync == true )
        {
            /* Set the custom ADR profile for geolocation scan & send */
            configure_adr( );

            /* Initialize GNSS middleware */
            gnss_mw_get_version( &mw_version );
            HAL_DBG_TRACE_INFO( "Initializing GNSS middleware v%d.%d.%d\n", mw_version.major, mw_version.minor,
                                mw_version.patch );
            gnss_mw_init( modem_radio, stack_id );
            gnss_mw_set_constellations( GNSS_MW_CONSTELLATION_GPS_BEIDOU );

            /* Initialize WIFI middleware */
            wifi_mw_get_version( &mw_version );
            HAL_DBG_TRACE_INFO( "Initializing Wi-Fi middleware v%d.%d.%d\n", mw_version.major, mw_version.minor,
                                mw_version.patch );
            wifi_mw_init( modem_radio, stack_id );

            /* Set user defined assistance position */
#if MODEM_EXAMPLE_ASSISTANCE_POSITION_AUTO == false
            const char* assistance_position_text = MODEM_EXAMPLE_ASSISTANCE_POSITION_TEXT;
            gnss_mw_set_user_aiding_position( MODEM_EXAMPLE_ASSISTANCE_POSITION_LAT,
                                              MODEM_EXAMPLE_ASSISTANCE_POSITION_LONG );
            HAL_DBG_TRACE_WARNING( "User defined assistance position has been set in %s\n", assistance_position_text );
#endif

            /* Start the GNSS scan group sequence */
            gnss_rc = gnss_mw_scan_start( GNSS_SCAN_MODE, 0 ); /* start ASAP */
            if( gnss_rc != MW_RC_OK )
            {
                HAL_DBG_TRACE_ERROR( "Failed to start GNSS scan\n" );
            }
        }

        is_first_time_sync = false;
    }
}

static void on_modem_down_data( int8_t rssi, int8_t snr, smtc_modem_event_downdata_window_t rx_window, uint8_t port,
                                const uint8_t* payload, uint8_t size )
{
    HAL_DBG_TRACE_INFO( "Downlink received:\n" );
    HAL_DBG_TRACE_INFO( "  - LoRaWAN Fport = %d\n", port );
    HAL_DBG_TRACE_INFO( "  - Payload size  = %d\n", size );
    HAL_DBG_TRACE_INFO( "  - RSSI          = %d dBm\n", rssi - 64 );
    HAL_DBG_TRACE_INFO( "  - SNR           = %d dB\n", snr >> 2 );

    if( size != 0 )
    {
        HAL_DBG_TRACE_ARRAY( "Payload", payload, size );

        /* Forward downlink to GNSS middleware to handle it if necessary */
        gnss_mw_handle_downlink( port, payload, size );
    }
}

static void on_modem_almanac_update( smtc_modem_event_almanac_update_status_t status )
{
    if( status == SMTC_MODEM_EVENT_ALMANAC_UPDATE_STATUS_REQUESTED )
    {
        HAL_DBG_TRACE_INFO( "Almanac update is not completed yet\n" );
#if 0
        uint8_t dm_almanac_status = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
        ASSERT_SMTC_MODEM_RC( smtc_modem_dm_request_single_uplink( &dm_almanac_status, 1 ) );
#endif
    }
    else
    {
        HAL_DBG_TRACE_INFO( "Almanac update is completed\n" );
    }
}

static void on_middleware_gnss_event( uint8_t pending_events )
{
    mw_return_code_t wifi_rc;

    /* Parse events */
    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_SCAN_DONE ) )
    {
        gnss_mw_event_data_scan_done_t event_data;

        HAL_DBG_TRACE_INFO( "GNSS middleware event - SCAN DONE\n" );
        gnss_mw_get_event_data_scan_done( &event_data );
        gnss_mw_display_results( &event_data );

        if( event_data.context.almanac_update_required )
        {
            uint8_t dm_almanac_status = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
            ASSERT_SMTC_MODEM_RC( smtc_modem_dm_request_single_uplink( &dm_almanac_status, 1 ) );
        }
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_TERMINATED ) )
    {
        gnss_mw_event_data_terminated_t event_data;

        /* Notify user with leds */
        smtc_board_led_pulse( smtc_board_get_led_tx_mask( ), true, LED_PERIOD_MS );

        HAL_DBG_TRACE_INFO( "GNSS middleware event - TERMINATED\n" );
        gnss_mw_get_event_data_terminated( &event_data );
        HAL_DBG_TRACE_PRINTF( "TERMINATED info:\n" );
        HAL_DBG_TRACE_PRINTF( "-- number of scans sent: %u\n", event_data.nb_scans_sent );
        HAL_DBG_TRACE_PRINTF( "-- aiding position check sent: %d\n", event_data.aiding_position_check_sent );
        HAL_DBG_TRACE_PRINTF( "-- indoor detected: %d\n", event_data.indoor_detected );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_SCAN_CANCELLED ) )
    {
        HAL_DBG_TRACE_INFO( "GNSS middleware event - SCAN CANCELLED\n" );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_TIME ) )
    {
        HAL_DBG_TRACE_ERROR( "GNSS middleware event - ERROR NO TIME\n" );
        ASSERT_SMTC_MODEM_RC( smtc_modem_time_trigger_sync_request( stack_id ) );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE ) )
    {
        HAL_DBG_TRACE_ERROR( "GNSS middleware event - ALMANAC UPDATE REQUIRED\n" );
        uint8_t dm_almanac_status = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
        ASSERT_SMTC_MODEM_RC( smtc_modem_dm_request_single_uplink( &dm_almanac_status, 1 ) );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_AIDING_POSITION ) )
    {
        HAL_DBG_TRACE_ERROR( "GNSS middleware event - ERROR NO AIDING POSITION set\n" );

/* Set user defined assistance position */
#if MODEM_EXAMPLE_ASSISTANCE_POSITION_AUTO == false
        const char* assistance_position_text = MODEM_EXAMPLE_ASSISTANCE_POSITION_TEXT;
        gnss_mw_set_user_aiding_position( MODEM_EXAMPLE_ASSISTANCE_POSITION_LAT,
                                          MODEM_EXAMPLE_ASSISTANCE_POSITION_LONG );
        HAL_DBG_TRACE_WARNING( "User defined assistance position has been set in %s\n", assistance_position_text );
#endif
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_UNKNOWN ) )
    {
        HAL_DBG_TRACE_ERROR( "GNSS middleware event - UNEXPECTED ERROR\n" );
    }

    /* Start Wifi scan */
    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_TERMINATED ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_TIME ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_AIDING_POSITION ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_UNKNOWN ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_SCAN_CANCELLED ) )
    {
        wifi_rc = wifi_mw_scan_start( 0 );
        if( wifi_rc != MW_RC_OK )
        {
            HAL_DBG_TRACE_ERROR( "Failed to start WiFi scan\n" );
        }
    }

    gnss_mw_clear_pending_events( );
}

static void on_middleware_wifi_event( uint8_t pending_events )
{
    mw_return_code_t gnss_rc;

    /* Parse events */
    if( wifi_mw_has_event( pending_events, WIFI_MW_EVENT_SCAN_DONE ) )
    {
        HAL_DBG_TRACE_INFO( "Wi-Fi middleware event - SCAN DONE\n" );
        wifi_mw_get_event_data_scan_done( &wifi_results );
        wifi_mw_display_results( &wifi_results );
    }

    if( wifi_mw_has_event( pending_events, WIFI_MW_EVENT_TERMINATED ) )
    {
        wifi_mw_event_data_terminated_t event_data;

        /* Notify user with leds */
        smtc_board_led_pulse( smtc_board_get_led_tx_mask( ), true, LED_PERIOD_MS );

        HAL_DBG_TRACE_INFO( "Wi-Fi middleware event - TERMINATED\n" );
        wifi_mw_get_event_data_terminated( &event_data );
        HAL_DBG_TRACE_PRINTF( "TERMINATED info:\n" );
        HAL_DBG_TRACE_PRINTF( "-- number of scans sent: %u\n", event_data.nb_scans_sent );
    }

    if( wifi_mw_has_event( pending_events, WIFI_MW_EVENT_SCAN_CANCELLED ) )
    {
        HAL_DBG_TRACE_INFO( "Wi-Fi middleware event - SCAN CANCELLED\n" );
    }

    if( wifi_mw_has_event( pending_events, WIFI_MW_EVENT_ERROR_UNKNOWN ) )
    {
        HAL_DBG_TRACE_INFO( "Wi-Fi middleware event - UNEXPECTED ERROR\n" );
    }

    /* Program next GNSS scan */
    if( wifi_mw_has_event( pending_events, WIFI_MW_EVENT_TERMINATED ) ||
        wifi_mw_has_event( pending_events, WIFI_MW_EVENT_ERROR_UNKNOWN ) ||
        wifi_mw_has_event( pending_events, WIFI_MW_EVENT_SCAN_CANCELLED ) )
    {
        gnss_rc = gnss_mw_scan_start( GNSS_SCAN_MODE, GEOLOCATION_SCAN_PERIOD );
        if( gnss_rc != MW_RC_OK )
        {
            HAL_DBG_TRACE_ERROR( "Failed to start GNSS scan\n" );
        }
    }

    wifi_mw_clear_pending_events( );
}

void configure_adr( void )
{
    smtc_modem_region_t region;

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_region( stack_id, &region ) );

    /* Set the ADR profile once joined */
    switch( region )
    {
    case SMTC_MODEM_REGION_EU_868:
    case SMTC_MODEM_REGION_IN_865:
    case SMTC_MODEM_REGION_RU_864:
    case SMTC_MODEM_REGION_AU_915:
    case SMTC_MODEM_REGION_AS_923_GRP1:
    case SMTC_MODEM_REGION_AS_923_GRP2:
    case SMTC_MODEM_REGION_AS_923_GRP3:
    case SMTC_MODEM_REGION_CN_470:
    case SMTC_MODEM_REGION_CN_470_RP_1_0:
    case SMTC_MODEM_REGION_KR_920:
        ASSERT_SMTC_MODEM_RC(
            smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_dr5_dr3 ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, custom_nb_trans_dr5_dr3 ) );
        break;
    case SMTC_MODEM_REGION_US_915:
        ASSERT_SMTC_MODEM_RC(
            smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_us915 ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, custom_nb_trans_us915 ) );
        break;
    case SMTC_MODEM_REGION_WW2G4:
        ASSERT_SMTC_MODEM_RC(
            smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_ww2g4 ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, custom_nb_trans_ww2g4 ) );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Region not supported in this example, could not set custom ADR profile\n" );
        break;
    }

    /* Disable auto switch to network controlled after a certain amount of TX without RX */
    ASSERT_SMTC_MODEM_RC( smtc_modem_connection_timeout_set_thresholds( stack_id, 0, 0 ) );
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
