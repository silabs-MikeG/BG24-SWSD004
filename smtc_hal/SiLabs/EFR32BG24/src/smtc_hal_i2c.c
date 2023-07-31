/*!
 * @file      smtc_hal_i2c.c
 *
 * @brief     Implements the i2c HAL functions
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

#include "smtc_hal.h"
#include "sl_i2cspm.h"
//#include "sl_i2cspm_i2cdriver_config.h"
#include "em_gpio.h"
#include "smtc_hal_gpio.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

//typedef struct hal_i2c_s
//{
//    I2C_TypeDef*      interface;
//    I2C_HandleTypeDef handle;
//    struct
//    {
//        hal_gpio_pin_names_t sda;
//        hal_gpio_pin_names_t scl;
//    } pins;
//} hal_i2c_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
sl_i2cspm_t *sl_i2cspm_i2cdriver = I2C0;

#if SL_I2CSPM_I2CDRIVER_SPEED_MODE == 0
#define SL_I2CSPM_I2CDRIVER_HLR i2cClockHLRStandard
#define SL_I2CSPM_I2CDRIVER_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_I2CDRIVER_SPEED_MODE == 1
#define SL_I2CSPM_I2CDRIVER_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_I2CDRIVER_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_I2CDRIVER_SPEED_MODE == 2
#define SL_I2CSPM_I2CDRIVER_HLR i2cClockHLRFast
#define SL_I2CSPM_I2CDRIVER_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif


static i2c_addr_size i2c_internal_addr_size;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Write data buffer to the I2C device
 *
 * @param [in] id               I2C interface id [1:N]
 * @param [in] deviceAddr       device address
 * @param [in] addr             data address
 * @param [in] buffer           data buffer to write
 * @param [in] size             number of data bytes to write
 *
 * @returns status [SUCCESS = 1, FAIL = 0]
 */
static uint8_t i2c_write_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer,
                                 uint16_t size );

/*!
 * @brief Write data buffer to the I2C device
 *
 * @param [in] id               I2C interface id [1:N]
 * @param [in] deviceAddr       device address
 * @param [in] addr             data address
 * @param [in] buffer           data buffer to write
 * @param [in] size             number of data bytes to write
 *
 * @returns status [SUCCESS = 1, FAIL = 0]
 */
static uint8_t i2c_read_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_i2c_init( const uint32_t id, const hal_gpio_pin_names_t sda, const hal_gpio_pin_names_t scl )
{
  I2CSPM_Init_TypeDef init_i2cdriver = {
    .port = I2C0,
    .sclPort = 0,
    .sclPin = 0,
    .sdaPort = 0,
    .sdaPin = 0,
    .i2cRefFreq = 0,
    .i2cMaxFreq = SL_I2CSPM_I2CDRIVER_MAX_FREQ,
    .i2cClhr = SL_I2CSPM_I2CDRIVER_HLR
  };


  init_i2cdriver.sclPort = hal_get_gpio_port(scl);
  init_i2cdriver.sclPin = hal_get_gpio_pin_num(scl);

  init_i2cdriver.sdaPort = hal_get_gpio_port(sda);
  init_i2cdriver.sdaPin = hal_get_gpio_pin_num(sda);

  I2CSPM_Init(&init_i2cdriver);
}

void hal_i2c_deinit( const uint32_t id )
{
//    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_i2c ) ) );
//    uint32_t local_id = id - 1;
//
//    HAL_I2C_DeInit( &hal_i2c[local_id].handle );
}


uint8_t hal_i2c_write( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t data )
{
    if( i2c_write_buffer( id, device_addr, addr, &data, 1u ) == SMTC_HAL_FAILURE )
    {
        // if first attempt fails due to an IRQ, try a second time
        if( i2c_write_buffer( id, device_addr, addr, &data, 1u ) == SMTC_HAL_FAILURE )
        {
            return SMTC_HAL_FAILURE;
        }
        else
        {
            return SMTC_HAL_SUCCESS;
        }
    }
    else
    {
        return SMTC_HAL_SUCCESS;
    }
}

uint8_t hal_i2c_write_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    if( i2c_write_buffer( id, device_addr, addr, buffer, size ) == SMTC_HAL_FAILURE )
    {
        // if first attempt fails due to an IRQ, try a second time
        if( i2c_write_buffer( id, device_addr, addr, buffer, size ) == SMTC_HAL_FAILURE )
        {
            return SMTC_HAL_FAILURE;
        }
        else
        {
            return SMTC_HAL_SUCCESS;
        }
    }
    else
    {
        return SMTC_HAL_SUCCESS;
    }
}

uint8_t hal_i2c_read( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* data )
{
    return ( i2c_read_buffer( id, device_addr, addr, data, 1 ) );
}

uint8_t hal_i2c_read_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    return ( i2c_read_buffer( id, device_addr, addr, buffer, size ) );
}

void i2c_set_addr_size( i2c_addr_size addr_size ) { i2c_internal_addr_size = addr_size; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint8_t i2c_write_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    uint8_t  write_status = SMTC_HAL_FAILURE;
    uint16_t mem_add_size = 0u;
    I2C_TransferSeq_TypeDef write_sequence;
//    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_i2c ) ) );
    uint32_t local_id = id - 1;
    write_sequence.addr = device_addr << 1;
    if( i2c_internal_addr_size == I2C_ADDR_SIZE_8 )
    {
        write_sequence.buf[0].data = addr;
        write_sequence.buf[1].data = buffer;
    }
    else
    {
        write_sequence.buf[0].data = addr;
        write_sequence.buf[1].data = (uint8_t)(addr>>8);
        write_sequence.buf[2].data = buffer;
    }

    write_sequence.flags = I2C_FLAG_WRITE;
    write_sequence.buf[0].len = size;
    I2CSPM_Transfer (sl_i2cspm_i2cdriver, &write_sequence);
    return write_status;
}

static uint8_t i2c_read_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    uint8_t  read_status  = SMTC_HAL_FAILURE;
    uint16_t mem_add_size = 0u;
    I2C_TransferSeq_TypeDef read_sequence;


    read_sequence.addr = device_addr << 1;
    if( i2c_internal_addr_size == I2C_ADDR_SIZE_8 )
    {
        read_sequence.buf[0].data = addr;
        read_sequence.buf[1].data = buffer;
    }
    else
    {
        read_sequence.buf[0].data = addr;
        read_sequence.buf[1].data = (uint8_t)(addr>>8);
        read_sequence.buf[2].data = buffer;
    }

    read_sequence.flags = I2C_FLAG_READ;
    read_sequence.buf[0].data = buffer;
    read_sequence.buf[0].len = size;
    I2CSPM_Transfer (sl_i2cspm_i2cdriver, &read_sequence);

    return read_status;
}
