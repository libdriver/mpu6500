/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_mpu6500_register_test.c
 * @brief     driver mpu6500 register test source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2024-07-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2024/07/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_mpu6500_register_test.h"
#include <stdlib.h>

static mpu6500_handle_t gs_handle;        /**< mpu6500 handle */

/**
 * @brief     register test
 * @param[in] interface used interface
 * @param[in] addr iic device address
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      none
 */
uint8_t mpu6500_register_test(mpu6500_interface_t interface, mpu6500_address_t addr)
{
    uint8_t res;
    uint8_t reg;
    uint8_t i;
    uint8_t div_in;
    uint8_t div_check;
    uint8_t data[8];
    uint8_t data_check[8];
    uint16_t cnt;
    uint8_t status;
    uint8_t test;
    uint8_t test_check;
    int16_t offset;
    int16_t offset_check;
    float test_f;
    float test_check_f;
    mpu6500_info_t info;
    mpu6500_address_t addr_pin;
    mpu6500_interface_t interface_check;
    mpu6500_bool_t enable;
    mpu6500_clock_source_t clock_source;
    mpu6500_extern_sync_t sync;
    mpu6500_low_pass_filter_t filter;
    mpu6500_gyroscope_range_t range_g;
    mpu6500_accelerometer_range_t range_a;
    mpu6500_pin_level_t level;
    mpu6500_pin_type_t pin;
    mpu6500_iic_clock_t clk;
    mpu6500_iic_read_mode_t read_mode;
    mpu6500_iic4_transaction_mode_t transaction_mode;
    mpu6500_iic_transaction_mode_t tran_mode;
    mpu6500_iic_mode_t mode;
    mpu6500_iic_group_order_t order;
    mpu6500_fifo_mode fifo_mode;
    mpu6500_accelerometer_low_pass_filter_t accelerometer_filter;
    mpu6500_low_power_accel_output_rate_t rate;
    
    /* link interface function */
    DRIVER_MPU6500_LINK_INIT(&gs_handle, mpu6500_handle_t);
    DRIVER_MPU6500_LINK_IIC_INIT(&gs_handle, mpu6500_interface_iic_init);
    DRIVER_MPU6500_LINK_IIC_DEINIT(&gs_handle, mpu6500_interface_iic_deinit);
    DRIVER_MPU6500_LINK_IIC_READ(&gs_handle, mpu6500_interface_iic_read);
    DRIVER_MPU6500_LINK_IIC_WRITE(&gs_handle, mpu6500_interface_iic_write);
    DRIVER_MPU6500_LINK_SPI_INIT(&gs_handle, mpu6500_interface_spi_init);
    DRIVER_MPU6500_LINK_SPI_DEINIT(&gs_handle, mpu6500_interface_spi_deinit);
    DRIVER_MPU6500_LINK_SPI_READ(&gs_handle, mpu6500_interface_spi_read);
    DRIVER_MPU6500_LINK_SPI_WRITE(&gs_handle, mpu6500_interface_spi_write);
    DRIVER_MPU6500_LINK_DELAY_MS(&gs_handle, mpu6500_interface_delay_ms);
    DRIVER_MPU6500_LINK_DEBUG_PRINT(&gs_handle, mpu6500_interface_debug_print);
    DRIVER_MPU6500_LINK_RECEIVE_CALLBACK(&gs_handle, mpu6500_interface_receive_callback);
    
    /* get information */
    res = mpu6500_info(&info);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get info failed.\n");
       
        return 1;
    }
    else
    {
        /* print chip info */
        mpu6500_interface_debug_print("mpu6500: chip is %s.\n", info.chip_name);
        mpu6500_interface_debug_print("mpu6500: manufacturer is %s.\n", info.manufacturer_name);
        mpu6500_interface_debug_print("mpu6500: interface is %s.\n", info.interface);
        mpu6500_interface_debug_print("mpu6500: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
        mpu6500_interface_debug_print("mpu6500: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
        mpu6500_interface_debug_print("mpu6500: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
        mpu6500_interface_debug_print("mpu6500: max current is %0.2fmA.\n", info.max_current_ma);
        mpu6500_interface_debug_print("mpu6500: max temperature is %0.1fC.\n", info.temperature_max);
        mpu6500_interface_debug_print("mpu6500: min temperature is %0.1fC.\n", info.temperature_min);
    }
    
    /* start register test */
    mpu6500_interface_debug_print("mpu6500: start register test.\n");
    
    /* mpu6500_set_interface/mpu6500_get_interface test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_interface/mpu6500_get_interface test.\n");
    
    /* iic */
    res = mpu6500_set_interface(&gs_handle, MPU6500_INTERFACE_IIC);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interface failed.\n");
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set interface iic.\n");
    res = mpu6500_get_interface(&gs_handle, &interface_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interface failed.\n");
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interface %s.\n", interface_check == MPU6500_INTERFACE_IIC ? "ok" : "error");
    
    /* spi */
    res = mpu6500_set_interface(&gs_handle, MPU6500_INTERFACE_SPI);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interface failed.\n");
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set interface spi.\n");
    res = mpu6500_get_interface(&gs_handle, &interface_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interface failed.\n");
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interface %s.\n", interface_check == MPU6500_INTERFACE_SPI ? "ok" : "error");
    
    /* mpu6500_set_addr_pin/mpu6500_get_addr_pin test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_addr_pin/mpu6500_get_addr_pin test.\n");
    
    /* set low */
    res = mpu6500_set_addr_pin(&gs_handle, MPU6500_ADDRESS_AD0_LOW);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set addr pin failed.\n");
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set addr pin low.\n");
    res = mpu6500_get_addr_pin(&gs_handle, &addr_pin);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get addr pin failed.\n");
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check addr pin %s.\n", addr_pin == MPU6500_ADDRESS_AD0_LOW ? "ok" : "error");
    
    /* set high */
    res = mpu6500_set_addr_pin(&gs_handle, MPU6500_ADDRESS_AD0_HIGH);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set addr pin failed.\n");
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set addr pin high.\n");
    res = mpu6500_get_addr_pin(&gs_handle, &addr_pin);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get addr pin failed.\n");
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check addr pin %s.\n", addr_pin == MPU6500_ADDRESS_AD0_HIGH ? "ok" : "error");
    
    /* set the interface */
    res = mpu6500_set_interface(&gs_handle, interface);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interface failed.\n");
       
        return 1;
    }
    
    /* set the addr pin */
    res = mpu6500_set_addr_pin(&gs_handle, addr);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set addr pin failed.\n");
       
        return 1;
    }
    
    /* init */
    res = mpu6500_init(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: init failed.\n");
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    /* disable sleep */
    res = mpu6500_set_sleep(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sleep failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* if spi interface, disable iic interface */
    if (interface == MPU6500_INTERFACE_SPI)
    {
        /* disable iic */
        res = mpu6500_set_disable_iic_slave(&gs_handle, MPU6500_BOOL_TRUE);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: set disable iic slave failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
    }
    
    /* set fifo 1024kb */
    res = mpu6500_set_fifo_1024kb(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo 1024kb failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* mpu6500_set_fifo/mpu6500_get_fifo test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_fifo/mpu6500_get_fifo test.\n");
    
    /* enable fifo */
    res = mpu6500_set_fifo(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable fifo.\n");
    
    res = mpu6500_get_fifo(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable fifo */
    res = mpu6500_set_fifo(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable fifo.\n");
    res = mpu6500_get_fifo(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic_master/mpu6500_get_iic_master test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_master/mpu6500_get_iic_master test.\n");
    
    /* enable iic master */
    res = mpu6500_set_iic_master(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic master.\n");
    res = mpu6500_get_iic_master(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic master %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic master */
    res = mpu6500_set_iic_master(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic master.\n");
    res = mpu6500_get_iic_master(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic master %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_fifo_reset/mpu6500_get_fifo_reset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_fifo_reset/mpu6500_get_fifo_reset test.\n");
    
    /* fifo reset */
    res = mpu6500_fifo_reset(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: fifo reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: fifo reset.\n");
    res = mpu6500_get_fifo_reset(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo reset %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_iic_master_reset/mpu6500_get_iic_master_reset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_iic_master_reset/mpu6500_get_iic_master_reset test.\n");
    
    /* reset the iic master */
    res = mpu6500_iic_master_reset(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: iic master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: iic master reset.\n");
    res = mpu6500_get_iic_master_reset(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic master %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_sensor_reset/mpu6500_get_sensor_reset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_sensor_reset/mpu6500_get_sensor_reset test.\n");
    
    /* sensor reset */
    res = mpu6500_sensor_reset(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: sensor reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: sensor reset.\n");
    res = mpu6500_get_sensor_reset(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get sensor reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check sensor reset %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_device_reset/mpu6500_get_device_reset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_device_reset/mpu6500_get_device_reset test.\n");
    
    /* device reset */
    res = mpu6500_device_reset(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: device reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: device reset.\n");
    res = mpu6500_get_device_reset(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get device reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check device reset %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    /* disable sleep */
    res = mpu6500_set_sleep(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sleep failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* if spi interface, disable iic interface */
    if (interface == MPU6500_INTERFACE_SPI)
    {
        /* disable iic */
        res = mpu6500_set_disable_iic_slave(&gs_handle, MPU6500_BOOL_TRUE);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: set disable iic slave failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
    }
    
    /* mpu6500_set_clock_source/mpu6500_get_clock_source test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_clock_source/mpu6500_get_clock_source test.\n");
    
    /* stop the clock */
    res = mpu6500_set_clock_source(&gs_handle, MPU6500_CLOCK_SOURCE_STOP_CLOCK);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set clock source failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: stop the clock.\n");
    res = mpu6500_get_clock_source(&gs_handle, &clock_source);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get clock source failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check clock source %s.\n", clock_source == MPU6500_CLOCK_SOURCE_STOP_CLOCK ? "ok" : "error");
    
    /* internal 20.0 MHz */
    res = mpu6500_set_clock_source(&gs_handle, MPU6500_CLOCK_SOURCE_INTERNAL_20MHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set clock source failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set the clock source extern 20.0 MHz.\n");
    res = mpu6500_get_clock_source(&gs_handle, &clock_source);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get clock source failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check clock source %s.\n", clock_source == MPU6500_CLOCK_SOURCE_INTERNAL_20MHZ ? "ok" : "error");
    
    /* pll */
    res = mpu6500_set_clock_source(&gs_handle, MPU6500_CLOCK_SOURCE_PLL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set clock source failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set the clock source pll.\n");
    res = mpu6500_get_clock_source(&gs_handle, &clock_source);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get clock source failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check clock source %s.\n", clock_source == MPU6500_CLOCK_SOURCE_PLL ? "ok" : "error");
    
    /* mpu6500_set_ptat/mpu6500_get_ptat test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_ptat/mpu6500_get_ptat test.\n");
    
    /* enable temperature sensor */
    res = mpu6500_set_ptat(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set ptat failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable temperature sensor.\n");
    res = mpu6500_get_ptat(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get ptat failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check ptat %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable temperature sensor */
    res = mpu6500_set_ptat(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set ptat failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable temperature sensor.\n");
    res = mpu6500_get_ptat(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get ptat failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check ptat %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_cycle_wake_up/mpu6500_get_cycle_wake_up test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_cycle_wake_up/mpu6500_get_cycle_wake_up test.\n");
    
    /* enable cycle wake up */
    res = mpu6500_set_cycle_wake_up(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set cycle wake up failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable cycle wake up.\n");
    res = mpu6500_get_cycle_wake_up(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get cycle wake up failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check cycle wake up %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable cycle wake up */
    res = mpu6500_set_cycle_wake_up(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set cycle wake up failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable cycle wake up.\n");
    res = mpu6500_get_cycle_wake_up(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get cycle wake up failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check cycle wake up %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_sleep/mpu6500_get_sleep test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_sleep/mpu6500_get_sleep test.\n");
    
    /* enable the sleep */
    res = mpu6500_set_sleep(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sleep failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable the sleep.\n");
    res = mpu6500_get_sleep(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get sleep failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check sleep %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable the sleep */
    res = mpu6500_set_sleep(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sleep failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable the sleep.\n");
    res = mpu6500_get_sleep(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get sleep failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check sleep %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_standby_mode/mpu6500_get_standby_mode test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_standby_mode/mpu6500_get_standby_mode test.\n");
    
    /* enable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_X, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable acc x standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable acc x standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Y, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable acc y standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable acc y standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Z, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable acc z standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable acc z standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_X, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable gyro x standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable gyro x standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Y, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable gyro y standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable gyro y standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Z, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable gyro z standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable standby mode */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable gyro z standby mode.\n");
    res = mpu6500_get_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check standby mode %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_fifo_get/mpu6500_fifo_set test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_fifo_get/mpu6500_fifo_set test.\n");
    
    for (i = 0; i < 8; i++)
    {
        data[i] = (uint8_t)(rand() % 256);
    }
    /* fifo set */
    res = mpu6500_fifo_set(&gs_handle, data, 8);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: fifo write failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo write %s.\n", (res == 0) ? "ok" : "error");
    
    /* fifo get */
    res = mpu6500_fifo_get(&gs_handle, data_check, 8);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: fifo read failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo read %s.\n", (res == 0) ? "ok" : "error");
    
    /* mpu6500_get_fifo_count test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_get_fifo_count test.\n");
    
    /* fifo count */
    res = mpu6500_get_fifo_count(&gs_handle, &cnt);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: fifo count failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: fifo count %d.\n", cnt);
    
    /* mpu6500_set_signal_path_reset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_signal_path_reset test.\n");
    
    /* temp reset */
    res = mpu6500_set_signal_path_reset(&gs_handle, MPU6500_SIGNAL_PATH_RESET_TEMP);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set signal path reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: temp signal path reset.\n");
    mpu6500_interface_debug_print("mpu6500: check signal path reset %s.\n", (res == 0) ? "ok" : "error");
    
    /* accel reset */
    res = mpu6500_set_signal_path_reset(&gs_handle, MPU6500_SIGNAL_PATH_RESET_ACCEL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set signal path reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: accel signal path reset.\n");
    mpu6500_interface_debug_print("mpu6500: check signal path reset %s.\n", (res == 0) ? "ok" : "error");
    
    /* gyro reset */
    res = mpu6500_set_signal_path_reset(&gs_handle, MPU6500_SIGNAL_PATH_RESET_GYRO);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set signal path reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: gyro signal path reset.\n");
    mpu6500_interface_debug_print("mpu6500: check signal path reset %s.\n", (res == 0) ? "ok" : "error");
    
    /* mpu6500_set_sample_rate_divider/mpu6500_get_sample_rate_divider test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_sample_rate_divider/mpu6500_get_sample_rate_divider test.\n");
    
    div_in = (uint8_t)(rand() % 256);
    res = mpu6500_set_sample_rate_divider(&gs_handle, div_in);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sample rate divider failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set sample rate divider 0x%02X.\n", div);
    res = mpu6500_get_sample_rate_divider(&gs_handle, &div_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get sample rate divider failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check sample rate divider %s.\n", (div_in == div_check) ? "ok" : "error");
    
    /* mpu6500_set_extern_sync/mpu6500_get_extern_sync test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_extern_sync/mpu6500_get_extern_sync test.\n");
    
    /* input disabled */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_INPUT_DISABLED);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set extern sync input disabled.\n");
    res = mpu6500_get_extern_sync(&gs_handle, &sync);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check extern sync %s.\n", (sync == MPU6500_EXTERN_SYNC_INPUT_DISABLED) ? "ok" : "error");
    
    /* temp out low */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_TEMP_OUT_L);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set extern sync temp out low.\n");
    res = mpu6500_get_extern_sync(&gs_handle, &sync);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check extern sync %s.\n", (sync == MPU6500_EXTERN_SYNC_TEMP_OUT_L) ? "ok" : "error");
    
    /* gyro xout low */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_GYRO_XOUT_L);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set extern sync gyro xout low.\n");
    res = mpu6500_get_extern_sync(&gs_handle, &sync);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check extern sync %s.\n", (sync == MPU6500_EXTERN_SYNC_GYRO_XOUT_L) ? "ok" : "error");
    
    /* gyro yout low */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_GYRO_YOUT_L);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set extern sync gyro yout low.\n");
    res = mpu6500_get_extern_sync(&gs_handle, &sync);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check extern sync %s.\n", (sync == MPU6500_EXTERN_SYNC_GYRO_YOUT_L) ? "ok" : "error");
    
    /* gyro zout low */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_GYRO_ZOUT_L);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set extern sync gyro zout low.\n");
    res = mpu6500_get_extern_sync(&gs_handle, &sync);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check extern sync %s.\n", (sync == MPU6500_EXTERN_SYNC_GYRO_ZOUT_L) ? "ok" : "error");
    
    /* accel xout low */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_ACCEL_XOUT_L);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set extern sync accel xout low.\n");
    res = mpu6500_get_extern_sync(&gs_handle, &sync);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check extern sync %s.\n", (sync == MPU6500_EXTERN_SYNC_ACCEL_XOUT_L) ? "ok" : "error");
    
    /* accel yout low */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_ACCEL_YOUT_L);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set extern sync accel yout low.\n");
    res = mpu6500_get_extern_sync(&gs_handle, &sync);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check extern sync %s.\n", (sync == MPU6500_EXTERN_SYNC_ACCEL_YOUT_L) ? "ok" : "error");
    
    /* accel zout low */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_ACCEL_ZOUT_L);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set extern sync accel zout low.\n");
    res = mpu6500_get_extern_sync(&gs_handle, &sync);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check extern sync %s.\n", (sync == MPU6500_EXTERN_SYNC_ACCEL_ZOUT_L) ? "ok" : "error");
    
    /* mpu6500_set_low_pass_filter/mpu6500_get_low_pass_filter test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_low_pass_filter/mpu6500_get_low_pass_filter test.\n");
    
    /* filter 0 */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_LOW_PASS_FILTER_0);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set low pass filter 0.\n");
    res = mpu6500_get_low_pass_filter(&gs_handle, &filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check low pass filter %s.\n", (filter == MPU6500_LOW_PASS_FILTER_0) ? "ok" : "error");
    
    /* filter 1 */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_LOW_PASS_FILTER_1);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set low pass filter 1.\n");
    res = mpu6500_get_low_pass_filter(&gs_handle, &filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check low pass filter %s.\n", (filter == MPU6500_LOW_PASS_FILTER_1) ? "ok" : "error");
    
    /* filter 2 */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_LOW_PASS_FILTER_2);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set low pass filter 2.\n");
    res = mpu6500_get_low_pass_filter(&gs_handle, &filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check low pass filter %s.\n", (filter == MPU6500_LOW_PASS_FILTER_2) ? "ok" : "error");
    
    /* filter 3 */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_LOW_PASS_FILTER_3);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set low pass filter 3.\n");
    res = mpu6500_get_low_pass_filter(&gs_handle, &filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check low pass filter %s.\n", (filter == MPU6500_LOW_PASS_FILTER_3) ? "ok" : "error");
    
    /* filter 4 */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_LOW_PASS_FILTER_4);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set low pass filter 4.\n");
    res = mpu6500_get_low_pass_filter(&gs_handle, &filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check low pass filter %s.\n", (filter == MPU6500_LOW_PASS_FILTER_4) ? "ok" : "error");
    
    /* filter 5 */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_LOW_PASS_FILTER_5);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set low pass filter 5.\n");
    res = mpu6500_get_low_pass_filter(&gs_handle, &filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check low pass filter %s.\n", (filter == MPU6500_LOW_PASS_FILTER_5) ? "ok" : "error");
    
    /* filter 6 */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_LOW_PASS_FILTER_6);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set low pass filter 6.\n");
    res = mpu6500_get_low_pass_filter(&gs_handle, &filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check low pass filter %s.\n", (filter == MPU6500_LOW_PASS_FILTER_6) ? "ok" : "error");
    
    /* mpu6500_set_gyroscope_test/mpu6500_get_gyroscope_test test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyroscope_test/mpu6500_get_gyroscope_test test.\n");
    
    /* enable axis x */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable axis x.\n");
    res = mpu6500_get_gyroscope_test(&gs_handle, MPU6500_AXIS_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope test %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable axis x */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable axis x.\n");
    res = mpu6500_get_gyroscope_test(&gs_handle, MPU6500_AXIS_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope test %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable axis y */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable axis y.\n");
    res = mpu6500_get_gyroscope_test(&gs_handle, MPU6500_AXIS_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope test %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable axis y */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable axis y.\n");
    res = mpu6500_get_gyroscope_test(&gs_handle, MPU6500_AXIS_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope test %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable axis z */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable axis z.\n");
    res = mpu6500_get_gyroscope_test(&gs_handle, MPU6500_AXIS_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope test %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable axis z */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable axis z.\n");
    res = mpu6500_get_gyroscope_test(&gs_handle, MPU6500_AXIS_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope test %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_gyroscope_range/mpu6500_get_gyroscope_range test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyroscope_range/mpu6500_get_gyroscope_range test.\n");
    
    /* 250 dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_250DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set gyroscope range 250 dps.\n");
    res = mpu6500_get_gyroscope_range(&gs_handle, &range_g);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope range %s.\n", range_g == MPU6500_GYROSCOPE_RANGE_250DPS ? "ok" : "error");
    
    /* 500 dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_500DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set gyroscope range 500 dps.\n");
    res = mpu6500_get_gyroscope_range(&gs_handle, &range_g);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope range %s.\n", range_g == MPU6500_GYROSCOPE_RANGE_500DPS ? "ok" : "error");
    
    /* 1000 dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_1000DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set gyroscope range 1000 dps.\n");
    res = mpu6500_get_gyroscope_range(&gs_handle, &range_g);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope range %s.\n", range_g == MPU6500_GYROSCOPE_RANGE_1000DPS ? "ok" : "error");
    
    /* 2000 dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_2000DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set gyroscope range 2000 dps.\n");
    res = mpu6500_get_gyroscope_range(&gs_handle, &range_g);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope range %s.\n", range_g == MPU6500_GYROSCOPE_RANGE_2000DPS ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_test/mpu6500_get_accelerometer_test test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_test/mpu6500_get_accelerometer_test test.\n");
    
    /* enable accelerometer x */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable accelerometer x.\n");
    res = mpu6500_get_accelerometer_test(&gs_handle, MPU6500_AXIS_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer test %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable accelerometer x */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable accelerometer x.\n");
    res = mpu6500_get_accelerometer_test(&gs_handle, MPU6500_AXIS_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer test %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable accelerometer y */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable accelerometer y.\n");
    res = mpu6500_get_accelerometer_test(&gs_handle, MPU6500_AXIS_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer test %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable accelerometer y */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable accelerometer y.\n");
    res = mpu6500_get_accelerometer_test(&gs_handle, MPU6500_AXIS_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer test %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable accelerometer z */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable accelerometer z.\n");
    res = mpu6500_get_accelerometer_test(&gs_handle, MPU6500_AXIS_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer test %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable accelerometer z */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable accelerometer z.\n");
    res = mpu6500_get_accelerometer_test(&gs_handle, MPU6500_AXIS_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer test %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_range/mpu6500_get_accelerometer_range test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_range/mpu6500_get_accelerometer_range test.\n");
    
    /* 2g */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_ACCELEROMETER_RANGE_2G);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer range 2g.\n");
    res = mpu6500_get_accelerometer_range(&gs_handle, &range_a);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer range %s.\n", range_a == MPU6500_ACCELEROMETER_RANGE_2G ? "ok" : "error");
    
    /* 4g */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_ACCELEROMETER_RANGE_4G);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer range 4g.\n");
    res = mpu6500_get_accelerometer_range(&gs_handle, &range_a);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer range %s.\n", range_a == MPU6500_ACCELEROMETER_RANGE_4G ? "ok" : "error");
    
    /* 8g */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_ACCELEROMETER_RANGE_8G);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer range 8g.\n");
    res = mpu6500_get_accelerometer_range(&gs_handle, &range_a);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer range %s.\n", range_a == MPU6500_ACCELEROMETER_RANGE_8G ? "ok" : "error");
    
    /* 16g */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_ACCELEROMETER_RANGE_16G);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer range 16g.\n");
    res = mpu6500_get_accelerometer_range(&gs_handle, &range_a);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer range %s.\n", range_a == MPU6500_ACCELEROMETER_RANGE_16G ? "ok" : "error");
    
    /* mpu6500_set_fifo_enable/mpu6500_get_fifo_enable test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_fifo_enable/mpu6500_get_fifo_enable test.\n");
    
    /* set fifo temp enable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_TEMP, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo temp enable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_TEMP, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* set fifo temp disable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_TEMP, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo temp disable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_TEMP, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* set fifo gyroscope x enable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_XG, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo gyroscope x enable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_XG, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* set fifo gyroscope x disable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_XG, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo gyroscope x disable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_XG, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* set fifo gyroscope y enable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_YG, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo gyroscope y enable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_YG, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* set fifo gyroscope y disable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_YG, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo gyroscope y disable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_YG, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* set fifo gyroscope z enable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_ZG, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo gyroscope z enable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_ZG, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* set fifo gyroscope z disable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_ZG, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo gyroscope z disable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_ZG, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* set fifo accelerometer enable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_ACCEL, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo accelerometer enable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_ACCEL, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* set fifo accelerometer disable */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_ACCEL, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo accelerometer disable.\n");
    res = mpu6500_get_fifo_enable(&gs_handle, MPU6500_FIFO_ACCEL, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_interrupt_level/mpu6500_get_interrupt_level test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_interrupt_level/mpu6500_get_interrupt_level test.\n");
    
    /* high level */
    res = mpu6500_set_interrupt_level(&gs_handle, MPU6500_PIN_LEVEL_HIGH);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set interrupt high level.\n");
    res = mpu6500_get_interrupt_level(&gs_handle, &level);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interrupt level %s.\n", level == MPU6500_PIN_LEVEL_HIGH ? "ok" : "error");
    
    /* low level */
    res = mpu6500_set_interrupt_level(&gs_handle, MPU6500_PIN_LEVEL_LOW);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set interrupt low level.\n");
    res = mpu6500_get_interrupt_level(&gs_handle, &level);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interrupt level %s.\n", level == MPU6500_PIN_LEVEL_LOW ? "ok" : "error");
    
    /* mpu6500_set_interrupt_pin_type/mpu6500_get_interrupt_pin_type test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_interrupt_pin_type/mpu6500_get_interrupt_pin_type test.\n");
    
    /* push-pull */
    res = mpu6500_set_interrupt_pin_type(&gs_handle, MPU6500_PIN_TYPE_PUSH_PULL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt pin type failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set interrupt pin type push pull.\n");
    res = mpu6500_get_interrupt_pin_type(&gs_handle, &pin);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt pin type failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interrupt pin type %s.\n", pin == MPU6500_PIN_TYPE_PUSH_PULL ? "ok" : "error");
    
    /* open drain */
    res = mpu6500_set_interrupt_pin_type(&gs_handle, MPU6500_PIN_TYPE_OPEN_DRAIN);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt pin type failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set interrupt pin type open drain.\n");
    res = mpu6500_get_interrupt_pin_type(&gs_handle, &pin);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt pin type failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interrupt pin type %s.\n", pin == MPU6500_PIN_TYPE_OPEN_DRAIN ? "ok" : "error");
    
    /* mpu6500_set_interrupt_latch/mpu6500_get_interrupt_latch test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_interrupt_latch/mpu6500_get_interrupt_latch test.\n");
    
    /* enable latch */
    res = mpu6500_set_interrupt_latch(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt latch failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable interrupt latch.\n");
    res = mpu6500_get_interrupt_latch(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt latch failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interrupt latch %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable latch */
    res = mpu6500_set_interrupt_latch(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt latch failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable interrupt latch.\n");
    res = mpu6500_get_interrupt_latch(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt latch failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interrupt latch %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_interrupt_read_clear/mpu6500_get_interrupt_read_clear test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_interrupt_read_clear/mpu6500_get_interrupt_read_clear test.\n");
    
    /* enable interrupt read clear */
    res = mpu6500_set_interrupt_read_clear(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt read clear failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable interrupt read clear.\n");
    res = mpu6500_get_interrupt_read_clear(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt read clear failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interrupt read clear %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable interrupt read clear */
    res = mpu6500_set_interrupt_read_clear(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt read clear failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable interrupt read clear.\n");
    res = mpu6500_get_interrupt_read_clear(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt read clear failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check interrupt read clear %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_fsync_interrupt_level/mpu6500_get_fsync_interrupt_level test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_fsync_interrupt_level/mpu6500_get_fsync_interrupt_level test.\n");
    
    /* set fsync interrupt level high */
    res = mpu6500_set_fsync_interrupt_level(&gs_handle, MPU6500_PIN_LEVEL_HIGH);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fsync interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fsync interrupt level high.\n");
    res = mpu6500_get_fsync_interrupt_level(&gs_handle, &level);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fsync interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fsync interrupt level %s.\n", level == MPU6500_PIN_LEVEL_HIGH ? "ok" : "error");
    
    /* set fsync interrupt level low */
    res = mpu6500_set_fsync_interrupt_level(&gs_handle, MPU6500_PIN_LEVEL_LOW);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fsync interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set fsync interrupt level low.\n");
    res = mpu6500_get_fsync_interrupt_level(&gs_handle, &level);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fsync interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fsync interrupt level %s.\n", level == MPU6500_PIN_LEVEL_LOW ? "ok" : "error");
    
    /* mpu6500_set_fsync_interrupt/mpu6500_get_fsync_interrupt test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_fsync_interrupt/mpu6500_get_fsync_interrupt test.\n");
    
    /* enable fsync interrupt */
    res = mpu6500_set_fsync_interrupt(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fsync interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable fsync interrupt.\n");
    res = mpu6500_get_fsync_interrupt(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fsync interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fsync interrupt %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable fsync interrupt */
    res = mpu6500_set_fsync_interrupt(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fsync interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable fsync interrupt.\n");
    res = mpu6500_get_fsync_interrupt(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fsync interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fsync interrupt %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic_bypass/mpu6500_get_iic_bypass test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_bypass/mpu6500_get_iic_bypass test.\n");
    
    /* enable iic bypass */
    res = mpu6500_set_iic_bypass(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic bypass failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic bypass.\n");
    res = mpu6500_get_iic_bypass(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic bypass failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic bypass %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic bypass */
    res = mpu6500_set_iic_bypass(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic bypass failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic bypass.\n");
    res = mpu6500_get_iic_bypass(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic bypass failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic bypass %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_interrupt/mpu6500_get_interrupt test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_interrupt/mpu6500_get_interrupt test.\n");
    
    /* enable motion interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_MOTION, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable motion interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_MOTION, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check motion interrupt %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable motion interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_MOTION, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable motion interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_MOTION, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check motion interrupt %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable fifo overflow interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FIFO_OVERFLOW, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable fifo overflow interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_FIFO_OVERFLOW, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo overflow interrupt %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable fifo overflow interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FIFO_OVERFLOW, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable fifo overflow interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_FIFO_OVERFLOW, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo overflow interrupt %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable dmp interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_DMP, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable dmp interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_DMP, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check dmp interrupt %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable dmp interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_DMP, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable dmp interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_DMP, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check dmp interrupt %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable fsync init interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FSYNC_INT, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable fsync init interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_FSYNC_INT, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fsync init interrupt %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable fsync init interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FSYNC_INT, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable fsync init interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_FSYNC_INT, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fsync init interrupt %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable data ready interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_DATA_READY, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable data ready interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_DATA_READY, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check data ready interrupt %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable data ready interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_DATA_READY, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable data ready interrupt.\n");
    res = mpu6500_get_interrupt(&gs_handle, MPU6500_INTERRUPT_DATA_READY, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check data ready interrupt %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_get_interrupt_status test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_get_interrupt_status test.\n");
    
    /* get interrupt status */
    res = mpu6500_get_interrupt_status(&gs_handle, &status);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get interrupt status failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: get interrupt status 0x%02x.\n", status);
    
    /* mpu6500_set_gyroscope_x_test/mpu6500_get_gyroscope_x_test test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyroscope_x_test/mpu6500_get_gyroscope_x_test test.\n");
    
    test = rand() % 0x20;
    res = mpu6500_set_gyroscope_x_test(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope x test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set gyroscope x test 0x%02X.\n", test);
    res = mpu6500_get_gyroscope_x_test(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope x test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope x test %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_set_gyroscope_y_test/mpu6500_get_gyroscope_y_test test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyroscope_y_test/mpu6500_get_gyroscope_y_test test.\n");
    
    test = rand() % 0x20;
    res = mpu6500_set_gyroscope_y_test(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope y test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set gyroscope y test 0x%02X.\n", test);
    res = mpu6500_get_gyroscope_y_test(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope y test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope y test %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_set_gyroscope_z_test/mpu6500_get_gyroscope_z_test test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyroscope_z_test/mpu6500_get_gyroscope_z_test test.\n");
    
    test = rand() % 0x20;
    res = mpu6500_set_gyroscope_z_test(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope z test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set gyroscope z test 0x%02X.\n", test);
    res = mpu6500_get_gyroscope_z_test(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope z test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope z test %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_x_test/mpu6500_get_accelerometer_x_test test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_x_test/mpu6500_get_accelerometer_x_test test.\n");
    
    test = rand() % 0x20;
    res = mpu6500_set_accelerometer_x_test(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer x test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer x test 0x%02X.\n", test);
    res = mpu6500_get_accelerometer_x_test(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer x test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer x test %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_y_test/mpu6500_get_accelerometer_y_test test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_y_test/mpu6500_get_accelerometer_y_test test.\n");
    
    test = rand() % 0x20;
    res = mpu6500_set_accelerometer_y_test(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer y test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer y test 0x%02X.\n", test);
    res = mpu6500_get_accelerometer_y_test(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer y test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer y test %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_z_test/mpu6500_get_accelerometer_z_test test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_z_test/mpu6500_get_accelerometer_z_test test.\n");
    
    test = rand() % 0x20;
    res = mpu6500_set_accelerometer_z_test(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer z test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer z test 0x%02X.\n", test);
    res = mpu6500_get_accelerometer_z_test(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer z test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer z test %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_set_motion_threshold/mpu6500_get_motion_threshold test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_motion_threshold/mpu6500_get_motion_threshold test.\n");
    
    test = rand() % 256;
    res = mpu6500_set_motion_threshold(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set motion threshold failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set motion threshold 0x%02X.\n", test);
    res = mpu6500_get_motion_threshold(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get motion threshold failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check motion threshold %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_motion_threshold_convert_to_register/mpu6500_motion_threshold_convert_to_data test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_motion_threshold_convert_to_register/mpu6500_motion_threshold_convert_to_data test.\n");
    
    test_f = (rand() % 10000) / 10.0f + 32.0f;
    res = mpu6500_motion_threshold_convert_to_register(&gs_handle, test_f, &reg);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: motion threshold convert to register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: motion threshold convert to register %0.2f.\n", test_f);
    res = mpu6500_motion_threshold_convert_to_data(&gs_handle, reg, &test_check_f);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: motion threshold convert to data failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check motion threshold %0.2f.\n", test_check_f);
    
    /* mpu6500_set_iic_clock/mpu6500_get_iic_clock test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_clock/mpu6500_get_iic_clock test.\n");
    
    /* 348 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_348_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 348 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_348_KHZ ? "ok" : "error");
    
    /* 333 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_333_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 333 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_333_KHZ ? "ok" : "error");
    
    /* 320 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_320_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 320 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_320_KHZ ? "ok" : "error");
    
    /* 308 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_308_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 308 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_308_KHZ ? "ok" : "error");
    
    /* 296 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_296_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 296 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_296_KHZ ? "ok" : "error");
    
    /* 286 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_286_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 286 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_286_KHZ ? "ok" : "error");
    
    /* 276 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_276_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 276 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_276_KHZ ? "ok" : "error");
    
    /* 267 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_267_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 267 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_267_KHZ ? "ok" : "error");
    
    /* 258 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_258_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 258 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_258_KHZ ? "ok" : "error");
    
    /* 500 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_500_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 500 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_500_KHZ ? "ok" : "error");
    
    /* 471 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_471_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 471 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_471_KHZ ? "ok" : "error");
    
    /* 444 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_444_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 444 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_444_KHZ ? "ok" : "error");
    
    /* 421 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_421_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 421 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_421_KHZ ? "ok" : "error");
    
    /* 400 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_400_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 400 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_400_KHZ ? "ok" : "error");
    
    /* 381 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_381_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 381 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_381_KHZ ? "ok" : "error");
    
    /* 364 kHz */
    res = mpu6500_set_iic_clock(&gs_handle, MPU6500_IIC_CLOCK_364_KHZ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set iic clock 364 kHz.\n");
    res = mpu6500_get_iic_clock(&gs_handle, &clk);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic clock failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic clock %s.\n", clk == MPU6500_IIC_CLOCK_364_KHZ ? "ok" : "error");
    
    /* mpu6500_set_iic_multi_master/mpu6500_get_iic_multi_master test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_multi_master/mpu6500_get_iic_multi_master test.\n");
    
    /* enable multi master */
    res = mpu6500_set_iic_multi_master(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic multi master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic multi master.\n");
    res = mpu6500_get_iic_multi_master(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic multi master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic multi master %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable multi master */
    res = mpu6500_set_iic_multi_master(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic multi master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic multi master.\n");
    res = mpu6500_get_iic_multi_master(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic multi master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic multi master %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic_wait_for_external_sensor/mpu6500_get_iic_wait_for_external_sensor test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_wait_for_external_sensor/mpu6500_get_iic_wait_for_external_sensor test.\n");
    
    /* enable iic wait for external sensor */
    res = mpu6500_set_iic_wait_for_external_sensor(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic wait for external sensor failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic wait for external sensor.\n");
    res = mpu6500_get_iic_wait_for_external_sensor(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic wait for external sensor failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic wait for external sensor %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic wait for external sensor */
    res = mpu6500_set_iic_wait_for_external_sensor(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic wait for external sensor failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic wait for external sensor.\n");
    res = mpu6500_get_iic_wait_for_external_sensor(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic wait for external sensor failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic wait for external sensor %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic_read_mode/mpu6500_get_iic_read_mode test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_read_mode/mpu6500_get_iic_read_mode test.\n");
    
    /* restart read mode */
    res = mpu6500_set_iic_read_mode(&gs_handle, MPU6500_IIC_READ_MODE_RESTART);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic read mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set restart read mode.\n");
    res = mpu6500_get_iic_read_mode(&gs_handle, &read_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic read mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic read mode %s.\n", read_mode == MPU6500_IIC_READ_MODE_RESTART ? "ok" : "error");
    
    /* stop and start read mode */
    res = mpu6500_set_iic_read_mode(&gs_handle, MPU6500_IIC_READ_MODE_STOP_AND_START);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic read mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set stop and start read mode.\n");
    res = mpu6500_get_iic_read_mode(&gs_handle, &read_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic read mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic read mode %s.\n", read_mode == MPU6500_IIC_READ_MODE_STOP_AND_START ? "ok" : "error");
    
    /* mpu6500_set_iic_fifo_enable/mpu6500_get_iic_fifo_enable test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_fifo_enable/mpu6500_get_iic_fifo_enable test.\n");
    
    /* enable iic fifo slave0 */
    res = mpu6500_set_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic fifo slave0.\n");
    res = mpu6500_get_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_0, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic fifo slave0 */
    res = mpu6500_set_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic fifo slave0.\n");
    res = mpu6500_get_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_0, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable iic fifo slave1 */
    res = mpu6500_set_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic fifo slave1.\n");
    res = mpu6500_get_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_1, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic fifo slave1 */
    res = mpu6500_set_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic fifo slave1.\n");
    res = mpu6500_get_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_1, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable iic fifo slave2 */
    res = mpu6500_set_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic fifo slave2.\n");
    res = mpu6500_get_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_2, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic fifo slave2 */
    res = mpu6500_set_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic fifo slave2.\n");
    res = mpu6500_get_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_2, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable iic fifo slave3 */
    res = mpu6500_set_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic fifo slave3.\n");
    res = mpu6500_get_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_3, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic fifo enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic fifo slave3 */
    res = mpu6500_set_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic fifo slave3.\n");
    res = mpu6500_get_iic_fifo_enable(&gs_handle, MPU6500_IIC_SLAVE_3, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic fifo enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic_mode/mpu6500_get_iic_mode test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_mode/mpu6500_get_iic_mode test.\n");
    
    /* write mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_IIC_MODE_WRITE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 iic write mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_0, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_WRITE ? "ok" : "error");
    
    /* read mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_IIC_MODE_READ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 iic read mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_0, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_READ ? "ok" : "error");
    
    /* write mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_IIC_MODE_WRITE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 iic write mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_1, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_WRITE ? "ok" : "error");
    
    /* read mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_IIC_MODE_READ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 iic read mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_1, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_READ ? "ok" : "error");
    
    /* write mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_IIC_MODE_WRITE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 iic write mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_2, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_WRITE ? "ok" : "error");
    
    /* read mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_IIC_MODE_READ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 iic read mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_2, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_READ ? "ok" : "error");
    
    /* write mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_IIC_MODE_WRITE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 iic write mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_3, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_WRITE ? "ok" : "error");
    
    /* read mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_IIC_MODE_READ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 iic read mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_3, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_READ ? "ok" : "error");
    
    /* write mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_4, MPU6500_IIC_MODE_WRITE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave4 iic write mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_4, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_WRITE ? "ok" : "error");
    
    /* read mode */
    res = mpu6500_set_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_4, MPU6500_IIC_MODE_READ);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave4 iic read mode.\n");
    res = mpu6500_get_iic_mode(&gs_handle, MPU6500_IIC_SLAVE_4, &mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic mode %s.\n", mode == MPU6500_IIC_MODE_READ ? "ok" : "error");
    
    /* mpu6500_set_iic_address/mpu6500_get_iic_address test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_address/mpu6500_get_iic_address test.\n");
    
    /* slave0 */
    test = rand() % 0x7F;
    res = mpu6500_set_iic_address(&gs_handle, MPU6500_IIC_SLAVE_0, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 iic address 0x%02X.\n", test);
    res = mpu6500_get_iic_address(&gs_handle, MPU6500_IIC_SLAVE_0, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic address %s.\n", test_check == test ? "ok" : "error");
    
    /* slave1 */
    test = rand() % 0x7F;
    res = mpu6500_set_iic_address(&gs_handle, MPU6500_IIC_SLAVE_1, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 iic address 0x%02X.\n", test);
    res = mpu6500_get_iic_address(&gs_handle, MPU6500_IIC_SLAVE_1, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic address %s.\n", test_check == test ? "ok" : "error");
    
    /* slave2 */
    test = rand() % 0x7F;
    res = mpu6500_set_iic_address(&gs_handle, MPU6500_IIC_SLAVE_2, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 iic address 0x%02X.\n", test);
    res = mpu6500_get_iic_address(&gs_handle, MPU6500_IIC_SLAVE_2, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic address %s.\n", test_check == test ? "ok" : "error");
    
    /* slave3 */
    test = rand() % 0x7F;
    res = mpu6500_set_iic_address(&gs_handle, MPU6500_IIC_SLAVE_3, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 iic address 0x%02X.\n", test);
    res = mpu6500_get_iic_address(&gs_handle, MPU6500_IIC_SLAVE_3, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic address %s.\n", test_check == test ? "ok" : "error");
    
    /* slave4 */
    test = rand() % 0x7F;
    res = mpu6500_set_iic_address(&gs_handle, MPU6500_IIC_SLAVE_4, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave4 iic address 0x%02X.\n", test);
    res = mpu6500_get_iic_address(&gs_handle, MPU6500_IIC_SLAVE_4, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic address failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic address %s.\n", test_check == test ? "ok" : "error");
    
    /* mpu6500_set_iic_register/mpu6500_get_iic_register test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_register/mpu6500_get_iic_register test.\n");
    
    /* slave0 */
    test = rand() % 256;
    res = mpu6500_set_iic_register(&gs_handle, MPU6500_IIC_SLAVE_0, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 iic register 0x%02X.\n", test);
    res = mpu6500_get_iic_register(&gs_handle, MPU6500_IIC_SLAVE_0, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic register %s.\n", test_check == test ? "ok" : "error");
    
    /* slave1 */
    test = rand() % 256;
    res = mpu6500_set_iic_register(&gs_handle, MPU6500_IIC_SLAVE_1, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 iic register 0x%02X.\n", test);
    res = mpu6500_get_iic_register(&gs_handle, MPU6500_IIC_SLAVE_1, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic register %s.\n", test_check == test ? "ok" : "error");
    
    /* slave2 */
    test = rand() % 256;
    res = mpu6500_set_iic_register(&gs_handle, MPU6500_IIC_SLAVE_2, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 iic register 0x%02X.\n", test);
    res = mpu6500_get_iic_register(&gs_handle, MPU6500_IIC_SLAVE_2, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic register %s.\n", test_check == test ? "ok" : "error");
    
    /* slave3 */
    test = rand() % 256;
    res = mpu6500_set_iic_register(&gs_handle, MPU6500_IIC_SLAVE_3, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 iic register 0x%02X.\n", test);
    res = mpu6500_get_iic_register(&gs_handle, MPU6500_IIC_SLAVE_3, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic register %s.\n", test_check == test ? "ok" : "error");
    
    /* slave4 */
    test = rand() % 256;
    res = mpu6500_set_iic_register(&gs_handle, MPU6500_IIC_SLAVE_4, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave4 iic register 0x%02X.\n", test);
    res = mpu6500_get_iic_register(&gs_handle, MPU6500_IIC_SLAVE_4, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic register %s.\n", test_check == test ? "ok" : "error");
    
    /* mpu6500_set_iic_data_out/mpu6500_get_iic_data_out test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_data_out/mpu6500_get_iic_data_out test.\n");
    
    /* slave0 */
    test = rand() % 256;
    res = mpu6500_set_iic_data_out(&gs_handle, MPU6500_IIC_SLAVE_0, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 iic data out 0x%02X.\n", test);
    res = mpu6500_get_iic_data_out(&gs_handle, MPU6500_IIC_SLAVE_0, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic data out %s.\n", test_check == test ? "ok" : "error");
    
    /* slave1 */
    test = rand() % 256;
    res = mpu6500_set_iic_data_out(&gs_handle, MPU6500_IIC_SLAVE_1, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 iic data out 0x%02X.\n", test);
    res = mpu6500_get_iic_data_out(&gs_handle, MPU6500_IIC_SLAVE_1, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic data out %s.\n", test_check == test ? "ok" : "error");
    
    /* slave2 */
    test = rand() % 256;
    res = mpu6500_set_iic_data_out(&gs_handle, MPU6500_IIC_SLAVE_2, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 iic data out 0x%02X.\n", test);
    res = mpu6500_get_iic_data_out(&gs_handle, MPU6500_IIC_SLAVE_2, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic data out %s.\n", test_check == test ? "ok" : "error");
    
    /* slave3 */
    test = rand() % 256;
    res = mpu6500_set_iic_data_out(&gs_handle, MPU6500_IIC_SLAVE_3, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 iic data out 0x%02X.\n", test);
    res = mpu6500_get_iic_data_out(&gs_handle, MPU6500_IIC_SLAVE_3, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic data out %s.\n", test_check == test ? "ok" : "error");
    
    /* mpu6500_set_iic_enable/mpu6500_get_iic_enable test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_enable/mpu6500_get_iic_enable test.\n");
    
    /* slave0 iic enable */
    res = mpu6500_set_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: slave0 iic enable.\n");
    res = mpu6500_get_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_0, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* slave0 iic disable */
    res = mpu6500_set_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: slave0 iic disable.\n");
    res = mpu6500_get_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_0, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* slave1 iic enable */
    res = mpu6500_set_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: slave1 iic enable.\n");
    res = mpu6500_get_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_1, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* slave1 iic disable */
    res = mpu6500_set_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: slave1 iic disable.\n");
    res = mpu6500_get_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_1, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* slave2 iic enable */
    res = mpu6500_set_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: slave2 iic enable.\n");
    res = mpu6500_get_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_2, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* slave2 iic disable */
    res = mpu6500_set_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: slave2 iic disable.\n");
    res = mpu6500_get_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_2, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* slave3 iic enable */
    res = mpu6500_set_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: slave3 iic enable.\n");
    res = mpu6500_get_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_3, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* slave3 iic disable */
    res = mpu6500_set_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: slave3 iic disable.\n");
    res = mpu6500_get_iic_enable(&gs_handle, MPU6500_IIC_SLAVE_3, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic_byte_swap/mpu6500_get_iic_byte_swap test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_byte_swap/mpu6500_get_iic_byte_swap test.\n");
    
    /* enable slave0 byte swap */
    res = mpu6500_set_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable slave0 byte swap.\n");
    res = mpu6500_get_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_0, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic byte swap %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable slave0 byte swap */
    res = mpu6500_set_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable slave0 byte swap.\n");
    res = mpu6500_get_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_0, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic byte swap %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable slave1 byte swap */
    res = mpu6500_set_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable slave1 byte swap.\n");
    res = mpu6500_get_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_1, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic byte swap %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable slave1 byte swap */
    res = mpu6500_set_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable slave1 byte swap.\n");
    res = mpu6500_get_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_1, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic byte swap %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable slave2 byte swap */
    res = mpu6500_set_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable slave2 byte swap.\n");
    res = mpu6500_get_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_2, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic byte swap %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable slave2 byte swap */
    res = mpu6500_set_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable slave2 byte swap.\n");
    res = mpu6500_get_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_2, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic byte swap %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable slave3 byte swap */
    res = mpu6500_set_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable slave3 byte swap.\n");
    res = mpu6500_get_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_3, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic byte swap %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable slave3 byte swap */
    res = mpu6500_set_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable slave3 byte swap.\n");
    res = mpu6500_get_iic_byte_swap(&gs_handle, MPU6500_IIC_SLAVE_3, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic byte swap failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic byte swap %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic_transaction_mode/mpu6500_get_iic_transaction_mode test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_transaction_mode/mpu6500_get_iic_transaction_mode test.\n");
    
    /* set slave0 data transaction mode */
    res = mpu6500_set_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_IIC_TRANSACTION_MODE_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 data transaction mode.\n");
    res = mpu6500_get_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_0, &tran_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transaction mode %s.\n", tran_mode == MPU6500_IIC_TRANSACTION_MODE_DATA ? "ok" : "error");
    
    /* set slave0 reg data transaction mode */
    res = mpu6500_set_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_IIC_TRANSACTION_MODE_REG_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 reg data transaction mode.\n");
    res = mpu6500_get_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_0, &tran_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transaction mode %s.\n", tran_mode == MPU6500_IIC_TRANSACTION_MODE_REG_DATA ? "ok" : "error");
    
    /* set slave1 data transaction mode */
    res = mpu6500_set_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_IIC_TRANSACTION_MODE_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 data transaction mode.\n");
    res = mpu6500_get_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_1, &tran_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transaction mode %s.\n", tran_mode == MPU6500_IIC_TRANSACTION_MODE_DATA ? "ok" : "error");
    
    /* set slave1 reg data transaction mode */
    res = mpu6500_set_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_IIC_TRANSACTION_MODE_REG_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 reg data transaction mode.\n");
    res = mpu6500_get_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_1, &tran_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transaction mode %s.\n", tran_mode == MPU6500_IIC_TRANSACTION_MODE_REG_DATA ? "ok" : "error");
    
    /* set slave2 data transaction mode */
    res = mpu6500_set_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_IIC_TRANSACTION_MODE_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 data transaction mode.\n");
    res = mpu6500_get_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_2, &tran_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transaction mode %s.\n", tran_mode == MPU6500_IIC_TRANSACTION_MODE_DATA ? "ok" : "error");
    
    /* set slave2 reg data transaction mode */
    res = mpu6500_set_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_IIC_TRANSACTION_MODE_REG_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 reg data transaction mode.\n");
    res = mpu6500_get_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_2, &tran_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transaction mode %s.\n", tran_mode == MPU6500_IIC_TRANSACTION_MODE_REG_DATA ? "ok" : "error");
    
    /* set slave3 data transaction mode */
    res = mpu6500_set_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_IIC_TRANSACTION_MODE_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 data transaction mode.\n");
    res = mpu6500_get_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_3, &tran_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transaction mode %s.\n", tran_mode == MPU6500_IIC_TRANSACTION_MODE_DATA ? "ok" : "error");
    
    /* set slave3 reg data transaction mode */
    res = mpu6500_set_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_IIC_TRANSACTION_MODE_REG_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 reg data transaction mode.\n");
    res = mpu6500_get_iic_transaction_mode(&gs_handle, MPU6500_IIC_SLAVE_3, &tran_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transaction mode %s.\n", tran_mode == MPU6500_IIC_TRANSACTION_MODE_REG_DATA ? "ok" : "error");
    
    /* mpu6500_set_iic_group_order/mpu6500_get_iic_group_order test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_group_order/mpu6500_get_iic_group_order test.\n");
    
    /* set slave0 group order even */
    res = mpu6500_set_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_IIC_GROUP_ORDER_EVEN);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 group order even.\n");
    res = mpu6500_get_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_0, &order);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic group order %s.\n", order == MPU6500_IIC_GROUP_ORDER_EVEN ? "ok" : "error");
    
    /* set slave0 group order odd */
    res = mpu6500_set_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_0, MPU6500_IIC_GROUP_ORDER_ODD);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 group order odd.\n");
    res = mpu6500_get_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_0, &order);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic group order %s.\n", order == MPU6500_IIC_GROUP_ORDER_ODD ? "ok" : "error");
    
    /* set slave1 group order even */
    res = mpu6500_set_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_IIC_GROUP_ORDER_EVEN);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 group order even.\n");
    res = mpu6500_get_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_1, &order);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic group order %s.\n", order == MPU6500_IIC_GROUP_ORDER_EVEN ? "ok" : "error");
    
    /* set slave1 group order odd */
    res = mpu6500_set_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_1, MPU6500_IIC_GROUP_ORDER_ODD);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 group order odd.\n");
    res = mpu6500_get_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_1, &order);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic group order %s.\n", order == MPU6500_IIC_GROUP_ORDER_ODD ? "ok" : "error");
    
    /* set slave2 group order even */
    res = mpu6500_set_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_IIC_GROUP_ORDER_EVEN);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 group order even.\n");
    res = mpu6500_get_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_2, &order);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic group order %s.\n", order == MPU6500_IIC_GROUP_ORDER_EVEN ? "ok" : "error");
    
    /* set slave2 group order odd */
    res = mpu6500_set_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_2, MPU6500_IIC_GROUP_ORDER_ODD);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 group order odd.\n");
    res = mpu6500_get_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_2, &order);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic group order %s.\n", order == MPU6500_IIC_GROUP_ORDER_ODD ? "ok" : "error");
    
    /* set slave3 group order even */
    res = mpu6500_set_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_IIC_GROUP_ORDER_EVEN);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 group order even.\n");
    res = mpu6500_get_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_3, &order);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic group order %s.\n", order == MPU6500_IIC_GROUP_ORDER_EVEN ? "ok" : "error");
    
    /* set slave3 group order odd */
    res = mpu6500_set_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_3, MPU6500_IIC_GROUP_ORDER_ODD);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 group order odd.\n");
    res = mpu6500_get_iic_group_order(&gs_handle, MPU6500_IIC_SLAVE_3, &order);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic group order failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic group order %s.\n", order == MPU6500_IIC_GROUP_ORDER_ODD ? "ok" : "error");
    
    /* mpu6500_set_iic_transferred_len/mpu6500_get_iic_transferred_len test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_transferred_len/mpu6500_get_iic_transferred_len test.\n");
    
    test = rand() % 16;
    res = mpu6500_set_iic_transferred_len(&gs_handle, MPU6500_IIC_SLAVE_0, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transferred len failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave0 iic transferred len %d.\n", test);
    res = mpu6500_get_iic_transferred_len(&gs_handle, MPU6500_IIC_SLAVE_0, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transferred len failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transferred len %s.\n", test_check == test ? "ok" : "error");
    
    test = rand() % 16;
    res = mpu6500_set_iic_transferred_len(&gs_handle, MPU6500_IIC_SLAVE_1, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transferred len failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave1 iic transferred len %d.\n", test);
    res = mpu6500_get_iic_transferred_len(&gs_handle, MPU6500_IIC_SLAVE_1, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transferred len failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transferred len %s.\n", test_check == test ? "ok" : "error");
    
    test = rand() % 16;
    res = mpu6500_set_iic_transferred_len(&gs_handle, MPU6500_IIC_SLAVE_2, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transferred len failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave2 iic transferred len %d.\n", test);
    res = mpu6500_get_iic_transferred_len(&gs_handle, MPU6500_IIC_SLAVE_2, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transferred len failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transferred len %s.\n", test_check == test ? "ok" : "error");
    
    test = rand() % 16;
    res = mpu6500_set_iic_transferred_len(&gs_handle, MPU6500_IIC_SLAVE_3, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic transferred len failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set slave3 iic transferred len %d.\n", test);
    res = mpu6500_get_iic_transferred_len(&gs_handle, MPU6500_IIC_SLAVE_3, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic transferred len failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic transferred len %s.\n", test_check == test ? "ok" : "error");
    
    /* mpu6500_get_iic_status test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_get_iic_status test.\n");
    
    /* get iic status */
    res = mpu6500_get_iic_status(&gs_handle, &status);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic status failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: iic status is 0x%02X.\n", status);
    
    /* mpu6500_set_iic_delay_enable/mpu6500_get_iic_delay_enable test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_delay_enable/mpu6500_get_iic_delay_enable test.\n");
    
    /* enable delay shadow */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_ES_SHADOW, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable delay shadow.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_ES_SHADOW, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable delay shadow */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_ES_SHADOW, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable delay shadow.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_ES_SHADOW, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable delay slave4 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_4, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable delay slave4.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_4, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable delay slave4 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_4, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable delay slave4.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_4, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable delay slave3 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_3, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable delay slave3.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_3, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable delay slave3 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_3, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable delay slave3.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_3, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable delay slave2 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_2, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable delay slave2.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_2, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable delay slave2 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_2, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable delay slave2.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_2, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable delay slave1 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_1, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable delay slave1.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_1, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable delay slave1 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_1, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable delay slave1.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_1, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable delay slave0 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_0, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: enable delay slave0.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_0, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable delay slave0 */
    res = mpu6500_set_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_0, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: disable delay slave0.\n");
    res = mpu6500_get_iic_delay_enable(&gs_handle, MPU6500_IIC_DELAY_SLAVE_0, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic4_enable/mpu6500_get_iic4_enable test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic4_enable/mpu6500_get_iic4_enable test.\n");
    
    /* enable iic4 */
    res = mpu6500_set_iic4_enable(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic4 enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic4.\n");
    res = mpu6500_get_iic4_enable(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic4 enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic4 enable %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic4 */
    res = mpu6500_set_iic4_enable(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic4 enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic4.\n");
    res = mpu6500_get_iic4_enable(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic4 enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic4 enable %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic4_interrupt/mpu6500_get_iic4_interrupt test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic4_interrupt/mpu6500_get_iic4_interrupt test.\n");
    
    /* enable iic4 interrupt */
    res = mpu6500_set_iic4_interrupt(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic4 interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: enable iic4 interrupt.\n");
    res = mpu6500_get_iic4_interrupt(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic4 interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic4 interrupt %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable iic4 interrupt */
    res = mpu6500_set_iic4_interrupt(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic4 interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: disable iic4 interrupt.\n");
    res = mpu6500_get_iic4_interrupt(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic4 interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic4 interrupt %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_iic4_transaction_mode/mpu6500_get_iic4_transaction_mode test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic4_transaction_mode/mpu6500_get_iic4_transaction_mode test.\n");
    
    /* transaction mode data */
    res = mpu6500_set_iic4_transaction_mode(&gs_handle, MPU6500_IIC4_TRANSACTION_MODE_DATA);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic4 transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set iic4 transaction mode data.\n");
    res = mpu6500_get_iic4_transaction_mode(&gs_handle, &transaction_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic4 transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic4 transaction mode %s.\n", transaction_mode == MPU6500_IIC4_TRANSACTION_MODE_DATA ? "ok" : "error");
    
    /* transaction mode reg */
    res = mpu6500_set_iic4_transaction_mode(&gs_handle, MPU6500_IIC4_TRANSACTION_MODE_REG);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic4 transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set iic4 transaction mode reg.\n");
    res = mpu6500_get_iic4_transaction_mode(&gs_handle, &transaction_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic4 transaction mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic4 transaction mode %s.\n", transaction_mode == MPU6500_IIC4_TRANSACTION_MODE_REG ? "ok" : "error");
    
    /* mpu6500_set_iic_delay/mpu6500_get_iic_delay test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic_delay/mpu6500_get_iic_delay test.\n");
    
    test = rand() % 0x1F;
    res = mpu6500_set_iic_delay(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic delay failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set iic delay 0x%02X.\n", test);
    res = mpu6500_get_iic_delay(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic delay failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic delay %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_set_iic4_data_out/mpu6500_get_iic4_data_out test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic4_data_out/mpu6500_get_iic4_data_out test.\n");
    
    test = rand() % 256;
    res = mpu6500_set_iic4_data_out(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic4 data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set iic4 data out 0x%02X.\n", test);
    res = mpu6500_get_iic4_data_out(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get iic4 data out failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check iic4 data out %s.\n", test == test_check ? "ok" : "error");
    
    /* mpu6500_set_iic4_data_in/mpu6500_get_iic4_data_in test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_iic4_data_in/mpu6500_get_iic4_data_in test.\n");
    
    test = rand() % 256;
    res = mpu6500_set_iic4_data_in(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic4 data in failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set iic4 data in 0x%02X.\n", test);
    mpu6500_interface_debug_print("mpu6500: check iic4 data in %s.\n", res == 0 ? "ok" : "error");
    
    /* if spi interface */
    if (interface == MPU6500_INTERFACE_SPI)
    {
        /* mpu6500_set_disable_iic_slave/mpu6500_get_disable_iic_slave test */
        mpu6500_interface_debug_print("mpu6500: mpu6500_set_disable_iic_slave/mpu6500_get_disable_iic_slave test.\n");
        
        /* enable iic slave */
        res = mpu6500_set_disable_iic_slave(&gs_handle, MPU6500_BOOL_FALSE);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: set disable iic slave failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 0;
        }
        mpu6500_interface_debug_print("mpu6500: enable iic slave.\n");
        res = mpu6500_get_disable_iic_slave(&gs_handle, &enable);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: get disable iic slave failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 0;
        }
        mpu6500_interface_debug_print("mpu6500: check disable iic slave %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
        
        /* disable iic slave */
        res = mpu6500_set_disable_iic_slave(&gs_handle, MPU6500_BOOL_TRUE);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: set disable iic slave failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 0;
        }
        mpu6500_interface_debug_print("mpu6500: disable iic slave.\n");
        res = mpu6500_get_disable_iic_slave(&gs_handle, &enable);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: get disable iic slave failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 0;
        }
        mpu6500_interface_debug_print("mpu6500: check disable iic slave %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    }
    
    /* mpu6500_set_gyro_standby/mpu6500_get_gyro_standby test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyro_standby/mpu6500_get_gyro_standby test.\n");
    
    /* enable gyro standby */
    res = mpu6500_set_gyro_standby(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyro standby failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: enable gyro standby.\n");
    res = mpu6500_get_gyro_standby(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyro standby failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check gyro standby %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable gyro standby */
    res = mpu6500_set_gyro_standby(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyro standby failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: disable gyro standby.\n");
    res = mpu6500_get_gyro_standby(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyro standby failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check gyro standby %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_fifo_mode/mpu6500_get_fifo_mode test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_fifo_mode/mpu6500_get_fifo_mode test.\n");
    
    /* set fifo stream mode */
    res = mpu6500_set_fifo_mode(&gs_handle, MPU6500_FIFO_MODE_STREAM);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo stream mode.\n");
    res = mpu6500_get_fifo_mode(&gs_handle, &fifo_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo mode %s.\n", fifo_mode == MPU6500_FIFO_MODE_STREAM ? "ok" : "error");
    
    /* set fifo normal mode */
    res = mpu6500_set_fifo_mode(&gs_handle, MPU6500_FIFO_MODE_NORMAL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set fifo normal mode.\n");
    res = mpu6500_get_fifo_mode(&gs_handle, &fifo_mode);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get fifo mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo mode %s.\n", fifo_mode == MPU6500_FIFO_MODE_NORMAL ? "ok" : "error");
    
    /* mpu6500_set_gyroscope_choice/mpu6500_get_gyroscope_choice test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyroscope_choice/mpu6500_get_gyroscope_choice test.\n");
    
    test = rand() % 4;
    res = mpu6500_set_gyroscope_choice(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope choice failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set gyroscope choice 0x%02X.\n", test);
    res = mpu6500_get_gyroscope_choice(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyroscope choice failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check gyroscope choice %s.\n", test_check == test ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_choice/mpu6500_get_accelerometer_choice test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_choice/mpu6500_get_accelerometer_choice test.\n");
    
    test = rand() % 2;
    res = mpu6500_set_accelerometer_choice(&gs_handle, test);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer choice failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer choice 0x%02X.\n", test);
    res = mpu6500_get_accelerometer_choice(&gs_handle, &test_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer choice failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer choice %s.\n", test_check == test ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_low_pass_filter/mpu6500_get_accelerometer_low_pass_filter test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_low_pass_filter/mpu6500_get_accelerometer_low_pass_filter test.\n");
    
    /* accelerometer low pass filter 0 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_0);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter 0.\n");
    res = mpu6500_get_accelerometer_low_pass_filter(&gs_handle, &accelerometer_filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer low pass filter %s.\n", 
                                  accelerometer_filter == MPU6500_ACCELEROMETER_LOW_PASS_FILTER_0 ? "ok" : "error");
    
    /* accelerometer low pass filter 1 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_1);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter 1.\n");
    res = mpu6500_get_accelerometer_low_pass_filter(&gs_handle, &accelerometer_filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer low pass filter %s.\n", 
                                  accelerometer_filter == MPU6500_ACCELEROMETER_LOW_PASS_FILTER_1 ? "ok" : "error");
    
    /* accelerometer low pass filter 2 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_2);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter 2.\n");
    res = mpu6500_get_accelerometer_low_pass_filter(&gs_handle, &accelerometer_filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer low pass filter %s.\n", 
                                  accelerometer_filter == MPU6500_ACCELEROMETER_LOW_PASS_FILTER_2 ? "ok" : "error");
    
    /* accelerometer low pass filter 3 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_3);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter 3.\n");
    res = mpu6500_get_accelerometer_low_pass_filter(&gs_handle, &accelerometer_filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer low pass filter %s.\n", 
                                  accelerometer_filter == MPU6500_ACCELEROMETER_LOW_PASS_FILTER_3 ? "ok" : "error");
    
    /* accelerometer low pass filter 4 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_4);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter 4.\n");
    res = mpu6500_get_accelerometer_low_pass_filter(&gs_handle, &accelerometer_filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer low pass filter %s.\n", 
                                  accelerometer_filter == MPU6500_ACCELEROMETER_LOW_PASS_FILTER_4 ? "ok" : "error");
    
    /* accelerometer low pass filter 5 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_5);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter 5.\n");
    res = mpu6500_get_accelerometer_low_pass_filter(&gs_handle, &accelerometer_filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer low pass filter %s.\n", 
                                  accelerometer_filter == MPU6500_ACCELEROMETER_LOW_PASS_FILTER_5 ? "ok" : "error");
    
    /* accelerometer low pass filter 6 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_6);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter 6.\n");
    res = mpu6500_get_accelerometer_low_pass_filter(&gs_handle, &accelerometer_filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer low pass filter %s.\n", 
                                  accelerometer_filter == MPU6500_ACCELEROMETER_LOW_PASS_FILTER_6 ? "ok" : "error");
    
    /* accelerometer low pass filter 7 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_7);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter 7.\n");
    res = mpu6500_get_accelerometer_low_pass_filter(&gs_handle, &accelerometer_filter);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer low pass filter %s.\n", 
                                  accelerometer_filter == MPU6500_ACCELEROMETER_LOW_PASS_FILTER_7 ? "ok" : "error");
    
    /* mpu6500_set_low_power_accel_output_rate/mpu6500_get_low_power_accel_output_rate test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_low_power_accel_output_rate/mpu6500_get_low_power_accel_output_rate test.\n");
    
    /* set 0.24Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_0P24);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 0.24Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_0P24 ? "ok" : "error");
    
    /* set 0.49Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_0P49);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 0.49Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_0P49 ? "ok" : "error");
    
    /* set 0.98Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_0P98);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 0.98Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_0P98 ? "ok" : "error");
    
    /* set 1.95Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_1P95);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 1.95Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_1P95 ? "ok" : "error");
    
    /* set 3.91Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_3P91);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 3.91Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_3P91 ? "ok" : "error");
    
    /* set 7.81Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_7P81);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 7.81Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_7P81 ? "ok" : "error");
    
    /* set 15.63Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_15P63);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 15.63Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_15P63 ? "ok" : "error");
    
    /* set 31.25Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_31P25);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 31.25Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_31P25 ? "ok" : "error");
    
    /* set 62.50Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_62P50);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 62.50Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_62P50 ? "ok" : "error");
    
    /* set 125Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_125);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 125Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_125 ? "ok" : "error");
    
    /* set 250Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_250);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 250Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_250 ? "ok" : "error");
    
    /* set 500Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_500);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set low power accel output rate 500Hz.\n");
    res = mpu6500_get_low_power_accel_output_rate(&gs_handle, &rate);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check low power accel output rate %s.\n", 
                                  rate == MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_500 ? "ok" : "error");
    
    /* mpu6500_set_wake_on_motion/mpu6500_get_wake_on_motion test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_wake_on_motion/mpu6500_get_wake_on_motion test.\n");
    
    /* enable wake on motion */
    res = mpu6500_set_wake_on_motion(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set wake on motion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: enable wake on motion.\n");
    res = mpu6500_get_wake_on_motion(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get wake on motion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check wake on motion %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable wake on motion */
    res = mpu6500_set_wake_on_motion(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set wake on motion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: disable wake on motion.\n");
    res = mpu6500_get_wake_on_motion(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get wake on motion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check wake on motion %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_accel_compare_with_previous_sample/mpu6500_get_accel_compare_with_previous_sample test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accel_compare_with_previous_sample/mpu6500_get_accel_compare_with_previous_sample test.\n");
    
    /* enable accel compare with previous sample */
    res = mpu6500_set_accel_compare_with_previous_sample(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accel compare with previous sample failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: enable accel compare with previous sample.\n");
    res = mpu6500_get_accel_compare_with_previous_sample(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accel compare with previous sample failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accel compare with previous sample %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable accel compare with previous sample */
    res = mpu6500_set_accel_compare_with_previous_sample(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accel compare with previous sample failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: disable accel compare with previous sample.\n");
    res = mpu6500_get_accel_compare_with_previous_sample(&gs_handle, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accel compare with previous sample failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accel compare with previous sample %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_x_offset/mpu6500_get_accelerometer_x_offset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_x_offset/mpu6500_get_accelerometer_x_offset test.\n");
    
    offset = rand() % 32768;
    res = mpu6500_set_accelerometer_x_offset(&gs_handle, offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer x offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer x offset %d.\n", offset);
    res = mpu6500_get_accelerometer_x_offset(&gs_handle, &offset_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer x offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer x offset %s.\n", offset_check == offset ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_y_offset/mpu6500_get_accelerometer_y_offset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_y_offset/mpu6500_get_accelerometer_y_offset test.\n");
    
    offset = -(rand() % 32768);
    res = mpu6500_set_accelerometer_y_offset(&gs_handle, offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer y offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer y offset %d.\n", offset);
    res = mpu6500_get_accelerometer_y_offset(&gs_handle, &offset_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer y offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer y offset %s.\n", offset_check == offset ? "ok" : "error");
    
    /* mpu6500_set_accelerometer_z_offset/mpu6500_get_accelerometer_z_offset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_accelerometer_z_offset/mpu6500_get_accelerometer_z_offset test.\n");
    
    offset = rand() % 32768;
    res = mpu6500_set_accelerometer_z_offset(&gs_handle, offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer z offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set accelerometer z offset %d.\n", offset);
    res = mpu6500_get_accelerometer_z_offset(&gs_handle, &offset_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get accelerometer z offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer z offset %s.\n", offset_check == offset ? "ok" : "error");
    
    /* mpu6500_accelerometer_offset_convert_to_register/mpu6500_accelerometer_offset_convert_to_data test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_accelerometer_offset_convert_to_register/mpu6500_accelerometer_offset_convert_to_data test.\n");
    
    test_f = (float)(rand() % 1000) / 10.0f;
    res = mpu6500_accelerometer_offset_convert_to_register(&gs_handle, test_f, &offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: accelerometer offset convert to register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: accelerometer offset convert to register %0.2f.\n", test_f);
    res = mpu6500_accelerometer_offset_convert_to_data(&gs_handle, offset, &test_check_f);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: accelerometer offset convert to data failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check accelerometer offset %0.2f.\n", test_check_f);
    
    /* mpu6500_set_gyro_x_offset/mpu6500_get_gyro_x_offset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyro_x_offset/mpu6500_get_gyro_x_offset test.\n");
    
    offset = rand() % 32768;
    res = mpu6500_set_gyro_x_offset(&gs_handle, offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyro x offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set gyro x offset %d.\n", offset);
    res = mpu6500_get_gyro_x_offset(&gs_handle, &offset_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyro x offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check gyro x offset %s.\n", offset_check == offset ? "ok" : "error");
    
    /* mpu6500_set_gyro_y_offset/mpu6500_get_gyro_y_offset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyro_y_offset/mpu6500_get_gyro_y_offset test.\n");
    
    offset = -(rand() % 32768);
    res = mpu6500_set_gyro_y_offset(&gs_handle, offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyro y offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set gyro y offset %d.\n", offset);
    res = mpu6500_get_gyro_y_offset(&gs_handle, &offset_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyro y offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check gyro y offset %s.\n", offset_check == offset ? "ok" : "error");
    
    /* mpu6500_set_gyro_z_offset/mpu6500_get_gyro_z_offset test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_set_gyro_z_offset/mpu6500_get_gyro_z_offset test.\n");
    
    offset = rand() % 32768;
    res = mpu6500_set_gyro_z_offset(&gs_handle, offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyro z offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: set gyro z offset %d.\n", offset);
    res = mpu6500_get_gyro_z_offset(&gs_handle, &offset_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: get gyro z offset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check gyro z offset %s.\n", offset_check == offset ? "ok" : "error");
    
    /* mpu6500_gyro_offset_convert_to_register/mpu6500_gyro_offset_convert_to_data test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_gyro_offset_convert_to_register/mpu6500_gyro_offset_convert_to_data test.\n");
    
    test_f = (float)(rand() % 1000) / 10.0f;
    res = mpu6500_gyro_offset_convert_to_register(&gs_handle, test_f, &offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: gyro offset convert to register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: gyro offset convert to register %0.2f.\n", test_f);
    res = mpu6500_gyro_offset_convert_to_data(&gs_handle, offset, &test_check_f);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: gyro offset convert to data failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 0;
    }
    mpu6500_interface_debug_print("mpu6500: check gyro offset %0.2f.\n", test_check_f);
    
    /* finish register test */
    mpu6500_interface_debug_print("mpu6500: finish register test.\n");
    (void)mpu6500_deinit(&gs_handle);
    
    return 0;
}
