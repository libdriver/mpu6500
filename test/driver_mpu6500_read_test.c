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
 * @file      driver_mpu6500_read_test.c
 * @brief     driver mpu6500 read test source file
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

#include "driver_mpu6500_read_test.h"

static mpu6500_handle_t gs_handle;        /**< mpu6500 handle */

/**
 * @brief     read test
 * @param[in] interface used interface
 * @param[in] addr iic device address
 * @param[in] times test times
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      none
 */
uint8_t mpu6500_read_test(mpu6500_interface_t interface, mpu6500_address_t addr, uint32_t times)
{
    uint8_t res;
    uint32_t i;
    mpu6500_info_t info;
    
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
    
    /* start read test */
    mpu6500_interface_debug_print("mpu6500: start read test.\n");
    
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
    
    /* set pll */
    res = mpu6500_set_clock_source(&gs_handle, MPU6500_CLOCK_SOURCE_PLL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set clock source failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set 50Hz */
    res = mpu6500_set_sample_rate_divider(&gs_handle, (1000 / 50) - 1);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sample rate divider failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable temperature sensor */
    res = mpu6500_set_ptat(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set ptat failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable cycle wake up */
    res = mpu6500_set_cycle_wake_up(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set cycle wake up failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable acc x */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable acc y */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable acc z */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable gyro x */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable gyro y */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable gyro z */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable gyroscope x test */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable gyroscope y test */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable gyroscope z test */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable accelerometer x test */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable accelerometer y test */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable accelerometer z test */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable fifo */
    res = mpu6500_set_fifo(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable temp fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_TEMP, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable xg fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_XG, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable yg fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_YG, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable zg fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_ZG, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable accel fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_ACCEL, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set interrupt level low */
    res = mpu6500_set_interrupt_level(&gs_handle, MPU6500_PIN_LEVEL_LOW);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* push-pull */
    res = mpu6500_set_interrupt_pin_type(&gs_handle, MPU6500_PIN_TYPE_PUSH_PULL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt pin type failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable motion */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_MOTION, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable fifo overflow */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FIFO_OVERFLOW, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable dmp interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_DMP, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable fsync int */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FSYNC_INT, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable data ready */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_DATA_READY, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable latch */
    res = mpu6500_set_interrupt_latch(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt latch failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable interrupt read clear */
    res = mpu6500_set_interrupt_read_clear(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt read clear failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable sync input */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_EXTERN_SYNC_INPUT_DISABLED);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable fsync interrupt */
    res = mpu6500_set_fsync_interrupt(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fsync interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* fsync interrupt level low */
    res = mpu6500_set_fsync_interrupt_level(&gs_handle, MPU6500_PIN_LEVEL_LOW);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fsync interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable iic master */
    res = mpu6500_set_iic_master(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic master failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable iic bypass */
    res = mpu6500_set_iic_bypass(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic bypass failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable gyro standby */
    res = mpu6500_set_gyro_standby(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyro standby failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set the fifo normal mode */
    res = mpu6500_set_fifo_mode(&gs_handle, MPU6500_FIFO_MODE_NORMAL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set gyroscope choice 0 */
    res = mpu6500_set_gyroscope_choice(&gs_handle, 0);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope choice failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set low pass filter 3 */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_LOW_PASS_FILTER_3);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set accelerometer choice 0 */
    res = mpu6500_set_accelerometer_choice(&gs_handle, 0);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer choice failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set accelerometer low pass filter 3 */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_ACCELEROMETER_LOW_PASS_FILTER_3);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set low power accel output rate 62.5Hz */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_62P50);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable wake on motion */
    res = mpu6500_set_wake_on_motion(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set wake on motion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable accel compare with previous sample */
    res = mpu6500_set_accel_compare_with_previous_sample(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accel compare with previous sample failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* ±2g */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_ACCELEROMETER_RANGE_2G);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    mpu6500_interface_debug_print("mpu6500: set accelerometer range 2g.\n");
    for (i = 0; i < times; i++)
    {
        int16_t accel_raw[1][3];
        float accel_g[1][3];
        int16_t gyro_raw[1][3];
        float gyro_dps[1][3];
        uint16_t len;
        
        len = 1;
        res = mpu6500_read(&gs_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: acc x is %0.2fg.\n", accel_g[0][0]);
        mpu6500_interface_debug_print("mpu6500: acc y is %0.2fg.\n", accel_g[0][1]);
        mpu6500_interface_debug_print("mpu6500: acc z is %0.2fg.\n", accel_g[0][2]);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    /* ±4g */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_ACCELEROMETER_RANGE_4G);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    mpu6500_interface_debug_print("mpu6500: set accelerometer range 4g.\n");
    for (i = 0; i < times; i++)
    {
        int16_t accel_raw[1][3];
        float accel_g[1][3];
        int16_t gyro_raw[1][3];
        float gyro_dps[1][3];
        uint16_t len;
        
        len = 1;
        res = mpu6500_read(&gs_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: acc x is %0.2fg.\n", accel_g[0][0]);
        mpu6500_interface_debug_print("mpu6500: acc y is %0.2fg.\n", accel_g[0][1]);
        mpu6500_interface_debug_print("mpu6500: acc z is %0.2fg.\n", accel_g[0][2]);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    /* ±8g */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_ACCELEROMETER_RANGE_8G);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    mpu6500_interface_debug_print("mpu6500: set accelerometer range 8g.\n");
    for (i = 0; i < times; i++)
    {
        int16_t accel_raw[1][3];
        float accel_g[1][3];
        int16_t gyro_raw[1][3];
        float gyro_dps[1][3];
        uint16_t len;
        
        len = 1;
        res = mpu6500_read(&gs_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: acc x is %0.2fg.\n", accel_g[0][0]);
        mpu6500_interface_debug_print("mpu6500: acc y is %0.2fg.\n", accel_g[0][1]);
        mpu6500_interface_debug_print("mpu6500: acc z is %0.2fg.\n", accel_g[0][2]);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    /* ±16g */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_ACCELEROMETER_RANGE_16G);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    mpu6500_interface_debug_print("mpu6500: set accelerometer range 16g.\n");
    for (i = 0; i < times; i++)
    {
        int16_t accel_raw[1][3];
        float accel_g[1][3];
        int16_t gyro_raw[1][3];
        float gyro_dps[1][3];
        uint16_t len;
        
        len = 1;
        res = mpu6500_read(&gs_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: acc x is %0.2fg.\n", accel_g[0][0]);
        mpu6500_interface_debug_print("mpu6500: acc y is %0.2fg.\n", accel_g[0][1]);
        mpu6500_interface_debug_print("mpu6500: acc z is %0.2fg.\n", accel_g[0][2]);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    /* ±250dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_250DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    mpu6500_interface_debug_print("mpu6500: set gyroscope range 250dps.\n");
    for (i = 0; i < times; i++)
    {
        int16_t accel_raw[1][3];
        float accel_g[1][3];
        int16_t gyro_raw[1][3];
        float gyro_dps[1][3];
        uint16_t len;
        
        len = 1;
        res = mpu6500_read(&gs_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: gyro x is %0.2fdps.\n", gyro_dps[0][0]);
        mpu6500_interface_debug_print("mpu6500: gyro y is %0.2fdps.\n", gyro_dps[0][1]);
        mpu6500_interface_debug_print("mpu6500: gyro z is %0.2fdps.\n", gyro_dps[0][2]);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    /* ±500dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_500DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    mpu6500_interface_debug_print("mpu6500: set gyroscope range 500dps.\n");
    for (i = 0; i < times; i++)
    {
        int16_t accel_raw[1][3];
        float accel_g[1][3];
        int16_t gyro_raw[1][3];
        float gyro_dps[1][3];
        uint16_t len;
        
        len = 1;
        res = mpu6500_read(&gs_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: gyro x is %0.2fdps.\n", gyro_dps[0][0]);
        mpu6500_interface_debug_print("mpu6500: gyro y is %0.2fdps.\n", gyro_dps[0][1]);
        mpu6500_interface_debug_print("mpu6500: gyro z is %0.2fdps.\n", gyro_dps[0][2]);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    /* ±1000dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_1000DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    mpu6500_interface_debug_print("mpu6500: set gyroscope range 1000dps.\n");
    for (i = 0; i < times; i++)
    {
        int16_t accel_raw[1][3];
        float accel_g[1][3];
        int16_t gyro_raw[1][3];
        float gyro_dps[1][3];
        uint16_t len;
        
        len = 1;
        res = mpu6500_read(&gs_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: gyro x is %0.2fdps.\n", gyro_dps[0][0]);
        mpu6500_interface_debug_print("mpu6500: gyro y is %0.2fdps.\n", gyro_dps[0][1]);
        mpu6500_interface_debug_print("mpu6500: gyro z is %0.2fdps.\n", gyro_dps[0][2]);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    /* ±2000dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_2000DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    mpu6500_interface_debug_print("mpu6500: set gyroscope range 2000dps.\n");
    for (i = 0; i < times; i++)
    {
        int16_t accel_raw[1][3];
        float accel_g[1][3];
        int16_t gyro_raw[1][3];
        float gyro_dps[1][3];
        uint16_t len;
        
        len = 1;
        res = mpu6500_read(&gs_handle, accel_raw, accel_g, gyro_raw, gyro_dps, &len);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: gyro x is %0.2fdps.\n", gyro_dps[0][0]);
        mpu6500_interface_debug_print("mpu6500: gyro y is %0.2fdps.\n", gyro_dps[0][1]);
        mpu6500_interface_debug_print("mpu6500: gyro z is %0.2fdps.\n", gyro_dps[0][2]);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    mpu6500_interface_debug_print("mpu6500: read temperature.\n");
    for (i = 0; i < times; i++)
    {
        int16_t raw;
        float degrees;
        
        /* read temperature */
        res = mpu6500_read_temperature(&gs_handle, &raw, &degrees);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: read temperature failed.\n");
            (void)mpu6500_deinit(&gs_handle);
           
            return 1;
        }
        mpu6500_interface_debug_print("mpu6500: temperature %0.2fC.\n", degrees);
        
        /* delay 1000 ms */
        mpu6500_interface_delay_ms(1000);
    }
    
    /* finish read test */
    mpu6500_interface_debug_print("mpu6500: finish read test.\n");
    (void)mpu6500_deinit(&gs_handle);
    
    return 0;
}
