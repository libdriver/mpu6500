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
 * @file      driver_mpu6500_dmp_tap_orient_motion_test.c
 * @brief     driver mpu6500 dmp tap orient motion test source file
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

#include "driver_mpu6500_dmp_tap_orient_motion_test.h"
#include <stdlib.h>

static mpu6500_handle_t gs_handle;        /**< mpu6500 handle */
static int16_t gs_accel_raw[128][3];      /**< accel raw buffer */
static float gs_accel_g[128][3];          /**< accel g buffer */
static int16_t gs_gyro_raw[128][3];       /**< gyro raw buffer */
static float gs_gyro_dps[128][3];         /**< gyro dps buffer */
static int32_t gs_quat[128][4];           /**< quat buffer */
static float gs_pitch[128];               /**< pitch buffer */
static float gs_roll[128];                /**< roll buffer */
static float gs_yaw[128];                 /**< yaw buffer */
static volatile uint8_t gs_flag;          /**< flag */

/**
 * @brief  dmp tap orient motion test irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t mpu6500_dmp_tap_orient_motion_test_irq_handler(void)
{
    if (mpu6500_irq_handler(&gs_handle) != 0)
    {
        return 1;
    }
    
    return 0;
}

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6500_INTERRUPT_MOTION :
        {
            gs_flag |= 1 << 0;
            mpu6500_interface_debug_print("mpu6500: irq motion.\n");
            
            break;
        }
        case MPU6500_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu6500_interface_debug_print("mpu6500: irq fifo overflow.\n");
            
            break;
        }
        case MPU6500_INTERRUPT_FSYNC_INT :
        {
            mpu6500_interface_debug_print("mpu6500: irq fsync int.\n");
            
            break;
        }
        case MPU6500_INTERRUPT_DMP :
        {
            break;
        }
        case MPU6500_INTERRUPT_DATA_READY :
        {
            break;
        }
        default :
        {
            mpu6500_interface_debug_print("mpu6500: irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count is the tap count
 * @param[in] direction is the tap direction
 * @note      none
 */
static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6500_DMP_TAP_X_UP :
        {
            gs_flag |= 1 << 1;
            mpu6500_interface_debug_print("mpu6500: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_X_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6500_interface_debug_print("mpu6500: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_Y_UP :
        {
            gs_flag |= 1 << 1;
            mpu6500_interface_debug_print("mpu6500: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_Y_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6500_interface_debug_print("mpu6500: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_Z_UP :
        {
            gs_flag |= 1 << 1;
            mpu6500_interface_debug_print("mpu6500: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_Z_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6500_interface_debug_print("mpu6500: tap irq z down with %d.\n", count);
            
            break;
        }
        default :
        {
            mpu6500_interface_debug_print("mpu6500: tap irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orient is the dmp orient
 * @note      none
 */
static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6500_DMP_ORIENT_PORTRAIT :
        {
            gs_flag |= 1 << 2;
            mpu6500_interface_debug_print("mpu6500: orient irq portrait.\n");
            
            break;
        }
        case MPU6500_DMP_ORIENT_LANDSCAPE :
        {
            gs_flag |= 1 << 2;
            mpu6500_interface_debug_print("mpu6500: orient irq landscape.\n");
            
            break;
        }
        case MPU6500_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            gs_flag |= 1 << 2;
            mpu6500_interface_debug_print("mpu6500: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6500_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
            gs_flag |= 1 << 2;
            mpu6500_interface_debug_print("mpu6500: orient irq reverse landscape.\n");
            
            break;
        }
        default :
        {
            mpu6500_interface_debug_print("mpu6500: orient irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     dmp test
 * @param[in] interface is the used interface
 * @param[in] addr is the iic device address
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      don't support spi interface
 */
uint8_t mpu6500_dmp_tap_orient_motion_test(mpu6500_interface_t interface, mpu6500_address_t addr)
{
    uint8_t res;
    uint8_t reg;
    uint32_t ms;
    uint32_t ms_check;
    uint32_t cnt;
    uint32_t cnt_check;
    uint32_t i;
    uint16_t m;
    uint16_t m_check;
    uint8_t c;
    uint8_t c_check;
    int32_t gyro_offset_raw[3];
    int32_t accel_offset_raw[3];
    int32_t gyro_offset[3];
    int32_t accel_offset[3];
    mpu6500_bool_t enable;
    mpu6500_info_t info;
    int8_t gyro_orientation[9] = {1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1};
    
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
    DRIVER_MPU6500_LINK_RECEIVE_CALLBACK(&gs_handle, a_receive_callback);
    
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
    
    /* start dmp tap orient motion test */
    mpu6500_interface_debug_print("mpu6500: start dmp tap orient motion test.\n");
    
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
    
    /* run the self test */
    res = mpu6500_self_test(&gs_handle, gyro_offset_raw, accel_offset_raw);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: self test failed.\n");
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
    res = mpu6500_set_sample_rate_divider(&gs_handle, 1000 / (50 - 1));
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sample rate divider failed.\n");
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
    
    /* ±2000dps */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_GYROSCOPE_RANGE_2000DPS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
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
    
    /* enable fifo */
    res = mpu6500_set_fifo(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo failed.\n");
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
    
    /* set 200mg */
    res = mpu6500_motion_threshold_convert_to_register(&gs_handle, 200.0f, &reg);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: motion threshold convert to register failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set the motion threshold */
    res = mpu6500_set_motion_threshold(&gs_handle, reg);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set motion threshold failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable motion */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_MOTION, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable fifo overflow */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FIFO_OVERFLOW, MPU6500_BOOL_TRUE);
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
    
    /* enable wake on motion */
    res = mpu6500_set_wake_on_motion(&gs_handle, MPU6500_BOOL_TRUE);
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
    
    /* load dmp firmware */
    mpu6500_interface_debug_print("mpu6500: load dmp firmware.\n");
    
    /* dmp load firmware */
    res = mpu6500_dmp_load_firmware(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp load firmware failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* load dmp firmware successful */
    mpu6500_interface_debug_print("mpu6500: load dmp firmware successful .\n");
    
    /* mpu6500_dmp_set_pedometer_walk_time/mpu6500_dmp_get_pedometer_walk_time test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_pedometer_walk_time/mpu6500_dmp_get_pedometer_walk_time test.\n");
    
    ms = 200;
    res = mpu6500_dmp_set_pedometer_walk_time(&gs_handle, ms);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set pedometer walk time failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set pedometer walk time %d ms.\n", ms);
    res = mpu6500_dmp_get_pedometer_walk_time(&gs_handle, &ms_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get pedometer walk time failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check pedometer walk time %s.\n", ms_check == ms ? "ok" : "error");
    
    /* mpu6500_dmp_set_pedometer_step_count/mpu6500_dmp_get_pedometer_step_count test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_pedometer_step_count/mpu6500_dmp_get_pedometer_step_count test.\n");
    
    cnt = rand() % 1000;
    res = mpu6500_dmp_set_pedometer_step_count(&gs_handle, cnt);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set pedometer step count failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set pedometer step count %d.\n", cnt);
    res = mpu6500_dmp_get_pedometer_step_count(&gs_handle, &cnt_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get pedometer step count failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check pedometer step count %s.\n", cnt_check == cnt ? "ok" : "error");
    
    /* mpu6500_dmp_set_shake_reject_timeout/mpu6500_dmp_get_shake_reject_timeout test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_shake_reject_timeout/mpu6500_dmp_get_shake_reject_timeout test.\n");
    
    m = 10;
    res = mpu6500_dmp_set_shake_reject_timeout(&gs_handle, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set shake reject timeout failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set shake reject timeout %d ms.\n", m);
    res = mpu6500_dmp_get_shake_reject_timeout(&gs_handle, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get shake reject timeout failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check shake reject timeout %s.\n", m_check == m ? "ok" : "error");
    
    /* mpu6500_dmp_set_shake_reject_time/mpu6500_dmp_get_shake_reject_time test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_shake_reject_time/mpu6500_dmp_get_shake_reject_time test.\n");
    
    m = 40;
    res = mpu6500_dmp_set_shake_reject_time(&gs_handle, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set shake reject time failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set shake reject time %d ms.\n", m);
    res = mpu6500_dmp_get_shake_reject_time(&gs_handle, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get shake reject time failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check shake reject time %s.\n", m_check == m ? "ok" : "error");
    
    /* mpu6500_dmp_set_shake_reject_thresh/mpu6500_dmp_get_shake_reject_thresh test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_shake_reject_thresh/mpu6500_dmp_get_shake_reject_thresh test.\n");
    
    m = 200;
    res = mpu6500_dmp_set_shake_reject_thresh(&gs_handle, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set shake reject thresh failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set shake reject thresh %d dps.\n", m);
    res = mpu6500_dmp_get_shake_reject_thresh(&gs_handle, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get shake reject thresh failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check shake reject thresh %s.\n", m_check == m ? "ok" : "error");
    
    /* mpu6500_dmp_set_tap_time_multi/mpu6500_dmp_get_tap_time_multi test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_tap_time_multi/mpu6500_dmp_get_tap_time_multi test.\n");
    
    m = 200;
    res = mpu6500_dmp_set_tap_time_multi(&gs_handle, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap time multi failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set tap time multi %d ms.\n", m);
    res = mpu6500_dmp_get_tap_time_multi(&gs_handle, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap time multi failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap time multi %s.\n", m_check == m ? "ok" : "error");
    
    /* mpu6500_dmp_set_tap_time/mpu6500_dmp_get_tap_time test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_tap_time/mpu6500_dmp_get_tap_time test.\n");
    
    m = 100;
    res = mpu6500_dmp_set_tap_time(&gs_handle, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap time failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set tap time %d ms.\n", m);
    res = mpu6500_dmp_get_tap_time(&gs_handle, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap time failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap time %s.\n", m_check == m ? "ok" : "error");
    
    /* mpu6500_dmp_set_min_tap_count/mpu6500_dmp_get_min_tap_count test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_min_tap_count/mpu6500_dmp_get_min_tap_count test.\n");
    
    c = 1;
    res = mpu6500_dmp_set_min_tap_count(&gs_handle, c);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set min tap count failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set min tap count %d.\n", c);
    res = mpu6500_dmp_get_min_tap_count(&gs_handle, &c_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get min tap count failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check min tap count %s.\n", c_check == c ? "ok" : "error");
    
    /* mpu6500_dmp_set_tap_axes/mpu6500_dmp_get_tap_axes test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_tap_axes/mpu6500_dmp_get_tap_axes test.\n");
    
    /* disable axis x */
    res = mpu6500_dmp_set_tap_axes(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable tap axes x.\n");
    res = mpu6500_dmp_get_tap_axes(&gs_handle, MPU6500_AXIS_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap axes %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable axis x */
    res = mpu6500_dmp_set_tap_axes(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable tap axes x.\n");
    res = mpu6500_dmp_get_tap_axes(&gs_handle, MPU6500_AXIS_X, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap axes %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable axis y */
    res = mpu6500_dmp_set_tap_axes(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable tap axes y.\n");
    res = mpu6500_dmp_get_tap_axes(&gs_handle, MPU6500_AXIS_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap axes %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable axis y */
    res = mpu6500_dmp_set_tap_axes(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable tap axes y.\n");
    res = mpu6500_dmp_get_tap_axes(&gs_handle, MPU6500_AXIS_Y, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap axes %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* disable axis z */
    res = mpu6500_dmp_set_tap_axes(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable tap axes z.\n");
    res = mpu6500_dmp_get_tap_axes(&gs_handle, MPU6500_AXIS_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap axes %s.\n", enable == MPU6500_BOOL_FALSE ? "ok" : "error");
    
    /* enable axis z */
    res = mpu6500_dmp_set_tap_axes(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable tap axes z.\n");
    res = mpu6500_dmp_get_tap_axes(&gs_handle, MPU6500_AXIS_Z, &enable);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap axes failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap axes %s.\n", enable == MPU6500_BOOL_TRUE ? "ok" : "error");
    
    /* mpu6500_dmp_set_tap_thresh/mpu6500_dmp_get_tap_thresh test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_tap_thresh/mpu6500_dmp_get_tap_thresh test.\n");
    
    /* set tap thresh x */
    m = 250;
    res = mpu6500_dmp_set_tap_thresh(&gs_handle, MPU6500_AXIS_X, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap thresh failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set tap thresh x %d mg/ms.\n", m);
    res = mpu6500_dmp_get_tap_thresh(&gs_handle, MPU6500_AXIS_X, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap thresh failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap thresh %s.\n", m_check == m ? "ok" : "error");
    
    /* set tap thresh y */
    m = 250;
    res = mpu6500_dmp_set_tap_thresh(&gs_handle, MPU6500_AXIS_Y, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap thresh failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set tap thresh y %d mg/ms.\n", m);
    res = mpu6500_dmp_get_tap_thresh(&gs_handle, MPU6500_AXIS_Y, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap thresh failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap thresh %s.\n", m_check == m ? "ok" : "error");
    
    /* set tap thresh z */
    m = 250;
    res = mpu6500_dmp_set_tap_thresh(&gs_handle, MPU6500_AXIS_Z, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap thresh failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set tap thresh z %d mg/ms.\n", m);
    res = mpu6500_dmp_get_tap_thresh(&gs_handle, MPU6500_AXIS_Z, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get tap thresh failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check tap thresh %s.\n", m_check == m ? "ok" : "error");
    
    /* mpu6500_dmp_set_fifo_rate/mpu6500_dmp_get_fifo_rate test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_fifo_rate/mpu6500_dmp_get_fifo_rate test.\n");
    
    m = 50;
    res = mpu6500_dmp_set_fifo_rate(&gs_handle, m);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set fifo rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set fifo rate %dHz.\n", m);
    res = mpu6500_dmp_get_fifo_rate(&gs_handle, &m_check);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp get fifo rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: check fifo rate %s.\n", m_check == m ? "ok" : "error");
    
    /* mpu6500_dmp_set_gyro_calibrate test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_gyro_calibrate test.\n");
    
    /* enable gyro calibrate */
    res = mpu6500_dmp_set_gyro_calibrate(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set gyro calibrate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable gyro calibrate.\n");
    
    /* disable gyro calibrate */
    res = mpu6500_dmp_set_gyro_calibrate(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set gyro calibrate failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable gyro calibrate.\n");
    
    /* mpu6500_dmp_set_3x_quaternion test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_3x_quaternion test.\n");
    
    /* enable 3x quaternion */
    res = mpu6500_dmp_set_3x_quaternion(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set 3x quaternion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable 3x quaternion.\n");
    
    /* disable 3x quaternion */
    res = mpu6500_dmp_set_3x_quaternion(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set 3x quaternion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable 3x quaternion.\n");
    
    /* mpu6500_dmp_set_6x_quaternion test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_6x_quaternion test.\n");
    
    /* enable 6x quaternion */
    res = mpu6500_dmp_set_6x_quaternion(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set 6x quaternion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable 6x quaternion.\n");
    
    /* disable 6x quaternion */
    res = mpu6500_dmp_set_6x_quaternion(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set 6x quaternion failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: disable 6x quaternion.\n");
    
    /* mpu6500_dmp_set_interrupt_mode test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_interrupt_mode test.\n");
    
    /* gesture mode */
    res = mpu6500_dmp_set_interrupt_mode(&gs_handle, MPU6500_DMP_INTERRUPT_MODE_GESTURE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set interrupt mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set gesture interrupt mode.\n");
    
    /* continuous mode */
    res = mpu6500_dmp_set_interrupt_mode(&gs_handle, MPU6500_DMP_INTERRUPT_MODE_CONTINUOUS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set interrupt mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: dmp set gesture continuous mode.\n");
    
    /* mpu6500_dmp_set_orientation test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_orientation test.\n");
    
    /* set the dmp orientation */
    res = mpu6500_dmp_set_orientation(&gs_handle, gyro_orientation);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set orientation failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: set the dmp orientation.\n");
    
    /* mpu6500_dmp_set_feature test */
    mpu6500_interface_debug_print("mpu6500: mpu6500_dmp_set_feature test.\n");
    
    /* enable feature */
    res = mpu6500_dmp_set_feature(&gs_handle, MPU6500_DMP_FEATURE_6X_QUAT | MPU6500_DMP_FEATURE_TAP | MPU6500_DMP_FEATURE_PEDOMETER |
                                              MPU6500_DMP_FEATURE_ORIENT | MPU6500_DMP_FEATURE_SEND_RAW_ACCEL |
                                              MPU6500_DMP_FEATURE_SEND_CAL_GYRO | MPU6500_DMP_FEATURE_GYRO_CAL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set feature failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    mpu6500_interface_debug_print("mpu6500: enable feature 6x quat.\n");
    mpu6500_interface_debug_print("mpu6500: enable feature tap.\n");
    mpu6500_interface_debug_print("mpu6500: enable feature pedometer.\n");
    mpu6500_interface_debug_print("mpu6500: enable feature orient.\n");
    mpu6500_interface_debug_print("mpu6500: enable feature send raw accel.\n");
    mpu6500_interface_debug_print("mpu6500: enable feature send cal gyro.\n");
    mpu6500_interface_debug_print("mpu6500: enable feature gyro cal.\n");
    
    /* dmp set tap callback */
    res = mpu6500_dmp_set_tap_callback(&gs_handle, a_dmp_tap_callback);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set tap callback failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* dmp set orient callback */
    res = mpu6500_dmp_set_orient_callback(&gs_handle, a_dmp_orient_callback);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set orient callback failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* dmp gyro accel raw offset convert */
    res = mpu6500_dmp_gyro_accel_raw_offset_convert(&gs_handle, gyro_offset_raw, accel_offset_raw, 
                                                    gyro_offset, accel_offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp gyro accel raw offset convert failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* dmp set accel bias */
    res = mpu6500_dmp_set_accel_bias(&gs_handle, accel_offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set accel bias failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* dmp set gyro bias */
    res = mpu6500_dmp_set_gyro_bias(&gs_handle, gyro_offset);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set gyro bias failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable the dmp */
    res = mpu6500_dmp_set_enable(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: dmp set enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* force reset the fifo */
    res = mpu6500_force_fifo_reset(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: force fifo reset failed.\n");
        (void)mpu6500_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 200 ms */
    mpu6500_interface_delay_ms(200);
    
    /* set 0 */
    gs_flag = 0;
    
    for (i = 0; i < 1000; i++)
    {
        uint16_t l;
        
        /* read the data */
        l = 128;
        res = mpu6500_dmp_read(&gs_handle, 
                               gs_accel_raw, gs_accel_g,
                               gs_gyro_raw, gs_gyro_dps,
                               gs_quat,
                               gs_pitch, gs_roll, gs_yaw,
                               &l
                              );
        if (res != 0)
        {
            /* output data */
            mpu6500_interface_debug_print("mpu6500: dmp read failed.\n");
        }
        mpu6500_interface_delay_ms(200);
        
        /* check the flag */
        if ((gs_flag & 0x7) == 0x7)
        {
            break;
        }
    }
    
    /* finish dmp tap orient motion test */
    mpu6500_interface_debug_print("mpu6500: finish dmp tap orient motion test.\n");
    (void)mpu6500_deinit(&gs_handle);
    
    return 0;
}
