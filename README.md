[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MPU6500

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mpu6500/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

The MPU-6500 is a 6-axis MotionTracking device that combines a 3-axis gyroscope, 3-axis accelerometer, and a Digital MotionProcessor® (DMP®) all in a small 3x3x0.9mm package. It also features a 512-byte FIFO that can lower the traffic on the serial bus interface, and reduce power consumption by allowing the system processor to burst read sensor data and then go into a low-power mode. With its dedicated I2C sensor bus, the MPU-6500 directly accepts inputs from external I2C devices. MPU-6500, with its 6-axis integration, on-chip DMP, and run-time calibration firmware, enables manufacturers to eliminate the costly and complex selection, qualification, and system level integration of discrete devices, guaranteeing optimal motion performance for consumers. MPU-6500 is also designed to interface with multiple non-inertial digital sensors, such as pressure sensors, on its auxiliary I2C port.The gyroscope has a programmable full-scale range of ±250, ±500, ±1000, and ±2000 degrees/sec and very low rate noise at 0.01 dps/√Hz. The accelerometer has a user-programmable accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g. Factory-calibrated initial sensitivity of both sensors reduces production-line calibration requirements.Other industry-leading features include on-chip 16-bit ADCs, programmable digital filters, a precision clock with 1% drift from -40°C to 85°C, an embedded temperature sensor, and programmable interrupts. The device features I2C and SPI serial interfaces, a VDD operating range of 1.71 to 3.6V, and a separate digital IO supply, VDDIO from 1.71V to 3.6V.Communication with all registers of the device is performed using either I2C at 400kHz or SPI at 1MHz. For applications requiring faster communications, the sensor and interrupt registers may be read using SPI at 20MHz.By leveraging its patented and volume-proven CMOS-MEMS fabrication platform, which integrates MEMS wafers with companion CMOS electronics through wafer-level bonding, InvenSense has driven the package size down to a footprint and thickness of 3x3x0.90mm (24-pin QFN), to provide a very small yet high performance low cost package. The device provides high robustness by supporting 10,000g shock reliability.

LibDriver MPU6500 is a full-featured driver for MPU6500, launched by LibDriver. It provides acceleration reading, angular velocity reading, attitude angle reading, dmp reading, tap detection and additional features. LibDriver is MISRA compliant.

### Table of Contents

  - [Instruction](#Instruction)
  - [Install](#Install)
  - [Usage](#Usage)
    - [example basic](#example-basic)
    - [example fifo](#example-fifo)
    - [example dmp](#example-dmp)
  - [Document](#Document)
  - [Contributing](#Contributing)
  - [License](#License)
  - [Contact Us](#Contact-Us)

### Instruction

/src includes LibDriver MPU6500 source files.

/interface includes LibDriver MPU6500 IIC, SPI platform independent template.

/test includes LibDriver MPU6500 driver test code and this code can test the chip necessary function simply.

/example includes LibDriver MPU6500 sample code.

/doc includes LibDriver MPU6500 offline document.

/datasheet includes MPU6500 datasheet.

/project includes the common Linux and MCU development board sample code. All projects use the shell script to debug the driver and the detail instruction can be found in each project's README.md.

/misra includes the LibDriver MISRA code scanning results.

### Install

Reference /interface IIC, SPI platform independent template and finish your platform IIC, SPI driver.

Add the /src directory, the interface driver for your platform, and your own drivers to your project, if you want to use the default example drivers, add the /example directory to your project.

### Usage

You can refer to the examples in the /example directory to complete your own driver. If you want to use the default programming examples, here's how to use them.

#### example basic

```C
#include "driver_mpu6500_basic.h"

uint8_t res;
uint32_t i;
uint32_t times;
float g[3];
float dps[3];
float degrees;
mpu6500_address_t addr;

/* init */
addr = MPU6500_ADDRESS_AD0_LOW;
res = mpu6500_basic_init(MPU6500_INTERFACE_IIC, addr);
if (res != 0)
{
    return 1;
}

...
    
/* read all */
times = 3;
for (i = 0; i < times; i++)
{
    /* read */
    if (mpu6500_basic_read(g, dps) != 0)
    {
        (void)mpu6500_basic_deinit();

        return 1;
    }

    ...
        
    if (mpu6500_basic_read_temperature(&degrees) != 0)
    {
        (void)mpu6500_basic_deinit();

        return 1;
    }

    ...
        
    /* output */
    mpu6500_interface_debug_print("mpu6500: %d/%d.\n", i + 1, times);
    mpu6500_interface_debug_print("mpu6500: acc x is %0.2fg.\n", g[0]);
    mpu6500_interface_debug_print("mpu6500: acc y is %0.2fg.\n", g[1]);
    mpu6500_interface_debug_print("mpu6500: acc z is %0.2fg.\n", g[2]);
    mpu6500_interface_debug_print("mpu6500: gyro x is %0.2fdps.\n", dps[0]);
    mpu6500_interface_debug_print("mpu6500: gyro y is %0.2fdps.\n", dps[1]);
    mpu6500_interface_debug_print("mpu6500: gyro z is %0.2fdps.\n", dps[2]);
    mpu6500_interface_debug_print("mpu6500: temperature %0.2fC.\n", degrees);

    ...
        
    /* delay 1000 ms */
    mpu6500_interface_delay_ms(1000);

    ...
}

...
    
/* deinit */
(void)mpu6500_basic_deinit();

return 0;
```

#### example fifo

```C
#include "driver_mpu6500_fifo.h"

uint32_t i;
uint32_t times;
uint16_t len;
uint8_t (*g_gpio_irq)(void) = NULL;
static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];
static float gs_gyro_dps[128][3];
mpu6500_address_t addr;

/* gpio init */
if (gpio_interrupt_init() != 0)
{
    return 1;
}
g_gpio_irq = mpu6500_fifo_irq_handler;

/* init */
addr = MPU6500_ADDRESS_AD0_LOW;
if (mpu6500_fifo_init(MPU6500_INTERFACE_IIC, addr) != 0)
{
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();

    return 1;
}

/* delay 100 ms */
mpu6500_interface_delay_ms(100);

...

times = 3;
for (i = 0; i < times; i++)
{
    len = 128;

    /* read */
    if (mpu6500_fifo_read(gs_accel_raw, gs_accel_g,
                          gs_gyro_raw, gs_gyro_dps, &len) != 0)
    {
        (void)mpu6500_fifo_deinit();
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        return 1;
    }
    
    ...
        
    /* output */
    mpu6500_interface_debug_print("mpu6500: %d/%d.\n", i + 1, times);
    mpu6500_interface_debug_print("mpu6500: fifo %d.\n", len);
    mpu6500_interface_debug_print("mpu6500: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
    mpu6500_interface_debug_print("mpu6500: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
    mpu6500_interface_debug_print("mpu6500: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
    mpu6500_interface_debug_print("mpu6500: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
    mpu6500_interface_debug_print("mpu6500: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
    mpu6500_interface_debug_print("mpu6500: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);
    
    ...
        
    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);
    
    ...
}

...
    
/* deinit */
(void)mpu6500_fifo_deinit();
g_gpio_irq = NULL;
(void)gpio_interrupt_deinit();

return 0;
```

#### example dmp

```C
#include "driver_mpu6500_dmp.h"

uint32_t i;
uint32_t times;
uint32_t cnt;
uint16_t len;
uint8_t (*g_gpio_irq)(void) = NULL;
static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];      
static float gs_gyro_dps[128][3];
static int32_t gs_quat[128][4];          
static float gs_pitch[128];              
static float gs_roll[128];                
static float gs_yaw[128];                      
mpu6500_address_t addr;

static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6500_INTERRUPT_MOTION :
        {
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
            mpu6500_interface_debug_print("mpu6500: irq dmp\n");
            
            break;
        }
        case MPU6500_INTERRUPT_DATA_READY :
        {
            mpu6500_interface_debug_print("mpu6500: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu6500_interface_debug_print("mpu6500: irq unknown code.\n");
            
            break;
        }
    }
}

static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6500_DMP_TAP_X_UP :
        {
            mpu6500_interface_debug_print("mpu6500: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_X_DOWN :
        {
            mpu6500_interface_debug_print("mpu6500: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_Y_UP :
        {
            mpu6500_interface_debug_print("mpu6500: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_Y_DOWN :
        {
            mpu6500_interface_debug_print("mpu6500: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_Z_UP :
        {
            mpu6500_interface_debug_print("mpu6500: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6500_DMP_TAP_Z_DOWN :
        {
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

static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6500_DMP_ORIENT_PORTRAIT :
        {
            mpu6500_interface_debug_print("mpu6500: orient irq portrait.\n");
            
            break;
        }
        case MPU6500_DMP_ORIENT_LANDSCAPE :
        {
            mpu6500_interface_debug_print("mpu6500: orient irq landscape.\n");
            
            break;
        }
        case MPU6500_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            mpu6500_interface_debug_print("mpu6500: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6500_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
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

/* init */
if (gpio_interrupt_init() != 0)
{
    return 1;
}
g_gpio_irq = mpu6500_dmp_irq_handler;

/* init */
addr = MPU6500_ADDRESS_AD0_LOW;
if (mpu6500_dmp_init(MPU6500_INTERFACE_IIC, addr, a_receive_callback, 
                     a_dmp_tap_callback, a_dmp_orient_callback) != 0)
{
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();

    return 1;
}

/* delay 500 ms */
mpu6500_interface_delay_ms(500);

...
    
times = 3;
for (i = 0; i < times; i++)
{
    len = 128;

    /* read */
    if (mpu6500_dmp_read_all(gs_accel_raw, gs_accel_g,
                             gs_gyro_raw, gs_gyro_dps, 
                             gs_quat,
                             gs_pitch, gs_roll, gs_yaw,
                             &len) != 0)
    {
        (void)mpu6500_dmp_deinit();
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        return 1;
    }

    /* output */
    mpu6500_interface_debug_print("mpu6500: %d/%d.\n", i + 1, times);
    mpu6500_interface_debug_print("mpu6500: fifo %d.\n", len);
    mpu6500_interface_debug_print("mpu6500: pitch[0] is %0.2fdeg.\n", gs_pitch[0]);
    mpu6500_interface_debug_print("mpu6500: roll[0] is %0.2fdeg.\n", gs_roll[0]);
    mpu6500_interface_debug_print("mpu6500: yaw[0] is %0.2fdeg.\n", gs_yaw[0]);
    mpu6500_interface_debug_print("mpu6500: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
    mpu6500_interface_debug_print("mpu6500: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
    mpu6500_interface_debug_print("mpu6500: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
    mpu6500_interface_debug_print("mpu6500: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
    mpu6500_interface_debug_print("mpu6500: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
    mpu6500_interface_debug_print("mpu6500: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);

    /* delay 500 ms */
    mpu6500_interface_delay_ms(500);
    
    ....
        
    /* get the pedometer step count */
    res = mpu6500_dmp_get_pedometer_counter(&cnt);
    if (res != 0)
    {
        (void)mpu6500_dmp_deinit();
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        return 1;
    }
    
    ...
}

...

/* deinit */
(void)mpu6500_dmp_deinit();
g_gpio_irq = NULL;
(void)gpio_interrupt_deinit();

return 0;
```

### Document

Online documents: [https://www.libdriver.com/docs/mpu6500/index.html](https://www.libdriver.com/docs/mpu6500/index.html).

Offline documents: /doc/html/index.html.

### Contributing

Please refer to CONTRIBUTING.md.

### License

Copyright (c) 2015 - present LibDriver All rights reserved



The MIT License (MIT) 



Permission is hereby granted, free of charge, to any person obtaining a copy

of this software and associated documentation files (the "Software"), to deal

in the Software without restriction, including without limitation the rights

to use, copy, modify, merge, publish, distribute, sublicense, and/or sell

copies of the Software, and to permit persons to whom the Software is

furnished to do so, subject to the following conditions: 



The above copyright notice and this permission notice shall be included in all

copies or substantial portions of the Software. 



THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR

IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,

FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE

AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER

LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,

OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE

SOFTWARE. 

### Contact Us

Please send an e-mail to lishifenging@outlook.com.