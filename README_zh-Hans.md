[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MPU6500

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mpu6500/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

MPU6500是一款6轴运动跟踪设备，它将3轴陀螺仪、3轴加速度计和数字运动处理器(DMP)组合在一个3.0x3.0x0.9mm的小封装中，它还具有512字节FIFO，可以降低串行总线接口流量，通过系统处理器突发读取传感器数据进入低功耗模式来降低功耗。通过其专用的IIC总线，它可以直接接受外部3轴磁力计的输入，提供完整的9轴运动融合输出。拥有6轴集成、数字运动处理器((DMP)和运行时校准固件的MPU6500运动跟踪设备，可以使制造商降低离散运动跟踪设备昂贵又复杂的选择、鉴定和系统集成成本，保证消费者产品的最佳运动跟踪性能。MPU6500还设计用于在其辅助IIC总线上与多个非惯性数字传感器（如压力传感器）的接口并具备自动FIFO采集功能。MPU6500具有3个用于陀螺仪输出的16位ADC和3个用于加速度计输出的16位ADC。用户可编程陀螺仪满标度范围为±250dps、±500dps、±1000dps和±2000dps，加速度计满标度范围为±2g、±4g、±8g和±16g。两个传感器的工厂校准初始灵敏度降低了生产线校准要求。其他业界领先的功能包括片上16位ADC、可编程数字滤波器、从-40°C到85°C漂移1%的精密时钟、嵌入式温度传感器和可编程中断。该设备具有I2C和SPI串行接口，VDD工作范围为1.71至3.6V，以及一个单独的数字IO电源，VDDIO范围为1.71V至3.6V。使用400kHz的I2C或1MHz的SPI与设备的所有寄存器进行通信。对于需要更快通信的应用，可以使用20MHz的SPI读取传感器和中断寄存器。InvenSense利用其获得专利并经过批量验证的CMOS-MEMS制造平台，通过晶圆级键将MEMS晶圆与配套的CMOS电子器件集成在一起，将封装尺寸缩小到3x3x0.90mm（24引脚QFN）的面积和厚度，以提供一个非常小但高性能的低成本封装。该设备通过10000g的冲击可靠性实验。

LibDriver MPU6500是LibDriver推出的MPU6500的全功能驱动，该驱动提供加速度读取、角速度读取、姿态角读取、DMP读取和敲击检测等功能并且它符合MISRA标准。

### 目录

  - [说明](#说明)
  - [安装](#安装)
  - [使用](#使用)
    - [example basic](#example-basic)
    - [example fifo](#example-fifo)
    - [example dmp](#example-dmp)
  - [文档](#文档)
  - [贡献](#贡献)
  - [版权](#版权)
  - [联系我们](#联系我们)

### 说明

/src目录包含了LibDriver MPU6500的源文件。

/interface目录包含了LibDriver MPU6500与平台无关的IIC，SPI总线模板。

/test目录包含了LibDriver MPU6500驱动测试程序，该程序可以简单的测试芯片必要功能。

/example目录包含了LibDriver MPU6500编程范例。

/doc目录包含了LibDriver MPU6500离线文档。

/datasheet目录包含了MPU6500数据手册。

/project目录包含了常用Linux与单片机开发板的工程样例。所有工程均采用shell脚本作为调试方法，详细内容可参考每个工程里面的README.md。

/misra目录包含了LibDriver MISRA代码扫描结果。

### 安装

参考/interface目录下与平台无关的IIC，SPI总线模板，完成指定平台的IIC，SPI总线驱动。

将/src目录，您使用平台的接口驱动和您开发的驱动加入工程，如果您想要使用默认的范例驱动，可以将/example目录加入您的工程。

### 使用

您可以参考/example目录下的编程范例完成适合您的驱动，如果您想要使用默认的编程范例，以下是它们的使用方法。

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

### 文档

在线文档: [https://www.libdriver.com/docs/mpu6500/index.html](https://www.libdriver.com/docs/mpu6500/index.html)。

离线文档: /doc/html/index.html。

### 贡献

请参考CONTRIBUTING.md。

### 版权

版权 (c) 2015 - 现在 LibDriver 版权所有

MIT 许可证（MIT）

特此免费授予任何获得本软件副本和相关文档文件（下称“软件”）的人不受限制地处置该软件的权利，包括不受限制地使用、复制、修改、合并、发布、分发、转授许可和/或出售该软件副本，以及再授权被配发了本软件的人如上的权利，须在下列条件下：

上述版权声明和本许可声明应包含在该软件的所有副本或实质成分中。

本软件是“如此”提供的，没有任何形式的明示或暗示的保证，包括但不限于对适销性、特定用途的适用性和不侵权的保证。在任何情况下，作者或版权持有人都不对任何索赔、损害或其他责任负责，无论这些追责来自合同、侵权或其它行为中，还是产生于、源于或有关于本软件以及本软件的使用或其它处置。

### 联系我们

请联系lishifenging@outlook.com。