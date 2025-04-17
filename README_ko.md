[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MPU6500

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mpu6500/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

PU-6500은 3축 자이로스코프, 3축 가속도, 디지털 모션 프로세서를 탑재한 6축 모션 추적 장치입니다.®（DMP®）3x3x0.9mm의 작은 패키지에 조합됩니다.또한 512바이트 FIFO를 사용하여 직렬 버스 인터페이스의 트래픽을 줄이고 시스템 프로세서가 센서 데이터를 갑자기 읽은 다음 저전력 모드로 전환하여 전력 소비량을 줄일 수 있습니다.전용 I2C 센서 버스를 통해 MPU-6500은 외부 I2C 장치로부터 직접 입력을 받습니다.PU-6500은 6축 통합, 슬라이스 DMP 및 런타임 교정 펌웨어를 갖추고 있어 제조업체가 비싸고 복잡한 개별 장치의 선택, 검증 및 시스템 수준의 통합을 제거하고 소비자의 최적의 운동 성능을 보장합니다.PU-6500은 또한 보조 I2C 포트에서 압력 센서와 같은 여러 비관성 디지털 센서 인터페이스와 연결하도록 설계되었습니다.자이로스코프의 프로그래밍 가능한 전체 범위는 ± 250, ± 500, ± 1000 및 ± 2000도/초이며 0.01dps/Hz에서 매우 낮은 소음률을 가지고 있습니다.가속도계의 사용자 프로그래밍 가능한 가속도계 전체 범위는 ±2g, ±4g, ±8g, ±16g이다.두 센서의 공장 교정 초기 감도는 생산 라인 교정 요구 사항을 낮춥니다.기타 업계 최고의 기능으로는 슬라이스 16비트 ADC, 프로그래밍 가능한 디지털 필터, -40°C에서 85°C로 1% 드리프트 가능한 정밀 클럭, 내장형 온도 센서 및 프로그래밍 중단 등이 있습니다.이 장치에는 I2C 및 SPI 직렬 인터페이스, VDD 1.71-3.6V 및 별도의 디지털 IO 전원 공급 장치, VDDIO 1.71V-3.6V 범위가 있습니다.400kHz의 I2C 또는 1MHz의 SPI를 사용하여 디바이스의 모든 레지스터와 통신합니다.더 빠른 통신이 필요한 애플리케이션의 경우 20MHz의 SPI 읽기 센서와 인터럽트 레지스터를 사용할 수 있습니다.InvenSense는 특허를 획득하고 대량 검증을 거친 CMOS-MEMS 제조 플랫폼을 이용하여 MEMS 웨이퍼를 결합된 CMOS 전자 부품과 통합하여 3x3x0.90mm(24핀 QFN)의 설치 면적과 두께로 패키징 크기를 축소하여 매우 작지만 고성능의 저비용 패키징을 제공한다.이 장치는 10000g의 충격 안정성을 지원함으로써 높은 노봉성을 제공합니다.

LibDriver MPU6500은 LibDriver가 출시한 MPU6500의 전체 기능 드라이버입니다. 가속도 판독, 각속도 판독, 자세각 판독, dmp 판독, 탭 감지 및 기타 기능을 제공합니다. LibDriver는 MISRA를 준수합니다.

### 콘텐츠

  - [설명](#설명)
  - [설치](#설치)
  - [사용](#사용)
    - [example basic](#example-basic)
    - [example fifo](#example-fifo)
    - [example dmp](#example-dmp)
  - [문서](#문서)
  - [기고](#기고)
  - [저작권](#저작권)
  - [문의하기](#문의하기)

### 설명

/src 디렉토리에는 LibDriver MPU6500의 소스 파일이 포함되어 있습니다.

/interface 디렉토리에는 LibDriver MPU6500용 플랫폼 독립적인 IIC, SPI버스 템플릿이 포함되어 있습니다.

/test 디렉토리에는 LibDriver MPU6500드라이버 테스트 프로그램이 포함되어 있어 칩의 필요한 기능을 간단히 테스트할 수 있습니다.

/example 디렉토리에는 LibDriver MPU6500프로그래밍 예제가 포함되어 있습니다.

/doc 디렉토리에는 LibDriver MPU6500오프라인 문서가 포함되어 있습니다.

/datasheet 디렉토리에는 MPU6500데이터시트가 있습니다.

/project 디렉토리에는 일반적으로 사용되는 Linux 및 마이크로컨트롤러 개발 보드의 프로젝트 샘플이 포함되어 있습니다. 모든 프로젝트는 디버깅 방법으로 셸 스크립트를 사용하며, 자세한 내용은 각 프로젝트의 README.md를 참조하십시오.

/misra 에는 LibDriver misra 코드 검색 결과가 포함됩니다.

### 설치

/interface 디렉토리에서 플랫폼 독립적인IIC, SPI버스 템플릿을 참조하여 지정된 플랫폼에 대한 IIC, SPI버스 드라이버를 완성하십시오.

/src 디렉터리, 플랫폼용 인터페이스 드라이버 및 자체 드라이버를 프로젝트에 추가합니다. 기본 예제 드라이버를 사용하려면 /example 디렉터리를 프로젝트에 추가합니다.

### 사용

/example 디렉터리의 예제를 참조하여 자신만의 드라이버를 완성할 수 있습니다. 기본 프로그래밍 예제를 사용하려는 경우 사용 방법은 다음과 같습니다.

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

```c
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

### 문서

온라인 문서: [https://www.libdriver.com/docs/mpu6500/index.html](https://www.libdriver.com/docs/mpu6500/index.html).

오프라인 문서: /doc/html/index.html.

### 기고

CONTRIBUTING.md 를 참조하십시오.

### 저작권

저작권 (c) 2015 - 지금 LibDriver 판권 소유

MIT 라이선스(MIT)

이 소프트웨어 및 관련 문서 파일("소프트웨어")의 사본을 얻은 모든 사람은 이에 따라 무제한 사용, 복제, 수정, 통합, 출판, 배포, 2차 라이선스를 포함하여 소프트웨어를 처분할 수 있는 권리가 부여됩니다. 소프트웨어의 사본에 대한 라이선스 및/또는 판매, 그리고 소프트웨어가 위와 같이 배포된 사람의 권리에 대한 2차 라이선스는 다음 조건에 따릅니다.

위의 저작권 표시 및 이 허가 표시는 이 소프트웨어의 모든 사본 또는 내용에 포함됩니다.

이 소프트웨어는 상품성, 특정 목적에의 적합성 및 비침해에 대한 보증을 포함하되 이에 국한되지 않는 어떠한 종류의 명시적 또는 묵시적 보증 없이 "있는 그대로" 제공됩니다. 어떤 경우에도 저자 또는 저작권 소유자는 계약, 불법 행위 또는 기타 방식에 관계없이 소프트웨어 및 기타 소프트웨어 사용으로 인해 발생하거나 이와 관련하여 발생하는 청구, 손해 또는 기타 책임에 대해 책임을 지지 않습니다.

### 문의하기

연락주세요lishifenging@outlook.com.