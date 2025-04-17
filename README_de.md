[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MPU6500

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mpu6500/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE) 

Der MPU-6500 ist ein 6-Achsen-MotionTracking-Gerät, das ein 3-Achsen-Gyroskop, einen 3-Achsen-Beschleunigungsmesser und einen Digital MotionProcessor® (DMP®) in einem kleinen 3x3x0,9-mm-Gehäuse vereint. Er verfügt außerdem über ein 512-Byte-FIFO, das den Datenverkehr auf der seriellen Busschnittstelle verringern und den Stromverbrauch reduzieren kann, indem der Systemprozessor Sensordaten in Burst-Schritten lesen und dann in einen Energiesparmodus wechseln kann. Mit seinem dedizierten I2C-Sensorbus akzeptiert der MPU-6500 direkt Eingaben von externen I2C-Geräten. Mit seiner 6-Achsen-Integration, dem On-Chip-DMP und der Laufzeitkalibrierungs-Firmware ermöglicht der MPU-6500 Herstellern, die kostspielige und komplexe Auswahl, Qualifizierung und Integration diskreter Geräte auf Systemebene zu vermeiden und so eine optimale Bewegungsleistung für Verbraucher zu gewährleisten. MPU-6500 ist auch für die Verbindung mit mehreren nicht-inertialen digitalen Sensoren, wie z. B. Drucksensoren, über seinen zusätzlichen I2C-Anschluss ausgelegt. Das Gyroskop hat einen programmierbaren Vollausschlagsbereich von ±250, ±500, ±1000 und ±2000 Grad/Sek. und ein sehr geringes Rauschen bei 0,01 dps/√Hz. Der Beschleunigungsmesser hat einen benutzerprogrammierbaren Vollausschlagsbereich von ±2g, ±4g, ±8g und ±16g. Die werkseitig kalibrierte Anfangsempfindlichkeit beider Sensoren reduziert die Anforderungen an die Kalibrierung in der Produktionslinie. Zu den weiteren branchenführenden Funktionen gehören integrierte 16-Bit-ADCs, programmierbare digitale Filter, eine Präzisionsuhr mit 1 % Drift von -40 °C bis 85 °C, ein eingebetteter Temperatursensor und programmierbare Interrupts. Das Gerät verfügt über serielle I2C- und SPI-Schnittstellen, einen VDD-Betriebsbereich von 1,71 bis 3,6 V und eine separate digitale IO-Versorgung, VDDIO von 1,71 V bis 3,6 V. Die Kommunikation mit allen Registern des Geräts erfolgt entweder über I2C bei 400 kHz oder SPI bei 1 MHz. Für Anwendungen, die eine schnellere Kommunikation erfordern, können die Sensor- und Interrupt-Register über SPI bei 20 MHz gelesen werden. Durch die Nutzung seiner patentierten und volumenerprobten CMOS-MEMS-Fertigungsplattform, die MEMS-Wafer mit zugehöriger CMOS-Elektronik durch Wafer-Level-Bonding integriert, hat InvenSense die Gehäusegröße auf eine Grundfläche und Dicke von 3 x 3 x 0,90 mm (24-poliges QFN) reduziert, um ein sehr kleines, aber dennoch leistungsstarkes und kostengünstiges Gehäuse bereitzustellen. Das Gerät bietet eine hohe Robustheit, da es eine Stoßfestigkeit von 10.000 g unterstützt.

LibDriver MPU6500 ist der voll funktionsfähige Treiber von MPU6500, der von LibDriver eingeführt wurde. Er bietet Beschleunigungs-, Winkelgeschwindigkeits-, Lagewinkel- und DMP-Messwerte, Tipperkennung und andere Funktionen. LibDriver ist MISRA-kompatibel.

### Inhaltsverzeichnis

  - [Anweisung](#Anweisung)
  - [Installieren](#Installieren)
  - [Nutzung](#Nutzung)
    - [example basic](#example-basic)
    - [example fifo](#example-fifo)
    - [example dmp](#example-dmp)
  - [Dokument](#Dokument)
  - [Beitrag](#Beitrag)
  - [Lizenz](#Lizenz)
  - [Kontaktieren Sie uns](#Kontaktieren-Sie-uns)

### Anweisung

/src enthält LibDriver MPU6500-Quelldateien.

/interface enthält die plattformunabhängige Vorlage LibDriver MPU6500 IIC, SPI.

/test enthält den Testcode des LibDriver MPU6500-Treibers und dieser Code kann die erforderliche Funktion des Chips einfach testen.

/example enthält LibDriver MPU6500-Beispielcode.

/doc enthält das LibDriver MPU6500-Offlinedokument.

/Datenblatt enthält MPU6500-Datenblatt.

/project enthält den allgemeinen Beispielcode für Linux- und MCU-Entwicklungsboards. Alle Projekte verwenden das Shell-Skript, um den Treiber zu debuggen, und die detaillierten Anweisungen finden Sie in der README.md jedes Projekts.

/misra enthält die Ergebnisse des LibDriver MISRA Code Scans.

### Installieren

Verweisen Sie auf eine plattformunabhängige IIC, SPI-Schnittstellenvorlage und stellen Sie Ihren Plattform-IIC, SPI-Treiber fertig.

Fügen Sie das Verzeichnis /src, den Schnittstellentreiber für Ihre Plattform und Ihre eigenen Treiber zu Ihrem Projekt hinzu. Wenn Sie die Standardbeispieltreiber verwenden möchten, fügen Sie das Verzeichnis /example zu Ihrem Projekt hinzu.

### Nutzung

Sie können auf die Beispiele im Verzeichnis /example zurückgreifen, um Ihren eigenen Treiber zu vervollständigen. Wenn Sie die Standardprogrammierbeispiele verwenden möchten, erfahren Sie hier, wie Sie diese verwenden.

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

### Dokument

Online-Dokumente: [https://www.libdriver.com/docs/mpu6500/index.html](https://www.libdriver.com/docs/mpu6500/index.html).

Offline-Dokumente: /doc/html/index.html.

### Beitrag

Bitte beachten Sie CONTRIBUTING.md.

### Lizenz

Urheberrechte © (c) 2015 - Gegenwart LibDriver Alle Rechte vorbehalten



Die MIT-Lizenz (MIT)



Hiermit wird jeder Person kostenlos die Erlaubnis erteilt, eine Kopie zu erhalten

dieser Software und zugehörigen Dokumentationsdateien (die „Software“) zu behandeln

in der Software ohne Einschränkung, einschließlich, aber nicht beschränkt auf die Rechte

zu verwenden, zu kopieren, zu modifizieren, zusammenzuführen, zu veröffentlichen, zu verteilen, unterzulizenzieren und/oder zu verkaufen

Kopien der Software und Personen, denen die Software gehört, zu gestatten

dazu eingerichtet werden, unter folgenden Bedingungen:



Der obige Urheberrechtshinweis und dieser Genehmigungshinweis müssen in allen enthalten sein

Kopien oder wesentliche Teile der Software.



DIE SOFTWARE WIRD "WIE BESEHEN" BEREITGESTELLT, OHNE JEGLICHE GEWÄHRLEISTUNG, AUSDRÜCKLICH ODER

STILLSCHWEIGEND, EINSCHLIESSLICH, ABER NICHT BESCHRÄNKT AUF DIE GEWÄHRLEISTUNG DER MARKTGÄNGIGKEIT,

EIGNUNG FÜR EINEN BESTIMMTEN ZWECK UND NICHTVERLETZUNG VON RECHTEN DRITTER. IN KEINEM FALL DARF DAS

AUTOREN ODER URHEBERRECHTSINHABER HAFTEN FÜR JEGLICHE ANSPRÜCHE, SCHÄDEN ODER ANDERE

HAFTUNG, OB AUS VERTRAG, DELIKT ODER ANDERWEITIG, ENTSTEHEND AUS,

AUS ODER IM ZUSAMMENHANG MIT DER SOFTWARE ODER DER VERWENDUNG ODER ANDEREN HANDLUNGEN MIT DER

SOFTWARE.

### Kontaktieren Sie uns

Bitte senden Sie eine E-Mail an lishifenging@outlook.com.