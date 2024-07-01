### 1. Board

#### 1.1 Board Info

Board Name: Raspberry Pi 4B.

IIC Pin: SCL/SDA GPIO3/GPIO2.

SPI Pin: SCLK/MOSI/MISO/CS GPIO11/GPIO10/GPIO9/GPIO8.

GPIO Pin: INT GPIO17.

### 2. Install

#### 2.1 Dependencies

Install the necessary dependencies.

```shell
sudo apt-get install libgpiod-dev pkg-config cmake -y
```

#### 2.2 Makefile

Build the project.

```shell
make
```

Install the project and this is optional.

```shell
sudo make install
```

Uninstall the project and this is optional.

```shell
sudo make uninstall
```

#### 2.3 CMake

Build the project.

```shell
mkdir build && cd build 
cmake .. 
make
```

Install the project and this is optional.

```shell
sudo make install
```

Uninstall the project and this is optional.

```shell
sudo make uninstall
```

Test the project and this is optional.

```shell
make test
```

Find the compiled library in CMake. 

```cmake
find_package(mpu6500 REQUIRED)
```


### 3. MPU6500

#### 3.1 Command Instruction

1. Show mpu6500 chip and driver information.

   ```shell
   mpu6500 (-i | --information)
   ```

2. Show mpu6500 help.

   ```shell
   mpu6500 (-h | --help)
   ```

3. Show mpu6500 pin connections of the current board.

   ```shell
   mpu6500 (-p | --port)
   ```

4. Run mpu6500 register test.

   ```shell
   mpu6500 (-t reg | --test=reg) [--addr=<0 | 1>] [--interface=<iic | spi>]
   ```

5. Run mpu6500 read test, num means the test times.

   ```shell
   mpu6500 (-t read | --test=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
   ```

6. Run mpu6500 fifo test, num means the test times.

   ```shell
   mpu6500 (-t fifo | --test=fifo) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
   ```

7. Run mpu6500 dmp test, num means the test times.

   ```shell
   mpu6500 (-t dmp | --test=dmp) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
   ```

8. Run mpu6500 motion test.

   ```shell
   mpu6500 (-t motion | --test=motion) [--addr=<0 | 1>] [--interface=<iic | spi>]
   ```

9. Run mpu6500 pedometer test, num means the test times.

   ```shell
   mpu6500 (-t pedometer | --test=pedometer) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
   ```

10. Run mpu6500 read function, num means the read times.

    ```shell
    mpu6500 (-e read | --example=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
    ```

11. Run mpu6500 fifo function, num means the read times.

    ```shell
    mpu6500 (-e fifo | --example=fifo) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
    ```

12. Run mpu6500 dmp function, num means the read times.

    ```shell
    mpu6500 (-e dmp | --example=dmp) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
    ```

13. Run mpu6500 motion function.

    ```shell
    mpu6500 (-e motion | --example=motion) [--addr=<0 | 1>] [--interface=<iic | spi>]
    ```

14. Run mpu6500 pedometer function, num means the read times.

    ```shell
    mpu6500 (-e pedometer | --example=pedometer) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
    ```

#### 3.2 Command Example

```shell
./mpu6500 -i

mpu6500: chip is TDK MPU6500.
mpu6500: manufacturer is TDK.
mpu6500: interface is IIC SPI.
mpu6500: driver version is 1.0.
mpu6500: min supply voltage is 1.7V.
mpu6500: max supply voltage is 3.5V.
mpu6500: max current is 3.20mA.
mpu6500: max temperature is 85.0C.
mpu6500: min temperature is -40.0C.
```

```shell
./mpu6500 -p

mpu6500: SPI interface SCK connected to GPIO11.
mpu6500: SPI interface MISO connected to GPIO9.
mpu6500: SPI interface MOSI connected to GPIO10.
mpu6500: SPI interface CS connected to GPIO8.
mpu6500: SCL connected to GPIO3.
mpu6500: SDA connected to GPIO2.
mpu6500: INT connected to GPIO17.
```

```shell
./mpu6500 -t reg --addr=0 --interface=iic

mpu6500: chip is TDK MPU6500.
mpu6500: manufacturer is TDK.
mpu6500: interface is IIC SPI.
mpu6500: driver version is 1.0.
mpu6500: min supply voltage is 1.7V.
mpu6500: max supply voltage is 3.5V.
mpu6500: max current is 3.20mA.
mpu6500: max temperature is 85.0C.
mpu6500: min temperature is -40.0C.
mpu6500: start register test.
mpu6500: mpu6500_set_interface/mpu6500_get_interface test.
mpu6500: set interface iic.
mpu6500: check interface ok.
mpu6500: set interface spi.
mpu6500: check interface ok.
mpu6500: mpu6500_set_addr_pin/mpu6500_get_addr_pin test.
mpu6500: set addr pin low.
mpu6500: check addr pin ok.
mpu6500: set addr pin high.
mpu6500: check addr pin ok.
mpu6500: mpu6500_set_fifo/mpu6500_get_fifo test.
mpu6500: enable fifo.
mpu6500: check fifo ok.
mpu6500: disable fifo.
mpu6500: check fifo ok.
mpu6500: mpu6500_set_iic_master/mpu6500_get_iic_master test.
mpu6500: enable iic master.
mpu6500: check iic master ok.
mpu6500: disable iic master.
mpu6500: check iic master ok.
mpu6500: mpu6500_fifo_reset/mpu6500_get_fifo_reset test.
mpu6500: fifo reset.
mpu6500: check fifo reset ok.
mpu6500: mpu6500_iic_master_reset/mpu6500_get_iic_master_reset test.
mpu6500: iic master reset.
mpu6500: check iic master ok.
mpu6500: mpu6500_sensor_reset/mpu6500_get_sensor_reset test.
mpu6500: sensor reset.
mpu6500: check sensor reset ok.
mpu6500: mpu6500_device_reset/mpu6500_get_device_reset test.
mpu6500: device reset.
mpu6500: check device reset ok.
mpu6500: mpu6500_set_clock_source/mpu6500_get_clock_source test.
mpu6500: stop the clock.
mpu6500: check clock source ok.
mpu6500: set the clock source extern 20.0 MHz.
mpu6500: check clock source ok.
mpu6500: set the clock source pll.
mpu6500: check clock source ok.
mpu6500: mpu6500_set_ptat/mpu6500_get_ptat test.
mpu6500: enable temperature sensor.
mpu6500: check ptat ok.
mpu6500: disable temperature sensor.
mpu6500: check ptat ok.
mpu6500: mpu6500_set_cycle_wake_up/mpu6500_get_cycle_wake_up test.
mpu6500: enable cycle wake up.
mpu6500: check cycle wake up ok.
mpu6500: disable cycle wake up.
mpu6500: check cycle wake up ok.
mpu6500: mpu6500_set_sleep/mpu6500_get_sleep test.
mpu6500: enable the sleep.
mpu6500: check sleep ok.
mpu6500: disable the sleep.
mpu6500: check sleep ok.
mpu6500: mpu6500_set_standby_mode/mpu6500_get_standby_mode test.
mpu6500: enable acc x standby mode.
mpu6500: check standby mode ok.
mpu6500: disable acc x standby mode.
mpu6500: check standby mode ok.
mpu6500: enable acc y standby mode.
mpu6500: check standby mode ok.
mpu6500: disable acc y standby mode.
mpu6500: check standby mode ok.
mpu6500: enable acc z standby mode.
mpu6500: check standby mode ok.
mpu6500: disable acc z standby mode.
mpu6500: check standby mode ok.
mpu6500: enable gyro x standby mode.
mpu6500: check standby mode ok.
mpu6500: disable gyro x standby mode.
mpu6500: check standby mode ok.
mpu6500: enable gyro y standby mode.
mpu6500: check standby mode ok.
mpu6500: disable gyro y standby mode.
mpu6500: check standby mode ok.
mpu6500: enable gyro z standby mode.
mpu6500: check standby mode ok.
mpu6500: disable gyro z standby mode.
mpu6500: check standby mode ok.
mpu6500: mpu6500_fifo_get/mpu6500_fifo_set test.
mpu6500: check fifo write ok.
mpu6500: check fifo read ok.
mpu6500: mpu6500_get_fifo_count test.
mpu6500: fifo count 0.
mpu6500: mpu6500_set_signal_path_reset test.
mpu6500: temp signal path reset.
mpu6500: check signal path reset ok.
mpu6500: accel signal path reset.
mpu6500: check signal path reset ok.
mpu6500: gyro signal path reset.
mpu6500: check signal path reset ok.
mpu6500: mpu6500_set_sample_rate_divider/mpu6500_get_sample_rate_divider test.
mpu6500: set sample rate divider 0x10A9C.
mpu6500: check sample rate divider ok.
mpu6500: mpu6500_set_extern_sync/mpu6500_get_extern_sync test.
mpu6500: set extern sync input disabled.
mpu6500: check extern sync ok.
mpu6500: set extern sync temp out low.
mpu6500: check extern sync ok.
mpu6500: set extern sync gyro xout low.
mpu6500: check extern sync ok.
mpu6500: set extern sync gyro yout low.
mpu6500: check extern sync ok.
mpu6500: set extern sync gyro zout low.
mpu6500: check extern sync ok.
mpu6500: set extern sync accel xout low.
mpu6500: check extern sync ok.
mpu6500: set extern sync accel yout low.
mpu6500: check extern sync ok.
mpu6500: set extern sync accel zout low.
mpu6500: check extern sync ok.
mpu6500: mpu6500_set_low_pass_filter/mpu6500_get_low_pass_filter test.
mpu6500: set low pass filter 0.
mpu6500: check low pass filter ok.
mpu6500: set low pass filter 1.
mpu6500: check low pass filter ok.
mpu6500: set low pass filter 2.
mpu6500: check low pass filter ok.
mpu6500: set low pass filter 3.
mpu6500: check low pass filter ok.
mpu6500: set low pass filter 4.
mpu6500: check low pass filter ok.
mpu6500: set low pass filter 5.
mpu6500: check low pass filter ok.
mpu6500: set low pass filter 6.
mpu6500: check low pass filter ok.
mpu6500: mpu6500_set_gyroscope_test/mpu6500_get_gyroscope_test test.
mpu6500: enable axis x.
mpu6500: check gyroscope test ok.
mpu6500: disable axis x.
mpu6500: check gyroscope test ok.
mpu6500: enable axis y.
mpu6500: check gyroscope test ok.
mpu6500: disable axis y.
mpu6500: check gyroscope test ok.
mpu6500: enable axis z.
mpu6500: check gyroscope test ok.
mpu6500: disable axis z.
mpu6500: check gyroscope test ok.
mpu6500: mpu6500_set_gyroscope_range/mpu6500_get_gyroscope_range test.
mpu6500: set gyroscope range 250 dps.
mpu6500: check gyroscope range ok.
mpu6500: set gyroscope range 500 dps.
mpu6500: check gyroscope range ok.
mpu6500: set gyroscope range 1000 dps.
mpu6500: check gyroscope range ok.
mpu6500: set gyroscope range 2000 dps.
mpu6500: check gyroscope range ok.
mpu6500: mpu6500_set_accelerometer_test/mpu6500_get_accelerometer_test test.
mpu6500: enable accelerometer x.
mpu6500: check accelerometer test ok.
mpu6500: disable accelerometer x.
mpu6500: check accelerometer test ok.
mpu6500: enable accelerometer y.
mpu6500: check accelerometer test ok.
mpu6500: disable accelerometer y.
mpu6500: check accelerometer test ok.
mpu6500: enable accelerometer z.
mpu6500: check accelerometer test ok.
mpu6500: disable accelerometer z.
mpu6500: check accelerometer test ok.
mpu6500: mpu6500_set_accelerometer_range/mpu6500_get_accelerometer_range test.
mpu6500: set accelerometer range 2g.
mpu6500: check accelerometer range ok.
mpu6500: set accelerometer range 4g.
mpu6500: check accelerometer range ok.
mpu6500: set accelerometer range 8g.
mpu6500: check accelerometer range ok.
mpu6500: set accelerometer range 16g.
mpu6500: check accelerometer range ok.
mpu6500: mpu6500_set_fifo_enable/mpu6500_get_fifo_enable test.
mpu6500: set fifo temp enable.
mpu6500: check fifo enable ok.
mpu6500: set fifo temp disable.
mpu6500: check fifo enable ok.
mpu6500: set fifo gyroscope x enable.
mpu6500: check fifo enable ok.
mpu6500: set fifo gyroscope x disable.
mpu6500: check fifo enable ok.
mpu6500: set fifo gyroscope y enable.
mpu6500: check fifo enable ok.
mpu6500: set fifo gyroscope y disable.
mpu6500: check fifo enable ok.
mpu6500: set fifo gyroscope z enable.
mpu6500: check fifo enable ok.
mpu6500: set fifo gyroscope z disable.
mpu6500: check fifo enable ok.
mpu6500: set fifo accelerometer enable.
mpu6500: check fifo enable ok.
mpu6500: set fifo accelerometer disable.
mpu6500: check fifo enable ok.
mpu6500: mpu6500_set_interrupt_level/mpu6500_get_interrupt_level test.
mpu6500: set interrupt high level.
mpu6500: check interrupt level ok.
mpu6500: set interrupt low level.
mpu6500: check interrupt level ok.
mpu6500: mpu6500_set_interrupt_pin_type/mpu6500_get_interrupt_pin_type test.
mpu6500: set interrupt pin type push pull.
mpu6500: check interrupt pin type ok.
mpu6500: set interrupt pin type open drain.
mpu6500: check interrupt pin type ok.
mpu6500: mpu6500_set_interrupt_latch/mpu6500_get_interrupt_latch test.
mpu6500: enable interrupt latch.
mpu6500: check interrupt latch ok.
mpu6500: disable interrupt latch.
mpu6500: check interrupt latch ok.
mpu6500: mpu6500_set_interrupt_read_clear/mpu6500_get_interrupt_read_clear test.
mpu6500: enable interrupt read clear.
mpu6500: check interrupt read clear ok.
mpu6500: disable interrupt read clear.
mpu6500: check interrupt read clear ok.
mpu6500: mpu6500_set_fsync_interrupt_level/mpu6500_get_fsync_interrupt_level test.
mpu6500: set fsync interrupt level high.
mpu6500: check fsync interrupt level ok.
mpu6500: set fsync interrupt level low.
mpu6500: check fsync interrupt level ok.
mpu6500: mpu6500_set_fsync_interrupt/mpu6500_get_fsync_interrupt test.
mpu6500: enable fsync interrupt.
mpu6500: check fsync interrupt ok.
mpu6500: disable fsync interrupt.
mpu6500: check fsync interrupt ok.
mpu6500: mpu6500_set_iic_bypass/mpu6500_get_iic_bypass test.
mpu6500: enable iic bypass.
mpu6500: check iic bypass ok.
mpu6500: disable iic bypass.
mpu6500: check iic bypass ok.
mpu6500: mpu6500_set_interrupt/mpu6500_get_interrupt test.
mpu6500: enable motion interrupt.
mpu6500: check motion interrupt ok.
mpu6500: disable motion interrupt.
mpu6500: check motion interrupt ok.
mpu6500: enable fifo overflow interrupt.
mpu6500: check fifo overflow interrupt ok.
mpu6500: disable fifo overflow interrupt.
mpu6500: check fifo overflow interrupt ok.
mpu6500: enable dmp interrupt.
mpu6500: check dmp interrupt ok.
mpu6500: disable dmp interrupt.
mpu6500: check dmp interrupt ok.
mpu6500: enable fsync init interrupt.
mpu6500: check fsync init interrupt ok.
mpu6500: disable fsync init interrupt.
mpu6500: check fsync init interrupt ok.
mpu6500: enable data ready interrupt.
mpu6500: check data ready interrupt ok.
mpu6500: disable data ready interrupt.
mpu6500: check data ready interrupt ok.
mpu6500: mpu6500_get_interrupt_status test.
mpu6500: get interrupt status 0x00.
mpu6500: mpu6500_set_gyroscope_x_test/mpu6500_get_gyroscope_x_test test.
mpu6500: set gyroscope x test 0x0D.
mpu6500: check gyroscope x test ok.
mpu6500: mpu6500_set_gyroscope_y_test/mpu6500_get_gyroscope_y_test test.
mpu6500: set gyroscope y test 0x1A.
mpu6500: check gyroscope y test ok.
mpu6500: mpu6500_set_gyroscope_z_test/mpu6500_get_gyroscope_z_test test.
mpu6500: set gyroscope z test 0x0B.
mpu6500: check gyroscope z test ok.
mpu6500: mpu6500_set_accelerometer_x_test/mpu6500_get_accelerometer_x_test test.
mpu6500: set accelerometer x test 0x12.
mpu6500: check accelerometer x test ok.
mpu6500: mpu6500_set_accelerometer_y_test/mpu6500_get_accelerometer_y_test test.
mpu6500: set accelerometer y test 0x1B.
mpu6500: check accelerometer y test ok.
mpu6500: mpu6500_set_accelerometer_z_test/mpu6500_get_accelerometer_z_test test.
mpu6500: set accelerometer z test 0x03.
mpu6500: check accelerometer z test ok.
mpu6500: mpu6500_set_motion_threshold/mpu6500_get_motion_threshold test.
mpu6500: set motion threshold 0x46.
mpu6500: check motion threshold ok.
mpu6500: mpu6500_motion_threshold_convert_to_register/mpu6500_motion_threshold_convert_to_data test.
mpu6500: motion threshold convert to register 86.00.
mpu6500: check motion threshold 84.00.
mpu6500: mpu6500_set_iic_clock/mpu6500_get_iic_clock test.
mpu6500: set iic clock 348 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 333 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 320 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 308 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 296 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 286 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 276 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 267 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 258 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 500 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 471 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 444 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 421 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 400 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 381 kHz.
mpu6500: check iic clock ok.
mpu6500: set iic clock 364 kHz.
mpu6500: check iic clock ok.
mpu6500: mpu6500_set_iic_multi_master/mpu6500_get_iic_multi_master test.
mpu6500: enable iic multi master.
mpu6500: check iic multi master ok.
mpu6500: disable iic multi master.
mpu6500: check iic multi master ok.
mpu6500: mpu6500_set_iic_wait_for_external_sensor/mpu6500_get_iic_wait_for_external_sensor test.
mpu6500: enable iic wait for external sensor.
mpu6500: check iic wait for external sensor ok.
mpu6500: disable iic wait for external sensor.
mpu6500: check iic wait for external sensor ok.
mpu6500: mpu6500_set_iic_read_mode/mpu6500_get_iic_read_mode test.
mpu6500: set restart read mode.
mpu6500: check iic read mode ok.
mpu6500: set stop and start read mode.
mpu6500: check iic read mode ok.
mpu6500: mpu6500_set_iic_fifo_enable/mpu6500_get_iic_fifo_enable test.
mpu6500: enable iic fifo slave0.
mpu6500: check iic fifo enable ok.
mpu6500: disable iic fifo slave0.
mpu6500: check iic fifo enable ok.
mpu6500: enable iic fifo slave1.
mpu6500: check iic fifo enable ok.
mpu6500: disable iic fifo slave1.
mpu6500: check iic fifo enable ok.
mpu6500: enable iic fifo slave2.
mpu6500: check iic fifo enable ok.
mpu6500: disable iic fifo slave2.
mpu6500: check iic fifo enable ok.
mpu6500: enable iic fifo slave3.
mpu6500: check iic fifo enable ok.
mpu6500: disable iic fifo slave3.
mpu6500: check iic fifo enable ok.
mpu6500: mpu6500_set_iic_mode/mpu6500_get_iic_mode test.
mpu6500: set slave0 iic write mode.
mpu6500: check iic mode ok.
mpu6500: set slave0 iic read mode.
mpu6500: check iic mode ok.
mpu6500: set slave1 iic write mode.
mpu6500: check iic mode ok.
mpu6500: set slave1 iic read mode.
mpu6500: check iic mode ok.
mpu6500: set slave2 iic write mode.
mpu6500: check iic mode ok.
mpu6500: set slave2 iic read mode.
mpu6500: check iic mode ok.
mpu6500: set slave3 iic write mode.
mpu6500: check iic mode ok.
mpu6500: set slave3 iic read mode.
mpu6500: check iic mode ok.
mpu6500: set slave4 iic write mode.
mpu6500: check iic mode ok.
mpu6500: set slave4 iic read mode.
mpu6500: check iic mode ok.
mpu6500: mpu6500_set_iic_address/mpu6500_get_iic_address test.
mpu6500: set slave0 iic address 0x2D.
mpu6500: check iic address ok.
mpu6500: set slave1 iic address 0x76.
mpu6500: check iic address ok.
mpu6500: set slave2 iic address 0x7E.
mpu6500: check iic address ok.
mpu6500: set slave3 iic address 0x4A.
mpu6500: check iic address ok.
mpu6500: set slave4 iic address 0x12.
mpu6500: check iic address ok.
mpu6500: mpu6500_set_iic_register/mpu6500_get_iic_register test.
mpu6500: set slave0 iic register 0xE7.
mpu6500: check iic register ok.
mpu6500: set slave1 iic register 0x8D.
mpu6500: check iic register ok.
mpu6500: set slave2 iic register 0x76.
mpu6500: check iic register ok.
mpu6500: set slave3 iic register 0x5A.
mpu6500: check iic register ok.
mpu6500: set slave4 iic register 0x2E.
mpu6500: check iic register ok.
mpu6500: mpu6500_set_iic_data_out/mpu6500_get_iic_data_out test.
mpu6500: set slave0 iic data out 0x63.
mpu6500: check iic data out ok.
mpu6500: set slave1 iic data out 0x33.
mpu6500: check iic data out ok.
mpu6500: set slave2 iic data out 0x9F.
mpu6500: check iic data out ok.
mpu6500: set slave3 iic data out 0xC9.
mpu6500: check iic data out ok.
mpu6500: mpu6500_set_iic_enable/mpu6500_get_iic_enable test.
mpu6500: slave0 iic enable.
mpu6500: check iic enable ok.
mpu6500: slave0 iic disable.
mpu6500: check iic enable ok.
mpu6500: slave1 iic enable.
mpu6500: check iic enable ok.
mpu6500: slave1 iic disable.
mpu6500: check iic enable ok.
mpu6500: slave2 iic enable.
mpu6500: check iic enable ok.
mpu6500: slave2 iic disable.
mpu6500: check iic enable ok.
mpu6500: slave3 iic enable.
mpu6500: check iic enable ok.
mpu6500: slave3 iic disable.
mpu6500: check iic enable ok.
mpu6500: mpu6500_set_iic_byte_swap/mpu6500_get_iic_byte_swap test.
mpu6500: enable slave0 byte swap.
mpu6500: check iic byte swap ok.
mpu6500: disable slave0 byte swap.
mpu6500: check iic byte swap ok.
mpu6500: enable slave1 byte swap.
mpu6500: check iic byte swap ok.
mpu6500: disable slave1 byte swap.
mpu6500: check iic byte swap ok.
mpu6500: enable slave2 byte swap.
mpu6500: check iic byte swap ok.
mpu6500: disable slave2 byte swap.
mpu6500: check iic byte swap ok.
mpu6500: enable slave3 byte swap.
mpu6500: check iic byte swap ok.
mpu6500: disable slave3 byte swap.
mpu6500: check iic byte swap ok.
mpu6500: mpu6500_set_iic_transaction_mode/mpu6500_get_iic_transaction_mode test.
mpu6500: set slave0 data transaction mode.
mpu6500: check iic transaction mode ok.
mpu6500: set slave0 reg data transaction mode.
mpu6500: check iic transaction mode ok.
mpu6500: set slave1 data transaction mode.
mpu6500: check iic transaction mode ok.
mpu6500: set slave1 reg data transaction mode.
mpu6500: check iic transaction mode ok.
mpu6500: set slave2 data transaction mode.
mpu6500: check iic transaction mode ok.
mpu6500: set slave2 reg data transaction mode.
mpu6500: check iic transaction mode ok.
mpu6500: set slave3 data transaction mode.
mpu6500: check iic transaction mode ok.
mpu6500: set slave3 reg data transaction mode.
mpu6500: check iic transaction mode ok.
mpu6500: mpu6500_set_iic_group_order/mpu6500_get_iic_group_order test.
mpu6500: set slave0 group order even.
mpu6500: check iic group order ok.
mpu6500: set slave0 group order odd.
mpu6500: check iic group order ok.
mpu6500: set slave1 group order even.
mpu6500: check iic group order ok.
mpu6500: set slave1 group order odd.
mpu6500: check iic group order ok.
mpu6500: set slave2 group order even.
mpu6500: check iic group order ok.
mpu6500: set slave2 group order odd.
mpu6500: check iic group order ok.
mpu6500: set slave3 group order even.
mpu6500: check iic group order ok.
mpu6500: set slave3 group order odd.
mpu6500: check iic group order ok.
mpu6500: mpu6500_set_iic_transferred_len/mpu6500_get_iic_transferred_len test.
mpu6500: set slave0 iic transferred len 10.
mpu6500: check iic transferred len ok.
mpu6500: set slave1 iic transferred len 6.
mpu6500: check iic transferred len ok.
mpu6500: set slave2 iic transferred len 2.
mpu6500: check iic transferred len ok.
mpu6500: set slave3 iic transferred len 13.
mpu6500: check iic transferred len ok.
mpu6500: mpu6500_get_iic_status test.
mpu6500: iic status is 0x00.
mpu6500: mpu6500_set_iic_delay_enable/mpu6500_get_iic_delay_enable test.
mpu6500: enable delay shadow.
mpu6500: check iic delay enable ok.
mpu6500: disable delay shadow.
mpu6500: check iic delay enable ok.
mpu6500: enable delay slave4.
mpu6500: check iic delay enable ok.
mpu6500: disable delay slave4.
mpu6500: check iic delay enable ok.
mpu6500: enable delay slave3.
mpu6500: check iic delay enable ok.
mpu6500: disable delay slave3.
mpu6500: check iic delay enable ok.
mpu6500: enable delay slave2.
mpu6500: check iic delay enable ok.
mpu6500: disable delay slave2.
mpu6500: check iic delay enable ok.
mpu6500: enable delay slave1.
mpu6500: check iic delay enable ok.
mpu6500: disable delay slave1.
mpu6500: check iic delay enable ok.
mpu6500: enable delay slave0.
mpu6500: check iic delay enable ok.
mpu6500: disable delay slave0.
mpu6500: check iic delay enable ok.
mpu6500: mpu6500_set_iic4_enable/mpu6500_get_iic4_enable test.
mpu6500: enable iic4.
mpu6500: check iic4 enable ok.
mpu6500: disable iic4.
mpu6500: check iic4 enable ok.
mpu6500: mpu6500_set_iic4_interrupt/mpu6500_get_iic4_interrupt test.
mpu6500: enable iic4 interrupt.
mpu6500: check iic4 interrupt ok.
mpu6500: disable iic4 interrupt.
mpu6500: check iic4 interrupt ok.
mpu6500: mpu6500_set_iic4_transaction_mode/mpu6500_get_iic4_transaction_mode test.
mpu6500: set iic4 transaction mode data.
mpu6500: check iic4 transaction mode ok.
mpu6500: set iic4 transaction mode reg.
mpu6500: check iic4 transaction mode ok.
mpu6500: mpu6500_set_iic_delay/mpu6500_get_iic_delay test.
mpu6500: set iic delay 0x16.
mpu6500: check iic delay ok.
mpu6500: mpu6500_set_iic4_data_out/mpu6500_get_iic4_data_out test.
mpu6500: set iic4 data out 0x31.
mpu6500: check iic4 data out ok.
mpu6500: mpu6500_set_iic4_data_in/mpu6500_get_iic4_data_in test.
mpu6500: set iic4 data in 0x58.
mpu6500: check iic4 data in ok.
mpu6500: mpu6500_set_gyro_standby/mpu6500_get_gyro_standby test.
mpu6500: enable gyro standby.
mpu6500: check gyro standby ok.
mpu6500: disable gyro standby.
mpu6500: check gyro standby ok.
mpu6500: mpu6500_set_fifo_mode/mpu6500_get_fifo_mode test.
mpu6500: set fifo stream mode.
mpu6500: check fifo mode ok.
mpu6500: set fifo normal mode.
mpu6500: check fifo mode ok.
mpu6500: mpu6500_set_gyroscope_choice/mpu6500_get_gyroscope_choice test.
mpu6500: set gyroscope choice 0x03.
mpu6500: check gyroscope choice ok.
mpu6500: mpu6500_set_accelerometer_choice/mpu6500_get_accelerometer_choice test.
mpu6500: set accelerometer choice 0x00.
mpu6500: check accelerometer choice ok.
mpu6500: mpu6500_set_accelerometer_low_pass_filter/mpu6500_get_accelerometer_low_pass_filter test.
mpu6500: set accelerometer low pass filter 0.
mpu6500: check accelerometer low pass filter ok.
mpu6500: set accelerometer low pass filter 1.
mpu6500: check accelerometer low pass filter ok.
mpu6500: set accelerometer low pass filter 2.
mpu6500: check accelerometer low pass filter ok.
mpu6500: set accelerometer low pass filter 3.
mpu6500: check accelerometer low pass filter ok.
mpu6500: set accelerometer low pass filter 4.
mpu6500: check accelerometer low pass filter ok.
mpu6500: set accelerometer low pass filter 5.
mpu6500: check accelerometer low pass filter ok.
mpu6500: set accelerometer low pass filter 6.
mpu6500: check accelerometer low pass filter ok.
mpu6500: set accelerometer low pass filter 7.
mpu6500: check accelerometer low pass filter ok.
mpu6500: mpu6500_set_low_power_accel_output_rate/mpu6500_get_low_power_accel_output_rate test.
mpu6500: set low power accel output rate 0.24Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 0.49Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 0.98Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 1.95Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 3.91Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 7.81Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 15.63Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 31.25Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 62.50Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 125Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 250Hz.
mpu6500: check low power accel output rate ok.
mpu6500: set low power accel output rate 500Hz.
mpu6500: check low power accel output rate ok.
mpu6500: mpu6500_set_wake_on_motion/mpu6500_get_wake_on_motion test.
mpu6500: enable wake on motion.
mpu6500: check wake on motion ok.
mpu6500: disable wake on motion.
mpu6500: check wake on motion ok.
mpu6500: mpu6500_set_accel_compare_with_previous_sample/mpu6500_get_accel_compare_with_previous_sample test.
mpu6500: enable accel compare with previous sample.
mpu6500: check accel compare with previous sample ok.
mpu6500: disable accel compare with previous sample.
mpu6500: check accel compare with previous sample ok.
mpu6500: mpu6500_set_accelerometer_x_offset/mpu6500_get_accelerometer_x_offset test.
mpu6500: set accelerometer x offset 24869.
mpu6500: check accelerometer x offset ok.
mpu6500: mpu6500_set_accelerometer_y_offset/mpu6500_get_accelerometer_y_offset test.
mpu6500: set accelerometer y offset -2397.
mpu6500: check accelerometer y offset ok.
mpu6500: mpu6500_set_accelerometer_z_offset/mpu6500_get_accelerometer_z_offset test.
mpu6500: set accelerometer z offset 12549.
mpu6500: check accelerometer z offset ok.
mpu6500: mpu6500_accelerometer_offset_convert_to_register/mpu6500_accelerometer_offset_convert_to_data test.
mpu6500: accelerometer offset convert to register 91.90.
mpu6500: check accelerometer offset 91.14.
mpu6500: mpu6500_set_gyro_x_offset/mpu6500_get_gyro_x_offset test.
mpu6500: set gyro x offset 10328.
mpu6500: check gyro x offset ok.
mpu6500: mpu6500_set_gyro_y_offset/mpu6500_get_gyro_y_offset test.
mpu6500: set gyro y offset -23273.
mpu6500: check gyro y offset ok.
mpu6500: mpu6500_set_gyro_z_offset/mpu6500_get_gyro_z_offset test.
mpu6500: set gyro z offset 1118.
mpu6500: check gyro z offset ok.
mpu6500: mpu6500_gyro_offset_convert_to_register/mpu6500_gyro_offset_convert_to_data test.
mpu6500: gyro offset convert to register 32.40.
mpu6500: check gyro offset 32.39.
mpu6500: finish register test.
```

```shell
./mpu6500 -t read --addr=0 --interface=iic --times=3

mpu6500: chip is TDK MPU6500.
mpu6500: manufacturer is TDK.
mpu6500: interface is IIC SPI.
mpu6500: driver version is 1.0.
mpu6500: min supply voltage is 1.7V.
mpu6500: max supply voltage is 3.5V.
mpu6500: max current is 3.20mA.
mpu6500: max temperature is 85.0C.
mpu6500: min temperature is -40.0C.
mpu6500: start read test.
mpu6500: set accelerometer range 2g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.02g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.02g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.01g.
mpu6500: set accelerometer range 4g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.02g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.02g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.02g.
mpu6500: set accelerometer range 8g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.02g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.01g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.02g.
mpu6500: set accelerometer range 16g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.02g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.01g.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.01g.
mpu6500: set gyroscope range 250dps.
mpu6500: gyro x is -0.97dps.
mpu6500: gyro y is 0.98dps.
mpu6500: gyro z is 0.08dps.
mpu6500: gyro x is -0.92dps.
mpu6500: gyro y is 0.98dps.
mpu6500: gyro z is 0.07dps.
mpu6500: gyro x is -0.94dps.
mpu6500: gyro y is 0.95dps.
mpu6500: gyro z is 0.03dps.
mpu6500: set gyroscope range 500dps.
mpu6500: gyro x is -0.93dps.
mpu6500: gyro y is 1.07dps.
mpu6500: gyro z is 0.09dps.
mpu6500: gyro x is -0.99dps.
mpu6500: gyro y is 1.01dps.
mpu6500: gyro z is 0.03dps.
mpu6500: gyro x is -0.92dps.
mpu6500: gyro y is 1.02dps.
mpu6500: gyro z is 0.11dps.
mpu6500: set gyroscope range 1000dps.
mpu6500: gyro x is -0.88dps.
mpu6500: gyro y is 1.01dps.
mpu6500: gyro z is 0.12dps.
mpu6500: gyro x is -0.82dps.
mpu6500: gyro y is 0.98dps.
mpu6500: gyro z is 0.03dps.
mpu6500: gyro x is -0.88dps.
mpu6500: gyro y is 0.95dps.
mpu6500: gyro z is 0.06dps.
mpu6500: set gyroscope range 2000dps.
mpu6500: gyro x is -0.91dps.
mpu6500: gyro y is 0.91dps.
mpu6500: gyro z is 0.06dps.
mpu6500: gyro x is -0.91dps.
mpu6500: gyro y is 0.98dps.
mpu6500: gyro z is 0.06dps.
mpu6500: gyro x is -1.04dps.
mpu6500: gyro y is 0.98dps.
mpu6500: gyro z is 0.06dps.
mpu6500: read temperature.
mpu6500: temperature 30.76C.
mpu6500: temperature 30.75C.
mpu6500: temperature 30.76C.
mpu6500: finish read test.
```

```shell
./mpu6500 -t fifo --addr=0 --interface=iic --times=3

mpu6500: chip is TDK MPU6500.
mpu6500: manufacturer is TDK.
mpu6500: interface is IIC SPI.
mpu6500: driver version is 1.0.
mpu6500: min supply voltage is 1.7V.
mpu6500: max supply voltage is 3.5V.
mpu6500: max current is 3.20mA.
mpu6500: max temperature is 85.0C.
mpu6500: min temperature is -40.0C.
mpu6500: start fifo test.
mpu6500: fifo 48.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.02g.
mpu6500: gyro x[0] is -0.85dps.
mpu6500: gyro y[0] is 0.98dps.
mpu6500: gyro z[0] is 0.06dps.
mpu6500: fifo 26.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.01g.
mpu6500: gyro x[0] is -0.98dps.
mpu6500: gyro y[0] is 0.91dps.
mpu6500: gyro z[0] is 0.06dps.
mpu6500: fifo 26.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.01g.
mpu6500: gyro x[0] is -0.98dps.
mpu6500: gyro y[0] is 0.98dps.
mpu6500: gyro z[0] is 0.06dps.
mpu6500: finish fifo test.
```

```shell
./mpu6500 -t dmp --addr=0 --interface=iic --times=3

mpu6500: chip is TDK MPU6500.
mpu6500: manufacturer is TDK.
mpu6500: interface is IIC SPI.
mpu6500: driver version is 1.0.
mpu6500: min supply voltage is 1.7V.
mpu6500: max supply voltage is 3.5V.
mpu6500: max current is 3.20mA.
mpu6500: max temperature is 85.0C.
mpu6500: min temperature is -40.0C.
mpu6500: start dmp read test.
mpu6500: load dmp firmware.
mpu6500: load dmp firmware successful .
mpu6500: mpu6500_dmp_set_pedometer_walk_time/mpu6500_dmp_get_pedometer_walk_time test.
mpu6500: dmp set pedometer walk time 200 ms.
mpu6500: check pedometer walk time ok.
mpu6500: mpu6500_dmp_set_pedometer_step_count/mpu6500_dmp_get_pedometer_step_count test.
mpu6500: dmp set pedometer step count 383.
mpu6500: check pedometer step count ok.
mpu6500: mpu6500_dmp_set_shake_reject_timeout/mpu6500_dmp_get_shake_reject_timeout test.
mpu6500: dmp set shake reject timeout 10 ms.
mpu6500: check shake reject timeout ok.
mpu6500: mpu6500_dmp_set_shake_reject_time/mpu6500_dmp_get_shake_reject_time test.
mpu6500: dmp set shake reject time 40 ms.
mpu6500: check shake reject time ok.
mpu6500: mpu6500_dmp_set_shake_reject_thresh/mpu6500_dmp_get_shake_reject_thresh test.
mpu6500: set shake reject thresh 0 dps.
mpu6500: check shake reject thresh ok.
mpu6500: mpu6500_dmp_set_tap_time_multi/mpu6500_dmp_get_tap_time_multi test.
mpu6500: dmp set tap time multi 500 ms.
mpu6500: check tap time multi ok.
mpu6500: mpu6500_dmp_set_tap_time/mpu6500_dmp_get_tap_time test.
mpu6500: dmp set tap time 100 ms.
mpu6500: check tap time ok.
mpu6500: mpu6500_dmp_set_min_tap_count/mpu6500_dmp_get_min_tap_count test.
mpu6500: dmp set min tap count 1.
mpu6500: check min tap count ok.
mpu6500: mpu6500_dmp_set_tap_axes/mpu6500_dmp_get_tap_axes test.
mpu6500: disable tap axes x.
mpu6500: check tap axes ok.
mpu6500: enable tap axes x.
mpu6500: check tap axes ok.
mpu6500: disable tap axes y.
mpu6500: check tap axes ok.
mpu6500: enable tap axes y.
mpu6500: check tap axes ok.
mpu6500: disable tap axes z.
mpu6500: check tap axes ok.
mpu6500: enable tap axes z.
mpu6500: check tap axes ok.
mpu6500: mpu6500_dmp_set_tap_thresh/mpu6500_dmp_get_tap_thresh test.
mpu6500: dmp set tap thresh x 250 mg/ms.
mpu6500: check tap thresh ok.
mpu6500: dmp set tap thresh y 250 mg/ms.
mpu6500: check tap thresh ok.
mpu6500: dmp set tap thresh z 250 mg/ms.
mpu6500: check tap thresh ok.
mpu6500: mpu6500_dmp_set_fifo_rate/mpu6500_dmp_get_fifo_rate test.
mpu6500: dmp set fifo rate 200Hz.
mpu6500: check fifo rate ok.
mpu6500: mpu6500_dmp_set_gyro_calibrate test.
mpu6500: enable gyro calibrate.
mpu6500: disable gyro calibrate.
mpu6500: mpu6500_dmp_set_3x_quaternion test.
mpu6500: enable 3x quaternion.
mpu6500: disable 3x quaternion.
mpu6500: mpu6500_dmp_set_6x_quaternion test.
mpu6500: enable 6x quaternion.
mpu6500: disable 6x quaternion.
mpu6500: mpu6500_dmp_set_interrupt_mode test.
mpu6500: dmp set gesture interrupt mode.
mpu6500: dmp set gesture continuous mode.
mpu6500: mpu6500_dmp_set_orientation test.
mpu6500: set the dmp orientation.
mpu6500: mpu6500_dmp_set_feature test.
mpu6500: enable feature 6x quat.
mpu6500: enable feature tap.
mpu6500: enable feature pedometer.
mpu6500: enable feature orient.
mpu6500: enable feature send raw accel.
mpu6500: enable feature send cal gyro.
mpu6500: enable feature gyro cal.
mpu6500: fifo 2.
mpu6500: pitch[0] is 0.12deg.
mpu6500: roll[0] is 0.86deg.
mpu6500: yaw[0] is 0.00deg.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.02g.
mpu6500: gyro x[0] is 0.00dps.
mpu6500: gyro y[0] is 0.00dps.
mpu6500: gyro z[0] is -0.06dps.
mpu6500: fifo 3.
mpu6500: pitch[0] is 0.12deg.
mpu6500: roll[0] is 0.85deg.
mpu6500: yaw[0] is 0.00deg.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.02g.
mpu6500: gyro x[0] is 0.00dps.
mpu6500: gyro y[0] is 0.00dps.
mpu6500: gyro z[0] is -0.06dps.
mpu6500: fifo 3.
mpu6500: pitch[0] is 0.12deg.
mpu6500: roll[0] is 0.84deg.
mpu6500: yaw[0] is 0.00deg.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.01g.
mpu6500: gyro x[0] is -0.12dps.
mpu6500: gyro y[0] is 0.06dps.
mpu6500: gyro z[0] is -0.06dps.
mpu6500: finish dmp read test.
```

```shell
./mpu6500 -t motion --addr=0 --interface=iic

mpu6500: chip is TDK MPU6500.
mpu6500: manufacturer is TDK.
mpu6500: interface is IIC SPI.
mpu6500: driver version is 1.0.
mpu6500: min supply voltage is 1.7V.
mpu6500: max supply voltage is 3.5V.
mpu6500: max current is 3.20mA.
mpu6500: max temperature is 85.0C.
mpu6500: min temperature is -40.0C.
mpu6500: start dmp tap orient motion test.
mpu6500: load dmp firmware.
mpu6500: load dmp firmware successful .
mpu6500: mpu6500_dmp_set_pedometer_walk_time/mpu6500_dmp_get_pedometer_walk_time test.
mpu6500: dmp set pedometer walk time 200 ms.
mpu6500: check pedometer walk time ok.
mpu6500: mpu6500_dmp_set_pedometer_step_count/mpu6500_dmp_get_pedometer_step_count test.
mpu6500: dmp set pedometer step count 755.
mpu6500: check pedometer step count ok.
mpu6500: mpu6500_dmp_set_shake_reject_timeout/mpu6500_dmp_get_shake_reject_timeout test.
mpu6500: dmp set shake reject timeout 10 ms.
mpu6500: check shake reject timeout ok.
mpu6500: mpu6500_dmp_set_shake_reject_time/mpu6500_dmp_get_shake_reject_time test.
mpu6500: dmp set shake reject time 40 ms.
mpu6500: check shake reject time ok.
mpu6500: mpu6500_dmp_set_shake_reject_thresh/mpu6500_dmp_get_shake_reject_thresh test.
mpu6500: set shake reject thresh 200 dps.
mpu6500: check shake reject thresh error.
mpu6500: mpu6500_dmp_set_tap_time_multi/mpu6500_dmp_get_tap_time_multi test.
mpu6500: dmp set tap time multi 200 ms.
mpu6500: check tap time multi ok.
mpu6500: mpu6500_dmp_set_tap_time/mpu6500_dmp_get_tap_time test.
mpu6500: dmp set tap time 100 ms.
mpu6500: check tap time ok.
mpu6500: mpu6500_dmp_set_min_tap_count/mpu6500_dmp_get_min_tap_count test.
mpu6500: dmp set min tap count 1.
mpu6500: check min tap count ok.
mpu6500: mpu6500_dmp_set_tap_axes/mpu6500_dmp_get_tap_axes test.
mpu6500: disable tap axes x.
mpu6500: check tap axes ok.
mpu6500: enable tap axes x.
mpu6500: check tap axes ok.
mpu6500: disable tap axes y.
mpu6500: check tap axes ok.
mpu6500: enable tap axes y.
mpu6500: check tap axes ok.
mpu6500: disable tap axes z.
mpu6500: check tap axes ok.
mpu6500: enable tap axes z.
mpu6500: check tap axes ok.
mpu6500: mpu6500_dmp_set_tap_thresh/mpu6500_dmp_get_tap_thresh test.
mpu6500: dmp set tap thresh x 250 mg/ms.
mpu6500: check tap thresh ok.
mpu6500: dmp set tap thresh y 250 mg/ms.
mpu6500: check tap thresh ok.
mpu6500: dmp set tap thresh z 250 mg/ms.
mpu6500: check tap thresh ok.
mpu6500: mpu6500_dmp_set_fifo_rate/mpu6500_dmp_get_fifo_rate test.
mpu6500: dmp set fifo rate 50Hz.
mpu6500: check fifo rate ok.
mpu6500: mpu6500_dmp_set_gyro_calibrate test.
mpu6500: enable gyro calibrate.
mpu6500: disable gyro calibrate.
mpu6500: mpu6500_dmp_set_3x_quaternion test.
mpu6500: enable 3x quaternion.
mpu6500: disable 3x quaternion.
mpu6500: mpu6500_dmp_set_6x_quaternion test.
mpu6500: enable 6x quaternion.
mpu6500: disable 6x quaternion.
mpu6500: mpu6500_dmp_set_interrupt_mode test.
mpu6500: dmp set gesture interrupt mode.
mpu6500: dmp set gesture continuous mode.
mpu6500: mpu6500_dmp_set_orientation test.
mpu6500: set the dmp orientation.
mpu6500: mpu6500_dmp_set_feature test.
mpu6500: enable feature 6x quat.
mpu6500: enable feature tap.
mpu6500: enable feature pedometer.
mpu6500: enable feature orient.
mpu6500: enable feature send raw accel.
mpu6500: enable feature send cal gyro.
mpu6500: enable feature gyro cal.
mpu6500: tap irq x up with 3.
mpu6500: orient irq reverse landscape.
mpu6500: irq motion.
mpu6500: finish dmp tap orient motion test.
```

```shell
./mpu6500 -t pedometer --addr=0 --interface=iic --times=3

mpu6500: chip is TDK MPU6500.
mpu6500: manufacturer is TDK.
mpu6500: interface is IIC SPI.
mpu6500: driver version is 1.0.
mpu6500: min supply voltage is 1.7V.
mpu6500: max supply voltage is 3.5V.
mpu6500: max current is 3.20mA.
mpu6500: max temperature is 85.0C.
mpu6500: min temperature is -40.0C.
mpu6500: start dmp pedometer test.
mpu6500: load dmp firmware.
mpu6500: load dmp firmware successful .
mpu6500: mpu6500_dmp_set_pedometer_walk_time/mpu6500_dmp_get_pedometer_walk_time test.
mpu6500: dmp set pedometer walk time 200 ms.
mpu6500: check pedometer walk time ok.
mpu6500: mpu6500_dmp_set_pedometer_step_count/mpu6500_dmp_get_pedometer_step_count test.
mpu6500: dmp set pedometer step count 13.
mpu6500: check pedometer step count ok.
mpu6500: dmp set gesture continuous mode.
mpu6500: mpu6500_dmp_set_fifo_rate/mpu6500_dmp_get_fifo_rate test.
mpu6500: dmp set fifo rate 50Hz.
mpu6500: check fifo rate ok.
mpu6500: mpu6500_dmp_set_feature test.
mpu6500: enable feature pedometer.
mpu6500: pedometer step count is 20.
mpu6500: pedometer step count is 21.
mpu6500: pedometer step count is 22.
mpu6500: finish dmp pedometer test.
```

```shell
./mpu6500 -e read --addr=0 --interface=iic --times=3

mpu6500: 1/3.
mpu6500: acc x is -0.06g.
mpu6500: acc y is 0.00g.
mpu6500: acc z is 1.02g.
mpu6500: gyro x is 0.00dps.
mpu6500: gyro y is 0.00dps.
mpu6500: gyro z is 0.00dps.
mpu6500: temperature 21.00C.
mpu6500: 2/3.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.01g.
mpu6500: gyro x is -0.91dps.
mpu6500: gyro y is 1.04dps.
mpu6500: gyro z is 0.06dps.
mpu6500: temperature 30.58C.
mpu6500: 3/3.
mpu6500: acc x is -0.05g.
mpu6500: acc y is 0.01g.
mpu6500: acc z is 1.01g.
mpu6500: gyro x is -0.91dps.
mpu6500: gyro y is 0.98dps.
mpu6500: gyro z is 0.06dps.
```

```shell
./mpu6500 -e fifo --addr=0 --interface=iic --times=3

mpu6500: 1/3.
mpu6500: fifo 48.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.01g.
mpu6500: gyro x[0] is -1.04dps.
mpu6500: gyro y[0] is 0.91dps.
mpu6500: gyro z[0] is 0.06dps.
mpu6500: 2/3.
mpu6500: fifo 26.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.01g.
mpu6500: gyro x[0] is -0.91dps.
mpu6500: gyro y[0] is 0.91dps.
mpu6500: gyro z[0] is 0.00dps.
mpu6500: 3/3.
mpu6500: fifo 26.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.01g.
mpu6500: gyro x[0] is -0.98dps.
mpu6500: gyro y[0] is 0.98dps.
mpu6500: gyro z[0] is -0.06dps.
```

```shell
./mpu6500 -e dmp --addr=0 --interface=iic --times=3

mpu6500: 1/3.
mpu6500: fifo 6.
mpu6500: pitch[0] is 0.11deg.
mpu6500: roll[0] is 0.86deg.
mpu6500: yaw[0] is 0.00deg.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.02g.
mpu6500: gyro x[0] is 0.00dps.
mpu6500: gyro y[0] is 0.12dps.
mpu6500: gyro z[0] is -0.12dps.
mpu6500: 2/3.
mpu6500: fifo 6.
mpu6500: pitch[0] is 0.10deg.
mpu6500: roll[0] is 0.80deg.
mpu6500: yaw[0] is -0.00deg.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.02g.
mpu6500: gyro x[0] is -0.06dps.
mpu6500: gyro y[0] is 0.06dps.
mpu6500: gyro z[0] is -0.12dps.
mpu6500: 3/3.
mpu6500: fifo 6.
mpu6500: pitch[0] is 0.10deg.
mpu6500: roll[0] is 0.74deg.
mpu6500: yaw[0] is -0.01deg.
mpu6500: acc x[0] is -0.05g.
mpu6500: acc y[0] is 0.01g.
mpu6500: acc z[0] is 1.02g.
mpu6500: gyro x[0] is 0.00dps.
mpu6500: gyro y[0] is 0.12dps.
mpu6500: gyro z[0] is -0.12dps.
```

```shell
./mpu6500 -e motion --addr=0 --interface=iic

mpu6500: irq motion.
mpu6500: irq data ready
mpu6500: irq dmp
mpu6500: tap irq x up with 3.
mpu6500: orient irq reverse portrait.
mpu6500: finish dmp tap orient motion.
```

```shell
./mpu6500 -e pedometer --addr=0 --interface=iic --times=3

mpu6500: pedometer step count is 7.
mpu6500: pedometer step count is 13.
mpu6500: pedometer step count is 15.
```

```shell
./mpu6500 -h

Usage:
  mpu6500 (-i | --information)
  mpu6500 (-h | --help)
  mpu6500 (-p | --port)
  mpu6500 (-t reg | --test=reg) [--addr=<0 | 1>] [--interface=<iic | spi>]
  mpu6500 (-t read | --test=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
  mpu6500 (-t fifo | --test=fifo) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
  mpu6500 (-t dmp | --test=dmp) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
  mpu6500 (-t motion | --test=motion) [--addr=<0 | 1>] [--interface=<iic | spi>]
  mpu6500 (-t pedometer | --test=pedometer) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
  mpu6500 (-e read | --example=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
  mpu6500 (-e fifo | --example=fifo) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
  mpu6500 (-e dmp | --example=dmp) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
  mpu6500 (-e motion | --example=motion) [--addr=<0 | 1>] [--interface=<iic | spi>]
  mpu6500 (-e pedometer | --example=pedometer) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]

Options:
      --addr=<0 | 1>      Set the addr pin.([default: 0])
  -e <read | fifo | dmp | motion | pedometer>, --example=<read | fifo | dmp | motion | pedometer>
                          Run the driver example.
  -h, --help              Show the help.
  -i, --information       Show the chip information.
      --interface=<iic | spi>
                          Set the chip interface.([default: iic])
  -p, --port              Display the pin connections of the current board.
  -t <reg | read | fifo | dmp | motion | pedometer>, --test=<reg | read | fifo | dmp | motion | pedometer>
                          Run the driver test.
      --times=<num>       Set the running times.([default: 3])
```

