MPU-9250 and BMP280
========
This fork changes only the MPU9250_BMP280_BasicAHRS_t3.ino to enable use of a NANO with the chinese (purple) GY-91 (=GY91) board.
You will need to copy both MPU9250_BMP280_BasicAHRS_t3.ino and quaternionFilters.ino into a singlefolder to compile it.

Nano and Purple Chinese GY-91 Hardware wiring setup:

 Nano --------- GY-91
 
 3V3 ----------- 3V3
 
 GND ----------- GND
 
 A4 (SDA) ------ SDA
 
 A5 (SCL) ------ SCL
 

changes from Kris' original: 
             update to enable use of Arduino Nano
             use #def and strings to decrease duplicate code and decrease firmware/on-chip size
             converted altitude estimation to metres
             Serial.print data in a table format to enable easy comparison of changes
             increase initial USER hard calibation variables
             PS the code looks dramatically messier than the original due to large number of #ifdefs...

I converted this code some time ago to make a sports watch (but they're now so cheap there probably isn't much need). Now wondering about a DIY version of the WOO, Verge, or PIQ for measuring tricks and jumps for kiteboarding, snowboarding, etc as I think £170 is a bit steep for something which could get lost or broken (or made for <£10), and irritatingly the cheap watches don't expose their accelerometers. Of course you could just attaching an old phone to the board, but if not this code can easily be extended to write to SD card or bluetooth to phone with an HC05 or similar breakout.
Then again using a Teensy 3.x would make the project dramatically more accurate and 3.5/3.6 have an onboard SD slot, so I might just end up back with Kris' original code...





The following is forked without change from the original README:

Most modern and correct version is:  	MPU9250_MS5637_AHRS_t3.ino, all require quaternionFilters.ino in the IDE folder also to use the Madgwick and/or Mahony sensor fusion algorithms.

Demonstrate MPU-9250 basic functionality including parameterizing the register addresses, initializing the sensor, 
getting properly scaled accelerometer, gyroscope, and magnetometer data out, calibration and self-test of sensors.
Added display functions to allow display to on-breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

A discussion of the use and limitations of this sensor and sensor fusion in general is found [here.](https://github.com/kriswiner/MPU-6050/wiki/Affordable-9-DoF-Sensor-Fusion)

I have also added a program to allow sensor fusion using the MPU-9250 9-axis motion sensor with the STM32F401 Nucleo board using the mbed compiler. The STM32F401 achieves a sensor fusion filter update rate using the Madgwick MARG fusion filter of 4800 Hz running the M4 Cortex ARM processor at 84 MHz; compare to the sensor fusion update rate of 2120 Hz achieved using the same filter with the Teensy 3.1 running its M4 Cortex ARM processor at 96 MHz.

One reason for this difference is the single-precision floating point engine embedded in the STM32F401 core. While both ARM processors achieve impressive rates of filtering, really more than necessary for most applications, the factor of two difference translates into much lower power consumption for the same sensor fusion performance. If adequate sensor fusion filtering, say, 1000 Hz, can be achieved at much lower processor clock speed, then over all power consumption will be reduced. This really matters for wearable and other portable motion sensing and control applications.

I added a version of the basic sketch that uses the i2c_t3.h 'Wire' library specifically designed for Teensy 3.1. It allows easy access to Teensy-specific  capabilities such as specification of which set of hardware i2c pins will be used, the bus speed (up to 1 MHz!) and also allows master and/or slave designation to handle multiplexing between i2c devices. See www.pjrc.com/teensy and  http://forum.pjrc.com/threads/21680-New-I2C-library-for-Teensy3 for details.

I added another version of the sketch intended specifically for the [MPU9250_MS5637 Mini Add-On shield](https://www.tindie.com/products/onehorse/mpu9250-teensy-31-add-on-shields/) for the Teensy 3.1. 

![](https://d3s5r33r268y59.cloudfront.net/44691/products/thumbs/2014-07-22T02:09:32.088Z-MPU9250micro1.png.114x76_q85_pad_rcrop.png) ![](https://d3s5r33r268y59.cloudfront.net/44691/products/thumbs/2014-07-22T02:00:54.264Z-mpu9250mini1.png.114x76_q85_pad_rcrop.png) ![](https://d3s5r33r268y59.cloudfront.net/44691/products/thumbs/2014-07-22T02:09:32.088Z-mpu9250mini2.png.114x76_q85_pad_rcrop.png)

_MPU9250 + MS5637 Micro (left) and Mini (right) add-on shields, which solder onto the bottom pads 23-34 or pins 8 -17 on the [Teensy 3.1](http://store.oshpark.com/products/teensy-3-1), respectively._

It can be simply modified to work with the corresponding micro shield as well. It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h. The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution mode with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of only 1 microAmp. The choice will depend on the application. The sketch calculates and outputs temperature in degrees Centigrade, pressure in millibar, and altitude in feet. In high resolution mode, the pressure is accurate to within 10 Pa or 0.1 millibar, and the height discrimination is about 13 cm. This is much better performance than achievable from the venerable MPL3115A2 and the MS5637 is in a very small package perfect for the small micro and mini add-on Teensy 3.1 shields.

For a discussion of the relative merits of modern board-mounted pressure sensors, see [here](https://github.com/kriswiner/MPU-9250/wiki/Small-pressure-sensors).

I added sketches for the various new Mini add-on shields for Teensy 3.1 with the MPU9250 9-axis motion sensor and either the MPL3115A2 or the newer LPS25H pressure sensor/altimeter. Now there are three flavors of 10 DoF Mini add-on boards specially designed for the Teensy 3.1 with state-of-the-art 20-bit (MPL3115A2) and 24-bit (MS5637 and LPS25H) altimeters. The LPS25H has a 32-byte FIFO and sophisticated hardware filtering which allows very low power operation while maintaining 0.01 millibar resolution. This is really quite a feat; this sensor deserves serious consideration for any airborne application you might have in mind.
