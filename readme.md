## AIM EtherCAT - Custom Slave Control Board

**MCU**: STM32H750

**ESC**: AX58100

**Support Tasks (Tested)**:

They're tested by various test cases, and should be working fine.

* RC
    * DBUS
    * SBUS
* ACTUATOR
    * LK
    * DM
    * DJI
    * DSHOT
    * PWM (Onboard 4x2 channel)
* SENSOR
    * HIPNUC IMU (CAN)

**Untested Tasks**:

They're probably not working or are not working properly.

Testing is in progress and will be updated.

* ACTUATOR
    * PWM (External 4x4 channel)
* SENSOR
  * MS5837 PRESSURE SENSOR (I2C)
  * PMU (CAN2, UAVCAN PROTOCOL ONLY)

![img.png](docs/img.png)

~~Dev/Deploy tutorials are on the way...~~