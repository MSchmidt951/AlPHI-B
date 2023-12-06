# AlPHI-B

The All Purpose Hardware Interface Board (AlPHI-B) is a board that can be used to control a large variety of devices, aimed towards vehicles or robots including quadcopter dones or RC cars.  
This board has features which are needed in most projects and if more capabilities are needed it has the ability to connect those additional features.  
These features include a variety of sensors, a radio and multiple motor and high current connectors in a compact form factor.

## Hardware

Microcontroller: STM32 H7 23ZET6

Circuit protection: fuse and reverse voltage protection

Voltage input connectors
- XT60
- 4mm solder pads
- 1.25mm pins

The main (5V) voltage regulator: AOZ2262AQI-11 
- Input voltage: 6V - 28V (2S - 6S LiPo)
- Max current: 9A
Secondary (3.3V) voltage regulator: AZ1084-3.3 
- Max current: 5A

Radio: nRF24l01  
[Controller](https://github.com/MSchmidt951/Multi-Purpose-Controller)

Sensors
- 2 accelerometer and gyroscope: ICM42688 and LSM6DSOX
- Magnetometer: BMM150
- Barometer: ICP 20100

Motors
- 9 Servos (3 have optional voltage output for ESCs)
- 4-in-1 ESC support (1mm JST 8 pin connector)

Screw terminals
- 1x 5V (2A max)
- 2x 3.3V (2A max total)

Breakout connectors
- 1x 6 GPIO pins
- 1x SPI with 2 CS pins
- 1x I2C/UART with 1 extra I/O pin
- 1x UART with 1 extra I/O pin

Output
- Buzzer
- RGB LED

Mounting compatible with 30mm drone stacks

## Software

The software is built to be able to adapted to work with different systems and setups as easily as possible.  
As much of the configuration as possible is done in a json file in the SD card, saving having to re-upload the whole code again or plugging the board into a computer.

Max update rate: 4kHz, higher rates may be possible

The state of the device is logged on the SD card, as often as every loop.  
When the board is running the state is converted into binary then saved to the SD card. When shutting down the log is converted to a csv file.  
