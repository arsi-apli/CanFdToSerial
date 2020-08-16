# CanFdToSerial

Thanks to Eugene Krashtan and Pontus Borg for the inspiration.

Serial port emulation over CAN FD bus.
Working on Raspberry Pi with MCP2517 module.
Main goal - emulate serial port to use Duet 3 Toolboard 1LC together with
Klipper controller for 3D-printers https://github.com/KevinOConnor/klipper


## Protocol

Klipper sends serial messages in the format  <len><sequence><data><crc><crc><end>
So each message has an extra 5 bytes, the application transmits only pure data.
The message length and sequence are transmitted via the CAN ID.
The CRC does not need to be transmitted because it is part of the CAN FD.
After receiving response from the MCU, the message is wrapped in the original message format 
and is sent to the virtual serial port.


## Build procedure

```
cd /home/pi
git clone git@github.com:arsi-apli/CanFdToSerial.git
cd CanFdToSerial
make CONF=Release
cd dist/Release/GNU-Linux
sudo ./canfdtoserial can0mcu1fd
```
sudo is necessary because the application runs commands:
```
ip link set %s down
ip link set %s up type can bitrate 1000000   dbitrate 1000000 restart-ms 1000 berr-reporting on fd on
```
on the configured port for the first time.
To change the line parameters, you need to edit the code in main.c:main

The application runs in multi instance mode: one application  = one MCU
can0mcu0fd - can0 MCU0
can0mcu1fd - can0 MCU1

can1mcu0fd - can1 MCU0
can1mcu1fd - can1 MCU1

fd - CAN FD
sd - CAN 2.0B Currently not supported

The application creates a serial port according to the command line argument:
/tmp/ttyCAN0MCU0
/tmp/ttyCAN0MCU1

/tmp/ttyCAN1MCU0
/tmp/ttyCAN1MCU1
And so on..

## Run CanSerial as service
The application and service runs in multi instance mode: one application  = one MCU
can0mcu0fd - can0 MCU0
can0mcu1fd - can0 MCU1

can1mcu0fd - can1 MCU0
can1mcu1fd - can1 MCU1

And so on..
fd - CAN FD
sd - CAN 2.0B Currently not supported

```
cd cd /home/pi/CanFdToSerial/systemd
$ sudo cp canfdtoserial@.service /lib/systemd/system/canfdtoserial@.service
$ sudo systemctl daemon-reload
$ sudo systemctl enable canfdtoserial@can0mcu0fd.service
$ sudo systemctl enable canfdtoserial@can0mcu1fd.service
$ sudo systemctl start canfdtoserial@can0mcu0fd.service
$ sudo systemctl start canfdtoserial@can0mcu1fd.service
```


