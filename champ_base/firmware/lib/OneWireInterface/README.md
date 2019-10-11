# OneWireInterface

Implements the Dynamixel 1.0 protocol to communicate on half-duplex UART with any compatible device

> Warning: this library doesn't implement the Dallas Semiconductor's 1-Wire Protocol. It is the Dynamixel v1.0

This library provides an interface to communicate through half-duplex UART using the Dynamixel 1.0 protocol as a master or as a slave. It can be used to implement easily a device control library for any device using this protocol, or to implement the device's firmware itself.

## Remark

It is provided as an Arduino library.  
By default, the Arduino IDE will include both `<OneWireMInterface.h>` and `<OneWireSInterface.h>`, usually only one is needed, you can remove the unnecessary include.

## Files

`<OneWireInterface.h>`: common definitions  
`<OneWireMInterface.h>`: master interface  
`<OneWireSInterface.h>`: slave interface  

## Dependecies

* Arduino core

## License

This library is released under the MIT License.
