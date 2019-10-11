/*
    Name:       ax12_example.ino
    Created:	01/09/2019 17:39:45
    Author:     Sylvain Gaultier
*/


#include <DynamixelAX12.h>

/*
    This example is supposed to work as-is with Arduino UNO/Mega and Teensy 3.x 
    For other platforms, you may need to change the HardwareSerial used
*/

#if defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_MEGA || defined ARDUINO_AVR_ADK || defined ARDUINO_AVR_LEONARDO || defined TEENSYDUINO
#define SERIAL_AX Serial1
#define SERIAL_DBG Serial
#define BAUDRATE_DBG 115200
#else
#define SERIAL_AX Serial
#endif

#define AX12_ID 1
#define AX12_BAUDRATE 1000000


void display_debug(const char* name, uint8_t com_status, const DynamixelAX12 & ax12)
{
#ifdef SERIAL_DBG
    SERIAL_DBG.print(name);
    SERIAL_DBG.print(": com=");
    SERIAL_DBG.print(com_status);
    SERIAL_DBG.print(" ax12=");
    SERIAL_DBG.println(ax12.status());
#endif
    if (com_status != OW_STATUS_OK || ax12.error()) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
    }
}

void setup() {}

void loop()
{
#ifdef SERIAL_DBG
    SERIAL_DBG.begin(BAUDRATE_DBG);
#endif
    delay(1000);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    /* One-wire interface */
    /* The provided HardwareSerial must be used only by one interface */
    OneWireMInterface ax12Interface(SERIAL_AX);

    /* Dynamixel AX12 */
    /* Define as many as needed, if they use the same interface they must have distinct IDs */
    DynamixelAX12 ax12(ax12Interface, AX12_ID);

    /* Begin the interface like a regular Stream, providing the baudrate and optionally the timeout */
    /* The default timeout is 50 ms */
    ax12Interface.begin(AX12_BAUDRATE, 100);

    OneWireStatus com_status;
    /* AX12 initialization */
    com_status = ax12.init();
    display_debug("init", com_status, ax12);
    if (com_status != OW_STATUS_OK) {
        /* 
            If the init failed, the Status Return Level is set to 0
            Afterwards, commands will still be sent but without expecting an answer,
            thus they will always succeed even though it is very unlikely that the command
            was actually received and executed.

            If it is normal to be unable to receive data from the AX12 (because the ID is
            the broadcast ID, or because the RX pin is not connected) you should not call
            the 'init' method at all.
        */
        delay(10000);
        return;
    }

    /* Set AX12 in joint mode (as opposed to wheel mode), with maximum stroke length */
    com_status = ax12.jointMode();
    display_debug("jointMode", com_status, ax12);

    /* Enable torque so that the AX12 can rotate by itself */
    com_status = ax12.enableTorque();
    display_debug("enableTorque", com_status, ax12);

    /* If possible, display the entire memory of the AX12 */
#ifdef SERIAL_DBG
    if (ax12.statusReturnLevel() == 0) {
        SERIAL_DBG.println("Status Return Level is 0. Cannot read AX12 registers.");
    }
    else {
        SERIAL_DBG.println("Addr\tValue");
        for (size_t i = 0; i < 50; i++) {
            uint8_t data = 0;
            com_status = ax12.read(i, data);
            if (com_status != OW_STATUS_OK) {
                SERIAL_DBG.print("Errno: ");
                SERIAL_DBG.println(com_status);
            }
            SERIAL_DBG.print(i);
            SERIAL_DBG.print("\t");
            SERIAL_DBG.println(data);
        }
    }
#endif

    /* Move the AX12 */
    for (size_t i = 0; i < 20; i++) {
        com_status = ax12.goalPositionDegree(140);
        display_debug("goalPositionDegree(140)", com_status, ax12);
        delay(1000);
        com_status = ax12.goalPositionDegree(160);
        display_debug("goalPositionDegree(160)", com_status, ax12);
        delay(1000);
    }

    /* Close the interface */
    ax12Interface.end();

#ifdef SERIAL_DBG
    SERIAL_DBG.end();
#endif
}
