#ifndef ARDUINO_VESC_SERIAL_H
#define ARDUINO_VESC_SERIAL_H

#include "Arduino.h"
#include "SoftwareSerial.h"
#include <string.h>

#include "datatypes.h"
#include "buffer.h"
#include "packet.h"
#include "crc.h"

#define VESC_BAUD_RATE 9600

const char* bldc_interface_fault_to_string(mc_fault_code fault);

class VescSerial
{
public:
    VescSerial(SoftwareSerial &serial, void (*msgHandler)(COMM_PACKET_ID type, void *msg));
    void begin(uint16_t baud = VESC_BAUD_RATE);
    void service();

    //setters
    void setCurrent(float current);
    void setCurrentBrake(float current);
    void setRPM(uint32_t rpm);
    void reboot();

    //Getters, the msgHandler will be invoked when it suceeds
    void requestValues();
    void requestVersion();

protected:
    void sendPacket(unsigned char *data, unsigned int len);
    void processPacket(unsigned char *data, unsigned int len);
private:
    SoftwareSerial _serial;
    PACKET_STATE_t _packet; // from packet.h
    uint32_t _timeout;

    void (*_msgHandler)(COMM_PACKET_ID type, void *msg);
};

#endif
