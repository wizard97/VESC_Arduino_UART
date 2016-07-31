#ifndef ARDUINO_VESC_SERIAL_H
#define ARDUINO_VESC_SERIAL_H

#include "Arduino.h"
#include "SoftwareSerial.h"
#include <string.h>

#include "datatypes.h"
#include "buffer.h"
#include "packet.h"
#include "crc.h"

#define VESC_BAUD_RATE 38400

#define debug Serial

const char* bldc_interface_fault_to_string(mc_fault_code fault);

class VescSerial
{
public:
    VescSerial(SoftwareSerial &serial, uint32_t baud, void (*msgHandler)(VescSerial &vesc, COMM_PACKET_ID type, void *msg));
    bool service();

    //setters
    bool setDuty(float dutyCycle);
    bool setCurrent(float current);
    bool setCurrentBrake(float current);
    bool setRPM(uint32_t rpm);
    bool reboot();

    // Block until SoftwareSerial is freed by another ESC
    //services the active ESC until VescSerial::usingSerial() return false
    // This will happen once the ESC gets the message it is waiting for or unti it times out
    void waitAvailable();


    //Getters, the msgHandler will be invoked when it suceeds
    bool requestValues();
    bool requestVersion();

    uint32_t getLastRecv() { return _lastRecv; }

    static bool usingSerial() { return _active; }

protected:
    void sendPacket(unsigned char *data, unsigned int len);
    void processPacket(unsigned char *data, unsigned int len);

    bool getLock();
    bool unlock();

    static VescSerial *_active;; //mutex for SoftwareSerial
    bool _request;
private:
    SoftwareSerial _serial;
    PACKET_STATE_t _packet; // from packet.h
    uint32_t _timeout;
    uint32_t _lastRecv;
    uint32_t _baud;

    //mc_values _values;

    void (*_msgHandler)(VescSerial &vesc, COMM_PACKET_ID type, void *msg);
};

#endif
