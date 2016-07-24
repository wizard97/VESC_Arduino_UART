#ifndef VESC_SERIAL_H
#define VESC_SERIAL_H

#include "Arduino.h"
#include "SoftwareSerial.h"
#include <string.h>

#include "buffer.h"
#include "packet.h"
#include "crc.h"

#define VESC_BAUD_RATE 9600;

class VescSerial
{
public:
    VescSerial(SoftwareSerial &serial);
    void begin(uint16_t baud = VESC_BAUD_RATE);
    void service();

protected:
    void sendPacket(unsigned char *data, unsigned int len);
private:
    SoftwareSerial _serial;
    PACKET_STATE_t _packet; // from packet.h
    uint32_t _timeout;
}

#endif
