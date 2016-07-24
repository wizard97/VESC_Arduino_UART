#ifndef ARDUINO_VESC_SERIAL_H
#define ARDUINO_VESC_SERIAL_H

#include "Arduino.h"
#include "SoftwareSerial.h"
#include <string.h>

#include "datatypes.h"
#include "buffer.h"
#include "packet.h"
#include "crc.h"

#define VESC_BAUD_RATE 9600;

class VescSerial
{
public:
    VescSerial(SoftwareSerial &serial, void (*msgHandler)(COMM_PACKET_ID type, void *msg));
    void begin(uint16_t baud = VESC_BAUD_RATE);
    void service();

    //setters
    void setCurrent(float current);
    void setCurrentBrake(float current);
    void setRPM(uin32_t rpm);
    void reboot();

    //Getters, the msgHandler will be invoked when it suceeds
    void requestValues();
    void requestVersion()

protected:
    void sendPacket(unsigned char *data, unsigned int len);
    void processPacket(unsigned char *data, unsigned int len);
private:
    SoftwareSerial _serial;
    PACKET_STATE_t _packet; // from packet.h
    uint32_t _timeout;

    void (*_msgHandler)(COMM_PACKET_ID type, void *msg);
};

//helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE";
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
	case FAULT_CODE_DRV8302: return "FAULT_CODE_DRV8302";
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
	default: return "Unknown fault";
	}
}
#endif
