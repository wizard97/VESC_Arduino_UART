#include "VescSerial.h"

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

VescSerial *VescSerial::_active = NULL;

VescSerial::VescSerial(SoftwareSerial &serial, uint32_t baud, void (*msgHandler)(VescSerial &vesc, COMM_PACKET_ID type, void *msg))
: _serial(serial), _msgHandler(msgHandler)
{
	_baud = baud;
    _timeout = 0;
	_request = false;
    memset(&_packet, 0, sizeof(_packet));
	//memset(&_values, 0, sizeof(_values));
	_lastRecv = millis();
}


bool VescSerial::requestVersion()
{
	if (!getLock())
		return false;

	_request = true;

    unsigned char buf[1];
    int16_t send_index = 0;
    buf[send_index++] = COMM_FW_VERSION;
    sendPacket(buf, send_index);

	_timeout = millis();
	return true; //must unlock later
}

bool VescSerial::requestValues()
{
	if (!getLock())
		return false;

	_request = true;

    unsigned char buf[1];
    int16_t send_index = 0;
	buf[send_index++] = COMM_GET_VALUES;
	sendPacket(buf, send_index);

	_timeout = millis();
	return true; //must unlock later
}

bool VescSerial::setDuty(float dutyCycle)
{
	if (!getLock())
		return false;

	unsigned char buf[5];
	int16_t send_index = 0;
	buf[send_index++] = COMM_SET_DUTY;
	buffer_append_float32(buf, dutyCycle, 100000.0, &send_index);
	sendPacket(buf, send_index);

	return unlock();
}

bool VescSerial::setCurrent(float current)
{
	if (!getLock())
		return false;

    unsigned char buf[5];
    int16_t send_index = 0;
	buf[send_index++] = COMM_SET_CURRENT;
	buffer_append_float32(buf, current, 1000.0, &send_index);
	sendPacket(buf, send_index);

	return unlock();
}


bool VescSerial::setCurrentBrake(float current)
{
	if (!getLock())
		return false;

    unsigned char buf[5];
    int16_t send_index = 0;
	buf[send_index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(buf, current, 1000.0, &send_index);
	sendPacket(buf, send_index);

	return unlock();
}

bool VescSerial::setRPM(uint32_t rpm)
{
	if (!getLock())
		return false;

    unsigned char buf[5];
    int16_t send_index = 0;
    buf[send_index++] = COMM_SET_RPM;
    buffer_append_int32(buf, rpm, &send_index);
    sendPacket(buf, send_index);

	return unlock();
}

bool VescSerial::reboot()
{
	if (!getLock())
		return false;

    unsigned char buf[1];
    int16_t send_index = 0;
    buf[send_index++] = COMM_REBOOT;
    sendPacket(buf, send_index);

	return unlock();
}


// only call if lock
void VescSerial::sendPacket(unsigned char *data, unsigned int len)
{
    if (len > PACKET_MAX_PL_LEN) {
		return;
	}

	int b_ind = 0;

	if (len <= 256) {
		_packet.tx_buffer[b_ind++] = 2;
		_packet.tx_buffer[b_ind++] = len;
	} else {
		_packet.tx_buffer[b_ind++] = 3;
		_packet.tx_buffer[b_ind++] = len >> 8;
		_packet.tx_buffer[b_ind++] = len & 0xFF;
	}

	memcpy(_packet.tx_buffer + b_ind, data, len);
	b_ind += len;

	unsigned short crc = crc16(data, len);
	_packet.tx_buffer[b_ind++] = (uint8_t)(crc >> 8);
	_packet.tx_buffer[b_ind++] = (uint8_t)(crc & 0xFF);
	_packet.tx_buffer[b_ind++] = 3;
	_packet.tx_buffer[b_ind] = 0;

    _serial.write(_packet.tx_buffer, b_ind);
}


void VescSerial::processPacket(unsigned char *data, unsigned int len)
{
    COMM_PACKET_ID type = (COMM_PACKET_ID)data[0];
    len--; data++;

    int16_t ind = 0;
    switch (type)
    {
        case COMM_GET_VALUES:
			mc_values values;
            values.temp_mos1 = buffer_get_float16(data, 10.0, &ind);
            values.temp_mos2 = buffer_get_float16(data, 10.0, &ind);
            values.temp_mos3 = buffer_get_float16(data, 10.0, &ind);
            values.temp_mos4 = buffer_get_float16(data, 10.0, &ind);
            values.temp_mos5 = buffer_get_float16(data, 10.0, &ind);
            values.temp_mos6 = buffer_get_float16(data, 10.0, &ind);
            values.temp_pcb = buffer_get_float16(data, 10.0, &ind);
            values.current_motor = buffer_get_float32(data, 100.0, &ind);
            values.current_in = buffer_get_float32(data, 100.0, &ind);
            values.duty_now = buffer_get_float16(data, 1000.0, &ind);
            values.rpm = buffer_get_float32(data, 1.0, &ind);
            values.v_in = buffer_get_float16(data, 10.0, &ind);
            values.amp_hours = buffer_get_float32(data, 10000.0, &ind);
            values.amp_hours_charged = buffer_get_float32(data, 10000.0, &ind);
        	values.watt_hours = buffer_get_float32(data, 10000.0, &ind);
            values.watt_hours_charged = buffer_get_float32(data, 10000.0, &ind);
            values.tachometer = buffer_get_int32(data, &ind);
            values.tachometer_abs = buffer_get_int32(data, &ind);
        	values.fault_code = (mc_fault_code)data[ind++];
            _msgHandler(*this, type, &values);
            break;

        case COMM_FW_VERSION:
            int fw_ver[2];
            if (len == 2) {
                ind = 0;
                fw_ver[0] = data[ind++]; // major
                fw_ver[1] = data[ind++]; //minor
            } else {
                fw_ver[0] = -1; // major
                fw_ver[1] = -1; //minor
            }
            _msgHandler(*this, type, fw_ver);
            break;

        default:
            _msgHandler(*this, type, data); //sensible default
            break;
    }
}


// State machine
bool VescSerial::service()
{
	if (_active != this)
		return false;

    uint8_t rx_data;

    // check if timeout

    uint32_t diff = (millis() - _timeout);
    if (!_serial.available() && diff) {
        if (_packet.rx_timeout)
            _packet.rx_timeout--;

        if (_packet.rx_timeout == 0) {
            _packet.rx_timeout = 0;
            _packet.rx_state = 0;

			unlock();
        }
		_timeout = millis();
		return false;
    }

    if (_serial.available()) {
		_timeout = millis();
        rx_data = _serial.read();
    } else {
        return false;
    }

    switch (_packet.rx_state) {
    case 0:
        if (rx_data == 2) {
            // 1 byte PL len
            _packet.rx_state += 2;
            _packet.rx_timeout = PACKET_RX_TIMEOUT;
            _packet.rx_data_ptr = 0;
            _packet.payload_length = 0;
        } else if (rx_data == 3) {
            // 2 byte PL len
            _packet.rx_state++;
            _packet.rx_timeout = PACKET_RX_TIMEOUT;
            _packet.rx_data_ptr = 0;
            _packet.payload_length = 0;
        } else {
            _packet.rx_state = 0;
        }
        break;

    case 1:
        _packet.payload_length = (unsigned int)rx_data << 8;
        _packet.rx_state++;
        _packet.rx_timeout = PACKET_RX_TIMEOUT;
        break;

    case 2:
        _packet.payload_length |= (unsigned int)rx_data;
        if (_packet.payload_length > 0 &&
                _packet.payload_length <= PACKET_MAX_PL_LEN) {
            _packet.rx_state++;
            _packet.rx_timeout = PACKET_RX_TIMEOUT;
        } else {
            _packet.rx_state = 0;
        }
        break;

    case 3:
        _packet.rx_buffer[_packet.rx_data_ptr++] = rx_data;
        if (_packet.rx_data_ptr == _packet.payload_length) {
            _packet.rx_state++;
        }
        _packet.rx_timeout = PACKET_RX_TIMEOUT;
        break;

    case 4:
        _packet.crc_high = rx_data;
        _packet.rx_state++;
        _packet.rx_timeout = PACKET_RX_TIMEOUT;
        break;

    case 5:
        _packet.crc_low = rx_data;
        _packet.rx_state++;
        _packet.rx_timeout = PACKET_RX_TIMEOUT;
        break;

    case 6:
        if (rx_data == 3) {
            if (crc16(_packet.rx_buffer, _packet.payload_length)
                    == ((unsigned short)_packet.crc_high << 8
                            | (unsigned short)_packet.crc_low)) {
                // Packet received!
                // Skip the start byte and crc
				_lastRecv = millis();
                processPacket(_packet.rx_buffer, _packet.payload_length);

            }
        }
        _packet.rx_state = 0;
		unlock();
        break;

    default:
        _packet.rx_state = 0;
        break;
    }
	return true;
}

bool VescSerial::getLock()
{
	if (_active != this && _request)
		return false;

	_active = this;
	_serial.begin(_baud);
	return true;
}


bool VescSerial::unlock()
{
	if (_active != this)
		return false;
	_serial.flush();
	_serial.end();
	_active = NULL;
	_request = false;
	return true;
}

void VescSerial::waitAvailable()
{
	while (VescSerial::usingSerial())
		_active->service();
}
