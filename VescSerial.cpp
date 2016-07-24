#include "VescSerial.h"

VescSerial::VescSerial()
{
    _timeout = millis();
    memset(&_packet, 0, sizeof(_packet));
}

VescSerial::begin(uint16_t baud)
{
    _serial.begin(baud);
}

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

    _serial.write(_packet.tx_buffer, b_ind);
}


// State machine
void VescSerial::service()
{
    uint8_t rx_data;

    // check if timeout
    int32_t diff = (int32_t)(millis() - _timeout);
    if (!_serial.available() && diff) {
        if (_packet.rx_timeout)
            _packet.rx_timeout -= diff;

        if (_packet.rx_timeout <= 0) {
            _packet.rx_timeout = 0;
            _packet.rx_state = 0;
        }
    }

    if (_serial.available()) {
        rx_data = _serial.read();
    } else {
        return;
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
                processPacket(_packet.rx_buffer+3, _packet.payload_length - 3 -2);

            }
        }
        _packet.rx_state = 0;
        break;

    default:
        _packet.rx_state = 0;
        break;
    }
}
