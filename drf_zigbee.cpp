#include <string.h>
#include "drf_zigbee.h"

DRF_Zigbee::DRF_Zigbee(uint8_t _rst_pin) : rst_pin(_rst_pin), pkt_head(buffer), wptr(buffer)
{
    
}

bool DRF_Zigbee::begin(Stream * ss, uint16_t _pan_id, uint16_t timeout) {
    port = ss;
    
    port.setTimeout(DRF_ZIGBEE_WAIT_TIMEOUT);
    
    uint8_t test_cmd[] = {0xFC, 0x00, 0x91, 0x07, 0x97, 0xA7, 0xD2};
    uint8_t expected[] = {0x1, 0x2, 0x3, 0x4, 0x5};
    
    while (port->read() != -1);             // clear buffer
    
    memset(buffer, 0, DRF_ZIGBEE_BUF_SZ);
    port->write(test_cmd, 7);
    
    //while (port->available() == 0);
    
    port->read(buffer, DRF_ZIGBEE_BUF_SZ);
    
    if (memcmp(buffer, expected, 5) != 0) {
        return false;
    }
    
    while (port->read() != -1);

    if (_pan_id != 0)
        set_pan_id(_pan_id);

    return true;
}

uint16_t DRF_Zigbee::write(uint8_t c, uint16_t to_addr) {
    buffer[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;
    buffer[1] = 0x1;                           // packet length
    buffer[2] = (uint8_t)to_addr >> 8; buffer[3] = (uint8_t)(to_addr & 0xff);       // coordinator address
    buffer[4] = c;
    port->write(buffer, 5);
    return 1;
}

uint16_t DRF_Zigbee::write(const uint8_t * data, uint16_t len, uint16_t to_addr) {
    uint8_t to_write;
    uint16_t written = 0;

    buffer[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;

    while (len) {
        to_write = (len > DRF_ZIGBEE_MAX_PKT_SZ) ? DRF_ZIGBEE_MAX_PKT_SZ : len;

        buffer[1] = to_write;
        buffer[2] = (uint8_t)(to_addr >> 8); buffer[3] = (uint8_t)(to_addr & 0xff);
        memcpy(&buffer[4], data, to_write);
        port->write(buffer, to_write + 4);

        len -= to_write;
        data += to_write;
        written += to_write;
    }

    return written;
}

uint16_t DRF_Zigbee::read(uint8_t * data, uint16_t len, uint16_t from_addr) {
    uint16_t count = 0;

    uint16_t rem_buf_sz;
    uint16_t rlen = 0;

    while (port->available()) {
        if (pkt_head != buffer) {
            memmove(buffer, pkt_head, wptr - pkt_head);
            wptr = buffer + (wptr - pkt_head);
            pkt_head = buffer;
        }

        rem_buf_sz = DRF_ZIGBEE_BUF_SZ - (wptr - buffer);
        rlen = port->read(wptr, rem_buf_sz);

        if (rlen == 0)
            return count;

        wptr += rlen;

        while (pkt_head < wptr) {
            uint16_t remn = wptr - pkt_head;
            uint8_t * new_pkt = (uint8_t *)memchr(pkt_head, DRF_ZIGBEE_DATA_TRANSFER_CMD, remn);
            if (new_pkt == NULL) {
                if (pkt_head == buffer && wptr - pkt_head == DRF_ZIGBEE_BUF_SZ) {                // if buffer is full of unparsed data, none being a packet header
                    wptr = buffer;
                }
                break;
            }
            if (wptr - new_pkt < 4)         // message header is all present?
                break;
            if ((new_pkt[2] == (self_addr >> 8) && new_pkt[3] == (self_addr & 0xff))) {      // message is for us?
                uint8_t plen = new_pkt[1];
                if (plen > DRF_ZIGBEE_MAX_PKT_SZ) {          // make sure packet size is < 32
                    pkt_head++;
                    continue;
                }
                if (plen + 4 + 2 > remn)                    // if we havent read the entire packet, try to read more
                    break;

                if (plen > len) {                           // if there isnt enough space in the buffer
                    return count;
                }

                if (from_addr != DRF_ZIGBEE_ANY_ADDR) {
                    uint16_t src_addr = ((uint16_t)new_pkt[plen + 4] << 8) | new_pkt[plen + 4 + 1];
                    if (src_addr != from_addr) {                  // message came from the gateway?
                        pkt_head = new_pkt + 4 + plen + 2;          // could make us miss a message
                        break;
                    }
                }

                memcpy(data, &new_pkt[4], plen);
                data += plen;
                count += plen;
                len -= plen;
                pkt_head = new_pkt + 4 + plen + 2;
            }
            else
                pkt_head++;
        }
    }
    return count;
}

uint16_t DRF_Zigbee::available(void) {
    return port->available();
}

void DRF_Zigbee::reset(void) {
    const uint8_t preamble[] = {0xfc, 0x0, 0x91, 0x87, 0x6a, 0x35};
    memcpy(buffer, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += buffer[i];
    }
    buffer[6] = (uint8_t)(chksum & 0xff);
    port->write(buffer, 7);
}

void DRF_Zigbee::set_pan_id(uint16_t _pan_id) {
    const uint8_t preamble[] = {0xfc, 0x2, 0x91, 0x1};
    memcpy(buffer, preamble, 4);
    buffer[4] = (uint8_t)_pan_id >> 8;
    buffer[5] = (uint8_t)_pan_id & 0xff;

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += buffer[i];
    }
    buffer[6] = (uint8_t)(chksum & 0xff);
    port->write(buffer, 7);
    
    pan_id = _pan_id;
    reset();
}

uint16_t DRF_Zigbee::get_pan_id(void) {
    while (port->read() != -1);
    
    const uint8_t preamble[] = {0xfc, 0x0, 0x91, 0x3, 0xa3, 0xb3};
    memcpy(buffer, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += buffer[i];
    }
    buffer[6] = (uint8_t)(chksum & 0xff);
    port->write(buffer, 7);
    
    if (port->read(buffer, 2) == 2) {
        pan_id = (((uint16_t)buffer[0] << 8) | buffer[1]);
        return pan_id;
    }
    return 0;
}

void DRF_Zigbee::set_baud_rate(DRF_BAUD_t baud) {
    const uint8_t preamble[]  = {0xfc, 0x1, 0x91, 0x6, 0x00, 0xf6};
    memcpy(buffer, preamble, 6);
    buffer[4] = (uint8_t)baud;

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += buffer[i];
    }
    buffer[6] = (uint8_t)(chksum & 0xff);
    port->write(buffer, 7);
    
    reset();
}

uint16_t DRF_Zigbee::get_self_address(void) {
    while (port->read() != -1);
    
    const uint8_t preamble[]  = {0xfc, 0x00, 0x91, 0x04, 0xC4, 0xD4};
    memcpy(buffer, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += buffer[i];
    }
    buffer[6] = (uint8_t)(chksum & 0xff);
    port->write(buffer, 7);
    
    if (port->read(buffer, 2) == 2) {
        self_addr = (((uint16_t)buffer[0] << 8) | buffer[1]);
        return self_addr;
    }

    return 0;
}
