// Written by Brian Ejike, 2018
// Under the MIT license

#include <string.h>
#include "drf_zigbee.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_STM32)

DRF_Zigbee::DRF_Zigbee(Stream * ss, uint8_t _rst_pin) : 
    out_fifo(_out_buf, DRF_ZIGBEE_MAX_BUFFERED_PKTS), 
    rst_pin(_rst_pin), last_send(0), 
    port(ss), read_state(DRF_STATE_READ_HEADER)
{
    if (rst_pin) {
        pinMode(rst_pin, OUTPUT);
        digitalWrite(rst_pin, HIGH);
    }
}

#else

DRF_Zigbee::DRF_Zigbee(Stream * ss, uint8_t _rst_pin) : 
    rst_pin(_rst_pin), port(ss),
    read_state(DRF_STATE_READ_HEADER)
{
    if (rst_pin) {
        pinMode(rst_pin, OUTPUT);
        digitalWrite(rst_pin, HIGH);
    }
}

#endif

// Timeout should be at least 500ms; can probably be shortened with more tests
bool DRF_Zigbee::begin(uint16_t _pan_id) {
    port->setTimeout(DRF_ZIGBEE_DEFAULT_TIMEOUT);
    
    reset();
    
    uint8_t test_cmd[] = {0xFC, 0x00, 0x91, 0x07, 0x97, 0xA7, 0xD2};
    uint8_t expected[] = {0x1, 0x2, 0x3, 0x4, 0x5};
    
    while (port->read() != -1) yield();            // clear buffer
    
    memset(obuf, 0, DRF_ZIGBEE_BUF_SZ);
    memset(ibuf, 0, DRF_ZIGBEE_BUF_SZ);
    
    port->write(test_cmd, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    port->readBytes(obuf, 5);
    
    if (memcmp(obuf, expected, 5) != 0)
        return false;
    
    while (port->read() != -1) yield();
    
    pan_id = get_pan_id();
    
    if (_pan_id != 0 && pan_id != _pan_id) {
        set_pan_id(_pan_id);
        while (get_pan_id() != _pan_id) yield();
    }

    return true;
}

uint16_t DRF_Zigbee::write(uint8_t c, uint16_t to_addr) {
    obuf[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;
    obuf[1] = 0x1;                           // packet length
    obuf[2] = (uint8_t)to_addr >> 8; obuf[3] = (uint8_t)(to_addr & 0xff);       // coordinator address
    obuf[4] = c;
    port->write(obuf, 5);
    
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    return 1;
}

/* Writes the passed data to the module
 * with the mandatory DRF_ZIGBEE_INTER_PACKET_INTERVAL ms between
 * packets. Delay value was obtained by trial and error
 * Function blocks till the wait is done
 */
uint16_t DRF_Zigbee::write(const uint8_t * data, uint16_t len, uint16_t to_addr) {
    uint8_t to_write;
    uint16_t written = 0;

    obuf[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;

    while (len) {
        to_write = (len > DRF_ZIGBEE_MAX_PKT_SZ) ? DRF_ZIGBEE_MAX_PKT_SZ : len;

        obuf[1] = to_write;
        obuf[2] = (uint8_t)(to_addr >> 8); obuf[3] = (uint8_t)(to_addr & 0xff);
        memcpy(&obuf[4], data, to_write);
        port->write(obuf, to_write + 4);

        len -= to_write;
        data += to_write;
        written += to_write;
        
        delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);          // wait for the shitty module
    }

    return written;
}

// Writes a single packet
uint16_t DRF_Zigbee::write_packet(const uint8_t * data, uint16_t len, uint16_t to_addr) {
    uint8_t to_write;
    obuf[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;

    if (len == 0)               // no zero-length writes allowed
        return 0;
    
    to_write = (len > DRF_ZIGBEE_MAX_PKT_SZ) ? DRF_ZIGBEE_MAX_PKT_SZ : len;

    obuf[1] = to_write;
    obuf[2] = (uint8_t)(to_addr >> 8); obuf[3] = (uint8_t)(to_addr & 0xff);
    memcpy(&obuf[4], data, to_write);
    port->write(obuf, to_write + 4);
    
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);

    return to_write;
}

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_STM32)
/* Just like write() except there's no delay,
 * Packets get buffered and are finally sent when flush() is called
 * At most 32 packets of DRF_ZIGBEE_MAX_PKT_SZ bytes are buffered (1k bytes for now)
 * For now, only supported for ESP8266, ESP32 and STM32 since they can spare the RAM
 * Returns number of bytes buffered
 */
 
uint16_t DRF_Zigbee::buffered_write(const uint8_t * data, uint16_t len, uint16_t to_addr) {
    uint8_t to_write;
    uint16_t written = 0;

    while (len) {
        if (out_fifo.full())
            return written;
        to_write = (len > DRF_ZIGBEE_MAX_PKT_SZ) ? DRF_ZIGBEE_MAX_PKT_SZ : len;

        temp_pkt.len = to_write;
        temp_pkt.dest = to_addr;
        memcpy(temp_pkt.bytes, data, to_write);
        out_fifo.enqueue(&temp_pkt);

        len -= to_write;
        data += to_write;
        written += to_write;
    }

    return written;
}

/* Just like buffered_write(), except it buffers only one packet
 * Returns number of bytes in buffered packet
 */
uint16_t DRF_Zigbee::buffered_write_packet(const uint8_t * data, uint16_t len, uint16_t to_addr) {
    while (out_fifo.full()) {                       // chill till there's space
        flush();
        delay(1);
    }
    
    uint8_t to_write = (len > DRF_ZIGBEE_MAX_PKT_SZ) ? DRF_ZIGBEE_MAX_PKT_SZ : len;

    temp_pkt.len = to_write;
    temp_pkt.dest = to_addr;
    memcpy(temp_pkt.bytes, data, to_write);
    out_fifo.enqueue(&temp_pkt);

    return to_write;
}

/* Should be called as often as possible 
 * (ideally every DRF_ZIGBEE_INTER_PACKET_INTERVAL ms) to dispatch packets
 */
void DRF_Zigbee::flush(void) {
    if (out_fifo.available() == 0)
        return;
    if ((uint32_t)(millis() - last_send) < DRF_ZIGBEE_INTER_PACKET_INTERVAL)
        return;
    out_fifo.dequeue(&temp_pkt);
    
    tempbuf[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;
    tempbuf[1] = temp_pkt.len;
    tempbuf[2] = (uint8_t)(temp_pkt.dest >> 8); tempbuf[3] = (uint8_t)(temp_pkt.dest & 0xff);
    memcpy(&tempbuf[4], temp_pkt.bytes, temp_pkt.len);
    port->write(tempbuf, temp_pkt.len + 4);
    last_send = millis();
}

#endif

/* Reads a single packet in non-blocking manner,
 * returning the src and dest address if needed.
 * If there's no complete packet, it returns DRF_ZIGBEE_READ_INCOMPLETE
 * If the passed buffer is too small, it returns 0 and doesn't read from Stream
 * until a sufficient buffer is passed
 * A value > 0 refers to the packet length
 */
int16_t DRF_Zigbee::read_packet(uint8_t * data, uint16_t len, uint16_t * from_addr, uint16_t * to_addr) {
    static uint16_t src_addr = 0;
    static uint16_t dest_addr = 0;
    static uint16_t to_read = 0;
    
    while (true) {
        switch (read_state) {
            case DRF_STATE_READ_HEADER:
                if (port->available() < 4)
                    return DRF_ZIGBEE_READ_INCOMPLETE;
                if (port->read() != DRF_ZIGBEE_DATA_TRANSFER_CMD)
                    return DRF_ZIGBEE_READ_INCOMPLETE;
                
                to_read = port->read();
                if (to_read > DRF_ZIGBEE_MAX_PKT_SZ) {
                    yield();
                    continue;
                }
                
                port->readBytes(ibuf, 2);
                dest_addr = 0;
                dest_addr = ((uint16_t)ibuf[0] << 8) | ibuf[1];
                
                read_state = DRF_STATE_READ_CONTENTS;
                break;
                
            case DRF_STATE_READ_CONTENTS:
                if (port->available() < to_read + 2)            // +2 for src address
                    return DRF_ZIGBEE_READ_INCOMPLETE;
                if (to_read > len)
                    return 0;
                port->readBytes(data, to_read);
                port->readBytes(ibuf, 2);
                
                src_addr = 0;
                src_addr = ((uint16_t)ibuf[0] << 8) | ibuf[1];
                if (from_addr != NULL)
                    *from_addr = src_addr;
                if (to_addr != NULL)
                    *to_addr = dest_addr;
                
                read_state = DRF_STATE_READ_HEADER;
                return to_read;
            default:
                break;
        }
        yield();
    }
}

/* Returns the number of unread
 * Stream bytes. A return value > 0 only means a read_packet()
 * can be attempted, with no guarantee that it will succeed. 
 * However if this function ever returns 0, it's guaranteed that
 * there's nothing useful to read
 */
uint16_t DRF_Zigbee::available(void) {
    return port->available();
}

/* First tries to restart module using its reset pin if one was given,
 * next tries to use a command (not supported on all modules)
 */
void DRF_Zigbee::reset(void) {
    if (rst_pin) {
        digitalWrite(rst_pin, LOW);
        delay(DRF_ZIGBEE_RESET_DELAY);
        digitalWrite(rst_pin, HIGH);
        return;
    }
    
    const uint8_t preamble[] = {0xfc, 0x0, 0x91, 0x87, 0x6a, 0x35};
    memcpy(obuf, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += obuf[i];
    }
    obuf[6] = (uint8_t)(chksum & 0xff);
    port->write(obuf, 7);
    
    delay(DRF_ZIGBEE_RESET_DELAY);
    
    while (port->read() != -1) yield();
    read_state = DRF_STATE_READ_HEADER;
}

// Sets the pan_id and restarts the module
void DRF_Zigbee::set_pan_id(uint16_t _pan_id) {
    const uint8_t preamble[] = {0xfc, 0x02, 0x91, 0x01};
    memcpy(obuf, preamble, 4);
    obuf[4] = (uint8_t)(_pan_id >> 8);
    obuf[5] = (uint8_t)(_pan_id & 0xff);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += obuf[i];
    }
    obuf[6] = (uint8_t)(chksum & 0xff);
    
    while (port->read() != -1) yield();
    port->write(obuf, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    if (port->readBytes(obuf, 2) == 2) {
        pan_id = (((uint16_t)obuf[0] << 8) | obuf[1]);
    }
    
    reset();
}

// Gets module's pan id
uint16_t DRF_Zigbee::get_pan_id(void) {    
    const uint8_t preamble[] = {0xfc, 0x0, 0x91, 0x3, 0xa3, 0xb3};
    memcpy(obuf, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += obuf[i];
    }
    obuf[6] = (uint8_t)(chksum & 0xff);
    
    while (port->read() != -1) yield();
    port->write(obuf, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    if (port->readBytes(obuf, 2) == 2) {
        pan_id = (((uint16_t)obuf[0] << 8) | obuf[1]);
        return pan_id;
    }
    return 0;
}

// Sets module's baud rate and restarts it
void DRF_Zigbee::set_baud_rate(drf_baud_e baud) {
    const uint8_t preamble[]  = {0xfc, 0x1, 0x91, 0x6, 0x00, 0xf6};
    memcpy(obuf, preamble, 6);
    obuf[4] = (uint8_t)baud;

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += obuf[i];
    }
    obuf[6] = (uint8_t)(chksum & 0xff);
    port->write(obuf, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    port->readBytes(obuf, 6);
    reset();
}

// Gets module's short address
uint16_t DRF_Zigbee::get_self_address(void) {    
    const uint8_t preamble[]  = {0xfc, 0x00, 0x91, 0x04, 0xC4, 0xD4};
    memcpy(obuf, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += obuf[i];
    }
    obuf[6] = (uint8_t)(chksum & 0xff);
    
    while (port->read() != -1) yield();
    port->write(obuf, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    if (port->readBytes(obuf, 2) == 2) {
        self_addr = (((uint16_t)obuf[0] << 8) | obuf[1]);
        return self_addr;
    }

    return 0;
}
