#include <string.h>
#include "drf_zigbee.h"

typedef enum {
	DRF_ZIGBEE_STATE_READ_HEADER,
	DRF_ZIGBEE_STATE_READ_CONTENTS,
} packeteer_state_e;

// Because there's no memchr in ESP8266 core
static uint8_t * memchr2(void * p, uint8_t ch, size_t size)
{
    uint8_t * ptr = (uint8_t *)p;
    for (size_t i = 0; i < size; i++)
        if (ptr[i] == ch)
            return ptr + i;
    return NULL;
}

#if !defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ARCH_ESP8266)
DRF_Zigbee::DRF_Zigbee(uint8_t _rst_pin) : rst_pin(_rst_pin), pkt_head(buffer), wptr(buffer)
{
    if (rst_pin) {
        pinMode(rst_pin, OUTPUT);
        digitalWrite(rst_pin, HIGH);
    }
}

#else
DRF_Zigbee::DRF_Zigbee(uint8_t _rst_pin) : out_fifo(_out_buf, DRF_ZIGBEE_MAX_BUFFERED_PKTS), 
                                           rst_pin(_rst_pin), pkt_head(buffer), wptr(buffer),
                                           last_send(0)
{
    if (rst_pin) {
        pinMode(rst_pin, OUTPUT);
        digitalWrite(rst_pin, HIGH);
    }
}
#endif

// Timeout should be at least 500ms; can probably be shortened with more tests
bool DRF_Zigbee::begin(Stream * ss, uint16_t _pan_id, uint16_t timeout) {
    port = ss;
    
    if (timeout)
        port->setTimeout(timeout);
    else
        port->setTimeout(DRF_ZIGBEE_DEFAULT_TIMEOUT);
    
    reset();
    
    uint8_t test_cmd[] = {0xFC, 0x00, 0x91, 0x07, 0x97, 0xA7, 0xD2};
    uint8_t expected[] = {0x1, 0x2, 0x3, 0x4, 0x5};
    
    while (port->read() != -1) yield();            // clear buffer
    
    memset(cmdbuf, 0, DRF_ZIGBEE_BUF_SZ);
    memset(buffer, 0, DRF_ZIGBEE_BUF_SZ);
    
    port->write(test_cmd, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    port->readBytes(cmdbuf, 5);
    
    if (memcmp(cmdbuf, expected, 5) != 0) {
        return false;
    }
    
    while (port->read() != -1) yield();
    
    pan_id = get_pan_id();
    
    if (_pan_id != 0 && pan_id != _pan_id) {
        set_pan_id(_pan_id);
        while (get_pan_id() != _pan_id) yield();
    }

    return true;
}

uint16_t DRF_Zigbee::write(uint8_t c, uint16_t to_addr) {
    cmdbuf[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;
    cmdbuf[1] = 0x1;                           // packet length
    cmdbuf[2] = (uint8_t)to_addr >> 8; buffer[3] = (uint8_t)(to_addr & 0xff);       // coordinator address
    cmdbuf[4] = c;
    port->write(buffer, 5);
    
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    return 1;
}

uint16_t DRF_Zigbee::write(const uint8_t * data, uint16_t len, uint16_t to_addr) {
    uint8_t to_write;
    uint16_t written = 0;

    cmdbuf[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;

    while (len) {
        to_write = (len > DRF_ZIGBEE_MAX_PKT_SZ) ? DRF_ZIGBEE_MAX_PKT_SZ : len;

        cmdbuf[1] = to_write;
        cmdbuf[2] = (uint8_t)(to_addr >> 8); cmdbuf[3] = (uint8_t)(to_addr & 0xff);
        memcpy(&cmdbuf[4], data, to_write);
        port->write(cmdbuf, to_write + 4);

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
    cmdbuf[0] = DRF_ZIGBEE_DATA_TRANSFER_CMD;

    if (!len)               // no zero-length writes allowed
        return 0;
    
    to_write = (len > DRF_ZIGBEE_MAX_PKT_SZ) ? DRF_ZIGBEE_MAX_PKT_SZ : len;

    cmdbuf[1] = to_write;
    cmdbuf[2] = (uint8_t)(to_addr >> 8); cmdbuf[3] = (uint8_t)(to_addr & 0xff);
    memcpy(&cmdbuf[4], data, to_write);
    port->write(cmdbuf, to_write + 4);

    return to_write;
}

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
/* Just like write() except there's no delay,
 * Packets get buffered and are finally sent when flush() is called
 * At most 32 packets of DRF_ZIGBEE_MAX_PKT_SZ bytes are buffered (1k bytes)
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

/* Should be called regularly (say, every DRF_ZIGBEE_INTER_PACKET_INTERVAL ms) to dispatch packets,
 * Not tested yet
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

/* Reads a complete packet and returns the packet length and source address.
 * To read anything longer than 32 bytes in a single call, it relies on higher
 * application code to call first check available(), and then call 
 * read_packet() repeatedly until it returns 0 which means there's nothing valid left to read
 * or the user's buffer is not sufficient.
 * TODO: use a state machine instead
 */
int16_t DRF_Zigbee::read_packet(uint8_t * data, uint16_t len, uint16_t * from_addr) {
    uint16_t rem_buf_sz;
    uint16_t to_read, avail, rlen = 0;
    bool need_more_data = false;
    
    while (true) {
        avail = port->available();
        /*if (avail == 0)
            return 0;*/
        
        if (pkt_head != buffer) {               // drag packet head to start of buffer, to make space
            memmove(buffer, pkt_head, wptr - pkt_head);
            wptr = buffer + (wptr - pkt_head);
            pkt_head = buffer;
        }
        
        rem_buf_sz = DRF_ZIGBEE_BUF_SZ - (wptr - buffer);
        
        to_read = (avail > rem_buf_sz) ? rem_buf_sz : avail;
        rlen = port->readBytes(wptr, to_read);
        wptr += rlen;
        
        yield();

        if ((rlen == 0 && need_more_data) || wptr == pkt_head)          // if we read nothing or theres nothing else left to parse
            return DRF_ZIGBEE_READ_FAIL;
            
        need_more_data = false;

        while (pkt_head < wptr) {
            uint16_t remn = wptr - pkt_head;
            uint8_t * new_pkt = (uint8_t *)memchr2(pkt_head, DRF_ZIGBEE_DATA_TRANSFER_CMD, remn);
            if (new_pkt == NULL) {
                pkt_head = wptr;                    // consume all the garbage and advance to the write ptr wherever it is
                break;
            }
            
            pkt_head = new_pkt;
            if (wptr - pkt_head < 4) {               // message header section is not all present?
                need_more_data = true;
                break;
            }
            
            if (pkt_head[2] != (self_addr >> 8) || pkt_head[3] != (self_addr & 0xff)) {     // message is not for us?
                pkt_head++;                          // advance past current pkt header
                continue;
            }
            uint8_t plen = pkt_head[1];
            if (plen > DRF_ZIGBEE_MAX_PKT_SZ) {          // make sure packet size is < 32
                pkt_head++;                  // advance past the current pkt header(?)
                continue;
            }
            
            if (plen > len) {                           // if there isnt enough space in the buffer
                return 0;
            }
            
            if (plen + 4 + 2 > wptr - pkt_head) {        // if we havent read the entire packet, try to read more
                need_more_data = true;
                break;
            }

            if (from_addr != NULL)
                *from_addr = ((uint16_t)pkt_head[plen + 4] << 8) | pkt_head[plen + 4 + 1];

            memcpy(data, &pkt_head[4], plen);
            pkt_head += 4 + plen + 2;                   // advance past header, len, dest addr, content, src addr
            if (plen == 0)
                continue;                               // if we somehow read an empty packet, move on
            return plen;
        }
    }
}

int16_t DRF_Zigbee::read_packet(uint8_t * data, uint16_t len, uint16_t * from_addr) {
    static uint16_t src_addr = 0;
    static uint16_t dest_addr = 0;
    static uint16_t to_read = 0;
    
    while (true) {
        switch (read_state) {
            case DRF_ZIGBEE_STATE_READ_HEADER:
                if (port->available() < 4)
                    return DRF_ZIGBEE_READ_FAIL;
                if (port->read() != DRF_ZIGBEE_DATA_TRANSFER_CMD)
                    return DRF_ZIGBEE_READ_FAIL;
                
                to_read = port->read();
                if (to_read > DRF_ZIGBEE_MAX_PKT_SZ)
                    return DRF_ZIGBEE_READ_FAIL;
                
                dest_addr = 0;
                port->readBytes(ibuf, 2);
                dest_addr = ((uint16_t)ibuf[0] << 8) | ibuf[1];
                read_state = DRF_ZIGBEE_STATE_READ_CONTENTS;
                break;
                
            case DRF_ZIGBEE_STATE_READ_CONTENTS:
                if (port->available() < to_read + 2)            // +2 for src address
                    return DRF_ZIGBEE_READ_FAIL;
                if (to_read > len)
                    return 0;
                port->readBytes(data, to_read);
                port->readBytes(ibuf, 2);
                src_addr = ((uint16_t)ibuf[0] << 8) | ibuf[1];
                *from_addr = src_addr;
                
                read_state = DRF_ZIGBEE_STATE_READ_HEADER;
                return to_read;
            default:
                break;
        }
    }
}

/* Checks if there's any packet available and returns
 * its length if so. Else, it returns the number of unread
 * Stream bytes. A return value > 0 means a read_packet()
 * can be attempted, though there's no guarantee that it will succeed. 
 * However if this function ever returns 0, it's guaranteed that
 * there's nothing useful to read
 */
uint16_t DRF_Zigbee::available(void) {
    return port->available();
}

/* Generic available(): Only checks if there are unread bytes from Stream.
 * Note that even if there are still complete packets left in the library's buffer,
 * available() can still return false because it only checks the Stream port.
 */
uint16_t DRF_Zigbee::unread_available(void) {
    return port->available();
}

/* Checks if there's a packet available in the library buffer
 * If it returns a value > 0, this is the length of the content of the unread packet.
 * Such a result guarantees that a read_packet() will succeed,
 * provided a sufficient buffer is passed to it of course.
 * Untested.
 */
 
uint16_t DRF_Zigbee::pkt_available(void)  {
    while (pkt_head < wptr) {
        uint16_t remn = wptr - pkt_head;
        
        uint8_t * new_pkt = (uint8_t *)memchr2(pkt_head, DRF_ZIGBEE_DATA_TRANSFER_CMD, remn);
        pkt_head = new_pkt;         //dvdvnfdvjvDDDDDDdvvvvvvvvvvvvvvvvvvvvvvvvv
        if (new_pkt == NULL) {              // if we didnt find the header
            pkt_head = wptr;                // advance pkt head to meet write ptr
            return 0;
        }
        if (wptr - new_pkt < 4)             // if we need more data
            return 0;
        if (new_pkt[2] != (self_addr >> 8) || new_pkt[3] != (self_addr & 0xff)) {       // if msg is not for us
            pkt_head = new_pkt + 1;
            continue;
        }
        uint8_t plen = new_pkt[1];
        if (plen > DRF_ZIGBEE_MAX_PKT_SZ) {         // if its bigger than permissible packet size
            pkt_head = new_pkt + 1;
            continue;
        }
        if (plen + 4 + 2 > wptr - new_pkt)          // if we need more data to complete the packet
            return 0;
        if (plen == 0) {
            pkt_head = 4 + new_pkt + plen + 2;      // no zero-length packets allowed
            continue;
        }
        
        return plen;
    }
    return 0;
}

uint16_t DRF_Zigbee::parse_buffer(uint8_t * pkt_loc) {
    while (pkt_head < wptr) {
        uint16_t remn = wptr - pkt_head;
        
        uint8_t * new_pkt = (uint8_t *)memchr2(pkt_head, DRF_ZIGBEE_DATA_TRANSFER_CMD, remn);
        pkt_head = new_pkt;
        if (new_pkt == NULL) {              // if we didnt find the header
            pkt_head = wptr;                // advance pkt head to meet write ptr
            return 0;
        }
        if (wptr - new_pkt < 4)             // if we need more data
            return 0;
        if (new_pkt[2] != (self_addr >> 8) || new_pkt[3] != (self_addr & 0xff)) {       // if msg is not for us
            pkt_head = new_pkt + 1;
            continue;
        }
        uint8_t plen = new_pkt[1];
        if (plen > DRF_ZIGBEE_MAX_PKT_SZ) {         // if its bigger than permissible packet size
            pkt_head = new_pkt + 1;
            continue;
        }
        if (plen + 4 + 2 > wptr - new_pkt)          // if we need more data to complete the packet
            return 0;
        if (plen == 0) {
            pkt_head = 4 + new_pkt + plen + 2;      // no zero-length packets allowed
            continue;
        }
        return plen;
    }
    return 0;
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
    memcpy(cmdbuf, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += cmdbuf[i];
    }
    cmdbuf[6] = (uint8_t)(chksum & 0xff);
    port->write(cmdbuf, 7);
    
    delay(DRF_ZIGBEE_RESET_DELAY);
}

// Sets the pan_id and restarts the module
void DRF_Zigbee::set_pan_id(uint16_t _pan_id) {
    const uint8_t preamble[] = {0xfc, 0x02, 0x91, 0x01};
    memcpy(cmdbuf, preamble, 4);
    cmdbuf[4] = (uint8_t)(_pan_id >> 8);
    cmdbuf[5] = (uint8_t)(_pan_id & 0xff);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += cmdbuf[i];
    }
    cmdbuf[6] = (uint8_t)(chksum & 0xff);
    port->write(cmdbuf, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    if (port->readBytes(cmdbuf, 2) == 2) {
        pan_id = (((uint16_t)cmdbuf[0] << 8) | cmdbuf[1]);
    }
    
    reset();
}

// Gets module's pan id
uint16_t DRF_Zigbee::get_pan_id(void) {
    while (port->read() != -1);
    
    const uint8_t preamble[] = {0xfc, 0x0, 0x91, 0x3, 0xa3, 0xb3};
    memcpy(cmdbuf, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += cmdbuf[i];
    }
    cmdbuf[6] = (uint8_t)(chksum & 0xff);
    port->write(cmdbuf, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    if (port->readBytes(cmdbuf, 2) == 2) {
        pan_id = (((uint16_t)cmdbuf[0] << 8) | cmdbuf[1]);
        return pan_id;
    }
    return 0;
}

// Sets module's baud rate and restarts it
void DRF_Zigbee::set_baud_rate(DRF_BAUD_t baud) {
    const uint8_t preamble[]  = {0xfc, 0x1, 0x91, 0x6, 0x00, 0xf6};
    memcpy(cmdbuf, preamble, 6);
    cmdbuf[4] = (uint8_t)baud;

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += cmdbuf[i];
    }
    cmdbuf[6] = (uint8_t)(chksum & 0xff);
    port->write(cmdbuf, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    port->readBytes(cmdbuf, 6);
    reset();
}

// Gets module's short address
uint16_t DRF_Zigbee::get_self_address(void) {
    while (port->read() != -1);
    
    const uint8_t preamble[]  = {0xfc, 0x00, 0x91, 0x04, 0xC4, 0xD4};
    memcpy(cmdbuf, preamble, 6);

    uint16_t chksum = 0;
    for (int i = 0; i < 6; i++) {
        chksum += cmdbuf[i];
    }
    cmdbuf[6] = (uint8_t)(chksum & 0xff);
    port->write(cmdbuf, 7);
    delay(DRF_ZIGBEE_INTER_PACKET_INTERVAL);
    
    if (port->readBytes(cmdbuf, 2) == 2) {
        self_addr = (((uint16_t)cmdbuf[0] << 8) | cmdbuf[1]);
        return self_addr;
    }

    return 0;
}
