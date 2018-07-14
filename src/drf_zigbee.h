#ifndef DRF_ZIGBEE_H_
#define DRF_ZIGBEE_H_

#if defined(ARDUINO)
    #include <Arduino.h>
#else
    #include "Stream.h"
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    #include "drf_fifo.h"
    #define DRF_ZIGBEE_MAX_PKT_SZ           (DRF_PKT_SZ)
    #define DRF_ZIGBEE_MAX_BUFFERED_PKTS    32
#else
    #define DRF_ZIGBEE_MAX_PKT_SZ           32
#endif

#define DRF_ZIGBEE_PANID_LEN                2
#define DRF_ZIGBEE_SHORT_ADDR_LEN           2

#define DRF_ZIGBEE_DEFAULT_TIMEOUT          500
#define DRF_ZIGBEE_INTER_PACKET_INTERVAL    50
#define DRF_ZIGBEE_RESET_DELAY              3000

#define DRF_ZIGBEE_BUF_SZ                   (DRF_ZIGBEE_MAX_PKT_SZ + 4 + 2)
#define DRF_ZIGBEE_DATA_TRANSFER_CMD        0xFD

#define DRF_ZIGBEE_JOIN_ANY                 0xFFFF
#define DRF_ZIGBEE_ANY_ADDR                 0xFFFE
#define DRF_ZIGBEE_NO_ADDR                  0xFFFE
#define DRF_ZIGBEE_COORDINATOR_ADDR         0x0000

typedef enum {
    DRF_BAUD_9600 = 1,
    DRF_BAUD_19200,
    DRF_BAUD_38400,
    DRF_BAUD_57600,
    DRF_BAUD_115200
} DRF_BAUD_t;

class DRF_Zigbee {
    public:
        DRF_Zigbee(uint8_t _rst_pin = 0);
        bool begin(Stream * ss, uint16_t _pan_id = 0, uint16_t timeout = 0);
        uint16_t write(uint8_t c, uint16_t to_addr = DRF_ZIGBEE_COORDINATOR_ADDR);
        uint16_t write(const uint8_t * data, uint16_t len, uint16_t to_addr = DRF_ZIGBEE_COORDINATOR_ADDR);
        uint16_t write_packet(const uint8_t * data, uint16_t len, uint16_t to_addr);
        uint16_t read_packet(uint8_t * data, uint16_t len, uint16_t * from_addr);
        uint16_t available(void);
        #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
            uint16_t buffered_write(const uint8_t * data, uint16_t len, uint16_t to_addr);
            void flush(void);
        #endif
        void set_pan_id(uint16_t _pan_id);
        uint16_t get_pan_id(void);
        void set_baud_rate(DRF_BAUD_t baud);
        uint16_t get_self_address(void);
        void reset(void);

    private:
        uint8_t buffer[DRF_ZIGBEE_BUF_SZ];
        uint8_t cmdbuf[DRF_ZIGBEE_BUF_SZ];
        
        #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
            DRFFifo out_fifo;
            drf_packet_t _out_buf[DRF_ZIGBEE_MAX_BUFFERED_PKTS];
            drf_packet_t temp_pkt;
            uint8_t tempbuf[DRF_ZIGBEE_BUF_SZ];
            uint32_t last_send;
        #endif
        
        uint16_t pan_id;
        uint16_t self_addr;
        
        uint8_t rst_pin;

        uint8_t * pkt_head;
        uint8_t * wptr;

        Stream * port;
};

#endif
