#ifndef DRF_ZIGBEE_H_
#define DRF_ZIGBEE_H_

#if defined(ARDUINO)
    #include <Arduino.h>
#else
    #include "Stream.h"
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_STM32)
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

#define DRF_ZIGBEE_BUF_SZ                   (32)
#define DRF_ZIGBEE_DATA_TRANSFER_CMD        0xFD

#define DRF_ZIGBEE_DISCOVER_PANID           0xFFFF
#define DRF_ZIGBEE_NO_PANID                 0xFFFE
#define DRF_ZIGBEE_BCAST_ADDR				0xFFFF
#define DRF_ZIGBEE_NO_ADDR                  0xFFFE
#define DRF_ZIGBEE_COORDINATOR_ADDR         0x0000

#define DRF_ZIGBEE_READ_INCOMPLETE          (-1)

typedef enum {
    DRF_BAUD_9600 = 1,
    DRF_BAUD_19200,
    DRF_BAUD_38400,
    DRF_BAUD_57600,
    DRF_BAUD_115200
} drf_baud_e;

typedef enum {
	DRF_STATE_READ_HEADER,
	DRF_STATE_READ_CONTENTS,
} drf_state_e;

class DRF_Zigbee {
    public:
        DRF_Zigbee(Stream * ss, uint8_t _rst_pin = 0);
        bool begin(uint16_t _pan_id = 0);
        uint16_t write(uint8_t c, uint16_t to_addr = DRF_ZIGBEE_COORDINATOR_ADDR);
        uint16_t write(const uint8_t * data, uint16_t len, uint16_t to_addr = DRF_ZIGBEE_COORDINATOR_ADDR);
        uint16_t write_packet(const uint8_t * data, uint16_t len, uint16_t to_addr = DRF_ZIGBEE_COORDINATOR_ADDR);
        int16_t read_packet(uint8_t * data, uint16_t len, uint16_t * from_addr = NULL, uint16_t * to_addr = NULL);
        uint16_t available(void);
        #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_STM32)
            uint16_t buffered_write(const uint8_t * data, uint16_t len, uint16_t to_addr = DRF_ZIGBEE_COORDINATOR_ADDR);
            uint16_t buffered_write_packet(const uint8_t * data, uint16_t len, uint16_t to_addr = DRF_ZIGBEE_COORDINATOR_ADDR);
            void flush(void);
        #endif
        void set_pan_id(uint16_t _pan_id);
        uint16_t get_pan_id(void);
        void set_baud_rate(drf_baud_e baud);
        uint16_t get_self_address(void);
        void reset(void);

    private:
        uint8_t ibuf[DRF_ZIGBEE_BUF_SZ];
        uint8_t obuf[DRF_ZIGBEE_BUF_SZ];
        
        #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_STM32)
            DRFFifo out_fifo;
            drf_packet_t _out_buf[DRF_ZIGBEE_MAX_BUFFERED_PKTS];
            drf_packet_t temp_pkt;
            uint8_t tempbuf[DRF_ZIGBEE_BUF_SZ];
            uint32_t last_send;
        #endif
        
        uint16_t pan_id;
        uint16_t self_addr;
        
        uint8_t rst_pin;

        Stream * port;
        drf_state_e read_state;
};

#endif
