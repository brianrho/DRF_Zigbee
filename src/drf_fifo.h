#ifndef DRF_FIFO_H
#define DRF_FIFO_H

#include <stdint.h>
    
#define DRF_PKT_SZ      32          // max packet size in bytes
   
typedef struct {
    uint8_t bytes[DRF_PKT_SZ];
    uint8_t len;
    uint16_t dest;
} drf_packet_t;

class DRFFifo {
    public:
        DRFFifo(drf_packet_t * buf, uint16_t sz);
        uint16_t enqueue(const drf_packet_t * buf);
        uint16_t dequeue(drf_packet_t * buf);
        uint16_t available(void);
        bool full(void);
    
    private:
        uint16_t head;
        uint16_t tail;
        uint16_t count;
        drf_packet_t * bufptr;
        uint16_t buffer_sz;
};

#endif
