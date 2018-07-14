#include <string.h>
#include "drf_fifo.h"

//This initializes the FIFO structure with the given buffer and size
DRFFifo::DRFFifo(drf_packet_t * buf, uint16_t sz) : head(0), tail(0), count(0), bufptr(buf), buffer_sz(sz)
{
    
}

// Dequeues a single packet of at most DRF_PKT_SZ bytes
uint16_t DRFFifo::dequeue(drf_packet_t * pkt) {
    if (count == 0)
        return 0;
    pkt->len = bufptr[tail].len;
    pkt->dest = bufptr[tail].dest;
    memcpy(pkt->bytes, bufptr[tail].bytes, DRF_PKT_SZ);
    tail++;
    if (tail == buffer_sz)
        tail = 0;
    count--;
    return 1;
}

// Enqueues a single packet of at most DRF_PKT_SZ bytes
uint16_t DRFFifo::enqueue(const drf_packet_t * pkt) {
    if (count == buffer_sz)
        return 0;
    bufptr[head].len = pkt->len;
    bufptr[head].dest = pkt->dest;
    memcpy(bufptr[head].bytes, pkt->bytes, DRF_PKT_SZ);
    head++;
    if (head == buffer_sz)
        head = 0;
    count++;
    return 1;
}

// Returns number of queued packets
uint16_t DRFFifo::available(void) {
    return count;
}

bool DRFFifo::full(void) {
    return (count == buffer_sz);
}
