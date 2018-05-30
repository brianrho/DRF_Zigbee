#include <SoftwareSerial.h>
#include "drf_zigbee.h"

#define RECV_BUF_SZ         100

#define Z_COORDINATOR       0
#define Z_ROUTER            1

#define ROLE                Z_COORDINATOR
//#define ROLE              Z_ROUTER

SoftwareSerial softy(2, 3);

DRF_Zigbee bee;

char recvbuf[RECV_BUF_SZ];

void setup(void) {
    Serial.begin(9600);
    softy.begin(9600);
    
    if (!bee.begin(&softy)) {
        Serial.println("Zigbee init failed.");
        while (1) yield();
    }
    
    Serial.print("Zigbee init success.");
    Serial.print("Short address: "); Serial.println(bee.get_self_address(), HEX);
}

int count = 0;

void loop(void) {
    
#if (ROLE == Z_COORDINATOR)
    while (bee.available()) {
        count = bee.read((uint8_t *)recvbuf, RECV_BUF_SZ);
        Serial.write(recvbuf, count);
    }
#elif (ROLE == Z_ROUTER)
    while (Serial.available()) {
        count = Serial.readBytes(recvbuf, RECV_BUF_SZ);
        bee.write((uint8_t *)recvbuf, count, DRF_ZIGBEE_COORDINATOR_ADDR);
    }
#endif
        
}
