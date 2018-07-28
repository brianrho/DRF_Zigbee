/* An example for two pre-configured modules:
 * One acting as a coordinator receiver
 * Other acting as a router transmitter
 * Anything sent to the Serial port of the router MCU
 * is transmitted to the coordinator and printed to its serial port
 */ 
#ifdef ARDUINO_ARCH_ESP32
  #include <HardwareSerial.h>
#else
  #include <SoftwareSerial.h>
#endif

#include "drf_zigbee.h"

#define RECV_BUF_SZ         100

#define Z_COORDINATOR       0
#define Z_ROUTER            1

// change the module's role here
#define ROLE                Z_COORDINATOR
//#define ROLE              Z_ROUTER

#ifdef ARDUINO_ARCH_ESP32
  HardwareSerial softy(1);
#else
  SoftwareSerial softy(4, 5);
#endif

DRF_Zigbee bee;

char recvbuf[RECV_BUF_SZ];

void setup(void) {
    Serial.begin(9600);
    Serial.setTimeout(100);
    #ifdef ARDUINO_ARCH_ESP32
      softy.begin(38400, SERIAL_8N1, 16, 17);
    #else
      softy.begin(38400);
    #endif
    
    if (!bee.begin(&softy)) {
        Serial.println("Zigbee init failed.");
        while (1) yield();
    }
    
    Serial.println("Zigbee init success.");
    Serial.print("Short address: "); Serial.println(bee.get_self_address(), HEX);
    Serial.print("PAN ID: 0x"); Serial.println(bee.get_pan_id(), HEX);

    if (ROLE == 0) Serial.println("Role: Coordinator Receiver");
    else if (ROLE == 1) Serial.println("Role: Router Transmitter");
}

int count = 0;

void loop(void) {
    
#if (ROLE == Z_COORDINATOR)
    uint16_t addr;
    while (bee.available()) {
      count = bee.read_packet((uint8_t *)recvbuf, RECV_BUF_SZ, &addr);
      if (count > 0) {
        Serial.print("From 0x"); Serial.print(addr, HEX); Serial.print(": ");
        Serial.write((uint8_t *)recvbuf, count);
        Serial.println();
      }
      yield();
    }
#elif (ROLE == Z_ROUTER)
    while (Serial.available()) {
        count = Serial.readBytes(recvbuf, RECV_BUF_SZ);
        Serial.print("To coordinator: "); Serial.write((uint8_t *)recvbuf, count); Serial.println();
        #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
          bee.buffered_write((uint8_t *)recvbuf, count, DRF_ZIGBEE_COORDINATOR_ADDR);
        #else
          bee.write((uint8_t *)recvbuf, count, DRF_ZIGBEE_COORDINATOR_ADDR);
        #endif
    }

    #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
      bee.flush();
    #endif   
#endif

}