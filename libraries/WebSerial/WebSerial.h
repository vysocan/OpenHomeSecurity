/*
 *
 *
 *
*/

#ifndef WebSerial_h
#define WebSerial_h
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#ifndef WEB_SERIAL_DEBUGGING
#define WEB_SERIAL_DEBUGGING 1
#endif

// Define buffer sizes
#define WEB_BUFFER_SIZE 1024

struct web_ring_buffer {
    uint8_t buffer[WEB_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
};

class WebSerial : public Print {
  private:
    volatile uint16_t _tail;

  public:

    WebSerial(web_ring_buffer *);
    void begin();
    void resetRead();
    uint16_t isAvailable();
    uint8_t read(void);
    size_t write(uint8_t);
    using Print::write;   
    void printP(const unsigned char *str);
};

extern WebSerial WS;

#endif
 