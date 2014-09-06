/**
 *       
 */
#include <stdlib.h>

#include "WebSerial.h"

// both buffers are static global vars in this file. interrupt handlers
// need to access them
static web_ring_buffer web_buffer;

/**
 * Initialize the USART module with the BAUD rate predefined
 * in HardwareSerial.h
 *
 * This may implement passing addresses of registers, like the HardwareSerial
 * class from the Arduino library, but that's not necessary at this point.
 *
 */
WebSerial::WebSerial(web_ring_buffer *web_buffer_ptr) {
 
}

/**
 * enable receiver, transmitter ...
 *    
 */
void WebSerial::begin() {
  web_buffer.tail = 0;
  web_buffer.head = 0;
  _tail = 0;
}
/**
 * 
 *    
 */
void WebSerial::resetRead() {
  web_buffer.tail = _tail;
}
/**
 * 
 *    
 */
uint16_t WebSerial::isAvailable() {
  if (web_buffer.head == web_buffer.tail) return 0;
  else return 1;
}

/**
 * Read procedure
 */
uint8_t WebSerial::read(void) {
  uint8_t datal  = web_buffer.buffer[web_buffer.tail];
  web_buffer.tail = (web_buffer.tail + 1) % WEB_BUFFER_SIZE;
  return datal;
} 


size_t WebSerial::write(uint8_t data) {
  // Save data byte at end of buffer
  web_buffer.buffer[web_buffer.head] = data;
  // Increment the head
  web_buffer.head = (web_buffer.head+1) % WEB_BUFFER_SIZE;

  if ((web_buffer.head) == web_buffer.tail) { // full, we are loosing data
    web_buffer.tail = (web_buffer.tail + 1) % WEB_BUFFER_SIZE;
  }

  if ((web_buffer.head) == _tail) { // full, we are loosing data
    _tail = (_tail + 1) % WEB_BUFFER_SIZE;
  }

  return 1;
}
void WebSerial::printP(const unsigned char *str){
  while (uint8_t value = pgm_read_byte(str++)){
    write(value);
  }
}

WebSerial WS(&web_buffer);