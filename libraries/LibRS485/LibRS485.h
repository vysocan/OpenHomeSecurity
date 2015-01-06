/**
 *
 * 
*/

#ifndef LibRS485_h
#define LibRS485_h

#include <inttypes.h>

#include "Print.h"

#include <util/delay_basic.h>

// Define message parameters
#define MSG_HEADER_SIZE 3        // 2 bytes and XOR
#define MSG_DATA_SIZE 64         // 64 bytes
#define MSG_CRC_SIZE 2           // 2 bytes data CRC16
#define LINE_READY_TIME_OUT 100  // max 255
// Define buffer sizes
#define USART_RX_BUFFER_SIZE 256 // must be 256
#define USART_TX_BUFFER_SIZE (MSG_HEADER_SIZE + MSG_DATA_SIZE + MSG_CRC_SIZE)
// Define the amount of tolerance upon which x2 will be enabled, in percent
#define BAUD_TOL 2

// RS485 header bits
#define FLAG_ACK 3
#define FLAG_NAK 2
#define FLAG_CMD 1
#define FLAG_DTA 0

struct rx_ring_buffer {
    uint8_t buffer[USART_RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};

struct tx_ring_buffer {
    uint8_t buffer[USART_TX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};

struct RS485_msg {
  unsigned char buffer[MSG_DATA_SIZE + MSG_CRC_SIZE]; // data + crc
  uint8_t address;
  uint8_t data_length;
  uint8_t ctrl;
};

class LibRS485 // : public Print
{
  private:
    void write(uint8_t);
    uint8_t read(void);
    int16_t _tick;    
    
  public:
    LibRS485(rx_ring_buffer *, tx_ring_buffer *);
    void begin(unsigned long, uint8_t);
    int8_t msg_read(RS485_msg *msg);
    int8_t msg_write(RS485_msg *msg);

};

extern LibRS485 RS485;

#endif
 