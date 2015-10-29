/*
 *
 *
 *
*/

#ifndef NilGSM_h
#define NilGSM_h
#include <Arduino.h>

#include <NilRTOS.h>
#include <WebSerial.h>

#define AT_WAIT            50          // how many delay loops wait for modem response
#define AT_DELAY           100         // delay in millis for modem response 
#define AT_OK              "OK"        // 
#define AT_is_alive        "AT"        // --'CR'OK'CR''CR'
#define AT_model           "AT+CGMM"   // --'CR'TC35'CR''CR'OK'CR'
#define AT_registered      "AT+CREG?"  // --'CR'+CREG: 0,1'CR''CR'OK'CR'
#define AT_signal_strength "AT+CSQ"    // --'CR'+CSQ: 21,99'CR''CR'OK'CR'
#define AT_set_sms_to_text "AT+CMGF=1" // --'CR'OK'CR'
#define AT_send_sms        "AT+CMGS="  // --"+6421494481" followed by message then CTRL-Z then enter
#define AT_send_sms_reply  "+CMGS:"
#define AT_CLIP_ON         "AT+CLIP=1" // Set CLI On
#define AT_CLIP_OFF        "AT+CLIP=1" // Set CLI Off

extern "C" void USART_TX_vect(void) __attribute__ ((signal));
extern "C" void USART_RX_vect(void) __attribute__ ((signal));
extern "C" void USART_UDRE_vect(void) __attribute__ ((signal));

#include <inttypes.h>

// Define buffer sizes
#define GSM_USART_RX_BUFFER_SIZE 64
#define GSM_USART_TX_BUFFER_SIZE 64

struct gsm_rx_ring_buffer {
    uint8_t buffer[GSM_USART_RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};

struct gsm_tx_ring_buffer {
    uint8_t buffer[GSM_USART_TX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};


#ifdef __cplusplus
extern "C" {
#endif
 bool nilWaitGSMNewMsg();
#ifdef __cplusplus
}
#endif

class NilGSM : public Print {
  private:
    

  public:
    //uint8_t _msg;

    NilGSM(gsm_rx_ring_buffer *, gsm_tx_ring_buffer *);
    void    begin(unsigned long);
    void    flushRX(void);
    uint8_t read(void);
    uint8_t read(uint8_t *msg);
    size_t  write(uint8_t);
    using   Print::write;
    void    printP(const unsigned char *str);
    
    uint8_t isMsg(void);

    uint8_t ATWaitMsg(void);
    int8_t  ATsendCmd(char *what);
    int8_t  ATsendCmdWR(char *what, uint8_t *response);
    int8_t  ATsendCmdWR(char *what, uint8_t *response, uint8_t index);
    int8_t  ATsendSMSBegin(char *number);
    int8_t  ATsendSMSEnd(char *what, uint8_t send);

    bool    nilWaitGSMNewMsg();

};

extern NilGSM GSM;

#endif
 