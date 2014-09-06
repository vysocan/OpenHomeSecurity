/**************************************************************************//**
 * EEPROM 24C512 library for NilRTOS 
 * vysocan 2014
 * based on:
 * \brief EEPROM 24C512 library for Arduino
 * \author Copyright (C) 2012  Julien Le Sech - www.idreammicro.com
 * \version 1.0
 * \date 20120203
 *
 ******************************************************************************/

#ifndef Eeprom24C512_h
#define Eeprom24C512_h

#include <Arduino.h>
#include <NilRTOS.h>

class Eeprom24C512{
    public:
        Eeprom24C512(byte deviceAddress, byte m_devicePageSize);

        void  initialize();
        void   writeByte(word address, char data);
        void  writeBytes(word address, word length, char* p_data);
        char    readByte(word address);
        void   readBytes(word address, word length, char* p_buffer);
        
    private:
        byte m_deviceAddress;
        byte m_devicePageSize;

        void   writePage(word address, byte length, char* p_data);
        void writeBuffer(word address, byte length, char* p_data);
        void  readBuffer(word address, byte length, char* p_data);
};

#endif // Eeprom24C512_h

