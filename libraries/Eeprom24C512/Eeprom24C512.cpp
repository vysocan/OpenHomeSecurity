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
#include <Arduino.h>
#include <TwiMaster.h>

#include <Eeprom24C512.h>

/**************************************************************************//**
 * This size is equal to BUFFER_LENGTH defined in Wire library (32 bytes).
 ******************************************************************************/
#define EEPROM__RD_BUFFER_SIZE    BUFFER_LENGTH

/**************************************************************************//**
 * This size is equal to BUFFER_LENGTH - 2 bytes reserved for address.
 ******************************************************************************/
#define EEPROM__WR_BUFFER_SIZE    (BUFFER_LENGTH - 2)

Eeprom24C512::Eeprom24C512(byte deviceAddress, byte devicePageSize){
    m_deviceAddress = deviceAddress;
    m_devicePageSize = devicePageSize;
}

/**************************************************************************//**
 * If several devices are connected to TWI bus, this method mustn't be
 * called. TWI bus must be initialized out of this library using
 * Wire.begin() method.
 ******************************************************************************/
void Eeprom24C512::initialize(){
    Wire.begin();
}

void Eeprom24C512::writeByte(word address, char data){
    Wire.beginTransmission(m_deviceAddress);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    Wire.write(data);
    Wire.endTransmission();
}

void Eeprom24C512::writeBytes(word address, word length, char* p_data){
    // Write first page if not aligned.
    byte notAlignedLength = 0;
    byte pageOffset = address % m_devicePageSize;
    if (pageOffset > 0){
        notAlignedLength = m_devicePageSize - pageOffset;
        if (notAlignedLength > length ) {
            writePage(address, length, p_data);
            length = 0;
        }
        else {
	   	   length -= notAlignedLength;
	       writePage(address, notAlignedLength, p_data);
        }
    }

    if (length > 0){
        address += notAlignedLength;
        p_data += notAlignedLength;
        // Write complete and aligned pages.
        word pageCount = length / m_devicePageSize;
        for (word i = 0; i < pageCount; i++){
            writePage(address, m_devicePageSize, p_data);
            address += m_devicePageSize;
            p_data += m_devicePageSize;
            length -= m_devicePageSize;
        }
        if (length > 0){
    	    writePage(address, length, p_data);
        }
    }
}

char Eeprom24C512::readByte(word address){
    Wire.beginTransmission(m_deviceAddress);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    Wire.endTransmission();
    Wire.requestFrom(m_deviceAddress, (byte)1);
    byte data = 0;
    if (Wire.available()){
        data = Wire.read();
    }
    return data;
}

void Eeprom24C512::readBytes(word address, word length, char* p_data){
    word bufferCount = length / EEPROM__RD_BUFFER_SIZE;
    for (word i = 0; i < bufferCount; i++){
        word offset = i * EEPROM__RD_BUFFER_SIZE;
        readBuffer(address + offset, EEPROM__RD_BUFFER_SIZE, p_data + offset);
    	length -= EEPROM__RD_BUFFER_SIZE;
    }
    byte remainingBytes = length % EEPROM__RD_BUFFER_SIZE;
    if (remainingBytes > 0){
      word offset = length - remainingBytes;
      readBuffer(address + offset, remainingBytes, p_data + offset);
    }

}

/******************************************************************************
 * Private method definitions.
 ******************************************************************************/
void Eeprom24C512::writePage(word address, byte length, char* p_data){
    // Write complete buffers.
    byte bufferCount = length / EEPROM__WR_BUFFER_SIZE;
    for (byte i = 0; i < bufferCount; i++){
        byte offset = i * EEPROM__WR_BUFFER_SIZE;
        writeBuffer(address + offset, EEPROM__WR_BUFFER_SIZE, p_data + offset);
    }
    // Write remaining bytes.
    byte remainingBytes = length % EEPROM__WR_BUFFER_SIZE;
    byte offset = length - remainingBytes;
    writeBuffer(address + offset, remainingBytes, p_data + offset);
}

void Eeprom24C512::writeBuffer(word address, byte length, char* p_data){
    Wire.beginTransmission(m_deviceAddress);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    for (byte i = 0; i < length; i++){
        Wire.write(p_data[i]);
    }
    Wire.endTransmission();
    
    // Write cycle time (tWR). See EEPROM memory datasheet for more details.
    //delay(10);
    if (length == EEPROM__WR_BUFFER_SIZE){ 
        // * test ? * if buffer is full probably next transfer will follow
        nilThdSleepMilliseconds(10);   
    }
}

void Eeprom24C512::readBuffer(word address, byte length, char* p_data){
    Wire.beginTransmission(m_deviceAddress);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    Wire.endTransmission();
    Wire.requestFrom(m_deviceAddress, length);
    for (byte i = 0; i < length; i++){
        if (Wire.available()){
            p_data[i] = Wire.read();
        }
    }
}