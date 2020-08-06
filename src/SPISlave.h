/*
 * SPI Slave library for AMBD Arduino
 * Copyright (c) 2020 Seeed Studio. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __SPISLAVE_H__
#define __SPISLAVE_H__

extern "C" {
#include "ameba_soc.h"
#include "gpio_api.h"
}
#include "Stream.h"
#include "variant.h"
#include "RingBuffer.h"

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

class SPISlave_ : public Stream
{
  public:
    SPISlave_(USI_TypeDef* udev, int pinMOSI, int pinMISO, int pinSCLK, int pinCS, int pinIRQ2Mst);
    void begin();
    void end();

    void setBitOrder(BitOrder order) { (void)order; }
    void setDataMode(uint8_t dataMode);
    void setClockDivider(uint8_t uc_div) { (void)uc_div;}

    size_t write(uint8_t data);
    size_t write(const uint8_t * data, size_t quantity);

    size_t readBytes( char *buffer, size_t length); // read chars from stream into buffer
    size_t readBytes( uint8_t *buffer, size_t length) { return readBytes((char *)buffer, length); }

    virtual int available(void);
    int availableForStore(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(int));

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    void onService(void);

  private:
    USI_TypeDef* udev;
    USI_SSI_InitTypeDef usIs;
    uint8_t pinMOSI;
    uint8_t pinMISO;
    uint8_t pinSCLK;
    uint8_t pinCS;
    int     pinIRQ;

    int irqUsi; /* USI irq for MCU interrupt */

    gpio_t irqOut; /* gpio irq to MASTER device */
    gpio_t syncOut;/* USI SPI Bug: transfer in FIFO access period,
                                   May destroy FIFO content */

    // RX Buffer
    RingBufferN<SERIAL_BUFFER_SIZE> rxBuffer;

    // TX buffer
    RingBufferN<SERIAL_BUFFER_SIZE> txBuffer;

    // Callback user functions
    void (*onRequestCallback)(int);
    void (*onReceiveCallback)(int);

    volatile int bytesRD, bytesWR;
    void _recv(void);
    void _regRD(void);
    void _regWR(void);

    // Directly write to SPI Controller TX FIFO
    size_t _write(const uint8_t * data, size_t quantity);
    size_t _writeFromTxBuf(int max_bytes);

    // small buffer for register access
    uint8_t _rxBuf[USI_SPI_RX_FIFO_DEPTH];
    volatile int _rxIdx;
    volatile int _rxCnt;
    volatile uint8_t regAddr;
    // Interrupt ENable
    volatile uint16_t rvIEN;
    // Interrupt STATUS
    volatile uint16_t rvIRQ;

    static const uint16_t ID;
    static const char* Name;

    bool _intr_write;
};

extern SPISlave_ SPISlave;

#endif//__SPISLAVE_H__
