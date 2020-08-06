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

#include "SPI.h"
#include "SPISlave.h"
extern "C" {
  #include "gpio_irq_api.h"
  #include "gpio_irq_ex_api.h"
}

#define USI_SPI_MOSI  PA_25
#define USI_SPI_MISO  PA_26
#define USI_SPI_SCLK  PA_30
#define USI_SPI_CS    PA_28
#define RTL87XX_IRQ0  PA_12
#define RTL87XX_SYNC  PA_13

/* IRQ0 Levels */
/* On master side, slave busy if it haven't data to send */
#define SPI_SLAVE_BUSY  1
/* On slave side, we haven't data to send, should named IDLE not BUSY. */
#define SPI_SLAVE_IDLE  1
#define SPI_SLAVE_READY 0

extern "C" {
void USI_SSI_StructInitEx(USI_SSI_InitTypeDef* USI_SSI_InitStruct);
void USI_SSI_InitEx(USI_TypeDef *usi_dev, USI_SSI_InitTypeDef *USI_SSI_InitStruct);
};

/***************************************************************
 Device type & version
 ***************************************************************/
const uint16_t DEV_ID  = 0x8720;
const uint16_t DEV_VER = 0x0001; // 0.1(high-byte.low-byte)

/***************************************************************
 * Register definitions
 ***************************************************************/
enum {
	REG_DMY = 0, /* rw */
	REG_ID,      /* ro */
	REG_VER,     /* ro */
	REG_NAME,    /* ro */     // single char each read, '\0' ends.
	REG_CTRL,    /* rw */
	REG_STS,     /* ro */
	REG_IEN,     /* rw */
	#define IEN_STX 0x0001     // have data to master, slave TX
	#define IEN_SRX 0x0002     // have space could receive from master, slave RX
	REG_IRQ,     /* ro */      // auto cleared each reading
	REG_RLEN = 0x10,
	             /* rw,
	              * R -- how many bytes could read from this slave
	              * W -- how many bytes will  read
	              */
	REG_WLEN,    /* rw,
	              * R -- how many bytes could write to this slave
	              * W -- how many bytes will  write
	              */
	REG_RDATA = 0x20,
	#define RDATA_ACK 0x55AA
	             /* wo, will enter DATA TX transfer mode */
	REG_WDATA,   /* wo, will enter DATA RX transfer mode */
	#define WDATA_ACK 0xA55A
	REG_CNT,

	REG_NULL  = 0x7F,
	REG_MASK  = 0x7F,
	TAG_WRITE = 0x80,
	TAG_ACK   = 0xBE, /* Master READ bytes ready */
	TAG_DMY   = 0xFF, /* dummy */
};

SPISlave_::SPISlave_(USI_TypeDef* udev, int pinMOSI, int pinMISO, int pinSCLK, int pinCS, int pinIRQ2Mst)
{
	this->udev    = udev;
	this->pinMOSI = pinMOSI;
	this->pinMISO = pinMISO;
	this->pinSCLK = pinSCLK;
	this->pinCS   = pinCS;
	this->pinIRQ  = pinIRQ2Mst;

	regAddr = REG_NULL;
	_rxIdx = _rxCnt = 0;

	bytesWR = bytesRD = 0;

	rvIRQ = rvIEN = 0;

	_intr_write = true;
}

// set ID & name
const uint16_t SPISlave_::ID = DEV_ID;
const char* SPISlave_::Name = "SPI-Slave";

extern "C" u32 SPISlave_IT_HANDLER(void* dummy) {
	(void)dummy;

	SPISlave.onService();
	return 0UL;
}

#define __USI_ENABLE_TX(_dev,_en) \
	do { \
	USI_SSI_INTConfig(_dev,	\
		(USI_TXFIFO_ALMOST_EMTY_INTS | USI_TXFIFO_OVERFLOW_INTS), \
		_en); \
	if (!_en) { \
		for (int i = 0; i < 1000; i++) { \
			if (! (USI_SPI_SLV_TX_ACTIVITY & USI_SSI_GetTransStatus(udev))) { \
				break; \
			} \
		} \
	} \
	USI_SSI_TRxPath_Cmd(udev, USI_SPI_TX_ENABLE, _en); \
	} while(0)

#define __USI_ENABLE_RX(_dev,_en) \
	do { \
	USI_SSI_INTConfig(_dev,	\
		(USI_RXFIFO_ALMOST_FULL_INTS | USI_RXFIFO_OVERFLOW_INTS | USI_RXFIFO_UNDERFLOW_INTS), \
		_en); \
	USI_SSI_TRxPath_Cmd(udev, USI_SPI_RX_ENABLE, _en); \
	} while(0)

#define __IRQ_OUT(_v) \
	do { \
		if (pinIRQ >= 0) { \
			gpio_write(&irqOut, _v); \
		} \
	} while(0)

#define __SYNC_OUT(_v) \
	do { \
		gpio_write(&syncOut, _v); \
	} while (0)


void SPISlave_::begin(void) {
	if (pinIRQ >= 0) {
		/*
		 * IRQ0 output high level,
		 * if this/slave have no events
		 */
		gpio_init(&irqOut, (PinName)pinIRQ);
		gpio_write(&irqOut, SPI_SLAVE_IDLE);
		gpio_mode(&irqOut, PullUp);
		gpio_dir(&irqOut, PIN_OUTPUT);
	}

	gpio_init(&syncOut, (PinName)RTL87XX_SYNC);
	gpio_write(&syncOut, SPI_SLAVE_IDLE);
	gpio_mode(&syncOut, PullUp);
	gpio_dir(&syncOut, PIN_OUTPUT);

	RCC_PeriphClockCmd(APBPeriph_USI_REG, APBPeriph_USI_CLOCK, ENABLE);

	/* slave pinmux */
	Pinmux_Config(pinMOSI, PINMUX_FUNCTION_SPIS);
	Pinmux_Config(pinMISO, PINMUX_FUNCTION_SPIS);
	Pinmux_Config(pinSCLK, PINMUX_FUNCTION_SPIS);
	Pinmux_Config(pinCS,   PINMUX_FUNCTION_SPIS);

	USI_SSI_StructInitEx(&usIs);
	usIs.USI_SPI_Role = USI_SPI_SLAVE;
	/* interrupt when TX FIFO empty through USI_TXFIFO_ALMOST_EMTY_INTS */
	usIs.USI_SPI_TxThresholdLevel = 0;
	USI_SSI_InitEx(udev, &usIs);

	USI_SSI_SetBaud(udev, 10* 1000000UL,  CPU_ClkGet(_FALSE) / 2);

	irqUsi = USI_DEV_TABLE[0].IrqNum;
	InterruptRegister((IRQ_FUN)SPISlave_IT_HANDLER, irqUsi, (u32)NULL, 5);
	InterruptEn(irqUsi, 0);

	if (usIs.USI_SPI_SclkPolarity == SCPOL_INACTIVE_IS_LOW) {
		PAD_PullCtrl((u32)pinSCLK, GPIO_PuPd_DOWN);
	} else {
		PAD_PullCtrl((u32)pinSCLK, GPIO_PuPd_UP);
	}
	PAD_PullCtrl((u32)pinCS, GPIO_PuPd_UP);

	__USI_ENABLE_RX(udev, ENABLE);

	__SYNC_OUT(SPI_SLAVE_READY);

	return;
}

void SPISlave_::setDataMode(uint8_t dataMode) {
	u32   SclkPhase;
	u32   SclkPolarity;

	/*
	* mode | POL PHA
	* -----+--------
	*   0  |  0   0
	*   1  |  0   1
	*   2  |  1   0
	*   3  |  1   1
	*
	* SCPOL_INACTIVE_IS_LOW  = 0,
	* SCPOL_INACTIVE_IS_HIGH = 1
	*
	* SCPH_TOGGLES_IN_MIDDLE = 0,
	* SCPH_TOGGLES_AT_START  = 1
	*/
	switch (dataMode) {
	case SPI_MODE0:
		SclkPolarity = SCPOL_INACTIVE_IS_LOW;
		SclkPhase    = SCPH_TOGGLES_IN_MIDDLE;
		break;
	case SPI_MODE1:
		SclkPolarity = SCPOL_INACTIVE_IS_LOW;
		SclkPhase    = SCPH_TOGGLES_AT_START;
		break;
	case SPI_MODE2:
		SclkPolarity = SCPOL_INACTIVE_IS_HIGH;
		SclkPhase    = SCPH_TOGGLES_IN_MIDDLE;
		break;
	case SPI_MODE3:
	default:  // same as 3
		SclkPolarity = SCPOL_INACTIVE_IS_HIGH;
		SclkPhase    = SCPH_TOGGLES_AT_START;
		break;
	}

	USI_SSI_SetSclkPhase(udev, SclkPhase);
	USI_SSI_SetSclkPolarity(udev, SclkPolarity);
	USI_SSI_SetDataFrameSize(udev, USI_SPI_DFS_8_BITS);

	if (SclkPolarity == SCPOL_INACTIVE_IS_LOW) {
		PAD_PullCtrl((u32)pinSCLK, GPIO_PuPd_DOWN);
	} else {
		PAD_PullCtrl((u32)pinSCLK, GPIO_PuPd_UP);
	}
	PAD_PullCtrl((u32)pinCS, GPIO_PuPd_UP);

	usIs.USI_SPI_SclkPhase = SclkPhase;
	usIs.USI_SPI_SclkPolarity = SclkPolarity;
	return;
}

void SPISlave_::end() {
	txBuffer.clear();
	rxBuffer.clear();
}

size_t SPISlave_::write(uint8_t ucData)
{
	// No writing, without a full buffer
	if (txBuffer.isFull()) {
		return 0 ;
	}

	txBuffer.store_char( ucData );
	if (_intr_write && (rvIEN & IEN_STX)) {
		rvIRQ |= IEN_STX;
		__IRQ_OUT(SPI_SLAVE_READY);
	}
	return 1 ;
}

size_t SPISlave_::write(const uint8_t * data, size_t quantity) {
	_intr_write = false;

	// Try to store all data
	for (size_t i = 0; i < quantity; ++i) {
		// Return the number of data stored,
		// when the buffer is full (if write return 0)
		if(!write(data[i])) {
			quantity = i;
			break;
		}
	}
	if (quantity && (rvIEN & IEN_STX)) {
		rvIRQ |= IEN_STX;
		__IRQ_OUT(SPI_SLAVE_READY);
	}

	_intr_write = true;

	// All data stored
	return quantity;
}

int SPISlave_::available(void)
{
	return rxBuffer.available();
}

int SPISlave_::availableForStore(void)
{
	return txBuffer.availableForStore();
}

int SPISlave_::read(void)
{
	int c;
	c = rxBuffer.read_char();

	if (rvIEN & IEN_SRX) {
		rvIRQ |= IEN_SRX;
		__IRQ_OUT(SPI_SLAVE_READY);
	}
	return c;
}

size_t SPISlave_::readBytes(char *buffer, size_t length)
{
	size_t av;

	if (!(av = rxBuffer.available())) {
		return av;
	}
	if (length > av) {
		length = av;
	}

	char* bufend = buffer + length;
	uint8_t c;

	for (; buffer < bufend;) {
		*buffer++ = c;
	}

	if (rvIEN & IEN_SRX) {
		rvIRQ |= IEN_SRX;
		__IRQ_OUT(SPI_SLAVE_READY);
	}
	return length;
}

int SPISlave_::peek(void)
{
	return rxBuffer.peek();
}

void SPISlave_::flush(void)
{
}

void SPISlave_::onReceive(void(*function)(int))
{
	onReceiveCallback = function;
}

void SPISlave_::onRequest(void(*function)(int))
{
	onRequestCallback = function;
}



/* --------------------------------------- private implementation ----------------------------------------------- */

/* Directly write to SPI controller TX FIFO */
size_t SPISlave_::_write(const uint8_t *data, size_t quantity) {
	size_t i;

	for (i = 0; i < quantity; ++i) {
		// Return the number of data stored
		// when the buffer is full (if write return 0)
		if (!USI_SSI_Writeable(udev)) {
			break;
		}
		USI_SSI_WriteData(udev, data[i]);
	}

	// data stored
	return i;
}

size_t SPISlave_::_writeFromTxBuf(int max_bytes) {
	uint8_t c;
	int s;

	for (s = max_bytes; s > 0;) {
		if (USI_SSI_Writeable(udev) && txBuffer.available()) {
			c = txBuffer.read_char();
			USI_SSI_WriteData(udev, c);
			s--;
		} else {
			break;
		}
	}
	return max_bytes - s;
}

void SPISlave_::_regRD(void) {
	static const char* name = "";
	uint16_t v;
	volatile uint32_t tag = TAG_ACK;
	volatile int av;

	__USI_ENABLE_RX(udev, DISABLE);
	/* Clear RX FIFO */
	udev->SW_RESET &= ~(USI_SW_RESET_RXFIFO_RSTB | USI_SW_RESET_RX_RSTB);
	udev->SW_RESET |=  (USI_SW_RESET_RXFIFO_RSTB | USI_SW_RESET_RX_RSTB);

	switch (regAddr) {
	case REG_ID:
		v = ID;
		break;

	case REG_VER:
		v = DEV_VER;
		break;

	case REG_NAME:
		v = (*name) & 0xFF;
		if (!v) {
			name = Name;
		} else {
			name++;
		}
		break;

	case REG_IEN:
		v = rvIEN;
		break;

	case REG_IRQ:
		v = rvIRQ;
		rvIRQ = 0;
		__IRQ_OUT(SPI_SLAVE_IDLE);
		break;

	case REG_RLEN:
		av = txBuffer.available();
		v = (av > 0xFFFF)? 0xFFFF: av;
		break;

	case REG_WLEN:
		av = rxBuffer.availableForStore();
		v = (av > 0xFFFF)? 0xFFFF: av;
		break;

	case REG_NULL:
	default:
		/* DEBUG */
		// tag = TAG_ACK;
		v = regAddr;
		tag = 0xFF;
		break;
	}

	tag |= ((unsigned)v << 8);
	av = tag;

	// uint32_t intr_mask = ulSetInterruptMaskFromISR();
	__set_PRIMASK(1);
	/*
	 * Bug: USI_SSI_Writeable
	 */
	__USI_ENABLE_TX(udev, DISABLE);
	for (v = 0; v < 3;) {
		(void)av;
		v += _write(((uint8_t*)&av + v), 3 - v);
	}
	// vClearInterruptMaskFromISR(intr_mask);
	__set_PRIMASK(0);
	if ((v = USI_SSI_GetTxCount(udev)) != 3) {
		printf("URX Bug #3 %d\n", v);
	}
	if (av != tag) {
		printf("GCC Bug #1\n");
	}

	__USI_ENABLE_TX(udev, ENABLE);

	#if 0
	regAddr = REG_NULL;
	#else
	// continuous reading is allowed
	#endif
	return;
}

void SPISlave_::_regWR(void) {
	uint16_t v;
	uint32_t ack;

	v = 0;
	v |= (_rxBuf[1] << 0);
	v |= (_rxBuf[2] << 8);

	if (regAddr == REG_NULL) {
		return;
	}

	switch (regAddr & REG_MASK) {
	case REG_IEN:
		rvIEN = v;
		// rvIRQ &= rvIEN;
		/* interrupt immediately if condition satisfied */
		if ((rvIEN & IEN_STX) && txBuffer.available()) {
			rvIRQ |= IEN_STX;
		}
		if ((rvIEN & IEN_SRX) && rxBuffer.availableForStore()) {
			rvIRQ |= IEN_SRX;
		}
		if (rvIRQ) {
			__IRQ_OUT(SPI_SLAVE_READY);
		}
		break;

	case REG_IRQ:
		// rvIRQ &= ~v;
		// __IRQ_OUT(SPI_SLAVE_IDLE);
		break;

	case REG_RLEN:
		bytesRD = v;
		ack = RDATA_ACK << 8;
		_write((uint8_t*)&ack, 3);
		break;

	case REG_WLEN:
		bytesWR = v;
		ack = WDATA_ACK << 8;
		_write((uint8_t*)&ack, 3);
		break;

	case REG_RDATA:
		if (v != bytesRD) {
			printf("USI Usage #1\n");
			return;
		}
		/* transfer data from slave to master */
		__USI_ENABLE_RX(udev, DISABLE);
		if (bytesRD > USI_SPI_TX_FIFO_DEPTH) {
			USI_SSI_SetTxFifoLevel(udev, USI_SPI_TX_FIFO_DEPTH / 2);
		} else {
			USI_SSI_SetTxFifoLevel(udev, 0);
		}
		break;

	case REG_WDATA:
		if (v != bytesWR) {
			printf("USI Usage #2\n");
			return;
		}
		/* transfer data from master to slave */
		if (bytesWR > USI_SPI_RX_FIFO_DEPTH) {
			USI_SSI_SetRxFifoLevel(udev, USI_SPI_RX_FIFO_DEPTH / 2);
		} else {
			USI_SSI_SetRxFifoLevel(udev, bytesWR - 1);
		}
		break;

	default:
		break;
	}

	/* tell master the regWR successful */
	v = TAG_ACK;
	_write((uint8_t*)&v, 1);
	__USI_ENABLE_TX(udev, ENABLE);

	return;
}

void SPISlave_::_recv(void) {
	uint8_t c;
	int i;

	for (i = _rxCnt; USI_SSI_Readable(udev);) {
		c = USI_SSI_ReadData(udev);
		if ((unsigned)i >= sizeof _rxBuf) {
			continue;
		}
		_rxBuf[i++] = c;
	}
	_rxCnt = i;

__more_bytes:
	/*
	 * In RX transfer, all _rxBuf element are DATA
	 */
	if (regAddr == (REG_WDATA | TAG_WRITE)) {
		for (i = _rxIdx; i < _rxCnt; i++) {
			if (!rxBuffer.isFull()) {
				rxBuffer.store_char(_rxBuf[i]);
			} else {
				printf("URX Bug #2\n");
			}
			if (--bytesWR <= 0) {
				break;
			}
		}

		if (bytesWR <= 0) {
			/* RX transfer complete */
			regAddr = REG_NULL;
			USI_SSI_SetRxFifoLevel(udev, 0);

			// Calling onReceiveCallback, if exists
			if (onReceiveCallback && rxBuffer.available()) {
				onReceiveCallback(rxBuffer.available());
			}
		} else
		if (bytesWR <= USI_SPI_RX_FIFO_DEPTH) {
			USI_SSI_SetRxFifoLevel(udev, bytesWR - 1);
		}

		goto __exit;
	}

	if (_rxBuf[0] & TAG_WRITE) {
		/* Don't change regAddr now */
		if (_rxCnt < 3) {
			/* insufficient bytes to write register */
			return;
		}
	}

	regAddr = _rxBuf[0];
	if ((regAddr & REG_MASK) >= REG_CNT) {
		regAddr = REG_NULL;
	}

	/* Ignore rubbish dummy, maybe query bytes of READ command */
	if (regAddr == REG_NULL) {
		goto __exit_1;
	}

	if (regAddr & TAG_WRITE) {
		_regWR();
		if (_rxCnt > 3) {
			printf("URX WRN#1\n");
			_rxIdx = 2;
			goto __more_bytes;
		}
	} else {
		_regRD();
		if (_rxCnt > 1) {
			// printf("URX WRN#2\n");
		}
	}
	goto __exit;

__exit_1:
	#if 1
	for (i = 0; i < _rxCnt; i++)
		printf("%02X ", _rxBuf[i]);
	printf("\n");
	#endif

__exit:
	_rxIdx = _rxCnt = 0;
	return;
}

void SPISlave_::onService(void)
{
	u32 status = USI_SSI_GetIsr(udev);

	USI_SSI_SetIsrClean(udev, status);

	__SYNC_OUT(SPI_SLAVE_BUSY);

	if (status &
	(USI_TXFIFO_OVERFLOW_INTS | USI_TXFIFO_UNDERFLOW_INTS |
	 /* USI_RXFIFO_OVERFLOW_INTS |*/ USI_RXFIFO_UNDERFLOW_INTS |
	 USI_SPI_RX_DATA_FRM_ERR_INTS)
	) {
		printf("*BugST=%x\n", (unsigned)status);
	}

	if ((status & USI_RXFIFO_ALMOST_FULL_INTS)) {
		if (!USI_SSI_Readable(udev)) {
			printf("URX Bug #1\n");
		}
		_recv();
	}

	if (status & USI_TXFIFO_ALMOST_EMTY_INTS) {
		if (regAddr != (REG_RDATA | TAG_WRITE)) {
			/* transfer limit register content complete */
			/* Not resort these statements order */
			__USI_ENABLE_TX(udev, DISABLE);
			__USI_ENABLE_RX(udev, ENABLE);
		} else if (bytesRD <= 0) {
			/* TX transfer complete */
			__USI_ENABLE_TX(udev, DISABLE);
			__USI_ENABLE_RX(udev, ENABLE);
			regAddr = REG_NULL;
			bytesRD = 0;

			// Calling onRequestCallback, if exists
			if (onRequestCallback) {
				onRequestCallback(txBuffer.availableForStore());
			}
		} else {
			/* TX transfer in progress */
			bytesRD -= _writeFromTxBuf(bytesRD);
			if (bytesRD <= 0) {
				USI_SSI_SetTxFifoLevel(udev, 0);
			}
		}
	}
	__SYNC_OUT(SPI_SLAVE_READY);
}

SPISlave_ SPISlave(USI0_DEV, USI_SPI_MOSI, USI_SPI_MISO, USI_SPI_SCLK, USI_SPI_CS, RTL87XX_IRQ0);
