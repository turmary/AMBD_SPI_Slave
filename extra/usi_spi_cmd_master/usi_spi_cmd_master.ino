/*
 created 27 March 2020
 by Peter Yang <turmary@126.com>
 */

#include <SPI.h>

// pins used for the connection with the device
// the other you need are controlled by the SPI library):
#define SPIX             SPI1
const int chipSelectPin = SS1;
const int chipSyncPin   = RTL8720D_GPIO0;
#ifndef RTL8720D_IRQ0
#define RTL8720D_IRQ0 IRQ0
#endif

/* On master side, slave busy if it haven't data to send */
#define SPI_SLAVE_BUSY  1
#define SPI_SLAVE_READY 0

bool speedMode = false;

// SPI transfer tags, commonly used by target SPI slave device
enum {
	SPT_TAG_PRE = 0x55, /* Master initiate a TRANSFER */
	SPT_TAG_ACK = 0xBE, /* Master READ bytes ready */
	SPT_TAG_WR  = 0x80, /* Master WRITE  to Slave */
	SPT_TAG_RD  = 0x00, /* Master READ from Slave */
	SPT_TAG_DMY = 0xFF, /* dummy */
};

enum {
	USPI_REG_DMY  = 0x00, /* rw */
	USPI_REG_ID,          /* ro */
	USPI_REG_VER,         /* ro */
	USPI_REG_NAME,        /* ro */
	USPI_REG_CTRL,        /* rw */
	USPI_REG_STS,         /* ro */
	USPI_REG_IEN,         /* rw */
	#define IEN_STX 0x0001     // have data to master
	#define IEN_SRX 0x0002     // have space could receive from master
	USPI_REG_IRQ,         /* ro */
	USPI_REG_RLEN  = 0x10,/* rw */
	USPI_REG_WLEN,        /* rw */
	USPI_REG_RDATA = 0x20,/* wo */
	#define RDATA_ACK 0x55AA
	USPI_REG_WDATA,       /* wo */
	#define WDATA_ACK 0xA55A
	USPI_REG_CNT,
};

/* No less than 60 us between REG(read) byte and VALUE bytes. */
const int _WAIT_SLAVE_READY_US = 70;
const int _WAIT_SLAVE_READ_ACK = 100000;

int at_wait_io(int level) {
  int i;
  for (i = 0; digitalRead(chipSyncPin) != level; i++) {
    delayMicroseconds(10);
    if (i > _WAIT_SLAVE_READ_ACK / 10) {
      return -1;
    }
  }
  return 0;
}

int spi_transfer_cs(uint8_t v) {

  // take the chip select low to select the device
  digitalWrite(chipSelectPin, LOW);

  v = SPIX.transfer(v);

  // take the chip select high to de-select
  digitalWrite(chipSelectPin, HIGH);

  return v;
}

int spi_transfer16_cs(uint16_t v) {
  uint16_t r;

  // take the chip select low to select the device
  digitalWrite(chipSelectPin, LOW);

  r  = 0;
  r |= SPIX.transfer(v & 0xFF);
  delayMicroseconds(1);
  r |= SPIX.transfer(v >> 8) << 8;

  // take the chip select high to de-select
  digitalWrite(chipSelectPin, HIGH);
  return r;
}

static int uspi_rd_reg(int reg) {
  uint16_t v;

  /* To stable from last rd_reg/wr_reg */
  // delayMicroseconds(_WAIT_SLAVE_READY_US);
  // spi_transfer16_cs(((SPT_TAG_RD | reg) << 8) | SPT_TAG_PRE);
  spi_transfer_cs(SPT_TAG_RD | reg);

  at_wait_io(SPI_SLAVE_READY);
  v = spi_transfer_cs(SPT_TAG_RD | reg);
  if (v == SPT_TAG_ACK) {
    return -1;
  }

  v = spi_transfer16_cs((SPT_TAG_DMY << 8) | SPT_TAG_DMY);
  return v;
}

static int uspi_wr_reg(int reg, unsigned val) {
  uint16_t v;
  int r;

  /* To stable from last rd_reg/wr_reg */
  // delayMicroseconds(_WAIT_SLAVE_READY_US);
  // spi_transfer16_cs(((SPT_TAG_WR | reg) << 8) | SPT_TAG_PRE);
  spi_transfer_cs(SPT_TAG_WR | reg);

  /* Maybe read & write concurrently */
  // delayMicroseconds(_WAIT_SLAVE_READY_US);
  r = spi_transfer16_cs(val);

  int i;
  for (i = 0; i < _WAIT_SLAVE_READ_ACK; i += 10) {
    v = spi_transfer_cs(SPT_TAG_DMY);
    if (v == SPT_TAG_ACK) {
      break;
    }
    delayMicroseconds(10);
  }
  Serial.printf("#WRW %d\n", i);
  if (i >= _WAIT_SLAVE_READ_ACK) {
    return -1;
  }
  return r;
}

int at_cmd_write(const uint8_t* buf, uint16_t len, int loop_wait = 50) {
  uint16_t wlen;
  uint16_t v;
  int r = 0;
  (void)loop_wait;

  // Free space could be wrote into
  if (! (wlen = uspi_rd_reg(USPI_REG_WLEN))) {
    goto __ret;
  }
  if (wlen > len)
    wlen = len;

  // Request writting wlen bytes
  uspi_wr_reg(USPI_REG_WLEN, wlen);

  // Do the slave RX transfer
  v = uspi_wr_reg(USPI_REG_WDATA, wlen);
  if (v != WDATA_ACK) {
    /* device something wrong happend */
    Serial.printf("No W ACK, R%04X\r\n", v);
    r = -1;
    goto __ret;
  }

  /* wait slave ready to transfer data */
  delayMicroseconds(_WAIT_SLAVE_READY_US);

  for (int i = 0; i < wlen; i++) {
    spi_transfer_cs(buf[i]);
  }

  Serial.print("Trans ");
  Serial.print(wlen);
  Serial.println("B");
  r = wlen; /* success transfer wlen bytes */

__ret:
  return r;
}

int at_cmd_read(uint8_t* buf, uint16_t len, int loop_wait = 50) {
  uint16_t rlen;
  uint16_t v;
  int r = 0;
  (void)loop_wait;

  // How many bytes could read out
  if (! (rlen = uspi_rd_reg(USPI_REG_RLEN))) {
    goto __ret;
  }

  if (rlen > len)
    rlen = len;

  // Request reading rlen bytes
  uspi_wr_reg(USPI_REG_RLEN, rlen);

  // Do the slave TX transfer
  v = uspi_wr_reg(USPI_REG_RDATA, rlen);
  if (v != RDATA_ACK) {
    /* device something wrong happend */
    Serial.printf("No R ACK, R%04X\r\n", v);
    r = -1;
    goto __ret;
  }

  if (rlen) {
    /* wait slave ready to transfer data */
    delayMicroseconds(_WAIT_SLAVE_READY_US);

    for (int i = 0; i < rlen; i++) {
      buf[i] = spi_transfer_cs(SPT_TAG_DMY);
    }
    r = rlen; /* success transfer rlen bytes */
  }

__ret:
  return r;
}

volatile bool slave_interrupt = false;
void spi_slave_isr(void) {
  /*
   *** Dangerous!!! to Read slave register here ***
   */
  slave_interrupt = true;
  return;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // Reset SPI slave device (entire RTL8720D)
  pinMode(RTL8720D_CHIP_PU, OUTPUT);
  digitalWrite(RTL8720D_CHIP_PU, LOW);

  // initalize the data ready and chip select pins:
  pinMode(chipSyncPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);

  // start the SPI library:
  Serial.println("Begin SPI:");
  SPIX.begin();
  // Start SPI transaction at a quarter of the MAX frequency
  SPIX.beginTransaction(SPISettings(MAX_SPI / 4, MSBFIRST, SPI_MODE0));

  Serial.println("\nConnecting");
  Serial.println("\nPress key T/t to enter speed testing mode.");

  // When RTL8720D startup, set pin UART_LOG_TXD to lowlevel
  // will force the device enter UARTBURN mode.
  // Explicit high level will prevent above things.
  pinMode(PIN_SERIAL2_RX, OUTPUT);
  digitalWrite(PIN_SERIAL2_RX, HIGH);

  // Release RTL8720D reset, start bootup.
  digitalWrite(RTL8720D_CHIP_PU, HIGH);

  // give the slave time to set up
  delay(50);
  pinMode(PIN_SERIAL2_RX, INPUT);
  delay(2000);

  // Enable RTL8720D_IRQ0 interrupt
  pinMode(RTL8720D_IRQ0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTL8720D_IRQ0), spi_slave_isr, FALLING);

  uint16_t id;
  int i;
  for (i = 0; i < 10000; i++) {
    id = uspi_rd_reg(USPI_REG_ID);
    if (id != 0x8720) {
      break;
    }
  }
  Serial.printf("Slave ID  = %04X loops = %d\n", id, i);

  id = uspi_rd_reg(USPI_REG_VER);
  Serial.printf("Slave VER = %d.%d\n", id >> 8, id & 0xFF);

  for (i = 0; i < 10000; i++) {
    uspi_wr_reg(USPI_REG_IEN, IEN_STX | IEN_SRX);
    id = uspi_rd_reg(USPI_REG_IEN);
    if (id != (IEN_STX | IEN_SRX)) {
      break;
    }
  }
  Serial.printf("Slave IEN = %04X loops = %d\n", id, i);

  if (Serial.available()) {
    uint8_t c;
    c = Serial.read();
    if (c == 't' || c == 'T') {
      speedMode = true;
    }
  }
  Serial.println("Ready!");
  if (speedMode) {
    Serial.println("Speed testing started ...");
  } else {
    Serial.println("Enter some String to transfer");
  }
  return;
}

/*
 * We assume the slave has a big enough buffer
 * The definition should change both
 * on Master & Slave example.
 */
#define SLAVE_BUF_SZ  (10 * 1024)
int data_cntr = 0;
int trans_dir = 0;
unsigned long tstart;
#define TEST_BUF_SZ 1024
uint8_t _tr_buf[TEST_BUF_SZ + 2];

static void speedTest(void) {
  uint16_t istatus;

  if (data_cntr == 0) {
    tstart = millis();
  }

  istatus = 0;
  if (slave_interrupt) {
    slave_interrupt = false;

    // Serial.printf("IRQ Level #0 = %d\n",
    //   digitalRead(RTL8720D_IRQ0));
    istatus = uspi_rd_reg(USPI_REG_IRQ);
    Serial.printf("\nIRQ = %04X\n", istatus);
  }

  if (0 == trans_dir && (istatus & IEN_SRX)) {
    int i;
    for (i = 0; i < TEST_BUF_SZ; i++) {
      _tr_buf[i] = data_cntr + i;
    }

    i = at_cmd_write(_tr_buf, TEST_BUF_SZ);
    if (i < 0) {
      Serial.println("TE#0");
      return;
    }
    data_cntr += i;
    if (data_cntr >= SLAVE_BUF_SZ) {
      Serial.printf("Slave RX Transfer %d bytes,\n", data_cntr);
      Serial.printf("      using %d ms\n", millis() - tstart);
      data_cntr = 0;
      trans_dir = 1;
    }
  } else
  if (trans_dir && (istatus & IEN_STX)) {
    int i, r;

    r = at_cmd_read(_tr_buf, TEST_BUF_SZ);
    if (r < 0) {
      Serial.println("RE#0");
      return;
    }
    for (i = 0; i < r; i++) {
      if (_tr_buf[i] != (uint8_t)(data_cntr + i)) {
        Serial.printf("RE#1 RCV %02X, EXP %02X\n",
                      _tr_buf[i], (uint8_t)(data_cntr + i));
        break;
      }
    }
    data_cntr += r;
    if (data_cntr >= SLAVE_BUF_SZ) {
      Serial.printf("Slave TX Transfer %d bytes,\n", data_cntr);
      Serial.printf("      using %d ms\n", millis() - tstart);
      data_cntr = 0;
      trans_dir = 0;
    }
  }
}

#define U_BUF_SZ 254
uint8_t u_buf[U_BUF_SZ + 2];
#define S_BUF_SZ 254
uint8_t s_buf[S_BUF_SZ + 2];
int idx = 0;

static void stringTransfer(void) {
  uint8_t c;
  int r;

  while (Serial.available()) {
    c = Serial.read();

    /* not empty line or \r\n */
    if (idx == 0) {
      if (c == '\r' || c == '\n') continue;
    }

    /* process all \r, \n, \r\n */
    if (c == '\n') c = '\r';
    u_buf[idx] = c;
    if (idx < U_BUF_SZ)
      idx++;

    /* maybe leave a char '\n' in Serial object RX buffer */
    if (c == '\r')
      break;
  }

  if (idx && u_buf[idx - 1] == '\r') {
    u_buf[idx] = '\0';

    r = at_cmd_write(u_buf, idx);
    if (r < 0) {
      Serial.print("AT_WRITE ERR ");
      Serial.println(r);
      delay(1000);
    } else {
      delay(r);
    }
    idx = 0;
  }

  r = at_cmd_read(s_buf, S_BUF_SZ);
  if (r < 0) {
    Serial.print("AT_READ ERR ");
    Serial.println(r);
    delay(1000);

  } else if (r >= 0) {
    int i;

    for (i = 0; i < r; i++) {
      char obuf[0x10];
      uint8_t b = s_buf[i];
      int o;

      o = 0;
      if (b < 0x20 || b == '[' || b >= 0x7F) {
        o += sprintf(obuf + o, "[%02X", s_buf[i]);
      } else {
        o += sprintf(obuf + o, "%c", s_buf[i]);
      }
      Serial.print(obuf);
      if (b == 0x0A) {
        Serial.println();
      }
    }
    // Serial.println("Read OK");
  }
  return;
}

void loop() {
  if (speedMode) {
    speedTest();
  } else {
    // stringTransfer();
    delay(1000);
  }
}
