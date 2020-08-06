/*
 * Name: IntrTransferSpeed.ino
 *   SPI Slave transfer speed demo.
 *
 * Author: turmary <turmary@126.com>
 * Copyright (c) 2020 Seeed Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <stdio.h>
#include <SPISlave.h>

/**************************************************************/
void requestEvent(int freeSpace);
void receiveEvent(int howMany);

void setup()
{
	Serial.begin(115200);

	SPISlave.begin();
	SPISlave.onReceive(receiveEvent);
	SPISlave.onRequest(requestEvent);
	return;
}

/*
 * We assume the slave has a big enough buffer
 * The definition should change both
 * on Master & Slave example.
 */
#define SLAVE_BUF_SZ  (10 * 1024)
RingBufferN<SLAVE_BUF_SZ> _trBuf;
static int trans_dir = 0;

void loop()
{
	if (trans_dir) {
		uint8_t c;

		while (_trBuf.available() && SPISlave.availableForStore()) {
			c = _trBuf.read_char();
			SPISlave.write(c);
		}
		if (!_trBuf.available()) {
			trans_dir = 0;
		}
	}
}

void receiveEvent(int howMany)
{
	uint8_t c;

	for (int i = 0; i < howMany; i++) {
		c = SPISlave.read();
		_trBuf.store_char(c);
	}
	if (_trBuf.isFull()) {
		trans_dir = 1;
	}
	return;
}

void requestEvent(int freeSpace)
{
	(void)freeSpace;
	return;
}
