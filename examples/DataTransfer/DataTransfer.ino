/*
 * Name: DataTransfer.ino
 *   SPI Slave Data Transfer Demo.
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

#define _SER_DEBUG 0

/**************************************************************/
void requestEvent(int freeSpace);
void receiveEvent(int howMany);

/***************************************************************
 * states debug function                                       *
 **************************************************************/
int debug(int v) {
	#if _SER_DEBUG
	Serial.println(v);
	#else
	(void)v;
	#endif
	return 0;
}

void setup()
{
	#if _SER_DEBUG
	Serial.begin(115200);
	#endif

	debug(0);

	SPISlave.begin();
	SPISlave.onReceive(receiveEvent);
	SPISlave.onRequest(requestEvent);

	debug(1);
	return;
}

void loop()
{
	uint32_t v;

	(void)v;
}

void receiveEvent(int howMany)
{
	uint8_t c;
	int i;
	(void)howMany;

	for (i = 0; i < howMany; i++) {
		c = SPISlave.read();
		printf("%02X ", c);
		SPISlave.write(c);
	}
	printf("\n");
	return;
}

void requestEvent(int freeSpace)
{
	(void)freeSpace;
	return;
}
