/*
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Renzo Mischianti www.mischianti.org All right reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef IO_EXPANDER_h
#define IO_EXPANDER_h

#include "Wire.h"

#include "Arduino.h"

#define READ_ELAPSED_TIME		10

#define INPUT_OUTPUT_CONFIG_REG	0X06
#define READ_REG			0x00
#define WRITE_REG			0x02

#define P0_0  	0      
#define P0_1  	1
#define P0_2  	2
#define P0_3  	3
#define P0_4  	4
#define P0_5  	5
#define P0_6  	6
#define P0_7  	7
#define P1_0  	8
#define P1_1	9
#define P1_2  	10
#define P1_3  	11
#define P1_4  	12
#define P1_5  	13
#define P1_6  	14
#define P1_7  	15

#include <math.h>


class ioExpander {
public:

	// ioExpander(uint8_t address);
	ioExpander(uint8_t address,  uint8_t sda, uint8_t scl);
	ioExpander(uint8_t address, uint8_t sda, uint8_t scl, uint8_t interruptPin,  void (*interruptFunction)() );

	// ioExpander(uint8_t address, uint8_t sda, uint8_t scl, uint8_t interruptPin,  void (*interruptFunction)());


	void begin();
	uint8_t deviceStatus();
	void pinMode(uint8_t pin, uint8_t mode);

	void readBuffer(bool force = true);
	uint8_t digitalRead(uint8_t pin);
	void digitalWrite(uint8_t pin, uint8_t value);
	uint16_t digitalReadAll();
	void attachInterrupt();
	void detachInterrupt();

private:
	uint8_t _address;
	
	uint8_t _sda = SDA;
	uint8_t _scl = SCL;

	TwoWire *_wire;
	
	bool _usingInterrupt = false;
	uint8_t _interruptPin = 2;
	void (*_interruptFunction)(){};

	uint16_t writeMode 	= 	0x00;
	uint16_t writeModeUp 	= 	0x00;
	uint16_t readMode 	= 	0x00;
	uint16_t readModePullDown = 0x00;
	uint16_t readModePullUp = 0x00;
	uint16_t initialBuffer= 0x00;
	uint16_t resetInitial = 0x00;
	uint16_t byteBuffered = 0;
	unsigned long lastReadMillis = 0;

	uint16_t writeByteBuffered = 0;

};

#endif

