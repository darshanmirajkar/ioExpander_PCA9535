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

#include "ioExpander.h"
#include "Wire.h"

/**
 * Constructor
 * @param address: i2c address
 */
ioExpander::ioExpander(uint8_t address, uint8_t sda, uint8_t scl){
	_wire = &Wire;

	_address = address;
	_sda = sda;
	_scl = scl;
};

ioExpander::ioExpander(uint8_t address, uint8_t sda, uint8_t scl, uint8_t interruptPin,  void (*interruptFunction)() ){
		_wire = &Wire;

		_address = address;
		_sda = sda;
		_scl = scl;

		_interruptPin = interruptPin;
		_interruptFunction = interruptFunction;

		_usingInterrupt = true;
	};


/**
 * Check for Device
 */
uint8_t ioExpander::deviceStatus(){
	_wire->begin(_sda, _scl);

	_wire->beginTransmission(_address);
	return _wire->endTransmission();

}
/**
 * wake up i2c controller
 */
void ioExpander::begin(){
	_wire->begin(_sda, _scl);

	// Check if there are pins to set low
	if (writeMode>0 || readMode>0){
		_wire->beginTransmission(_address);
		uint16_t usedPin = writeMode | readMode;
		_wire->write(INPUT_OUTPUT_CONFIG_REG);	
		_wire->write((uint8_t) ~usedPin);
		_wire->write((uint8_t) (~(usedPin >> 8)));
		_wire->endTransmission();
	}

	// If using interrupt set interrupt value to pin
	attachInterrupt();
	// inizialize last read
	lastReadMillis = millis();
}


void ioExpander::attachInterrupt(){
	// If using interrupt set interrupt value to pin
	if (_usingInterrupt){
		::pinMode(_interruptPin, INPUT_PULLUP);
		::attachInterrupt(digitalPinToInterrupt(_interruptPin), (*_interruptFunction), FALLING );
	}
}
void ioExpander::detachInterrupt(){
	// If using interrupt set interrupt value to pin
	if (_usingInterrupt){
		::detachInterrupt(digitalPinToInterrupt(_interruptPin));
	}
}
/**
 * Set if fin is OUTPUT or INPUT
 * @param pin: pin to set
 * @param mode: mode, supported only INPUT or OUTPUT (to semplify)
 */
void ioExpander::pinMode(uint8_t pin, uint8_t mode){
	if (mode == OUTPUT){
		writeMode = writeMode | bit(pin);
		readMode =  readMode & ~bit(pin);
	}else if (mode == INPUT){
		writeMode = writeMode & ~bit(pin);
		readMode =   readMode | bit(pin);;
	}else{
		Serial.println("Mode NOT Supported");
	}
};


/**
 * Read value of specified pin
 * Debounce read more fast than 10millis, non managed for interrupt mode
 * @param pin
 * @return
 */
uint8_t ioExpander::digitalRead(uint8_t pin){
	uint8_t value = LOW;
	if ((bit(pin) & writeMode)>0){
		if ((bit(pin) & writeByteBuffered)>0){
			  value = HIGH;
		  }else{
			  value = LOW;
		  }
		return value;
	}
	// Check if pin already HIGH than read and prevent reread of i2c
	if ((bit(pin) & byteBuffered)>0){
		// Serial.println("read req 1");
		value = HIGH;
	 }else if ((/*(bit(pin) & byteBuffered)<=0 && */millis() > ioExpander::lastReadMillis+READ_ELAPSED_TIME) /*|| _usingInterrupt*/){
		// Serial.println("read req 2");
		_wire->beginTransmission(_address);
		_wire->write(READ_REG);
		_wire->endTransmission(); 
		// Serial.println("Num of bytes: "+String(_wire->requestFrom(_address,(uint8_t)2)));// Begin transmission to ioExpander with the buttons
		_wire->requestFrom(_address,(uint8_t)2);
		lastReadMillis = millis();
		if(_wire->available()>0)   // If bytes are available to be recieved
		{
			// Serial.println("data received");
			uint16_t iInput = _wire->read();// Read a uint16_t
				iInput |= _wire->read() << 8;// Read a uint16_t

				// Serial.println(iInput, BIN);

			if ((iInput >0 )&& (readMode >0)){
				// Serial.println("Pin toggled");
				byteBuffered = byteBuffered | (uint16_t)iInput;
				if ((bit(pin) & byteBuffered)>0){
					value = HIGH;
				}
			}
		}
	}
	// If HIGH set to low to read buffer only one time
	if (value==HIGH){
		byteBuffered = ~bit(pin) & byteBuffered;
	}
	return value;
};

/**
 * Write on pin
 * @param pin
 * @param value
 */
void ioExpander::digitalWrite(uint8_t pin, uint8_t value){
	_wire->beginTransmission(_address);     //Begin the transmission to ioExpander
	if (value==HIGH){
		writeByteBuffered = writeByteBuffered | bit(pin);
	}else{
		writeByteBuffered = writeByteBuffered & ~bit(pin);
	}

	writeByteBuffered = writeByteBuffered & writeMode;
	// Serial.println(writeByteBuffered,BIN);
	_wire->write(WRITE_REG);
	_wire->write((uint8_t) writeByteBuffered);
	_wire->write((uint8_t) (writeByteBuffered >> 8));
	_wire->endTransmission(); 

};


uint16_t ioExpander::digitalReadAll(){
	delay(100);
	_wire->beginTransmission(_address);
	_wire->write(READ_REG);
	_wire->endTransmission();
	_wire->requestFrom(_address,(uint8_t)2);
	lastReadMillis = millis();
	if(_wire->available())   // If bytes are available to be recieved
	{
			uint16_t iInput = _wire->read();// Read a uint16_t
			iInput |= _wire->read() << 8;// Read a uint16_t
			if ((iInput)>0 or ( ~iInput)>0){
				byteBuffered = (byteBuffered & ~readMode) | (uint16_t)iInput;

			}
	}
	uint16_t byteRead = byteBuffered | writeByteBuffered;
	byteBuffered = ( readMode) | (byteBuffered  & ~readMode); //~readMode & byteBuffered;
	_wire->endTransmission(); 
	return byteRead;
}


