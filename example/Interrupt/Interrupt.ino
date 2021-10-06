

#include "Arduino.h"
#include "ioExpander.h"

#define CONTROLER_INT_PIN 39
// Set i2c address
void intrFunction();
ioExpander expander(0x21,19,21,CONTROLER_INT_PIN,intrFunction);
// ioExpander expander(0x20,19,21);
void setup()
{
	delay(5000);
	Serial.begin(115200);
	// Set pinMode to OUTPUT
		
	Serial.println("Setup Begin!");
	expander.pinMode(P0_0, OUTPUT);
	expander.pinMode(P0_1, OUTPUT);
	expander.pinMode(P0_2, OUTPUT);
	expander.pinMode(P0_3, OUTPUT);
	expander.pinMode(P0_4, OUTPUT);
	expander.pinMode(P0_5, OUTPUT);
	expander.pinMode(P0_6, OUTPUT);
	expander.pinMode(P0_7, OUTPUT);

	expander.pinMode(P1_0, OUTPUT);
	expander.pinMode(P1_1, INPUT);
	expander.pinMode(P1_2, INPUT);
	expander.pinMode(P1_3, OUTPUT);
	expander.pinMode(P1_4, OUTPUT);
	expander.pinMode(P1_5, OUTPUT);
	expander.pinMode(P1_6, OUTPUT);
	expander.pinMode(P1_7, OUTPUT);

	
	expander.begin();
	
}
bool trig = false;
void loop()
{
	if(trig){
		expander.detachInterrupt();
			Serial.println(expander.digitalReadAll(),BIN);
		
		
		Serial.println("Interrupt trigger!");
		expander.attachInterrupt();															
		trig = false;
	}
	expander.digitalRead(P1_2);
  delay(100);
}

void intrFunction(){

	trig = true;

}
