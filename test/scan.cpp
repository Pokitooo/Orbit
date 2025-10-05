#include <Arduino_Extended.h>
#include "orbit_pin_def.h"


TwoWire i2c1(PIN_SDA, PIN_SCL);

void setup() {
    Serial.begin();
    i2c1.begin();
    pinMode(PB5, OUTPUT);
    pinMode(PA0, OUTPUT);
}

void loop() {
    Serial.println("Hello");
    digitalToggle(PB5);
    digitalToggle(PA0);

    i2c_detect(Serial, i2c1, 0x00, 127);
    delay(1000);
}
