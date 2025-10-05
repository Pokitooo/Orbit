
// Basic demo for accelerometer readings from Adafruit H3LIS331

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS331HH.h>
#include <Adafruit_Sensor.h>
#include "orbit_pin_def.h"

TwoWire i2c1(PIN_SDA, PIN_SCL);

bool waiting_to_reset;
// Adafruit_H3LIS331 lis = Adafruit_H3LIS331();
Adafruit_LIS331HH lis = Adafruit_LIS331HH();

void setup(void)
{
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("H3LIS331 test!");
    i2c1.begin();
    i2c1.setClock(340000ul);

    //  if (!lis.begin_SPI(H3LIS331_CS)) {
    //  if (!lis.begin_SPI(H3LIS331_CS, H3LIS331_SCK, H3LIS331_MISO, H3LIS331_MOSI)) {
    if (!lis.begin_I2C(0x18, &i2c1))
    { // change this to 0x19 for alternative i2c address
        Serial.println("Couldnt start");
        while (1)
            yield();
    }
    Serial.println("LIS331 found!");

    // lis.setRange(H3LIS331_RANGE_100_G); // For the H3LIS331
    lis.setRange(LIS331HH_RANGE_6_G);

    // lis.setDataRate(LIS331_DATARATE_LOWPOWER_10_HZ); // Use to test the low-pass filter
    lis.setDataRate(LIS331_DATARATE_400_HZ); // Use the highest data rate for a higher resolution signal
    // INT1, active low, open drain, (while inactive, pin is high-impedence. When active pin is connected to GND)
    lis.configIntDataReady(1, true, true);

    // INT2, active low, push-pull, (When Active pin is connected to GND. While inactive, pin is connected to VCC)
    //  lis.configIntDataReady(2, true, false);

    // INT2, active high, push-pull, (When Active pin is connected to VCC. While inactive, pin is high-impedence)
    //  lis.configIntDataReady(2, false, false);
}

void loop()
{
    /* Get a new sensor event, normalized */
    sensors_event_t event;
    lis.getEvent(&event);

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("\t\tX: ");
    Serial.print(event.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(event.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(event.acceleration.z);
    Serial.println(" m/s^2 ");

    /* Alternately, given the range of the H3LIS331, display the results measured in g */
    // Serial.print("\t\tX:"); Serial.print(event.acceleration.x / SENSORS_GRAVITY_STANDARD);
    // Serial.print(" \tY: "); Serial.print(event.acceleration.y / SENSORS_GRAVITY_STANDARD);
    // Serial.print(" \tZ: "); Serial.print(event.acceleration.z / SENSORS_GRAVITY_STANDARD);
    // Serial.println(" g");

    delay(500);
}
