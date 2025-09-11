#include <Arduino.h>

// Blink LED using a variable for the pin
constexpr uint32_t ledPin = PB14;   // LED connected to PB15

void setup() {
  pinMode(ledPin, OUTPUT);   // Configure pin as output
}

void loop() {
  digitalWrite(ledPin, HIGH);  // Turn LED ON
  delay(500);                  // Wait 500 ms
  digitalWrite(ledPin, LOW);   // Turn LED OFF
  delay(500);                  // Wait 500 ms
}
