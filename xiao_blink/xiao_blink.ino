/*
 * Minimal hardware check for Seeed XIAO nRF52840 Sense.
 * Blinks the onboard LED and prints a heartbeat over USB Serial.
 */

#include <Adafruit_TinyUSB.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  // USB CDC may take a moment after reset
  delay(1500);
  Serial.println();
  Serial.println("XIAO nRF52840 Sense — blink test OK");
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);   // LED on (active low on XIAO)
  Serial.println("LED ON");
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH);  // LED off
  Serial.println("LED OFF");
  delay(500);
}
