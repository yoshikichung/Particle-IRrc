#include <IRrc.h>

#define IRRX_PIN D2;

IRrecv irrecv(IRRX_PIN);
decode_results results;

void setup()
{
  pinMode(IRRX_PIN, INPUT);
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.print("Value: ");
    Serial.print(results.value, HEX);
    Serial.print(", Bits: ");
    Serial.print(results.bits);
    Serial.print(", Decode_type: ");
    Serial.println(results.decode_type);
    irrecv.resume(); // Receive the next value
  }
}
