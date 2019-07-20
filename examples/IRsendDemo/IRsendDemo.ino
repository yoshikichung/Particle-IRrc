#include <IRrc.h>

// default TX pin is A5
#define IRTX_PIN A5

IRsend irsend;

void setup() {
  pinMode(IRTX_PIN, OUTPUT);
  digitalWrite(IRTX_PIN, LOW);
  Serial.begin(9600);
}

void loop() {
  int d;
  if ((d = Serial.read()) != -1) {
    unsigned long addr = 0x0;
    unsigned long data = 0x0;
    switch(d) {
      case '0': //off
        addr = 0x80000000; data = 0x001CA;
        break;
      case '1': //on
        addr = 0x83000000; data = 0x001C7;
        break;
      default:
        addr = 0x0; data = 0x0;
    }
    if (addr != 0x0) {
      Serial.println("Send MAXE command...");
      for (int i=0; i<3; i++) {
        irsend.sendMAXE(addr, data);
        delay(40);
      }
    }
  }
}
