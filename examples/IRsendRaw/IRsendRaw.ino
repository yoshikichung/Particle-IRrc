#include <IRrc.h>

// default TX pin is A5
#define IRTX_PIN A5

IRsend irsend;

void setup() {
  pinMode(IRTX_PIN, OUTPUT);
  digitalWrite(IRTX_PIN, LOW);
  Serial.begin(9600);
}

unsigned int irSignalOn[] = {
 600, 3500,  550, 1550,  600,  450,  550,  450, 
 500,  500,  500,  500,  500,  450,  550, 1650, 
 550, 1550,  600,  400,  600,  400,  500,  500, 
 600,  450,  500,  500,  500,  500,  500,  500, 
 550,  450,  500,  500,  500,  550,  450,  550, 
 450,  500,  500,  550,  550,  400,  550,  450, 
 600,  450,  450,  500,  500,  500,  550,  450, 
 550,  500,  500,  500,  550,  450,  500,  500, 
 500,  500,  550,  450,  500,  500,  500,  500, 
 500,  550,  450,  500,  600,  400,  500,  500, 
 500,  550,  500,  450,  550,  500,  500,  500, 
 550, 1550,  600, 1550,  550, 1600,  550,  450, 
 550,  450,  500,  550,  450, 1650,  500, 1600, 
 600, 1500,  550, 3500,  550
};
unsigned int irSignalOff[] = {
 500, 3600,  500, 1600,  550,  450,  500,  500, 
 500,  500,  500,  550,  450,  550,  450,  550, 
 450,  550,  450,  550,  450,  550,  550,  450, 
 450,  550,  500,  500,  500,  500,  500,  500, 
 500,  500,  500,  500,  500,  500,  500,  500, 
 500,  550,  500,  500,  450,  550,  450,  550, 
 450,  550,  550,  450,  450,  550,  450,  550, 
 450,  550,  500,  500,  500,  500,  500,  500, 
 500,  500,  550,  450,  550,  450,  500,  500, 
 500,  550,  450,  550,  500,  500,  450,  550, 
 450,  550,  450,  550,  500,  500,  500,  500, 
 550, 1600,  450, 1700,  450, 1650,  500,  500, 
 550,  450,  500, 1650,  500,  500,  550, 1600, 
 450,  500,  600, 3450,  500
};


void loop() {
  int d;
  if ((d = Serial.read()) != -1) {
    switch(d) {
      case '0':
        irsend.sendRaw(irSignalOff, sizeof(irSignalOff), 38);
        Serial.println("Send MAXE command off...");
        break;
      case '1':
        irsend.sendRaw(irSignalOn,  sizeof(irSignalOn),  38);
        Serial.println("Send MAXE command on...");
        break;
    }
  }
}
