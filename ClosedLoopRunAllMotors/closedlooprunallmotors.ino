/*
  Program to automatically control the operation of the ventilation fans in the Suderbyn Closed-Loop Dome.
  The fans are turned on when the temperature at the top of the dome is higher than the air underneath the floor,
  and turned off when the temperature of the air underneath the floor is greater than or equal to the air at the top of the dome.
*/


// Declare variables for digital pins.
// D0 is used as ground for the optocouplers. It will always be set to low.
// The other digital pins are set high/low, to drive the optocouplers, and subsuquently turn on/off the fans.
int fan1 = D2;
int fan2 = D3;
int fan3 = D0;
int fan4 = D1;
int fan5 = D4;
int fan6 = D5;


void setup() {
    // Use all digital pins as output
    // Drive Ground to LOW, as it will always be LOW.
    pinMode(fan1, OUTPUT);
    pinMode(fan2, OUTPUT);
    pinMode(fan3, OUTPUT);
    pinMode(fan4, OUTPUT);
    pinMode(fan5, OUTPUT);
    pinMode(fan6, OUTPUT);
    
    // Initially set all fans on
    digitalWrite(fan1, HIGH);
    digitalWrite(fan2, HIGH);
    digitalWrite(fan3, HIGH);
    digitalWrite(fan4, HIGH);
    digitalWrite(fan5, HIGH);
    digitalWrite(fan6, HIGH);
    
    String message = "Hello World";
    Particle.publish("Event", message);
}


void loop() {
}
