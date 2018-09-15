/*
  Program to automatically control the operation of the ventilation fans in the Suderbyn Closed-Loop Dome.
  The fans are turned on when the temperature at the top of the dome is higher than the air underneath the floor,
  and turned off when the temperature of the air underneath the floor is greater than or equal to the air at the top of the dome.
*/

#include <string.h>

bool verbose = false;

// Declare variables for digital pins.
// The other digital pins are set high/low, to drive the optocouplers, and subsuquently turn on/off the fans.
// Fan 1-6 coincides with the physical fans in the dome from left to right.
int fan1 = D2;
int fan2 = D3;
int fan3 = D0;
int fan4 = D1;
int fan5 = D4;
int fan6 = D5;

// Declare TCP Server for reading temperature sensors via ModbbusTCP
TCPClient sensor;

// Define address of sensor hubs

uint8_t hub0[4] = {192,168,0,10}; // Temperature probe at top of the dome ceiling.
uint8_t hub1[4] = {192,168,0,11}; // Temperature probe at top of the dome ceiling.
uint8_t hub2[4] = {192,168,0,12}; // Temperature probe at top of the dome ceiling.
uint8_t hub3[4] = {192,168,0,13}; // Temperature probe at top of the dome ceiling.
uint8_t hub4[4] = {192,168,0,14}; // Sensor probes placed inside of the pipes.

// For some reason, each Sensor hub uses a different port for ModbusTCP
// 192.168.0.10 port 500
// 192.168.0.11 port 501
// 192.168.0.12 port 502
// 192.168.0.13 port 503
// 192.168.0.14 port 504

// MODBUS TCP default port
uint16_t port0 = 500;
uint16_t port1 = 501;
uint16_t port2 = 502;
uint16_t port3 = 503;
uint16_t port4 = 504;

// Define how often to read sensors
//#define readPeriod 1000 // 60 seconds
//unsigned long lastPublish = 0;
//unsigned long now = 0;
unsigned long readStart = 0;
int loopCounter = 0;

// Define state variables
double ceilingTemp;
double pipe1temp;
double pipe2temp;
double pipe3temp;
int pipe1power = 0;
int pipe2power= 0;
int pipe3power= 0;

// Annualized geo-solar floor temperatures
double centerHigh;
double centerLow;
double westHigh;
double westLow;
double eastHigh;
double eastLow;
double southHigh;
double southLow;
double deep;

// Averaged temperatures used to toggle fans
double eastAvg;
double centerAvg;
double southAvg;
double westAvg;

//
unsigned long pipe1lastToggle = 0;
unsigned long pipe2lastToggle = 0;
unsigned long pipe3lastToggle = 0;

// Define request to read sensor data, which adheres to MODBUS TCP specification
byte channel1[12] = {0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x04,0x9C,0x40,0x00,0x01};
byte channel2[12] = {0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x04,0x9C,0x46,0x00,0x01};
byte channel3[12] = {0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x04,0x9C,0x4C,0x00,0x01};
byte channel4[12] = {0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x04,0x9C,0x52,0x00,0x01};
char inputBuffer[11]; // Response from sensor hub is always 11 bytes when reading one temperature value.


void setup() {
    if (verbose) publish("Entering setup");
    
    // Define state variables in cloud
    Particle.variable("Ceiling Temp", ceilingTemp);
    Particle.variable("Pipe 1 Temp", pipe1temp);
    Particle.variable("Pipe 2 Temp", pipe2temp);
    Particle.variable("Pipe 3 Temp", pipe3temp);
    Particle.variable("Pipe 1 Power", pipe1power);
    Particle.variable("Pipe 2 Power", pipe2power);
    Particle.variable("Pipe 3 Power", pipe3power);
    
    // Define heat battery variables in cloud
    Particle.variable("Center High", centerHigh);
    Particle.variable("Center Low", centerLow);
    Particle.variable("West High", westHigh);
    Particle.variable("West Low", westLow);
    Particle.variable("East High", eastHigh);
    Particle.variable("South High", southHigh);
    Particle.variable("South Low", southLow);
    Particle.variable("Deep", deep);
    
    // Use all digital pins as output
    // Drive Ground to LOW, as it will always be LOW.
    pinMode(fan1, OUTPUT);
    pinMode(fan2, OUTPUT);
    pinMode(fan3, OUTPUT);
    pinMode(fan4, OUTPUT);
    pinMode(fan5, OUTPUT);
    pinMode(fan6, OUTPUT);
    
    // Initially set all fans on
    /*digitalWrite(fan1, HIGH);
    digitalWrite(fan2, HIGH);
    digitalWrite(fan3, HIGH);
    digitalWrite(fan4, HIGH);
    digitalWrite(fan5, HIGH);
    digitalWrite(fan6, HIGH);*/

    // register the cloud function
    Particle.function("reset", pReset);
    
    delay(1000);
}


void loop() {
    // Read sensor every 60 seconds
    /*now = millis();
    
    if (now < lastPublish) { //the clock rolled over
        lastPublish = 0;
    } else if ((now - lastPublish) > readPeriod) {
        // It's been 30 seconds, read sensors.
        lastPublish = now;
        loopCounter = loopCounter + 1;*/
        
        // Read temperatures of ceiling and entrance of pipes
        ceilingTemp = readSensor(hub2, port2, channel4);
        publish3("Ceiling Temp", String(ceilingTemp), PRIVATE);
        pipe1temp = readSensor(hub4, port4, channel3);
        publish3("Pipe 1 Temp", String(pipe1temp), PRIVATE);
        pipe2temp = readSensor(hub4, port4, channel1);
        publish3("Pipe 2 Temp", String(pipe2temp), PRIVATE);
        pipe3temp = readSensor(hub4, port4, channel2);
        publish3("Pipe 3 Temp", String(pipe3temp), PRIVATE);
        
        // Read temperatures of heat battery
        centerHigh = readSensor(hub3, port3, channel2);
        publish3("Center High", String(centerHigh), PRIVATE);
        centerLow = readSensor(hub3, port3, channel1);
        publish3("Center Low", String(centerLow), PRIVATE);
        westHigh = readSensor(hub1, port1, channel1);
        publish3("West High", String(westHigh), PRIVATE);
        westLow = readSensor(hub0, port0, channel1);
        publish3("West Low", String(westLow), PRIVATE);
        eastHigh = readSensor(hub3, port3, channel4);
        publish3("East High", String(eastHigh), PRIVATE);
        eastLow = readSensor(hub3, port3, channel3);
        publish3("East Low", String(eastLow), PRIVATE);
        southHigh = readSensor(hub0, port0, channel3);
        publish3("South High", String(southHigh), PRIVATE);
        southLow = readSensor(hub0, port0, channel2);
        publish3("South Low", String(southLow), PRIVATE);
        deep = readSensor(hub0, port0, channel4);
        publish3("Deep", String(deep), PRIVATE);
        
        // Average temperatures in relevant zones to toggle fans
        double two = 2.0;
        eastAvg = (eastHigh + eastLow) / two;
        centerAvg = (centerHigh + centerLow) / two;
        southAvg = (southHigh + southLow) / two;
        westAvg = (westHigh + westLow) / two;
        
        pipe1power = checkMotorStates(fan1);
        publish3("Pipe 1 Power", String(pipe1power), PRIVATE);
        pipe2power = checkMotorStates(fan2);
        publish3("Pipe 2 Power", String(pipe2power), PRIVATE);
        pipe3power = checkMotorStates(fan3);
        publish3("Pipe 3 Power", String(pipe3power), PRIVATE);
    //}
}


double readSensor(uint8_t * hub, uint16_t port, byte * channel) {
    if (verbose) publish("Connecting");
    double ret;
    
    if (sensor.connect(hub, port)) {
        if (verbose) publish("Connected!");
        
        if (sensor.write(channel,12)) {
            if (verbose) publish("Request written");
            readStart = millis();//grab the current time in milliseconds
        
            while (!sensor.available() && (millis()-readStart<5000) ) {
                // Wait until data is available, or 5 seconds --- whichever comes first
            }
        
            if (sensor.available()) {
                // Read the data into the input buffer
                if (verbose) publish("We have data!");
                int i = 0;
      
                while (sensor.available()) {
                    inputBuffer[i] = sensor.read();
                    i++;
                }
                
                if (verbose) Particle.publish("Finished reading");
                if (verbose) Particle.publish("Bytes Read:", String(i));
                
                ret = processSensorData();
                
            } else {
                publish("No data from sensor.");
                return -1;
            }
        
            if (verbose) publish("Disconnecting...");
            sensor.stop();
            return ret;
        
        } else {
            if (verbose) publish("Write failed");
            return -1;
        }
    
    } else {
        if (verbose) publish("Connection failed.");
        return -1;
    }
}


double processSensorData() {
    // Process data from the sensor
    if (verbose) printPreamble();
  
    // Read two bytes, big endian
    int16_t temp = (inputBuffer[9] << 8) + inputBuffer[10];
    double ten = 10;
    double temperature = temp / ten;
    if (verbose) publish2("Temperature Reading", String(temperature));
    return temperature;
}

int checkMotorStates(int fan) {
    // Check the temperatures of the pipes and dome ceiling
    // Turn motors on/off in necessary
    
    switch (fan) {
        case D2:
            if ((ceilingTemp > southAvg) || (ceilingTemp > westAvg)) {
                digitalWrite(fan, HIGH);
                digitalWrite(fan4, HIGH);
                return 1;
            } else {
                digitalWrite(fan, LOW);
                digitalWrite(fan4, LOW);
                return 0;
            }
        
        case D3:
             if (ceilingTemp > centerAvg) {
                digitalWrite(fan, HIGH);
                digitalWrite(fan5, HIGH);
                return 1;
            } else {
                digitalWrite(fan, LOW);
                digitalWrite(fan5, LOW);
                return 0;
            }
        
        case D0:
             if (ceilingTemp > eastAvg) {
                digitalWrite(fan, HIGH);
                digitalWrite(fan6, HIGH);
                return 1;
            } else {
                digitalWrite(fan, LOW);
                digitalWrite(fan6, LOW);
                return 0;
            }
    }
    
    /*if (ceilingTemp > pipeTemp) {
        digitalWrite(fan, HIGH);
        return 1;
    } else {
        digitalWrite(fan, LOW);
        return 0;
    }*/
}


void printPreamble() {
    // Ignore the TCP preamble
    /*for (int i = 0; i < 5; i++){
        Serial.print("Byte ");
        Serial.print(i);
        Serial.print("=");
        Serial.println(inputBuffer[i],HEX);
    }*/
  
    int8_t byte;
    byte = inputBuffer[5];
    publish2("TCP message length", String(byte));
    
    byte = inputBuffer[6];
    publish2("Unit Identifier", String(byte));

    byte = inputBuffer[7];
    publish2("Function Performed", String(byte));

    byte = inputBuffer[8];
    publish2("Bytes returned by sensor", String(byte));
}


int pReset(String command) {
  //function to call to remotely reset Photon
  System.reset();
  return 1;
}

void publish(String msg){
    Particle.publish(msg);
    delay(1000);
}

void publish2(String msg, String var){
    Particle.publish(msg, var);
    delay(1000);
}

void publish3(String msg, String var, PublishFlag security){
    Particle.publish(msg, var, security);
    delay(60000);
}