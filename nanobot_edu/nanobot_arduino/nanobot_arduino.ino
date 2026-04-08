#include <WiFiNINA.h>
#include <ArduinoJson.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoMotorCarrier.h>
#include <WiFiUdp.h>
#include "udp_access_point.h"
#include <QTRSensors.h>            // Click here to get the library: http://librarymanager/All#QTRSensors 
#include "Adafruit_TCS34725.h"     // Click here to get the library: http://librarymanager/ALL#Adafruit_TCS34725

//////// USER FLAGs /////////
int sendMode = 0; //0 for serial, 1 for wifi
/////////////////////////////


//Global variables
float red, green, blue;

// create the WiFi-UDP object
udp_access_point * wifi;

// Reflectance sensor setup variables
QTRSensors qtr;
const uint8_t SensorCount = 6; // Was 4
uint16_t sensorValues[SensorCount];

//Create RGB Sensor
Adafruit_TCS34725 rgbSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//JSON variables
const size_t JSON_BUFFER_SIZE = 256;
char jsonBuffer[JSON_BUFFER_SIZE];
StaticJsonDocument<JSON_BUFFER_SIZE> doc;

//Peripheral pins that get initialized by user
int trigPin1 = 255;
int echoPin1 = 255;
int trigPin2 = 255;
int echoPin2 = 255;
int tonePin = 255;
int redPin = 255;
int greenPin = 255;
int bluePin = 255;

// Function Declarations
//Peripheral interface
void performDigitalRead(int pin);
void performAnalogRead(int pin);
void performDigitalWrite(int pin, int value);
void performReflectanceRead();
void performRGBRead();
void performAccelRead();
void performUltrasonicRead();
void performEncoderRead(int pin);
void performPiezoTone(int frequency, int duration);
void performRGBSet(int red, int green, int blue);
//Peripheral inits
void initReflectance();
void initColor();
void piezoMotorSwitcher(int pin);
void initRGB(int redpin, int greenpin, int bluepin);
//Motor drivers
void setMotor(int motor, int value);
void setServo(int servo, int value);
void initPID(float p1, float i1, float d1, float p2, float i2, float d2);
void setPID(int target1, int target2);
//Helpers
bool parseJsonMessage(const char* jsonMsg);
void sendJsonValue(int val);
void sendAck();
void sendError();
void sendJson(char replyBuffer[JSON_BUFFER_SIZE], size_t replySize);
void executeCommand(String input);

//Function implementations
// Function to perform digitalRead operation and generate JSON reply
void performDigitalRead(int pin) {
  int digitalValue = digitalRead(pin);
  sendJsonValue(digitalValue);
}

// Function to perform analogRead operation and generate JSON reply
void performAnalogRead(int pin) {
  int analogValue = analogRead(pin);
  sendJsonValue(analogValue);
}

// Function to perform digitalWrite operation
void performDigitalWrite(int pin, int value) {
  digitalWrite(pin, value);
  sendAck();
}

void performRGBSet(int red, int green, int blue) {
  if (redPin < 255 && greenPin < 255 && bluePin < 255) {
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
    sendAck();
  }
}


// Function to perform reflectance sensor read and generate JSON reply
void performReflectanceRead() {
  qtr.read(sensorValues);
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["one"] = sensorValues[5];
  replyDoc["two"] = sensorValues[1];
  replyDoc["three"] = sensorValues[2];
  replyDoc["four"] = sensorValues[3];
  replyDoc["five"] = sensorValues[4]; // Added
  replyDoc["six"] = sensorValues[0]; // Added
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  sendJson(replyBuffer, replySize);
}

// Function to perform analogRead operation and generate JSON reply
void performAccelRead() {
  // Read accelerometer values
  float x = 0;
  float y = 0;
  float z = 0;
  while (!IMU.accelerationAvailable()) {}
  IMU.readAcceleration(x, y, z);
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["x"] = x;
  replyDoc["y"] = y;
  replyDoc["z"] = z;
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  sendJson(replyBuffer, replySize);
}

void performUltrasonicRead(int trigPin, int echoPin) {
  if (trigPin < 255 && echoPin < 255) {
    unsigned long ultraread = 0; //pulseIn returns an unsigned long
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    ultraread = pulseIn(echoPin, HIGH, 10000);
    sendJsonValue(int(ultraread));
  }
  else {
    sendError();
  }
}

void performEncoderRead(int pin) {
  int count = 0;
  int countper = 0;
  switch (pin) {
    case 1:
      count = encoder1.getRawCount();
      countper = encoder1.getCountPerSecond();
      encoder1.resetCounter(0);
      break;
    case 2:
      count = encoder2.getRawCount();
      countper = encoder2.getCountPerSecond();
      encoder2.resetCounter(0);
      break;
  }
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["count"] = count;
  replyDoc["countper"] = countper;
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  sendJson(replyBuffer, replySize);
}

void performRGBRead() {
  rgbSensor.getRGB(&red, &green, &blue);
  
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["red"] = int(red);
  replyDoc["green"] = int(green);
  replyDoc["blue"] = int(blue);
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  sendJson(replyBuffer, replySize);
}

void performPiezoTone(int frequency, int duration) {
  if (tonePin < 255) { //Only do this if the piezo has been initialized
    tone(tonePin, frequency, duration);
    sendAck();
  }
  else {
    sendError();
  }
}


void initReflectance() {
  qtr.setTypeRC();
  const uint8_t SensorCount = 6; // prev: 4
  qtr.setSensorPins((const uint8_t[]) {
    A0, 12, 11, 10, 8, A1 // Added A0, A1
  }, SensorCount);
}

void initRGB(int redpin, int greenpin, int bluepin) {
  redPin = redpin;
  greenPin = greenpin;
  bluePin = bluepin;
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void initPID(float p, float i, float d){
  pid1.setGains(p, i, d);
  pid2.setGains(p, i, d);
  encoder1.resetCounter(0);
  encoder2.resetCounter(0);
}

void initColor(){
  rgbSensor.begin();
}

void setMotor(int motor, int value) {
  switch (motor) {
    case 1:
      M1.setDuty(value);
      break;
    case 2:
      M2.setDuty(value);
      break;
    case 3:
      M3.setDuty(value);
      break;
    case 4:
      M4.setDuty(value);
      break;
  }
  sendAck();
}
void setServo(int servo, int value) {
  switch (servo) {
    case 1:
      servo1.setAngle(value);
      break;
    case 2:
      servo2.setAngle(value);
      break;
    case 3:
      servo3.setAngle(value);
      break;
    case 4:
      servo4.setAngle(value);
      break;
  }
  sendAck();
}

void setPID(int target1, int target2) {
  pid1.setSetpoint(TARGET_VELOCITY, target1);
  pid2.setSetpoint(TARGET_VELOCITY, target2);
  sendAck();
}

void piezoMotorSwitcher(int pin) {
  int plusPin = 255;
  int minusPin = 255;
  switch (pin) {
    case 3:
      plusPin = 3;
      minusPin = 2;
      break;
    case 4:
      plusPin = 5;
      minusPin = 4;
      break;
  }
  pinMode(plusPin, OUTPUT);
  pinMode(minusPin, OUTPUT);
  tonePin = plusPin;
  digitalWrite(minusPin, LOW);
}

// Function to parse the JSON message
bool parseJsonMessage(const char* jsonMsg) {
  return deserializeJson(doc, jsonMsg) == DeserializationError::Ok;
}

void sendJsonValue(int val) {
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["value"] = val;
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  sendJson(replyBuffer, replySize);
}

void sendAck() {
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["message"] = "ack";
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  sendJson(replyBuffer, replySize);
}

void sendError() {
  StaticJsonDocument<JSON_BUFFER_SIZE> replyDoc;
  replyDoc["message"] = "error";
  char replyBuffer[JSON_BUFFER_SIZE];
  size_t replySize = serializeJson(replyDoc, replyBuffer, JSON_BUFFER_SIZE);
  sendJson(replyBuffer, replySize);
}

void sendJson(char replyBuffer[JSON_BUFFER_SIZE], size_t replySize) {
  if (sendMode == 0) {
    Serial.write(replyBuffer, replySize);
    Serial.println();
  }
  else if (sendMode == 1) {
    wifi->sendPacket(replyBuffer);
    Serial.print("Sent: ");
    Serial.println(replyBuffer);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize communications
  Serial.begin(115200);
  
  // Check for IMU functionality
  if (!IMU.begin()) {
    Serial.println("Failed to initialize LSM6DS3!");
    while (1);
  }
  
//  // Check for Motor Carrier
//  if (!controller.begin()) {
//    Serial.println("Failed to connect to Motor Carrier!");
//    while (1);
//  }

  // Reboot and initialize all the motors
  controller.reboot();
  delay(500);
  encoder1.resetCounter(0);
  encoder2.resetCounter(0);
  M1.setDuty(0);
  M2.setDuty(0);
  M3.setDuty(0);
  M4.setDuty(0);

  if (sendMode == 0) { //Only hang here if the arduino is for serial comm
    while (!Serial);
  }

  if (sendMode == 1) { //Only do this if the arduino is for wifi comm
    wifi = new udp_access_point(551, IPAddress(192, 168, 1, 100));
    if (wifi->isReady()) {
      Serial.println("WiFi Initialized");
    }
  }
}

void loop() {

  if (sendMode == 0) {
    if (Serial.available() > 0) {
      // Read the incoming data into the jsonBuffer
      size_t bytesRead = Serial.readBytesUntil('\n', jsonBuffer, JSON_BUFFER_SIZE - 1);
      jsonBuffer[bytesRead] = '\0'; // Null-terminate the string
      executeCommand(jsonBuffer);
    }
  }
  else if (sendMode == 1) {
    if (wifi->checkForPacket()) {
      strncpy(jsonBuffer, wifi->getPacket(), sizeof(jsonBuffer) - 1);
      Serial.println(jsonBuffer);
      executeCommand(jsonBuffer);
    }
  }
}

void executeCommand(String input) {
  // Parse the JSON message
  if (parseJsonMessage(jsonBuffer)) {
    // Retrieve the JSON packet fields
    const char* mode = doc["mode"];
    const char* periph = doc["periph"];
    int pin = doc["pin"];
    int value = doc["value"];

    // perform a "read" operation
    if (strcmp(mode, "read") == 0) {
      if (strcmp(periph, "digital") == 0) {
        performDigitalRead(pin);
      }
      else if (strcmp(periph, "analog") == 0) {
        performAnalogRead(pin);
      }
      else if (strcmp(periph, "accel") == 0) {
        performAccelRead();
      }
      else if (strcmp(periph, "encoder") == 0) {
        performEncoderRead(pin);
      }
      else if (strcmp(periph, "ultrasonic1") == 0) {
        performUltrasonicRead(trigPin1,echoPin1);
      }
      else if (strcmp(periph, "ultrasonic2") == 0) {
        performUltrasonicRead(trigPin2, echoPin2);
      }
      else if (strcmp(periph, "reflectance") == 0) {
        performReflectanceRead();
      }
      else if (strcmp(periph, "color") ==0){
        performRGBRead();
      }
    }
    // perform a "write" operation
    else if (strcmp(mode, "write") == 0) {
      if (strcmp(periph, "digital") == 0) {
        performDigitalWrite(pin, value);
      }
      else if (strcmp(periph, "led") == 0) {
        performDigitalWrite(LED_BUILTIN, value);
      }
      else if (strcmp(periph, "motor") == 0) {
        setMotor(pin, value);
      }
      else if (strcmp(periph, "motors") == 0) {
        setMotor(1, pin);
        setMotor(2, value);
      }
      else if (strcmp(periph, "servo") == 0) {
        setServo(pin, value);
      }
      else if (strcmp(periph, "piezo") == 0) {
        performPiezoTone(pin, value);
      }
      else if (strcmp(periph, "rgb") == 0) {
        int blueval = doc["blue"];
        performRGBSet(pin,value,blueval);
      }
      else if (strcmp(periph, "pid") == 0) {
        setPID(pin, value);
      }
      
      
    }
    // perform an "init" operation
    else if (strcmp(mode, "init") == 0) {
      if (strcmp(periph, "arduino") == 0) {
        digitalWrite(LED_BUILTIN, 1);
        sendAck();
      }
      else if (strcmp(periph, "wifi") == 0) {
        digitalWrite(LED_BUILTIN, 1);
        sendAck();
      }
      else if (strcmp(periph, "dinput") == 0) {
        pinMode(pin, INPUT_PULLUP);
        sendAck();
      }
      else if (strcmp(periph, "ainput") == 0) {
        pinMode(pin, INPUT);
        sendAck();
      }
      else if (strcmp(periph, "output") == 0) {
        pinMode(pin, OUTPUT);
        sendAck();
      }
      else if (strcmp(periph, "ultrasonic1") == 0) {
        trigPin1 = pin; pinMode(trigPin1, OUTPUT);
        echoPin1 = value; pinMode(echoPin1, INPUT);
        sendAck();
      }
        else if (strcmp(periph, "ultrasonic2") == 0) {
        trigPin2 = pin; pinMode(trigPin2, OUTPUT);
        echoPin2 = value; pinMode(echoPin2, INPUT);
        sendAck();
      } 
      else if (strcmp(periph, "piezo") == 0) {
        piezoMotorSwitcher(pin);
        sendAck();
      }
      else if (strcmp(periph, "reflectance") == 0) {
        initReflectance();
        sendAck();
      }
      else if (strcmp(periph, "rgb") == 0) {
        int bluepin = doc["bluePin"];
        initRGB(pin, value, bluepin);
        sendAck();
      }
      else if (strcmp(periph, "pid") == 0) {
        float p = doc["p"];
        float i = doc["i"];
        float d = doc["d"];
        initPID(p,i,d);
        sendAck();
      }
      else if (strcmp(periph, "color") == 0) {
        initColor();
        sendAck();
      }     
    }
  }
  else {
    // Print an error message if parsing failed
    Serial.print("JSON parsing error");
  }
}
