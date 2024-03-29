// Motor A
int enable1Pin = 4; 
int motor1Pin1 = 13; 
int motor1Pin2 = 27; 

// Motor B
int enable2Pin = 32; 
int motor2Pin1 = 33; 
int motor2Pin2 = 25; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 5;
const int pwmChannel2 = 6;
const int correctionFactor = 10;
const int resolution = 8;

#include <Servo.h>
int servoPin = 21;
Servo myservo;

#include <HCSR04.h>

int echoPin = 14;
int triggerPin = 15;
int distance = 100;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

bool autonomous = false;

void remoteForward()
{
  moveForward();
}

void remoteBackward()
{
  moveBackward();
  moveStop();
}

void remoteLeft()
{
  turnLeft();
  moveStop();
}

void remoteRight()
{
  turnRight();
  moveStop();
}

void remoteStop()
{
  moveStop();
}

void moveForward()
{
  // Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(pwmChannel1, 140); //right motor
  ledcWrite(pwmChannel2, 230 + correctionFactor); //left motor
  delay(600);
}

void moveBackward()
{
   // Move DC motor backwards at maximum speed
  Serial.println("Moving Backwards");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 
  ledcWrite(pwmChannel1, 110);
  ledcWrite(pwmChannel2, 240 + correctionFactor);
  delay(400);
}

void moveStop()
{
   // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  delay(100);
}

void turnRight()
{
   // Move DC motor backwards at maximum speed
  Serial.println("Moving Right");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 
  ledcWrite(pwmChannel1, 190);
  //ledcWrite(pwmChannel2, 190 + correctionFactor);
  delay(200);
}

void turnLeft()
{
   // Move DC motor backwards at maximum speed
  Serial.println("Moving Left");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 
  //ledcWrite(pwmChannel1, 190);
  ledcWrite(pwmChannel2, 190 + correctionFactor);
  delay(200);
}

int lookLeft()
{
  myservo.write(130);
  delay(1000);
  int distance = distanceSensor.measureDistanceCm();
  myservo.write(90);
  delay(500);
  return distance;
}


int lookRight()
{
  myservo.write(60);
  delay(1000);
  int distance = distanceSensor.measureDistanceCm();
  myservo.write(90);
  delay(500);
  return distance;
}

void autonomousCar() {

 int distanceR = 0;
 int distanceL =  0;

 if(distance <= 50)
 {
  moveStop();
  delay(100);
  moveBackward();
  delay(300);
  moveStop();
  delay(200);
  distanceR = lookRight();
  delay(200);
  distanceL = lookLeft();
  delay(200);

  if(distanceR >= distanceL)
  {
    turnRight();
    moveStop();
  } 
  else {
    turnLeft();
    moveStop();
  }
 }
 else {
  moveForward();
 }
 distance = distanceSensor.measureDistanceCm();
 if(distance <= 50)
 {
  moveStop();
  delay(100);
 }
 Serial.print(distance);
}

void setup_motorshield()
{
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);
}

void setup_servo()
{
  myservo.attach(
        servoPin, 
        Servo::CHANNEL_NOT_ATTACHED, 
        30,
        150
    );
  myservo.write(90);
}

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
          //printf("%s", rxValue.c_str());
          printf("%d", rxValue.length());
          //if(strncmp(rxValue.c_str(), "Forward", rxValue.length()) == 0)
          if(strncmp(rxValue.c_str(), "0,80,0", rxValue.length() - 2) == 0)
          {
            autonomous = false;
            remoteForward();
          }
          //if(strncmp(rxValue.c_str(), "Left", rxValue.length()) == 0)
          if(strncmp(rxValue.c_str(), "-80,80,0", rxValue.length() - 2) == 0)
          {
            autonomous = false;
            remoteLeft();
          }
          //if(strncmp(rxValue.c_str(), "Right", rxValue.length()) == 0)
          if(strncmp(rxValue.c_str(), "80,80,0", rxValue.length() - 2) == 0)
          {
            autonomous = false;
            remoteRight();
          }
          //if(strncmp(rxValue.c_str(), "Back", rxValue.length()) == 0)
          if(strncmp(rxValue.c_str(), "0,-80,0", rxValue.length() - 2) == 0)
         
          {
            autonomous = false;
            remoteBackward();
          }
          if(strncmp(rxValue.c_str(), "0,0,0", rxValue.length() - 2) == 0)
          //if(strncmp(rxValue.c_str(), "Stop", rxValue.length()) == 0)
          {
            autonomous = false;
            remoteStop();
          }
          if(strncmp(rxValue.c_str(), "Autonomous", rxValue.length()) == 0)
          {
            autonomous = true;
            autonomousCar();
          }            
      }
    }
};

void setup() {
  Serial.begin(115200);

  setup_motorshield();

  setup_servo();

  // Create the BLE Device
  BLEDevice::init("BLE Smart Car");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->addServiceUUID(pService->getUUID());
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  if(autonomous)
    autonomousCar();

    if (deviceConnected) {
        pTxCharacteristic->setValue(&txValue, 1);
        pTxCharacteristic->notify();
        txValue++;
        delay(10); // bluetooth stack will go into congestion, if too many packets are sent
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
