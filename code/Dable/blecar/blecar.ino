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
 //delay(40);
 
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

#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

void setup() {
  Serial.begin(115200);
  setup_motorshield();
  setup_servo();
  Dabble.begin("BLE Smart Car");
}

void loop() {

  if(autonomous)
    autonomousCar();

  Dabble.processInput();

  if (GamePad.isUpPressed())
  {
    autonomous = false;
    remoteForward();
  }

  if (GamePad.isDownPressed())
  {
    autonomous = false;
    remoteBackward();
  }

  if (GamePad.isLeftPressed())
  {
    autonomous = false;
    remoteLeft();
  }

  if (GamePad.isRightPressed())
  {
    autonomous = false;
    remoteRight();
  }

  if (GamePad.isCirclePressed())
  {
    autonomous = true;
    autonomousCar();
  }

  if (GamePad.isCrossPressed())
  {
     autonomous = false;
     remoteStop();
  }
}
