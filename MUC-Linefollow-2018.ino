#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <TimerOne.h>

#define nspeed 100
#define tspeed 60
#define sspeed 60
#define tt 1100

#define rightMotor 'A'
#define leftMotor 'B'

#define trigPin 7
#define echoPin 2
#define piezo 49
#define echo_int 0

#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

volatile long echo_start = 0;                         // Records start of echo pulse
volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time
volatile int distance = 0;

boolean amhindernissvorbei;
int lastSensor;
int searchCounter;

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   22     // this defines the emitter and needs to be here for the sensor to work, i gave it the same pin as one of the sensor pins since it dosent matter (Edit v Leo: it matters if yuo want to switch the IR LEDs off) (Edit v Julius: Changed to A0 to use Serial COM)
QTRSensorsRC qtrrc((unsigned char[]) {
  A8, A9, A10, A11, A12, A13, A14, A15
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
#define black 1200          // lower number is greater reflection => higer=darker

Adafruit_MotorShield shield = Adafruit_MotorShield();
Adafruit_DCMotor *right = shield.getMotor(2);
Adafruit_DCMotor *left = shield.getMotor(1);

int drivecount = 0;
boolean linefound;

void setup() {
  Serial.begin(115200);
  Serial.println("Init begin!");
  shield.begin();
  stop();

  pinMode(trigPin, OUTPUT);
  pinMode(piezo, OUTPUT);
  pinMode(echoPin, INPUT);

  Timer1.initialize(TIMER_US);
  Timer1.attachInterrupt( timerIsr );
  attachInterrupt(echo_int, echo_interrupt, CHANGE);

  delay(5000);
  while (distance == 0);
  Serial.println("Init ended!");
  Serial.println("Debug:");
  Serial.println("Distance: " + distance);
}

void turnTest() {
  turnLeft();
  delay(tt);
  stop();
  delay(1000);
  turnRight();
  delay(tt);
  stop();
  delay(1000);
  turnRight();
  delay(tt);
  stop();
  delay(1000);
  turnLeft();
  delay(tt);
  stop();
  delay(1000);
}

void loop() {
  Serial.println(distance);
  if (distance <= 15) {
    stop();
    digitalWrite(piezo, HIGH);
    ausweichen();
  }

  readSensorArrey();

  followLineWithTurnOnPoint();

  delay(10);
}

void testMotors() {
  forward();
  delay(2000);
  stop();
  turnLeft();
  delay(1000);
  stop();
  forward();
  delay(2000);
  stop();
  turnRight();
  delay(1000);
  stop();
  backward();
  delay(2000);
  stop();
}

void setMotorSpeed(char motor, int motorspeed)    //choose a motor rightMotor or leftMotor and then choose a speed between 255 and -255 => motor gets set to that speed
{
  if (motor == rightMotor)
  {
    if (motorspeed > 0)
    {
      right->setSpeed(motorspeed);
      right->run(FORWARD);
    }
    else
    {
      right->setSpeed(abs(motorspeed));
      right->run(BACKWARD);
    }
  }
  else
  {
    if (motorspeed > 0)
    {
      left->setSpeed(motorspeed);
      left->run(FORWARD);
    }
    else
    {
      left->setSpeed(abs(motorspeed));
      left->run(BACKWARD);
    }
  }
}


void readSensorArrey() {
  qtrrc.read(sensorValues);

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print((sensorValues[i] >= black) ? "B" : "W");
    Serial.print('\t');
  }
  Serial.println();
}

void forward() {
  Serial.println("forward");
  setMotorSpeed(rightMotor, -nspeed);
  setMotorSpeed(leftMotor, nspeed);
}

void backward() {
  Serial.println("Backward");
  setMotorSpeed(rightMotor, nspeed);
  setMotorSpeed(leftMotor, -nspeed);
}

void turnLeft() {
  Serial.println("Turn Left");
  setMotorSpeed(rightMotor, tspeed);
  setMotorSpeed(leftMotor, tspeed);
}

void turnRight() {
  Serial.println("Turn Right");
  setMotorSpeed(rightMotor, -tspeed);
  setMotorSpeed(leftMotor, -tspeed);
}

void stop() {
  Serial.println("stop");
  right->run(RELEASE);
  left->run(RELEASE);
}

void search()
{
  searchCounter = 0;
  Serial.println("Searching line");
  linefound = false;
  setMotorSpeed(rightMotor, -sspeed);
  setMotorSpeed(leftMotor, sspeed);
  long end = millis() + 400;
  while (end >= millis()) {
    readSensorArrey();
    if (anyBlack()) {
      linefound = true;
      break;
    }
  }
  stop();
  while (linefound == false)
  {
    setMotorSpeed(rightMotor, -sspeed);
    setMotorSpeed(leftMotor, sspeed);
    long end = millis() + 300;
    while (end >= millis()) {
      readSensorArrey();
      if (anyBlack()) {
        linefound = true;
        break;
      }
    }
    stop();

    if (linefound) {
      break;
    }
    for (int i = 0; i < 2 + searchCounter; i++) {
      setMotorSpeed(rightMotor, -sspeed);
      setMotorSpeed(leftMotor, 0);

      end = millis() + 640;
      while (end >= millis()) {
        readSensorArrey();
        if (anyBlack()) {
          linefound = true;
          break;
        }
      }
      stop();

      if (linefound) {
        break;
      }
    }
    for (int i = 0; i < 2 + searchCounter; i++) {
      setMotorSpeed(rightMotor, sspeed);
      setMotorSpeed(leftMotor, 0);

      end = millis() + 640;
      while (end >= millis()) {
        readSensorArrey();
        if (anyBlack()) {
          linefound = true;
          break;
        }
      }
      stop();

      if (linefound) {
        break;
      }
    }
    if (linefound) {
      break;
    }
    for (int i = 0; i < 2 + searchCounter; i++) {
      setMotorSpeed(rightMotor, 0);
      setMotorSpeed(leftMotor, sspeed);

      end = millis() + 660;
      while (end >= millis()) {
        readSensorArrey();
        if (anyBlack()) {
          linefound = true;
          break;
        }
      }
      stop();

      if (linefound) {
        break;
      }
    }
    for (int i = 0; i < 2 + searchCounter; i++) {
      setMotorSpeed(rightMotor, 0);
      setMotorSpeed(leftMotor, -sspeed);

      end = millis() + 660;
      while (end >= millis()) {
        readSensorArrey();
        if (anyBlack()) {
          linefound = true;
          break;
        }
      }
      stop();

      if (linefound) {
        break;
      }
    }
    searchCounter++;
  }
}

boolean anyBlack() {
  return sensorValues[0] > black || sensorValues[1] > black || sensorValues[2] > black || sensorValues[3] > black || sensorValues[4] > black || sensorValues[5] > black || sensorValues[6] > black || sensorValues[7] > black;
}

void followLineWithTurnOnPoint() {
  if (sensorValues[3] >= black || sensorValues[4] >= black) {
    forward();
  } else if (sensorValues[5] >= black || sensorValues[6] >= black || sensorValues[7] >= black) {
    turnRight();
    delay(200);
  } else if (sensorValues[0] >= black || sensorValues[1] >= black || sensorValues[2] >= black) {
    turnLeft();
    delay(200);
  } else {
    search();
  }
}

void ausweichen () {
  Serial.println("Ausweichen");
  delay(400);
  digitalWrite(piezo,LOW);
  turnLeft();
  delay (tt);
  stop();
  delay(400);
  forward();
  delay(1300);
  stop();
  delay(400);
  turnRight();
  delay (tt - 250);
  stop();
  delay(400);
  if (distance <= 15) {
    drivecount++;
    ausweichen();
  } else {
    forward();
    delay (2000);
    stop();
    delay(200);
    turnRight();
    delay (tt - 100);
    stop();
    delay(400);
    forward();
    readSensorArrey();
    while(!anyBlack()) {
      readSensorArrey();
    }
    stop();
    delay(400);
    turnLeft();
    delay (tt - 100);
    stop();
    delay(400);
  }
}


// --------------------------
// timerIsr() 50uS second interrupt ISR()
// Called every time the hardware timer 1 times out.
// --------------------------
void timerIsr()
{
  trigger_pulse();                                 // Schedule the trigger pulses
}

// --------------------------
// trigger_pulse() called every 50 uS to schedule trigger pulses.
// Generates a pulse one timer tick long.
// Minimum trigger pulse width for the HC-SR04 is 10 us. This system
// delivers a 50 uS pulse.
// --------------------------
void trigger_pulse()
{
  static volatile int state = 0;                 // State machine variable

  if (!(--trigger_time_count))                   // Count to 200mS
  { // Time out - Initiate trigger pulse
    trigger_time_count = TICK_COUNTS;           // Reload
    state = 1;                                  // Changing to state 1 initiates a pulse
  }

  switch (state)                                 // State machine handles delivery of trigger pulse
  {
    case 0:                                      // Normal state does nothing
      break;

    case 1:                                      // Initiate pulse
      digitalWrite(trigPin, HIGH);              // Set the trigger output high
      state = 2;                                // and set state to 2
      break;

    case 2:                                      // Complete the pulse
    default:
      digitalWrite(trigPin, LOW);               // Set the trigger output low
      state = 0;                                // and return state to normal 0
      break;
  }
}

// --------------------------
// echo_interrupt() External interrupt from HC-SR04 echo signal.
// Called every time the echo signal changes state.
//
// Note: this routine does not handle the case where the timer
//       counter overflows which will result in the occassional error.
// --------------------------
void echo_interrupt()
{
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_start = micros();                        // Save the start time
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      distance = (micros() - echo_start) / 60;
      break;
  }
}

// 28.03.2018
