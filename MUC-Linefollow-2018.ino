#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <TimerOne.h>

#define nspeed 100                                //define normal speed
#define tspeed 60                                 //define speed to turn with
#define sspeed 60                                 //define moving speed while searching
#define tt 1100

#define rightMotor 'A'                            //right Motor identification character
#define leftMotor 'B'                             //left Motor identification character

#define trigPin 7                                 //Trigger Pin of Ultrasonic Sensor
#define echoPin 2                                 //Echo Pin of Ultrasonic Sensor
#define piezo 49                                  //Pin for the Piezo
#define echo_int 0                                //Interrupt Pin for US

#define TIMER_US 50                                   // 50 uS timer duration for US
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

volatile long echo_start = 0;                         // Records start time of echo pulse
volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time
volatile int distance = 0;                            // Distance calculated form US

int searchCounter;                                    //Search cycles

#define NUM_SENSORS   8                               // number of sensors used
#define TIMEOUT       2500                            // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   22                              // Pin of the Sensor Enable
QTRSensorsRC qtrrc((unsigned char[]) {
  A8, A9, A10, A11, A12, A13, A14, A15
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);                // Create Sensor Object
unsigned int sensorValues[NUM_SENSORS];               // Values read from the sensor will be saved here
#define black 1200                                    // lower number is greater reflection => higer=darker

Adafruit_MotorShield shield = Adafruit_MotorShield(); // Create Adafruit MS Object
Adafruit_DCMotor *right = shield.getMotor(2);         // Create DC Motor object for right motor
Adafruit_DCMotor *left = shield.getMotor(1);          // Create DC Motor object for left motor

int drivecount = 0;                                   // used to know how many times we looked for obstacels
boolean linefound;                                    // is a line found?

void setup() {
  Serial.begin(115200);                               // Begin Serial COM for Debugging
  Serial.println("Init begin!");  
  shield.begin();                                     // Start COM over I2C with Adafruit MS
  stop();                                             // Stop the bot to prevent moving during Init

  pinMode(trigPin, OUTPUT);                           // Set type of pin IN/OUT
  pinMode(piezo, OUTPUT);                             // Set type of pin IN/OUT
  pinMode(echoPin, INPUT);                            // Set type of pin IN/OUT

  Timer1.initialize(TIMER_US);                        // Init timer for triggering pulses for US
  Timer1.attachInterrupt( timerIsr );                 // Set Timer method
  attachInterrupt(echo_int, echo_interrupt, CHANGE);  // Attach interrupt for US

  delay(5000);                                        // Wait a bit to ensure we have a value from the US
  while (distance == 0);                              // Wait until e we have a value from the US
  Serial.println("Init ended!");
  Serial.println("Debug:");
  Serial.println("Distance: " + distance);
}

void turnTest() {                                     // JUST FOR TESTING
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
  if (distance <= 17) {
    stop();                                          // Stop all movements of the bot
    digitalWrite(piezo, HIGH);                       // Beep a bit
    ausweichen();                                    // Go to the avoidance Method
  }

  readSensorArrey();                                 // Read the SA

  followLineWithTurnOnPoint();                       // Execute Line-Follow-Method

  delay(10);                                         // Wait a bit so things are not tooo fast
}

void testMotors() {                                  // JUST FOR TESTING
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


void readSensorArrey() {                    // Read the SA and print debug
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

void search() {
  searchCounter = 0;                                  // Init var or reset it
  Serial.println("Searching line");
  linefound = false;                                  // Init var or reset it
  
  setMotorSpeed(rightMotor, -sspeed);                 // Drive fwd with
  setMotorSpeed(leftMotor, sspeed);                   // speed defined on top
  
  long end = millis() + 400;                          // Using this to avoid missing the line
  while (end >= millis()) {
    readSensorArrey();
    if (anyBlack()) {
      linefound = true;
      break;
    }
  }
  
  stop();
  
  while (linefound == false) {
    setMotorSpeed(rightMotor, -sspeed);             // Drive fwd with
    setMotorSpeed(leftMotor, sspeed);               // speed defined on top
    
    long end = millis() + 300;                      // Using this to avoid missing the line
    while (end >= millis()) {
      readSensorArrey();
      if (anyBlack()) {
        linefound = true;
        break;
      }
    }
    
    stop();

    if (linefound) {                             // Break the while loop and enter normal working
      break;
    }
    for (int i = 0; i < 2 + searchCounter; i++) {// For every time we don't find the line we go further
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

      if (linefound) {                          // Break the for loop and enter the while loop
        break;
      }
    }
    
    for (int i = 0; i < 2 + searchCounter; i++) {// Go exactly the way back we came
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

      if (linefound) {                          // Break the for loop and enter the while loop
        break;
      }
    }
    
    if (linefound) {                            // Break the while loop and enter normal working
      break;
    }
    for (int i = 0; i < 2 + searchCounter; i++) {// For every time we don't find the line we go further
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

      if (linefound) {                            // Break the for loop and enter while loop
        break;
      }
    }
    for (int i = 0; i < 2 + searchCounter; i++) {// Go exactly the way back we came
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

      if (linefound) {                          // Break the for loop and enter while loop
        break;
      }
    }
    searchCounter++;                            //increase the var so we turn furter
  }
}

boolean anyBlack() {                            // spimple logic comparison to check if any sensor has a value that is black
  return sensorValues[0] > black || sensorValues[1] > black || sensorValues[2] > black || sensorValues[3] > black || sensorValues[4] > black || sensorValues[5] > black || sensorValues[6] > black || sensorValues[7] > black;
}

void followLineWithTurnOnPoint() {
  if (sensorValues[3] >= black || sensorValues[4] >= black) {                                     // If line is in the middle
    forward();
  } else if (sensorValues[5] >= black || sensorValues[6] >= black || sensorValues[7] >= black) {  // If line is on the right
    turnRight();
    delay(200);
  } else if (sensorValues[0] >= black || sensorValues[1] >= black || sensorValues[2] >= black) {  // If line is left
    turnLeft();
    delay(200);
  } else {                                                                                        // If line is nowhere
    search();
  }
}

void ausweichen () {                            // Obstacle avoidance method
  Serial.println("Ausweichen");
  delay(400);
  digitalWrite(piezo,LOW);                      // Shut off that beeping
  turnRight();                                   // Turn left for 'tt' seconds to get 90*
  delay (tt-100);
  stop();                                       // Stop the bot
  delay(400);                                   // Wait to be sure we're standing still
  forward();                                    // Forward long enough so we should have passed the obstacle
  delay(1300);
  stop();                                       // Stop the bot
  delay(400);                                   // Wait to be sure we're standing still
  turnLeft();                                  // Turn back to face the obstacle
  delay (tt + 100);
  stop();                                       // Stop the bot
  delay(400);                                   // Wait to be sure we're standing still
  if (distance <= 15) {                         // If there is still something
    drivecount++;                                 
    ausweichen();                               // Recall the method
  } else {
    forward();                                  // Drive around the obstacle
    delay (1500);
    stop();                                     // Stooooop!!!!
    delay(200);                                 // Wait to be sure we're standing still
    turnLeft();                                // Drive back to the line
    delay (tt);
    stop();                                     // Stop the bot!
    delay(400);                                 // Wait to be sure we're standing still
    forward();                                  // Drive fwd until we find the line
    readSensorArrey();
    while(!anyBlack()) {
      readSensorArrey();
    }
    stop();                                     // Stop the bot
    delay(400);                                 
    turnRight();                                 // Turn back to original pos
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
