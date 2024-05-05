// Arduino libraries
#include <Servo.h>

// Arduino pin definitions

#define ECHO_PIN 9 // Attach pin D2 Arduino to pin Echo of Ultrasonic 1
#define TRIG_PIN 10 // Attach pin D3 Arduino to pin Trig of Ultrasonic 1

const int DRIVE_MOTOR_PIN = 3;      //Servo 1 Pin 5
const int SHOOTER_PIN = 7;      //Servo 2 Pin 6
const int LIMIT_SWITCH_PIN = 5;       // Limit Switch 1 Pin 7

// Variable definitions
// State definitions
const int IDLE = 0;
const int DRIVE = 1;
const int REVERSE = 2;
const int SHOOT = 3;
const int RELOAD = 4;
const int SET_POS = 5;

// Max run time in millis
const unsigned int MAX_TIME = 1 * 60 * 1000 + 500;

// Ultrasonic sensor variables
int distance = 0;     // Variable for distance measurement of Ultrasonic 1

// Timer variables
unsigned long previous_millis = 0;
unsigned long timer_sh = 0;
unsigned long timer_re = 0; 
unsigned long timer_full = 0;
unsigned long full_prev_m = 0;
// unsigned long timer_delay;

// Servo Initialization
Servo DriveMotor;                     //Servo 1 initialize
Servo Shooter;                         //Servo 2 initialize

// Set the start state for the robot
int current_state = IDLE;

void setup() {  
  // Ultrasonic 1 pin declarations
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ECHO_PIN, INPUT);
  // Servos pin declarations
  DriveMotor.attach(DRIVE_MOTOR_PIN);
  Shooter.attach(SHOOTER_PIN);

  // Limit switches pin declarations
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Serial port declaration
  Serial.begin(9600);
}

void loop() {
  distance = GetDistance();
  Serial.println(distance);
  // Update current time
  unsigned long current_millis = millis();

  // Update full timer
  timer_full = current_millis - full_prev_m;
  
  // State machine
  switch (current_state) {
    case IDLE:
      DriveVelocity(0);
      break;
    
    case DRIVE:
      DriveVelocity(1);
      break;
    
    case REVERSE:
      DriveVelocity(-1);
      break;
    
    case SHOOT:
      DriveVelocity(0);
      Shoot();
      timer_sh = current_millis - previous_millis;

      break;
    
    case RELOAD:
      DriveVelocity(0);
      Reload();
      timer_re = current_millis - previous_millis;
      break;
  }

  // State machine switcher
  if (current_state == IDLE && digitalRead(LIMIT_SWITCH_PIN) == LOW) {
    // If robot is idle and limit switched is pressed switch to drive
    current_state = DRIVE;

    // Reset run timer when limit switch is pressed
    full_prev_m = millis();
  }
  else if (current_state == DRIVE && distance < 15) {
    // If robot reaches barrier switch to shoot
    current_state = SHOOT;
    previous_millis = millis();
  }
  else if (current_state == SHOOT && timer_sh > 500) {
    // Wait a time before switching to reverse
    current_state = REVERSE;
  }
  else if (current_state == REVERSE && digitalRead(LIMIT_SWITCH_PIN) == LOW) {
    current_state = RELOAD;
    previous_millis = millis();
  }
  else if (current_state == RELOAD && timer_re > 1500) {
    current_state = DRIVE;
  }

  if (timer_full > MAX_TIME) {
    Serial.println("TIME OVER");
    // Go to idle when the timer is done
    current_state = IDLE;
  }

}

int DriveVelocity(long vel) {
  vel = 90 * (vel + 1);
  DriveMotor.write(vel);
}

int GetDistance() {
  long duration;
  int distance;
  // Sample code for ultrasonic sensor 1
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  //Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  //Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);

  //Calculating the distance
  distance = duration * 0.034 / 2;

  return distance;
}

void Shoot() {
  // Shoot ping pong ball
  Shooter.write(0);
}

void Reload() {
  // Reload shooting mechanism
  Shooter.write(120);
}
