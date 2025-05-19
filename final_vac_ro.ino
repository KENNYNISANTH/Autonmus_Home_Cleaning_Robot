
#include <Servo.h> // include Servo library for ESC (Electronic Speed Controller)
Servo esc; // create servo object to control ESC

// Define ultrasonic sensor pins
const int trigPin1 = 3;  // Left sensor trig pin
const int echoPin1 = 5;  // Left sensor echo pin
const int trigPin2 = 6;  // Front sensor trig pin
const int echoPin2 = 9;  // Front sensor echo pin
const int trigPin3 = 10; // Right sensor trig pin
const int echoPin3 = 11; // Right sensor echo pin

int irpin = 2; // IR sensor input pin

// Variables to store duration and calculated distance for each sensor
long duration1;
long duration2;
long duration3;
int distanceleft;
int distancefront;
int distanceright; 

int a = 0; // flag to track IR detection
char cmnd; // variable to store command from Serial

void setup() {
  // Set ultrasonic sensor pins as input/output
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  
  pinMode(irpin, INPUT); // set IR sensor pin as input

  // Motor control pins
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(A0, INPUT); // analog input pin for speed control (potentiometer)

  Serial.begin(9600); // start serial communication

  esc.attach(13); // attach ESC to pin 13
  esc.writeMicroseconds(1000); // initialize ESC with minimum throttle
  delay(2000); // wait for ESC to arm
}

void loop() {
  // Measure distance from left sensor
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distanceleft = duration1 * 0.034 / 2;
  Serial.print("Distance1: ");
  Serial.println(distanceleft);

  // Measure distance from front sensor
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distancefront = duration2 * 0.034 / 2;
  Serial.print("Distance2: ");
  Serial.println(distancefront);

  // Measure distance from right sensor
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distanceright = duration3 * 0.034 / 2;
  Serial.print("Distance3: ");
  Serial.println(distanceright);

  int s = digitalRead(irpin); // read IR sensor state

  int val; 
  val = analogRead(A0); // read analog value from potentiometer
  val = map(val, 0, 1023, 1000, 2000); // map to ESC throttle range
  esc.writeMicroseconds(val); // set ESC speed based on potentiometer

  // If obstacle detected by IR sensor, move backward
  if(s == HIGH) { 
    digitalWrite(4, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(12, HIGH);
    delay(1000);
    a = 1;
  }

  // If no obstacle, move forward
  if ((a == 0) && (s == LOW) && (distanceleft <= 15 && distancefront > 15 && distanceright <= 15) || 
      (a == 0) && (s == LOW) && (distanceleft > 15 && distancefront > 15 && distanceright > 15)) {
    digitalWrite(4, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(12, LOW);
  }

  // Various obstacle combinations, turn left
  if ((a == 1) && (s == LOW) || 
      (s == LOW) && (distanceleft <= 15 && distancefront <= 15 && distanceright > 15) ||
      (s == LOW) && (distanceleft <= 15 && distancefront <= 15 && distanceright > 15) ||
      (s == LOW) && (distanceleft <= 15 && distancefront > 15 && distanceright > 15) ||
      (distanceleft <= 15 && distancefront > 15 && distanceright > 15)) {
    digitalWrite(4, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(12, HIGH);
    delay(100);
    a = 0;
  }

  // Turn right if only right side is blocked or front blocked
  if ((s == LOW) && (distanceleft > 15 && distancefront <= 15 && distanceright <= 15) ||
      (s == LOW) && (distanceleft > 15 && distancefront > 15 && distanceright <= 15) ||
      (s == LOW) && (distanceleft > 15 && distancefront <= 15 && distanceright > 15)) {
    digitalWrite(4, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(12, LOW);
  }

  // Check for serial command from Bluetooth or PC
  while(Serial.available() > 0) {
    digitalWrite(4, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(12, LOW);

    cmnd = Serial.read(); // read the command
    Serial.println(cmnd); // print received command

    // Movement based on command
    if(cmnd == 'F') { // Forward
      digitalWrite(4, HIGH);
      digitalWrite(7, LOW);
      digitalWrite(8, HIGH);
      digitalWrite(12, LOW);  
    }
    else if(cmnd == 'B') { // Backward
      digitalWrite(4, LOW);
      digitalWrite(7, HIGH);
      digitalWrite(8, LOW);
      digitalWrite(12, HIGH); 
    }
    else if(cmnd == 'R') { // Right
      digitalWrite(4, LOW);
      digitalWrite(7, HIGH);
      digitalWrite(8, HIGH);
      digitalWrite(12, LOW);
    }
    else if(cmnd == 'L') { // Left
      digitalWrite(4, HIGH);
      digitalWrite(7, LOW);
      digitalWrite(8, LOW);
      digitalWrite(12, HIGH); 
    }
    else if(cmnd == 'I') { // Forward Right
      digitalWrite(4, HIGH);
      digitalWrite(7, LOW);
      digitalWrite(8, LOW);
      digitalWrite(12, LOW);
    }
    else if(cmnd == 'J') { // Backward Right
      digitalWrite(4, LOW);
      digitalWrite(7, HIGH);
      digitalWrite(8, LOW);
      digitalWrite(12, LOW);
    }
    else if(cmnd == 'G') { // Forward Left
      digitalWrite(4, LOW);
      digitalWrite(7, LOW);
      digitalWrite(8, HIGH);  
      digitalWrite(12, LOW);
    }
    else if(cmnd == 'H') { // Backward Left
      digitalWrite(4, LOW);
      digitalWrite(7, LOW);
      digitalWrite(8, LOW);
      digitalWrite(12, HIGH); 
    }
  }
}
