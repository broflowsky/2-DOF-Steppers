/*
   Speed to Accel ratio gotta be 5:1, or not?


*/
#include <AccelStepper.h>
#include <MultiStepper.h>
#define STEPPER_LENGTH_DIR_PIN 4
#define STEPPER_LENGTH_STEP_PIN 5
#define STEPPER_LENGTH_MOTOR_INTERFACE 1
#define STEPPER_LENGTH_MAX_SPEED 5000
#define STEPPER_LENGTH_MAX_ACCEL 1400
#define STEPPER_LENGTH_CONSTANT_SPEED 1000
#define STEPPER_LENGTH_MIN_POS 0
#define STEPPER_LENGTH_MAX_POS 4200


#define STEPPER_WIDTH_DIR_PIN 6
#define STEPPER_WIDTH_STEP_PIN 7
#define STEPPER_WIDTH_MOTOR_INTERFACE 1
#define STEPPER_WIDTH_MAX_SPEED 5000
#define STEPPER_WIDTH_MAX_ACCEL 1400
#define STEPPER_WIDTH_CONSTANT_SPEED 1000
#define STEPPER_WIDTH_MIN_POS 0
#define STEPPER_WIDTH_MAX_POS 4200

AccelStepper stepper_length = AccelStepper(STEPPER_LENGTH_MOTOR_INTERFACE, STEPPER_LENGTH_STEP_PIN, STEPPER_LENGTH_DIR_PIN);
AccelStepper stepper_width = AccelStepper(STEPPER_WIDTH_MOTOR_INTERFACE, STEPPER_WIDTH_STEP_PIN, STEPPER_WIDTH_DIR_PIN);
MultiStepper steppers;

#define LED_BIT_0 10
#define LED_BIT_1 11
#define LED_BIT_2 12
#define LED_BIT_3 9
byte ledPins[] = {LED_BIT_0, LED_BIT_1, LED_BIT_2, LED_BIT_3};

#define echoPin_length 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin_length 3 //attach pin D3 Arduino to pin Trig of HC-SR04




//0 -> 15 byte input, blinks the leds according to each bit in byte argument
void TaskSchedulerLED(byte);
void LedSetup();

long pos[2];

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  LedSetup();
  TaskSchedulerLED(0);
  InitializeStepperPosition();
  TaskSchedulerLED(0);


}

void loop() {

  pos[0] = random(STEPPER_LENGTH_MIN_POS, STEPPER_LENGTH_MAX_POS);
  pos[1] = random(STEPPER_WIDTH_MIN_POS, STEPPER_WIDTH_MAX_POS);

  steppers.moveTo(pos);
  Serial.println(pos[0]);
  Serial.println(pos[1]);


  steppers.runSpeedToPosition();
  TaskSchedulerLED(0);

}
void LedSetup() {

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);


}
void TaskSchedulerLED(byte task) {

  //flash LEDS if task is 0, meaning going home
  if (!task) {
    byte count = 10;
    while (count-- > 0) {
      for (byte i = 0; i < 4 ; ++i) {
        digitalWrite(ledPins[i], bitRead(1, i));
        delay(10);
      }
      for (byte i = 0; i < 4 ; ++i) {
        digitalWrite(ledPins[i], bitRead(2, i));
        delay(10);
      }
      for (byte i = 0; i < 4 ; ++i) {
        digitalWrite(ledPins[i], bitRead(4, i));
        delay(10);
      }
      for (byte i = 0; i < 4 ; ++i) {
        digitalWrite(ledPins[i], bitRead(8, i));
        delay(10);
      }

    }
  }
  for (byte i = 0; i < 4 ; ++i)
    digitalWrite(ledPins[i], bitRead(task, i));
}
void InitializeStepperPosition() {

  long duration; // variable for the duration of sound wave travel
  long distance = 0, prev_distance = 9999; // variable for the distance measurement
  pinMode(trigPin_length, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin_length, INPUT); // Sets the echoPin as an INPUT


  stepper_length.setMaxSpeed(STEPPER_LENGTH_CONSTANT_SPEED);
  stepper_length.setAcceleration(STEPPER_LENGTH_MAX_ACCEL);
  stepper_width.setMaxSpeed(STEPPER_WIDTH_CONSTANT_SPEED);

  int iteration = 1;
  do {

    TaskSchedulerLED(iteration++);

    stepper_length.moveTo(stepper_length.currentPosition()-10);
    
    while (stepper_length.distanceToGo()) {
      stepper_length.runSpeed();
    }

    for (int i = 0 ; i < 10; ++i) {
      digitalWrite(trigPin_length, LOW);
      delayMicroseconds(2);
      // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
      digitalWrite(trigPin_length, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin_length, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin_length, HIGH);
      // Calculating the distance, troncation is ok?
      distance += duration * 0.034 * 0.5; // Speed of sound wave divided by 2 (go and back)

      delay(5);
    }
    distance /= 10;
 
  }while (distance > 5);

//  stepper_length.setCurrentPosition(0);
//  stepper_length.setSpeed(STEPPER_LENGTH_CONSTANT_SPEED);




  steppers.addStepper(stepper_length);
  steppers.addStepper(stepper_width);

}
