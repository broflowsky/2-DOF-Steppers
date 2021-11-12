/*
   Speed to Accel ratio gotta be 5:1, or not?


*/
#include <AccelStepper.h>
#include <MultiStepper.h>
#define STEPPER_LENGTH_DIR_PIN 2
#define STEPPER_LENGTH_STEP_PIN 3
#define STEPPER_LENGTH_MOTOR_INTERFACE 1
#define STEPPER_LENGTH_MAX_SPEED 5000
#define STEPPER_LENGTH_MAX_ACCEL 1400
#define STEPPER_LENGTH_CONSTANT_SPEED 5000
#define STEPPER_LENGTH_MIN_POS 0
#define STEPPER_LENGTH_MAX_POS 28000
#define STEPPER_MICROSTEP 4 // could add this to step to distance conversion


#define STEPPER_WIDTH_DIR_PIN 4
#define STEPPER_WIDTH_STEP_PIN 5
#define STEPPER_WIDTH_MOTOR_INTERFACE 1
#define STEPPER_WIDTH_MAX_SPEED 5000
#define STEPPER_WIDTH_MAX_ACCEL 1400
#define STEPPER_WIDTH_CONSTANT_SPEED 5000
#define STEPPER_WIDTH_MIN_POS 0
#define STEPPER_WIDTH_MAX_POS -28000

//static allocation, multistepper takes a reference to an accelstepper
//then the objects remain the same
AccelStepper stepper_length = AccelStepper(STEPPER_LENGTH_MOTOR_INTERFACE, STEPPER_LENGTH_STEP_PIN, STEPPER_LENGTH_DIR_PIN);
AccelStepper stepper_width = AccelStepper(STEPPER_WIDTH_MOTOR_INTERFACE, STEPPER_WIDTH_STEP_PIN, STEPPER_WIDTH_DIR_PIN);
MultiStepper steppers;

#define LED_BIT_0 A3//10
#define LED_BIT_1 A0//11
#define LED_BIT_2 A2//12
#define LED_BIT_3 A1//9
byte ledPins[] = {LED_BIT_0, LED_BIT_1, LED_BIT_2, LED_BIT_3};

#define echoPin_length 6 // 
#define trigPin_length 7 //
#define echoPin_width 8 // 
#define trigPin_width 9
#define INIT_DIS 4
byte echoPins[] = {8, 6};
byte trigPins[] = {9, 7};


//0 -> 15 byte input, blinks the leds according to each bit in byte argument
void TaskSchedulerLED(byte);
void StepperSetup();
void SensorSetup();
void LedSetup();
void  InitializeStepperPosition(byte);
long stepperPos[2];//width first, length second

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  LedSetup();

  StepperSetup();
  SensorSetup();
  TaskSchedulerLED(0);
  InitializeStepperPosition(INIT_DIS);
  TaskSchedulerLED(0);


}

void loop() {

  //  stepperPos[0] = random(STEPPER_LENGTH_MIN_POS, STEPPER_LENGTH_MAX_POS);
  //  stepperPos[1] = random(STEPPER_WIDTH_MIN_POS, STEPPER_WIDTH_MAX_POS);
  stepperPos[0] = STEPPER_WIDTH_MAX_POS;
  stepperPos[1] = STEPPER_LENGTH_MAX_POS;

  steppers.moveTo(stepperPos);
  Serial.print("Width stepper destination: ");
  Serial.println(stepperPos[0]);

  Serial.print("Length stepper destination: ");
  Serial.println(stepperPos[1]);

  steppers.runSpeedToPosition();//BLOCKING
  TaskSchedulerLED(0);


  stepperPos[0] = STEPPER_WIDTH_MIN_POS;
  stepperPos[1] = STEPPER_LENGTH_MIN_POS;
  steppers.moveTo(stepperPos);
  steppers.runSpeedToPosition();//BLOCKING
  TaskSchedulerLED(0);
}
void LedSetup() {

  pinMode(LED_BIT_0, OUTPUT);
  pinMode(LED_BIT_1, OUTPUT);
  pinMode(LED_BIT_2, OUTPUT);
  pinMode(LED_BIT_3, OUTPUT);


}
void StepperSetup() {

  stepper_length.setMaxSpeed(STEPPER_LENGTH_CONSTANT_SPEED);
  stepper_length.setAcceleration(STEPPER_LENGTH_MAX_ACCEL);

  stepper_width.setMaxSpeed(STEPPER_WIDTH_CONSTANT_SPEED);
  stepper_width.setAcceleration(STEPPER_WIDTH_MAX_ACCEL);

  //C++
  steppers.addStepper(stepper_width);
  steppers.addStepper(stepper_length);

}
void SensorSetup() {

  //Ultrasound sensors
  for (byte i = 0; i < 2 ; ++i) {
    pinMode(trigPins[i], OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPins[i], INPUT); // Sets the echoPin as an INPUT
  }


}
void InitializeStepperPosition(byte distanceGoal) {

  long duration; // variable for the duration of sound wave travel
  long distance[2] = {0, 0}; // variable for the distance measurement
  //int distanceGoal = 6; //cm
  int iteration = 1;
  byte readingNb = 5;

  do {//while(distance> goal)

    TaskSchedulerLED(iteration++);//
    Serial.print("distance width in steps: ");
    Serial.println(distance[0] * 1000 * 0.45);
    Serial.print("distance length in steps: ");
    Serial.println(distance[1] * 1000 * 0.45);
    int temp = (stepper_width.currentPosition() + distance[0] * 1000 * 0.45);
    Serial.println(temp);
    stepper_width.moveTo(temp);
    stepper_length.moveTo(stepper_length.currentPosition() - distance[1] * 1000 * 0.45);

    while (stepper_length.distanceToGo() || stepper_width.distanceToGo()) {
      stepper_length.run();
      stepper_width.run();
    }
    /*
       Looks like 1mm = 100 steps at 1/4 microstepping.
    */

    for (byte i = 0 ; i < 2; ++i) {
      for (byte j = 0; j < readingNb; ++j) {
        digitalWrite(trigPins[i], LOW);
        delayMicroseconds(2);
        // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
        digitalWrite(trigPins[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPins[i], LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoPins[i], HIGH);
        // Calculating the distance, troncation is ok?
        distance[i] += duration * 0.034 * 0.5; // Speed of sound wave divided by 2 (go and back)

        delay(5);
      }
      //take the average
      distance[i] /= readingNb;
      //if too far, reduce step to account for sensor inaccuracy
      if (distance[i] > 15)
        distance[i] = 15;

      //check if one or both distanceGoal is reached
            for (byte i = 0 ; i < 2; ++i)
              distance[i] =  distance[i] < distanceGoal ? 0 : distance[i];
      //      Serial.print("distance width in cm: ");
      //      Serial.println(distance[0]);
      //      Serial.print("distance length in cm: ");
      //      Serial.println(distance[1]);
    }

  } while (distance[0] > distanceGoal || distance[1] > distanceGoal);

  stepper_length.setCurrentPosition(0);
  stepper_width.setCurrentPosition(0);



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
