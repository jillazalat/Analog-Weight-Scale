// ---------------------------------------------------------------- //
// Jill Aghyourli Zalat
// 500811215
// MEC751 Section 05
// Project Code
// ---------------------------------------------------------------- //
// Ultrasonic Sensor
// ----------------------------------------
#define echoPin 2
#define trigPin 3
#define PULSE_TIMEOUT 150000

// Stepper Motor
// ----------------------------------------
#include <Stepper.h>
#define STEPS 2048
Stepper stepper(STEPS, 8, 10, 9, 11);

// Data Filtering
// ----------------------------------------
#include <Filters.h>
int steps_to_take;
FilterOnePole lowPassFilter(LOWPASS, 0.09);

// Mass Calculation
// ----------------------------------------
float k = 270; // spring constant; g / cm
float currentMass = 0;

// Program Functionality
// ----------------------------------------
#define EPS1 0.25
#define EPS 0.08
#define ITERS_TO_SKIP 115 // number of iterations where stepper motor does not turn
int totalRotation = 0;
int count = 0;
float initialDistance;
float initialMass;
float previousDistance = 0;
float previousMass = 0;
bool stepped = false;
bool movedBack = true;
int steps = 0;
float requiredDisplacement;
float motorOutput;
float changeInMass;

float currentAnalogValue;
float previousAnalogValue = 0;
// Closed Loop Functionality
// ----------------------------------------
int potPin = 0;
int newFeedback = 0;
int previousFeedback = 0;
int initialAnalogValue;
float changeInFeedback;
float error=0;
// ===================================================================================
void setup() {
  // set up ultrasonic sensor
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  // set up stepper
  stepper.setSpeed(10);

  // set up serial monitor for debugging
  Serial.begin(9600);

  // wait a bit for system to initialize; calculate initial displacement and mass
  delay(5000);
  initialDistance = readDistance(); // cm
  initialMass = (initialDistance) * k; // g
  initialAnalogValue = analogRead(potPin);
}
// ===================================================================================
// ===================================================================================
void loop() {
  // Read in distance to spring platform and filter out erratic data
  float springDistance = lowPassFilter.input(readDistance());

  // Distance to platform may not always be exactly equal to initial distance; if within EPS cm of each other, consider no weight placed
  if (abs(springDistance - initialDistance) <= EPS1)
  {
    springDistance = initialDistance;
    currentMass = 0;
  }

  else // otherwise, calculate mass currently placed on platform
  {
    currentMass = (initialDistance - springDistance) * k;
  }

  // Some noise still present after filtering; if current distance is within EPS of previous distance, then consider no change in weight placed
  if (abs(springDistance - previousDistance) <= EPS)
  {
    springDistance = previousDistance;
    currentMass = (initialDistance - springDistance) * k;  // TODO CHECK IF NECESSARY
  }

  //steps_to_take = int(currentMass *0.948);
//  Serial.print("Distance: " + String(springDistance));
//  Serial.print(" Current mass: " + String(currentMass) + "Steps:" + String(steps) + " error = " + String(error) + " count: " + String(count) + "\n");
  // For first ITERS_TO_SKIP number of loop() calls, do not allow motor actuation while system is settling down
  if (count <= ITERS_TO_SKIP)
  {
    count++;
    return;
  }

      changeInMass = currentMass - previousMass;
      steps = changeInMass ;
      previousFeedback = analogRead(potPin);
      stepper.step(steps*2);
//     delay(300);
      newFeedback = analogRead(potPin);
      changeInFeedback = (abs(previousFeedback - newFeedback))*(2048.0/708.0)*(265.0/360.0);
      error = steps/1.7 - changeInFeedback;
      if (error >= 3)
      {
          stepper.step(error);  
      }
     Serial.print(steps);
     Serial.print("\t");
     Serial.println(error);
     
   
  
 previousDistance = springDistance;
  previousMass = currentMass;
  previousAnalogValue = currentAnalogValue;

}

// ===================================================================================
float readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  unsigned long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT);
  return duration / 58.82f; // microseconds to cm
   // return duration ; // for calibration
}
