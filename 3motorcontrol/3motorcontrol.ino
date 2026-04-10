#include <AccelStepper.h>
#include <MultiStepper.h>
#include "StepperMotor.h"
#include "Cablebot.h"

// Define the connection pins
/****************************************************************************************************
Please make the motors are placed in the right location based on the motor base/wired correctly
Additonally, make sure the string is strung correctly and wrapped around the winch counter-clockwise.
*****************************************************************************************************/
// Motor1
#define STEPPIN1 9 // Connect to PUL+
#define DIRPIN1 8 // Connect to DIR+

// Motor 2
#define STEPPIN2 6 // Connect to PUL+
#define DIRPIN2 5 // Connect to DIR+

// Motor 3
#define STEPPIN3 4 // Connect to PUL+
#define DIRPIN3 3 // Connect to DIR+

#define INTERRUPT_PIN 2 // Button connected to pin 2

#define STEPS 500
#define WINCH_RADIUS 0.375
// default is 200 steps per revolution.
#define STEPS_PER_REVOLUTION 200
#define CIRCLE_RADIUS 10.0

// for your motor
AccelStepper Stepper1(AccelStepper::DRIVER, STEPPIN1, DIRPIN1);
AccelStepper Stepper2(AccelStepper::DRIVER, STEPPIN2, DIRPIN2);
AccelStepper Stepper3(AccelStepper::DRIVER, STEPPIN3, DIRPIN3);
AccelStepper* steppersArray[3] = {&Stepper1, &Stepper2, &Stepper3};

MultiStepper steppers;

StepperMotor motorArray[3];
Cablebot cablebot = Cablebot();

long motorSteps[3];

// 1. -40
// 2. -39
float pointA[3] = {28.0, 28.0, -50.0};
float pointB[3] = {44.0, 42.0, -50.0};

volatile bool eStop = 0;

void moveToPoint();
void e_stop_ISR();

void setup() {
	Serial.begin(9600);
    // set the speed at 60 rpm:
	Stepper1.setMaxSpeed(400); //Defined in steps per second
	Stepper2.setMaxSpeed(400);
	Stepper3.setMaxSpeed(400);

	steppers.addStepper(Stepper1);
	steppers.addStepper(Stepper2);
	steppers.addStepper(Stepper3);

	// Interrupt initialization
	attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), e_stop_ISR, CHANGE);

	// Motor initialization for driving
	pinMode(STEPPIN1, OUTPUT);// rotate
	pinMode(DIRPIN1, OUTPUT);// direction
	pinMode(STEPPIN2, OUTPUT);// rotate
	pinMode(DIRPIN2, OUTPUT);// direction
	pinMode(STEPPIN3, OUTPUT);// rotate
	pinMode(DIRPIN3, OUTPUT);// direction
	pinMode(11,INPUT); // clockwise
	pinMode(12,INPUT); // anti clockwise

	// Initial placements for quadpods and end effectors
	float motor1Base[3] = {0.0, 0.0, 0.0};
	float motor2Base[3] = {87.88, 48.0, 0.0};
	float motor3Base[3] = {17.31, 89.5, 0.0};
	float anchor1EE[3] = {0.0, 0.0, 0.0};
	float anchor2EE[3] = {0.0, 0.0, 0.0};
	float anchor3EE[3] = {0.0, 0.0, 0.0};
	// Max height is 71.5
	float EEInitBase[3] = {39.56, 50.75, -71.5};

	motorArray[0] = StepperMotor(motor1Base, anchor1EE, EEInitBase, WINCH_RADIUS, STEPS_PER_REVOLUTION);
	motorArray[1] = StepperMotor(motor2Base, anchor2EE, EEInitBase, WINCH_RADIUS, STEPS_PER_REVOLUTION);
	motorArray[2] = StepperMotor(motor3Base, anchor3EE, EEInitBase, WINCH_RADIUS, STEPS_PER_REVOLUTION);

	float circle_center[3] = {39.56, 50.75, -61.5};
	cablebot = Cablebot(EEInitBase, circle_center, STEPS, CIRCLE_RADIUS);

	float point[3] = {29.56, 50.75, -61.5};
	cablebot.lineTrajectory(point);
	moveToPoint();
} 

void loop() {
	// Perform linear trajectory from point a to point b
	// cablebot.lineTrajectory(pointB);
	// moveToPoint();
	// cablebot.lineTrajectory(pointB);
	// moveToPoint();
	// while(1);
	// cablebot.lineTrajectory(pointB);
	// while(eStop);

	// Perform circular trajectory around center point
	for(int i = 0; i < STEPS && !eStop; i++){
		cablebot.stepFlatCircleTrajectory();
		moveToPoint();
	}
	while(eStop);
}

void moveToPoint(){
	if(eStop) return;
	for(int i = 0; i < 3 && !eStop; i++){
		motorSteps[i] = steppersArray[i]->currentPosition() + 
						motorArray[i].calculateMotorSteps(cablebot.getEEPosition());
		Serial.println(motorSteps[i]);
	}
	Serial.println();
	steppers.moveTo(motorSteps);
	while (steppers.run() && !eStop);
}

void e_stop_ISR(){
	eStop = 1;
	for(int i = 0; i < 3; i++){
		steppersArray[i] -> stop();
	}
}
