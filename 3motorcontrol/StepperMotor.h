#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <math.h>

/*
Parameters:
* motorBase: Vector of size 3 representing the position of the motor base in 3D space
* anchorEE: Vector of size 3 representing the position of the anchor point on the end effector in 3D space
* EEInitBase: Vector of size 3 representing the initial position of the end effector in 3D space
* pulleyRadius: Radius of the pulley
* stepsPerRevolution: Number of steps the motor takes to complete one revolution

Returns: A StepperMotor object that can be used to calculate the motor steps 
needed to move the end effector to a desired position
*/
class StepperMotor{
    public:
    // Constructor
    StepperMotor(){};
    StepperMotor(const float motorBase[3], 
				 const float anchorEE[3],
				 const float EEInitBase[3], 
				 float winchRadius,
				 float stepsPerRevolution);

    /*
	Parameters:
	EEPosition: Vector of size 3 representing the position of the end effector in

	Returns: The number of steps the motor needs to move to reach the desired end effector position
	*/
	int calculateMotorSteps(const float* EEPosition);

    private:
    float motorBase[3];
	float anchorEE[3];
	float winchCircle;
	float stepsPerRevolution;
	float stringLength;

    /*
	Parameters:
	EEPosition: Vector of size 3 representing the position of the end effector in 3D space

	Returns: The length of the string from the motor base to the end effector
	*/
	double calculateStringLength(const float* EEPosition);
};

#endif