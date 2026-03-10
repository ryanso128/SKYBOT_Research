#ifndef CABLEBOT_H
#define CABLEBOT_H

#include <math.h>
#include "StepperMotor.h"

/*
Parameters:
* motorArray: A vector of StepperMotor objects representing the motors in the system
* EEInitBase: Vector of size 3 representing the initial position of the end effector

Returns: A Cablebot object that can be used to calculate the motor steps needed to move 
the end effector to a desired position and to generate trajectories for the end effector
*/
class Cablebot{
    public:
    // Constructors
    Cablebot(){};
    Cablebot(StepperMotor* motorArray, int motorCount,
             float* EEInitBase);
    
    // Accessors
    const float* getEEPosition(){return currEEPosition;}
    const StepperMotor* getMotorArray(){return motorArray;}

    // Methods

    /*
	Parameters:
	Circlepoints: A vector of vectors to store the points on the circle
	Center: Vector of size 3 representing the center of the circle in 3D space
	Radius: Radius of the circle

	Returns: void
	*/
	void flatCircleTrajectory(float Circlepoints[][3],
							  const float* center, 
							  float radius, int steps);
    
    /*
	Parameters:
	linePoints: A vector of vectors to store the points on the line
	destination_EE_position: Vector of size 3 representing the destination position of the end effector in 3D space
	
	Returns: void
	*/
	void lineTrajectory(float linePoints[][3],
						const float* destination_EE_position,
                        int steps);
    
    private:
	StepperMotor* motorArray;
	float* currEEPosition;
};

#endif