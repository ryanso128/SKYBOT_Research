#ifndef CABLEBOT_H
#define CABLEBOT_H

#include <math.h>

/*
Parameters:
* EEInitBase: Vector of size 3 representing the initial position of the end effector
* center: Vector of size 3 representing the center of the circle in 3D space
* steps: Number of steps to complete one circle
* circleRadius: Radius of the circle

Returns: A Cablebot object that can be used to calculate the next position 
of the end effector
*/
class Cablebot{
    public:
    // Constructors
    Cablebot(){};
    Cablebot(float EEInitBase[3],
			 float center[3],
			 int steps,
			 float circleRadius);
    
    // Accessors
    const float* getEEPosition(){return currEEPosition;}

    // Modifiers 

	/*
	Parameters:
	newRadius: radius of the new circle trajectory
	Returns: void
	Modifies: circleRadius to newRadius and updates dx and dy accordingly
	*/
	void setCircleRadius(float newRadius);

    /*
	Parameters: void
	Returns: void
	Modifies: currEEPosition to the next point on the flat circle trajectory
	*/
	void stepFlatCircleTrajectory();
    
    /*
	Parameters:
	destination_EE_position: Vector of size 3 representing the destination position of the end effector in 3D space
	Returns: void
	Modifies: currEEPosition to the destination_EE_position
	*/
	void lineTrajectory(const float destination_EE_position[3]);
    
    private:
	float currEEPosition[3];
	float center[3];
	float circleRadius;

	float dx;
	float dy;
	float cos_dtheta;
	float sin_dtheta;
};

#endif