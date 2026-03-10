#include "Cablebot.h"

Cablebot::Cablebot(StepperMotor* motorArray, int motorCount,
                   float* EEInitBase){
    this -> motorArray = motorArray;
    this -> currEEPosition = EEInitBase;
}

void Cablebot::flatCircleTrajectory(float Circlepoints[][3],
                                    const float* center, 
                                    float radius, int steps){
    for (int i = 0; i < steps; i++){
        float points[3] = {0.0, 0.0, 0.0};
        float angle = i * 360.0 / steps; 
        float x = center[0] + radius * cos(angle * M_PI / 180.0);
        float y = center[1] + radius * sin(angle * M_PI / 180.0);
        points[0] = x;
        points[1] = y;
        Circlepoints[i][0] = points[0];
        Circlepoints[i][1] = points[1];
        Circlepoints[i][2] = points[2];
    }
}

void Cablebot::lineTrajectory(float linePoints[][3],
                        const float* destination_EE_position,
                        int steps){
    for (int i = 0; i < steps; i++){
        float points[3] = {0.0, 0.0, 0.0};
        points[0] = currEEPosition[0] + (destination_EE_position[0] - currEEPosition[0]) * i / steps;
        points[1] = currEEPosition[1] + (destination_EE_position[1] - currEEPosition[1]) * i / steps;
        points[2] = currEEPosition[2] + (destination_EE_position[2] - currEEPosition[2]) * i / steps;
        linePoints[i][0] = points[0];
        linePoints[i][1] = points[1];
        linePoints[i][2] = points[2];
    }
}