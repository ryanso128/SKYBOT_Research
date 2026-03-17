#include "Cablebot.h"

Cablebot::Cablebot(float EEInitBase[3],
                     float center[3],
                     int steps,
                     float circleRadius){
    for(int i = 0; i < 3; i++){
        this -> currEEPosition[i] = EEInitBase[i];
    }
    for(int i = 0; i < 3; i++){
        this -> center[i] = center[i];
    }
    this -> circleRadius = circleRadius;

    float dtheta = 2.0 * M_PI / steps;

    cos_dtheta = cos(dtheta);
    sin_dtheta = sin(dtheta);

    dx = circleRadius;
    dy = 0.0;
}

void Cablebot::setCircleRadius(float newRadius){
    circleRadius = newRadius;
    dx = circleRadius;
    dy = 0.0;
}

void Cablebot::stepFlatCircleTrajectory(){
    currEEPosition[0] = center[0] + dx;
    currEEPosition[1] = center[1] + dy;
    currEEPosition[2] = center[2];

    dx = dx * cos_dtheta - dy * sin_dtheta;
    dy = dx * sin_dtheta + dy * cos_dtheta;
}

void Cablebot::lineTrajectory(const float destination_EE_position[3]){
    for (int i = 0; i < 3; i++){
        currEEPosition[i] = destination_EE_position[i];
    }
}