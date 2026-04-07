#include "StepperMotor.h"

StepperMotor::StepperMotor(const float motorBase[3], 
                           const float anchorEE[3],
                           const float EEInitBase[3], 
                           float winchRadius,
                           float stepsPerRevolution){
        for(int i = 0; i < 3; i++){
            this -> motorBase[i] = motorBase[i];
        }
        for(int i = 0; i < 3; i++){
            this -> anchorEE[i] = anchorEE[i];
        }

        this -> winchCircle = 2 * M_PI * winchRadius;
        this -> stepsPerRevolution = stepsPerRevolution;
        this -> stringLength = calculateStringLength(EEInitBase);
}

int StepperMotor::calculateMotorSteps(const float EEPosition[3]){
        float destinationLength = calculateStringLength(EEPosition);
        float lengthChange = destinationLength - stringLength;
        stringLength = destinationLength;
        // return int(lengthChange * stepsPerRevolution / winchCircle);
        return int(lengthChange * stepsPerRevolution);
}

float StepperMotor::calculateStringLength(const float EEPosition[3]){
        float anchorBaseToEE[3];
        for(int i = 0; i < 3; i++){
            anchorBaseToEE[i] = EEPosition[i] + anchorEE[i] - motorBase[i];
        }
        float length = 0;
        for(int i = 0; i < 3; i++){
            length += anchorBaseToEE[i] * anchorBaseToEE[i];
        }
        return sqrt(length);
}
