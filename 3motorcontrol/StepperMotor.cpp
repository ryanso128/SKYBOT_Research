#include "StepperMotor.h"

StepperMotor::StepperMotor(const float* motorBase, 
                 const float* anchorEE,
                 const float* EEInitBase, 
                 float winchRadius,
                 float stepsPerRevolution){
        for(int i = 0; i < 3; i++){
            this -> motorBase[i] = motorBase[i];
        }
        for(int i = 0; i < 3; i++){
            this -> anchorEE[i] = anchorEE[i];
        }

        for(int i = 0; i < 3; i++){
            this -> anchorBase[i] = anchorEE[i] + EEInitBase[i];
        }

        this -> winchCircle = 2 * M_PI * winchRadius;
        this -> stepsPerRevolution = stepsPerRevolution;
        this -> stringLength = calculateStringLength(EEInitBase);
}

int StepperMotor::calculateMotorSteps(const float* EEPosition){
        double destinationLength = calculateStringLength(EEPosition);
        double lengthChange = destinationLength - stringLength;
        stringLength = destinationLength;
        return int(lengthChange * stepsPerRevolution / winchCircle);
}

double StepperMotor::calculateStringLength(const float* EEPosition){
        float anchorBaseToEE[3];
        for(int i = 0; i < 3; i++){
            anchorBaseToEE[i] = EEPosition[i] + anchorEE[i] - anchorBase[i];
        }
        double length = 0;
        for(int i = 0; i < 3; i++){
            length += pow(anchorBaseToEE[i], 2);
        }
        return sqrt(length);
}
