#include "StepperMotor.h"

StepperMotor::StepperMotor(const float* motorBase, 
                 const float* anchorEE,
                 const float* EEInitBase, 
                 float pulleyRadius,
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

        this -> pulleyCircle = 2 * M_PI * pulleyRadius;
        this -> stepsPerRevolution = stepsPerRevolution;
        this -> stringLength = calculateStringLength(EEInitBase);
}

int StepperMotor::calculateMotorSteps(const float* EEPosition){
        double destinationLength = calculateStringLength(EEPosition);
        stringLength = destinationLength;
        double lengthChange = destinationLength - stringLength;
        return int(lengthChange * stepsPerRevolution / pulleyCircle);
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
