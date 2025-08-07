#ifndef HEADER_H
#define HEADER_H

//#include <iostream>

//Constructer: Steps, Direction, Pulses Pin, Direction Pin
struct moveoJoint
{
    int id;
    int pulseRev;
    int direction;
    int pinPulse;
    int pinDir;
    AccelStepper motor;
    
    moveoJoint(int id, int step, int dir, int pPulse, int pDir)
     : id(id), pulseRev(step), direction(dir), pinPulse(pPulse), pinDir(pDir)
    {
        if (direction != 1 && direction != -1) 
        {
            //std::cerr << "Warning: Invalid direction value. Setting to default (1)." << std::endl;
            direction = 1;  // Set to default value
        }   
        motor = AccelStepper(AccelStepper::DRIVER, pinPulse, pinDir);
        
    }   
};


#endif