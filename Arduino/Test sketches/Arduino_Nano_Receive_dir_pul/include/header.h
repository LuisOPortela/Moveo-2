#include <iostream>
using  namespace std;

//Constructer: Steps, Direction, Pulses Pin, Direction Pin
struct moveoJoint
{

    int pulseRev;
    int direction;
    int pinPulse;
    int pinDir;
    AccelStepper motor;
    
    moveoJoint(int step, int dir, int pPulse, int pDir)
     : pulseRev(step), direction(dir), pinPulse(pPulse), pinDir(pDir)
    {
     
           
        if (direction != 1 && direction != -1) 
        {
            std::cerr << "Warning: Invalid direction value. Setting to default (1)." << std::endl;
            direction = 1;  // Set to default value
        }   
        motor = AccelStepper(AccelStepper::DRIVER, pinPulse, pinDir);
    }   
};
