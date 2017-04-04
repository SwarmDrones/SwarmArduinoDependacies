//
//  DroneMotor.h
//  
//
//  Created by Ariel Feliz on 1/18/17.
//
//

#ifndef DroneMotor_h
#define DroneMotor_h
#include <Arduino.h>

class DroneMotor
{
private:
    int min;
    int max;
    int pin;
    double speed;
public:
    DroneMotor(int _pin, int _min, int _max)
    {
        pin = _pin;
        min = _min;
        max = _max;
    }
    void setupMotor()
    {
        pinMode(pin, OUTPUT);
        enable();
    }
    void enable()
    {
        analogWrite(pin, 117);//10);
        delay(4000);
    }
    void setSpeed(float _speed)
    {
        speed = _speed;
        speed = mapDouble(speed, 0, 100.0, min, max);
        speed = constrain(speed, min, max);
        analogWrite(pin, speed);
    }
    
    float mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
    }
};

#endif /* DroneMotor_h */
