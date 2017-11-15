#ifndef SERVOCONTEXT_H
#define SERVOCONTEXT_H

#include <pid.h>

typedef struct ServoContext
{
    char id;
    char control_pin;
    char feedback_pin;

    int high_cycle_length;
    int low_cycle_length;
    int total_cycle_length;
    int duty_cycle;
    int turns;
    int theta;
    int theta_prev;
    
    int angle;

    char deadband_positive_offset;
    char deadband_negative_offset;

    char is_attached;
    char prev_is_attached;
 
    double pid_input;
    double pid_output;
    double pid_setpoint;
    
    unsigned long millis;
    unsigned long pid_counter;
    

    PID pid;
    
    ServoContext() :
        id(0), 
        control_pin(0), 
        feedback_pin(0), 
        high_cycle_length(0), 
        low_cycle_length(0), 
        total_cycle_length(0), 
        duty_cycle(0),
        turns(0), 
        theta(0), 
        theta_prev(0), 
        angle(0), 
        deadband_positive_offset(0), 
        deadband_negative_offset(0),
        is_attached(0),
        prev_is_attached(0), 
        pid_input(0),
        pid_setpoint(0),
        pid_output(0),
        millis(0),
        pid_counter(0),
        pid(&pid_input, &pid_output, &pid_setpoint, 0.0, 0.0, 0.0, P_ON_E, DIRECT)
     {
     }            

    inline void SetSetPoint(int angle)
    {
        pid_setpoint = angle;
        is_attached = 1;
    }
    
    inline void Release()
    {
        is_attached = 0;
    }                

} ServoContext;

#endif