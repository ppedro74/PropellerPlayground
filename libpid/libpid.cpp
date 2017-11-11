#include <stdio.h>              // Recommended over iostream for saving space
#include <propeller.h>          // Propeller-specific functions
#include "simpletools.h"        // Include simpletools
#include "adcDCpropab.h"        // Include adcDCpropab
#include <pid.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

static int count_millis_cog_stack[128];
volatile unsigned long millis_;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);

unsigned long millis()
{
    return millis_;
}    

void count_millis(void *par)
{
	millis_ = 0;

	int count = CNT;
	while (1)
	{
		millis_++;
		count += ms;
		waitcnt(count);
	}
}

int main()
{
    print("Test int size=%d long size=%d\r\n", sizeof(int), sizeof(long));
    
    cogstart(&count_millis, NULL, count_millis_cog_stack, sizeof count_millis_cog_stack - 1);

    adc_init(21, 20, 19, 18);                   // CS=21, SCL=20, DO=19, DI=18
    pwm_start (1000);
    
    //initialize the variables we're linked to
    Setpoint = 100;
    // Check A/D 0 
    // 0..4095
    Input = adc_in(0);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    
    while (1)
    {
        // Check A/D 0 
        Input = adc_in(0);                        
        myPID.Compute();
        
        //pin, channel (0-1), tHigh (0-1000)
        pwm_set (0, 1, Output);
    }
}    

