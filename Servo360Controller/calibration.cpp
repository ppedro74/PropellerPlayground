#include "global.h"

#define CALIBRATION_SIG "Servo360Controller.01.00"

bool check_servo_calibration(ServoContext* ctx)
{
    bool ret = false;
    
    print("INFO check_servo_calibration id=%d\n", ctx->id);
    
    if (ctx->deadband_positive_offset==0)
    {
        ret = true;
        print("...calibration started\n");
        
        int angle = ctx->angle;
        for (ctx->deadband_positive_offset=0; ctx->deadband_positive_offset<100; ctx->deadband_positive_offset++)
        {
            servo_speed(ctx->control_pin, ctx->deadband_positive_offset);
            pause(20);
            if (abs(ctx->angle-angle)>=2)
            {
                break;
            }                
        }
        
        servo_speed(ctx->control_pin, 0); 
        pause(100);
    }

    print("...positive_offset=%d\n", ctx->deadband_positive_offset);

    if (ctx->deadband_negative_offset==0)
    {
        ret = true;
        print("...calibration started\n");
        
        int angle = ctx->angle;
        for (ctx->deadband_negative_offset=0; ctx->deadband_negative_offset<100; ctx->deadband_negative_offset++)
        {
            servo_speed(ctx->control_pin, -ctx->deadband_negative_offset); 
            pause(20);
            if (abs(ctx->angle-angle)>=2)
            {
                break;
            }                
        }
        
        servo_speed(ctx->control_pin, 0); 
        pause(100);
    }
        
    print("...negative_offset=%d\n", ctx->deadband_negative_offset);
    
    return ret;
}    

void calibration_load()
{

    unsigned char sigStr[sizeof(CALIBRATION_SIG) + 1];
    int eeAddr = EEADR_CALIBRATION_START;
    int int_value;

    ee_getStr(sigStr, sizeof(CALIBRATION_SIG) + 1, eeAddr);
    if (strcmp((const char*)sigStr, CALIBRATION_SIG))
    {
        print("INFO no calibration data\n");
	   return;
    }

    eeAddr += sizeof(CALIBRATION_SIG) + 1;
    
    int_value = ee_getInt(eeAddr);
    memcpy(&pid_kp_, &int_value, 4);
    eeAddr += 4;  
    
    int_value = ee_getInt(eeAddr);
    memcpy(&pid_kd_, &int_value, 4);
    eeAddr += 4;  

    int_value = ee_getInt(eeAddr);
    memcpy(&pid_ki_, &int_value, 4);
    eeAddr += 4;  

    for (int i = 0; i < kNumberOfServos; i++)
    {
        servos_contexts_[i].deadband_positive_offset = ee_getByte(eeAddr);
        eeAddr += 1;        
        
        servos_contexts_[i].deadband_negative_offset = ee_getByte(eeAddr); 
        eeAddr += 1;    
    }        

    print("INFO calibration data loaded\n");
}    

void calibration_save()
{
    int eeAddr = EEADR_CALIBRATION_START;
    int int_value;

    ee_putStr((unsigned char*) CALIBRATION_SIG, sizeof(CALIBRATION_SIG) + 1, eeAddr);
    eeAddr += sizeof(CALIBRATION_SIG) + 1;

    memcpy(&int_value, &pid_kp_, 4);
    ee_putInt(int_value, eeAddr);
    eeAddr += 4;

    memcpy(&int_value, &pid_kd_, 4);
    ee_putInt(int_value, eeAddr);
    eeAddr += 4;
    
    memcpy(&int_value, &pid_ki_, 4);
    ee_putInt(int_value, eeAddr);
    eeAddr += 4;
    
    for (int i = 0; i < kNumberOfServos; i++)
    {
        ee_putByte(servos_contexts_[i].deadband_positive_offset, eeAddr);
        eeAddr += 1;

        ee_putByte(servos_contexts_[i].deadband_negative_offset, eeAddr);
        eeAddr += 1;
    }

    print("INFO calibration data saved\n");
}