#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdio.h>            // Recommended over iostream for saving space          
#include <propeller.h>        // Propeller-specific functions
#include <simpletools.h>
#include <fdserial.h>  
#include <simpletools.h>
#include <servo.h>
#include <serialcommands.h>
#include <pid.h>

#include "ServoContext.h"

//EEPROM Parallax Addresses
//#define _abvolts_EE_start_          63400
//#define _AB360_EE_Start_            63418
#define EEADR_CALIBRATION_START       32768


#define BAUDRATE                      115200
#define SERIAL_COMMANDS_BUFFER_SIZE   64
#define HOST_TX_PIN                   22
#define HOST_RX_PIN                   23
#define USB_TX_PIN                    30
#define USB_RX_PIN                    31

#define SERVO0_CONTROL_PIN            8
#define SERVO0_FEEDBACK_PIN           9

#define WHO_AM_I                      "PARALLAX 360 SERVO CONTROLLER (PROPELLER)\r\n"
#define MSG_INVALID_COMMAND           "INVALID COMMAND\r\n"
#define ID                            "id"
#define ANGLE                         "a"
#define SPEED                         "s"
#define PID                           "pid"
#define PID_MS                        10

const int kDutyScale = 1000;

const int kUnitsPerCycle = 360;
const int kQ2Min = kUnitsPerCycle / 4;
const int kQ3Max = kUnitsPerCycle / 4 * 3;

const int kDutyHighCycleMin = 29;
const int kDutyHighCycleMax = 971;
const int kDutyCycleLength = 971 - 29; //kDutyHighCycleMax - kDutyHighCycleMin + 1;

const int kNumberOfServos = 4;
//const int kServoControlPins[kNumberOfServos] = { 0, 2, 4, 6};
//const int kServoFeedbackPins[kNumberOfServos] = { 1, 3, 5, 7};
const int kCogServoFeedbackStackSize = 300;

extern ServoContext servos_contexts_[];
extern fdserial* host_serial_;
extern SerialCommands host_serial_commands_;
extern double pid_kp_;
extern double pid_kd_;
extern double pid_ki_;


unsigned long millis();
void serial_setup();
bool check_servo_calibration(ServoContext* ctx);
void calibration_save();
void calibration_load();



#endif