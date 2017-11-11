#include "global.h"

void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
	dprint(sender->GetSerial(), MSG_INVALID_COMMAND);
}

void cmd_id(SerialCommands* sender)
{
	dprint(sender->GetSerial(), WHO_AM_I);
}

SerialCommand cmd_id_(ID, cmd_id);

void cmd_set_angle(SerialCommands* sender)
{
	int id;
	int angle;
	char* arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
	id = atoi(arg);
	if (id < 0 || id >= kNumberOfServos)
	{
		return;
	}
	arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
	angle = atoi(arg);
	ServoContext* ctx = &servos_contexts_[id];

	ctx->pid_setpoint = angle;
	ctx->move = 1;

	//while (lockset(usb_serial_lock_id_));
	dprint(sender->GetSerial(), "SET_ANGLE %d %d\r\n", id, angle);
	//lockclr(usb_serial_lock_id_);
}

SerialCommand cmd_set_angle_(ANGLE, cmd_set_angle);

void cmd_set_speed(SerialCommands* sender)
{
	int id;
	char* arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
	id = atoi(arg);
	if (id < 0 || id >= kNumberOfServos)
	{
		return;
	}
	arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
     int speed = atoi(arg);
	ServoContext* ctx = &servos_contexts_[id];
     servo_speed(ctx->control_pin, speed);
	dprint(sender->GetSerial(), "SET_SPEED %d %d\r\n", id, speed);
}

SerialCommand cmd_set_speed_(SPEED, cmd_set_speed);

void cmd_set_pid(SerialCommands* sender)
{
	int params_count = 0;

	dprint(sender->GetSerial(), "PID %f %f %f\r\n", pid_kp_, pid_kd_, pid_ki_);

	char* arg = sender->Next();
	if (arg != NULL)
	{
		pid_kp_ = atof(arg);
		params_count++;
	}

	arg = sender->Next();
	if (arg != NULL)
	{
		pid_kd_ = atof(arg);
		params_count++;
	}

	arg = sender->Next();
	if (arg != NULL)
	{
		pid_ki_ = atof(arg);
		params_count++;
	}

	if (params_count > 0)
	{
          for (int i = 0; i < kNumberOfServos; i++)
          {
              servos_contexts_[i].pid.SetTunings(pid_kp_, pid_ki_, pid_kd_);
          }
          calibration_save();
          
	     dprint(sender->GetSerial(), "SET_PID %f %f %f\r\n", pid_kp_, pid_kd_, pid_ki_);
	}
}

SerialCommand cmd_set_pid_(PID, cmd_set_pid);


void serial_setup()
{
    	//cog1
	host_serial_ = fdserial_open(HOST_RX_PIN, HOST_TX_PIN, 0, 115200);
	host_serial_commands_.AttachSerial(host_serial_);
	host_serial_commands_.SetDefaultHandler(cmd_unrecognized);
	host_serial_commands_.AddCommand(&cmd_id_);
	host_serial_commands_.AddCommand(&cmd_set_angle_);
 	host_serial_commands_.AddCommand(&cmd_set_speed_);
   	host_serial_commands_.AddCommand(&cmd_set_pid_);
}    

