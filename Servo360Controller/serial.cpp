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
     int tran_id;
	int id;
	int angle;
	
     char* arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
 	tran_id = atoi(arg);
  
     arg = sender->Next();
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
     high(LED0_PIN + ctx->id); 

	dprint(sender->GetSerial(), "ACK %d\r\n", tran_id);
}

SerialCommand cmd_set_angle_(SET_ANGLE, cmd_set_angle);

void cmd_set_speed(SerialCommands* sender)
{
     int tran_id;
	int id;
 
     char* arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
 	tran_id = atoi(arg);
  
	arg = sender->Next();
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
	dprint(sender->GetSerial(), "ACK %d\r\n", tran_id);
}

SerialCommand cmd_set_speed_(SET_SPEED, cmd_set_speed);

void cmd_get_pid(SerialCommands* sender)
{
     int tran_id;
 
     char* arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
 	tran_id = atoi(arg);
     
	dprint(sender->GetSerial(), "PID %d %f %f %f\r\n", tran_id, pid_kp_, pid_kd_, pid_ki_);
}

SerialCommand cmd_get_pid_(GET_PID, cmd_get_pid);

void cmd_set_pid(SerialCommands* sender)
{
     int tran_id;
	int params_count = 0;

     char* arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
 	tran_id = atoi(arg);
 
	arg = sender->Next();
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
	}

     dprint(sender->GetSerial(), "ACK %d\r\n", tran_id, params_count);
}

SerialCommand cmd_set_pid_(SET_PID, cmd_set_pid);

void cmd_get_servo_config(SerialCommands* sender)
{
     int tran_id;
	int id;
 
     char* arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
 	tran_id = atoi(arg);
  
	arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
	id = atoi(arg);
 	if (id < 0 || id >= kNumberOfServos)
	{
		return;
	}

	ServoContext* ctx = &servos_contexts_[id];
    
	dprint(sender->GetSerial(), "SERVO %d %d %d %d\r\n", tran_id, id, ctx->deadband_positive_offset, ctx->deadband_negative_offset);
}

SerialCommand cmd_get_servo_config_(GET_SERVO_CONFIG, cmd_get_servo_config);


void cmd_set_servo_config(SerialCommands* sender)
{
     int params_count=0;
     int tran_id;
	int id;
 
     char* arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
 	tran_id = atoi(arg);
  
	arg = sender->Next();
	if (arg == NULL)
	{
		return;
	}
	id = atoi(arg);
 	if (id < 0 || id >= kNumberOfServos)
	{
		return;
	}
	ServoContext* ctx = &servos_contexts_[id];

	arg = sender->Next();
	if (arg != NULL)
	{
        params_count++;
        ctx->deadband_positive_offset = atoi(arg);
	}

	arg = sender->Next();
	if (arg != NULL)
	{
        params_count++;
        ctx->deadband_negative_offset = atoi(arg);
	}

     if (params_count > 0)
	{
          calibration_save();
     }          

     dprint(sender->GetSerial(), "ACK %d %d\r\n", tran_id, params_count);     
}

SerialCommand cmd_set_servo_config_(SET_SERVO_CONFIG, cmd_set_servo_config);

void serial_setup()
{
    	//cog1
	host_serial_ = fdserial_open(HOST_RX_PIN, HOST_TX_PIN, 0, 115200);
	host_serial_commands_.AttachSerial(host_serial_);
	host_serial_commands_.SetDefaultHandler(cmd_unrecognized);
	host_serial_commands_.AddCommand(&cmd_id_);
	host_serial_commands_.AddCommand(&cmd_set_angle_);
 	host_serial_commands_.AddCommand(&cmd_set_speed_);
   	host_serial_commands_.AddCommand(&cmd_get_pid_);
   	host_serial_commands_.AddCommand(&cmd_set_pid_);
	host_serial_commands_.AddCommand(&cmd_get_servo_config_);   
	host_serial_commands_.AddCommand(&cmd_set_servo_config_);   
}    

