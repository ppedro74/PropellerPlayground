#include "global.h"

static int cog_servo_feedback_stack_[kNumberOfServos * kCogServoFeedbackStackSize];
volatile static unsigned long millis_ = 0;
ServoContext servos_contexts_[kNumberOfServos];

char serial_command_buffer_[SERIAL_COMMANDS_BUFFER_SIZE];
SerialCommands host_serial_commands_(serial_command_buffer_, sizeof(serial_command_buffer_), '\n');
fdserial* host_serial_;
//fdserial *usb_serial_;
//volatile unsigned int usb_serial_lock_id_;
volatile int seq_;
double pid_kp_ = 1.0;
double pid_kd_ = 0.0;
double pid_ki_ = 0.0;


unsigned long millis()
{
	return millis_;
}

void servo_feedback(void *par)
{
	ServoContext* ctx = (ServoContext*)par;

	ctx->turns = 0;

	ctx->low_cycle_length = pulse_in(ctx->feedback_pin, 0);
	ctx->high_cycle_length = pulse_in(ctx->feedback_pin, 1);
	ctx->total_cycle_length = ctx->high_cycle_length + ctx->low_cycle_length;

	ctx->duty_cycle = (kDutyScale * ctx->high_cycle_length) / ctx->total_cycle_length;
	ctx->theta = ((ctx->duty_cycle - kDutyHighCycleMin) * kUnitsPerCycle) / kDutyCycleLength;

	ctx->theta = (kUnitsPerCycle - 1) - ctx->theta;
	ctx->theta_prev = ctx->theta;

	while (1)
	{
		// Measure high and low times, making sure to only take valid cycle
		// times (a high and a low on opposite sides of the 0/359 boundary
		// will not be valid.

		ctx->total_cycle_length = 0;
		while (1)
		{
			ctx->high_cycle_length = pulse_in(ctx->feedback_pin, 1);
			ctx->low_cycle_length = pulse_in(ctx->feedback_pin, 0);
			ctx->total_cycle_length = ctx->high_cycle_length + ctx->low_cycle_length;

			// If cycle time valid 
			if (ctx->total_cycle_length > 1000 && ctx->total_cycle_length < 1200)
			{
				// break from loop
				break;
			}
		}
		ctx->duty_cycle = (kDutyScale * ctx->high_cycle_length) / ctx->total_cycle_length;
		ctx->theta = ((ctx->duty_cycle - kDutyHighCycleMin) * kUnitsPerCycle) / kDutyCycleLength;

		//This gives a theta increasing int the counterclockwise direction.
		ctx->theta = (kUnitsPerCycle - 1) - ctx->theta;

		//Keep theta valid
		if (ctx->theta < 0)
		{
			ctx->theta = 0;
		}
		else if (ctx->theta > (kUnitsPerCycle - 1))
		{
			ctx->theta = kUnitsPerCycle - 1;
		}

		if ((ctx->theta < kQ2Min) && (ctx->theta_prev > kQ3Max))
		{
			//If transition from quadrant 4 to quadrant 1, increase turns count. 
			ctx->turns++;
		}
		else if ((ctx->theta_prev < kQ2Min) && (ctx->theta > kQ3Max))
		{
			//If transition from quadrant 1 to quadrant 4, decrease turns count. 
			ctx->turns--;
		}

		//Construct the angle measurement from the turns count and current theta value.
		if (ctx->turns >= 0)
		{
			ctx->angle = (ctx->turns * kUnitsPerCycle) + ctx->theta;
		}
		else if (ctx->turns < 0)
		{
			ctx->angle = ((ctx->turns + 1) * kUnitsPerCycle) - (kUnitsPerCycle - ctx->theta);
		}

          ctx->pid_input = ctx->angle;
          
          if (ctx->move == 1 && abs(ctx->pid_input - ctx->pid_setpoint)<2)
          {
              ctx->move == 0;
              servo_speed(ctx->control_pin, 0);
          }              
          
		//ctx->angle = theta;

		// Theta previous for next rep
		ctx->theta_prev = ctx->theta;
	}
}

//void dummy(void *par)
//{
//	ServoContext* ctx = (ServoContext*)par;
//
//	//while (lockset(usb_serial_lock_id_));
//	dprint(usb_serial_, "INFO dummy cogId=%d id=%d ctl=%d feedback=%d\n", cogid(), ctx->id, ctx->control_pin, ctx->feedback_pin);
//	//dprint(usb_serial_, "INFO dummy cogId=%d\n", cogid());
//	//lockclr(usb_serial_lock_id_);
//
//	for (;;)
//	{
//	}
//}

void print_angles()
{
	print("ANGLES seq=%d", seq_);
	for (int i = 0; i < kNumberOfServos; i++)
	{
		print(" %d", servos_contexts_[i].angle);
	}
	print("\n");
}


void print_angles(fdserial *serial)
{
	dprint(serial, "ANGLES %d", seq_);
	for (int i = 0; i < kNumberOfServos; i++)
	{
		dprint(serial, " %d", servos_contexts_[i].angle);
	}
	dprint(serial, "\n");
}

void check_servo_pid(ServoContext* ctx)
{
	if (ctx->move == 0)
	{
		return;
	}

	if (ctx->pid.Compute() == false)
	{
		return;
	}

	//add deadband offset
	int offset = 0;
	if (ctx->pid_output > 0)
	{
		offset = ctx->deadband_positive_offset;
	}
	else if (ctx->pid_output < 0)
	{
		offset = -ctx->deadband_negative_offset;
	}

	servo_speed(ctx->control_pin, ctx->pid_output + offset);
}

int main()
{
	//usb_serial_lock_id_ = locknew();
	//simpleterm_close();
	//usb_serial_ = fdserial_open(USB_RX_PIN, USB_TX_PIN, 0, BAUDRATE);

	float maxSecs = 0xffffffff;
	maxSecs /= CLKFREQ;

	print("INFO main cogId=%d CLKFREQ=%d ms=%d us=%d maxSecs=%f\n", cogid(), CLKFREQ, ms, us, maxSecs);

     serial_setup();
     
	//cog2: servo control cog
	for (int i = 0; i < kNumberOfServos; i++)
	{
		ServoContext* ctx = &servos_contexts_[i];
		ctx->id = i;
		ctx->control_pin = SERVO0_CONTROL_PIN + (i * 2);
		ctx->feedback_pin = SERVO0_FEEDBACK_PIN + (i * 2);

		ctx->move = 0;
		ctx->pid.SetTunings(pid_kp_, pid_ki_, pid_kd_);
		ctx->pid.SetMode(AUTOMATIC);
		ctx->pid.SetSampleTime(PID_MS);
		ctx->pid.SetOutputLimits(-200, 200);

		servo_speed(ctx->control_pin, 0);
	}

	//cog3, cog4, cog5, cog6
	for (int i = 0; i < kNumberOfServos; i++)
	{
		ServoContext* ctx = &servos_contexts_[i];

		print("INFO servo id=%d ctl=%d feedback=%d\n", ctx->id, ctx->control_pin, ctx->feedback_pin);

		cogstart(&servo_feedback, ctx, &cog_servo_feedback_stack_[i * kCogServoFeedbackStackSize], kCogServoFeedbackStackSize - 1);

		pause(150);
	}

	pause(500);

     calibration_load();
     bool changes = false;   
	for (int i = 0; i < kNumberOfServos; i++)
	{
		ServoContext* ctx = &servos_contexts_[i];
          changes |= check_servo_calibration(ctx);  
	}
    
     if (changes)
     {
         calibration_save();
     }         

	pause(500);

	print("INFO Ready!\n");

	const int dt = ms;
	int t = CNT;

	int print_angle_next_millis = millis_;
	int check_target_angle_next_millis = millis_;

	while (1)
	{
		host_serial_commands_.ReadSerial();

		if (CNT - t > dt)
		{
			t += dt;
			millis_++;
		}

		if (millis_ > print_angle_next_millis)
		{
			print_angle_next_millis += 1000;

			print_angles(host_serial_);
			print_angles();

			seq_++;
		}

          for (int i = 0; i < kNumberOfServos; i++)
          {
              check_servo_pid(&servos_contexts_[i]);
          }
	}

	return 0;
}

