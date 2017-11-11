//#define SERIAL_COMMANDS_DEBUG
#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

//#include <propeller.h>
#include "fdserial.h"

class SerialCommands;

typedef class SerialCommand SerialCommand;
class SerialCommand
{
public:
	SerialCommand(const char* cmd, void(*func)(SerialCommands*))
		: command(cmd),
		function(func),
		next(NULL)
	{
	}

	const char* command;
	void(*function)(SerialCommands*);
	SerialCommand* next;
};

class SerialCommands
{
public:
	SerialCommands(char* buffer, int16_t buffer_len, char term) :
		buffer_(buffer),
		buffer_len_(buffer_len - 1), //string termination char '\0'
		term_(term),
		default_handler_(NULL),
		//delim_{ ' ', 0 },
		buffer_pos_(0),
		last_token_(NULL),
		serial_(NULL),
		commands_head_(NULL),
		commands_tail_(NULL),
		commands_count_(0)
	{
     delim_[0]=' ';
     delim_[1]=0;
	}

	void AddCommand(SerialCommand* command);
	int8_t ReadSerial();
	fdserial * GetSerial();
	void AttachSerial(fdserial * serial);
	void DetachSerial();
	void SetDefaultHandler(void(*function)(SerialCommands*, const char*));
	void ClearBuffer();
	char* Next();

private:
	char* buffer_;
	int16_t buffer_len_;
	uint8_t term_;
	void(*default_handler_)(SerialCommands*, const char*);
	char delim_[2];
	int16_t buffer_pos_;
	char* last_token_;
	fdserial * serial_;
	SerialCommand* commands_head_;
	SerialCommand* commands_tail_;
	uint8_t commands_count_;
};

#endif
