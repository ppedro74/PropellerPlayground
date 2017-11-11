//#include <stdio.h>
//#include <string.h>
#include <propeller.h>
#include "strtok_r.h"
#include <ctype.h>
#include "SerialCommands.h"

void SerialCommands::AddCommand(SerialCommand* command)
{
#ifdef SERIAL_COMMANDS_DEBUG
	print("Adding #%d cmd=[%s]\r\n", commands_count_, command->command);
#endif
	command->next = NULL;
	if (commands_head_ == NULL)
	{
		commands_head_ = commands_tail_ = command;
	}
	else
	{
		commands_tail_->next = command;
		commands_tail_ = command;
	}
	commands_count_++;
}

int8_t SerialCommands::ReadSerial()
{
	if (serial_ == NULL)
	{
		return -1;
	}

	while (fdserial_rxReady(serial_) > 0)
	{
		char ch = fdserial_rxChar(serial_);
#ifdef SERIAL_COMMANDS_DEBUG
		print(ch); // Echo back to serial stream
#endif

		if (ch == term_)
		{
#ifdef SERIAL_COMMANDS_DEBUG
			print("Received: %s\r\n", buffer_);
#endif
			char* command = strtok_r(buffer_, delim_, &last_token_);
			if (command != NULL)
			{
				bool matched = false;
				int cx;
				SerialCommand* cmd;
				for (cmd = commands_head_, cx = 0; cmd != NULL; cmd = cmd->next, cx++)
				{
#ifdef SERIAL_COMMANDS_DEBUG
					print("Comparing [%s] to [%s]\r\n", command, cmd->command);
#endif

					if (strncmp(command, cmd->command, strlen(cmd->command) + 1) == 0)
					{
#ifdef SERIAL_COMMANDS_DEBUG
						print("Matched #%d\r\n", cx);
#endif
						cmd->function(this);
						matched = true;
						break;
					}
				}
				if (!matched && default_handler_ != NULL)
				{
					(*default_handler_)(this, command);
				}
			}

			ClearBuffer();
		}
		else if (isprint(ch))
		{
			if (buffer_pos_ < buffer_len_)
			{
				buffer_[buffer_pos_++] = ch;
				buffer_[buffer_pos_] = '\0';
			}
			else
			{
#ifdef SERIAL_COMMANDS_DEBUG			
				print("Warning: Buffer full\r\n");
#endif
				return -2;
			}
		}
	}

	return 0;
}

fdserial * SerialCommands::GetSerial()
{
	return serial_;
}

void SerialCommands::AttachSerial(fdserial * serial)
{
	serial_ = serial;
}

void SerialCommands::DetachSerial()
{
	serial_ = NULL;
}

void SerialCommands::SetDefaultHandler(void(*function)(SerialCommands*, const char*))
{
	default_handler_ = function;
}

void SerialCommands::ClearBuffer()
{
	buffer_[0] = '\0';
	buffer_pos_ = 0;
}

char* SerialCommands::Next()
{
	return strtok_r(NULL, delim_, &last_token_);
}
