#include <propeller.h>
//#include <stdio.h>
//#include <string.h>
#include "SerialCommands.h"
#include "simpletools.h" 

#define WHO_AM_I                      "PARALLAX 360 SERVO CONTROLLER (PROPELLER)\r\n"
#define MSG_INVALID_COMMAND           "INVALID COMMAND\r\n"
#define SERIAL_COMMANDS_BUFFER_SIZE   64
#define HOST_RX_PIN                   23
#define HOST_TX_PIN                   22
#define ID                            "id"

static char serial_command_buffer_[SERIAL_COMMANDS_BUFFER_SIZE];
fdserial *host_serial_;
SerialCommands host_serial_commands_(serial_command_buffer_, sizeof(serial_command_buffer_), '\n');


void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
   dprint(sender->GetSerial(), MSG_INVALID_COMMAND);
}

void cmd_id(SerialCommands* sender)
{
	dprint(sender->GetSerial(), WHO_AM_I);
}

SerialCommand cmd_id_(ID, cmd_id);


int main()
{
  float maxSecs = 0xffffffff;
  maxSecs /= CLKFREQ;
  print("INFO: main cogId=%d CLKFREQ=%d ms=%d us=%d maxSecs=%f\r\n", cogid(), CLKFREQ, ms, us, maxSecs);

  
  host_serial_ = fdserial_open(HOST_RX_PIN, HOST_TX_PIN, 0, 115200);
  host_serial_commands_.AttachSerial(host_serial_);
  host_serial_commands_.SetDefaultHandler(cmd_unrecognized);
  host_serial_commands_.AddCommand(&cmd_id_);
  int seq = 0;
  
  for (;;)
  {
    host_serial_commands_.ReadSerial();
    
    //dprint(host_serial_, "hello %d\n", seq++);
    //pause(1000);
  }    

  return 0;
}
