#include <SerialCommands.h>
#include <AVR_main.h>




//==================SERIAL COMMAND VARIABLES========================
char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r", " ");     // "\r\n", " ");
bool DEBUG_ON = false;
//==================================================================




//==================================================================
//                  SERIAL CMD - UNKNOWN COMMAND
//==================================================================
void CMD_Unknown(SerialCommands* sender, const char* cmd)
{
	sender->GetSerial()->print("Unknown Command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}



//==================================================================
//                   SERIAL CMD - CMD_DEBUG
//==================================================================
void CMD_Relay(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter
    char Error_MSG[] = "USAGE: RELAY [1-3] [ON | OFF]";
	char* Relay_No = sender->Next();
    char* Relay_Action = sender->Next();

	if ((Relay_No == NULL) || (Relay_Action == NULL))
	{
		sender->GetSerial()->println(Error_MSG);
		return;
	}
	
	
    // REALAY 1
    if (strcmp(Relay_No, "1") == 0)
	{
        if (strcmp(Relay_Action, "ON") == 0)    
        {   
            SET_RELAY1(true);
            sender->GetSerial()->println("RELAY 1 Is ON");
        }
        else if (strcmp(Relay_Action, "OFF") == 0)  
        {   
            SET_RELAY1(false);
            sender->GetSerial()->println("RELAY 1 Is OFF");
        }
        else    
        {
         	sender->GetSerial()->println(Error_MSG);
		    return;   
        }
    }
    // REALAY 2
    else if (strcmp(Relay_No, "2") == 0)
	{
        if (strcmp(Relay_Action, "ON") == 0)    
        {   
            SET_RELAY2(true);
            sender->GetSerial()->println("RELAY 2 Is ON");
        }
        else if (strcmp(Relay_Action, "OFF") == 0)  
        {   
            SET_RELAY2(false);
            sender->GetSerial()->println("RELAY 2 Is OFF");
        }
        else    
        {
         	sender->GetSerial()->println(Error_MSG);
		    return;   
        }
    }
    else if (strcmp(Relay_No, "3") == 0)
	{
        if (strcmp(Relay_Action, "ON") == 0)    
        {   
            SET_RELAY3(true);
            sender->GetSerial()->println("RELAY 3 Is ON");
        }
        else if (strcmp(Relay_Action, "OFF") == 0)  
        {   
            SET_RELAY3(false);
            sender->GetSerial()->println("RELAY 3 Is OFF");
        }
        else    
        {
         	sender->GetSerial()->println(Error_MSG);
		    return;   
        }
    }
    else
    {
		sender->GetSerial()->println(Error_MSG);
		return;
    }
}







//==================================================================
//                   SERIAL CMD - CMD_DEBUG
//==================================================================
void CMD_Debug(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter

	char* Action = sender->Next();
	if (Action == NULL)
	{
		sender->GetSerial()->println("Please Specify 'DEBUG ON' or 'DEBUG OFF'");
		return;
	}
	
	if (strcmp(Action, "ON") == 0)
	{
		DEBUG_ON = true;
        sender->GetSerial()->println("DEBUG is ON");   
	}
	else if (strcmp(Action, "OFF") == 0)
	{
		DEBUG_ON = false;
        sender->GetSerial()->println("DEBUG is OFF");
	}
    else
    {
		sender->GetSerial()->println("Please Specify 'DEBUG ON' or 'DEBUG OFF'");
		return;
    }
}





//==================================================================
//                   PROCESS SERIAL COMMANDS
//==================================================================
void Process_Serial_Commands()
{
    serial_commands_.ReadSerial();
}


SerialCommand SCMD_Debug_("DEBUG", CMD_Debug);
SerialCommand SCMD_Relay_("RELAY", CMD_Relay);



//==================================================================
//                   SETUP SERIAL COMMANDS
//==================================================================
void Setup_Serial_Commands()
{
    serial_commands_.SetDefaultHandler(CMD_Unknown);
	serial_commands_.AddCommand(&SCMD_Debug_);
    serial_commands_.AddCommand(&SCMD_Relay_);
    Serial.println("Ready!");
}

