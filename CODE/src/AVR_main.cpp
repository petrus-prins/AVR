/************************************************************************
*    AVR for 3phase generator - Petrus Prins 2022
*************************************************************************
* Description: AVR
* 
*     NOTES:
*     1. ATMEGA328P - (DIP28)
*     2. 
*
************************************************************************/


#include <Arduino.h>
#include <Statistic.h>      
#include <FireTimer.h>
#include <ArduPID.h>
#include <math.h>
#include <PID_Eeprom.h>
#include <SerialCommands.h>


//==========================ANALOG PINS============================================
int POT_AI_Pin    = PIN_A0;              // AT328P_PIN_23
int PHASE1_AI_Pin = PIN_A1;              // AT328P_PIN_24
int PHASE2_AI_Pin = PIN_A1;              // AT328P_PIN_25
int PHASE3_AI_Pin = PIN_A1;              // AT328P_PIN_26
//=================================================================================

//===============================DIGITAL PINS======================================
int RELAY1_PIN = PIN_A1;              // AT328P_PIN_
int RELAY2_PIN = PIN_A1;              // AT328P_PIN_
int RELAY3_PIN = PIN_A1;              // AT328P_PIN_
//=================================================================================

//=============================PWM PINS============================================
int PWM_Pin = PIN_PD5;                   // AT328P_PIN_11
int PWM_DUTY = 0;
//=================================================================================

//=====================================LEDs========================================
const int ledPin =  PIN_PB5;             // AT328P_PIN_19
int ledState = LOW;
//=================================================================================

//================================ATOD TIMER========================================
unsigned long ATOD_Prev_ms = 0;              
const long ATOD_Interval_ms = 200;
float VREF = 5.00;                       // Supply Voltage            
//=================================================================================

//===============================LCD===============================================
const int LCD_RX_Pin = PIN_PD2;
const int LCD_TX_Pin = PIN_PD3;
char LCD_LINE0[20];
char buff[20];
char SERIAL_BUFFER1[20];
char SERIAL_BUFFER2[20];
float fval;
//=================================================================================


//==========================240V STATS =============================================
float PHASE1_Voltage = 0;                     // Voltage
float PHASE2_Voltage = 0;
float PHASE3_Voltage = 0;
float RMS_Voltage = 0;

//PHASE1
statistic::Statistic<float, uint32_t, true> PHASE1_Raw;
statistic::Statistic<float, uint32_t, true> PHASE1_Min;
statistic::Statistic<float, uint32_t, true> PHASE1_Max;
//PHASE2
statistic::Statistic<float, uint32_t, true> PHASE2_Raw;
statistic::Statistic<float, uint32_t, true> PHASE2_Min;
statistic::Statistic<float, uint32_t, true> PHASE2_Max;
//PHASE3
statistic::Statistic<float, uint32_t, true> PHASE3_Raw;
statistic::Statistic<float, uint32_t, true> PHASE3_Min;
statistic::Statistic<float, uint32_t, true> PHASE3_Max;
//=================================================================================


//===================================FIRE TIMERS===================================
FireTimer PID_Timer;
unsigned long VOLTAGE_Prev_ms = 0;           
const ulong PID_Interval_ms = 250;            
//const long PID = 350;                         
//=================================================================================


//================================PID CONTROLLER===================================
ArduPID PID_AVR;
double  PID_input;
double  PID_output;
double  PID_setpoint = 220;
double  P = 10;                    
double  I = 1;
double  D = 0.5;
//=================================================================================


//==================================EEPROM=========================================
FireTimer EEPROM_Timer;
//=================================================================================

//=========================SERIAL COMMANDS=========================================
char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r", " ");     // "\r\n", " ");
bool DEBUG_ON = false;
//=================================================================================




//==========================================
//             SETUP PID       
//==========================================
void Setup_PID()
{
    PID_AVR.begin(&PID_input, &PID_output, &PID_setpoint, P, I, D);
    PID_AVR.setOutputLimits(0, 255);
    PID_AVR.setBias(255.0 / 2.0);
    PID_AVR.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
    PID_AVR.start();

  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();
    PID_AVR.reverse();               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
}



//==========================================
//             SETUP FireTimers      
//==========================================
void Setup_FireTimer()
{
    PID_Timer.begin(PID_Interval_ms);
}





//==========================================
//             PROCESS ATOD      
//==========================================
void Process_ATOD()
{
    unsigned long Current_ms = millis();
    if (Current_ms - ATOD_Prev_ms >= ATOD_Interval_ms) 
    {
        ATOD_Prev_ms = Current_ms;
        fval = analogRead(POT_AI_Pin)*5.0/1024;
        dtostrf(fval,4,2,buff);
        PWM_DUTY = fval*51;         // 5V->255.    5*51 = 255.
        dtostrf(PWM_DUTY,3,0,buff);
        analogWrite(PWM_Pin,PWM_DUTY);
    }
}



 void Capture_Manual_Setpoint()
 {
     PID_setpoint = analogRead(POT_AI_Pin);
 }






//==========================================
//             PROCESS LED STATES      
//==========================================
void Process_LEDS()
{
    if (ledState == LOW) 
    {
        ledState = HIGH;
         Serial.println("LED: ON");
    }
    else 
    {
        ledState = LOW;
        Serial.println("LED: OFF");
    }
    digitalWrite(ledPin, ledState);
   
}





/*
 float Calculate_Phase_Voltage(float ADC_Max, float ADC_Min)

 Description:
 ------------
    Calculate line voltage based on MAX and MIN ADC values.
    ADC has constant DC offset with AC ponent on top.
    Mesurement shows that 210 ADC points == 236V.  

 Inputs:
 -------
    float ADC_Max, float ADC_Min

 Return:
 -------
    float - correctly scaled AC voltage in VOLTS.
*/

//===============================================================================
//             CALCUALTE PHASE VOLTAGE
//             Calculate AC LN voltage based on max and min ADC values.
//             ADCmin-ADCmax =  210 ADC points = 236V.  
//             ADCmin-ADCmax =  182 ADC points = 226V.
//===============================================================================
float Calculate_Phase_Voltage(float ADC_Max, float ADC_Min)
{
    float Volt_Error_Offset = 5.0;
    float fval = 0;
    fval = fabs(ADC_Max-ADC_Min);           //  ADC difference in max and min
    fval = fval * VREF;                     // * 5.05V
    fval = fval / 1024.0;                   // convert ADC to Volts So AC part of signal p-p voltage. 226VAC is equal to ==> 0.9VADC
    fval = fval * 240;                      // Constant to scale ADCV to ACV
    fval = fval-Volt_Error_Offset;
    if (fval < 1.0) {fval = 0;}
    return fval;
}



int Read_ADC_Average(int ADC_PIN, int num_samples = 1)
{
    int local_ADC = 0;
    for (int i=1; i <= num_samples; i++)
    {
        local_ADC += analogRead(ADC_PIN);
    }
    local_ADC = local_ADC/num_samples;
    return local_ADC;  
}



void READ_PHASE_Voltage_Stats()
{
    PHASE1_Voltage = Read_ADC_Average(PHASE1_AI_Pin,1);
    PHASE2_Voltage = Read_ADC_Average(PHASE2_AI_Pin,1);
    PHASE3_Voltage = Read_ADC_Average(PHASE3_AI_Pin,1);




   // PHASE1_Voltage = (analogRead(PHASE1_AI_Pin) + analogRead(PHASE1_AI_Pin) + analogRead(PHASE1_AI_Pin))/3;
  //  PHASE2_Voltage = analogRead(PHASE1_AI_Pin);
  //  PHASE3_Voltage = analogRead(PHASE1_AI_Pin);
 
    PHASE1_Raw.add(PHASE1_Voltage);
    PHASE1_Min.add(PHASE1_Raw.minimum());
    PHASE1_Max.add(PHASE1_Raw.maximum());

    PHASE2_Raw.add(PHASE2_Voltage);
    PHASE2_Min.add(PHASE2_Raw.minimum());
    PHASE2_Max.add(PHASE2_Raw.maximum());

    PHASE3_Raw.add(PHASE3_Voltage);
    PHASE3_Min.add(PHASE3_Raw.minimum());
    PHASE3_Max.add(PHASE3_Raw.maximum());
}


float CALCULATE_RMS_Voltage_From_Stats()
{
    float PHASE1 = Calculate_Phase_Voltage(PHASE1_Max.average(), PHASE1_Min.average());
    float PHASE2 = Calculate_Phase_Voltage(PHASE2_Max.average(), PHASE2_Min.average());
    float PHASE3 = Calculate_Phase_Voltage(PHASE3_Max.average(), PHASE3_Min.average());

   //-------------------
        strcpy(SERIAL_BUFFER1,"");
        strcat(SERIAL_BUFFER1, "[P1: ");
        dtostrf(PHASE1,3,0,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "V] ");
        //Serial.print(SERIAL_BUFFER1);

        strcpy(SERIAL_BUFFER1,"");
        strcat(SERIAL_BUFFER1, "[P2: ");
        dtostrf(PHASE2,3,0,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "V] ");
        //Serial.print(SERIAL_BUFFER1); 

        strcpy(SERIAL_BUFFER1,"");
        strcat(SERIAL_BUFFER1, "[P3: ");
        dtostrf(PHASE3,3,0,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "V] ");
        //Serial.print(SERIAL_BUFFER1); 

        strcpy(SERIAL_BUFFER1,"");
        strcat(SERIAL_BUFFER1, "[Pin: ");
        dtostrf(PID_input,3,0,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "] ");
        //Serial.print(SERIAL_BUFFER1); 
     
        strcpy(SERIAL_BUFFER1,"");
        strcat(SERIAL_BUFFER1, "[Pout: ");
        dtostrf(PID_output,3,0,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "] ");
        //Serial.print(SERIAL_BUFFER1); 
     
        strcpy(SERIAL_BUFFER1,"");
        strcat(SERIAL_BUFFER1, "[SP: ");
        dtostrf(PID_setpoint,3,2,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "]");
        //Serial.println(SERIAL_BUFFER1); 

      /* PID_AVR.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);
                                              */


    //------------------
    float RMS = sqrt(square(PHASE1)+square(PHASE2)+square(PHASE3));   // 380V
    return RMS/sqrt(3);    // 220V
}



void CLEAR_PHASE_Voltage_Stats()
{
        PHASE1_Raw.clear();
        PHASE1_Min.clear();
        PHASE1_Max.clear();

        PHASE2_Raw.clear();
        PHASE2_Min.clear();
        PHASE2_Max.clear();

        PHASE3_Raw.clear();
        PHASE3_Min.clear();
        PHASE3_Max.clear();
}



//===============================================================================
//             PROCESS VOLTAGE
//===============================================================================
void Process_VOLTAGE()
{
    
    uint32_t  Sample_count = 0;
    
    READ_PHASE_Voltage_Stats();

    if (PID_Timer.fire())
    {
        RMS_Voltage = CALCULATE_RMS_Voltage_From_Stats();
        Sample_count = PHASE1_Raw.count();
        //Sample_count = PHASE1_Raw.count();
   


        // ====================== PID TEST SECTION========================
             //PID_setpoint = 240;
             PID_input = RMS_Voltage;           //= analogRead(A0); // Replace with sensor feedback
             PID_AVR.compute();

             //analogWrite(3, output); // Replace with plant control signal
        //=================================================================



    

 
         //LCD        
        dtostrf(RMS_Voltage,3,0,LCD_LINE0);

        dtostrf(Sample_count,4,0,LCD_LINE0);
        //---------------------------------------------------------------



        CLEAR_PHASE_Voltage_Stats();
        Process_LEDS();

    }
}






void Process_EEPROM()
{
    if (EEPROM_Timer.fire())
    {
        //eedump();
    
        strcpy(SERIAL_BUFFER1," [");
        strcat(SERIAL_BUFFER1, "P:=");
        dtostrf(P,4,1,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "] ");
       // Serial.print(SERIAL_BUFFER1);

        strcpy(SERIAL_BUFFER1," [");
        strcat(SERIAL_BUFFER1, "I:=");
        dtostrf(I,4,1,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "] ");
       // Serial.print(SERIAL_BUFFER1);

        strcpy(SERIAL_BUFFER1," [");
        strcat(SERIAL_BUFFER1, "D:=");
        dtostrf(D,4,1,SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, SERIAL_BUFFER2);
        strcat(SERIAL_BUFFER1, "] ");
       // Serial.println(SERIAL_BUFFER1);

    }
}






void Process_Scratch_Pad()
{
    float t = millis()/1000.0; // get program time
    Serial.print(t);           // output time in seconds as first variable
    Serial.print(" ");         // add spacing between variables
    
    float var_sin = RMS_Voltage;
    Serial.print(var_sin);     // output sin(t) variable
    Serial.print(" ");      // add spacing between variables


    Serial.print(PHASE1_Max.average());     // output sin(t) variable
    Serial.print(" ");      // add spacing between variables



    Serial.print(PHASE1_Min.average());     // output sin(t) variable
    Serial.print(" ");      // add spacing between variables


  //  float var_cos = cos(t); 
  // Serial.print(var_cos);     // output cos(t) variable
    Serial.println();       // this just prints a \n character if you don't provide an argument

    // at 2.5 seconds, this prints out like so, where numbers
    // are separated by spaces, and \n is the newline character
    // 2500 0.598 -0.801\n
}








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


SerialCommand CMD_Debug_("DEBUG", CMD_Debug);


//==================================================================
//                   SETUP SERIAL COMMANDS
//==================================================================
void Setup_Serial_Commands()
{
    serial_commands_.SetDefaultHandler(CMD_Unknown);
	serial_commands_.AddCommand(&CMD_Debug_);
    Serial.println("Ready!");
}






//==================================================================
//                   EEPROM REOUTINES
//==================================================================
void Setup_EEPROM()
{
    EEPROM_Timer.begin(1000);
    //writetoEEPROM(1.0,2.0,3.0);
    //writetoEEPROM(P,I,D);
    recoverPIDfromEEPROM(P,I,D);
   



    //eedump();
}









//====================================
//              SETUP       
//====================================
void setup() 
{
  Serial.begin(115200);
  Serial.println("[CPU RESET DETECTED]");
  pinMode(ledPin, OUTPUT);
  pinMode(PWM_Pin, OUTPUT);
  Setup_EEPROM();
  Setup_FireTimer();
  Setup_PID();
  Setup_Serial_Commands();
}

//====================================
//              MAIN LOOP       
//====================================
void loop() 
{
   //Process_LEDS();
  // Process_ATOD();
   Capture_Manual_Setpoint();
   Process_VOLTAGE();
   Process_EEPROM();
   Process_Serial_Commands();
   Process_Scratch_Pad();
}













