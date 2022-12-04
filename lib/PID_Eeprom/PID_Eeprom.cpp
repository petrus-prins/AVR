#include <EEPROM.h>
#include <Arduino.h>
#include <PID_Eeprom.h>






void eeput(double value, int EE_index) 
{ 
    char * addr = (char * ) &value;
    for(int i=EE_index; i<EE_index+4; i++)  
    {
        EEPROM.write(i,addr[i-EE_index]);
    }
}



double eeget(int EE_index) 
{ 
    double value;
    char * addr = (char * ) &value;
    for(int i=EE_index; i<EE_index+4; i++) 
    {
        addr[i-EE_index]=EEPROM.read(i);
    }
    return value;
}





// keep PID set values in EEPROM so they are kept when arduino goes off
void writetoEEPROM(double kp, double ki, double kd) 
{ 
    eeput(kp,0);
    eeput(ki,4);
    eeput(kd,8);
    double local_checksum=0;
    for(int i=0; i<12; i++) local_checksum+=EEPROM.read(i);
    eeput(local_checksum,12);
    Serial.println("\nPID values stored to EEPROM");
    //Serial.println(local_checksum);
}





void recoverPIDfromEEPROM(double &kp, double &ki, double &kd)  
{
    double local_checksum=0;
    double EE_checksum;
    for(int i=0; i<12; i++) 
    {
        local_checksum+=EEPROM.read(i);
    }
    EE_checksum=eeget(12);
    //Serial.println(local_checksum);
    if(local_checksum==EE_checksum) 
    {
       kp=eeget(0);
       ki=eeget(4);
       kd=eeget(8);
    }
    else Serial.println(F("*** Bad chelocal_checksumum"));
}




void eedump() {
  for(int i=0; i<16; i++) {
    Serial.print(EEPROM.read(i),HEX); Serial.print(" ");
  }
  Serial.println(); 
}
