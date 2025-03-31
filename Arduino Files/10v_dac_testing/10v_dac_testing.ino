#include "DFRobot_GP8403.h"
DFRobot_GP8403 dac(&Wire,0x5F);

void setup() {
  Serial.begin(115200);
  while(dac.begin()!=0){
    Serial.println("init error");
    delay(1000);
   }
  Serial.println("init succeed");
  //Set DAC output range
  dac.setDACOutRange(dac.eOutputRange5V);
  dac.setDACOutVoltage(5000,1);
}



void loop(){
 dac.outputSin(2500, 1, 2500, 0);//(voltage value, frequency (1Hz), voltage value, channel number)
}
