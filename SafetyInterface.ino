#include "SafetyInterface.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <Serial.h>
#include "TimerOne.h"

uint16_t eepromStart=5;
uint8_t commMode=DEFAULT_COMMUNICATION;
uint8_t mode=SETUP_MODE;
uint8_t registers[NR_OF_REGISTERS]={0};

volatile uint8_t bufferTarget=0xFF;
volatile uint16_t buffer=0;
volatile boolean writeFlag=false;
volatile uint32_t time=0;
volatile uint16_t timers[NR_OF_TIMERS]={0};
volatile uint8_t led0Pin=0;
volatile uint8_t led1Pin=0;

boolean CheckAdc(){
  SetRegUInt16_t(REG_ADC0_VALUE,analogRead(0));
  SetRegUInt16_t(REG_ADC1_VALUE,analogRead(1));
  SetRegUInt16_t(REG_ADC2_VALUE,analogRead(2));
  SetRegUInt16_t(REG_ADC3_VALUE,analogRead(3));
  SetRegUInt16_t(REG_ADC4_VALUE,analogRead(6));
  SetRegUInt16_t(REG_ADC5_VALUE,analogRead(7));
  return true;
}

boolean CheckInput(){
  boolean result=false;
  if(BOARD_TYPE==BOARD_ARDUINO_MINI){
  }else if(BOARD_TYPE==BOARD_CNC_SHIELD){
  }
  return result;
}

void SetOutput(){
  //set outputs
  //set 74hc595
}

void LoadDefaults(){
  memset(registers,0,NR_OF_REGISTERS);
  registers[REG_COMMUNICATION_MODE]=DEFAULT_COMMUNICATION;
  SetRegUInt16_t(REG_HEARTBEAT_INTERVAL,1000);
  SetRegUInt16_t(REG_INPUT_INTERVAL,100);
  SetRegUInt16_t(REG_ADC_INTERVAL,100);
  SetRegUInt16_t(REG_OUTPUT_INTERVAL,100);
  SetRegUInt16_t(REG_OUTPUT_230V_DELAY,500);
  commMode=DEFAULT_COMMUNICATION;
}

boolean LoadFromEeprom(){
  boolean result=false;
  uint8_t i=0;
  LoadDefaults();
  if(EEPROM.read(0)==MAGIC_EEPROM_BYTE_0){
    if(EEPROM.read(1)==MAGIC_EEPROM_BYTE_1){
      if(EEPROM.read(2)==MAGIC_EEPROM_BYTE_2){
        eepromStart=GetUInt16_t(EEPROM.read(3),EEPROM.read(4));
        for(i=0;i<NR_OF_SAVED_REGISTERS;i++)
          registers[i]=EEPROM.read(eepromStart+i);
        result=true;
      }
    }
  }
  return result;
}

uint16_t SaveToEeprom(uint16_t forcedStart){
  uint16_t result=0;
  uint8_t i=0;

  if(EEPROM.read(0)==MAGIC_EEPROM_BYTE_0){
    if(EEPROM.read(1)==MAGIC_EEPROM_BYTE_1){
      if(EEPROM.read(2)==MAGIC_EEPROM_BYTE_2){
        if(forcedStart==0){
          eepromStart=GetUInt16_t(EEPROM.read(4),EEPROM.read(3));
          if((eepromStart+(NR_OF_SAVED_REGISTERS*2))>1024)
            forcedStart=5;
          else
	    forcedStart=eepromStart+NR_OF_SAVED_REGISTERS;
	  result=1;
        }
      }
    }
  }

  if(forcedStart==0)
    forcedStart=5;

  EEPROM.write(0,MAGIC_EEPROM_BYTE_0);
  EEPROM.write(1,MAGIC_EEPROM_BYTE_1);
  EEPROM.write(2,MAGIC_EEPROM_BYTE_2);
	eepromStart=forcedStart;

  EEPROM.write(3,LowByte(eepromStart));
  EEPROM.write(4,HighByte(eepromStart));

  for(uint8_t i=0;i<NR_OF_SAVED_REGISTERS;i++)
    EEPROM.write(eepromStart+i,registers[i]);

  result=eepromStart;
  return result;
}

boolean InitPheripherals(){
  boolean result=true;
  switch(registers[REG_COMMUNICATION_MODE]){
    case COMM_SERIAL:
      if(BOARD_TYPE==BOARD_ARDUINO_MINI)
        InitSerial();
      break;
    default:
      result=false;
      registers[REG_COMMUNICATION_MODE]=DEFAULT_COMMUNICATION;
  }
  return result;
}

void InitSerial(){
  led0Pin=I2C_SDA_PIN;
  led1Pin=I2C_SCL_PIN;
  Serial.begin(115200);
}

uint8_t CheckTime(){
  uint8_t result=0;
  uint32_t now=millis();
  int32_t timePassed=now-time;
  if(timePassed<0)
    timePassed=timePassed*-1;
  for(uint8_t i=0;i<NR_OF_TIMERS;i++){
    SetTime(timers[i],timePassed);
    if(timers[i]==0)
      result++;
  }
  digitalWrite(led1Pin,!digitalRead(led1Pin));
  time=now;
  return result;
}

void serialEvent(){
  while(Serial.available()){
	  Serial.read();
	}
}
  while(Serial.available()>0)
    Serial.read();
}

void SetLargeRegister(uint8_t target,uint16_t value){
  buffer=value;
  writeFlag=true;
  bufferTarget=target;
  while(writeFlag);
}

void ErrorLedFlash(){
  digitalWrite(led0Pin,!digitalRead(led0Pin));
  digitalWrite(led1Pin,!digitalRead(led1Pin));
  delay(500);
}

void TimerCallback(){
  if(writeFlag){
    if(bufferTarget<NR_OF_REGISTERS){
      SetRegUInt16_t(bufferTarget,buffer);
      buffer=0;
      bufferTarget=0xFF;
    }
    writeFlag=false;
  }
}

void setup(){
  InitSerial();
  if(mode==SETUP_MODE){
    if(!LoadFromEeprom()){
      #ifdef DEBUG
      Serial.println("Failed LoadFromEeprom. Save eeprom.");
      #endif
      SaveToEeprom();
    }
    if(!InitPheripherals()){
      if(!InitPheripherals())
        mode=ERROR_MODE;
    }
  }else
    mode=ERROR_MODE;
  if(mode==SETUP_MODE){
    #ifdef DEBUG
    Serial.println("Running");
    #endif
    commMode=registers[REG_COMMUNICATION_MODE];
    timers[HEARTBEAT_TIMER]=GetRegUInt16_t(REG_HEARTBEAT_INTERVAL);
    timers[OUTPUT_TIMER]=GetRegUInt16_t(REG_OUTPUT_INTERVAL);
    timers[OUTPUT_230V_TIMER]=GetRegUInt16_t(REG_OUTPUT_230V_DELAY);
    mode=RUNNING_MODE;
  }
  #ifdef DEBUG
  Serial.println("registers values");
  for(uint8_t i=0;i<NR_OF_REGISTERS;i++){
    Serial.print(i);
    Serial.print(" ");
    Serial.println(registers[i],DEC);
  }
  Serial.println("");
  Serial.println("UInt16_t registers");
  Serial.println(GetRegUInt16_t(REG_HEARTBEAT_INTERVAL),DEC);
  Serial.println(GetRegUInt16_t(REG_INPUT_INTERVAL),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC_INTERVAL),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC0_THRESHOLD),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC0_VALUE),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC1_THRESHOLD),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC1_VALUE),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC2_THRESHOLD),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC2_VALUE),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC3_THRESHOLD),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC3_VALUE),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC4_THRESHOLD),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC4_VALUE),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC5_THRESHOLD),DEC);
  Serial.println(GetRegUInt16_t(REG_ADC5_VALUE),DEC);
  Serial.println(GetRegUInt16_t(REG_INTERNAL_TEMP),DEC);
  Serial.println(GetRegUInt16_t(REG_OUTPUT_INTERVAL),DEC);
  Serial.println(GetRegUInt16_t(REG_OUTPUT_230V_DELAY),DEC);
  #endif
  Timer1.initialize(1000);
  Timer1.attachInterrupt(TimerCallback);
  time=millis();
}

void loop(){
  switch(mode){
    case RUNNING_MODE:
       CheckTime();
       digitalWrite(led0Pin,0);
       break;
     case ALARM_MODE:
       CheckTime();
       digitalWrite(led0Pin,1);
       break;
     default:
       ErrorLedFlash();
       break;
  }
  #ifdef DEBUG
  if(timers[HEARTBEAT_TIMER]==0){
    Serial.print("HEARTBEAT: ");
    Serial.println(time);
    timers[HEARTBEAT_TIMER]=GetRegUInt16_t(REG_HEARTBEAT_INTERVAL);
  }
  #endif
}

