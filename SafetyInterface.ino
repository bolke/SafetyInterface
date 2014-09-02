#include "SafetyInterface.h"

#include <Arduino.h>
#include <EEPROM.h>
#include "TimerOne.h"

uint16_t eepromStart=5;
uint8_t commMode=DEFAULT_COMMUNICATION;
uint8_t mode=SETUP_MODE;
uint8_t registers[NR_OF_REGISTERS]={0};

volatile uint16_t* ptrValue=NULL;
volatile uint8_t bufferTarget=0xFF;
volatile uint16_t buffer=0;
volatile boolean writeFlag=false;
volatile uint32_t time=0;
volatile uint16_t timers[NR_OF_TIMERS]={0};
volatile uint8_t led0Pin=0;
volatile uint8_t led1Pin=0;

boolean CheckAdc(){
  SetLargeRegister(REG_ADC0_VALUE,analogRead(0));
  SetLargeRegister(REG_ADC1_VALUE,analogRead(1));
  SetLargeRegister(REG_ADC2_VALUE,analogRead(2));
  SetLargeRegister(REG_ADC3_VALUE,analogRead(3));
  SetLargeRegister(REG_ADC4_VALUE,analogRead(6));
  SetLargeRegister(REG_ADC5_VALUE,analogRead(7));
  return true;
}

uint8_t CheckInput(){
  uint8_t result=false;
  uint8_t bitVal;

  digitalWrite(D165_EN_PIN, HIGH);
  digitalWrite(D165_LO_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(D165_LO_PIN, HIGH);
  digitalWrite(D165_EN_PIN, LOW);

  for(int i=0;i<8;i++){
    bitVal = digitalRead(D165_DO_PIN);

    result |= (bitVal << i);

    digitalWrite(D165_CK_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(D165_CK_PIN, LOW);
  }

  registers[REG_INPUT_VALUE]=result;
//  CheckAlarm(ALARM_INPUT);
  GetLargeRegister(REG_INPUT_INTERVAL,(uint16_t*)&timers[INPUT_TIMER]);

  return result;
}

uint8_t SetOutput(){
  uint8_t output=registers[REG_OUTPUT_VALUE];
  uint16_t outputInterval=0;
  uint16_t output230vInterval=0;

  outputInterval=GetUInt16_t(registers[REG_OUTPUT_INTERVAL_HIGHBYTE],registers[REG_OUTPUT_INTERVAL_HIGHBYTE]);

  if(IsFunctionEnabled(FNC_OUTPUT_230V)){
    output230vInterval=GetUInt16_t(registers[REG_OUTPUT_INTERVAL_HIGHBYTE],registers[REG_OUTPUT_INTERVAL_HIGHBYTE]);
    if(output230vInterval<outputInterval)
      output230vInterval=outputInterval;

    if(timers[OUTPUT_230V_TIMER]==0){
      GetLargeRegister(REG_OUTPUT_230V_DELAY,(uint16_t*)&timers[OUTPUT_230V_TIMER]);
      output=output|(0x0F & registers[REG_OUTPUT_TARGET_VALUE]);
    }
  }

  if(timers[OUTPUT_TIMER]==0){
    GetLargeRegister(REG_OUTPUT_INTERVAL,(uint16_t*)&timers[OUTPUT_TIMER]);
    output=output|(0xF0 & registers[REG_OUTPUT_TARGET_VALUE]);
  }

//  if(alarmFlag)
//    output=output&alarmMask;

  if(!IsFunctionEnabled(FNC_OUTPUT))
    output=0;

  digitalWrite(D595_LT_PIN, LOW); //latch
  shiftOut(D595_DI_PIN, D595_CK_PIN, LSBFIRST,output);
  digitalWrite(D595_LT_PIN, HIGH); //unlatch

  registers[REG_OUTPUT_VALUE]=output;

  return 0;
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

void GetLargeRegister(uint8_t target,volatile uint16_t* value){
  if(value!=NULL){
    bufferTarget=target;
    ptrValue=value;
    writeFlag=true;
    while(writeFlag);
  }
}

void SetLargeRegister(uint8_t target,uint16_t value){
  ptrValue=NULL;
  buffer=value;
  bufferTarget=target;
  writeFlag=true;
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
      if(ptrValue==NULL){
        SetRegUInt16_t(bufferTarget,buffer);
      }else{
        *ptrValue=GetRegUInt16_t(bufferTarget);
      }
      buffer=0;
      bufferTarget=0xFF;
      ptrValue=NULL;
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

