#include "SafetyInterface.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>                                             //http://playground.arduino.cc/Main/WireLibraryDetailedReference
#include <SPI.h>
#include <avr/wdt.h>
#include "TimerOne.h"

uint16_t eepromStart=5;
uint8_t action=0;
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
volatile uint8_t alarm=ALARM_OFF;
volatile uint8_t alarmMask=0;

boolean CheckAdc(){
  SetLargeRegister(REG_ADC0_VALUE,analogRead(0));
  SetLargeRegister(REG_ADC1_VALUE,analogRead(1));
  SetLargeRegister(REG_ADC2_VALUE,analogRead(2));
  SetLargeRegister(REG_ADC3_VALUE,analogRead(3));
  SetLargeRegister(REG_ADC4_VALUE,analogRead(6));
  SetLargeRegister(REG_ADC5_VALUE,analogRead(7));
  CheckAlarm(ALARM_ADC);
  GetLargeRegister(REG_ADC_INTERVAL,&timers[ADC_TIMER]);
  return true;
}

boolean CheckInput(){
  uint8_t value=0;
  uint8_t shifted=0;
  for(uint8_t i=0;i<inputCnt;i++){
    shifted=1<<i;
    if(digitalRead(inputPins[i]))
      value=value|shifted;
  }
  registers[REG_INPUT_VALUE]=value;
  return CheckAlarm(ALARM_INPUT)==ALARM_OFF;
}

uint8_t SetOutput(){
  uint8_t value=0;
  return value;
}

void LoadDefaults(){
  memset(registers,0,NR_OF_REGISTERS);
  registers[REG_COMMUNICATION_MODE]=DEFAULT_COMMUNICATION;
  registers[REG_I2C_ADDRESS]=I2C_ADDRESS;
  registers[REG_FUNCTION_ENABLE]=FNC_HEARTBEAT|FNC_ADC|FNC_INPUT|FNC_OUTPUT;
  SetLargeRegister(REG_HEARTBEAT_INTERVAL,1000);
  SetLargeRegister(REG_INPUT_INTERVAL,100);
  SetLargeRegister(REG_ADC_INTERVAL,100);
  SetLargeRegister(REG_OUTPUT_INTERVAL,100);
  SetLargeRegister(REG_OUTPUT_230V_DELAY,500);
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
    case COMM_I2C:
      InitI2C();
      break;
    case COMM_SPI:
      InitSPI();
      break;
    case COMM_SERIAL:
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

void InitI2C(){
  Wire.begin(registers[REG_I2C_ADDRESS]);
  Wire.onReceive(I2CReceive);
  Wire.onRequest(I2CRequest);
  led0Pin=SPI_MISO_PIN;
  led1Pin=SPI_CLK_PIN;
  pinMode(SPI_MOSI_PIN,INPUT);
  pinMode(SPI_SS_PIN,INPUT);
  pinMode(led0Pin,OUTPUT);
  pinMode(led1Pin,OUTPUT);
}

void InitSPI(){
}

void I2CRequest(){
  if(Wire.available()){
    if(Wire.read()==MSG_READ){
      if(Wire.available()>0){
        uint8_t start=Wire.read();
        uint8_t cnt=1;
        if(Wire.available()>0)
          cnt=Wire.read();
        while(cnt>0){
          if(NR_OF_REGISTERS>start){
            Wire.write(registers[start]);
            start++;
            cnt--;
          }else
            break;
        }
      }
    }
  }
}

void I2CReceive(int16_t byteCnt){
  if(Wire.available()>0){
    switch(Wire.read()){
      case MSG_HEARTBEAT:
        timers[HEARTBEAT_TIMER]=GetRegUInt16_t(REG_HEARTBEAT_INTERVAL);
        break;
      case MSG_WRITE:
        if(Wire.available()>0){
          uint8_t start=Wire.read();
          uint8_t cnt=1;
          if(Wire.available()>1)
            cnt=Wire.read();
          while(cnt>0){
            if(Wire.available()>0)
              if(!WriteRegister(start,Wire.read()))
                break;
            cnt--;
          }
        }
        break;
      case MSG_RESET_ALARM:
        if(Wire.available()==3){
          if((Wire.read()==MAGIC_EEPROM_BYTE_0)&&(Wire.read()==MAGIC_EEPROM_BYTE_1)&&(Wire.read()==MAGIC_EEPROM_BYTE_2)){
            alarm=ALARM_OFF;
            registers[REG_ALARM_INTERRUPTS_0]=0;
            registers[REG_ALARM_INTERRUPTS_1]=0;
          }
        }
        break;
      case MSG_RESET_DEFAULT_SETTINGS:
        if(Wire.available()==3){
          if((Wire.read()==MAGIC_EEPROM_BYTE_0)&&(Wire.read()==MAGIC_EEPROM_BYTE_1)&&(Wire.read()==MAGIC_EEPROM_BYTE_2)){
            LoadDefaults();
          }
        }
        break;
      case MSG_RELOAD_EEPROM_SETTINGS:
        if(Wire.available()==3){
          if((Wire.read()==MAGIC_EEPROM_BYTE_0)&&(Wire.read()==MAGIC_EEPROM_BYTE_1)&&(Wire.read()==MAGIC_EEPROM_BYTE_2)){
            action=MSG_RELOAD_EEPROM_SETTINGS;
          }
        }
        break;
      case MSG_SAVE_SETTINGS:
        if(Wire.available()==3){
          if((Wire.read()==MAGIC_EEPROM_BYTE_0)&&(Wire.read()==MAGIC_EEPROM_BYTE_1)&&(Wire.read()==MAGIC_EEPROM_BYTE_2)){
            action=MSG_SAVE_SETTINGS;
          }
        }
        break;
      case MSG_RESET:
        if(Wire.available()==3){
          if((Wire.read()==MAGIC_EEPROM_BYTE_0)&&(Wire.read()==MAGIC_EEPROM_BYTE_1)&&(Wire.read()==MAGIC_EEPROM_BYTE_2)){
            action=MSG_RESET;
          }
        }
        break;
    }
    while(Wire.available()>0)
      Wire.read();
  }
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

boolean WriteRegister(uint8_t address,uint8_t value){
  boolean result=false;
  if(address<NR_OF_SAVED_REGISTERS){
    registers[address]=value;
    result=true;
  }
  return result;
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

uint8_t CheckAlarm(uint8_t cause){
  uint8_t result=ALARM_OFF;
  switch(cause){
    case ALARM_HEARTBEAT:
      if(IsFunctionEnabled(FNC_HEARTBEAT)){
        if(timers[HEARTBEAT_TIMER]==0){
          result=cause;
          if(alarm==ALARM_OFF){
            alarmMask=registers[REG_HEARTBEAT_MASK];
            registers[REG_ALARM_INTERRUPTS_0]=ALARM_HEARTBEAT;
            registers[REG_ALARM_INTERRUPTS_1]=0;
          }
        }
      }
      break;
    case ALARM_INPUT:
      if(IsFunctionEnabled(FNC_INPUT)){
        uint8_t value=registers[REG_INPUT_VALUE];
        for(uint8_t i=0;i<8;i++){
          uint8_t shifted=1<<i;
          if((registers[REG_INPUT_ENABLE]&shifted)>0){
            if((registers[REG_INPUT_TRIGGER]&shifted)==(value&shifted)){
              result=cause;
              if(alarm==ALARM_OFF){
                alarmMask=registers[REG_INPUT0_MASK+i];
                registers[REG_ALARM_INTERRUPTS_0]=ALARM_INPUT;
                registers[REG_ALARM_INTERRUPTS_1]=shifted;
              }
              break;
            }
          }
        }
      }
      break;
    case ALARM_ADC:
      if(IsFunctionEnabled(FNC_ADC)){
        uint8_t shifted=0;
        boolean highThreshold=false;
        uint16_t value=0;
        uint16_t compare=0;
        for(uint8_t i=0;i<6;i++){
          shifted=1<<i;
          highThreshold=false;
          if((registers[REG_ADC_ENABLE]&shifted)>0){
            GetLargeRegister(REG_ADC0_VALUE+i*2,&value);
            GetLargeRegister(REG_ADC0_THRESHOLD+i*2,&compare);
            highThreshold=(registers[REG_ADC_TRIGGER]&shifted)>0;
            if(highThreshold)
              if(value>=compare)
                result=ALARM_ADC;
            else
              if(value<=compare)
                result=ALARM_ADC;
            if(alarm==ALARM_OFF){
              registers[REG_ALARM_INTERRUPTS_0]=ALARM_ADC;
              registers[REG_ALARM_INTERRUPTS_1]=i;
              break;
            }
          }
        }
      }
      break;
  }
  if(alarm==ALARM_OFF)
    alarm=result;
  return result;
}

void setup(){
  wdt_disable();

  Timer1.initialize(1000);
  Timer1.attachInterrupt(TimerCallback);

  if(mode==SETUP_MODE){
    if(!LoadFromEeprom()){
      SaveToEeprom();
    }
    if(!InitPheripherals()){
      if(!InitPheripherals())
        mode=ERROR_MODE;
    }
  }else
    mode=ERROR_MODE;
  if(mode==SETUP_MODE){
    commMode=registers[REG_COMMUNICATION_MODE];
    GetLargeRegister(REG_HEARTBEAT_INTERVAL,&timers[HEARTBEAT_TIMER]);
    GetLargeRegister(REG_INPUT_INTERVAL,&timers[INPUT_TIMER]);
    GetLargeRegister(REG_ADC_INTERVAL,&timers[ADC_TIMER]);
    GetLargeRegister(REG_OUTPUT_INTERVAL,&timers[OUTPUT_TIMER]);
    mode=RUNNING_MODE;
  }
  time=millis();
  wdt_enable(WDTO_250MS);
}

void loop(){
  switch(mode){
    case RUNNING_MODE:
      CheckTime();
      CheckAdc();
      CheckInput();
      if(alarm==ALARM_OFF)
        SetOutput();
      else
        mode=ALARM_MODE;
      digitalWrite(led0Pin,0);
      break;
    case ALARM_MODE:
      CheckTime();
      SetOutput();
      if(IsFunctionEnabled(FNC_SELFRESTORE)){
        if(CheckAlarm(alarm)==ALARM_OFF){
          alarm=ALARM_OFF;
          mode=RUNNING_MODE;
        }
      }
      digitalWrite(led0Pin,1);
      break;
    default:
      ErrorLedFlash();
      break;
  }

  if(mode!=ALARM_MODE){
    switch(action){
      case MSG_RELOAD_EEPROM_SETTINGS:
        LoadFromEeprom();
        action=0;
        break;
      case MSG_SAVE_SETTINGS:
        SaveToEeprom();
        action=0;
        break;
      case MSG_RESET:
        while(1);
        action=0;
        break;
    }
  }else
    action=0;

  wdt_reset();
}
