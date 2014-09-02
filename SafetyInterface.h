#ifndef SAFETYINTERFACE_H
#define SAFETYINTERFACE_H

#include <Arduino.h>

#define LowByte(a)                    (a&0xFF)
#define HighByte(a)                   (((uint16_t)a)>>8)
#define SplitUInt16_t(a,b,c)          (b=LowByte(a));(c=HighByte(a))
#define GetUInt16_t(a,b)              ((((uint16_t)b)<<8)+a)
#define SetTime(a,b)                  (a=(a>b?a-b:0))
#define GetRegUInt16_t(a)             GetUInt16_t(registers[a],registers[a+1])
#define SetRegUInt16_t(a,b)           SplitUInt16_t(b,registers[a],registers[a+1])
#define IsFunctionEnabled(a)          ((registers[REG_FUNCTION_ENABLE]&a)!=0)

#define MAGIC_EEPROM_BYTE_0           0xA2
#define MAGIC_EEPROM_BYTE_1           0x14
#define MAGIC_EEPROM_BYTE_2           0x6C

#define NR_OF_SAVED_REGISTERS         47
#define NR_OF_NOT_SAVED_REGISTERS     19
#define NR_OF_REGISTERS               (NR_OF_SAVED_REGISTERS+NR_OF_NOT_SAVED_REGISTERS)
#define NR_OF_TIMERS                  5

#define COMM_I2C                      1
#define COMM_SPI                      2

#define I2C_ADDRESS                    0x2A

#define DEFAULT_COMMUNICATION         COMM_I2C

enum SavedRegisters{
  REG_COMMUNICATION_MODE,
  REG_I2C_ADDRESS,
  REG_FUNCTION_ENABLE,
  REG_HEARTBEAT_INTERVAL_LOWBYTE,
  REG_HEARTBEAT_INTERVAL_HIGHBYTE,
  REG_HEARTBEAT_MASK,
  REG_INPUT_ENABLE,
  REG_INPUT_TRIGGER,
  REG_INPUT_INTERVAL_LOWBYTE,
  REG_INPUT_INTERVAL_HIGHBYTE,
  REG_INPUT0_MASK,
  REG_INPUT1_MASK,
  REG_INPUT2_MASK,
  REG_INPUT3_MASK,
  REG_INPUT4_MASK,
  REG_INPUT5_MASK,
  REG_INPUT6_MASK,
  REG_INPUT7_MASK,
  REG_ADC_ENABLE,
  REG_ADC_TRIGGER,
  REG_ADC_INTERVAL_LOWBYTE,
  REG_ADC_INTERVAL_HIGHBYTE,
  REG_ADC0_MASK,
  REG_ADC1_MASK,
  REG_ADC2_MASK,
  REG_ADC3_MASK,
  REG_ADC4_MASK,
  REG_ADC5_MASK,
  REG_ADC0_THRESHOLD_LOWBYTE,
  REG_ADC0_THRESHOLD_HIGHBYTE,
  REG_ADC1_THRESHOLD_LOWBYTE,
  REG_ADC1_THRESHOLD_HIGHBYTE,
  REG_ADC2_THRESHOLD_LOWBYTE,
  REG_ADC2_THRESHOLD_HIGHBYTE,
  REG_ADC3_THRESHOLD_LOWBYTE,
  REG_ADC3_THRESHOLD_HIGHBYTE,
  REG_ADC4_THRESHOLD_LOWBYTE,
  REG_ADC4_THRESHOLD_HIGHBYTE,
  REG_ADC5_THRESHOLD_LOWBYTE,
  REG_ADC5_THRESHOLD_HIGHBYTE,
  REG_OUTPUT_ENABLE,
  REG_OUTPUT_TARGET_VALUE,
  REG_OUTPUT_INTERVAL_LOWBYTE,
  REG_OUTPUT_INTERVAL_HIGHBYTE,
  REG_OUTPUT_230V_DELAY_LOWBYTE,
  REG_OUTPUT_230V_DELAY_HIGHBYTE,
  REG_PROBE_MASK
};

enum RuntimeRegisters{
  REG_ALARM_INTERRUPTS_0=REG_PROBE_MASK+1,
  REG_ALARM_INTERRUPTS_1,
  REG_ADC0_VALUE_LOWBYTE,
  REG_ADC0_VALUE_HIGHBYTE,
  REG_ADC1_VALUE_LOWBYTE,
  REG_ADC1_VALUE_HIGHBYTE,
  REG_ADC2_VALUE_LOWBYTE,
  REG_ADC2_VALUE_HIGHBYTE,
  REG_ADC3_VALUE_LOWBYTE,
  REG_ADC3_VALUE_HIGHBYTE,
  REG_ADC4_VALUE_LOWBYTE,
  REG_ADC4_VALUE_HIGHBYTE,
  REG_ADC5_VALUE_LOWBYTE,
  REG_ADC5_VALUE_HIGHBYTE,
  REG_INPUT_VALUE,
  REG_OUTPUT_VALUE,
  REG_INTERNAL_TEMP_LOWBYTE,
  REG_INTERNAL_TEMP_HIGHBYTE,
  REG_PROBE_VALUE
};

enum UInt16_tRegisters{
  REG_HEARTBEAT_INTERVAL=REG_HEARTBEAT_INTERVAL_LOWBYTE,
  REG_INPUT_INTERVAL=REG_INPUT_INTERVAL_LOWBYTE,
  REG_ADC_INTERVAL=REG_ADC_INTERVAL_LOWBYTE,
  REG_ADC0_THRESHOLD=REG_ADC0_THRESHOLD_LOWBYTE,
  REG_ADC0_VALUE=REG_ADC0_VALUE_LOWBYTE,
  REG_ADC1_THRESHOLD=REG_ADC1_THRESHOLD_LOWBYTE,
  REG_ADC1_VALUE=REG_ADC1_VALUE_LOWBYTE,
  REG_ADC2_THRESHOLD=REG_ADC2_THRESHOLD_LOWBYTE,
  REG_ADC2_VALUE=REG_ADC2_VALUE_LOWBYTE,
  REG_ADC3_THRESHOLD=REG_ADC3_THRESHOLD_LOWBYTE,
  REG_ADC3_VALUE=REG_ADC3_VALUE_LOWBYTE,
  REG_ADC4_THRESHOLD=REG_ADC4_THRESHOLD_LOWBYTE,
  REG_ADC4_VALUE=REG_ADC4_VALUE_LOWBYTE,
  REG_ADC5_THRESHOLD=REG_ADC5_THRESHOLD_LOWBYTE,
  REG_ADC5_VALUE=REG_ADC5_VALUE_LOWBYTE,
  REG_INTERNAL_TEMP=REG_INTERNAL_TEMP_LOWBYTE,
  REG_OUTPUT_INTERVAL=REG_OUTPUT_INTERVAL_LOWBYTE,
  REG_OUTPUT_230V_DELAY=REG_OUTPUT_230V_DELAY_LOWBYTE
};

const uint8_t REG_IO_ASSIGNMENT=REG_OUTPUT_230V_DELAY;

enum Messages{
  MSG_HEARTBEAT=1,
  MSG_READ,
  MSG_WRITE,
  MSG_RESET_ALARM,
  MSG_RESET_SETTINGS
};

enum Functions{
  FNC_HEARTBEAT=1,
  FNC_INPUT=2,
  FNC_ADC=4,
  FNC_OUTPUT=8,
  FNC_OUTPUT_230V=16,
  FNC_PROBE=32
  //FNC_=64,
  //FNC_=128
};

enum AlarmTriggers{
  ALARM_OFF,
  ALARM_HEARTBEAT=1,
  ALARM_INPUT,
  ALARM_ADC,
  ALARM_PROBE
};

enum PhysicalPins{
  PROBE_PIN=0,                  //rx, if serial
  D595_DI_PIN=1,                //tx, if serial
  D595_OE_PIN=2,
  D595_LT_PIN=3,
  D595_CK_PIN=4,
  D165_CK_PIN=5,
  D165_LO_PIN=6,
  D165_DO_PIN=7,
  D165_EN_PIN=8,
  D595_MR_PIN=9,
  SPI_SS_PIN=10,
  SPI_MOSI_PIN=11,
  SPI_MISO_PIN=12,              //led0Pin if not spi
  SPI_CLK_PIN=13,               //led1Pin if not spi
  I2C_SDA_PIN=18,               //led0Pin if not i2c
  I2C_SCL_PIN=19                //led1Pin if not i2c
};

enum RunningModes{
  RUNNING_MODE,
  ALARM_MODE,
  ERROR_MODE,
  SETUP_MODE,
};

enum TimerIds{
  HEARTBEAT_TIMER,
  INPUT_TIMER,
  ADC_TIMER,
  OUTPUT_TIMER,
  OUTPUT_230V_TIMER
};

void LoadDefaults();
boolean LoadFromEeprom();
uint16_t SaveToEeprom(uint16_t forcedStart=0);
boolean InitPheripherals();
void InitSerial();
boolean WriteRegister(uint8_t address,uint8_t value);
void SetLargeRegister(uint8_t target,uint16_t value);
void GetLargeRegister(uint8_t target,volatile uint16_t* value);
uint8_t CheckTime();
void ErrorLedFlash();
void TimerCallback();
boolean CheckAdc();
uint8_t CheckInput();
uint8_t SetOutput();
uint8_t CheckAlarm(uint8_t cause);

#endif
