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

#define DEBUG                         1

#define MAGIC_EEPROM_BYTE_0           0xA2
#define MAGIC_EEPROM_BYTE_1           0x14
#define MAGIC_EEPROM_BYTE_2           0x6C

#define NR_OF_SAVED_REGISTERS         47
#define NR_OF_NOT_SAVED_REGISTERS     19
#define NR_OF_REGISTERS               (NR_OF_SAVED_REGISTERS+NR_OF_NOT_SAVED_REGISTERS)
#define NR_OF_TIMERS                  5

#define COMM_I2C                      1
#define COMM_SPI                      2
#define COMM_SERIAL                   3

#define I2C_ADDRESS                    0x2A

#define DEFAULT_COMMUNICATION         COMM_SERIAL

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

enum LargeRegisters{
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

enum Messages{
  MSG_HEARTBEAT=1,
  MSG_READ,
  MSG_WRITE,
  MSG_RESET_ALARM,
  MSG_RESET_DEFAULT_SETTINGS,
  MSG_RELOAD_EEPROM_SETTINGS,
  MSG_SAVE_SETTINGS,
  MSG_RESET
};

enum Functions{
  FNC_NONE=0,
  FNC_HEARTBEAT=1,
  FNC_INPUT=2,
  FNC_ADC=4,
  FNC_OUTPUT=8,
  FNC_SELFRESTORE=64
};

enum AlarmTriggers{
  ALARM_OFF=0,
  ALARM_HEARTBEAT=1,
  ALARM_INPUT=2,
  ALARM_ADC=4
};

enum PhysicalPins{
  SPI_SS_PIN=10,
  SPI_MOSI_PIN=11,
  SPI_MISO_PIN=12,              ///led0Pin if not spi
  SPI_CLK_PIN=13,               ///led1Pin if not spi
  I2C_SDA_PIN=18,               ///led0Pin if not i2c
  I2C_SCL_PIN=19                ///led1Pin if not i2c
};

enum RunningModes{
  SETUP_MODE,
  RUNNING_MODE,
  ALARM_MODE,
  ERROR_MODE
};

enum TimerIds{
  HEARTBEAT_TIMER,
  INPUT_TIMER,
  ADC_TIMER,
  OUTPUT_TIMER
};

const uint8_t inputPins[]={};                             ///pin numbers used for input. digitalRead uses these.
const uint8_t outputPins[]={};                            ///pin numbers used for output. digitalWrite uses these.
const uint8_t adcPins[]={};                               ///pin numbers used for adc. analogRead uses these.
const uint8_t inputCnt=sizeof(inputPins);                 ///nr of input pins.
const uint8_t outputCnt=sizeof(outputPins);               ///nr of output pins.
const uint8_t adcCnt=sizeof(adcPins);                     ///nr of adc pins.

/// Set most values of registers to 0, except some timer intervals,
/// and communication mode.
void LoadDefaults();

/// Load variable from eeprom.
/// @return Loaded or not?
boolean LoadFromEeprom();

/// Save data to eeprom, using address given or determine the address.
/// If forcedStart eq 0, address is read from eeprom and incremented with NR_OF_SAVED_REGISTERS.
/// When data doesn't fit, it'll reset forcedStart to 5, the default address. Eeprom has 3 id bytes
/// and 2 address bytes before data starts.
/// @param forcedStart Address in eeprom to save data. If 0, determine own address.
/// @return Saved or not?
boolean SaveToEeprom(uint16_t forcedStart=0);

/// Start i2c, spi or serial. Depends upon commMode;
/// @return initialized or not?
boolean InitPheripherals();

/// Start serial, 115200 baud, normal settings
void InitSerial();

/// Start default arduino Wire object. 100k hz.
/// Initialises event handlers.
void InitI2C();

/// Start spi slave mode.
void InitSPI();

/// i2c request event handler. Handles read messages, returns register values.
void I2CRequest();

/// i2c receive handler. Handles all other messages.
void I2CReceive(int16_t byteCnt);

/// Wire value to address, and check if everything is valid.
/// @param address register address to write, saved registers are writeable, others not
/// @param value the value to write.
boolean WriteRegister(uint8_t address,uint8_t value);

/// Set a large register using the timer. Function blocks until timer goes of and sets
/// value. Done because of non atomic operations (16bit value, 8bit proc, etc).
/// @param target address register address to set. Must be value in LargeRegister enum.
/// @param value value to insert into address.
void SetLargeRegister(uint8_t target,uint16_t value);

/// Set a uint16_t to the value at target address.
/// @param target address register address to set. Must be value in LargeRegister enum.
/// @param value value variable to be written.
void GetLargeRegister(uint8_t target,volatile uint16_t* value);

/// Toggle the 2 leds, to indicate error mode.
void ErrorLedFlash();

/// Callback for timer object. SetLargeRegister and GetLargeRegister depend upon this.
void TimerCallback();

/// Timekeeper function. Sets countdown timers for the different functions of the device.
/// @return the number of countdown timers that have reached 0.
uint8_t CheckTime();

/// Read the given adc pins and check the alarm.
void CheckAdc();

/// Read the given input pins and check the alarm.
void CheckInput();

/// Set the output pins according to the requested value and the alarm mask, if any.
///@return the output value, including alarm mask.
uint8_t SetOutput();

/// Check the alarm for the given cause. Checks if an input is triggered, and adc value
/// passed a threshold, etc.
/// @param cause an item out of AlarmTriggers enum.
/// @return the state of alarm, what alarm is active.
uint8_t CheckAlarm(uint8_t cause);

#endif
