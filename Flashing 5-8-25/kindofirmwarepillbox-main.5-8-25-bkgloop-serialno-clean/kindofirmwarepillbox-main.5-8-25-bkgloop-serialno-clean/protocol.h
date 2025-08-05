/*
 * Copyright (c) 2020 PeanutKing Solution
 *
 * @file        protocol.h
 * @summary     Kin pill dispenser stm32 android protocol
 * @version     1.0
 * @author      Jack Kwok
 * @data        1 August 2020
 */


#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include "main.h"

#define funcListLen 18

// register address list

typedef enum {
//  Power Ctrl & Status   0x00
  A33_PWR     = 0x01,   //    power key  of the A33 module
  BATT_CHRG   = 0x02,   //    charging or not ?
  BATT_STDBY  = 0x03,   //    check the battery charging finished or not
  PWR_EN      = 0x04,   //    Enable for RAW 5v output
  AIN_SNES    = 0x05,   //    check RAW to sys 5v current (A33 + sensor + stepper)
  VIN_SENS    = 0x06,   //    check the DC Vin 12v voltage
  VBAT_SENS   = 0x07,   //    Measure IPS battery voltage
  DCIN_DET    = 0x08,   //    check DC pwr is here or not, wakeup the device
  
// internal RTC             14    
  INT_RTC_YEAR    = 0x10,  //     
  INT_RTC_MONTH   = 0x11,  //    
  INT_RTC_DAY     = 0x12,  //     
  INT_RTC_HOUR    = 0x13,  //     
  INT_RTC_MINUTE  = 0x14,  //     
  INT_RTC_SECOND  = 0x15,  //     
  
// Sensor Read        
  getMPU6050      = 0x20,   //    Connect to the MPU6050 for tilt check
  ROLL_H          = 0x20,   //    Connect to the MPU6050 for tilt check
  ROLL_L          = 0x21,   //    Connect to the MPU6050 for tilt check
  PITCH_H         = 0x22,   //    Connect to the MPU6050 for tilt check
  PITCH_L         = 0x23,   //    Connect to the MPU6050 for tilt check
  YAW_H           = 0x24,   //    Connect to the MPU6050 for tilt check
  YAW_L           = 0x25,   //    Connect to the MPU6050 for tilt check
  getHDC1080      = 0x26,   //    Connect to the HDC1080 temp humid sensor
  getAIRPRESSURE  = 0x28,   //    check pump air pressure
  NFC_DATA        = 0x2a,   //
  PILLBX_DET      = 0x2f,   //    detect the pillbx is here or not, hall sensor 
  
  PROT_ENCI       = 0x30,   //    Pillbox rotary encoder 
  PROT_ENCZ       = 0x32,   //    External encoder
  CROT_ENCI       = 0x34,   //    Canister rotary encoder 
  CROT_ENCZ       = 0x36,   //    External encoder
  FRNTDR_BUTT     = 0x38,   //    Door gate button
  

// Limit switch     0x5x    7bytes
  LIM_SWITCH      = 0x50,
  FRNTDR_CLS      = 0x51,   //    Door gate closed ?
  PILLDR_OPN      = 0x52,   //    Pillbox door ?
  ARM_X_STP1      = 0x54,   //    top arm X-dir limit switch 1
  ARM_X_STP2      = 0x55,   //    top arm X-dir limit switch 2
  ARM_Z_STP1      = 0x56,   //    top arm Z-dir limit switch 1
  ARM_Z_STP2      = 0x57,   //    top arm Z-dir limit switch 2
  
  
// Stepper Ctrl     0x90    2byte * 5
  STP_X           = 0x90,   //    2byte
  STP_Z           = 0x92,   //    2byte
  STP_Y           = 0x94,   //    2byte
  STP_CAN         = 0x96,   //    2byte
  STP_PILLBX      = 0x98,   //    2byte

// Actuator Ctrl    
  FRNTDR_CTRL     = 0x9d,   //    Door gate control
  PNEU_PUMP       = 0x9e,   //    pump control  
  PNEU_VALV       = 0x9f,   //    valve control

// Led Ctrl         0xa0
  FRNTDR_LED0     = 0xa0,   //    external module leds
  PILLBX_LLED0    = 0xa3,   //    external module leds
  PILLBX_RLED0    = 0xa6,   //    external module leds
  PANEL_LED0      = 0xb0,   //    WS2813 control on top 24 leds (3*24 = 72)

// flag


// Extra Module Ctrl
// SPI1    RFID_NSS SCK MOIS MISO  comm to NFC module
// PB6,7    RFID_IRQ, RFID_NRST    interrupt and reset
// SPI2    AD_SCK DIN      comm to DAC for stepper (reserved)
// PD2, 0    + AD_LDAC, AD_SYNC  (GPIO)

// Communication

// USART1  DFU_TX RX    A33 upload program to STM (reserved only)
// USART2  CMD_TX RX    main comm between STM to A33
// USART6  DEBUG_TX RX    STM debug to PC

} regAddr_t;


typedef enum {
ctrlStepperX    = 1,
  ctrlStepperZ    = 2,
  rotateCan       = 3,   // 4 - x y (y 0.1 deg) (0.0-359.9)
  rotatePillbox   = 0x04,   // 2
  ctrlChuteDoor   = 0x05,   // 1
  ctrlPump        = 0x06,   // 1
  ctrlLED         = 0x07,   // 72
  ctrlGate        = 0x08,   // 1
  readAllLimits   = 0x09,   // send 7
  readAllSensors  = 0x0a,   // 20?
  readNFC         = 0x0b,   // 4
  checkPillbox    = 0x0c,
  checkGateOpen   = 0x0d,
  checkMachineStatus = 0x0e,
  ctriticalError  = 16,
  moveLimStepperX = 17,
  moveLimStepperZ = 18,
  rotateLimCan    = 19,
  movePnP         = 20,   // 4 - x z dist in mm?
  dispense        = 21,
  pickAndPlace    = 22,
  reviewCanister  = 23,
	stopDispense    =	25,
	battery 				= 27,
  none            = 30,
} header_t;


typedef enum {
  overTemp  = 1,
  tilted    = 2,
  overHumid = 3,
  nopillbox = 4,
  noNFC     = 5,
  retryfailed = 6,
} errorCode_t;

typedef enum {
  x   = 0,
  y   = 1,
  z   = 2,
  temp  = 3,
  humid = 4,
  nfc     = 5,
  gate    = 6,
  pillbox = 7,
  tiltAngle  = 8,
  systemTime = 9,
  firmwareVersion = 10,
} machineStatus;



typedef struct {
  uint8_t   h;
  regAddr_t r;
  uint8_t   l;
} func_t;


typedef enum {
  Idle            = 0,
  ReceiveData     = 2,
  SetParameter    = 3,
  SendData        = 4,
} protocolStatus;


extern uint8_t rxBuff[10];
extern uint8_t rxBuffLen;

extern uint8_t dataStream[256];
extern protocolStatus protocolState;

extern uint8_t ARM_Z_dir, ARM_X_dir;

void protocolInit(void);
void protocolReset(void);
void dataAnalysis(void);
void protocolActionCallback(uint8_t addr);
void sendMachineStatus(void);


extern func_t func[funcListLen];
#endif





