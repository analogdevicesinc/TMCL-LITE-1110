/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  stepRocker.h
           Definitions of globally used data types and macros

   Copyright (C) 2016 TRINAMIC Motion Control GmbH & Co KG
                      Waterloohain 5
                      D - 22769 Hamburg, Germany
                      http://www.trinamic.com/

   This program is free software; you can redistribute it and/or modify it
   freely.

   This program is distributed "as is" in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

/**
  \file stepRocker.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Basic type and macro definitions

  This file contains basic type and macro definitions needed by all
  modules of this project.
*/

typedef unsigned char UCHAR;                   //!< 8 bits unsigned
typedef unsigned short USHORT;                 //!< 16 bits unsigned
typedef unsigned int UINT;                     //!< 32 bits unsigned

#define TRUE 1
#define FALSE 0

#define SW_TYPE_HIGH 0x04                      //!< module number (1110) high byte
#define SW_TYPE_LOW  0x56                      //!< module number (1110) low byte

#define SW_VERSION_HIGH 0x01                   //!< software version high byte
#define SW_VERSION_LOW  0x01                   //!< software version low byte

#define N_O_MOTORS 1                           //!< number of motors supported by this module
#define MOTOR_NUMBER(a) (a)                    //!< extract TMC429 motor number (for future extensions)
#define WHICH_262(a) (a)                       //!< extract TMC262 motor number (for future extensions)

#if defined(MK20DX128)
  #define DISABLE_DRIVERS() GPIOD_PSOR = BIT2    //!< turn off all motor drivers
  #define ENABLE_DRIVERS()  GPIOD_PCOR = BIT2    //!< turn on all motor drivers

  #define LED1_ON()       GPIOA_PSOR = BIT5      //!< turn on LED 1
  #define LED1_OFF()      GPIOA_PCOR = BIT5      //!< turn off LED 1
  #define LED1_TOGGLE()   GPIOA_PTOR = BIT5      //!< toggle LED 1

  #define LED2_ON()       GPIOE_PSOR = BIT5      //!< turn on LED 2
  #define LED2_OFF()      GPIOE_PCOR = BIT5      //!< turn off LED 2
  #define LED2_TOGGLE()   GPIOE_PTOR = BIT5      //!< toggle LED 2

  #define SPI_DEV_EEPROM  0x0001                 //!< SPI device number of the EEPROM
  #define SPI_DEV_TMC429  0x0101                 //!< SPI device number of TMC429
  #define SPI_DEV_TMC262  0x0102                 //!< SPI device number of TMC262

#elif defined(GD32F425)
  #define ENABLE_DRIVERS()          GPIO_BC(GPIOE)=BIT1
  #define DISABLE_DRIVERS()         GPIO_BOP(GPIOE)=BIT1

  #define LED1_ON()                 GPIO_BOP(GPIOC)=BIT5
  #define LED1_OFF()                GPIO_BC(GPIOC)=BIT5
  #define LED1_TOGGLE()             GPIO_TG(GPIOC)=BIT5
  #define LED2_ON()                 GPIO_BOP(GPIOC)=BIT4
  #define LED2_OFF()                GPIO_BC(GPIOC)=BIT4
  #define LED2_TOGGLE()             GPIO_TG(GPIOC)=BIT4

  #define SPI_DEV_EEPROM              1
  #define SPI_DEV_TMC429              2
  #define SPI_DEV_TMC262              3

  #define SELECT_EEPROM()       GPIO_BC(GPIOA)=BIT15
  #define DESELECT_EEPROM()     GPIO_BOP(GPIOA)=BIT15
  #define SELECT_TMC429()       GPIO_BC(GPIOA)=BIT4
  #define DESELECT_TMC429()     GPIO_BOP(GPIOA)=BIT4
  #define SELECT_TMC262()       GPIO_BC(GPIOC)=BIT3
  #define DESELECT_TMC262()     GPIO_BOP(GPIOC)=BIT3

#endif

//! Global module settings (here only RS485 settings)
typedef struct
{
  UCHAR SerialBitrate;         //!< RS485 baud rate (0..7, 0=9600bps)
  UCHAR SerialModuleAddress;   //!< RS485 TMCL module address
  UCHAR SerialHostAddress;     //!< RS485 TMCL reply address
  UCHAR CANReceiveID;          //!< CAN receive ID
  UCHAR CANSendID;             //!< CAN send ID
  UCHAR CANBitrate;            //!< CAN bit rate
} TModuleConfig;

//! Motor configuration data
typedef struct
{
  UINT VMax;                   //!< maximum positioning velocity
  UINT AMax;                   //!< maximum acceleration
  UCHAR PulseDiv;              //!< pulse divisor (TMC429)
  UCHAR RampDiv;               //!< ramp divisor (TMC429)
  UCHAR IRun;                  //!< run current (0..255)
  UCHAR IStandby;              //!< stand-by current(0..255)
  UINT StallVMin;              //!< minimum speed for stallGuard
  UINT FreewheelingDelay;      //!< freewheeling delay time (*10ms)
  UINT SettingDelay;           //!< delay for switching to stand-by current (*10ms)
} TMotorConfig;

//! coolStep configuration data
typedef struct
{
  UINT ThresholdSpeed;         //!< coolStep threshold speed
  UCHAR SlowRunCurrent;        //!< coolStep slow run current
  UCHAR HysteresisStart;       //!< coolStep hysteresis start
} TCoolStepConfig;
