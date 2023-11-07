/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides all functions needed for easy
  access to the TMC262 stepper motor driver IC.
*/

#include <stdlib.h>
#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
#endif
#include "bits.h"
#include "stepRocker.h"
#include "TMC262.h"
#include "SPI.h"


//! Data structure for software copy of the TMC262 DRVCTRL register
typedef struct
{
  UCHAR Intpol;
  UCHAR DEdge;
  UCHAR MRes;
} TStepDirConfig;

//! Data structure for software copy of the TMC262 CHOPCONF register
typedef struct
{
  UCHAR BlankTime;
  UCHAR ChopperMode;
  UCHAR HysteresisDecay;
  UCHAR RandomTOff;
  UCHAR HysteresisEnd;
  UCHAR HysteresisStart;
  UCHAR TOff;
  UCHAR DisableFlag;
} TChopperConfig;

//! Data structure for software copy of the TMC262 SMARTEN register
typedef struct
{
  UCHAR SmartIMin;
  UCHAR SmartDownStep;
  UCHAR SmartStallLevelMax;
  UCHAR SmartUpStep;
  UCHAR SmartStallLevelMin;
} TSmartEnergyControl;

//! Data structure for software copy of the TMC262 SGSCONF register
typedef struct
{
  UCHAR FilterEnable;
  signed char StallGuardThreshold;
  UCHAR CurrentScale;
} TStallGuardConfig;

//! Data structure for software copy of the TMC262 DRVCONF register
typedef struct
{
  UCHAR SlopeHighSide;
  UCHAR SlopeLowSide;
  UCHAR ProtectionDisable;
  UCHAR ProtectionTimer;
  UCHAR StepDirectionDisable;
  UCHAR VSenseScale;
  UCHAR ReadBackSelect;
} TDriverConfig;

//! Possible TMC262 read back modes
typedef enum
{RB_CHOPPER, RB_DRIVER, RB_SMART_ENERGY, RB_STALL_GUARD, RB_STEP_DIR} TReadBackDatagram;

//Global variables of this module
static TStepDirConfig StepDirConfig[N_O_MOTORS];           //!< Software copies of the DRVCTRL registers of all TMC26x ICs
static TChopperConfig ChopperConfig[N_O_MOTORS];           //!< Software copies of the CHOPCONF registers of all TMC26x ICs
static TSmartEnergyControl SmartEnergyControl[N_O_MOTORS]; //!< Software copies of the SMARTEN registers of all TMC26x ICs
static TStallGuardConfig StallGuardConfig[N_O_MOTORS];     //!< Software copies of the SGSCONF registers of all TMC26x ICs
static TDriverConfig DriverConfig[N_O_MOTORS];             //!< Software copies of the DRVCONF registers of all TMC26x ICs
static TReadBackDatagram ReadBackDatagram[N_O_MOTORS];     //!< Next datagram type used for reading the actual state of the TMC26x
static UINT SPIReadInt;                                    //!< SPI read buffer
static UINT SPIWriteInt;                                   //!< SPI write buffer


//Functions

/***************************************************************//**
   \fn ReadWrite262(UCHAR Which262, UINT *ReadInt, UINT WriteInt)
   \brief SPI communictation with TMC26x
   \param Which262  Index of TMC26x to be used (with stepRocker always 0)
   \param ReadInt   Pointer at UINT for the result (read from TMC26x)
   \param WriteInt  Value to be written to the TMC26x

  This is the low level TMC26x communication function. It can be
  extended for use with more than one TMC26x.
********************************************************************/
static void ReadWrite262(UCHAR Which262, UINT *ReadInt, UINT WriteInt)
{
  *ReadInt=ReadWriteSPI(SPI_DEV_TMC262, WriteInt>>16, FALSE);
  *ReadInt<<=8;
  *ReadInt|=ReadWriteSPI(SPI_DEV_TMC262, WriteInt>>8, FALSE);
  *ReadInt<<=8;
  *ReadInt|=ReadWriteSPI(SPI_DEV_TMC262, WriteInt & 0xff, TRUE);
  *ReadInt>>=4;
}


/***************************************************************//**
   \fn WriteStepDirConfig(UCHAR Which262)
   \brief Write DRVCTRL register data to TMC26x
   \param Which262  Index of TMC26x to be used (with stepRocker always 0)

  This is the low level function for writing data to the DRVCTRL
  register.
********************************************************************/
static void WriteStepDirConfig(UCHAR Which262)
{
  SPIWriteInt=0;
  if(StepDirConfig[Which262].Intpol) SPIWriteInt|=BIT9;
  if(StepDirConfig[Which262].DEdge) SPIWriteInt|=BIT8;
  if(StepDirConfig[Which262].MRes>15) StepDirConfig[Which262].MRes=15;
  SPIWriteInt|=StepDirConfig[Which262].MRes;

  ReadWrite262(Which262, &SPIReadInt, SPIWriteInt);
}


/***************************************************************//**
   \fn WriteChopperConfig(UCHAR Which262)
   \brief Write CHOPCONF register data to TMC26x
   \param Which262  Index of TMC26x to be used (with stepRocker always 0)

  This is the low level function for writing data to the CHOPCONF
  register.
********************************************************************/
static void WriteChopperConfig(UCHAR Which262)
{
  if(ChopperConfig[Which262].BlankTime>3) ChopperConfig[Which262].BlankTime=3;
  if(ChopperConfig[Which262].HysteresisDecay>3) ChopperConfig[Which262].HysteresisDecay=3;
  if(ChopperConfig[Which262].HysteresisEnd>15) ChopperConfig[Which262].HysteresisEnd=15;
  if(ChopperConfig[Which262].HysteresisStart>7) ChopperConfig[Which262].HysteresisStart=7;
  if(ChopperConfig[Which262].TOff>15) ChopperConfig[Which262].TOff=15;

  SPIWriteInt=0;
  SPIWriteInt|=BIT19;  //Registeraddresse CHOPCONF;
  SPIWriteInt|=((UINT) ChopperConfig[Which262].BlankTime) << 15;
  if(ChopperConfig[Which262].ChopperMode) SPIWriteInt|=BIT14;
  if(ChopperConfig[Which262].RandomTOff) SPIWriteInt|=BIT13;
  SPIWriteInt|=((UINT) ChopperConfig[Which262].HysteresisDecay) << 11;
  SPIWriteInt|=((UINT) ChopperConfig[Which262].HysteresisEnd) << 7;
  SPIWriteInt|=((UINT) ChopperConfig[Which262].HysteresisStart) << 4;
  if(!ChopperConfig[Which262].DisableFlag) SPIWriteInt|=((UINT) ChopperConfig[Which262].TOff);  //wenn DisableFlag gesetzt wird 0 gesendet

  ReadWrite262(Which262, &SPIReadInt, SPIWriteInt);
}


/***************************************************************//**
   \fn WriteSmartEnergyControl(UCHAR Which262)
   \brief Write SMARTEN register data to TMC26x
   \param Which262  Index of TMC26x to be used (with stepRocker always 0)

  This is the low level function for writing data to the SMARTEN
  register.
********************************************************************/
static void WriteSmartEnergyControl(UCHAR Which262)
{
  if(SmartEnergyControl[Which262].SmartIMin>1) SmartEnergyControl[Which262].SmartIMin=1;
  if(SmartEnergyControl[Which262].SmartDownStep>3) SmartEnergyControl[Which262].SmartDownStep=3;
  if(SmartEnergyControl[Which262].SmartStallLevelMax>15) SmartEnergyControl[Which262].SmartStallLevelMax=15;
  if(SmartEnergyControl[Which262].SmartUpStep>3) SmartEnergyControl[Which262].SmartUpStep=3;
  if(SmartEnergyControl[Which262].SmartStallLevelMin>15) SmartEnergyControl[Which262].SmartStallLevelMin=15;

  SPIWriteInt=0;
  SPIWriteInt|=BIT19|BIT17;  //Registeradresse SMARTEN
  SPIWriteInt|=((UINT) SmartEnergyControl[Which262].SmartIMin) << 15;
  SPIWriteInt|=((UINT) SmartEnergyControl[Which262].SmartDownStep)  << 13;
  SPIWriteInt|=((UINT) SmartEnergyControl[Which262].SmartStallLevelMax) << 8;
  SPIWriteInt|=((UINT) SmartEnergyControl[Which262].SmartUpStep) << 5;
  SPIWriteInt|=((UINT) SmartEnergyControl[Which262].SmartStallLevelMin);

  ReadWrite262(Which262, &SPIReadInt, SPIWriteInt);
}


/***************************************************************//**
   \fn WriteStallGuardConfig(UCHAR Which262)
   \brief Write SGSCONF register data to TMC26x
   \param Which262  Index of TMC26x to be used (with stepRocker always 0)

  This is the low level function for writing data to the SGSCONF
  register.
********************************************************************/
static void WriteStallGuardConfig(UCHAR Which262)
{
  if(StallGuardConfig[Which262].StallGuardThreshold>63) StallGuardConfig[Which262].StallGuardThreshold=63;
  if(StallGuardConfig[Which262].StallGuardThreshold<-63) StallGuardConfig[Which262].StallGuardThreshold=-63;
  if(StallGuardConfig[Which262].CurrentScale>31) StallGuardConfig[Which262].CurrentScale=31;

  SPIWriteInt=0;
  SPIWriteInt|=BIT19|BIT18;  //Registeradresse SGSCONF
  if(StallGuardConfig[Which262].FilterEnable==1) SPIWriteInt|=BIT16;
  SPIWriteInt|=((UINT) StallGuardConfig[Which262].StallGuardThreshold & 0x7f) << 8;
  SPIWriteInt|=((UINT) StallGuardConfig[Which262].CurrentScale);

  ReadWrite262(Which262, &SPIReadInt, SPIWriteInt);
}


/***************************************************************//**
   \fn WriteDriverConfig(UCHAR Which262)
   \brief Write DRVCONF register data to TMC26x
   \param Which262  Index of TMC26x to be used (with stepRocker always 0)

  This is the low level function for writing data to the DRVCONF
  register.
********************************************************************/
static void WriteDriverConfig(UCHAR Which262)
{
  SPIWriteInt=0;
  SPIWriteInt|=BIT19|BIT18|BIT17;  //Registeradresse DRVCONF
  SPIWriteInt|=((UINT) DriverConfig[Which262].SlopeHighSide) << 14;
  SPIWriteInt|=((UINT) DriverConfig[Which262].SlopeLowSide) << 12;
  if(DriverConfig[Which262].ProtectionDisable==1) SPIWriteInt|=BIT10;
  SPIWriteInt|=((UINT) DriverConfig[Which262].ProtectionTimer) << 8;
  if(DriverConfig[Which262].StepDirectionDisable==1) SPIWriteInt|=BIT7;
  if(DriverConfig[Which262].VSenseScale==1) SPIWriteInt|=BIT6;
  SPIWriteInt|=((UINT) DriverConfig[Which262].ReadBackSelect) << 4;

  ReadWrite262(Which262, &SPIReadInt, SPIWriteInt);
}


/***************************************************************//**
   \fn InitMotorDrivers(void)
   \brief Initialise all motor drivers

   This function initalizes the software copies of all TMC26x
   registers and sends this basic initialization data to all
   TMC26x ICs.
********************************************************************/
void InitMotorDrivers(void)
{
  UINT i;

  //Initialize data structures
  for(i=0; i<N_O_MOTORS; i++)
  {
    StallGuardConfig[i].FilterEnable=1;
    StallGuardConfig[i].StallGuardThreshold=5;
    StallGuardConfig[i].CurrentScale=5; //16;

    DriverConfig[i].SlopeHighSide=3;
    DriverConfig[i].SlopeLowSide=3;
    DriverConfig[i].ProtectionDisable=0;
    DriverConfig[i].ProtectionTimer=0;
    DriverConfig[i].StepDirectionDisable=0;
    DriverConfig[i].VSenseScale=1;
    DriverConfig[i].ReadBackSelect=TMC262_RB_SMART_ENERGY;

    SmartEnergyControl[i].SmartIMin=0;
    SmartEnergyControl[i].SmartDownStep=0;
    SmartEnergyControl[i].SmartStallLevelMax=0;
    SmartEnergyControl[i].SmartUpStep=0;
    SmartEnergyControl[i].SmartStallLevelMin=0;

    StepDirConfig[i].Intpol=0;
    StepDirConfig[i].DEdge=0;
    StepDirConfig[i].MRes=0;

    ChopperConfig[i].BlankTime=2;
    ChopperConfig[i].ChopperMode=0;
    ChopperConfig[i].RandomTOff=0;
    ChopperConfig[i].HysteresisDecay=0;
    ChopperConfig[i].HysteresisEnd=2;
    ChopperConfig[i].HysteresisStart=3;
    ChopperConfig[i].TOff=5;
    ChopperConfig[i].DisableFlag=FALSE;
  }

  ENABLE_DRIVERS();

  //Send the values to the ICs
  for(i=0; i<N_O_MOTORS; i++)
  {
    WriteSmartEnergyControl(WHICH_262(i));
    WriteStallGuardConfig(WHICH_262(i));
    WriteDriverConfig(WHICH_262(i));
    WriteStepDirConfig(WHICH_262(i));
    WriteChopperConfig(WHICH_262(i));
  }
}


/*************************************************************************//**
  \fn Set262StepDirMStepRes(UCHAR Which262, UCHAR MicrostepResolution)
  \brief Set the microstep resolution
  \param Which262  Axis number (with stepRocker always 0)
  \param MicrostepResolution  micrstep resolution (0..8)

  This function sets the microstep resolution, where 0 selects the highest
  resolution (256 microsteps) and 8 the lowest (full step).
*****************************************************************************/
void Set262StepDirMStepRes(UCHAR Which262, UCHAR MicrostepResolution)
{
  StepDirConfig[Which262].MRes=MicrostepResolution;
  if(MicrostepResolution!=4) StepDirConfig[Which262].Intpol=0;

  WriteStepDirConfig(Which262);
}


/*************************************************************************//**
  \fn Set262StepDirInterpolation(UCHAR Which262, UCHAR Interpolation)
  \brief Turn interpolation mode on or off
  \param Which262  Axis number (with stepRocker always 0)
  \param Interpolation  Interpolation mode (0=off, 1=on)

  This function turns interpolation mode on or off. Interpolation mode only
  works with microstep resolution set to 16 microsteps.
*****************************************************************************/
void Set262StepDirInterpolation(UCHAR Which262, UCHAR Interpolation)
{
  StepDirConfig[Which262].Intpol=Interpolation;
  if(Interpolation) StepDirConfig[Which262].MRes=4;

  WriteStepDirConfig(Which262);
}


/*************************************************************************//**
  \fn Set262StepDirDoubleEdge(UCHAR Which262, UCHAR DoubleEdge)
  \brief Set double step mode
  \param Which262  Axis number (with stepRocker always 0)
  \param DoubleEdge  0=double step off, 1=double step on

  This function sets the double step mode, so that with every step pulse
  two microsteps will be executed (one with the rising edge end one with
  the falling edge of every pulse).
*****************************************************************************/
void Set262StepDirDoubleEdge(UCHAR Which262, UCHAR DoubleEdge)
{
  StepDirConfig[Which262].DEdge=DoubleEdge;

  WriteStepDirConfig(Which262);
}


/*************************************************************************//**
  \fn Get262StepDirMStepRes(UCHAR Which262)
  \brief Get microstep resolution
  \param Which262  Axis number (with stepRocker always 0)
  \return Microstep resolution

  This function reads back the microstep resolution value from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262StepDirMStepRes(UCHAR Which262)
{
  return StepDirConfig[Which262].MRes;
}


/*************************************************************************//**
  \fn Get262StepDirInterpolation(UCHAR Which262)
  \brief Get interpolation mode
  \param Which262  Axis number (with stepRocker always 0)
  \return Interpolation mode

  This function reads back the microstep interpolation mode from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262StepDirInterpolation(UCHAR Which262)
{
  return StepDirConfig[Which262].Intpol;
}

/*************************************************************************//**
  \fn Get262StepDirDoubleEdge(UCHAR Which262)
  \brief Get microstep resolution
  \param Which262  Axis number (with stepRocker always 0)
  \return Double step mode

  This function reads back the double step mode from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262StepDirDoubleEdge(UCHAR Which262)
{
  return StepDirConfig[Which262].DEdge;
}


/*************************************************************************//**
  \fn Set262ChopperBlankTime(UCHAR Which262, UCHAR BlankTime)
  \brief Set chopper blank time
  \param Which262   Axis number (with stepRocker always 0)
  \param BlankTime  chopper blank time

  This function sets the chopper blank time.
*****************************************************************************/
void Set262ChopperBlankTime(UCHAR Which262, UCHAR BlankTime)
{
  ChopperConfig[Which262].BlankTime=BlankTime;
  WriteChopperConfig(Which262);
}


/*************************************************************************//**
  \fn Set262ChopperMode(UCHAR Which262, UCHAR Mode)
  \brief Set chopper mode
  \param Which262  Axis number (with stepRocker always 0)
  \param Mode      chopper mode (0=spread cycle mode, 1=classic mode)

  This function sets the chopper mode.
*****************************************************************************/
void Set262ChopperMode(UCHAR Which262, UCHAR Mode)
{
  ChopperConfig[Which262].ChopperMode=Mode;
  WriteChopperConfig(Which262);
}


/*************************************************************************//**
  \fn Set262ChopperRandomTOff(UCHAR Which262, UCHAR RandomTOff)
  \brief Set random TOff mode
  \param Which262    Axis number (with stepRocker always 0)
  \param RandomTOff  Random TOff mode (0=off, 1=on)

  This function turns random TOff mode on or off.
*****************************************************************************/
void Set262ChopperRandomTOff(UCHAR Which262, UCHAR RandomTOff)
{
  ChopperConfig[Which262].RandomTOff=RandomTOff;
  WriteChopperConfig(Which262);
}


/*************************************************************************//**
  \fn Set262ChopperHysteresisDecay(UCHAR Which262, UCHAR HysteresisDecay)
  \brief Set chopper hysteresis decay parameter (HDEC)
  \param Which262         Axis number (with stepRocker always 0)
  \param HysteresisDecay  HDEC parameter

  This function sets the chopper hysteresis decrement parameter (HDEC).
*****************************************************************************/
void Set262ChopperHysteresisDecay(UCHAR Which262, UCHAR HysteresisDecay)
{
  ChopperConfig[Which262].HysteresisDecay=HysteresisDecay;
  WriteChopperConfig(Which262);
}


/*************************************************************************//**
  \fn Set262ChopperHysteresisEnd(UCHAR Which262, UCHAR HysteresisEnd)
  \brief Set chopper hysteresis end parameter (HEND)
  \param Which262       Axis number (with stepRocker always 0)
  \param HysteresisEnd  HEND parameter

  This function sets the chopper hysteresis end parameter (HEND).
*****************************************************************************/
void Set262ChopperHysteresisEnd(UCHAR Which262, UCHAR HysteresisEnd)
{
  ChopperConfig[Which262].HysteresisEnd=HysteresisEnd;
  WriteChopperConfig(Which262);
}


/*************************************************************************//**
  \fn Set262ChopperHysteresisStart(UCHAR Which262, UCHAR HysteresisStart)
  \brief Set chopper hysteresis start parameter (HSTART)
  \param Which262         Axis number (with stepRocker always 0)
  \param HysteresisStart  HSTART parameter

  This function sets the chopper hysteresis start parameter (HSTART).
*****************************************************************************/
void Set262ChopperHysteresisStart(UCHAR Which262, UCHAR HysteresisStart)
{
  ChopperConfig[Which262].HysteresisStart=HysteresisStart;
  WriteChopperConfig(Which262);
}


/*************************************************************************//**
  \fn Set262ChopperTOff(UCHAR Which262, UCHAR TOff)
  \brief Set chopper off time (TOff)
  \param Which262  Axis number (with stepRocker always 0)
  \param TOff      TOff parameter

  This function sets the chopper off time parameter (TOff).
*****************************************************************************/
void Set262ChopperTOff(UCHAR Which262, UCHAR TOff)
{
  ChopperConfig[Which262].TOff=TOff;
  WriteChopperConfig(Which262);
}


/*************************************************************************//**
  \fn Get262ChopperBlankTime(UCHAR Which262)
  \brief Get chopper blank time
  \param Which262  Axis number (with stepRocker always 0)
  \return  chopper blank time

  This function reads back the chopper blank time from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262ChopperBlankTime(UCHAR Which262)
{
  return ChopperConfig[Which262].BlankTime;
}


/*************************************************************************//**
  \fn Get262ChopperMode(UCHAR Which262)
  \brief Get chopper mode
  \param Which262  Axis number (with stepRocker always 0)
  \return Chopper mode

  This function reads back the chopper mode from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262ChopperMode(UCHAR Which262)
{
  return ChopperConfig[Which262].ChopperMode;
}


/*************************************************************************//**
  \fn Get262ChopperRandomTOff(UCHAR Which262)
  \brief Get random TOff mode
  \param Which262  Axis number (with stepRocker always 0)
  \return Random TOff mode

  This function reads back the random TOff mode from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262ChopperRandomTOff(UCHAR Which262)
{
  return ChopperConfig[Which262].RandomTOff;
}


/*************************************************************************//**
  \fn Get262ChopperHysteresisDecay(UCHAR Which262)
  \brief Get chopper hysteresis decay (HDEC)
  \param Which262  Axis number (with stepRocker always 0)
  \return HDEC value

  This function reads back the chopper hysteresis decay value from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262ChopperHysteresisDecay(UCHAR Which262)
{
  return ChopperConfig[Which262].HysteresisDecay;
}


/*************************************************************************//**
  \fn Get262ChopperHysteresisEnd(UCHAR Which262)
  \brief Get chopper hysteresis end (HEND)
  \param Which262  Axis number (with stepRocker always 0)
  \return HEND value

  This function reads back the chopper hysteresis end value from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262ChopperHysteresisEnd(UCHAR Which262)
{
  return ChopperConfig[Which262].HysteresisEnd;
}


/*************************************************************************//**
  \fn Get262ChopperHysteresisStart(UCHAR Which262)
  \brief Get chopper hysteresis start (HSTART)
  \param Which262  Axis number (with stepRocker always 0)
  \return HSTART value

  This function reads back the chopper hysteresis start value from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262ChopperHysteresisStart(UCHAR Which262)
{
  return ChopperConfig[Which262].HysteresisStart;
}


/*************************************************************************//**
  \fn Get262ChopperTOff(UCHAR Which262)
  \brief Get chopper off time value
  \param Which262  Axis number (with stepRocker always 0)
  \return TOff value

  This function reads back the TOff value from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262ChopperTOff(UCHAR Which262)
{
  return ChopperConfig[Which262].TOff;
}


/*************************************************************************//**
  \fn Set262SmartEnergyIMin(UCHAR Which262, UCHAR SmartIMin)
  \brief Set smart energy minimum current
  \param Which262   Axis number (with stepRocker always 0)
  \param SmartIMin  Minimum current (0=1/2, 1=1/4 of current setting)

  This function sets the minimum current used with coolStep, which can
  be either 1/2 or 1/4 of the normal current setting.
*****************************************************************************/
void Set262SmartEnergyIMin(UCHAR Which262, UCHAR SmartIMin)
{
  SmartEnergyControl[Which262].SmartIMin=SmartIMin;
  WriteSmartEnergyControl(Which262);
}


/*************************************************************************//**
  \fn Set262SmartEnergyDownStep(UCHAR Which262, UCHAR SmartDownStep)
  \brief Set smart energy down step
  \param Which262       Axis number (with stepRocker always 0)
  \param SmartDownStep  down step speed (0..3)

  This function sets the current down step speed used with coolStep, where 0 ist
  the highest and 3 is the lowest speed.
*****************************************************************************/
void Set262SmartEnergyDownStep(UCHAR Which262, UCHAR SmartDownStep)
{
  SmartEnergyControl[Which262].SmartDownStep=SmartDownStep;
  WriteSmartEnergyControl(Which262);
}


/*************************************************************************//**
  \fn Set262SmartEnergyStallLevelMax(UCHAR Which262, UCHAR StallLevelMax)
  \brief Set smart enery hysteresis width
  \param Which262       Axis number (with stepRocker always 0)
  \param StallLevelMax  hysteresis width (0..15)

  This function sets the SEMAX parameter which defines the width of the
  smart energy stall level hysteresis.
*****************************************************************************/
void Set262SmartEnergyStallLevelMax(UCHAR Which262, UCHAR StallLevelMax)
{
  SmartEnergyControl[Which262].SmartStallLevelMax=StallLevelMax;
  WriteSmartEnergyControl(Which262);
}


/*************************************************************************//**
  \fn Set262SmartEnergyUpStep(UCHAR Which262, UCHAR SmartUpStep)
  \brief Set smart energy up step
  \param Which262     Axis number (with stepRocker always 0)
  \param SmartUpStep  up step width (0..3)

  This function sets the current up step width used with coolStep, where 0 ist
  the lowest and 3 is the highest up step width.
*****************************************************************************/
void Set262SmartEnergyUpStep(UCHAR Which262, UCHAR SmartUpStep)
{
  SmartEnergyControl[Which262].SmartUpStep=SmartUpStep;
  WriteSmartEnergyControl(Which262);
}


/*************************************************************************//**
  \fn Set262SmartEnergyStallLevelMin(UCHAR Which262, UCHAR StallLevelMin)
  \brief Set smart energy hysteresis start
  \param Which262       Axis number (with stepRocker always 0)
  \param StallLevelMin  minimum stall level (0..15)

  This function sets the start point of the hysteresis used for coolStep.
  A value of 0 completely turns off coolStep.
*****************************************************************************/
void Set262SmartEnergyStallLevelMin(UCHAR Which262, UCHAR StallLevelMin)
{
  SmartEnergyControl[Which262].SmartStallLevelMin=StallLevelMin;
  WriteSmartEnergyControl(Which262);
}


/*************************************************************************//**
  \fn Get262SmartEnergyIMin(UCHAR Which262)
  \brief Get minimum current
  \param Which262  Axis number (with stepRocker always 0)
  \return Minimum currrent

  This function reads back the smart energy minimum current setting from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262SmartEnergyIMin(UCHAR Which262)
{
  return SmartEnergyControl[Which262].SmartIMin;
}


/*************************************************************************//**
  \fn Get262SmartEnergyDownStep(UCHAR Which262)
  \brief Get current down step speed
  \param Which262  Axis number (with stepRocker always 0)
  \return Current down step speed

  This function reads back the smart energy current down step speed setting from
  the software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262SmartEnergyDownStep(UCHAR Which262)
{
  return SmartEnergyControl[Which262].SmartDownStep;
}


/*************************************************************************//**
  \fn Get262SmartEnergyStallLevelMax(UCHAR Which262)
  \brief Get hystersis width
  \param Which262  Axis number (with stepRocker always 0)
  \return SEMAX value

  This function reads back the stall level maximum value (which is the coolStep
  hysteresis width9 from the software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262SmartEnergyStallLevelMax(UCHAR Which262)
{
  return SmartEnergyControl[Which262].SmartStallLevelMax;
}


/*************************************************************************//**
  \fn Get262SmartEnergyUpStep(UCHAR Which262)
  \brief Get current up step width
  \param Which262  Axis number (with stepRocker always 0)
  \return SEUP value

  This function reads back the current up step setting from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262SmartEnergyUpStep(UCHAR Which262)
{
  return SmartEnergyControl[Which262].SmartUpStep;
}


/*************************************************************************//**
  \fn Get262SmartEnergyStallLevelMin(UCHAR Which262)
  \brief Get hysteresis start
  \param Which262  Axis number (with stepRocker always 0)
  \return hysteresis start

  This function reads back the smart energy minimum stall level (which is the
  start of the coolStep hystetesis) from the software copy of the TMC26x
  registers.
*****************************************************************************/
UCHAR Get262SmartEnergyStallLevelMin(UCHAR Which262)
{
  return SmartEnergyControl[Which262].SmartStallLevelMin;
}


/*************************************************************************//**
  \fn Set262StallGuardFilter(UCHAR Which262, UCHAR Enable)
  \brief Set stallGuard filter
  \param Which262          Axis number (with stepRocker always 0)
  \param Enable            stallGuard filter (0=off, 1=on)

  This function turns the stallGuard filter on or off.
*****************************************************************************/
void Set262StallGuardFilter(UCHAR Which262, UCHAR Enable)
{
  StallGuardConfig[Which262].FilterEnable=Enable;
  WriteStallGuardConfig(Which262);
}


/*************************************************************************//**
  \fn Set262StallGuardThreshold(UCHAR Which262, signed char Threshold)
  \brief Set stallGuard threshold value
  \param Which262    Axis number (with stepRocker always 0)
  \param Threshold   stallGuard threshold (-63..+63)

  This function sets the stallGuard threshold value.
*****************************************************************************/
void Set262StallGuardThreshold(UCHAR Which262, signed char Threshold)
{
  StallGuardConfig[Which262].StallGuardThreshold=Threshold;
  WriteStallGuardConfig(Which262);
}


/*************************************************************************//**
  \fn Set262StallGuardCurrentScale(UCHAR Which262, UCHAR CurrentScale)
  \brief Set current scale value
  \param Which262       Axis number (with stepRocker always 0)
  \param CurrentScale   Current scale value (0..31)

  This function sets the maximum motor current.
*****************************************************************************/
void Set262StallGuardCurrentScale(UCHAR Which262, UCHAR CurrentScale)
{
  StallGuardConfig[Which262].CurrentScale=CurrentScale;
  WriteStallGuardConfig(Which262);
}


/*************************************************************************//**
  \fn Get262StallGuardFilter(UCHAR Which262)
  \brief Get stallGuard filter
  \param Which262  Axis number (with stepRocker always 0)
  \return stallGuard filter (0=off, 1=on)

  This function reads back the stallGuard filter settings from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262StallGuardFilter(UCHAR Which262)
{
  return StallGuardConfig[Which262].FilterEnable;
}


/*************************************************************************//**
  \fn Get262StallGuardThreshold(UCHAR Which262)
  \brief Get stallGuard threshold setting
  \param Which262  Axis number (with stepRocker always 0)
  \return stallGuard threshold value

  This function reads back the stallGuard thresholg value from the
  software copy of the TMC26x registers.
*****************************************************************************/
signed char Get262StallGuardThreshold(UCHAR Which262)
{
  return StallGuardConfig[Which262].StallGuardThreshold;
}


/*************************************************************************//**
  \fn Get262StallGuardCurrentScale(UCHAR Which262)
  \brief Get motor current setting
  \param Which262  Axis number (with stepRocker always 0)
  \return Motor current setting

  This function reads back the motor current setting from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262StallGuardCurrentScale(UCHAR Which262)
{
  return StallGuardConfig[Which262].CurrentScale;
}


/*************************************************************************//**
  \fn Set262DriverSlopeHighSide(UCHAR Which262, UCHAR SlopeHighSide)
  \brief Set slope high side value (SLPH)
  \param Which262        Axis number (with stepRocker always 0)
  \param SlopeHighSide   Slope high side value

  This function sets the slope control high side value (SLPH).
*****************************************************************************/
void Set262DriverSlopeHighSide(UCHAR Which262, UCHAR SlopeHighSide)
{
  DriverConfig[Which262].SlopeHighSide=SlopeHighSide;
  WriteDriverConfig(Which262);
}


/*************************************************************************//**
  \fn Set262DriverSlopeLowSide(UCHAR Which262, UCHAR SlopeLowSide)
  \brief Set slope low side value (SLPL)
  \param Which262        Axis number (with stepRocker always 0)
  \param SlopeLowSide   Slope low side value

  This function sets the slope control low side value (SLPL).
*****************************************************************************/
void Set262DriverSlopeLowSide(UCHAR Which262, UCHAR SlopeLowSide)
{
  DriverConfig[Which262].SlopeLowSide=SlopeLowSide;
  WriteDriverConfig(Which262);
}


/*************************************************************************//**
  \fn Set262DriverDisableProtection(UCHAR Which262, UCHAR DisableProtection)
  \brief Disable short to GND protection
  \param Which262            Axis number (with stepRocker always 0)
  \param DisableProtection   0=protection enabled\n
                             1=protection disabled

  This function disables or enables the short to ground protection.
*****************************************************************************/
void Set262DriverDisableProtection(UCHAR Which262, UCHAR DisableProtection)
{
  DriverConfig[Which262].ProtectionDisable=DisableProtection;
  WriteDriverConfig(Which262);
}


/*************************************************************************//**
  \fn Set262DriverProtectionTimer(UCHAR Which262, UCHAR ProtectionTimer)
  \brief Set protection timer value
  \param Which262         Axis number (with stepRocker always 0)
  \param ProtectionTimer  Protection time (0..3)

  This function sets the short to ground protection timer value.
*****************************************************************************/
void Set262DriverProtectionTimer(UCHAR Which262, UCHAR ProtectionTimer)
{
  DriverConfig[Which262].ProtectionTimer=ProtectionTimer;
  WriteDriverConfig(Which262);
}


/*************************************************************************//**
  \fn Set262DriverStepDirectionOff(UCHAR Which262, UCHAR SDOff)
  \brief Disable/enable step direction input
  \param Which262   Axis number (with stepRocker always 0)
  \param SDOff      0=step direction input enable\n
                    1=step direction input disable

  This function turns the step direction input on or off.
*****************************************************************************/
void Set262DriverStepDirectionOff(UCHAR Which262, UCHAR SDOff)
{
  DriverConfig[Which262].StepDirectionDisable=SDOff;
  WriteDriverConfig(Which262);
}


/*************************************************************************//**
  \fn Set262DriverVSenseScale(UCHAR Which262, UCHAR Scale)
  \brief Sets the sense voltage
  \param Which262   Axis number (with stepRocker always 0)
  \param Scale      0=300mV\n
                    1=160mV

  This function sets the sense voltage.
*****************************************************************************/
void Set262DriverVSenseScale(UCHAR Which262, UCHAR Scale)
{
  DriverConfig[Which262].VSenseScale=Scale;
  WriteDriverConfig(Which262);
}


/*************************************************************************//**
  \fn Set262DriverReadSelect(UCHAR Which262, UCHAR ReadSelect)
  \brief Selects the values to be read
  \param Which262   Axis number (with stepRocker always 0)
  \param ReadSelect  readback mode (TMC262_RB_MSTEP, TMC262_RB_STALL_GUARD,
                     TMC262_RB_SMART_ENERGY)

  This selects which values are to be read back from the TMC26x with every
  SPI datagram (important for the Read262State() function).
*****************************************************************************/
void Set262DriverReadSelect(UCHAR Which262, UCHAR ReadSelect)
{
  DriverConfig[Which262].ReadBackSelect=ReadSelect;
  WriteDriverConfig(Which262);
}


/*************************************************************************//**
  \fn Get262DriverSlopeHighSide(UCHAR Which262)
  \brief Get slope high side value (SLPH)
  \param Which262  Axis number (with stepRocker always 0)
  \return SLPH value

  This function reads back the SLPH value from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262DriverSlopeHighSide(UCHAR Which262)
{
  return DriverConfig[Which262].SlopeHighSide;
}


/*************************************************************************//**
  \fn Get262DriverSlopeLowSide(UCHAR Which262)
  \brief Get slope high side value (SLPL)
  \param Which262  Axis number (with stepRocker always 0)
  \return SLPL value

  This function reads back the SLPL value from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262DriverSlopeLowSide(UCHAR Which262)
{
  return DriverConfig[Which262].SlopeLowSide;
}


/*************************************************************************//**
  \fn Get262DriverDisableProtection(UCHAR Which262)
  \brief Get driver protection disable setting
  \param Which262  Axis number (with stepRocker always 0)
  \return 0=protection enabled, 1=protection disables

  This function reads back the protection disable setting value from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262DriverDisableProtection(UCHAR Which262)
{
  return DriverConfig[Which262].ProtectionDisable;
}


/*************************************************************************//**
  \fn Get262DriverProtectionTimer(UCHAR Which262)
  \brief Get protection timer setting
  \param Which262  Axis number (with stepRocker always 0)
  \return Protetion timer setting

  This function reads back the protection timer setting from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262DriverProtectionTimer(UCHAR Which262)
{
  return DriverConfig[Which262].ProtectionTimer;
}


/*************************************************************************//**
  \fn Get262DriverStepDirectionOff(UCHAR Which262)
  \brief Get step direction input enable setting
  \param Which262  Axis number (with stepRocker always 0)
  \return 0=enabled, 1=disabled

  This function reads back the step direction input setting from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262DriverStepDirectionOff(UCHAR Which262)
{
  return DriverConfig[Which262].StepDirectionDisable;
}


/*************************************************************************//**
  \fn Get262DriverVSenseScale(UCHAR Which262)
  \brief Get VSense scale setting
  \param Which262  Axis number (with stepRocker always 0)
  \return VSense scale setting (0 or 1)

  This function reads back the VSense scale setting from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262DriverVSenseScale(UCHAR Which262)
{
  return DriverConfig[Which262].VSenseScale;
}

/*************************************************************************//**
  \fn Get262DriverReadSelect(UCHAR Which262)
  \brief Get selected read back mode
  \param Which262  Axis number (with stepRocker always 0)
  \return Read back mode

  This function reads back the read back mode setting from the
  software copy of the TMC26x registers.
*****************************************************************************/
UCHAR Get262DriverReadSelect(UCHAR Which262)
{
  return DriverConfig[Which262].ReadBackSelect;
}


/***************************************************************//**
   \fn Read262State(UCHAR Which262, UCHAR *Phases, UCHAR *MStep, UINT *StallGuard, UCHAR *SmartEnergy, UCHAR *Flags)
   \brief Read the status register of a TMC26x
   \param  Which262  axis number (always 0 on stepRocker)
   \param  Phases    pointer at variable for phase bits
   \param  MStep     pointer at variable for microstep position
   \param  StallGuard  pointer at variable for stallGuard load value
   \param  SmartEnergy  pointer at variable for smartEnergy actual current
   \param  Flags     pointer at variable for driver error flags

   This function reads out the actual status information of a TMC26x.
   The data will be decoded according to the read back mode that has
   been set using Set262DriverReadSelect().
   For values that are not needed NULL pointers can be specified.
********************************************************************/
void Read262State(UCHAR Which262, UCHAR *Phases, UCHAR *MStep, UINT *StallGuard, UCHAR *SmartEnergy, UCHAR *Flags)
{
  //Abwechselnd alle Datagrammtypen des TMC26x zum Auslesen der Zustände benutzen.
  //So wird sichergestellt, daß alle Register immer mit den richtigen Werten beschrieben
  //sind (falls es zwischendurch mal zu einem Reset des TMC26x kommen sollte).
  switch(ReadBackDatagram[Which262])
  {
    case RB_CHOPPER:
      WriteChopperConfig(Which262);
      ReadBackDatagram[Which262]=RB_DRIVER;
      break;

    case RB_DRIVER:
      WriteDriverConfig(Which262);
      ReadBackDatagram[Which262]=RB_SMART_ENERGY;
      break;

    case RB_SMART_ENERGY:
      WriteSmartEnergyControl(Which262);
      ReadBackDatagram[Which262]=RB_STALL_GUARD;
      break;

    case RB_STALL_GUARD:
      WriteStallGuardConfig(Which262);
      ReadBackDatagram[Which262]=RB_STEP_DIR;
      break;

    case RB_STEP_DIR:
      WriteStepDirConfig(Which262);
      ReadBackDatagram[Which262]=RB_CHOPPER;
      break;

    default:
      ReadBackDatagram[Which262]=RB_CHOPPER;
      break;
  }

  //Decode the read back values
  switch(DriverConfig[Which262].ReadBackSelect)
  {
    case TMC262_RB_MSTEP:
      if(Phases!=NULL) *Phases=SPIReadInt >> 18;
      if(MStep!=NULL) *MStep=SPIReadInt >> 10;
      break;

    case TMC262_RB_STALL_GUARD:
      if(StallGuard!=NULL) *StallGuard=SPIReadInt >> 10;
      break;

    case TMC262_RB_SMART_ENERGY:
      if(StallGuard!=NULL)
      {
        *StallGuard=SPIReadInt >> 15;
        *StallGuard<<=5;
      }
      if(SmartEnergy!=NULL)
      {
        *SmartEnergy=(SPIReadInt >> 10) & 0x1f;
      }
      break;
  }

  if(Flags!=NULL) *Flags=SPIReadInt & 0xff;
}


/***************************************************************//**
   \fn Disable262(UCHAR Which262)
   \brief Disable a motor driver
   \param Which262  Axis number (always 0 on stepRocker)

   Completely switch off a motor driver (by setting its TOff value
   to zero).
********************************************************************/
void Disable262(UCHAR Which262)
{
  if(!ChopperConfig[Which262].DisableFlag)
  {
    ChopperConfig[Which262].DisableFlag=TRUE;
    WriteChopperConfig(Which262);
  }
}


/***************************************************************//**
   \fn Enable262(UCHAR Which262)
   \brief Enable a motor driver
   \param Which262  Axis number (always 0 on stepRocker)

   Re-enable a motor driver (by setting its TOff value back to the
   original value).
********************************************************************/
void Enable262(UCHAR Which262)
{
  if(ChopperConfig[Which262].DisableFlag)
  {
    ChopperConfig[Which262].DisableFlag=FALSE;
    WriteChopperConfig(Which262);
  }
}
