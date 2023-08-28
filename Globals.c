/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  Globals.c
           Global variables and data structures

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
  \file Globals.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Global variables

  This file contains all globally used variables.
*/

#include "stepRocker.h"


//! Global parameters (here only used for the RS232 interface)
TModuleConfig ModuleConfig=
{
  0,      //!< RS485 bitrate (0=9600)
  1,      //!< RS485 address
  2,      //!< RS485 reply address
  1,      //!< CAN receive ID
  2,      //!< CAN send ID
  8,      //!< CAN bit rate (8=1000kBit/s)
};

//! Motor configuration data
TMotorConfig MotorConfig[N_O_MOTORS]=
{{
  1000,   //!< VMax
  500,    //!< AMax
  2,      //!< Pulsediv
  3,      //!< Rampdiv
  255,    //!< IRun
  32,     //!< IStandby
  0,      //!< StallVMin
  0,      //!< FreewheelingDelay
  200     //!< SettingDelay
 }
};

UCHAR SmartEnergy[N_O_MOTORS];      //!< actual smartEnergy values
UCHAR StallFlag[N_O_MOTORS];        //!< actual stallGuard flag states
UINT StallLevel[N_O_MOTORS];        //!< actual stallGuard load values
UCHAR DriverFlags[N_O_MOTORS];      //!< actual driver error flags
UCHAR MotorDisable[N_O_MOTORS];     //!< actual motor disable flags
UCHAR StandbyFlag[N_O_MOTORS];      //!< standby current flags
UCHAR FreewheelingActive[N_O_MOTORS];        //!< freewheeling flags
UCHAR SlowRunCurrent[N_O_MOTORS];            //!< slow run current
TCoolStepConfig CoolStepConfig[N_O_MOTORS];  //!< coolStep configuration

UCHAR ExitTMCLFlag;   //!< This will be set to TRUE for exiting TMCL and branching to the boot loader
