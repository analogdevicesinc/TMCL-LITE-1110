/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
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
