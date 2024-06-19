/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains the definitions for importing the variables
  defined in Globals.c.
*/

extern TModuleConfig ModuleConfig;
extern TMotorConfig MotorConfig[3];

extern UCHAR SmartEnergy[N_O_MOTORS];
extern UCHAR StallFlag[N_O_MOTORS];
extern UINT StallLevel[N_O_MOTORS];
extern UCHAR DriverFlags[N_O_MOTORS];
extern UCHAR MotorDisable[N_O_MOTORS];
extern UCHAR StandbyFlag[N_O_MOTORS];
extern UCHAR FreewheelingActive[N_O_MOTORS];
extern UCHAR SlowRunCurrent[N_O_MOTORS];
extern TCoolStepConfig CoolStepConfig[N_O_MOTORS];

extern UCHAR ExitTMCLFlag;
