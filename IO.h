/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains definitions for using the I/O functions.
*/

void InitIO(void);
void EnableInterrupts(void);
void DisableInterrupts(void);
void ResetCPU(UCHAR ResetPeripherals);
void SetMotorCurrent(UCHAR Motor, UCHAR Current);
UCHAR GetStallState(UCHAR Motor);
