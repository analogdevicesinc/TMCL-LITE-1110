/*******************************************************************************
* Copyright © 2011 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides functions for intializing and
  using the CAN interface.
*/

#ifndef __CAN_H
#define __CAN_H

typedef struct
{
  unsigned char Dlc, Ext, Rtr;
  unsigned long Id;
  unsigned char Data[8];
} TCanFrame;

void InitCan(UCHAR Baudrate, USHORT ReceiveID, USHORT SecondaryID);
int CanSendMessage(TCanFrame *Msg);
int CanGetMessage(TCanFrame *Msg);

#endif
