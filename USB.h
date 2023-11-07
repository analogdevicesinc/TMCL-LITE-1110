/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains high level USB functions.
*/

#ifndef __USB_H
#define __USB_H

void InitUSB(void);
UCHAR GetUSBCmd(UCHAR *Cmd);
void SendUSBReply(UCHAR *Reply);
void DeInitUSB(void);

#endif
