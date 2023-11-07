/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains the definitions of the RS485 functions.
*/

void InitRS485(UCHAR Baudrate);
void WriteRS485(UCHAR Byte);
UCHAR ReadRS485(UCHAR *Byte);
void SetUARTTransmitDelay(UINT Delay);
UINT CheckUARTTimeout(void);
