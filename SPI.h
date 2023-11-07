/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains the definitions of the SPI access functions.
*/

void InitSPI(void);
UCHAR ReadWriteSPI(USHORT DeviceNumber, UCHAR Data, UCHAR LastTransfer);



