/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains high level USB communication functions.
*/

#include "derivative.h"
#include "stepRocker.h"
#include "CDC1.h"
#include "USB0.h"
#include "USB1.h"
#include "Tx1.h"
#include "Rx1.h"
#include "usb_cdc.h"


/***************************************************************//**
   \fn InitUSB()
   \brief Initialize USB interface

    This function initializes the USB interface.
********************************************************************/
void InitUSB(void)
{
  USB0_Init();
  Tx1_Init();
  Rx1_Init();
  USB1_Init();
}


/***************************************************************//**
   \fn GetUSB(UCHAR *Cmd)
   \brief Read TMCL command from USB
   \param *Cmd Pointer to nine byte char array
   \return TRUE if nine bytes have been read

    This function tries to read nine bytes from the USB interface
    (one TMCL command). If there are more than nine bytes in the
    USB receive buffer then these bytes will be discarded.
********************************************************************/
UCHAR GetUSBCmd(UCHAR *Cmd)
{
  UINT i;

  if(CDC1_GetCharsInRxBuf()>=9)
  {
    for(i=0; i<9; i++) CDC1_GetChar(&Cmd[i]);
    CDC1_ClearRxBuffer();
    return TRUE;
  }
  else return FALSE;
}


/***************************************************************//**
   \fn SendUSBReply(UCHAR *Reply)
   \brief Send nine bytes via USB
   \param *Reply Pointer to nine byte array

    This function sends nine byte (a TMCL reply) out via USB.
********************************************************************/
void SendUSBReply(UCHAR *Reply)
{
  CDC1_SendBlock(Reply, 9);
}


/***************************************************************//**
   \fn DeInitUSB()
   \brief Switch off USB interface

    This function switches off the USB interface and so detaches
    the module from USB.
********************************************************************/
void DeInitUSB(void)
{
  USB_Class_CDC_DeInit(0);
}
