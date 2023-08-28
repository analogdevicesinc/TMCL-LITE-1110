/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  USB.h
           Processing of TMCL commands

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
  \file USB.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief USB functions

  This file contains high level USB functions.
*/

#ifndef __USB_H
#define __USB_H

void InitUSB(void);
UCHAR GetUSBCmd(UCHAR *Cmd);
void SendUSBReply(UCHAR *Reply);
void DeInitUSB(void);

#endif
