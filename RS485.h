/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  RS485.h
           Definitons of RS485 functions

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
  \file RS485.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Motor monitoring

  This file contains the definitions of the RS485 functions.
*/

void InitRS485(UCHAR Baudrate);
void WriteRS485(UCHAR Byte);
UCHAR ReadRS485(UCHAR *Byte);
void SetUARTTransmitDelay(UINT Delay);
UINT CheckUARTTimeout(void);
