/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  Eeprom.h
           Definitions of EEPROM access functions

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
  \file Eeprom.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief EEPROM access functions

  This file contains the EEPROM access function defintions.
*/

void WriteEepromByte(UINT Address, UCHAR Value);
void WriteEepromBlock(UINT Address, UCHAR *Block, UINT Size);
UCHAR ReadEepromByte(UINT Address);
void ReadEepromBlock(UINT Address, UCHAR *Block, UINT Size);

