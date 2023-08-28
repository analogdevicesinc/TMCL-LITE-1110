/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  TMC262.h
           Definitions of TMC262 library functions

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
  \file TMC262.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief TMC262 library

  This file contains all macro and function definitions of the TMC26x library.
*/

#ifndef __TMCM262_H
#define __TMCM262_H

#define TMC262_RB_MSTEP        0
#define TMC262_RB_STALL_GUARD  1
#define TMC262_RB_SMART_ENERGY 2
#define TMC262_RB_ENCODER      3

#define TMC262_FLAG_STST  0x80
#define TMC262_FLAG_OLB   0x40
#define TMC262_FLAG_OLA   0x20
#define TMC262_FLAG_S2GB  0x10
#define TMC262_FLAG_S2GA  0x08
#define TMC262_FLAG_OTPW  0x04
#define TMC262_FLAG_OT    0x02
#define TMC262_FLAG_SG    0x01

//Funktionen für den Zugriff auf den TMC262
void InitMotorDrivers(void);
void Set262StepDirMStepRes(UCHAR Which262, UCHAR MicrostepResolution);
void Set262StepDirInterpolation(UCHAR Which262, UCHAR Interpolation);
void Set262StepDirDoubleEdge(UCHAR Which262, UCHAR DoubleEdge);
UCHAR Get262StepDirMStepRes(UCHAR Which262);
UCHAR Get262StepDirInterpolation(UCHAR Which262);
UCHAR Get262StepDirDoubleEdge(UCHAR Which262);

void Set262ChopperBlankTime(UCHAR Which262, UCHAR BlankTime);
void Set262ChopperMode(UCHAR Which262, UCHAR Mode);
void Set262ChopperRandomTOff(UCHAR Which262, UCHAR RandomTOff);
void Set262ChopperHysteresisDecay(UCHAR Which262, UCHAR HysteresisDecay);
void Set262ChopperHysteresisEnd(UCHAR Which262, UCHAR HysteresisEnd);
void Set262ChopperHysteresisStart(UCHAR Which262, UCHAR HysteresisStart);
void Set262ChopperTOff(UCHAR Which262, UCHAR TOff);
UCHAR Get262ChopperBlankTime(UCHAR Which262);
UCHAR Get262ChopperMode(UCHAR Which262);
UCHAR Get262ChopperRandomTOff(UCHAR Which262);
UCHAR Get262ChopperHysteresisDecay(UCHAR Which262);
UCHAR Get262ChopperHysteresisEnd(UCHAR Which262);
UCHAR Get262ChopperHysteresisStart(UCHAR Which262);
UCHAR Get262ChopperTOff(UCHAR Which262);

void Set262SmartEnergyIMin(UCHAR Which262, UCHAR SmartIMin);
void Set262SmartEnergyDownStep(UCHAR Which262, UCHAR SmartDownStep);
void Set262SmartEnergyStallLevelMax(UCHAR Which262, UCHAR StallLevelMax);
void Set262SmartEnergyUpStep(UCHAR Which262, UCHAR SmartUpStep);
void Set262SmartEnergyStallLevelMin(UCHAR Which262, UCHAR StallLevelMin);
UCHAR Get262SmartEnergyIMin(UCHAR Which262);
UCHAR Get262SmartEnergyDownStep(UCHAR Which262);
UCHAR Get262SmartEnergyStallLevelMax(UCHAR Which262);
UCHAR Get262SmartEnergyUpStep(UCHAR Which262);
UCHAR Get262SmartEnergyStallLevelMin(UCHAR Which262);

void Set262StallGuardFilter(UCHAR Which262, UCHAR Enable);
void Set262StallGuardThreshold(UCHAR Which262, signed char Threshold);
void Set262StallGuardCurrentScale(UCHAR Which262, UCHAR CurrentScale);
UCHAR Get262StallGuardFilter(UCHAR Which262);
signed char Get262StallGuardThreshold(UCHAR Which262);
UCHAR Get262StallGuardCurrentScale(UCHAR Which262);

void Set262DriverSlopeHighSide(UCHAR Which262, UCHAR SlopeHighSide);
void Set262DriverSlopeLowSide(UCHAR Which262, UCHAR SlopeLowSide);
void Set262DriverDisableProtection(UCHAR Which262, UCHAR DisableProtection);
void Set262DriverProtectionTimer(UCHAR Which262, UCHAR ProtectionTimer);
void Set262DriverStepDirectionOff(UCHAR Which262, UCHAR SDOff);
void Set262DriverVSenseScale(UCHAR Which262, UCHAR Scale);
void Set262DriverReadSelect(UCHAR Which262, UCHAR ReadSelect);
UCHAR Get262DriverSlopeHighSide(UCHAR Which262);
UCHAR Get262DriverSlopeLowSide(UCHAR Which262);
UCHAR Get262DriverDisableProtection(UCHAR Which262);
UCHAR Get262DriverProtectionTimer(UCHAR Which262);
UCHAR Get262DriverStepDirectionOff(UCHAR Which262);
UCHAR Get262DriverVSenseScale(UCHAR Which262);
UCHAR Get262DriverReadSelect(UCHAR Which262);

void Disable262(UCHAR Which262);
void Enable262(UCHAR Which262);

void Read262State(UCHAR Which262, UCHAR *Phases, UCHAR *MStep, UINT *StallGuard, UCHAR *SmartEnergy, UCHAR *Flags);
#endif
