/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  SysTick.c
           Use of the system tick timer (1ms timer)

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
  \file SysTick.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief 1ms system tick timer functions

  This file provides all functions needed for easy
  access to the TMC262 stepper motor driver IC.
*/

#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
  #define SYSTICK_PRE_EMPTION_PRIORITY 3
#endif

#include "bits.h"
#include "stepRocker.h"

static volatile UINT TickCounter;              //!< System tick counter
extern volatile UINT UARTTransmitDelayTimer;
extern volatile UCHAR UARTTimeoutFlag;
extern volatile UINT UARTTimeoutTimer;

void __attribute__((interrupt)) SysTick_Handler(void);

/***************************************************//**
  \fn InitSysTimer(void)
  \brief Initalize system timer

  Initalize the tick timer for generating an interrupt
  every 1ms.
*******************************************************/
void InitSysTimer(void)
{
#if defined(MK20DX128)
  SYST_RVR=72000;
  SYST_CSR=7;
#elif defined(GD32F425)  
  SysTick_Config(SystemCoreClock/2000);
  NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRE_EMPTION_PRIORITY);
#endif
}


/***************************************************//**
  \fn SysTick_Handler(void)
  \brief System timer interrupt handler

  The system timer interrupt handler counts up the
  1ms counter.
*******************************************************/
void SysTick_Handler(void)
{
  //Update 1ms counter
  TickCounter++;

  //Count down RS485 transmit delay
  if(UARTTransmitDelayTimer>0) UARTTransmitDelayTimer--;

  //Count down RS485 receive timeout
  if(UARTTimeoutTimer>0)
  {
    UARTTimeoutTimer--;
    if(UARTTimeoutTimer==0) UARTTimeoutFlag=TRUE;
  }
}


/***************************************************//**
  \fn GetSysTimer(void)
  \brief Read the system timer
  \return System timer (1ms)

  This function returns the actual value of the 1ms
  system timer.
*******************************************************/
UINT GetSysTimer(void)
{
  return TickCounter;
}
