/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides functions for intializing and
  using the I/O ports and some other miscellaneous
  stuff.
*/

#include <stdlib.h>
#include "derivative.h"
#include "bits.h"
#include "stepRocker.h"
#include "TMC262.h"
#include "IO.h"


#ifdef BOOTLOADER
extern UINT BLMagic;                        //!< Magic code for bootloader
#endif


/*********************************************//**
  \fn InitIO(void)
  \brief Initialize I/O

  This function initalizes all I/O port pins of
  the CPU that are not initialized in somewhere
  else in other initialization functions.
*************************************************/
void InitIO(void)
{
#if defined(BOOTLOADER)
  UINT Count1;
  UINT Count2;
  UINT i;
  volatile UINT t;
#endif

  PORTA_PCR4=PORT_PCR_MUX(1);
  PORTA_PCR5=PORT_PCR_MUX(1);
  PORTA_PCR14=PORT_PCR_MUX(1);
  PORTA_PCR15=PORT_PCR_MUX(1);
  PORTA_PCR16=PORT_PCR_MUX(1);
  PORTA_PCR17=PORT_PCR_MUX(1);
  GPIOA_PDDR=BIT17|BIT16|BIT15|BIT5|BIT3;
  GPIOA_PCOR=BIT17|BIT16|BIT15|BIT5|BIT3;

  PORTB_PCR0=PORT_PCR_MUX(1);
  PORTB_PCR1=PORT_PCR_MUX(1);
  PORTB_PCR2=PORT_PCR_MUX(1);
  PORTB_PCR3=PORT_PCR_MUX(1);
  PORTB_PCR10=PORT_PCR_MUX(1);
  PORTB_PCR11=PORT_PCR_MUX(1);
  PORTB_PCR16=PORT_PCR_MUX(1);
  PORTB_PCR17=PORT_PCR_MUX(1);
  GPIOB_PDDR=BIT17|BIT16;
  GPIOB_PSOR=BIT17|BIT16;

  PORTC_PCR0=PORT_PCR_MUX(1);
  PORTC_PCR1=PORT_PCR_MUX(1);
  PORTC_PCR2=PORT_PCR_MUX(1);
  PORTC_PCR11=PORT_PCR_MUX(1);
  GPIOC_PDDR=BIT11|BIT2|BIT1|BIT0;
  GPIOC_PSOR=BIT11|BIT2|BIT1|BIT0;

  PORTD_PCR0=PORT_PCR_MUX(1);
  PORTD_PCR1=PORT_PCR_MUX(1);
  PORTD_PCR2=PORT_PCR_MUX(1);
  PORTD_PCR3=PORT_PCR_MUX(1);
  PORTD_PCR4=PORT_PCR_MUX(1);
  GPIOD_PDDR=BIT4|BIT2;
  GPIOD_PSOR=BIT2;
  GPIOD_PCOR=BIT4;

  PORTE_PCR5=PORT_PCR_MUX(1);
  GPIOE_PDDR=BIT5;

  SIM_SOPT2 &= ~SIM_SOPT2_CLKOUTSEL_MASK;
  SIM_SOPT2 |= SIM_SOPT2_CLKOUTSEL(6);
  PORTC_PCR3=PORT_PCR_MUX(5);

#if defined(BOOTLOADER)
  //Check if SDIO and SDCLK are connected together and
  //return to bootloader if yes.
  PORTA_PCR0=PORT_PCR_MUX(1);
  PORTA_PCR3=PORT_PCR_MUX(1);

  Count1=Count2=0;
  for(i=0; i<10; i++)
  {
    GPIOA_PSOR=BIT3;
    for(t=0; t<1000; t++);
    if(GPIOA_PDIR & BIT0) Count1++;

    GPIOA_PCOR=BIT3;
    for(t=0; t<1000; t++);
    if(!(GPIOA_PDIR & BIT0)) Count2++;
  }
  PORTA_PCR0=PORT_PCR_MUX(7);
  PORTA_PCR3=PORT_PCR_MUX(7);

  if(Count1==10 && Count2==10)
  {
    DisableInterrupts();
    SYST_CSR=0;
    NVICICER0=0xFFFFFFFF;
    NVICICPR0=0xFFFFFFFF;
    NVICICER1=0xFFFFFFFF;
    NVICICPR1=0xFFFFFFFF;
    NVICICER2=0xFFFFFFFF;
    NVICICPR2=0xFFFFFFFF;
    NVICICER3=0xFFFFFFFF;
    NVICICPR3=0xFFFFFFFF;
    BLMagic=0x12345678;
    ResetCPU(TRUE);
  }
#endif
}


/***************************************************************//**
   \fn EnableInterrupts(void)
   \brief Enable all interrupts

  This function globally enables all interrupts.
********************************************************************/
void EnableInterrupts(void)
{
  asm volatile("CPSIE I\n");
}


/***************************************************************//**
   \fn DisableInterrupts(void)
   \brief Disable all interrupts

  This function globally disables all interrupts.
********************************************************************/
void DisableInterrupts(void)
{
  asm volatile("CPSID I\n");
}


/***************************************************************//**
   \fn ResetCPU(UCHAR ResetPeripherals)
   \brief Reset the CPU
   \param ResetPeripherals Reset also periperals when TRUE

  This function resets the CPU with or without the integrated
  peripherals.
********************************************************************/
void ResetCPU(UCHAR ResetPeripherals)
{
  if(ResetPeripherals)
    SCB_AIRCR = SCB_AIRCR_VECTKEY(0x5FA) | SCB_AIRCR_SYSRESETREQ_MASK;
  else
    SCB_AIRCR = SCB_AIRCR_VECTKEY(0x5FA) | SCB_AIRCR_VECTRESET_MASK;
}


/***************************************************************//**
   \fn SetMotorCurrent(UCHAR Motor, UCHAR Current)
   \brief Set motor current
   \param Motor    Motor number (with stepRocker always 0)
   \param Current  Motor current scaled to 0..255

   This function sets the motor current, scaled to 0..255 as
   usual in TMCL.
********************************************************************/
void SetMotorCurrent(UCHAR Motor, UCHAR Current)
{
  Set262StallGuardCurrentScale(WHICH_262(Motor), Current>>3);
}


/***************************************************************//**
   \fn GetStallState(UCHAR Motor)
   \brief Get stall flag state of motor driver
   \param Motor  Motor number (with stepRocker always 0)
   \return  TRUE stall\n
            FALSE no stall

   This function reads the state of the SG_TEST pin of the TMC262.
********************************************************************/
UCHAR GetStallState(UCHAR Motor)
{
  if(Motor==0)
  {
    if(GPIOA_PDIR & BIT13)
      return TRUE;
    else
      return FALSE;
  }
  else return FALSE;
}
