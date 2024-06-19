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
#include "gd32f4xx.h"
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

  //===== Port A =====
  rcu_periph_clock_enable(RCU_GPIOA);

  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15|GPIO_PIN_4);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_15|GPIO_PIN_4);
  GPIO_BOP(GPIOA)=GPIO_PIN_15|GPIO_PIN_4;

  //Clock output (PA8)
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_8);
  gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_8);
  rcu_ckout0_config(RCU_CKOUT0SRC_HXTAL, RCU_CKOUT0_DIV1);

  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);

  //===== Port B =====
  rcu_periph_clock_enable(RCU_GPIOB);

  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_7);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_15|GPIO_PIN_14);
  GPIO_BOP(GPIOB)=GPIO_PIN_15|GPIO_PIN_14;

  gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_13|GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_5);

  //===== Port C =====
  rcu_periph_clock_enable(RCU_GPIOC);

  gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3);
  GPIO_BC(GPIOC)=GPIO_PIN_5|GPIO_PIN_4;
  GPIO_BOP(GPIOC)=GPIO_PIN_3;

  gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_14|GPIO_PIN_13);

  //===== Port D =====
  rcu_periph_clock_enable(RCU_GPIOD);

  gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_11|GPIO_PIN_10);
  gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_11|GPIO_PIN_10);
  GPIO_BOP(GPIOD)=GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_11|GPIO_PIN_10;

  gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_13|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_8);

  //===== Port E =====
  rcu_periph_clock_enable(RCU_GPIOE);

  gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_1);
  gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_1);
  GPIO_BOP(GPIOE)=GPIO_PIN_1;
  GPIO_BC(GPIOE)=GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13;

  gpio_mode_set(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9|
                                                        GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_0);

  #ifdef BOOTLOADER
  //Check if SDIO and SDCLK are connected together and
  //return to bootloader if yes.
  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_14);

  Count1=Count2=0;
  for(i=0; i<10; i++)
  {
    GPIO_BC(GPIOA)=BIT13;
    for(t=0; t<4000; t++) asm volatile("nop\n");
    if(!(GPIO_ISTAT(GPIOA) & BIT14)) Count1++;

    GPIO_BOP(GPIOA)=BIT13;
    for(t=0; t<4000; t++) asm volatile("nop\n");
    if((GPIO_ISTAT(GPIOA) & BIT14)) Count2++;
  }
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14|GPIO_PIN_13);
  gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_14|GPIO_PIN_13);

  if(Count1==10 && Count2==10)
  {
    __disable_irq();
    NVIC_DeInit();
    SysTick->CTRL=0;

    BLMagic=0x12345678;
    NVIC_SystemReset();
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
    if(GPIO_ISTAT(GPIOE) & BIT0)
      return TRUE;
    else
      return FALSE;
  }
  else return FALSE;
}

void NVIC_DeInit(void)
{
  uint32_t index;

  for(index=0; index<8; index++)
  {
    NVIC->ICER[index] = 0xFFFFFFFF;
    NVIC->ICPR[index] = 0xFFFFFFFF;
  }

  for(index = 0; index < 240; index++)
  {
     NVIC->IP[index] = 0x00000000;
  }
}
