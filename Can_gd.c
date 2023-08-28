/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  Can_gd.c
           CAN routines

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
  \file Can_gd.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief CAN functions

  This file provides functions for intializing and
  using the CAN interface.
*/


#include "gd32f4xx.h"
#include "stepRocker.h"
#include "Can.h"

#define CAN_INTR_PRI 7
#define CAN_TX_BUF_SIZE 8
#define CAN_RX_BUF_SIZE 8

volatile int CanRxWritePtr, CanRxReadPtr, CanTxWritePtr, CanTxReadPtr;
volatile TCanFrame CanTxBuffer[CAN_TX_BUF_SIZE];
volatile TCanFrame CanRxBuffer[CAN_RX_BUF_SIZE];

#define CANx   CAN0

void __attribute__ ((interrupt)) CAN0_RX0_IRQHandler(void);
void __attribute__ ((interrupt)) CAN0_RX1_IRQHandler(void);
void __attribute__ ((interrupt)) CAN0_TX_IRQHandler(void);



/*******************************************************************
  CAN Receive Interrupt Handler for FIFO 0
********************************************************************/
void CAN0_RX0_IRQHandler(void)
{
  can_receive_message_struct CanMessage;
  int i;
  int j;

  i=CanRxWritePtr+1;
  if(i==CAN_RX_BUF_SIZE) i=0;

  if(i!=CanRxReadPtr)
  {
    can_message_receive(CANx, CAN_FIFO0, &CanMessage);

    if(CanMessage.rx_ff==CAN_FF_EXTENDED)
    {
      CanRxBuffer[CanRxWritePtr].Ext=TRUE;
      CanRxBuffer[CanRxWritePtr].Id=CanMessage.rx_efid;
    }
    else
    {
      CanRxBuffer[CanRxWritePtr].Ext=FALSE;
      CanRxBuffer[CanRxWritePtr].Id=CanMessage.rx_sfid;
    }

    if(CanMessage.rx_ft==CAN_FT_REMOTE)
      CanRxBuffer[CanRxWritePtr].Rtr=TRUE;
    else
      CanRxBuffer[CanRxWritePtr].Rtr=FALSE;

    CanRxBuffer[CanRxWritePtr].Dlc=CanMessage.rx_dlen;
    for(j=0; j<8; j++) CanRxBuffer[CanRxWritePtr].Data[j]=CanMessage.rx_data[j];

    CanRxWritePtr=i;
  }
  else
  {
  	//just release FIFO when buffer full
    can_fifo_release(CANx, CAN_FIFO0);
  }
}


/*******************************************************************
  CAN Receive Interrupt Handler for FIFO 1
********************************************************************/
void CAN0_RX1_IRQHandler(void)
{
  can_receive_message_struct CanMessage;
  int i;
  int j;

  i=CanRxWritePtr+1;
  if(i==CAN_RX_BUF_SIZE) i=0;

  if(i!=CanRxReadPtr)
  {
    can_message_receive(CANx, CAN_FIFO1, &CanMessage);

    if(CanMessage.rx_ff==CAN_FF_EXTENDED)
    {
      CanRxBuffer[CanRxWritePtr].Ext=TRUE;
      CanRxBuffer[CanRxWritePtr].Id=CanMessage.rx_efid;
    }
    else
    {
      CanRxBuffer[CanRxWritePtr].Ext=FALSE;
      CanRxBuffer[CanRxWritePtr].Id=CanMessage.rx_sfid;
    }

    if(CanMessage.rx_ft==CAN_FT_REMOTE)
      CanRxBuffer[CanRxWritePtr].Rtr=TRUE;
    else
      CanRxBuffer[CanRxWritePtr].Rtr=FALSE;

    CanRxBuffer[CanRxWritePtr].Dlc=CanMessage.rx_dlen;
    for(j=0; j<8; j++) CanRxBuffer[CanRxWritePtr].Data[j]=CanMessage.rx_data[j];

    CanRxWritePtr=i;
  }
  else
  {
  	//just release FIFO when buffer full
    can_fifo_release(CANx, CAN_FIFO1);
  }
}


/*******************************************************************
  CAN Transmit Interrupt Handler
********************************************************************/
void CAN0_TX_IRQHandler(void)
{
  can_trasnmit_message_struct CanMessage;  //typo in library
  int i;

  if(CanTxReadPtr==CanTxWritePtr)  //nothing to send?
  {
    can_interrupt_disable(CANx, CAN_INTEN_TMEIE);
  }
  else
  {
    if(CanTxBuffer[CanTxReadPtr].Ext)
    {
      CanMessage.tx_ff=CAN_FF_EXTENDED;
      CanMessage.tx_sfid=0;
      CanMessage.tx_efid=CanTxBuffer[CanTxReadPtr].Id;
    }
    else
    {
      CanMessage.tx_ff=CAN_FF_STANDARD;
      CanMessage.tx_sfid=CanTxBuffer[CanTxReadPtr].Id;
      CanMessage.tx_efid=0;
    }

    if(CanTxBuffer[CanTxReadPtr].Rtr)
      CanMessage.tx_ft=CAN_FT_REMOTE;
    else
      CanMessage.tx_ft=CAN_FT_DATA;

    CanMessage.tx_dlen=CanTxBuffer[CanTxReadPtr].Dlc;
    for(i=0; i<8; i++) CanMessage.tx_data[i]=CanTxBuffer[CanTxReadPtr].Data[i];
    CanTxReadPtr++;
    if(CanTxReadPtr==CAN_TX_BUF_SIZE) CanTxReadPtr=0;

    can_message_transmit(CANx, &CanMessage);
  }
}


/*****************************************************************//**
  \fn InitCan()
  \brief Initialization of the CAN interface

  \param Baudrate  CAN-Bitrate  1  2  3   4   5   6   7    8
                               10 20 50 100 125 250 500 1000kBit/s

  \param ReceiveID  Only CAN messages with this ID (standard or
                    extended frames) will be accepted.

  \param SecondaryID  If this is !=0 then also messgaes with this
                      ID (standard or extended frames) will be
                      accepted.
*********************************************************************/
void InitCan(UCHAR Bitrate, USHORT ReceiveID, USHORT SecondaryID)
{
  can_parameter_struct CANInit;
  can_filter_parameter_struct CANFilterInit;

  CANInit.time_triggered=DISABLE;
  CANInit.auto_bus_off_recovery=ENABLE;
  CANInit.auto_wake_up=DISABLE;
  CANInit.auto_retrans=ENABLE;
  CANInit.rec_fifo_overwrite=ENABLE;
  CANInit.trans_fifo_order=ENABLE;
  CANInit.working_mode = CAN_NORMAL_MODE;
  switch(Bitrate)  //240MHz clock frequency (=> 60MHz CAN clock frequency)
  {
    case 8:     //1000kBit/s: 15 Time Quanta, Sampling Point 86.7%
      CANInit.prescaler=4;
      CANInit.time_segment_1=CAN_BT_BS1_12TQ;
      CANInit.time_segment_2=CAN_BT_BS2_2TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;

    case 7:     //500kBit/s: 20 Time Quanta, Sampling Point 85.0%
      CANInit.prescaler=6;
      CANInit.time_segment_1=CAN_BT_BS1_16TQ;
      CANInit.time_segment_2=CAN_BT_BS2_3TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;

    case 6:     //250kBit/s: 20 Time Quanta, Sampling Point 85.0%
      CANInit.prescaler=12;
      CANInit.time_segment_1=CAN_BT_BS1_16TQ;
      CANInit.time_segment_2=CAN_BT_BS2_3TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;

    case 5:     //125kBit/s: 20 Time Quanta, Sampling Point 85.0%
      CANInit.prescaler=24;
      CANInit.time_segment_1=CAN_BT_BS1_16TQ;
      CANInit.time_segment_2=CAN_BT_BS2_3TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;

    case 4:     //100kBit/s: 20 Time Quanta, Sampling Point 85.0%
      CANInit.prescaler=30;
      CANInit.time_segment_1=CAN_BT_BS1_16TQ;
      CANInit.time_segment_2=CAN_BT_BS2_3TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;

    case 3:     //50kBit/s: 20 Time Quanta, Sampling Point 85.0%
      CANInit.prescaler=60;
      CANInit.time_segment_1=CAN_BT_BS1_16TQ;
      CANInit.time_segment_2=CAN_BT_BS2_3TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;

    case 2:     //20kBit/s: 25 Time Quanta, Sampling Point 68.0%
      CANInit.prescaler=120;
      CANInit.time_segment_1=CAN_BT_BS1_16TQ;
      CANInit.time_segment_2=CAN_BT_BS2_8TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;

    case 1:     //10kBit/s: 25 Time Quanta, Sampling Point 68.0%
      CANInit.prescaler=240;
      CANInit.time_segment_1=CAN_BT_BS1_13TQ;
      CANInit.time_segment_2=CAN_BT_BS2_4TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;

    default:    //250kBit/s: 20 Time Quanta, Sampling Point 85.0%
      CANInit.prescaler=12;
      CANInit.time_segment_1=CAN_BT_BS1_16TQ;
      CANInit.time_segment_2=CAN_BT_BS2_3TQ;
      CANInit.resync_jump_width=CAN_BT_SJW_1TQ;
      break;
  }

  rcu_periph_clock_enable(RCU_CAN0);
  gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1|GPIO_PIN_0);
  gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1|GPIO_PIN_0);
  gpio_af_set(GPIOD, GPIO_AF_9, GPIO_PIN_1|GPIO_PIN_0);
  can_init(CAN0, &CANInit);
  nvic_irq_enable(CAN0_RX0_IRQn, CAN_INTR_PRI, 0);
  nvic_irq_enable(CAN0_RX1_IRQn, CAN_INTR_PRI, 0);
  nvic_irq_enable(CAN0_TX_IRQn, CAN_INTR_PRI, 0);

  //Configure CAN acceptance filters
  //Filter 0 used for standard frames on FIFO 0
  if(ReceiveID<0x800)
  {
    CANFilterInit.filter_number=1;
    CANFilterInit.filter_mode=CAN_FILTERMODE_MASK;
    CANFilterInit.filter_bits=CAN_FILTERBITS_32BIT;
    CANFilterInit.filter_list_high=ReceiveID << 5;
    CANFilterInit.filter_list_low=0;
    CANFilterInit.filter_mask_high=0xffff;
    CANFilterInit.filter_mask_low=0xffff;
    CANFilterInit.filter_fifo_number=CAN_FIFO0;
    CANFilterInit.filter_enable=ENABLE;
    can_filter_init(&CANFilterInit);
  }
  else
  {
    //Deactivate filter 0 when extended frames used
    CANFilterInit.filter_number=1;
    CANFilterInit.filter_mode=CAN_FILTERMODE_MASK;
    CANFilterInit.filter_bits=CAN_FILTERBITS_32BIT;
    CANFilterInit.filter_list_high=0;
    CANFilterInit.filter_list_low=0;
    CANFilterInit.filter_mask_high=0xffff;
    CANFilterInit.filter_mask_low=0xffff;
    CANFilterInit.filter_fifo_number=CAN_FIFO0;
    CANFilterInit.filter_enable=DISABLE;
    can_filter_init(&CANFilterInit);
  }

  //Filter 1 used for standard frames on FIFO 1
  if(SecondaryID!=0 && SecondaryID<0x800)
  {
    CANFilterInit.filter_number=2;
    CANFilterInit.filter_mode=CAN_FILTERMODE_MASK;
    CANFilterInit.filter_bits=CAN_FILTERBITS_32BIT;
    CANFilterInit.filter_list_high=SecondaryID << 5;
    CANFilterInit.filter_list_low=0;
    CANFilterInit.filter_mask_high=0xffff;
    CANFilterInit.filter_mask_low=0xffff;
    CANFilterInit.filter_fifo_number=CAN_FIFO1;
    CANFilterInit.filter_enable=ENABLE;
    can_filter_init(&CANFilterInit);
  }
  else
  {
    //Deactivate filter 1 when extended ID selected or secondary ID set to 0
    CANFilterInit.filter_number=2;
    CANFilterInit.filter_mode=CAN_FILTERMODE_MASK;
    CANFilterInit.filter_bits=CAN_FILTERBITS_32BIT;
    CANFilterInit.filter_list_high=0;
    CANFilterInit.filter_list_low=0;
    CANFilterInit.filter_mask_high=0xffff;
    CANFilterInit.filter_mask_low=0xffff;
    CANFilterInit.filter_fifo_number=CAN_FIFO1;
    CANFilterInit.filter_enable=DISABLE;
    can_filter_init(&CANFilterInit);
  }

  //Filter 2 used for extended frames on FIFO 0
  CANFilterInit.filter_number=3;
  CANFilterInit.filter_mode=CAN_FILTERMODE_MASK;
  CANFilterInit.filter_bits=CAN_FILTERBITS_32BIT;
  CANFilterInit.filter_list_high=ReceiveID >> 13;
  CANFilterInit.filter_list_low=(ReceiveID << 3) | CAN_FF_EXTENDED;
  CANFilterInit.filter_mask_high=0xffff;
  CANFilterInit.filter_mask_low=0xffff;
  CANFilterInit.filter_fifo_number=CAN_FIFO0;
  CANFilterInit.filter_enable=ENABLE;
  can_filter_init(&CANFilterInit);

  //Filter 3 used for extended frames on FIFO 1
  if(SecondaryID>0)
  {
    CANFilterInit.filter_number=4;
    CANFilterInit.filter_mode=CAN_FILTERMODE_MASK;
    CANFilterInit.filter_bits=CAN_FILTERBITS_32BIT;
    CANFilterInit.filter_list_high=SecondaryID >> 13;
    CANFilterInit.filter_list_low=(SecondaryID << 3) | CAN_FF_EXTENDED;
    CANFilterInit.filter_mask_high=0xffff;
    CANFilterInit.filter_mask_low=0xffff;
    CANFilterInit.filter_fifo_number=CAN_FIFO1;
    CANFilterInit.filter_enable=ENABLE;
    can_filter_init(&CANFilterInit);
  }
  else
  {
    //Deactivate filter 3 when secondary ID set to 0
    CANFilterInit.filter_number=4;
    CANFilterInit.filter_mode=CAN_FILTERMODE_MASK;
    CANFilterInit.filter_bits=CAN_FILTERBITS_32BIT;
    CANFilterInit.filter_list_high=0;
    CANFilterInit.filter_list_low=CAN_FF_EXTENDED;
    CANFilterInit.filter_mask_high=0xffff;
    CANFilterInit.filter_mask_low=0xffff;
    CANFilterInit.filter_fifo_number=CAN_FIFO1;
    CANFilterInit.filter_enable=DISABLE;
    can_filter_init(&CANFilterInit);
  }

  can_interrupt_enable(CANx, CAN_INTEN_RFNEIE0);
  can_interrupt_enable(CANx, CAN_INTEN_RFNEIE1);
}


/***************************************************************//**
   \fn CanSendMessage()
   \param *Msg: Pointer to CAN message (type TCanFrame) to be sent

   \return  TRUE if message could be written\n
            FALSE if buffer full.

   Try to send a CAN message by writing it to mailbox 15.
********************************************************************/
int CanSendMessage(TCanFrame *Msg)
{
  can_trasnmit_message_struct CanMessage;
  int i;

  if(Msg->Ext)
  {
    CanMessage.tx_ff=CAN_FF_EXTENDED;
    CanMessage.tx_efid=Msg->Id;
    CanMessage.tx_sfid=0;
  }
  else
  {
    CanMessage.tx_ff=CAN_FF_STANDARD;
    CanMessage.tx_sfid=Msg->Id;
    CanMessage.tx_efid=0;
  }

  if(Msg->Rtr)
    CanMessage.tx_ft=CAN_FT_REMOTE;
  else
    CanMessage.tx_ft=CAN_FT_DATA;

  CanMessage.tx_dlen=Msg->Dlc;
  for(i=0; i<8; i++) CanMessage.tx_data[i]=Msg->Data[i];

  if(can_message_transmit(CANx, &CanMessage)!=CAN_NOMAILBOX) return TRUE;

  i=CanTxWritePtr+1;
  if(i==CAN_TX_BUF_SIZE) i=0;

  if(i!=CanTxReadPtr)
  {
    CanTxBuffer[CanTxWritePtr]=*Msg;
    CanTxWritePtr=i;

    can_interrupt_enable(CANx, CAN_INTEN_TMEIE);

    return TRUE;
  } else return FALSE;
}


/***************************************************************//**
   \fn CanGetMessage()
   \param *Msg: Pointer to TCanFrame structure to take the message

   \return  TRUE: Message successfully read\n
            FALSE: No message read (buffer empty).

   Try to read a message from the receive buffer if it is not
   empty.
********************************************************************/
int CanGetMessage(TCanFrame *Msg)
{
  //Buffer empty?
  if(CanRxReadPtr==CanRxWritePtr) return FALSE;

  *Msg=CanRxBuffer[CanRxReadPtr++];
  if(CanRxReadPtr==CAN_RX_BUF_SIZE) CanRxReadPtr=0;

  return TRUE;
}
