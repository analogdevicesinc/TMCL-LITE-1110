/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides functions for intializing and
  using the CAN interface.
*/

#include <stdlib.h>
#include "derivative.h"
#include "bits.h"
#include "stepRocker.h"
#include "Can.h"

#define CAN_MB_RX_INACTIVE 0x00
#define CAN_MB_FULL        0x02
#define CAN_MB_EMPTY       0x04
#define CAN_MB_TX_INACTIVE 0x08
#define CAN_MB_DATA        0x0C

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
void InitCan(UCHAR Bitrate, USHORT ReceiveID1, USHORT ReceiveID2)
{
  //Switch on clock for CAN interface
  SIM_SCGC6|=SIM_SCGC6_FLEXCAN0_MASK;

  //Link to CAN pins
  PORTB_PCR18=PORT_PCR_MUX(2);
  PORTB_PCR19=PORT_PCR_MUX(2);

  //Initialize CAN interface
  CAN0_MCR=CAN_MCR_MDIS_MASK|CAN_MCR_FRZ_MASK|CAN_MCR_HALT_MASK|CAN_MCR_SUPV_MASK|CAN_MCR_MAXMB(15);
  CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK;
  CAN0_MCR=CAN_MCR_FRZ_MASK|CAN_MCR_HALT_MASK|CAN_MCR_SUPV_MASK|CAN_MCR_MAXMB(15);
  while(!(CAN0_MCR & CAN_MCR_FRZACK_MASK));

  //72MHz (36MHz for CAN-Modul)
  //CAN-Timing: Phase 1 (=PROPSEG+PSEG1+2 Tq) + Phase 2 (=PSEG2+1 Tq) + SJW (1 Tq)
  //With PROPSEG=7 (8), PSEG1=7 (8) and PSEG2=6 (7) we have 24 time quanta and 70% sampling point.
  switch(Bitrate)
  {
    case 8:   //1MBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(1)|CAN_CTRL1_PROPSEG(3)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(4)|CAN_CTRL1_RJW(3);
      break;

    case 7:   //500kBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(2)|CAN_CTRL1_PROPSEG(7)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(6)|CAN_CTRL1_RJW(3);
      break;

    case 6:   //250kBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(5)|CAN_CTRL1_PROPSEG(7)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(6)|CAN_CTRL1_RJW(3);
      break;

    case 5:   //125kBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(11)|CAN_CTRL1_PROPSEG(7)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(6)|CAN_CTRL1_RJW(3);
      break;

    case 4:   //100kBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(14)|CAN_CTRL1_PROPSEG(7)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(6)|CAN_CTRL1_RJW(3);
      break;

    case 3:   //50kBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(29)|CAN_CTRL1_PROPSEG(7)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(6)|CAN_CTRL1_RJW(3);
      break;

    case 2:   //20kBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(74)|CAN_CTRL1_PROPSEG(7)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(6)|CAN_CTRL1_RJW(3);
      break;

    case 1:   //10kBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(149)|CAN_CTRL1_PROPSEG(7)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(6)|CAN_CTRL1_RJW(3);
      break;

    default:  //250kBit/s
      CAN0_CTRL1=CAN_CTRL1_CLKSRC_MASK|CAN_CTRL1_SMP_MASK|CAN_CTRL1_PRESDIV(5)|CAN_CTRL1_PROPSEG(7)|CAN_CTRL1_PSEG1(7)|CAN_CTRL1_PSEG2(6)|CAN_CTRL1_RJW(3);
      break;
  }

  CAN0_CTRL2=0;

  CAN0_MCR=CAN_MCR_SUPV_MASK|CAN_MCR_MAXMB(15);

  //Mailbox 0 for incoming telegrams (first ID (11 Bit))
  CAN0_CS0=CAN_CS_CODE(CAN_MB_EMPTY);
  CAN0_ID0=CAN_ID_STD(ReceiveID1);

  //Mailbox 1 for incoming telegrams (secondary ID (11 Bit))
  if(ReceiveID2>0)
  {
    CAN0_CS1=CAN_CS_CODE(CAN_MB_EMPTY);
    CAN0_ID1=CAN_ID_STD(ReceiveID2);
  }
  else CAN0_CS1=CAN_CS_CODE(CAN_MB_RX_INACTIVE);

  //Mailbox 2 for incoming telegrams (first ID (29 Bit))
  CAN0_CS2=CAN_CS_CODE(CAN_MB_EMPTY)|CAN_CS_IDE_MASK;
  CAN0_ID2=CAN_ID_EXT(ReceiveID1);

  //Mailbox 3 for incoming telegrams (secondary ID (29 Bit))
  if(ReceiveID2>0)
  {
    CAN0_CS3=CAN_CS_CODE(CAN_MB_EMPTY)|CAN_CS_IDE_MASK;
    CAN0_ID3=CAN_ID_EXT(ReceiveID2);
  }
  else CAN0_CS3=CAN_CS_CODE(CAN_MB_RX_INACTIVE);
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
  UINT Code;

  //Mailbox still occupied?
  if(((CAN0_CS15 >> 24) & 0x0f)==CAN_MB_DATA) return FALSE;

  //Write outgoing data into mailbox 15
  if(Msg->Ext)
    CAN0_ID15=CAN_ID_EXT(Msg->Id);
  else
    CAN0_ID15=CAN_ID_STD(Msg->Id);

  CAN0_WORD015=(Msg->Data[0] << 24)|(Msg->Data[1] << 16)|(Msg->Data[2] << 8)|Msg->Data[3];
  CAN0_WORD115=(Msg->Data[4] << 24)|(Msg->Data[5] << 16)|(Msg->Data[6] << 8)|Msg->Data[7];

  Code=CAN_CS_CODE(CAN_MB_DATA);
  if(Msg->Ext) Code|=CAN_CS_IDE_MASK;
  if(Msg->Rtr) Code|=CAN_CS_RTR_MASK;
  Code|=CAN_CS_DLC(Msg->Dlc);

  //Activate mailbox
  CAN0_CS15=Code;

  return TRUE;
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
  UINT Mailbox;

  //See which mailbox has data
  if(((CAN0_CS0 >> 24) & 0x0f)==CAN_MB_FULL) Mailbox=0;
  else if(((CAN0_CS1 >> 24) & 0x0f)==CAN_MB_FULL) Mailbox=1;
  else if(((CAN0_CS2 >> 24) & 0x0f)==CAN_MB_FULL) Mailbox=2;
  else if(((CAN0_CS3 >> 24) & 0x0f)==CAN_MB_FULL) Mailbox=3;
  else return FALSE;

  //Copy data from that mailbox
  Msg->Dlc=(CAN_CS_REG(CAN0_BASE_PTR, Mailbox) >> 16) & 0x0f;
  if(CAN_CS_REG(CAN0_BASE_PTR, Mailbox) & BIT20)
    Msg->Rtr=TRUE;
  else
    Msg->Rtr=FALSE;

  if(CAN_CS_REG(CAN0_BASE_PTR, Mailbox) & BIT21)
    Msg->Ext=TRUE;
  else
    Msg->Ext=FALSE;

  if(!Msg->Ext)
    Msg->Id=(CAN_ID_REG(CAN0_BASE_PTR, Mailbox) >> 18) & 0x7ff;
  else
    Msg->Id=CAN_ID_REG(CAN0_BASE_PTR, Mailbox) & 0x1fffffff;

  Msg->Data[0]=CAN_WORD0_REG(CAN0_BASE_PTR, Mailbox) >> 24;
  Msg->Data[1]=CAN_WORD0_REG(CAN0_BASE_PTR, Mailbox) >> 16;
  Msg->Data[2]=CAN_WORD0_REG(CAN0_BASE_PTR, Mailbox) >> 8;
  Msg->Data[3]=CAN_WORD0_REG(CAN0_BASE_PTR, Mailbox) & 0xff;
  Msg->Data[4]=CAN_WORD1_REG(CAN0_BASE_PTR, Mailbox) >> 24;
  Msg->Data[5]=CAN_WORD1_REG(CAN0_BASE_PTR, Mailbox) >> 16;
  Msg->Data[6]=CAN_WORD1_REG(CAN0_BASE_PTR, Mailbox) >> 8;
  Msg->Data[7]=CAN_WORD1_REG(CAN0_BASE_PTR, Mailbox) & 0xff;

  //Free that mailbox
  if(CAN_CS_REG(CAN0_BASE_PTR, Mailbox) & CAN_CS_IDE_MASK)
    CAN_CS_REG(CAN0_BASE_PTR, Mailbox)=CAN_CS_CODE(CAN_MB_EMPTY)|CAN_CS_IDE_MASK;
  else
    CAN_CS_REG(CAN0_BASE_PTR, Mailbox)=CAN_CS_CODE(CAN_MB_EMPTY);

  return TRUE;
}
