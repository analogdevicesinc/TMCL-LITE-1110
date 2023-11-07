/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file all functions necessary to implement a small TMCL interpreter.
*/

#include <stdlib.h>
#include <stddef.h>
#if defined(MK20DX128)
  #include "derivative.h"
#elif defined(GD32F425)
  #include "gd32f4xx.h"
#endif
#include "bits.h"
#include "stepRocker.h"
#include "Commands.h"
#include "Globals.h"
#include "RS485.h"
#include "SysTick.h"
#include "TMC429.h"
#include "TMC262.h"
#include "Eeprom.h"
#include "IO.h"
#include "Can.h"
#include "USB.h"


#ifdef BOOTLOADER
extern UINT BLMagic;                        //!< Magic code for bootloader
#endif

//Variables
static UCHAR UARTCmd[9];                    //!< RS485 command buffer
static UCHAR UARTCount;                     //!< RS485 commnd byte counter
static UCHAR TMCLCommandState;              //!< State of the interpreter
static TTMCLCommand ActualCommand;          //!< TMCL command to be executed (with all parameters)
static TTMCLReply ActualReply;              //!< Reply of last executed TMCL command
static UCHAR TMCLReplyFormat;               //!< format of next reply (RF_NORMAL or RF_SPECIAL)
static UCHAR SpecialReply[9];               //!< buffer for special replies
static UCHAR SpeedChangedFlag[N_O_MOTORS];  //!< TRUE when motor max. speed has been changed by ROL/ROR command
static UCHAR ResetRequested;                //!< TRUE after executing the software reset command
static UCHAR ExtendedCANFrame;              //!< TRUE when extended CAN frame used


//Prototypes
static void RotateRight(void);
static void RotateLeft(void);
static void MotorStop(void);
static void MoveToPosition(void);
static void SetAxisParameter(void);
static void GetAxisParameter(void);
static void GetVersion(void);
static void Boot(void);
static void SoftwareReset(void);


//Imported variables
extern char VersionString[];   //!< Imported version string


//Functions

/***************************************************************//**
   \fn ExecuteActualCommand()
   \brief Execute actual TMCL command

   Execute the TMCL command that must have been written
   to the global variable "ActualCommand" before.
********************************************************************/
static void ExecuteActualCommand(void)
{
  //Prepare answer
  ActualReply.Opcode=ActualCommand.Opcode;
  ActualReply.Status=REPLY_OK;
  ActualReply.Value.Int32=ActualCommand.Value.Int32;

  //Execute command
  switch(ActualCommand.Opcode)
  {
    case TMCL_ROR:
      RotateRight();
      break;

    case TMCL_ROL:
      RotateLeft();
      break;

    case TMCL_MST:
      MotorStop();
      break;

    case TMCL_MVP:
      MoveToPosition();
      break;

    case TMCL_SAP:
      SetAxisParameter();
      break;

    case TMCL_GAP:
      GetAxisParameter();
      break;

    case TMCL_GetVersion:
      GetVersion();
      break;

    case TMCL_Boot:
      Boot();
      break;

    case TMCL_SoftwareReset:
      SoftwareReset();
      break;

    default:
      ActualReply.Status=REPLY_INVALID_CMD;
      break;
  }
}


/***************************************************************//**
   \fn InitTMCL(void)
   \brief Initialize TMCL interpreter

   Intialise the TMCL interpreter. Must be called once at startup.
********************************************************************/
void InitTMCL(void)
{
  UINT i;

  TMCLCommandState=TCS_IDLE;
  for(i=0; i<N_O_MOTORS; i++) SpeedChangedFlag[i]=TRUE;
}


/***************************************************************//**
   \fn ProcessCommand(void)
   \brief Fetch and execute TMCL commands

   This is the main function for fetching and executing TMCL commands
   and has to be called periodically from the main loop.
********************************************************************/
void ProcessCommand(void)
{
  UCHAR Byte;
  UCHAR Checksum;
  UCHAR i;
  TCanFrame CanFrame;
  UCHAR USBCmd[9];
  UCHAR USBReply[9];

  //**Send answer for the last command**
  if(TMCLCommandState==TCS_CAN7 || TMCLCommandState==TCS_CAN8)  //via CAN
  {
    CanFrame.Id=ModuleConfig.CANSendID;
    CanFrame.Dlc=(TMCLCommandState==TCS_CAN7 ? 7:8);
    CanFrame.Ext=ExtendedCANFrame;
    CanFrame.Rtr=FALSE;

    if(TMCLReplyFormat==RF_STANDARD)
    {
      CanFrame.Data[0]=ModuleConfig.CANReceiveID & 0xff;
      CanFrame.Data[1]=ActualReply.Status;
      CanFrame.Data[2]=ActualReply.Opcode;
      CanFrame.Data[3]=ActualReply.Value.Byte[3];
      CanFrame.Data[4]=ActualReply.Value.Byte[2];
      CanFrame.Data[5]=ActualReply.Value.Byte[1];
      CanFrame.Data[6]=ActualReply.Value.Byte[0];
      CanFrame.Data[7]=0;
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<8; i++)
        CanFrame.Data[i]=SpecialReply[i+1];
    }

    //Antwort senden
    if(!CanSendMessage(&CanFrame)) return;
  }
  else if(TMCLCommandState==TCS_UART)  //via UART
  {
    if(TMCLReplyFormat==RF_STANDARD)
    {
      Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
               ActualReply.Status+ActualReply.Opcode+
               ActualReply.Value.Byte[3]+
               ActualReply.Value.Byte[2]+
               ActualReply.Value.Byte[1]+
               ActualReply.Value.Byte[0];

      WriteRS485(ModuleConfig.SerialHostAddress);
      WriteRS485(ModuleConfig.SerialModuleAddress);
      WriteRS485(ActualReply.Status);
      WriteRS485(ActualReply.Opcode);
      WriteRS485(ActualReply.Value.Byte[3]);
      WriteRS485(ActualReply.Value.Byte[2]);
      WriteRS485(ActualReply.Value.Byte[1]);
      WriteRS485(ActualReply.Value.Byte[0]);
      WriteRS485(Checksum);
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<9; i++)
      {
        WriteRS485(SpecialReply[i]);
      }
    }
  }
  else if(TMCLCommandState==TCS_UART_ERROR)  //check sum of the last command has been wrong
  {
    ActualReply.Opcode=0;
    ActualReply.Status=REPLY_CHKERR;
    ActualReply.Value.Int32=0;

    Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
             ActualReply.Status+ActualReply.Opcode+
             ActualReply.Value.Byte[3]+
             ActualReply.Value.Byte[2]+
             ActualReply.Value.Byte[1]+
             ActualReply.Value.Byte[0];

    WriteRS485(ModuleConfig.SerialHostAddress);
    WriteRS485(ModuleConfig.SerialModuleAddress);
    WriteRS485(ActualReply.Status);
    WriteRS485(ActualReply.Opcode);
    WriteRS485(ActualReply.Value.Byte[3]);
    WriteRS485(ActualReply.Value.Byte[2]);
    WriteRS485(ActualReply.Value.Byte[1]);
    WriteRS485(ActualReply.Value.Byte[0]);
    WriteRS485(Checksum);
  }
  else if(TMCLCommandState==TCS_USB)  //via USB
  {
    if(TMCLReplyFormat==RF_STANDARD)
    {
      Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
               ActualReply.Status+ActualReply.Opcode+
               ActualReply.Value.Byte[3]+
               ActualReply.Value.Byte[2]+
               ActualReply.Value.Byte[1]+
               ActualReply.Value.Byte[0];

      USBReply[0]=ModuleConfig.SerialHostAddress;
      USBReply[1]=ModuleConfig.SerialModuleAddress;
      USBReply[2]=ActualReply.Status;
      USBReply[3]=ActualReply.Opcode;
      USBReply[4]=ActualReply.Value.Byte[3];
      USBReply[5]=ActualReply.Value.Byte[2];
      USBReply[6]=ActualReply.Value.Byte[1];
      USBReply[7]=ActualReply.Value.Byte[0];
      USBReply[8]=Checksum;
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      for(i=0; i<9; i++)
      {
        USBReply[i]=SpecialReply[i];
      }
    }

    SendUSBReply(USBReply);
  }
  else if(TMCLCommandState==TCS_USB_ERROR)  //Check sum of last USB command was wrong
  {
    ActualReply.Opcode=0;
    ActualReply.Status=REPLY_CHKERR;
    ActualReply.Value.Int32=0;

    Checksum=ModuleConfig.SerialHostAddress+ModuleConfig.SerialModuleAddress+
             ActualReply.Status+ActualReply.Opcode+
             ActualReply.Value.Byte[3]+
             ActualReply.Value.Byte[2]+
             ActualReply.Value.Byte[1]+
             ActualReply.Value.Byte[0];

    USBReply[0]=ModuleConfig.SerialHostAddress;
    USBReply[1]=ModuleConfig.SerialModuleAddress;
    USBReply[2]=ActualReply.Status;
    USBReply[3]=ActualReply.Opcode;
    USBReply[4]=ActualReply.Value.Byte[3];
    USBReply[5]=ActualReply.Value.Byte[2];
    USBReply[6]=ActualReply.Value.Byte[1];
    USBReply[7]=ActualReply.Value.Byte[0];
    USBReply[8]=Checksum;

    //Antwort senden
    SendUSBReply(USBReply);
  }


  //Reset state (answer has been sent now)
  TMCLCommandState=TCS_IDLE;
  TMCLReplyFormat=RF_STANDARD;

  //Generate a system reset if requested by the host
  #if defined(MK20DX128)
  if(ResetRequested) ResetCPU(TRUE);
  #elif defined(GD32F425)
  if(ResetRequested) NVIC_SystemReset();
  #endif

  //**Try to get a new command**
  if(CanGetMessage(&CanFrame))  //From CAN?
  {
    ActualCommand.Opcode=CanFrame.Data[0];
    ActualCommand.Type=CanFrame.Data[1];
    ActualCommand.Motor=CanFrame.Data[2];
    ActualCommand.Value.Byte[3]=CanFrame.Data[3];
    ActualCommand.Value.Byte[2]=CanFrame.Data[4];
    ActualCommand.Value.Byte[1]=CanFrame.Data[5];
    ActualCommand.Value.Byte[0]=CanFrame.Data[6];
    ExtendedCANFrame=CanFrame.Ext;

    if(CanFrame.Dlc==7)
      TMCLCommandState=TCS_CAN7;
    else
      TMCLCommandState=TCS_CAN8;
  }
  else if(ReadRS485(&Byte))  //Get data from UART
  {
    if(CheckUARTTimeout()) UARTCount=0;  //discard everything when there has been a command timeout
    UARTCmd[UARTCount++]=Byte;

    if(UARTCount==9)  //Nine bytes have been received without timeout
    {
      UARTCount=0;

      if(UARTCmd[0]==ModuleConfig.SerialModuleAddress)  //Is this our addresss?
      {
        Checksum=0;
        for(i=0; i<8; i++) Checksum+=UARTCmd[i];

        if(Checksum==UARTCmd[8])  //Is the checksum correct?
        {
          ActualCommand.Opcode=UARTCmd[1];
          ActualCommand.Type=UARTCmd[2];
          ActualCommand.Motor=UARTCmd[3];
          ActualCommand.Value.Byte[3]=UARTCmd[4];
          ActualCommand.Value.Byte[2]=UARTCmd[5];
          ActualCommand.Value.Byte[1]=UARTCmd[6];
          ActualCommand.Value.Byte[0]=UARTCmd[7];

          TMCLCommandState=TCS_UART;

          UARTCount=0;
        }
        else TMCLCommandState=TCS_UART_ERROR;  //Checksum wrong
      }
    }
  }
  else if(GetUSBCmd(USBCmd))
  {
    //Ignore address byte
    Checksum=0;
    for(i=0; i<8; i++) Checksum+=USBCmd[i];

    if(Checksum==USBCmd[8])  //Checksum correct?
    {
      ActualCommand.Opcode=USBCmd[1];
      ActualCommand.Type=USBCmd[2];
      ActualCommand.Motor=USBCmd[3];
      ActualCommand.Value.Byte[3]=USBCmd[4];
      ActualCommand.Value.Byte[2]=USBCmd[5];
      ActualCommand.Value.Byte[1]=USBCmd[6];
      ActualCommand.Value.Byte[0]=USBCmd[7];

      TMCLCommandState=TCS_USB;

    } else TMCLCommandState=TCS_USB_ERROR;  //Checksum wrong
  }

  //**Execute the command**
  //Check if a command could be fetched and execute it.
  if(TMCLCommandState!=TCS_IDLE && TMCLCommandState!=TCS_UART_ERROR) ExecuteActualCommand();
}


//** TMCL Commands **

/***************************************************************//**
  \fn RotateRight(void)
  \brief Command ROR (see TMCL manual)

  ROR (ROtate Right) command (see TMCL manual).
********************************************************************/
static void RotateRight(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    SpeedChangedFlag[ActualCommand.Motor]=TRUE;
    Set429RampMode(ActualCommand.Motor, RM_VELOCITY);
    Write429Short(IDX_VMAX|(ActualCommand.Motor<<5), 2047);
    Write429Short(IDX_VTARGET|(ActualCommand.Motor<<5), ActualCommand.Value.Int32);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn RotateLeft(void)
  \brief Command ROL

  ROL (ROtate Left) command (see TMCL manual).
********************************************************************/
static void RotateLeft(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    SpeedChangedFlag[ActualCommand.Motor]=TRUE;
    Set429RampMode(ActualCommand.Motor, RM_VELOCITY);
    Write429Short(IDX_VMAX|(ActualCommand.Motor<<5), 2047);
    Write429Short(IDX_VTARGET|(ActualCommand.Motor<<5), -ActualCommand.Value.Int32);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn MotorStop(void)
  \brief Command MST

  MST (Motor StoP) command (see TMCL manual).
********************************************************************/
static void MotorStop(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    Set429RampMode(0, RM_VELOCITY);
    Write429Zero(IDX_VTARGET|(ActualCommand.Motor<<5));
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn MoveToPosition(void)
  \brief Command MVP

  MVP (Move To Position) command (see TMCL manual).
********************************************************************/
static void MoveToPosition(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    if(ActualCommand.Type==MVP_ABS || ActualCommand.Type==MVP_REL)
    {
      if(SpeedChangedFlag[ActualCommand.Motor])
      {
        Write429Short(IDX_VMAX|(ActualCommand.Motor<<5), MotorConfig[ActualCommand.Motor].VMax);
        SetAMax(ActualCommand.Motor, MotorConfig[ActualCommand.Motor].AMax);
        SpeedChangedFlag[ActualCommand.Motor]=FALSE;
      }

      if(ActualCommand.Type==MVP_ABS)
        Write429Int(IDX_XTARGET|(ActualCommand.Motor<<5), ActualCommand.Value.Int32);
      else
        Write429Int(IDX_XTARGET|(ActualCommand.Motor<<5), ActualCommand.Value.Int32+Read429Int(IDX_XACTUAL|(ActualCommand.Motor<<5)));
      Set429RampMode(0, RM_RAMP);
    }
    else ActualReply.Status=REPLY_WRONG_TYPE;
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn SetAxisParameter(void)
   \brief Command SAP

  SAP (Set Axis Parameter) command (see TMCL manual).
********************************************************************/
static void SetAxisParameter(void)
{
  UCHAR Read[4], Write[4];

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
      case 0:
      case 128:
        Write429Int(IDX_XTARGET|(ActualCommand.Motor<<5), ActualCommand.Value.Int32);
        break;

      case 1:
      case 129:
        Write429Int(IDX_XACTUAL|(ActualCommand.Motor<<5), ActualCommand.Value.Int32);
        break;

      case 2:
      case 132:
        Write429Short(IDX_VTARGET|(ActualCommand.Motor<<5), ActualCommand.Value.Int32);
        break;

      case 3:
      case 133:
        Write429Short(IDX_VACTUAL|(ActualCommand.Motor<<5), ActualCommand.Value.Int32);
        break;

      case 4:
      case 131:
        MotorConfig[ActualCommand.Motor].VMax=ActualCommand.Value.Int32;
        Write429Short(IDX_VMAX|MOTOR0, MotorConfig[ActualCommand.Motor].VMax);
        break;

      case 5:
      case 134:
        MotorConfig[ActualCommand.Motor].AMax=ActualCommand.Value.Int32;
        SetAMax(ActualCommand.Motor, MotorConfig[ActualCommand.Motor].AMax);
        break;

      case 6:
        MotorConfig[ActualCommand.Motor].IRun=ActualCommand.Value.Byte[0];
        Set262StallGuardCurrentScale(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Byte[0]/8);
        break;

      case 7:
        MotorConfig[ActualCommand.Motor].IStandby=ActualCommand.Value.Byte[0];
        break;

      case 12:
      case 148:
        Write[0]=IDX_REFCONF_RM|TMC429_READ|(ActualCommand.Motor<<5);
        ReadWrite429(Read, Write);
        Write[1]=Read[1];
        if(ActualCommand.Value.Byte[0]!=0)
          Write[2]=Read[2]|0x02;
        else
          Write[2]=Read[2]&  ~0x02;
        Write[3]=Read[3];
        Write[0]=IDX_REFCONF_RM|(ActualCommand.Motor<<5);
        ReadWrite429(Read, Write);
        break;

      case 13:
      case 147:
        Write[0]=IDX_REFCONF_RM|TMC429_READ|(ActualCommand.Motor<<5);
        ReadWrite429(Read, Write);
        Write[1]=Read[1];
        if(ActualCommand.Value.Byte[0]!=0)
          Write[2]=Read[2]|0x01;
        else
          Write[2]=Read[2]&  ~0x01;
        Write[3]=Read[3];
        Write[0]=IDX_REFCONF_RM|(ActualCommand.Motor<<5);
        ReadWrite429(Read, Write);
        break;

      case 130:
        Write429Short(IDX_VMIN|(ActualCommand.Motor<<5), ActualCommand.Value.Int32);
        break;

      case 138:
        Set429RampMode(ActualCommand.Motor, ActualCommand.Value.Byte[0]);
        break;

      case 140:
        Set262StepDirMStepRes(WHICH_262(ActualCommand.Motor), 8-ActualCommand.Value.Byte[0]);
        break;

      case 141:
        Write429Short(MOTOR_NUMBER(ActualCommand.Motor)<<5|IDX_DX_REFTOLERANCE, ActualCommand.Value.Int32);
        break;

      case 149:
        Read429Bytes(IDX_REFCONF_RM|MOTOR_NUMBER(ActualCommand.Motor)<<5, Read);
        if(ActualCommand.Value.Int32!=0)
          Read[1]|=0x04;
        else
          Read[1]&= ~0x04;
        Write429Bytes(IDX_REFCONF_RM|MOTOR_NUMBER(ActualCommand.Motor)<<5, Read);
        break;

      case 153:
        Write[0]=IDX_PULSEDIV_RAMPDIV|TMC429_READ|(ActualCommand.Motor<<5);
        ReadWrite429(Read, Write);
        Write[1]=Read[1];
        Write[2]=(Read[2] & 0xf0) | (ActualCommand.Value.Byte[0] & 0x0f);
        Write[3]=Read[3];
        Write[0]=IDX_PULSEDIV_RAMPDIV|(ActualCommand.Motor<<5);
        ReadWrite429(Read, Write);
        MotorConfig[ActualCommand.Motor].RampDiv=ActualCommand.Value.Byte[0] & 0x0f;
        break;

      case 154:
        Write[0]=IDX_PULSEDIV_RAMPDIV|TMC429_READ|(ActualCommand.Motor<<5);
        ReadWrite429(Read, Write);
        Write[1]=Read[1];
        Write[2]=(Read[2] & 0x0f) | (ActualCommand.Value.Byte[0] << 4);
        Write[3]=Read[3];
        Write[0]=IDX_PULSEDIV_RAMPDIV|(ActualCommand.Motor<<5);
        ReadWrite429(Read, Write);
        MotorConfig[ActualCommand.Motor].PulseDiv=ActualCommand.Value.Byte[0]& 0x0f;
        break;

      case 204:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].FreewheelingDelay;
        break;

      case 214:
       ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].SettingDelay;
       break;

//TMC262 specific parameters
      case 160:
        Set262StepDirInterpolation(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 161:
        Set262StepDirDoubleEdge(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 162:
        Set262ChopperBlankTime(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 163:
        Set262ChopperMode(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 164:
        Set262ChopperHysteresisDecay(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 165:
        Set262ChopperHysteresisEnd(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 166:
        Set262ChopperHysteresisStart(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 167:
        Set262ChopperTOff(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 168:
        Set262SmartEnergyIMin(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 169:
        Set262SmartEnergyDownStep(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 170:
        Set262SmartEnergyStallLevelMax(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 171:
        Set262SmartEnergyUpStep(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 172:
        Set262SmartEnergyStallLevelMin(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        CoolStepConfig[ActualCommand.Motor].HysteresisStart=ActualCommand.Value.Int32;
        break;

      case 173:
        Set262StallGuardFilter(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 174:
        Set262StallGuardThreshold(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 175:
        Set262DriverSlopeHighSide(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 176:
        Set262DriverSlopeLowSide(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 177:
        Set262DriverDisableProtection(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 178:
        Set262DriverProtectionTimer(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 179:
        Set262DriverVSenseScale(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 181:
        MotorConfig[ActualCommand.Motor].StallVMin=ActualCommand.Value.Int32;
        break;

      case 182:
        CoolStepConfig[ActualCommand.Motor].ThresholdSpeed=ActualCommand.Value.Int32;
        if(ActualCommand.Value.Int32==0)
          Set262SmartEnergyStallLevelMin(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      case 183:
        CoolStepConfig[ActualCommand.Motor].SlowRunCurrent=ActualCommand.Value.Int32;
        break;

      case 184:
        Set262ChopperRandomTOff(WHICH_262(ActualCommand.Motor), ActualCommand.Value.Int32);
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn GetAxisParameter(void)
  \brief Command GAP

  GAP (Get Axis Parameter) command (see TMCL manual).
********************************************************************/
static void GetAxisParameter(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    ActualReply.Value.Int32=0;

    switch(ActualCommand.Type)
    {
      case 0:
      case 128:
        ActualReply.Value.Int32=Read429Int(IDX_XTARGET|(ActualCommand.Motor<<5));
        break;

      case 1:
      case 129:
        ActualReply.Value.Int32=Read429Int(IDX_XACTUAL|(ActualCommand.Motor<<5));
        break;

      case 2:
      case 132:
        ActualReply.Value.Int32=Read429Short(IDX_VTARGET|(ActualCommand.Motor<<5));
        break;

      case 3:
      case 133:
        ActualReply.Value.Int32=Read429Short(IDX_VACTUAL|(ActualCommand.Motor<<5));
        break;

      case 4:
      case 131:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].VMax;
        break;

      case 5:
      case 134:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].AMax;
        break;

      case 6:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].IRun;
        break;

      case 7:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].IStandby;
        break;

      case 8:
        if(Read429Status() & 0x01) ActualReply.Value.Byte[0]=1;
        break;

      case 9:
      case 11:
      case 155:
        ActualReply.Value.Int32=(Read429SingleByte(IDX_REF_SWITCHES, 3) & (0x02<<ActualCommand.Motor*2)) ? 1:0;
        break;

      case 10:
      case 156:
        ActualReply.Value.Int32=(Read429SingleByte(IDX_REF_SWITCHES, 3) & (0x01<<ActualCommand.Motor*2)) ? 1:0;
        break;

      case 12:
      case 148:
        ActualReply.Value.Byte[0]=(Read429SingleByte(IDX_REFCONF_RM|(ActualCommand.Motor<<5), 2) & 0x02) ? 1:0;
        break;

      case 13:
      case 147:
        ActualReply.Value.Byte[0]=(Read429SingleByte(IDX_REFCONF_RM|(ActualCommand.Motor<<5), 2) & 0x01) ? 1:0;
        break;

      case 130:
        ActualReply.Value.Int32=Read429Short(IDX_VMIN|(ActualCommand.Motor<<5));
        break;

      case 138:
        ActualReply.Value.Byte[0]=Read429SingleByte(IDX_REFCONF_RM|(ActualCommand.Motor<<5), 3);
        break;

      case 140:
        ActualReply.Value.Int32=8-Get262StepDirMStepRes(WHICH_262(ActualCommand.Motor));
        break;

      case 141:
        ActualReply.Value.Int32=Read429Short(MOTOR_NUMBER(ActualCommand.Motor)<<5|IDX_DX_REFTOLERANCE);
        break;

      case 149:
        ActualReply.Value.Int32=(Read429SingleByte(IDX_REFCONF_RM|MOTOR_NUMBER(ActualCommand.Motor)<<5, 0) & 0x04) ? 1:0;
        break;

      case 153:
        ActualReply.Value.Byte[0]=Read429SingleByte(IDX_PULSEDIV_RAMPDIV|(ActualCommand.Motor<<5), 2) & 0x0f;
        break;

      case 154:
        ActualReply.Value.Byte[0]=Read429SingleByte(IDX_PULSEDIV_RAMPDIV|(ActualCommand.Motor<<5), 2) >> 4;
        break;

      case 204:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].FreewheelingDelay;
        break;

      case 206:
        ActualReply.Value.Int32=StallLevel[ActualCommand.Motor];
        break;

      case 208:
        ActualReply.Value.Int32=DriverFlags[ActualCommand.Motor];
        break;

      case 214:
       ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].SettingDelay;
       break;

//TMC262 specific parameters
      case 160:
        ActualReply.Value.Int32=Get262StepDirInterpolation(WHICH_262(ActualCommand.Motor));
        break;

      case 161:
        ActualReply.Value.Int32=Get262StepDirDoubleEdge(WHICH_262(ActualCommand.Motor));
        break;

      case 162:
        ActualReply.Value.Int32=Get262ChopperBlankTime(WHICH_262(ActualCommand.Motor));
        break;

      case 163:
        ActualReply.Value.Int32=Get262ChopperMode(WHICH_262(ActualCommand.Motor));
        break;

      case 164:
        ActualReply.Value.Int32=Get262ChopperHysteresisDecay(WHICH_262(ActualCommand.Motor));
        break;

      case 165:
        ActualReply.Value.Int32=Get262ChopperHysteresisEnd(WHICH_262(ActualCommand.Motor));
        break;

      case 166:
        ActualReply.Value.Int32=Get262ChopperHysteresisStart(WHICH_262(ActualCommand.Motor));
        break;

      case 167:
        ActualReply.Value.Int32=Get262ChopperTOff(WHICH_262(ActualCommand.Motor));
        break;

      case 168:
        ActualReply.Value.Int32=Get262SmartEnergyIMin(WHICH_262(ActualCommand.Motor));
        break;

      case 169:
        ActualReply.Value.Int32=Get262SmartEnergyDownStep(WHICH_262(ActualCommand.Motor));
        break;

      case 170:
        ActualReply.Value.Int32=Get262SmartEnergyStallLevelMax(WHICH_262(ActualCommand.Motor));
        break;

      case 171:
        ActualReply.Value.Int32=Get262SmartEnergyUpStep(WHICH_262(ActualCommand.Motor));
        break;

      case 172:
        //ActualReply.Value.Int32=Get262SmartEnergyStallLevelMin(WHICH_262(ActualCommand.Motor));
        ActualReply.Value.Int32=CoolStepConfig[ActualCommand.Motor].HysteresisStart;
        break;

      case 173:
        ActualReply.Value.Int32=Get262StallGuardFilter(WHICH_262(ActualCommand.Motor));
        break;

      case 174:
        ActualReply.Value.Int32=Get262StallGuardThreshold(WHICH_262(ActualCommand.Motor));
        break;

      case 175:
        ActualReply.Value.Int32=Get262DriverSlopeHighSide(WHICH_262(ActualCommand.Motor));
        break;

      case 176:
        ActualReply.Value.Int32=Get262DriverSlopeLowSide(WHICH_262(ActualCommand.Motor));
        break;

      case 177:
        ActualReply.Value.Int32=Get262DriverDisableProtection(WHICH_262(ActualCommand.Motor));
        break;

      case 178:
        ActualReply.Value.Int32=Get262DriverProtectionTimer(WHICH_262(ActualCommand.Motor));
        break;

      case 179:
        ActualReply.Value.Int32=Get262DriverVSenseScale(WHICH_262(ActualCommand.Motor));
        break;

      case 180:
        ActualReply.Value.Int32=SmartEnergy[ActualCommand.Motor];
        break;

      case 181:
        ActualReply.Value.Int32=MotorConfig[ActualCommand.Motor].StallVMin;
        break;

      case 182:
        ActualReply.Value.Int32=CoolStepConfig[ActualCommand.Motor].ThresholdSpeed;
        break;

      case 183:
        ActualReply.Value.Int32=CoolStepConfig[ActualCommand.Motor].SlowRunCurrent;
        break;

      case 184:
        ActualReply.Value.Int32=Get262ChopperRandomTOff(WHICH_262(ActualCommand.Motor));
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn GetVersion(void)
  \brief Command 136 (get version)

  Get the version number (when type==0) or
  the version string (when type==1).
********************************************************************/
static void GetVersion(void)
{
  UCHAR i;

  if(ActualCommand.Type==0)
  {
    TMCLReplyFormat=RF_SPECIAL;
    SpecialReply[0]=ModuleConfig.SerialHostAddress;
    for(i=0; i<8; i++)
      SpecialReply[i+1]=VersionString[i];
  }
  else if(ActualCommand.Type==1)
  {
    ActualReply.Value.Byte[3]=SW_TYPE_HIGH;
    ActualReply.Value.Byte[2]=SW_TYPE_LOW;
    ActualReply.Value.Byte[1]=SW_VERSION_HIGH;
    ActualReply.Value.Byte[0]=SW_VERSION_LOW;
  }
}


/************************************//**
   \fn Boot(void)
   \brief Enter bootloader mode

   Special command for exiting TMCL
   and calling the boot loader.
 ***************************************/
static void Boot(void)
{
#ifdef BOOTLOADER
  UINT Delay;

  if(ActualCommand.Type==0x81 && ActualCommand.Motor==0x92 &&
     ActualCommand.Value.Byte[3]==0xa3 && ActualCommand.Value.Byte[2]==0xb4 &&
     ActualCommand.Value.Byte[1]==0xc5 && ActualCommand.Value.Byte[0]==0xd6)
  {
    DISABLE_DRIVERS();

#if defined(MK20DX128)
    DeInitUSB();
    Delay=GetSysTimer();
    while(abs(GetSysTimer()-Delay)<1000);

    DisableInterrupts();
    SYST_CSR=0;
    NVICICER0=0xFFFFFFFF;  //alle Interrupts sperren
    NVICICPR0=0xFFFFFFFF;
    NVICICER1=0xFFFFFFFF;
    NVICICPR1=0xFFFFFFFF;
    NVICICER2=0xFFFFFFFF;
    NVICICPR2=0xFFFFFFFF;
    NVICICER3=0xFFFFFFFF;
    NVICICPR3=0xFFFFFFFF;
    BLMagic=0x12345678;
    ResetCPU(TRUE);
#elif defined(GD32F425)
    DetachUSB();
    Delay=GetSysTimer();
    while(abs(GetSysTimer()-Delay)<1000);
    __disable_irq();
    NVIC_DeInit();
    SysTick->CTRL=0;

    BLMagic=0x12345678;
    NVIC_SystemReset();
#endif
  }
#else
  ActualReply.Status=REPLY_CMD_NOT_AVAILABLE;
#endif
}


/**************************//**
   \fn SoftwareReset(void)
   \brief TMCL software reset command

   Issue a TMCL software reset.
 *******************************/
static void SoftwareReset(void)
{
  if(ActualCommand.Value.Int32==1234) ResetRequested=TRUE;
}
