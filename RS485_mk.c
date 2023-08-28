/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  RS485.c
           Access to the RS485 interface

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
  \file RS485.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief RS485 functions

  This file provides all functions needed for using
  the RS485 interface.
*/

#include "derivative.h"
#include "bits.h"
#include "stepRocker.h"
#include "nvic.h"


#define UART0 UART0_BASE_PTR


#define UART_BUFFER_SIZE 32        //!< Size of the RS485 transmit and receive buffers
#define UART_TIMEOUT_VALUE 5       //!< Timeout value (ms)

#define SET_RS485_SEND_MODE()     GPIOA_PSOR=BIT17      //!< Switch RS485 transceiver to send mode
#define SET_RS485_RECEIVE_MODE()  GPIOA_PCOR=BIT17      //!< Switch RS485 transceiver to receive mode
#define IS_RS485_SENDING()        (GPIOA_PDOR & BIT17)  //!< TRUE when transceiver is in send mode

// 0    1    2    3    4    5    6    7     8     9   10   11
//9.6  14.4 19.2 28.8 38.4 57.6 76.8 115.2 230.4 250 500 1000 kBaud
static const UINT baudrateTable[] = {9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 230400, 250000, 500000, 1000000};

static volatile char UARTRxBuffer[UART_BUFFER_SIZE];    //!< Receive buffer
static volatile char UARTTxBuffer[UART_BUFFER_SIZE];    //!< Transmit buffer
static volatile int UARTRxWritePtr;                     //!< Receive buffer read index
static volatile int UARTRxReadPtr;                      //!< Receive buffer write index
static volatile int UARTTxWritePtr;                     //!< Transmit buffer write index
static volatile int UARTTxReadPtr;                      //!< Transmit buffer read index
static volatile UINT UARTTransmitDelay;                 //!< Delay between receiving and sending
volatile UCHAR UARTTimeoutFlag;                         //!< Timeout flag (gets set in the system timer interrupt)
volatile UINT UARTTimeoutTimer;                         //!< Timeout timer (gets deceremented in the system timer interrupt)
volatile UINT UARTTransmitDelayTimer;                   //!< Timer for delay between receiving and sending (gets decremented in the system timer interrupt)

/******************************************************//**
  \fn UART0_RX_TX_IRQHandler(void)
  \brief UART0 interrupt handler

  Interrupt handler function for the RS485 interface. It
  handles transmit buffer full and receive buffer empty
  interrupts.
**********************************************************/
void __attribute__ ((interrupt)) UART0_RX_TX_IRQHandler(void);
void UART0_RX_TX_IRQHandler(void)
{
  UINT Status;
  int i;

  //Read status register
  Status = UART0->S1;

  //Character received?
  if(Status & UART_S1_RDRF_MASK)
  {
    //Copy to receive buffer
    i=UARTRxWritePtr+1;
    if(i==UART_BUFFER_SIZE) i=0;

    if(i!=UARTRxReadPtr)
    {
      UARTRxBuffer[UARTRxWritePtr]=UART0->D;
      UARTRxWritePtr=i;
    }

    //Set receive timeout to start value
    UARTTimeoutTimer=UART_TIMEOUT_VALUE;

    //Set transmit delay to start value
    UARTTransmitDelayTimer=UARTTransmitDelay;
  }

  //Last bit sent?
  if(Status & UART_S1_TC_MASK)
  {
    //in this case set RS485 back to receive mode
    SET_RS485_RECEIVE_MODE();
    UART0->C2&= ~UART_C2_TCIE_MASK;
  }

  //Transmit register emty?
  if(Status & UART_S1_TDRE_MASK)
  {
    if(UARTTransmitDelayTimer==0)
    {
      if(UARTTxWritePtr!=UARTTxReadPtr)
      {
        //Copy to send register if character in buffer
        SET_RS485_SEND_MODE();
        UART0->D=UARTTxBuffer[UARTTxReadPtr++];
        if(UARTTxReadPtr==UART_BUFFER_SIZE) UARTTxReadPtr=0;

        //activate transmission complete interrupt for RS485 direction switching
        UART0->C2|=UART_C2_TCIE_MASK;
      }
      else
      {
        //Buffer empty: switch off transmit interrupt
        UART0->C2&= ~UART_C2_TIE_MASK;
      }
    }
  }
}


/********************************************************//**
  \fn InitRS485(UCHAR baudrateIndex)
  \brief Initalize RS485 interface
  \param Baudrate  baud rate code (0..11)

  This function initializes the RS485 interface. The baud
  rate codes are the same as with TMCL.
************************************************************/
void InitRS485(UCHAR baudrateIndex)
{
  USHORT divisor;

  //Use default Baudrate (9600) if index out of range
  if(baudrateIndex>11) baudrateIndex=0;

  //Calculate baud rate, swtich pins to UART0
  divisor=(72000000/16)/baudrateTable[baudrateIndex];

  SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
  PORTA_PCR1 = PORT_PCR_MUX(2);
  PORTA_PCR2 = PORT_PCR_MUX(2);

  enable_irq(INT_UART0_RX_TX-16);

  //Set options and baudrate
  UART0->C2 = 0;
  UART0->C1 = 0;
  UART0->C3 = 0;
  UART0->C5 = 0;
  UART0->S2 = 0;
  UART0->BDH=(divisor>>8) & UART_BDH_SBR_MASK;
  UART0->BDL=divisor & UART_BDL_SBR_MASK;

  //Switch on UART interrupts
  UART0->C2|=UART_C2_TE_MASK|UART_C2_RE_MASK|UART_C2_RIE_MASK;
}


/****************************************************//**
  \fn WriteRS485(UCHAR Byte)
  \brief Write to the RS485 interface
  \param Byte  Byte to be written

  This function puts a byte into the RS485 transmit
  buffer and starts sending if not already done.
********************************************************/
void WriteRS485(char Byte)
{
  int i;

  i=UARTTxWritePtr+1;
  if(i==UART_BUFFER_SIZE) i=0;

  //Copy to buffer is space is available
  if(i!=UARTTxReadPtr)
  {
    UARTTxBuffer[UARTTxWritePtr] = Byte;
    UARTTxWritePtr = i;

    //Switch on transmit interrupt
    UART0->C2|=UART_C2_TIE_MASK;
  }
}


/****************************************************//**
  \fn ReadRS485(UCHAR *Byte)
  \brief Read from the RS485 interface
  \param Byte  Pointer to variable for result
  \return TRUE if a byte could be read\n
          FALSE if the receive buffer was empty

  This function tries to read a byte from the RS485 receive
  buffer.
********************************************************/
UCHAR ReadRS485(char *Byte)
{
  //Character available in receive buffer?
  if(UARTRxReadPtr == UARTRxWritePtr) return FALSE;

  //Copy from receive buffer
  *Byte = UARTRxBuffer[UARTRxReadPtr++];

  if(UARTRxReadPtr == UART_BUFFER_SIZE) UARTRxReadPtr = 0;

  return TRUE;
}


/****************************************************//**
  \fn SetUARTTransmitDelay(UINT Delay)
  \brief Set RS485 transmit delay
  \param Delay  Delay in ms

  This function sets the delay between receiving the last
  byte and sending the first byte. This can be necessary
  with some RS485 interfaces.
********************************************************/
void SetUARTTransmitDelay(UINT Delay)
{
  UARTTransmitDelay=Delay;
}


/*******************************************************************************//**
  \fn CheckUARTTimeout(void)
  \brief Check and reset RS485 timeout flag
  \return TRUE if there has been a timeout (>5ms after last received byte)\n
          FALSE if there has not been a timeout since the last call of this function

  This function checks the timeout flag and then resets it.
***********************************************************************************/
UINT CheckUARTTimeout(void)
{
  if(UARTTimeoutFlag)
  {
    UARTTimeoutFlag=FALSE;
    return TRUE;
  }
  else return FALSE;
}
