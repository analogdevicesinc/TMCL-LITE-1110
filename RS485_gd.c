/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides all functions needed for using
  the RS485 interface.
*/

#include "gd32f4xx.h"
#include "bits.h"
#include "stepRocker.h"

#define UART_INTR_PRI        6

#define UART_BUFFER_SIZE 32     //Size of buffers
#define UART_TIMEOUT_VALUE 5    //Timeout value (ms)

#define USARTx USART2
#define SET_RS485_SEND_MODE_1()       GPIO_BOP(GPIOE)=BIT15
#define SET_RS485_RECEIVE_MODE_1()    GPIO_BC(GPIOE)=BIT15
#define IS_RS485_SENDING_1()          (GPIO_OCTL(GPIOE) & BIT15)

void __attribute__ ((interrupt)) USART2_IRQHandler(void);

static volatile char UARTRxBuffer[UART_BUFFER_SIZE];
static volatile char UARTTxBuffer[UART_BUFFER_SIZE];
static volatile int UARTRxReadPtr;
static volatile int UARTRxWritePtr;
static volatile int UARTTxReadPtr;
static volatile int UARTTxWritePtr;
volatile uint8_t UARTTimeoutFlag;
volatile uint32_t UARTTimeoutTimer;
static volatile uint32_t UARTTransmitDelay;
volatile uint32_t UARTTransmitDelayTimer;


/********************************************************//**
  \fn InitRS485(UCHAR baudrateIndex)
  \brief Initalize RS485 interface
  \param Baudrate  baud rate code (0..11)

  This function initializes the RS485 interface. The baud
  rate codes are the same as with TMCL.
************************************************************/
void InitRS485(int BaudRate)
{
  rcu_periph_clock_enable(RCU_USART2);
  gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_11|GPIO_PIN_10);
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11|GPIO_PIN_10);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11|GPIO_PIN_10);

  usart_deinit(USARTx);
  switch(BaudRate)
  {
    case 0:
      usart_baudrate_set(USARTx, 9600);
      break;

    case 1:
      usart_baudrate_set(USARTx, 14400);
      break;

    case 2:
      usart_baudrate_set(USARTx, 19200);
      break;

    case 3:
      usart_baudrate_set(USARTx, 28800);
      break;

    case 4:
      usart_baudrate_set(USARTx, 38400);
      break;

    case 5:
      usart_baudrate_set(USARTx, 57600);
      break;

    case 6:
      usart_baudrate_set(USARTx, 76800);
      break;

    case 7:
      usart_baudrate_set(USARTx, 115200);
      break;

    case 8:
      usart_baudrate_set(USARTx, 230400);
      break;

    case 9:
      usart_baudrate_set(USARTx, 250000);
      break;

    case 10:
      usart_baudrate_set(USARTx, 500000);
      break;

    case 11:
      usart_baudrate_set(USARTx, 1000000);
      break;

    default:
      usart_baudrate_set(USARTx, 9600);
      break;
  }

  usart_word_length_set(USARTx, USART_WL_8BIT);
  usart_stop_bit_set(USARTx, USART_STB_1BIT);
  usart_parity_config(USARTx, USART_PM_NONE);
  usart_hardware_flow_rts_config(USARTx, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(USARTx, USART_CTS_DISABLE);
  usart_receive_config(USARTx, USART_RECEIVE_ENABLE);
  usart_transmit_config(USARTx, USART_TRANSMIT_ENABLE);
  usart_enable(USARTx);
  usart_interrupt_enable(USARTx, USART_INT_TBE);
  usart_interrupt_enable(USARTx, USART_INT_TC);
  usart_interrupt_enable(USARTx, USART_INT_RBNE);

  nvic_irq_enable(USART2_IRQn, UART_INTR_PRI, 0);
}


/*******************************************************************
  UART interrupt handler
********************************************************************/
void USART2_IRQHandler(void)
{
  int i;
  int x;
  uint8_t Data;

  if(USART_STAT0(USARTx) & USART_STAT0_RBNE)
  {
    if(IS_RS485_SENDING_1())
    {
      i=USART_DATA(USARTx);
    }
    else
    {
      i=UARTRxWritePtr+1;
      if(i==UART_BUFFER_SIZE) i=0;

      Data=USART_DATA(USARTx);
      if(i!=UARTRxReadPtr)
      {
        UARTRxBuffer[UARTRxWritePtr]=Data;
        UARTRxWritePtr=i;
      }

      UARTTimeoutTimer=UART_TIMEOUT_VALUE;

      UARTTransmitDelayTimer=UARTTransmitDelay;
    }
  }

  if(USART_STAT0(USARTx) & USART_STAT0_TBE)
  {
      if(UARTTxWritePtr!=UARTTxReadPtr)
      {
        SET_RS485_SEND_MODE_1();
        USART_DATA(USARTx)=UARTTxBuffer[UARTTxReadPtr++];
        if(UARTTxReadPtr==UART_BUFFER_SIZE) UARTTxReadPtr=0;
      }
      else
      {
        usart_interrupt_disable(USARTx, USART_INT_TBE);
      }
  }

  if(USART_STAT0(USARTx) & USART_STAT0_TC)
  {
    usart_interrupt_flag_clear(USARTx, USART_INT_FLAG_TC);
    if(UARTTxReadPtr==UARTTxWritePtr) SET_RS485_RECEIVE_MODE_1();
  }
}


/****************************************************//**
  \fn WriteRS485(UCHAR Byte)
  \brief Write to the RS485 interface
  \param Byte  Byte to be written

  This function puts a byte into the RS485 transmit
  buffer and starts sending if not already done.
********************************************************/
void WriteRS485(char ch)
{
  int i;

  i=UARTTxWritePtr+1;
  if(i==UART_BUFFER_SIZE) i=0;

  if(i!=UARTTxReadPtr)
  {
    UARTTxBuffer[UARTTxWritePtr]=ch;
    UARTTxWritePtr=i;

    usart_interrupt_enable(USARTx, USART_INT_TBE);
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
char ReadRS485(char *ch)
{
  if(UARTRxReadPtr==UARTRxWritePtr) return FALSE;

  *ch=UARTRxBuffer[UARTRxReadPtr++];
  if(UARTRxReadPtr==UART_BUFFER_SIZE)  UARTRxReadPtr=0;

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
void SetUARTTransmitDelay(uint32_t Delay)
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
uint32_t CheckUARTTimeout(void)
{
  if(UARTTimeoutFlag)
  {
    UARTTimeoutFlag=FALSE;
    return TRUE;
  }
  else return FALSE;
}
