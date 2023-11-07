/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides all functions needed for SPI
  access to the other ICs (TMC429, TMC262, EEPROM).
*/

#include "derivative.h"
#include "bits.h"
#include "stepRocker.h"


/****************************************************//**
  \fn InitSPI(void)
  \brief SPI intialization

  This functions initializes the SPI. It has to be called
  once at the beginning of the main() function, before
  any other things are done that need SPI access.
********************************************************/
void InitSPI(void)
{
  //Switch on clock for SPI0 and SPI1
  SIM_SCGC6|=SIM_SCGC6_SPI1_MASK|SIM_SCGC6_SPI0_MASK;

  //Configure SPI0
  SPI0_MCR=  SPI_MCR_HALT_MASK;
  SPI0_MCR|= SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0x1f) | SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
  SPI0_CTAR0= SPI_CTAR_FMSZ(7)|SPI_CTAR_CPOL_MASK|SPI_CTAR_CPHA_MASK|SPI_CTAR_PBR(1)|SPI_CTAR_BR(3);  //8 Bit, SPI Mode 3, 2MHz
  SPI0_CTAR1= SPI_CTAR_FMSZ(7)|SPI_CTAR_CPOL_MASK|SPI_CTAR_CPHA_MASK|SPI_CTAR_PBR(1)|SPI_CTAR_BR(3);  //8 Bit, SPI Mode 3, 2MHz
  SPI0_SR|=  SPI_SR_EOQF_MASK;
  SPI0_MCR&= ~SPI_MCR_HALT_MASK;
  SPI0_MCR&= ~SPI_MCR_FRZ_MASK;


  //Put SPI0 pins on PTC4/PTC5/PTC6/PTC7
  PORTC_PCR4=PORT_PCR_MUX(2);
  PORTC_PCR5=PORT_PCR_MUX(2);
  PORTC_PCR6=PORT_PCR_MUX(2);
  PORTC_PCR7=PORT_PCR_MUX(2);

  //Configure SPI1
  SPI1_MCR=  SPI_MCR_HALT_MASK;
  SPI1_MCR|= SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0x1f) | SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
  SPI1_CTAR0= SPI_CTAR_FMSZ(7)|SPI_CTAR_CPOL_MASK|SPI_CTAR_CPHA_MASK|SPI_CTAR_PBR(1)|SPI_CTAR_BR(4)|SPI_CTAR_PCSSCK(1)|SPI_CTAR_PASC(1);
  SPI1_CTAR1= SPI_CTAR_FMSZ(7)|SPI_CTAR_CPOL_MASK|SPI_CTAR_CPHA_MASK|SPI_CTAR_PBR(1)|SPI_CTAR_BR(4)|SPI_CTAR_PCSSCK(3)|SPI_CTAR_PASC(3); //8 Bit, SPI Mode 3,  2MHz
  SPI1_SR|=  SPI_SR_EOQF_MASK;
  SPI1_MCR&= ~SPI_MCR_HALT_MASK;
  SPI1_MCR&= ~SPI_MCR_FRZ_MASK;

  //Put SPI1 pins on PTE0/PTE1/PTE2/PTE3/PTE4
  PORTE_PCR0=PORT_PCR_MUX(2);
  PORTE_PCR1=PORT_PCR_MUX(2);
  PORTE_PCR2=PORT_PCR_MUX(2);
  PORTE_PCR3=PORT_PCR_MUX(2);
  PORTE_PCR4=PORT_PCR_MUX(2);
}


/***************************************************************//**
   \fn ReadWriteSPI(UCHAR DeviceNumber, UCHAR WriteData, UCHAR LastTransfer)
   \brief SPI communication

   \param DeviceNumber  Index of the SPI device (see the constants in stepRocker.h)
   \param WriteData     Data byte to be sent
   \param LastTransfer  FALSE: device will be kept selected (for sending more bytes) \n
                        TRUE: the device will be deselected after the transfer \n

   \return Received byte

   This function handles SPI transfer. One byte will be sent, and
   one byte will be received.
********************************************************************/
UCHAR ReadWriteSPI(USHORT DeviceNumber, UCHAR WriteData, UCHAR LastTransfer)
{
  UCHAR ReadData;

  if(DeviceNumber<0x0100)
  {
    if(LastTransfer)
    {
      SPI0_PUSHR=SPI_PUSHR_EOQ_MASK|SPI_PUSHR_TXDATA(WriteData)|SPI_PUSHR_PCS(DeviceNumber & 0x3f);
      while (!(SPI0_SR & SPI_SR_TCF_MASK));
      while (!(SPI0_SR & SPI_SR_EOQF_MASK));
      ReadData = SPI0_POPR;
      SPI0_SR|=SPI_SR_EOQF_MASK|SPI_SR_TCF_MASK;
    }
    else
    {
      SPI0_PUSHR=SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(WriteData)|SPI_PUSHR_PCS(DeviceNumber & 0x3f);
      while (!(SPI0_SR & SPI_SR_TCF_MASK));
      ReadData = SPI0_POPR;
      SPI0_SR|=SPI_SR_EOQF_MASK|SPI_SR_TCF_MASK;
    }
  }
  else if(DeviceNumber<0x0200)
  {
    if(LastTransfer)
    {
      SPI1_PUSHR=SPI_PUSHR_EOQ_MASK|SPI_PUSHR_TXDATA(WriteData)|SPI_PUSHR_PCS(DeviceNumber & 0x3f);
      while (!(SPI1_SR & SPI_SR_TCF_MASK));
      while (!(SPI1_SR & SPI_SR_EOQF_MASK));
      ReadData = SPI1_POPR;
      SPI1_SR|=SPI_SR_EOQF_MASK|SPI_SR_TCF_MASK;
    }
    else
    {
      SPI1_PUSHR=SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(WriteData)|SPI_PUSHR_PCS(DeviceNumber & 0x3f);
      while (!(SPI1_SR & SPI_SR_TCF_MASK));
      ReadData = SPI1_POPR;
      SPI1_SR|=SPI_SR_EOQF_MASK|SPI_SR_TCF_MASK;
    }
  }
  else ReadData=0;

  return ReadData;
}
