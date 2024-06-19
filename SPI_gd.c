/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file provides all functions needed for SPI
  access to the other ICs (TMC429, TMC262, EEPROM).
*/

#include "gd32f4xx.h"
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
  spi_parameter_struct SPIInit;

  //Initialize SPI0 and link with PA5..PA7 pins
  rcu_periph_clock_enable(RCU_SPI0);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5);
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5);
  gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5);

  spi_i2s_deinit(SPI0);
  spi_struct_para_init(&SPIInit);

  SPIInit.trans_mode=SPI_TRANSMODE_FULLDUPLEX;
  SPIInit.device_mode=SPI_MASTER;
  SPIInit.frame_size=SPI_FRAMESIZE_8BIT;
  SPIInit.clock_polarity_phase=SPI_CK_PL_HIGH_PH_2EDGE;
  SPIInit.nss=SPI_NSS_SOFT;
  SPIInit.prescale=SPI_PSC_64;
  SPIInit.endian=SPI_ENDIAN_MSB;
  spi_init(SPI0, &SPIInit);
  spi_nss_internal_high(SPI0);
  spi_enable(SPI0);

  //Initialize SPI2 and link with PC10..PC12 pins
  rcu_periph_clock_enable(RCU_SPI2);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10);
  gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10);
  gpio_af_set(GPIOC, GPIO_AF_6, GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10);

  spi_i2s_deinit(SPI2);
  spi_struct_para_init(&SPIInit);

  SPIInit.trans_mode=SPI_TRANSMODE_FULLDUPLEX;
  SPIInit.device_mode=SPI_MASTER;
  SPIInit.frame_size=SPI_FRAMESIZE_8BIT;
  SPIInit.clock_polarity_phase=SPI_CK_PL_HIGH_PH_2EDGE;
  SPIInit.nss=SPI_NSS_SOFT;
  SPIInit.prescale=SPI_PSC_16;
  SPIInit.endian=SPI_ENDIAN_MSB;
  spi_init(SPI2, &SPIInit);
  spi_nss_internal_high(SPI2);
  spi_enable(SPI2);
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
uint8_t ReadWriteSPI(uint8_t DeviceNumber, uint8_t Data, uint8_t LastTransfer)
{
  //Set CS low
  switch(DeviceNumber)
  {
    #ifdef SPI_DEV_EEPROM
    case SPI_DEV_EEPROM:    SELECT_EEPROM(); break;
    #endif
    #ifdef SPI_DEV_TMC43xx_0
    case SPI_DEV_TMC43xx_0: SELECT_TMC43xx_0(); break;
    #endif
    #ifdef SPI_DEV_TMC5160_0
    case SPI_DEV_TMC5160_0: SELECT_TMC5160_0(); break;
    #endif
    #ifdef SPI_DEV_TMC429
    case SPI_DEV_TMC429:    SELECT_TMC429(); break;
    #endif
    #ifdef SPI_DEV_TMC262
    case SPI_DEV_TMC262:    SELECT_TMC262(); break;
    #endif

    default:
      return 0;
      break;
  }

  switch(DeviceNumber)
  {
    #ifdef SPI_DEV_TMC43xx_0
    case SPI_DEV_TMC43xx_0:
      while(spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)==RESET);
      spi_i2s_data_transmit(SPI0, Data);
      while(spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)==RESET);
      break;
    #endif

    #ifdef SPI_DEV_TMC5160_0
    case SPI_DEV_TMC5160_0:
      while(spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)==RESET);
      spi_i2s_data_transmit(SPI0, Data);
      while(spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)==RESET);
      break;
    #endif

    #ifdef SPI_DEV_TMC429
    case SPI_DEV_TMC429:
      while(spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)==RESET);
      spi_i2s_data_transmit(SPI0, Data);
      while(spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)==RESET);
      break;
    #endif

    #ifdef SPI_DEV_TMC262
    case SPI_DEV_TMC262:
      while(spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)==RESET);
      spi_i2s_data_transmit(SPI0, Data);
      while(spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)==RESET);
      break;
    #endif

    #ifdef SPI_DEV_EEPROM
    case SPI_DEV_EEPROM:
      while(spi_i2s_flag_get(SPI2, SPI_FLAG_TBE)==RESET);
      spi_i2s_data_transmit(SPI2, Data);
      while(spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE)==RESET);
      break;
    #endif
  }

  //Set CS high when this was the last transfer
  if(LastTransfer)
  {
    switch(DeviceNumber)
    {
      #ifdef SPI_DEV_EEPROM
      case SPI_DEV_EEPROM:    DESELECT_EEPROM(); break;
      #endif
      #ifdef SPI_DEV_TMC43xx_0
      case SPI_DEV_TMC43xx_0: DESELECT_TMC43xx_0(); break;
      #endif
      #ifdef SPI_DEV_TMC5160_0
      case SPI_DEV_TMC5160_0: DESELECT_TMC5160_0(); break;
      #endif
      #ifdef SPI_DEV_TMC429
      case SPI_DEV_TMC429: DESELECT_TMC429(); break;
      #endif
      #ifdef SPI_DEV_TMC262
      case SPI_DEV_TMC262: DESELECT_TMC262(); break;
      #endif

      default:
        return 0;
        break;
    }
  }

  switch(DeviceNumber)
  {
    #ifdef SPI_DEV_TMC43xx_0
    case SPI_DEV_TMC43xx_0:
      return spi_i2s_data_receive(SPI0);
    #endif

    #ifdef SPI_DEV_TMC5160_0
    case SPI_DEV_TMC5160_0:
      return spi_i2s_data_receive(SPI0);
    #endif

    #ifdef SPI_DEV_TMC429
    case SPI_DEV_TMC429:
      return spi_i2s_data_receive(SPI0);
    #endif

    #ifdef SPI_DEV_TMC262
    case SPI_DEV_TMC262:
      return spi_i2s_data_receive(SPI0);
    #endif

    #ifdef SPI_DEV_EEPROM
    case SPI_DEV_EEPROM:
      return spi_i2s_data_receive(SPI2);
    #endif

    default:
      return 0;
  }
}
