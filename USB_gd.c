/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/**
  This file contains high level USB communication functions.
*/

#include "gd32f4xx.h"
#include "drv_usbd_int.h"
#include "cdc_acm_core.h"
#include "drv_usb_hw.h"

#include "stepRocker.h"

static usb_core_driver cdc_acm;
static uint8_t USBDataTxBuffer[256];

void usb_timer_irq (void);

/******************************************************************************
  Timer 6 Interrupt: used for delay functions in the USB stack
*******************************************************************************/
void TIMER6_IRQHandler(void)
{
    usb_timer_irq();
}

/*******************************************************************
  USB Interrupt
********************************************************************/
void USBFS_IRQHandler(void)
{
    usbd_isr(&cdc_acm);
}


/***************************************************************//**
   \fn InitUSB()
   \brief Initialize USB interface

    This function initializes the USB interface.
********************************************************************/
void InitUSB(void)
{
  usb_gpio_config();
  usb_rcu_config();
  usb_timer_init();
  usbd_init(&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);
  usb_intr_config();
}

/***************************************************************//**
   \fn DetachUSB()
   \brief Switch off USB interface

    This function switches off the USB interface and so detaches
    the module from USB.
********************************************************************/
void DetachUSB(void)
{
  usb_dev_stop(&cdc_acm);
}

/***************************************************************//**
   \fn USBSendData(uint8_t *Buffer, uint32_t Size)
   \brief Send data via USB
   \param *Buffer Pointer to data buffer
   \param Size Size of data buffer

    This function sends out data via USB.
********************************************************************/
void USBSendData(uint8_t *Buffer, uint32_t Size)
{
  uint32_t i;

  for(i=0; i<Size; i++) USBDataTxBuffer[i]=Buffer[i];

  usbd_ep_send((usb_dev *) &cdc_acm, CDC_DATA_IN_EP, USBDataTxBuffer, Size);
}

/***************************************************************//**
   \fn GetUSB(UCHAR *Cmd)
   \brief Read TMCL command from USB
   \param *Cmd Pointer to nine byte char array
   \return TRUE if nine bytes have been read

    This function tries to read nine bytes from the USB interface
    (one TMCL command). If there are more than nine bytes in the
    USB receive buffer then these bytes will be discarded.
********************************************************************/
uint8_t GetUSBCmd(uint8_t *Cmd)
{
  uint8_t flag;
  uint32_t i;
  usb_cdc_handler *cdc = (usb_cdc_handler *) (&cdc_acm)->dev.class_data[CDC_COM_INTERFACE];

  flag=FALSE;
  if(USBD_CONFIGURED == cdc_acm.dev.cur_status)
  {
    if(cdc->packet_receive)
    {
      if(cdc->receive_length>=9)
      {
        for(i=0; i<9; i++) Cmd[i]=cdc->data[i];
        flag=TRUE;
      }
      cdc->packet_receive=0;
      usbd_ep_recev((usb_dev *) &cdc_acm, CDC_DATA_OUT_EP, (uint8_t *)(cdc->data), USB_CDC_DATA_PACKET_SIZE);
    }
  }
  return flag;
}

/***************************************************************//**
   \fn SendUSBReply(UCHAR *Reply)
   \brief Send nine bytes via USB
   \param *Reply Pointer to nine byte array

    This function sends nine byte (a TMCL reply) out via USB.
********************************************************************/
void SendUSBReply(uint8_t *Reply)
{
	USBSendData(Reply, 9);
}
