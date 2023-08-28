/*
 *    kinetis_sysinit.c - Default init routines for Flycatcher
 *                     		Kinetis ARM systems
 *    Copyright © 2012 Freescale semiConductor Inc. All Rights Reserved.
 */
 
#include "derivative.h"
#include "kinetis_sysinit.h"

/**
 **===========================================================================
 **  External declarations
 **===========================================================================
 */
#if __cplusplus
extern "C" {
#endif
extern uint32_t __vector_table[];
extern unsigned long _estack;
extern void __thumb_startup(void);
#if __cplusplus
}
#endif

/**
 **===========================================================================
 **  Default interrupt handler
 **===========================================================================
 */
void Default_Handler(void)
{
//	__asm("bkpt");
  while(1);
}

/**
 **===========================================================================
 **  Reset handler
 **===========================================================================
 */
void __init_hardware(void)
{
  #if defined(MK22FN512)
  SCB->VTOR = (uint32_t)__vector_table; /* Set the interrupt vector table position */
  #else
	SCB_VTOR = (uint32_t)__vector_table; /* Set the interrupt vector table position */
	#endif

  #if defined(MKL25Z128)
	 // Disable the Watchdog because it may reset the core before entering main().
	SIM_COPC = KINETIS_WDOG_DISABLED_CTRL;
	
	#elif defined(MKE02Z64)
	 // Disable the Watchdog because it may reset the core before entering main().
	WDOG_CNT = 0x20C5;
	WDOG_CNT = 0x28D9;
	WDOG_TOVAL = 0x28D9;
	WDOG_CS2 = WDOG_CS2_CLK(0);
	WDOG_CS1 = WDOG_CS1_UPDATE_MASK;   //watchdog setting can be changed later

  #elif defined(MK20DX128)
	 // Disable the Watchdog because it may reset the core before entering main().
	WDOG_UNLOCK = 0xC520;			// Write 0xC520 to the unlock register
	WDOG_UNLOCK = 0xD928;			// Followed by 0xD928 to complete the unlock
	WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;	// Clear the WDOGEN bit to disable the watchdog

  #elif defined(MK20DN512) || defined(MK20DX256)
	 // Disable the Watchdog because it may reset the core before entering main().
	WDOG_UNLOCK = 0xC520;			// Write 0xC520 to the unlock register
	WDOG_UNLOCK = 0xD928;			// Followed by 0xD928 to complete the unlock
	WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;	// Clear the WDOGEN bit to disable the watchdog

  #elif defined(MK22FX512) || defined(MK22FN512)
	 // Disable the Watchdog because it may reset the core before entering main().
	WDOG_UNLOCK = 0xC520;			// Write 0xC520 to the unlock register
	WDOG_UNLOCK = 0xD928;			// Followed by 0xD928 to complete the unlock
	WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;	// Clear the WDOGEN bit to disable the watchdog
	#else
	#error "MCU sub-model not supported!"
	#endif
}

/* Weak definitions of handlers point to Default_Handler if not implemented */
void NMI_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__ ((weak, alias("Default_Handler")));

void DMA0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 0 transfer complete interrupt */
void DMA1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 1 transfer complete interrupt */
void DMA2_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 2 transfer complete interrupt */
void DMA3_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 3 transfer complete interrupt */
void DMA4_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 4 transfer complete interrupt */
void DMA5_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 5 transfer complete interrupt */
void DMA6_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 6 transfer complete interrupt */
void DMA7_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 7 transfer complete interrupt */
void DMA8_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 8 transfer complete interrupt */
void DMA9_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DMA channel 9 transfer complete interrupt */
void DMA10_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* DMA channel 10 transfer complete interrupt */
void DMA11_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* DMA channel 11 transfer complete interrupt */
void DMA12_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* DMA channel 12 transfer complete interrupt */
void DMA13_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* DMA channel 13 transfer complete interrupt */
void DMA14_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* DMA channel 14 transfer complete interrupt */
void DMA15_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* DMA channel 15 transfer complete interrupt */
void DMA_Error_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* DMA error interrupt */
void MCMNormal_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* MCM normal interrupt */
void FlashCmd_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* Flash memory command complete interrupt */
void FlashReadErr_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Flash read collision interrupt */
void LVD_LVW_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* Low Voltage Detect, Low Voltage Warning */
void LLW_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* Low Leakage Wakeup */
void Watchdog_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* WDOG or EVM interrupt (shared) */
void Reserved39_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 39 */
void I2C0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* I2C0 interrupt */
void I2C1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* I2C1 interrupt */
void SPI0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* SPI0 interrupt */
void SPI1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* SPI1 interrupt */
void SPI2_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	    /* SPI 2 interrupt */
void CAN0Msg_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* CAN0 message buffer (0-15) interrupt */
void CAN0BusOff_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* CAN0 Bus Off interrupt */
void CAN0Error_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* CAN0 Error interrupt */
void CAN0Xmit_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* CAN0 Transmit warning interrupt */  
void CAN0Rcv_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* CAN0 Recieve warning interrupt */
void CAN0Wake_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* CAN0 Wake Up interrupt */ 
void I2S0_Tx_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* I2S0 transmit interrupt */
void I2S0_Rx_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* I2S0 receive interrupt */
void CAN1Msg_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* CAN1Msg interrupt */
void CAN1BusOff_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* CAN1BusOff interrupt */
void CAN1Error_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* CAN1Error interrupt */
void CAN1Xmit_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* CAN1Xmit interrupt */
void CAN1Rcv_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* CAN1Rcv interrupt */
void CAN1Wake_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* CAN1Wake interrupt */
void Reserved59_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 59 */
void UART0_LON_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* UART0 LON interrupt */
void UART0_RX_TX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /* UART0 receive/transmit interrupt */
void UART0Error_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* UART0 error interrupt */
void UART1_RX_TX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /* UART1 receive/transmit interrupt */
void UART1Error_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* UART1 error interrupt */
void UART2_RX_TX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /* UART2 receive/transmit interrupt */
void UART2Error_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* UART2 error interrupt */
void UART3_RX_TX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* UART3 receive/transmit interrupt */
void UART3Error_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* UART3 error interrupt */
void UART4_RX_TX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* UART4 receive/transmit interrupt */
void UART4Error_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* UART4 error interrupt */
void Reserved71_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 71 */
void Reserved72_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 72 */
void ADC0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* ADC0 interrupt */
void ADC1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* ADC1 interrupt */
void CMP0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* CMP0 interrupt */
void CMP1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* CMP1 interrupt */
void CMP2_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* CMP2 interrupt */
void FTM0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* FTM0 fault, all sources interrupt */
void FTM1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* FTM1 fault, all sources interrupt */
void FTM2_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* FTM2 fault, all sources interrupt */
void CMT_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* CMT interrupt */
void RTC_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* RTC alarm interrupt */
void RTCSeconds_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /* RTC seconds interrupt */
void PIT0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* PIT timer channel 0 interrupt */
void PIT1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* PIT timer channel 1 interrupt */
void PIT2_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* PIT timer channel 2 interrupt */
void PIT3_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* PIT timer channel 3 interrupt */
void PDB0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* PDB0 interrupt */
void USB_ISR(void) __attribute__ ((weak, alias("Default_Handler")));	           	/* USB OTG interrupt */
void USBCharge_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* USB charger detect interrupt */
void Reserved91_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 91 */
void Reserved92_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 92 */
void Reserved93_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 93 */
void Reserved94_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 94 */
void Reserved95_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 95 */
void SDHC_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	      /* SDHC interrupt */
void DAC0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* DAC0 interrupt */
void Reserved98_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 98 */
void TSI_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* TSI all sources interrupt */
void MCG_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* MCG interrupt */
void LPTimer_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* Low-power Timer interrupt */
void Reserved102_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 102 */
void PORTA_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* Port A pin detect interrupt */
void PORTB_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* Port B pin detect interrupt */
void PORTC_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* Port C pin detect interrupt */
void PORTD_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* Port D pin detect interrupt */
void PORTE_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));		/* Port E pin detect interrupt */
void Reserved108_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 108 */
void Reserved109_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));	/* Reserved interrupt 109 */
void SWI_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));			/* Software interrupt */

#if defined(MK20DX128)
void Reserved33_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 33*/
void Reserved44_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 44*/
void Reserved53_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 53*/
void Reserved54_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 54*/
void Reserved55_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 55*/
void Reserved56_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 56*/
void Reserved57_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 57*/
void Reserved58_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 58*/
void Reserved66_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 66*/
void Reserved67_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 67*/
void Reserved68_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 68*/
void Reserved69_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 69*/
void Reserved70_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 70*/
void Reserved96_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));  /*Reserved interrupt 96*/
#endif


#if defined(MK22FX512) || defined(MK22FN512)
void Reserved84_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler"))); /* Reserved interrupt 84 */
void Reserved85_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler"))); /* Reserved interrupt 85 */
void FTM3_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));   		 /* FTM3 fault, all sources interrupt */
void DAC1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler"))); 		   /* DAC1 interrupt */
void I2C2_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler"))); 		   /* I2C2 interrupt */
#endif

/* The Interrupt Vector Table for K20xxxVL1x0*/
#if defined(MK20DN512) || defined(MK20DX256)
void (* const InterruptVector[])(void) __attribute__ ((section(".vectortable"))) = {
    /* Processor exceptions */
    (void(*)(void)) &_estack,
    __thumb_startup,
    NMI_Handler,
    HardFault_Handler,
	  MemManage_Handler,
	  BusFault_Handler,
	  UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
	  DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    /* Interrupts */
    DMA0_IRQHandler,		/* DMA channel 0 transfer complete interrupt */
    DMA1_IRQHandler,		/* DMA channel 1 transfer complete interrupt */
    DMA2_IRQHandler,		/* DMA channel 2 transfer complete interrupt */
    DMA3_IRQHandler,		/* DMA channel 3 transfer complete interrupt */
    DMA4_IRQHandler,		/* DMA channel 4 transfer complete interrupt */
    DMA5_IRQHandler,		/* DMA channel 5 transfer complete interrupt */
    DMA6_IRQHandler,		/* DMA channel 6 transfer complete interrupt */
    DMA7_IRQHandler,		/* DMA channel 7 transfer complete interrupt */
    DMA8_IRQHandler,		/* DMA channel 8 transfer complete interrupt */
    DMA9_IRQHandler,		/* DMA channel 9 transfer complete interrupt */
    DMA10_IRQHandler,	/* DMA channel 10 transfer complete interrupt */
    DMA11_IRQHandler,	/* DMA channel 11 transfer complete interrupt */
    DMA12_IRQHandler,	/* DMA channel 12 transfer complete interrupt */
    DMA13_IRQHandler,	/* DMA channel 13 transfer complete interrupt */
    DMA14_IRQHandler,	/* DMA channel 14 transfer complete interrupt */
    DMA15_IRQHandler,	/* DMA channel 15 transfer complete interrupt */
    DMA_Error_IRQHandler,/* DMA error interrupt */
    MCMNormal_IRQHandler,/* MCM normal interrupt */
    FlashCmd_IRQHandler,	/* Flash memory command complete interrupt */
    FlashReadErr_IRQHandler,/* Flash read collision interrupt */
    LVD_LVW_IRQHandler,	/* Low Voltage Detect, Low Voltage Warning */
    LLW_IRQHandler,		/* Low Leakage Wakeup */
    Watchdog_IRQHandler,	/* WDOG or EVM interrupt (shared) */
  	Reserved39_IRQHandler,/* Reserved interrupt 39 */
    I2C0_IRQHandler,		/* I2C0 interrupt */
    I2C1_IRQHandler,		/* I2C1 interrupt */
    SPI0_IRQHandler,		/* SPI0 interrupt */
    SPI1_IRQHandler,		/* SPI1 interrupt */
  	SPI2_IRQHandler,    /* SPI2 interrupt 44 */
		CAN0Msg_IRQHandler,	/* CAN0 message buffer (0-15) interrupt */
		CAN0BusOff_IRQHandler,/* CAN0 Bus Off interrupt */
		CAN0Error_IRQHandler,/* CAN0 Error interrupt */
		CAN0Xmit_IRQHandler,	/* CAN0 Transmit warning interrupt */  
		CAN0Rcv_IRQHandler,	/* CAN0 Recieve warning interrupt */
		CAN0Wake_IRQHandler,	/* CAN0 Wake Up interrupt */ 
    I2S0_Tx_IRQHandler,	/* I2S0 transmit interrupt */
    I2S0_Rx_IRQHandler,	/* I2S0 receive interrupt */
  	CAN1Msg_IRQHandler,/* Reserved interrupt 53 */
   	CAN1BusOff_IRQHandler,/* Reserved interrupt 54 */
   	CAN1Error_IRQHandler,/* Reserved interrupt 55 */
   	CAN1Xmit_IRQHandler,/* Reserved interrupt 56 */
   	CAN1Rcv_IRQHandler,/* Reserved interrupt 57 */
   	CAN1Wake_IRQHandler,/* Reserved interrupt 58 */
   	Reserved59_IRQHandler,/* Reserved interrupt 59 */
    UART0_LON_IRQHandler,/* UART0 LON interrupt */
    UART0_RX_TX_IRQHandler, /* UART0 receive/transmit interrupt */
  	UART0Error_IRQHandler,/* UART0 error interrupt */
    UART1_RX_TX_IRQHandler, /* UART1 receive/transmit interrupt */
    UART1Error_IRQHandler,/* UART1 error interrupt */
    UART2_RX_TX_IRQHandler, /* UART2 receive/transmit interrupt */
    UART2Error_IRQHandler,/* UART2 error interrupt */
  	UART3_RX_TX_IRQHandler,/* UART3 receive/transmit interrupt */
   	UART3Error_IRQHandler,/* UART3 error interrupt */
   	UART4_RX_TX_IRQHandler,/* UART4 receive/transmit */
   	UART4Error_IRQHandler,/* UART4 error interrupt */
   	Reserved71_IRQHandler,/* Reserved interrupt 71 */
   	Reserved72_IRQHandler,/* Reserved interrupt 72 */
    ADC0_IRQHandler,		/* ADC0 interrupt */
    ADC1_IRQHandler,		/* ADC1 interrupt */
    CMP0_IRQHandler,		/* CMP0 interrupt */
    CMP1_IRQHandler,		/* CMP1 interrupt */
    CMP2_IRQHandler,		/* CMP2 interrupt */
    FTM0_IRQHandler,		/* FTM0 fault, all sources interrupt */
    FTM1_IRQHandler,		/* FTM1 fault, all sources interrupt */
    FTM2_IRQHandler,		/* FTM2 fault, all sources interrupt */
    CMT_IRQHandler,		/* CMT interrupt */
    RTC_IRQHandler,		/* RTC alarm interrupt */
    RTCSeconds_IRQHandler,  /* RTC seconds interrupt */
    PIT0_IRQHandler,		/* PIT timer channel 0 interrupt */
    PIT1_IRQHandler,		/* PIT timer channel 1 interrupt */
    PIT2_IRQHandler,		/* PIT timer channel 2 interrupt */
    PIT3_IRQHandler,		/* PIT timer channel 3 interrupt */
    PDB0_IRQHandler,		/* PDB0 interrupt */
    USB_ISR,	          /* USB OTG interrupt */
    USBCharge_IRQHandler,/* USB charger detect interrupt */
  	Reserved91_IRQHandler,/* Reserved interrupt 91 */
  	Reserved92_IRQHandler,/* Reserved interrupt 92 */
   	Reserved93_IRQHandler,/* Reserved interrupt 93 */
   	Reserved94_IRQHandler,/* Reserved interrupt 94 */
   	Reserved95_IRQHandler,/* Reserved interrupt 95 */
   	SDHC_IRQHandler,/* SDHC interrupt */
   	DAC0_IRQHandler,		/* DAC0 interrupt */
   	Reserved98_IRQHandler,/* Reserved interrupt 98 */
    TSI_IRQHandler,		/* TSI all sources interrupt */
    MCG_IRQHandler,		/* MCG interrupt */
    LPTimer_IRQHandler,	/* Low-power Timer interrupt */
   	Reserved102_IRQHandler,/* Reserved interrupt 102 */
    PORTA_IRQHandler,	/* Port A pin detect interrupt */
    PORTB_IRQHandler,	/* Port B pin detect interrupt */
    PORTC_IRQHandler,	/* Port C pin detect interrupt */
    PORTD_IRQHandler,	/* Port D pin detect interrupt */
    PORTE_IRQHandler,	/* Port E pin detect interrupt */
   	Reserved108_IRQHandler,/* Reserved interrupt 108 */
   	Reserved109_IRQHandler,/* Reserved interrupt 109 */
    SWI_IRQHandler			/* Software interrupt */
};

/* The Interrupt Vector Table for K20xxxVL7*/
#elif defined(MK20DX128)
void (* const InterruptVector[])(void) __attribute__ ((section(".vectortable"))) = {
    /* Processor exceptions */
    (void(*)(void)) &_estack,
    __thumb_startup,
    NMI_Handler,
    HardFault_Handler,
	  MemManage_Handler,
	  BusFault_Handler,
	  UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
	  DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    /* Interrupts */
    DMA0_IRQHandler,		/* DMA channel 0 transfer complete interrupt */
    DMA1_IRQHandler,		/* DMA channel 1 transfer complete interrupt */
    DMA2_IRQHandler,		/* DMA channel 2 transfer complete interrupt */
    DMA3_IRQHandler,		/* DMA channel 3 transfer complete interrupt */
    DMA4_IRQHandler,		/* DMA channel 4 transfer complete interrupt */
    DMA5_IRQHandler,		/* DMA channel 5 transfer complete interrupt */
    DMA6_IRQHandler,		/* DMA channel 6 transfer complete interrupt */
    DMA7_IRQHandler,		/* DMA channel 7 transfer complete interrupt */
    DMA8_IRQHandler,		/* DMA channel 8 transfer complete interrupt */
    DMA9_IRQHandler,		/* DMA channel 9 transfer complete interrupt */
    DMA10_IRQHandler,	/* DMA channel 10 transfer complete interrupt */
    DMA11_IRQHandler,	/* DMA channel 11 transfer complete interrupt */
    DMA12_IRQHandler,	/* DMA channel 12 transfer complete interrupt */
    DMA13_IRQHandler,	/* DMA channel 13 transfer complete interrupt */
    DMA14_IRQHandler,	/* DMA channel 14 transfer complete interrupt */
    DMA15_IRQHandler,	/* DMA channel 15 transfer complete interrupt */
    DMA_Error_IRQHandler,/* DMA error interrupt */
    Reserved33_IRQHandler, /* Reserved interrupt 33*/
    FlashCmd_IRQHandler,	/* Flash memory command complete interrupt */
    FlashReadErr_IRQHandler,/* Flash read collision interrupt */
    LVD_LVW_IRQHandler,	/* Low Voltage Detect, Low Voltage Warning */
    LLW_IRQHandler,		/* Low Leakage Wakeup */
    Watchdog_IRQHandler,	/* WDOG or EVM interrupt (shared) */
  	Reserved39_IRQHandler,/* Reserved interrupt 39 */
    I2C0_IRQHandler,		/* I2C0 interrupt */
    I2C1_IRQHandler,		/* I2C1 interrupt */
    SPI0_IRQHandler,		/* SPI0 interrupt */
    SPI1_IRQHandler,		/* SPI1 interrupt */
  	Reserved44_IRQHandler,    /* Reserved interrupt 44 */
		CAN0Msg_IRQHandler,	/* CAN0 message buffer (0-15) interrupt */
		CAN0BusOff_IRQHandler,/* CAN0 Bus Off interrupt */
		CAN0Error_IRQHandler,/* CAN0 Error interrupt */
		CAN0Xmit_IRQHandler,	/* CAN0 Transmit warning interrupt */  
		CAN0Rcv_IRQHandler,	/* CAN0 Recieve warning interrupt */
		CAN0Wake_IRQHandler,	/* CAN0 Wake Up interrupt */ 
    I2S0_Tx_IRQHandler,	/* I2S0 transmit interrupt */
    I2S0_Rx_IRQHandler,	/* I2S0 receive interrupt */
  	Reserved53_IRQHandler,/* Reserved interrupt 53 */
   	Reserved54_IRQHandler,/* Reserved interrupt 54 */
   	Reserved55_IRQHandler,/* Reserved interrupt 55 */
   	Reserved56_IRQHandler,/* Reserved interrupt 56 */
   	Reserved57_IRQHandler,/* Reserved interrupt 57 */
   	Reserved58_IRQHandler,/* Reserved interrupt 58 */
   	Reserved59_IRQHandler,/* Reserved interrupt 59 */
    UART0_LON_IRQHandler,/* UART0 LON interrupt */
    UART0_RX_TX_IRQHandler, /* UART0 receive/transmit interrupt */
  	UART0Error_IRQHandler,/* UART0 error interrupt */
    UART1_RX_TX_IRQHandler, /* UART1 receive/transmit interrupt */
    UART1Error_IRQHandler,/* UART1 error interrupt */
    UART2_RX_TX_IRQHandler, /* UART2 receive/transmit interrupt */
    Reserved66_IRQHandler,/* UART2 error interrupt */
  	Reserved67_IRQHandler,/* UART3 receive/transmit interrupt */
   	Reserved68_IRQHandler,/* UART3 error interrupt */
   	Reserved69_IRQHandler,/* UART4 receive/transmit */
   	Reserved70_IRQHandler,/* UART4 error interrupt */
   	Reserved71_IRQHandler,/* Reserved interrupt 71 */
   	Reserved72_IRQHandler,/* Reserved interrupt 72 */
    ADC0_IRQHandler,		/* ADC0 interrupt */
    ADC1_IRQHandler,		/* ADC1 interrupt */
    CMP0_IRQHandler,		/* CMP0 interrupt */
    CMP1_IRQHandler,		/* CMP1 interrupt */
    CMP2_IRQHandler,		/* CMP2 interrupt */
    FTM0_IRQHandler,		/* FTM0 fault, all sources interrupt */
    FTM1_IRQHandler,		/* FTM1 fault, all sources interrupt */
    FTM2_IRQHandler,		/* FTM2 fault, all sources interrupt */
    CMT_IRQHandler,		/* CMT interrupt */
    RTC_IRQHandler,		/* RTC alarm interrupt */
    RTCSeconds_IRQHandler,  /* RTC seconds interrupt */
    PIT0_IRQHandler,		/* PIT timer channel 0 interrupt */
    PIT1_IRQHandler,		/* PIT timer channel 1 interrupt */
    PIT2_IRQHandler,		/* PIT timer channel 2 interrupt */
    PIT3_IRQHandler,		/* PIT timer channel 3 interrupt */
    PDB0_IRQHandler,		/* PDB0 interrupt */
    USB_ISR,	          /* USB OTG interrupt */
    USBCharge_IRQHandler,/* USB charger detect interrupt */
  	Reserved91_IRQHandler,/* Reserved interrupt 91 */
  	Reserved92_IRQHandler,/* Reserved interrupt 92 */
   	Reserved93_IRQHandler,/* Reserved interrupt 93 */
   	Reserved94_IRQHandler,/* Reserved interrupt 94 */
   	Reserved95_IRQHandler,/* Reserved interrupt 95 */
   	Reserved96_IRQHandler,/* Reserved interrupt 96 */
   	DAC0_IRQHandler,		/* DAC0 interrupt */
   	Reserved98_IRQHandler,/* Reserved interrupt 98 */
    TSI_IRQHandler,		/* TSI all sources interrupt */
    MCG_IRQHandler,		/* MCG interrupt */
    LPTimer_IRQHandler,	/* Low-power Timer interrupt */
   	Reserved102_IRQHandler,/* Reserved interrupt 102 */
    PORTA_IRQHandler,	/* Port A pin detect interrupt */
    PORTB_IRQHandler,	/* Port B pin detect interrupt */
    PORTC_IRQHandler,	/* Port C pin detect interrupt */
    PORTD_IRQHandler,	/* Port D pin detect interrupt */
    PORTE_IRQHandler,	/* Port E pin detect interrupt */
   	Reserved108_IRQHandler,/* Reserved interrupt 108 */
   	Reserved109_IRQHandler,/* Reserved interrupt 109 */
    SWI_IRQHandler			/* Software interrupt */
};

/* The Interrupt Vector Table for K22*/
#elif defined(MK22FX512) || defined(MK22FN512)
void (* const InterruptVector[])(void) __attribute__ ((section(".vectortable"))) = {
    /* Processor exceptions */
    (void(*)(void)) &_estack,
    __thumb_startup,
    NMI_Handler,
    HardFault_Handler,
	  MemManage_Handler,
	  BusFault_Handler,
	  UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
	  DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    /* Interrupts */
    DMA0_IRQHandler,		/* DMA channel 0 transfer complete interrupt */
    DMA1_IRQHandler,		/* DMA channel 1 transfer complete interrupt */
    DMA2_IRQHandler,		/* DMA channel 2 transfer complete interrupt */
    DMA3_IRQHandler,		/* DMA channel 3 transfer complete interrupt */
    DMA4_IRQHandler,		/* DMA channel 4 transfer complete interrupt */
    DMA5_IRQHandler,		/* DMA channel 5 transfer complete interrupt */
    DMA6_IRQHandler,		/* DMA channel 6 transfer complete interrupt */
    DMA7_IRQHandler,		/* DMA channel 7 transfer complete interrupt */
    DMA8_IRQHandler,		/* DMA channel 8 transfer complete interrupt */
    DMA9_IRQHandler,		/* DMA channel 9 transfer complete interrupt */
    DMA10_IRQHandler,	/* DMA channel 10 transfer complete interrupt */
    DMA11_IRQHandler,	/* DMA channel 11 transfer complete interrupt */
    DMA12_IRQHandler,	/* DMA channel 12 transfer complete interrupt */
    DMA13_IRQHandler,	/* DMA channel 13 transfer complete interrupt */
    DMA14_IRQHandler,	/* DMA channel 14 transfer complete interrupt */
    DMA15_IRQHandler,	/* DMA channel 15 transfer complete interrupt */
    DMA_Error_IRQHandler,/* DMA error interrupt */
    MCMNormal_IRQHandler,/* MCM normal interrupt */
    FlashCmd_IRQHandler,	/* Flash memory command complete interrupt */
    FlashReadErr_IRQHandler,/* Flash read collision interrupt */
    LVD_LVW_IRQHandler,	/* Low Voltage Detect, Low Voltage Warning */
    LLW_IRQHandler,		/* Low Leakage Wakeup */
    Watchdog_IRQHandler,	/* WDOG or EVM interrupt (shared) */
  	Reserved39_IRQHandler,/* Reserved interrupt 39 */
    I2C0_IRQHandler,		/* I2C0 interrupt */
    I2C1_IRQHandler,		/* I2C1 interrupt */
    SPI0_IRQHandler,		/* SPI0 interrupt */
    SPI1_IRQHandler,		/* SPI1 interrupt */
    I2S0_Tx_IRQHandler,	/* I2S0 transmit interrupt */
    I2S0_Rx_IRQHandler,	/* I2S0 receive interrupt */
    UART0_LON_IRQHandler,/* UART0 LON interrupt */
    UART0_RX_TX_IRQHandler, /* UART0 receive/transmit interrupt */
  	UART0Error_IRQHandler,/* UART0 error interrupt */
    UART1_RX_TX_IRQHandler, /* UART1 receive/transmit interrupt */
    UART1Error_IRQHandler,/* UART1 error interrupt */
    UART2_RX_TX_IRQHandler, /* UART2 receive/transmit interrupt */
    UART2Error_IRQHandler,/* UART2 error interrupt */
  	UART3_RX_TX_IRQHandler,/* UART3 receive/transmit interrupt */
   	UART3Error_IRQHandler,/* UART3 error interrupt */
    ADC0_IRQHandler,		/* ADC0 interrupt */
    CMP0_IRQHandler,		/* CMP0 interrupt */
    CMP1_IRQHandler,		/* CMP1 interrupt */
    FTM0_IRQHandler,		/* FTM0 fault, all sources interrupt */
    FTM1_IRQHandler,		/* FTM1 fault, all sources interrupt */
    FTM2_IRQHandler,		/* FTM2 fault, all sources interrupt */
    CMT_IRQHandler,		/* CMT interrupt */
    RTC_IRQHandler,		/* RTC alarm interrupt */
    RTCSeconds_IRQHandler,  /* RTC seconds interrupt */
    PIT0_IRQHandler,		/* PIT timer channel 0 interrupt */
    PIT1_IRQHandler,		/* PIT timer channel 1 interrupt */
    PIT2_IRQHandler,		/* PIT timer channel 2 interrupt */
    PIT3_IRQHandler,		/* PIT timer channel 3 interrupt */
    PDB0_IRQHandler,		/* PDB0 interrupt */
    USB_ISR,	          /* USB OTG interrupt */
    USBCharge_IRQHandler,/* USB charger detect interrupt */
   	Reserved71_IRQHandler,/* Reserved interrupt 71 */
   	DAC0_IRQHandler,		/* DAC0 interrupt */
    MCG_IRQHandler,		/* MCG interrupt */
    LPTimer_IRQHandler,	/* Low-power Timer interrupt */
    PORTA_IRQHandler,	/* Port A pin detect interrupt */
    PORTB_IRQHandler,	/* Port B pin detect interrupt */
    PORTC_IRQHandler,	/* Port C pin detect interrupt */
    PORTD_IRQHandler,	/* Port D pin detect interrupt */
    PORTE_IRQHandler,	/* Port E pin detect interrupt */
    SWI_IRQHandler,			/* Software interrupt */
  	SPI2_IRQHandler,    /* SPI2 interrupt */
   	UART4_RX_TX_IRQHandler,/* UART4 receive/transmit */
   	UART4Error_IRQHandler,/* UART4 error interrupt */
   	Reserved84_IRQHandler,/* Reserved interrupt 84 */
   	Reserved85_IRQHandler,/* Reserved interrupt 85 */
    CMP2_IRQHandler,		/* CMP2 interrupt */
    FTM3_IRQHandler,		/* FTM3 fault, all sources interrupt */
   	DAC1_IRQHandler,		/* DAC1 interrupt */
    ADC1_IRQHandler,		/* ADC1 interrupt */
    I2C2_IRQHandler,		/* I2C2 interrupt */
		CAN0Msg_IRQHandler,	/* CAN0 message buffer (0-15) interrupt */
		CAN0BusOff_IRQHandler,/* CAN0 Bus Off interrupt */
		CAN0Error_IRQHandler,/* CAN0 Error interrupt */
		CAN0Xmit_IRQHandler,	/* CAN0 Transmit warning interrupt */  
		CAN0Rcv_IRQHandler,	/* CAN0 Recieve warning interrupt */
		CAN0Wake_IRQHandler,	/* CAN0 Wake Up interrupt */ 
   	SDHC_IRQHandler /* SDHC interrupt */
};

#else
#error "No interrupt table for submodel"
#endif
