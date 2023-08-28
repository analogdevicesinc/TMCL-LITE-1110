
#include "nvic.h"

/***********************************************************************/
/*
 * Initialize the NVIC to enable the specified IRQ.
 * 
 * NOTE: The function only initializes the NVIC to enable a single IRQ. 
 * Interrupts will also need to be enabled in the ARM core. This can be 
 * done using the EnableInterrupts macro.
 *
 * Parameters:
 * irq    irq number to be enabled (the irq number NOT the vector number)
 */

void enable_irq (int irq)
{   
    /* Make sure that the IRQ is an allowable number. Up to 160 is 
     * used.
     *
     * NOTE: If you are using the interrupt definitions from the header
     * file, you MUST SUBTRACT 16!!!
     */
    #if defined(MK22FN512)
    if(irq > 128)
    {
      /* Set the ICPR and ISER registers accordingly */
      NVIC->ICPR[3] = 1 << (irq%32);
      NVIC->ISER[3] = 1 << (irq%32);
    }
    else if(irq > 64)
    {
      /* Set the ICPR and ISER registers accordingly */
      NVIC->ICPR[2] = 1 << (irq%32);
      NVIC->ISER[2] = 1 << (irq%32);
    }
    else if(irq > 32)
    {
      /* Set the ICPR and ISER registers accordingly */
      NVIC->ICPR[1] = 1 << (irq%32);
      NVIC->ISER[1] = 1 << (irq%32);
    }
    else
    {
      /* Set the ICPR and ISER registers accordingly */
      NVIC->ICPR[0] = 1 << (irq%32);
      NVIC->ISER[0] = 1 << (irq%32);
    }
    
    #else
    if(irq > 128)
    {
      /* Set the ICPR and ISER registers accordingly */
      NVICICPR3 = 1 << (irq%32);
      NVICISER3 = 1 << (irq%32);
    }
    else if(irq > 64)
    {
      /* Set the ICPR and ISER registers accordingly */
      NVICICPR2 = 1 << (irq%32);
      NVICISER2 = 1 << (irq%32);
    }
    else if(irq > 32)
    {
      /* Set the ICPR and ISER registers accordingly */
      NVICICPR1 = 1 << (irq%32);
      NVICISER1 = 1 << (irq%32);
    }
    else
    {
      /* Set the ICPR and ISER registers accordingly */
      NVICICPR0 = 1 << (irq%32);
      NVICISER0 = 1 << (irq%32);
    }
    #endif
}
/***********************************************************************/
/*
 * Initialize the NVIC to disable the specified IRQ.
 * 
 * NOTE: The function only initializes the NVIC to disable a single IRQ. 
 * If you want to disable all interrupts, then use the DisableInterrupts
 * macro instead. 
 *
 * Parameters:
 * irq    irq number to be disabled (the irq number NOT the vector number)
 */

void disable_irq (int irq)
{
    
    /* Make sure that the IRQ is an allowable number. Right now up to 32 is 
     * used.
     *
     * NOTE: If you are using the interrupt definitions from the header
     * file, you MUST SUBTRACT 16!!!
     */
    #if defined(MK22FN512)
    if(irq > 128)
    {	
      /* Set the ICER register accordingly */
      NVIC->ICER[3] = 1 << (irq%32);
    }
    else if(irq > 64)
    {	
      /* Set the ICER register accordingly */
      NVIC->ICER[2] = 1 << (irq%32);
    }
    else if(irq > 32)
    {	
      /* Set the ICER register accordingly */
      NVIC->ICER[1] = 1 << (irq%32);
    }
    else
    {	
      /* Set the ICER register accordingly */
      NVIC->ICER[0] = 1 << (irq%32);
    }
    #else
    if(irq > 128)
    {	
      /* Set the ICER register accordingly */
      NVICICER3 = 1 << (irq%32);
    }
    else if(irq > 64)
    {	
      /* Set the ICER register accordingly */
      NVICICER2 = 1 << (irq%32);
    }
    else if(irq > 32)
    {	
      /* Set the ICER register accordingly */
      NVICICER1 = 1 << (irq%32);
    }
    else
    {	
      /* Set the ICER register accordingly */
      NVICICER0 = 1 << (irq%32);
    }
    #endif
}
