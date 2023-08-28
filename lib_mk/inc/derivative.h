/*
 * Note: This file is recreated by the project wizard whenever the MCU is
 *       changed and should not be edited by hand
 */

/* Include the derivative-specific header file */

#if defined(MKL25Z128)
  #include <MKL25Z4.h>
#elif defined(MKE02Z64)
  #include <MKE02Z2.h>
#elif defined(MK20DX128)
  #include <MK20D7.h>
  #define CPU_LITTLE_ENDIAN   //Wichtig für den USB-Stack!!!
  #define __MK_xxx_H__        //Wichtig für den USB-Stack!!!
#elif defined(MK20DN512) || defined(MK20DX256)
  #include <MK20D10.h>
  #define CPU_LITTLE_ENDIAN   //Wichtig für den USB-Stack!!!
  #define __MK_xxx_H__        //Wichtig für den USB-Stack!!!
#elif defined(MK22FX512)
  #include <MK22F12.h>
  #define CPU_LITTLE_ENDIAN   //Wichtig für den USB-Stack!!!
  #define __MK_xxx_H__        //Wichtig für den USB-Stack!!!
#elif defined(MK22FN512)
  #include <MK22F51212.h>
  #define CPU_LITTLE_ENDIAN   //Wichtig für den USB-Stack!!!
  #define __MK_xxx_H__        //Wichtig für den USB-Stack!!!
#else
  #error "Unknown derivative. Check Makefile (SUBMDL)."
#endif
