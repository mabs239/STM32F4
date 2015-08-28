/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Include file for MRFI common code between
 *   radio family 1 and radio family 2.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_COMMON_H
#define MRFI_COMMON_H
#define BV(n)      (1 << (n))

#define MRFI_CC1101

/* --------------------------------------------------------------------------------------------
 *                                       SmartRF Configuration
 * --------------------------------------------------------------------------------------------
 */
#if (defined MRFI_CC1100)
#include "smartrf_CC1100.h"
#elif (defined MRFI_CC1101)
#include "smartrf_CC1101.h"
#else
#error "ERROR: A valid radio is not specified."
#endif


/* --------------------------------------------------------------------------------------------
 *                                         Prototypes
 * --------------------------------------------------------------------------------------------
 */
uint8 MRFI_RxAddrIsFiltered(uint8 * pAddr);


/*******************************************************************************************
 */
#endif
