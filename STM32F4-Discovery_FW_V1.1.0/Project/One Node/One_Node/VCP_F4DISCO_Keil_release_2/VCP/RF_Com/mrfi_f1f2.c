/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Shared code between Radio Family 1 & Family 2
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#ifndef MRFI_F1F2
#define MRFI_F1F2
#include "mrfi.h"
#include "mrfi_spi.h"
#include "mrfi_f1f2.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Common
 * ------------------------------------------------------------------------------------------------
 */

/* Packet automation control - base value is power up value whick has APPEND_STATUS enabled; no CRC autoflush */
#define PKTCTRL1_BASE_VALUE         BV(2)
#define PKTCTRL1_ADDR_FILTER_OFF    PKTCTRL1_BASE_VALUE
#define PKTCTRL1_ADDR_FILTER_ON     (PKTCTRL1_BASE_VALUE | (BV(0)|BV(1)))

#define MRFI_ASSERTS_ARE_ON 0

#ifdef MRFI_ASSERTS_ARE_ON
#define RX_FILTER_ADDR_INITIAL_VALUE  0xFF
#endif


/* ------------------------------------------------------------------------------------------------
 *                                       Radio Abstraction
 * ------------------------------------------------------------------------------------------------
 */

/* ----- Radio Family 1 ----- */
//#if (defined MRFI_RADIO_FAMILY1)
#include "mrfi_spi.h"
#define MRFI_WRITE_REGISTER(x,y)      mrfiSpiWriteReg( x, y )



/* ------------------------------------------------------------------------------------------------
 *                                    Global Constants
 * ------------------------------------------------------------------------------------------------
 */
const uint8 mrfiBroadcastAddr[] = { 0xFF, 0xFF, 0xFF, 0xFF };

/* ------------------------------------------------------------------------------------------------
 *                                    Local Constants
 * ------------------------------------------------------------------------------------------------
 */

/*
 *  Logical channel table - this table translates logical channel into
 *  actual radio channel number.  Channel 0, the default channel, is
 *  determined by the channel exported from SmartRF Studio.  The other
 *  table entries are derived from that default.  Each derived channel is
 *  masked with 0xFF to prevent generation of an illegal channel number.
 *
 *  This table is easily customized.  Just replace or add entries as needed.
 *  If the number of entries changes, the corresponding #define must also
 *  be adjusted.  It is located in mrfi_defs.h and is called __mrfi_NUM_LOGICAL_CHANS__.
 *  The static assert below ensures that there is no mismatch.
 */
#if defined( MRFI_CC2500 ) || defined( MRFI_CC2510 ) || defined( MRFI_CC2511 )
static const uint8 mrfiLogicalChanTable[] =
{
  3,
  103,
  202,
  212
};
#elif defined( MRFI_CC1100 ) || defined( MRFI_CC1101 ) || defined( MRFI_CC1110 ) || defined( MRFI_CC1111 )
static const uint8 mrfiLogicalChanTable[] =
{
  0,
  50,
  80,
  110
};
#else
#error "ERROR: A valid radio is not specified."
#endif


/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static uint8 mrfiRxFilterEnabled=0;
static uint8 mrfiRxFilterAddr[MRFI_ADDR_SIZE] = { RX_FILTER_ADDR_INITIAL_VALUE };
static uint8 mrfiRadioState;  // MT check this since it is defined in mrfi_radio.c as well

/**************************************************************************************************
 * @fn          MRFI_SetLogicalChannel
 *
 * @brief       Set logical channel.
 *
 * @param       chan - logical channel number
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_SetLogicalChannel(uint8 chan)
{
  /* logical channel is not valid? */
  if( MRFI_NUM_LOGICAL_CHANS < chan)
	return;

  /* make sure radio is off before changing channels */
  Mrfi_RxModeOff();

  MRFI_WRITE_REGISTER( CHANNR, mrfiLogicalChanTable[chan] );

  /* turn radio back on if it was on before channel change */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }
}


/**************************************************************************************************
 * @fn          MRFI_SetRxAddrFilter
 *
 * @brief       Set the address used for filtering received packets.
 *
 * @param       pAddr - pointer to address to use for filtering
 *
 * @return      zero     : successfully set filter address
 *              non-zero : illegal address
 **************************************************************************************************
 */
uint8 MRFI_SetRxAddrFilter(uint8 * pAddr)
{
  /*
   *  If first byte of filter address match fir byte of broadcast address,
   *  there is a conflict with hardware filtering.
   */
  if (pAddr[0] == mrfiBroadcastAddr[0])
  {
    /* unable to set filter address */
    return( 1 );
  }

  /*
   *  Set the hardware address register.  The hardware address filtering only recognizes
   *  a single byte but this does provide at least some automatic hardware filtering.
   */
  MRFI_WRITE_REGISTER( ADDR, pAddr[0] );

  /* save a copy of the filter address */
  {
    uint8 i;

    for (i=0; i<MRFI_ADDR_SIZE; i++)
    {
      mrfiRxFilterAddr[i] = pAddr[i];
    }
  }

  /* successfully set filter address */
  return( 0 );
}


/**************************************************************************************************
 * @fn          MRFI_EnableRxAddrFilter
 *
 * @brief       Enable received packet filtering.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_EnableRxAddrFilter(void)
{
  if(mrfiRxFilterAddr[0] == mrfiBroadcastAddr[0]) /* filter address must be set before enabling filter */
		return;

  /* set flag to indicate filtering is enabled */
  mrfiRxFilterEnabled = 1;

  /* enable hardware filtering on the radio */
  MRFI_WRITE_REGISTER( PKTCTRL1, PKTCTRL1_ADDR_FILTER_ON );
}


/**************************************************************************************************
 * @fn          MRFI_DisableRxAddrFilter
 *
 * @brief       Disable received packet filtering.
 *
 * @param       pAddr - pointer to address to test for filtering
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_DisableRxAddrFilter(void)
{
  /* clear flag that indicates filtering is enabled */
  mrfiRxFilterEnabled = 0;

  /* disable hardware filtering on the radio */
  MRFI_WRITE_REGISTER( PKTCTRL1, PKTCTRL1_ADDR_FILTER_OFF );
}


/**************************************************************************************************
 * @fn          MRFI_RxAddrIsFiltered
 *
 * @brief       Determine if address is filtered.
 *
 * @param       none
 *
 * @return      zero     : address is not filtered
 *              non-zero : address is filtered
 **************************************************************************************************
 */
uint8 MRFI_RxAddrIsFiltered(uint8 * pAddr)
{
  uint8 i;
  uint8 addrByte;
  uint8 filterAddrMatches;
  uint8 broadcastAddrMatches;

  /* first check to see if filtering is even enabled */
  if (!mrfiRxFilterEnabled)
  {
    /*
     *  Filtering is not enabled, so by definition the address is
     *  not filtered.  Return zero to indicate address is not filtered.
     */
    return( 0 );
  }

  /* clear address byte match counts */
  filterAddrMatches    = 0;
  broadcastAddrMatches = 0;

  /* loop through address to see if there is a match to filter address of broadcast address */
  for (i=0; i<MRFI_ADDR_SIZE; i++)
  {
    /* get byte from address to check */
    addrByte = pAddr[i];

    /* compare byte to filter address byte */
    if (addrByte == mrfiRxFilterAddr[i])
    {
      filterAddrMatches++;
    }
    if (addrByte == mrfiBroadcastAddr[i])
    {
      broadcastAddrMatches++;
    }
  }

  /*
   *  If address is *not* filtered, either the "filter address match count" or
   *  the "broadcast address match count" will equal the total number of bytes
   *  in the address.
   */
  if ((broadcastAddrMatches == MRFI_ADDR_SIZE) || (filterAddrMatches == MRFI_ADDR_SIZE))
  {
    /* address *not* filtered, return zero */
    return( 0 );
  }
  else
  {
    /* address filtered, return non-zero */
    return( 1 );
  }
}


/**************************************************************************************************
*/
#endif
