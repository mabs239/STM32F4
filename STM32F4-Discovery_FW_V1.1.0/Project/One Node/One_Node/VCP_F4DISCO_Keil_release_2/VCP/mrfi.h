/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Include file for all MRFI services.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_H
#define MRFI_H
//#include "mrfi_spi.h"
//#include "mrfi_board_defs.h"
//#include "mrfi_f1f2.h"
//#include "mrfi.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#define int8 int8_t
#define uint8 uint8_t
#define uint16 uint16_t
#define uint32 uint32_t


/* ------------------------------------------------------------------------------------------------
 *                                       Common Defines
 * ------------------------------------------------------------------------------------------------
 */
#define MRFI_CCA_RETRIES        4


/* ------------------------------------------------------------------------------------------------
 *                                    Radio Family Assigment
 * ------------------------------------------------------------------------------------------------
 */


#define MRFI_RADIO_FAMILY1


/* ------------------------------------------------------------------------------------------------
 *                                Radio Family 1 / Radio Family 2
 * ------------------------------------------------------------------------------------------------
 */
#if (defined MRFI_RADIO_FAMILY1) || (defined MRFI_RADIO_FAMILY2)

#define __mrfi_LENGTH_FIELD_SIZE__      1
#define __mrfi_ADDR_SIZE__              4
#define __mrfi_MAX_PAYLOAD_SIZE__       20

#define __mrfi_RX_METRICS_SIZE__        2
#define __mrfi_RX_METRICS_RSSI_OFS__    0
#define __mrfi_RX_METRICS_CRC_LQI_OFS__ 1
#define __mrfi_RX_METRICS_CRC_OK_MASK__ 0x80
#define __mrfi_RX_METRICS_LQI_MASK__    0x7F

#define __mrfi_NUM_LOGICAL_CHANS__      4

#define __mrfi_BACKOFF_PERIOD_USECS__   250

#define __mrfi_LENGTH_FIELD_OFS__       0
#define __mrfi_DST_ADDR_OFS__           (__mrfi_LENGTH_FIELD_OFS__ + __mrfi_LENGTH_FIELD_SIZE__)
#define __mrfi_SRC_ADDR_OFS__           (__mrfi_DST_ADDR_OFS__ + __mrfi_ADDR_SIZE__)
#define __mrfi_PAYLOAD_OFS__            (__mrfi_SRC_ADDR_OFS__ + __mrfi_ADDR_SIZE__)

#define __mrfi_HEADER_SIZE__            (2 * __mrfi_ADDR_SIZE__)
#define __mrfi_FRAME_OVERHEAD_SIZE__    (__mrfi_LENGTH_FIELD_SIZE__ + __mrfi_HEADER_SIZE__)

#define __mrfi_GET_PAYLOAD_LEN__(p)     ((p)->frame[__mrfi_LENGTH_FIELD_OFS__] - __mrfi_HEADER_SIZE__)
#define __mrfi_SET_PAYLOAD_LEN__(p,x)   st( (p)->frame[__mrfi_LENGTH_FIELD_OFS__] = x + __mrfi_HEADER_SIZE__; )

#endif

/* ------------------------------------------------------------------------------------------------
 *                                   Radio Family Commonality
 * ------------------------------------------------------------------------------------------------
 */
#define __mrfi_P_DST_ADDR__(p)          (&((p)->frame[__mrfi_DST_ADDR_OFS__]))
#define __mrfi_P_SRC_ADDR__(p)          (&((p)->frame[__mrfi_SRC_ADDR_OFS__]))
#define __mrfi_P_PAYLOAD__(p)           (&((p)->frame[__mrfi_PAYLOAD_OFS__]))



/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
	

#define MRFI_NUM_LOGICAL_CHANS           __mrfi_NUM_LOGICAL_CHANS__

/* return values for MRFI_Transmit */
#define MRFI_TX_RESULT_SUCCESS        0
#define MRFI_TX_RESULT_FAILED         1

/* transmit type parameter for MRFI_Transmit */
#define MRFI_TX_TYPE_FORCED           1
#define MRFI_TX_TYPE_CCA              0

/* if external code has defined a maximum payload, use that instead of default */
#ifdef MAX_APP_PAYLOAD
#define MRFI_MAX_PAYLOAD_SIZE  (MAX_APP_PAYLOAD+3) /* SimpliciTI payload size plus three byte overhead */
#endif

/* frame definitions */
#define MRFI_ADDR_SIZE              __mrfi_ADDR_SIZE__
#ifndef MRFI_MAX_PAYLOAD_SIZE
#define MRFI_MAX_PAYLOAD_SIZE       __mrfi_MAX_PAYLOAD_SIZE__
#endif
#define MRFI_MAX_FRAME_SIZE         (MRFI_MAX_PAYLOAD_SIZE + __mrfi_FRAME_OVERHEAD_SIZE__)
#define MRFI_RX_METRICS_SIZE        __mrfi_RX_METRICS_SIZE__
#define MRFI_RX_METRICS_RSSI_OFS    __mrfi_RX_METRICS_RSSI_OFS__
#define MRFI_RX_METRICS_CRC_LQI_OFS __mrfi_RX_METRICS_CRC_LQI_OFS__

/* Radio States */
#define MRFI_RADIO_STATE_UNKNOWN  0
#define MRFI_RADIO_STATE_OFF      1
#define MRFI_RADIO_STATE_IDLE     2
#define MRFI_RADIO_STATE_RX       3

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MRFI_GET_PAYLOAD_LEN(p)         __mrfi_GET_PAYLOAD_LEN__(p)
#define MRFI_SET_PAYLOAD_LEN(p,x)       __mrfi_SET_PAYLOAD_LEN__(p,x)

#define MRFI_P_DST_ADDR(p)              __mrfi_P_DST_ADDR__(p)
#define MRFI_P_SRC_ADDR(p)              __mrfi_P_SRC_ADDR__(p)
#define MRFI_P_PAYLOAD(p)               __mrfi_P_PAYLOAD__(p)

/* ------------------------------------------------------------------------------------------------
 *                                          Typdefs
 * ------------------------------------------------------------------------------------------------
 */
typedef struct
{
  uint8 frame[MRFI_MAX_FRAME_SIZE];
  uint8 rxMetrics[MRFI_RX_METRICS_SIZE];
} mrfiPacket_t;


/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
#ifdef does_into_does
void MRFI_Init(void);
void MRFI_GpioIsr(void); 

uint8 MRFI_Transmit(mrfiPacket_t * pPacket, uint8 txType);

void MRFI_Receive(mrfiPacket_t * pPacket);
void MRFI_RxCompleteISR(void); /* populated by code using MRFI */

uint8 MRFI_GetRadioState(void);

void MRFI_RxOn(void);
void MRFI_RxIdle(void);

int8 MRFI_Rssi(void);

void MRFI_SetLogicalChannel(uint8 chan);

uint8 MRFI_SetRxAddrFilter(uint8 * pAddr);
void MRFI_EnableRxAddrFilter(void);
void MRFI_DisableRxAddrFilter(void);

void MRFI_Sleep(void);
void MRFI_WakeUp(void);

uint8 MRFI_RandomByte(void);
void MRFI_DelayMs(uint16 milliseconds);
void MRFI_ReplyDelay(void);
void MRFI_PostKillSem(void);

void Mrfi_RxModeOn(void);
void Mrfi_RxModeOff(void);
#endif
/* ------------------------------------------------------------------------------------------------
 *                                       Global Constants
 * ------------------------------------------------------------------------------------------------
 */
extern const uint8 mrfiBroadcastAddr[];


/**************************************************************************************************
 */
#endif
