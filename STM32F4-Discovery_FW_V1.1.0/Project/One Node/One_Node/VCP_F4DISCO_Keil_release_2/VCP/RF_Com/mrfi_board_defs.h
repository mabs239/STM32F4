
#ifndef MRFI_BOARD_DEFS_H
#define MRFI_BOARD_DEFS_H
#define uint8 uint8_t
#define uint16 uint16_t
#define uint32 uint32_t
//#include "msp430x54x.h"
//#include  "../types.h"
//#include  "../defs.h"
//#include "../Interfaces/spi.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#define GDO2_PORT              GPIOB
#define GDO2_PIN               GPIO_Pin_7
typedef int mrfiSpiIState_t;
/* ------------------------------------------------------------------------------------------------
 *                                      GDO0 Pin Configuration
 * ------------------------------------------------------------------------------------------------
 */
/*
void Hello_hunny_bunny()
{
	    GPIO_InitTypeDef GPIO_InitStructure;
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);


}
*/


#define MRFI_CONFIG_GDO2_PIN_AS_INPUT()       GPIO_InitTypeDef GPIO_InitStructure; \
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; \
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
GPIO_Init(GPIOC, &GPIO_InitStructure);

#define MRFI_GDO2_PIN_IS_HIGH()               GPIO_ReadInputData(GPIOB)//MCU_IO_GET(GDO2_PORT, GDO2_PIN)

//#define MRFI_GDO0_INT_VECTOR                  ISR_hi
#define MRFI_ENABLE_GDO2_INT()                EXTI->IMR |=0x01;//st(P2IE |= BIT5; )  /* atomic operation */
#define MRFI_DISABLE_GDO2_INT()               EXTI->IMR &=0xFE;//st(P2IE &= ~BIT5; ) /* atomic operation */
#define MRFI_GDO2_INT_IS_ENABLED()            EXTI->IMR & 0x01;
#define MRFI_CLEAR_GDO2_INT_FLAG()            EXTI_ClearFlag(EXTI_Line0) /* atomic operation */
#define MRFI_GDO2_INT_FLAG_IS_SET()            EXTI_GetITStatus(EXTI_Line0)//(  P2IFG & BIT5 )
#define MRFI_CONFIG_GDO2_RISING_EDGE_INT()    ;//st( P2IES &= ~BIT5; ) /* atomic operation */
//#define MRFI_CONFIG_GDO2_FALLING_EDGE_INT()   st( P2IES |= BIT5; ) /* atomic operation */


/* ------------------------------------------------------------------------------------------------
 *                                      GDO2 Pin Configuration
 * ------------------------------------------------------------------------------------------------
 */
//#define MRFI_CONFIG_GDO2_PIN_AS_INPUT()       st( TRISDbits.TRISD3 = INPUT; )
//#define MRFI_GDO2_PIN_IS_HIGH()               (PORTDbits.RD3)
//
//#define MRFI_GDO2_INT_VECTOR                  ISR_hi
//#define MRFI_ENABLE_GDO2_INT()                st(INTCON3bits.INT2IE = 1; ) /* atomic operation */
//#define MRFI_DISABLE_GDO2_INT()               st(INTCON3bits.INT2IE = 0; ) /* atomic operation */
//#define MRFI_GDO2_INT_IS_ENABLED()             (  INTCON3bits.INT2IE )
//#define MRFI_CLEAR_GDO2_INT_FLAG()            st( INTCON3bits.INT2IF = 0; ) /* atomic operation */
//#define MRFI_GDO2_INT_FLAG_IS_SET()            (  INTCON3bits.INT2IF )
//#define MRFI_CONFIG_GDO2_RISING_EDGE_INT()    st( INTCON2bits.INTEDG2 = 0; ) /* atomic operation */
//#define MRFI_CONFIG_GDO2_FALLING_EDGE_INT()   st( INTCON2bits.INTEDG2 = 1; ) /* atomic operation */
//

/* ------------------------------------------------------------------------------------------------
 *                                      SPI Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* CSn Pin Configuration */
#define MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT()   ;//HAL_SPI_CS_PIN_DIR_OUTPUT
#define MRFI_SPI_DRIVE_CSN_HIGH()             GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define MRFI_SPI_DRIVE_CSN_LOW()              GPIO_ResetBits(GPIOB, GPIO_Pin_12)//HAL_SPI_CS_ASSERT
#define MRFI_SPI_CSN_IS_HIGH()                GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_12)

/* SCLK Pin Configuration */
#define MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT()  ;//HAL_SPI_CLK_PIN_DIR_OUTPUT
#define MRFI_SPI_DRIVE_SCLK_HIGH()            ;//HAL_SPI_CLK_PIN_HIGH
#define MRFI_SPI_DRIVE_SCLK_LOW()             ;//HAL_SPI_CLK_PIN_LOW

/* SI Pin Configuration */
#define MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT()    ;//HAL_SPI_SIMO_PIN_DIR_OUTPUT
#define MRFI_SPI_DRIVE_SI_HIGH()              	GPIO_SetBits(GPIOB,GPIO_Pin_15)//HAL_SPI_SIMO_PIN_HIGH
#define MRFI_SPI_DRIVE_SI_LOW()                 GPIO_ResetBits(GPIOB,GPIO_Pin_15)

/* SO Pin Configuration */
#define MRFI_SPI_CONFIG_SO_PIN_AS_INPUT()     ;//HAL_SPI_SOMI_PIN_DIR_INPUT
#define MRFI_SPI_SO_IS_HIGH()                 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)//HAL_SPI_SOMI_VAL

/* SPI Port Configuration */
#define MRFI_SPI_CONFIG_PORT()                ;//st( UCB0CTL1 &= ~UCSWRST; )
          

/* read/write macros */
#define MRFI_SPI_WRITE_BYTE(x)                SPI_I2S_SendData(SPI2, x);//HAL_SPI_TXBUF_SET(x)
#define MRFI_SPI_READ_BYTE()                  SPI_I2S_ReceiveData(SPI2);//HAL_SPI_RXBUF
#define MRFI_SPI_WAIT_DONE()                  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

/* SPI critical section macros */
//typedef istate_t mrfiSpiIState_t;
#define MRFI_SPI_ENTER_CRITICAL_SECTION(x)    if(x | 1)//MCU_ENTER_CRITICAL_SECTION(x)
#define MRFI_SPI_EXIT_CRITICAL_SECTION(x)    if(x | 1)// MCU_EXIT_CRITICAL_SECTION(x)


/*
 *  Radio SPI Specifications
 * -----------------------------------------------
 *    Max SPI Clock   :  10 MHz
 *    Data Order      :  MSB transmitted first
 *    Clock Polarity  :  low when idle
 *    Clock Phase     :  sample leading edge
 */

/* initialization macro */
#define MRFI_SPI_INIT		      if(1)//Spi_Init_Rf

#define MRFI_SPI_IS_INITIALIZED()     (UCB0CTL0 >> 7 | UCB0CTL0 >> 6)

/**************************************************************************************************
 */
#endif



