/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H
#include "stm32f4_discovery.h"

#define GPIO_SEND_CS                  			GPIOA

#define GPIO_SEND_Pin_CS              		GPIO_Pin_0 

#define GPIO_SEND_Pin_SCLK			  GPIO_Pin_5
#define GPIO_SEND_Pin_SO			  	GPIO_Pin_6
#define GPIO_SEND_Pin_SI			  	GPIO_Pin_7
 
 #define GPIO_SEND_Pin_GD0			  	GPIO_Pin_8
 #define GPIO_SEND_Pin_GD2			  	GPIO_Pin_10



 typedef enum
{
	CC1101_TYPE_SEND,
	CC1101_TYPE_RECEIVE,
}CC1101_TYPE;

/* Exported macro ------------------------------------------------------------*/
/* Select SPI : Chip Select pin low  */
#define SPI_SEND_CS_LOW()       GPIO_ResetBits(GPIOB, GPIO_Pin_0)
/* Deselect SPI : Chip Select pin high */
#define SPI_SEND_CS_HIGH()      GPIO_SetBits(GPIOB, GPIO_Pin_0)


/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void SPI_FLASH_Init(void);
void SPI_FLASH_PageErase(uint32_t SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t SPI_FLASH_ReadID(void);

/*----- Low layer function -----*/
uint8_t SPI_FLASH_ReadByte(CC1101_TYPE type);
uint8_t SPI_FLASH_SendByte(uint8_t byte, CC1101_TYPE type);
uint16_t SPI_FLASH_SendHalfWord(uint16_t HalfWord);
void SPI_FLASH_WaitForWriteEnd(void);

void CC1101_Init(void);
void	CC1101_POWER_RESET(void);
void CC1101_Send_Main(uint16_t Angle31);
void CC1101_Receive_Main(void);
void halRfWriteRfSettings(CC1101_TYPE type);
void Recieve();
uint8_t getFlagID();
uint8_t getExtiFlag();
#endif
