
#include "cc1101.h"
#include "stm32f4_discovery.h"
#include "main.h"

#define   Dummy_Byte			0xff

#define 	WRITE_BURST     		0x40		/* Á¬ÐøÐ´Èë							*/
#define 	READ_SINGLE     		0x80		/* ¶Á 									*/
#define 	READ_BURST      		0xC0		/* Á¬Ðø¶Á							*/
#define 	BYTES_IN_RXFIFO     	0x7F  		/* ½ÓÊÕ»º³åÇøµÄÓÐÐ§×Ö½ÚÊý	*/
#define 	CRC_OK              		0x80 		/* CRCÐ£ÑéÍ¨¹ýÎ»±êÖ¾			*/

// CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CCxxx0_IOCFG2       	0x00        	/* GDO2 output pin configuration 					*/
#define CCxxx0_IOCFG1       	0x01        	/* GDO1 output pin configuration 					*/
#define CCxxx0_IOCFG0       	0x02        	/* GDO0 output pin configuration 					*/
#define CCxxx0_FIFOTHR     	  0x03       	/* RX FIFO and TX FIFO thresholds				*/
#define CCxxx0_SYNC1        	0x04        	/* Sync word, high uint8_t						*/
#define CCxxx0_SYNC0        	0x05        	/* Sync word, low uint8_t						*/
#define CCxxx0_PKTLEN       	0x06        	/* Packet length								*/
#define CCxxx0_PKTCTRL1     	0x07       	/* Packet automation control						*/
#define CCxxx0_PKTCTRL0    	  0x08       	/* Packet automation control						*/
#define CCxxx0_ADDR         	0x09        	/* Device address								*/
#define CCxxx0_CHANNR       	0x0A        	/* Channel number							*/	
#define CCxxx0_FSCTRL1      	0x0B        	/* Frequency synthesizer control					*/	
#define CCxxx0_FSCTRL0      	0x0C        	/* Frequency synthesizer control					*/
#define CCxxx0_FREQ2        	0x0D        	/* Frequency control word, high uint8_t				*/
#define CCxxx0_FREQ1        	0x0E        	/* Frequency control word, middle uint8_t			*/	
#define CCxxx0_FREQ0        	0x0F        	/* Frequency control word, low uint8_t				*/
#define CCxxx0_MDMCFG4      	0x10          	/* Modem configuration							*/	
#define CCxxx0_MDMCFG3      	0x11        	/* Modem configuration							*/
#define CCxxx0_MDMCFG2      	0x12        	/* Modem configuration							*/
#define CCxxx0_MDMCFG1      	0x13        	/* Modem configuration							*/
#define CCxxx0_MDMCFG0      	0x14        	/* Modem configuration							*/
#define CCxxx0_DEVIATN      	0x15        	/* Modem deviation setting						*/
#define CCxxx0_MCSM2        	0x16        	/* Main Radio Control State Machine configuration	*/
#define CCxxx0_MCSM1        	0x17        	/* Main Radio Control State Machine configuration	*/
#define CCxxx0_MCSM0        	0x18        	/* Main Radio Control State Machine configuration	*/
#define CCxxx0_FOCCFG       	0x19        	/* Frequency Offset Compensation configuration		*/
#define CCxxx0_BSCFG        	0x1A        	/* Bit Synchronization configuration				*/
#define CCxxx0_AGCCTRL2     	0x1B        	/* AGC control								*/
#define CCxxx0_AGCCTRL1     	0x1C        	/* AGC control								*/
#define CCxxx0_AGCCTRL0     	0x1D        	/* AGC control								*/
#define CCxxx0_WOREVT1      	0x1E        	/* High uint8_t Event 0 timeout					*/
#define CCxxx0_WOREVT0      	0x1F        	/* Low uint8_t Event 0 timeout					*/
#define CCxxx0_WORCTRL      	0x20        	/* Wake On Radio control						*/
#define CCxxx0_FREND1       	0x21        	/* Front end RX configuration					*/
#define CCxxx0_FREND0       	0x22       	/* Front end TX configuration					*/
#define CCxxx0_FSCAL3       	0x23        	/* Frequency synthesizer calibration				*/
#define CCxxx0_FSCAL2       	0x24        	/* Frequency synthesizer calibration				*/
#define CCxxx0_FSCAL1       	0x25        	/* Frequency synthesizer calibration				*/
#define CCxxx0_FSCAL0       	0x26        	/* Frequency synthesizer calibration				*/
#define CCxxx0_RCCTRL1      	0x27        	/* RC oscillator configuration					*/
#define CCxxx0_RCCTRL0      	0x28        	/* RC oscillator configuration					*/
#define CCxxx0_FSTEST       	0x29        	/* Frequency synthesizer calibration control		*/
#define CCxxx0_PTEST        	0x2A        	/* Production test								*/
#define CCxxx0_AGCTEST      	0x2B        	/* AGC test									*/
#define CCxxx0_TEST2        	0x2C        	/* Various test settings							*/
#define CCxxx0_TEST1        	0x2D        	/* Various test settings							*/
#define CCxxx0_TEST0        	0x2E        	/* Various test settings							*/

// Strobe commands
#define CCxxx0_SRES         0x30        	/* Reset chip.										*/
#define CCxxx0_SFSTXON      0x31        	/* 	Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        					If in RX/TX: Go to a wait state where only the synthesizer is
                                        					running (for quick RX / TX turnaround).                                              */
#define CCxxx0_SXOFF        0x32        	/* 	Turn off crystal oscillator.                                                */
#define CCxxx0_SCAL         0x33        	/* 	Calibrate frequency synthesizer and turn it off			*/
                                        				/* 	(enables quick start).								*/
#define CCxxx0_SRX          0x34        	/* 	Enable RX. Perform calibration first if coming from IDLE and
                                        					MCSM0.FS_AUTOCAL=1.							*/
#define CCxxx0_STX          0x35        	/* 	In IDLE state: Enable TX. Perform calibration first if
                                        					MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        					Only go to TX if channel is clear.							*/
#define CCxxx0_SIDLE        0x36        	/*	Exit RX / TX, turn off frequency synthesizer and exit
                                        					Wake-On-Radio mode if applicable.					*/
#define CCxxx0_SAFC         0x37        	/*	Perform AFC adjustment of the frequency synthesizer	*/
#define CCxxx0_SWOR         0x38        	/*	Start automatic RX polling sequence (Wake-on-Radio)	*/
#define CCxxx0_SPWD         0x39        	/*	Enter power down mode when CSn goes high.			*/
#define CCxxx0_SFRX         0x3A        	/*	Flush the RX FIFO buffer.							*/
#define CCxxx0_SFTX         0x3B        	/*	Flush the TX FIFO buffer.							*/
#define CCxxx0_SWORRST      0x3C        	/*	Reset real time clock.								*/
#define CCxxx0_SNOP         0x3D        	/*	No operation. May be used to pad strobe commands to two
                                        					uint8_ts for simpler software.							*/

#define CCxxx0_PARTNUM      0x30
#define CCxxx0_VERSION      0x31
#define CCxxx0_FREQEST      0x32
#define CCxxx0_LQI          0x33
#define CCxxx0_RSSI         0x34
#define CCxxx0_MARCSTATE    0x35
#define CCxxx0_WORTIME1     0x36
#define CCxxx0_WORTIME0     0x37
#define CCxxx0_PKTSTATUS    0x38
#define CCxxx0_VCO_VC_DAC   0x39
#define CCxxx0_TXBYTES      0x3A
#define CCxxx0_RXBYTES      0x3B

#define CCxxx0_PATABLE      0x3E
#define CCxxx0_TXFIFO       0x3F
#define CCxxx0_RXFIFO       0x3F

uint8_t PaTabel[8] = {0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0};
uint8_t paTableLen = 4;                                    /*	¶ÔÓ¦¹¦ÂÊÖµ£¬Ä¬ÈÏ4ÔòÎª0dbm·¢ËÍ*/

uint8_t ID =0x31;
uint8_t Received_id;
uint8_t RxFifoData;

uint8_t flag[2];		
uint8_t txBuffer[20]={15,1,3,4//,5,6,7,8,9,4,5,5,6,2,1,3,4,5,6,7
                  //,3,4,5,6,7,8,9,4,5,5,6,2,1,3,4,5,6,7,8,9
                  ,7,8,9,4,5,5,6,2,1,3,4,5,6,7,8,6,};

//RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS
{
	uint8_t FSCTRL2;			/*	Frequency synthesizer control 2.				*/
	uint8_t FSCTRL1;   			/*	Frequency synthesizer control 1.				*/
	uint8_t FSCTRL0;   			/*	Frequency synthesizer control.				*/
	uint8_t FREQ2;     			/*	Frequency control word, high uint8_t.			*/
	uint8_t FREQ1;     			/*	Frequency control word, middle uint8_t.			*/
	uint8_t FREQ0;     			/*	Frequency control word, low uint8_t.			*/
	uint8_t MDMCFG4;   		/*	Modem configuration.						*/
	uint8_t MDMCFG3;   		/*	Modem configuration.						*/
	uint8_t MDMCFG2;   		/*	Modem configuration.						*/
	uint8_t MDMCFG1;   		/*	Modem configuration.						*/
	uint8_t MDMCFG0;   		/*	Modem configuration.						*/
	uint8_t CHANNR;    		/*	Channel number.							*/
	uint8_t DEVIATN;   		/*	Modem deviation setting (when FSK modulation is enabled).	*/
	uint8_t FREND1;    			/*	Front end RX configuration.					*/
	uint8_t FREND0;    			/*	Front end RX configuration.					*/
	uint8_t MCSM0;     			/*	Main Radio Control State Machine configuration.	*/
	uint8_t FOCCFG;    			/*	Frequency Offset Compensation Configuration.	*/
	uint8_t BSCFG;     			/*	Bit synchronization Configuration.				*/
	uint8_t AGCCTRL2;  		/*	AGC control.								*/
	uint8_t AGCCTRL1;  		/*	AGC control.								*/
	uint8_t AGCCTRL0;  		/*	AGC control.								*/
	uint8_t FSCAL3;    			/*	Frequency synthesizer calibration.				*/
	uint8_t FSCAL2;    			/*	Frequency synthesizer calibration.				*/
	uint8_t FSCAL1;    			/*	Frequency synthesizer calibration.				*/
	uint8_t FSCAL0;    			/*	Frequency synthesizer calibration.				*/
	uint8_t FSTEST;    			/*	Frequency synthesizer calibration control		*/
	uint8_t TEST2;     			/*	Various test settings.						*/
	uint8_t TEST1;     			/*	Various test settings.						*/
	uint8_t TEST0;     			/*	Various test settings.						*/
	uint8_t IOCFG2;    			/*	GDO2 output pin configuration				*/
	uint8_t IOCFG0;    			/*	GDO0 output pin configuration				*/
	uint8_t PKTCTRL1;  		/*	Packet automation control.					*/
	uint8_t PKTCTRL0;  		/*	Packet automation control.					*/
	uint8_t ADDR;      			/*	Device address.							*/
	uint8_t PKTLEN;    			/*	Packet length.								*/
} RF_SETTINGS;

const RF_SETTINGS rfSettings = {
	0x00,
	0x0B, // FSCTRL1 Frequency synthesizer control.
	0x00, // FSCTRL0 Frequency synthesizer control.
	0x21, // FREQ2 Frequency control word, high byte.
	0x62, // FREQ1 Frequency control word, middle byte.
	0x76, // FREQ0 Frequency control word, low byte.
	0x2D, //0x2D, // MDMCFG4 Modem configuration.
	0x3B, //0x3B, // MDMCFG3 Modem configuration.
	0x73, //0x73, // MDMCFG2 Modem configuration.
	0x22, // MDMCFG1 Modem configuration.
	0xF8, // MDMCFG0 Modem configuration.
	
	0x00, // CHANNR Channel number.
	0x62, // DEVIATN Modem deviation setting (when FSK modulation is enabled).
	0xB6, // FREND1 Front end RX configuration.
	0x10, // FREND0 Front end RX configuration.
	0x18, // MCSM0 Main Radio Control State Machine configuration.
	0x1D, // FOCCFG Frequency Offset Compensation Configuration.
	0x1C, // BSCFG Bit synchronization Configuration.
	0xC7, // AGCCTRL2 AGC control.
	0x00, //0x00, // AGCCTRL1 AGC control.
	0xB2, // AGCCTRL0 AGC control.
	
	0xEA, // FSCAL3 Frequency synthesizer calibration.
	0x0A, // FSCAL2 Frequency synthesizer calibration.
	0x00, // FSCAL1 Frequency synthesizer calibration.
	0x11, // FSCAL0 Frequency synthesizer calibration.
	0x59, // FSTEST Frequency synthesizer calibration.
	0x88, // TEST2 Various test settings.
	0x31, // TEST1 Various test settings.
	0x0B, // TEST0 Various test settings.
	0x06, // IOCFG2 GDO2 output pin configuration.
	0x06, // IOCFG0D GDO0 output pin configuration.
	
	0x07, // PKTCTRL1 Packet automation control.  
	0x05, // PKTCTRL0 Packet automation control. // variable length mode
	0x02, // ADDR Device address.
	0xff // PKTLEN Packet length.
};

static unsigned char send_num=0;

/*******************************************************************************
* Function Name  : CC1101_Init()
* Description    : Initializes the peripherals .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CC1101_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE , ENABLE);

	/* Configure SPI2 pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_SEND_Pin_SCLK | GPIO_SEND_Pin_SO | GPIO_SEND_Pin_SI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);


	
	/* Configure I/O for CC1101 SEND Chip select */
	GPIO_InitStructure.GPIO_Pin = GPIO_SEND_Pin_CS ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	
	/* Configure I/O for PC4,PC5,PC6,PC7  */
	GPIO_InitStructure.GPIO_Pin = GPIO_SEND_Pin_GD2| GPIO_SEND_Pin_GD0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
	
	/* Enable SPI1 && SPI2 && GPIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* SPI1 configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_2Lines_RxOnly;//SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
uint8_t SPI_FLASH_ReadByte(CC1101_TYPE type)
{
	if(type == CC1101_TYPE_SEND){
		return (SPI_FLASH_SendByte(Dummy_Byte, CC1101_TYPE_SEND));
	}
	else if(type == CC1101_TYPE_RECEIVE){
		return (SPI_FLASH_SendByte(Dummy_Byte, CC1101_TYPE_RECEIVE));
	}
	else
		return 0;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
uint8_t SPI_FLASH_SendByte(uint8_t byte, CC1101_TYPE type)
{
	if(type == CC1101_TYPE_SEND){
		/* Loop while DR register in not emplty */
//		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

		/* Send byte through the SPI2 peripheral */
		SPI_I2S_SendData(SPI1, byte);

		/* Wait to receive a byte */
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

		/* Return the byte read from the SPI bus */
		return SPI_I2S_ReceiveData(SPI1);
	}
	else if(type == CC1101_TYPE_RECEIVE)
	{
		/* Loop while DR register in not emplty */
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

		/* Send byte through the SPI2 peripheral */
		SPI_I2S_SendData(SPI1, byte);

		/* Wait to receive a byte */
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

		/* Return the byte read from the SPI bus */
		return SPI_I2S_ReceiveData(SPI1);
	}
	else
		return 0;
}

/**********************************CC1101********************/

void Delay_1(uint32_t nCount)
{
  int i,j;
  for(j=0;j<nCount;j++)
  {
     for(i=0;i<10;i++);
  }
}


uint8_t SPI_CC1101_ReadID(void)
{
	 uint8_t id;
	 SPI_SEND_CS_LOW();
	 
	 
	 SPI_FLASH_SendByte(0x31, CC1101_TYPE_SEND);
	 id = SPI_FLASH_SendByte(0xff, CC1101_TYPE_SEND);
	 SPI_SEND_CS_HIGH();

	 return id;
}

void CC1101_POWER_RESET(void)
{
	/* Deselect the FLASH: Chip Select high */
	SPI_SEND_CS_HIGH();     
	GPIO_SetBits(GPIOA, GPIO_SEND_Pin_SCLK); //SCLK=1
	GPIO_ResetBits(GPIOA, GPIO_SEND_Pin_SI); //SI=0
	
	Delay_1(5000);
	SPI_SEND_CS_HIGH();
	
	Delay_1(1);
	SPI_SEND_CS_LOW();
	
	Delay_1(1);
	SPI_SEND_CS_HIGH();
	
	Delay_1(41);
	SPI_SEND_CS_LOW();
	
	while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO));//waite SO =0
	SPI_FLASH_SendByte(CCxxx0_SRES, CC1101_TYPE_SEND);

	while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO));//waite SO =0 again 
	SPI_SEND_CS_HIGH(); 
	
}

//*****************************************************************************************
//º¯ÊýÃû£ºvoid halSpiWriteReg(uint8_t addr, uint8_t value, CC1101_TYPE type) 
//ÊäÈë£ºµØÖ·ºÍÅäÖÃ×Ö
//Êä³ö£ºÎÞ
//¹¦ÄÜÃèÊö£ºSPIÐ´¼Ä´æÆ÷
//*****************************************************************************************
void halSpiWriteReg(uint8_t addr, uint8_t value, CC1101_TYPE type) 
{
	if(type == CC1101_TYPE_SEND){
		SPI_SEND_CS_LOW();
		while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );
		SPI_FLASH_SendByte(addr, CC1101_TYPE_SEND);		// Send address to CC1101 through SPI·
		SPI_FLASH_SendByte(value, CC1101_TYPE_SEND);	// Send Value to CC1101 through SPI·
		SPI_SEND_CS_HIGH(); 
	}
	else
	if(type == CC1101_TYPE_RECEIVE){
		SPI_SEND_CS_LOW();
		while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );
		SPI_FLASH_SendByte(addr, CC1101_TYPE_SEND);		// Send address to CC1101 through SPI·
		SPI_FLASH_SendByte(value, CC1101_TYPE_SEND);	// Send Value to CC1101 through SPI·
		SPI_SEND_CS_HIGH();
	}
}

//*****************************************************************************************


//*****************************************************************************************
void halSpiWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count, CC1101_TYPE type) 
{
	if(type == CC1101_TYPE_SEND){
		uint8_t i, temp;
		temp = addr | WRITE_BURST;
		SPI_SEND_CS_LOW();
		while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );
		SPI_FLASH_SendByte(temp, CC1101_TYPE_SEND);
		
		for (i = 0; i < count; i++)
		{
			SPI_FLASH_SendByte(buffer[i], CC1101_TYPE_SEND);
		}
		
		SPI_SEND_CS_HIGH(); 
	}
	else
	if(type == CC1101_TYPE_RECEIVE){
		uint8_t i, temp;
		temp = addr | WRITE_BURST;
		SPI_SEND_CS_LOW();
		while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );
		SPI_FLASH_SendByte(temp, CC1101_TYPE_SEND);
		
		for (i = 0; i < count; i++)
		{
			SPI_FLASH_SendByte(buffer[i], CC1101_TYPE_SEND);
		}
		
		SPI_SEND_CS_HIGH();
	}
}

//*****************************************************************************************

//*****************************************************************************************
void halSpiStrobe(uint8_t strobe, CC1101_TYPE type) 
{
	if(type == CC1101_TYPE_SEND){
		SPI_SEND_CS_LOW();
		while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );
		SPI_FLASH_SendByte(strobe, CC1101_TYPE_SEND);		//Ð´ÈëÃüÁî
		SPI_SEND_CS_HIGH();
	}
	else
	if(type == CC1101_TYPE_RECEIVE){
		SPI_SEND_CS_LOW();
		while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );
		SPI_FLASH_SendByte(strobe, CC1101_TYPE_SEND);		//Ð´ÈëÃüÁî
		SPI_SEND_CS_HIGH();;
	}
}

//*****************************************************************************************
//º¯ÊýÃû£ºvoid halRfSendPacket(uint8_t *txBuffer, uint8_t size)
//ÊäÈë£º·¢ËÍµÄ»º³åÇø£¬·¢ËÍÊý¾Ý¸öÊý
//Êä³ö£ºÎÞ
//¹¦ÄÜÃèÊö£ºCC1100·¢ËÍÒ»×éÊý¾Ý
//*****************************************************************************************

void halRfSendPacket(uint8_t *txBuffer, uint8_t size) 
{
	static uint32_t i;
	i++;
	halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size, CC1101_TYPE_SEND);	// Write the value of Data to TXFIFO
	halSpiStrobe(CCxxx0_STX, CC1101_TYPE_SEND);		//  Start Transmission

	// Wait for GDO0 to be set -> sync transmitted
	while (!GPIO_ReadInputDataBit(GPIOE,GPIO_SEND_Pin_GD0) );/* while (!GDO0);*/

	// Wait for GDO0 to be cleared -> end of packet
	while (GPIO_ReadInputDataBit(GPIOE,GPIO_SEND_Pin_GD0) ); /* while (GDO0);*/

	halSpiStrobe(CCxxx0_SFTX, CC1101_TYPE_SEND);   /// Flush The FIFO Register
}

//*****************************************************************************************
//º¯ÊýÃû£ºvoid halRfSendData(uint8_t txData)
//ÊäÈë£º·¢ËÍµÄÊý¾Ý
//Êä³ö£ºÎÞ
//¹¦ÄÜÃèÊö£ºCC1100·¢ËÍÒ»¸öÊý¾Ý
//*****************************************************************************************

void halRfSendData(uint8_t txData)
{
    halSpiWriteReg(CCxxx0_TXFIFO, txData, CC1101_TYPE_SEND);	//Ð´ÈëÒª·¢ËÍµÄÊý¾Ý

    halSpiStrobe(CCxxx0_STX, CC1101_TYPE_SEND);		//½øÈë·¢ËÍÄ£Ê½·¢ËÍÊý¾Ý	

    // Wait for GDO0 to be set -> sync transmitted
    while (!GPIO_ReadInputDataBit(GPIOE,GPIO_SEND_Pin_GD0) );//while (!GDO0);
    // Wait for GDO0 to be cleared -> end of packet
    while (GPIO_ReadInputDataBit(GPIOE,GPIO_SEND_Pin_GD0) );// while (GDO0);
	halSpiStrobe(CCxxx0_SFTX, CC1101_TYPE_SEND);
}

//*****************************************************************************************
//º¯ÊýÃû£ºuint8_t halSpiReadReg(uint8_t addr)
//ÊäÈë£ºµØÖ·
//Êä³ö£º¸Ã¼Ä´æÆ÷µÄÅäÖÃ×Ö
//¹¦ÄÜÃèÊö£ºSPI¶Á¼Ä´æÆ÷
//*****************************************************************************************
uint8_t halSpiReadReg(uint8_t addr) 
{
	uint8_t temp, value;
	temp = addr|READ_SINGLE;//¶Á¼Ä´æÆ÷ÃüÁî
	SPI_SEND_CS_LOW();
	while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );//MISO
	SPI_FLASH_SendByte(temp, CC1101_TYPE_SEND);
	value = SPI_FLASH_SendByte(0, CC1101_TYPE_SEND);
	 SPI_SEND_CS_HIGH();
	return value;
}

//*****************************************************************************************
//º¯ÊýÃû£ºvoid halSpiReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
//ÊäÈë£ºµØÖ·£¬¶Á³öÊý¾ÝºóÔÝ´æµÄ»º³åÇø£¬¶Á³öÅäÖÃ¸öÊý
//Êä³ö£ºÎÞ
//¹¦ÄÜÃèÊö£ºSPIÁ¬ÐøÐ´ÅäÖÃ¼Ä´æÆ÷
//*****************************************************************************************
void halSpiReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count) 
{
	uint8_t i,temp;
	temp = addr | READ_BURST;		//Ð´ÈëÒª¶ÁµÄÅäÖÃ¼Ä´æÆ÷µØÖ·ºÍ¶ÁÃüÁî
	SPI_SEND_CS_LOW();
	while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );//MISO
	SPI_FLASH_SendByte(temp, CC1101_TYPE_SEND);   

	for (i = 0; i < count; i++) 
	{
		buffer[i] = SPI_FLASH_SendByte(0, CC1101_TYPE_SEND);
	}

	SPI_SEND_CS_HIGH();
}


uint8_t halSpiReadStatus(uint8_t addr) 
{
	uint8_t value,temp;
	temp = addr | READ_BURST;		/*/Ð´ÈëÒª¶ÁµÄ×´Ì¬¼Ä´æÆ÷µÄµØÖ·Í¬Ê±Ð´Èë¶ÁÃüÁî*/
	SPI_SEND_CS_LOW();
	while (GPIO_ReadInputDataBit(GPIOA,GPIO_SEND_Pin_SO) );
	SPI_FLASH_SendByte(temp, CC1101_TYPE_SEND);
	value = SPI_FLASH_SendByte(0, CC1101_TYPE_SEND);
	SPI_SEND_CS_HIGH();
	return value;
}


uint8_t halRfReceivePacket(uint8_t *rxBuffer, uint8_t *length) 
{
	uint8_t status[2];
	uint8_t packetLength;
	uint8_t address;
	uint8_t i=(*length)*4;  /*/ ¾ßÌå¶àÉÙÒª¸ù¾ÝdatarateºÍlengthÀ´¾ö¶¨*/

	static uint32_t j=0;
	
	halSpiStrobe(CCxxx0_SRX, CC1101_TYPE_RECEIVE);		/*/½øÈë½ÓÊÕ×´Ì¬*/
  
//	while (!GPIO_ReadInputDataBit(GPIOE,GPIO_SEND_Pin_GD0) );//while (GDO0)
//	while (GPIO_ReadInputDataBit(GPIOE,GPIO_SEND_Pin_GD0));
//	j++;

	RxFifoData =halSpiReadStatus(CCxxx0_RXBYTES);

	if (RxFifoData & BYTES_IN_RXFIFO) 
	{
		flag[1]=1;
		halSpiReadBurstReg(CCxxx0_RXFIFO, rxBuffer,2); 
 		*length = rxBuffer[1];				
		// Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
		halSpiReadBurstReg(CCxxx0_RXFIFO, status, 2); 	
		halSpiStrobe(CCxxx0_SFRX, CC1101_TYPE_RECEIVE);		
		
		return (status[1] & CRC_OK);			
	} 
	else
		flag[1]=0;  ///flag_exti=0
 	
	return 0;
}


//*****************************************************************************************
//º¯ÊýÃû£ºvoid halRfWriteRfSettings(RF_SETTINGS *pRfSettings)
//ÊäÈë£ºÎÞ
//Êä³ö£ºÎÞ
//¹¦ÄÜÃèÊö£ºÅäÖÃCC1100µÄ¼Ä´æÆ÷
//*****************************************************************************************
void halRfWriteRfSettings(CC1101_TYPE type) 
{

	halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL2, type);
	// Write register settings
	halSpiWriteReg(CCxxx0_FSCTRL1,  rfSettings.FSCTRL1, type);
	halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL0, type);
	halSpiWriteReg(CCxxx0_FREQ2,    rfSettings.FREQ2, type);
	halSpiWriteReg(CCxxx0_FREQ1,    rfSettings.FREQ1, type);
	halSpiWriteReg(CCxxx0_FREQ0,    rfSettings.FREQ0, type);
	halSpiWriteReg(CCxxx0_MDMCFG4,  rfSettings.MDMCFG4, type);
	halSpiWriteReg(CCxxx0_MDMCFG3,  rfSettings.MDMCFG3, type);
	halSpiWriteReg(CCxxx0_MDMCFG2,  rfSettings.MDMCFG2, type);
	halSpiWriteReg(CCxxx0_MDMCFG1,  rfSettings.MDMCFG1, type);
	halSpiWriteReg(CCxxx0_MDMCFG0,  rfSettings.MDMCFG0, type);
	halSpiWriteReg(CCxxx0_CHANNR,   rfSettings.CHANNR, type);
	halSpiWriteReg(CCxxx0_DEVIATN,  rfSettings.DEVIATN, type);
	halSpiWriteReg(CCxxx0_FREND1,   rfSettings.FREND1, type);
	halSpiWriteReg(CCxxx0_FREND0,   rfSettings.FREND0, type);
	halSpiWriteReg(CCxxx0_MCSM0,   rfSettings.MCSM0, type);
	halSpiWriteReg(CCxxx0_FOCCFG,   rfSettings.FOCCFG, type);
	halSpiWriteReg(CCxxx0_BSCFG,    rfSettings.BSCFG, type);
	halSpiWriteReg(CCxxx0_AGCCTRL2, rfSettings.AGCCTRL2, type);
	halSpiWriteReg(CCxxx0_AGCCTRL1, rfSettings.AGCCTRL1, type);	 //
	halSpiWriteReg(CCxxx0_AGCCTRL0, rfSettings.AGCCTRL0, type);
	halSpiWriteReg(CCxxx0_FSCAL3,   rfSettings.FSCAL3, type);
	halSpiWriteReg(CCxxx0_FSCAL2,   rfSettings.FSCAL2, type);
	halSpiWriteReg(CCxxx0_FSCAL1,   rfSettings.FSCAL1, type);	//
	halSpiWriteReg(CCxxx0_FSCAL0,   rfSettings.FSCAL0, type);
	halSpiWriteReg(CCxxx0_FSTEST,   rfSettings.FSTEST, type);
	halSpiWriteReg(CCxxx0_TEST2,    rfSettings.TEST2, type);
	halSpiWriteReg(CCxxx0_TEST1,    rfSettings.TEST1, type);
	halSpiWriteReg(CCxxx0_TEST0,    rfSettings.TEST0, type);
	halSpiWriteReg(CCxxx0_IOCFG2,   rfSettings.IOCFG2, type);
	if(type==CC1101_TYPE_RECEIVE)
	halSpiWriteReg(CCxxx0_IOCFG0,  0x06, type);  
	else
	halSpiWriteReg(CCxxx0_IOCFG0,   rfSettings.IOCFG0, type);    
	
	halSpiWriteReg(CCxxx0_PKTCTRL1, rfSettings.PKTCTRL1, type);
	halSpiWriteReg(CCxxx0_PKTCTRL0, rfSettings.PKTCTRL0, type);
	if(type==CC1101_TYPE_RECEIVE)
	halSpiWriteReg(CCxxx0_ADDR, rfSettings.ADDR , type);
	else
	halSpiWriteReg(CCxxx0_ADDR,     rfSettings.ADDR, type);
	halSpiWriteReg(CCxxx0_PKTLEN,   rfSettings.PKTLEN, type);
}

uint8_t length=3;
uint8_t txBuf[4] = {0x03, 0x01,0x00,0x00};
uint8_t rxBuf[9] = {0, 0, 0, 0, 0, 0, 0, 0};

void CC1101_Send_Main(uint16_t Angle31)
{
  uint8_t num=0;	
	uint8_t tf =0;	

	if (Angle31 > 255)
	{
		txBuf[2] = 255;
txBuf[3] = Angle31-255;
	}
else
{
txBuf[2] = Angle31;
txBuf[3] = 0;
}

	
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8, CC1101_TYPE_SEND);

		halSpiStrobe(CCxxx0_SIDLE, CC1101_TYPE_SEND);            	
		halRfSendPacket(txBuf,6);
	//	halSpiStrobe(CCxxx0_SRX, CC1101_TYPE_SEND);
}

void CC1101_Receive_Main(void)
{
	uint8_t i;
	
	
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8, CC1101_TYPE_RECEIVE);
	halSpiStrobe(CCxxx0_SRX, CC1101_TYPE_RECEIVE);
//	halSpiStrobe(CCxxx0_SFRX, CC1101_TYPE_RECEIVE);
	//	halRfReceivePacket(rxBuf, &length);
		
}

void Recieve()
{
	//halSpiStrobe(CCxxx0_SFRX, CC1101_TYPE_RECEIVE);
  halRfReceivePacket(rxBuf, &length);
}

uint8_t getFlagID()
{
	return flag[0];
}

uint8_t getExtiFlag()
{
	return flag[1];
}