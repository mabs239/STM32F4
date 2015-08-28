/* ADC working for Main Board Accoustic Data over UART2*/
#include <stm32f4xx.h>
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include "time_nwn.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "three_adcs_nwn.h"
#include "usbd_cdc_vcp.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

#define MAX_STRLEN 12 // this is the maximum string length of our string in characters
//volatile char received_string[MAX_STRLEN+1] = ""; // this will hold the recieved string

__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_dev  __ALIGN_END ;

volatile unsigned int ConvertedValue = 0;
volatile int data = 0,v1=0,v2=0,v3=0,d1=0x40,d2=0x42;
volatile unsigned char sample_data;
unsigned char v1_8,v2_8,temp;



int adc_convert(){
 ADC_SoftwareStartConv(ADC1);//Start the conversion
 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
 return ADC_GetConversionValue(ADC1); //Return the converted data
}


void USART_puts(USART_TypeDef* USARTx, volatile int s);

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/* This funcion initializes the  peripheral
 * 
 * Arguments: baudrate --> the baudrate at which the USART is 
 * 						   supposed to operate
 */
void init_USART2(uint32_t baudrate){
	
	
	GPIO_InitTypeDef GPIO_InitStruct;               // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct;             // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure;            // this is used to configure the NVIC (nested vector interrupt controller)
	
	/* enable APB2 peripheral clock for USART1 
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART1, PC6 for TX and PC7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;     // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                  // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART2, ENABLE);
}


void TIM2_IRQHandler(void)                                              //Timer Interupt for sending data
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
	//GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    //count++;
       v1=aADCDualConvertedValue[0];
        v2=aADCDualConvertedValue[1];
       // v3=aADCDualConvertedValue[2];
         v1_8= (unsigned char)(v1>>4);
         
         v2_8= (unsigned char)(v2>>4);
        	    /* if(v1_8&0x01)
        		  	v1_8=v1_8 & 0xFE;
                          if(!(v2_8&0x01))
        		      	v2_8=v2_8 | 0x01;*/

    //sample_data = (unsigned char)(ConvertedValue >>4);

    USART_puts(USART2, v1_8);
   USART_puts(USART2, v2_8);
// GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
  
  }
}



void USART_puts(USART_TypeDef* USARTx, volatile int s){

	//while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx,s);
	//	*s++;
	//}
}


int main(void) {
  
  int i;
  data = 65;
  adc_config();
  init_USART2(921600); 						// initialize USART1 @ 9600 baud
  INTTIM3_Config();
  //adc_configure();
  
  
  while(1)
  {
 int k=1;
	//  ConvertedValue = adc_convert();                      //Conversion :sent on Timer Interupt only
    //USART_puts(USART2,ConvertedValue);
        
    
  }
}

