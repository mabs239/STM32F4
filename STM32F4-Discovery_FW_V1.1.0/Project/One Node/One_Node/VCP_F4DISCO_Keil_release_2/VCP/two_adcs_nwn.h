/*------ADC DUAL Channel USING DMA2----*/ 


/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4_discovery.h"


#define ADC_CDR_ADDRESS    ((uint32_t)0x40012308)

/** @addtogroup STM32F4xx_StdPeriph_Examples

  * @{

  */



/** @addtogroup ADC_DualModeRegulSimu

  * @{

  */ 



/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint16_t aADCDualConvertedValue[2];



/* Private function prototypes -----------------------------------------------*/

static void DMA_Config(void);

static void GPIO_Config(void);

static void ADC1_CH11_Config(void);

static void ADC2_CH9_Config(void);



/* Private functions ---------------------------------------------------------*/



/**

  * @brief   Main program

  * @param  None

  * @retval None

  */
#define DMA2_IRQHANDLER   DMA2_Stream0_IRQHandler
//void DMA2_IRQHANDLER(void)
//{
//    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
//    {
//        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);      // Reset du Flag
 
//        ADC1->SR = ADC1->SR & 0xFFED;
 
        /* Change l'etat de la LED4 */
//        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
//    }
//}


void adc_config(void)

{

  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  NVIC_InitTypeDef NVIC_InitStructure;  

  /*!< At this stage the microcontroller clock setting is already configured, 

       this is done through SystemInit() function which is called from startup

       files (startup_stm32f40xx.s/startup_stm32f427x.s) before to branch to 

       application main. 

       To reconfigure the default setting of SystemInit() function, refer to

       system_stm32f4xx.c file

     */


  /* Enable peripheral clocks *************************************************/

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA , ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    

  /* DMA2 Stream0 channe0 and channel configuration **************************************/

  DMA_Config();

  

  /* ADCs configuration ------------------------------------------------------*/

  /* Configure ADC Channel10, 11, 12 pin as analog input */

  GPIO_Config();



  /* ADC Common Init */

  ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;

  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;

  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;

  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

  ADC_CommonInit(&ADC_CommonInitStructure);



  /* ADC1 regular channels 10, 11 configuration */

  ADC1_CH11_Config();



  /* ADC2 regular channels 11, 12 configuration */

  ADC2_CH9_Config();
  
/* Enable the DMA2 Stream0 Interrupt */
/*
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
*/


  /* Enable DMA request after last transfer (Multi-ADC mode)  */

  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
//  ADC_EOCOnEachRegularChannelCmd(ADC1,ENABLE);
 

  /* Enable ADC1 */

       ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC2 */

       ADC_Cmd(ADC2, ENABLE);

      //  ADC_ITConfig(ADC1,ADC_IT_EOC, ENABLE);
 
      /* Enable DMA2 Stream0 Transfer complete interrupt */
      //DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
 
      /* Enable ADC1 DMA since ADC1 is the Master*/
      ADC_DMACmd(ADC1, ENABLE);

  /* Start ADC1 Software Conversion */

  ADC_SoftwareStartConv(ADC1);

}



/**

  * @brief  ADC1 regular channels 10 and 11 configuration

  * @param  None

  * @retval None

  */

static void ADC1_CH11_Config(void)

{

  ADC_InitTypeDef ADC_InitStructure;



  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;

  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//ENABLE;

  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;

  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

  ADC_InitStructure.ADC_NbrOfConversion = 1;

  ADC_Init(ADC1, &ADC_InitStructure);



  /* ADC1 regular channels 10, 11 configuration */ 

 // ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_144Cycles);

}



/**

  * @brief  ADC2 regular channels 11, 12 configuration

  * @param  None

  * @retval None

  */

static void ADC2_CH9_Config(void)

{

  ADC_InitTypeDef ADC_InitStructure;



  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;

  ADC_InitStructure.ADC_ScanConvMode = DISABLE;                      //ENABLE;

  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;

  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

  ADC_InitStructure.ADC_NbrOfConversion = 1;

  ADC_Init(ADC2, &ADC_InitStructure);



  /* ADC2 regular channels 11, 12 configuration */ 

 // ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);

  ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_144Cycles);

}



/**

  * @brief  DMA Configuration

  * @param  None

  * @retval None

  */

static void DMA_Config(void)

{

  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&aADCDualConvertedValue;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CDR_ADDRESS;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  DMA_Init(DMA2_Stream0, &DMA_InitStructure);



  /* DMA2_Stream0 enable */

  DMA_Cmd(DMA2_Stream0, ENABLE);

}



/**

  * @brief Configure ADC Channels 10, 11, 12 pins as analog inputs

  * @param  None

  * @retval None

  */

static void GPIO_Config(void)

{

  GPIO_InitTypeDef GPIO_InitStructure,gpio2,gpioB;

  /* ADC Channel 10 -> PC0

     ADC Channel 11 -> PC1

     ADC Channel 12 -> PC2

  */

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /*******************************************************/
  
  gpioB.GPIO_Pin = GPIO_Pin_1;

  gpioB.GPIO_Mode = GPIO_Mode_AN;

  gpioB.GPIO_PuPd = GPIO_PuPd_NOPULL ;

  GPIO_Init(GPIOB, &gpioB);
  
//GPIO configuration to test timing constraints of ADC's
  gpio2.GPIO_Pin=GPIO_Pin_13 | GPIO_Pin_14;
  gpio2.GPIO_Mode= GPIO_Mode_OUT;
  gpio2.GPIO_OType = GPIO_OType_PP;
  gpio2.GPIO_Speed= GPIO_Speed_100MHz;
  gpio2.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD ,&gpio2);

}



