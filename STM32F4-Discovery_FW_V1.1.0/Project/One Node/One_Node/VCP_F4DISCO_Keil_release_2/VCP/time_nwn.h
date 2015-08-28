/**
  ******************************************************************************
  * @file    USART/main.c 
  * @author  Akela M.Shahid 
  * @version V1.0.0
  * @date    27-April-2012
  * @brief   Main program body
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include <stdio.h>


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure1;

void INTTIM2_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 10000-1;  // 1 MHz down to 1 KHz (1 ms)
  TIM_TimeBaseStructure.TIM_Prescaler = 33600 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  /* TIM2 enable counter */
  TIM_Cmd(TIM2,ENABLE);
}



void INTTIM3_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure1.TIM_Period = 22-1;  // 1 MHz down to 1 KHz (1 ms)
  TIM_TimeBaseStructure1.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
  TIM_TimeBaseStructure1.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure1.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure1);
  /* TIM IT enable */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void INTTIM4_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure1.TIM_Period = 10000-1;  // 1 MHz down to 1 KHz (1 ms)
  TIM_TimeBaseStructure1.TIM_Prescaler = 33600 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
  TIM_TimeBaseStructure1.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure1.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure1);
  /* TIM IT enable */
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  /* TIM3 enable counter */
   TIM_Cmd(TIM4, ENABLE);
}


void INTTIM5_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure1.TIM_Period = 10000-1;  // 1 MHz down to 1 KHz (1 ms)
  TIM_TimeBaseStructure1.TIM_Prescaler = 8400 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
  TIM_TimeBaseStructure1.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure1.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure1);
  /* TIM IT enable */
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  /* TIM3 enable counter */
   TIM_Cmd(TIM5, ENABLE);
}

