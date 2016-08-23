/*
 * vectorsf407.h
 *
 *  Created on: 23 рту. 2016 у.
 *      Author: tihonov
 */

#ifndef INTERRUPT_VECTORSF407_H_
#define INTERRUPT_VECTORSF407_H_



//******************************************************************************************
//Declaration of interrupt vectors
//******************************************************************************************
extern "C" void WWDG_IRQHandler(void);
extern "C" void PVD_IRQHandler(void);
extern "C" void TAMP_STAMP_IRQHandler(void);
extern "C" void RTC_WKUP_IRQHandler(void) ;
extern "C" void FLASH_IRQHandler(void) ;
extern "C" void RCC_IRQHandler(void) ;
extern "C" void EXTI0_IRQHandler(void) ;
extern "C" void EXTI1_IRQHandler(void) ;
extern "C" void EXTI2_IRQHandler(void) ;
extern "C" void EXTI3_IRQHandler(void) ;
extern "C" void EXTI4_IRQHandler(void) ;
extern "C" void DMA1_Stream0_IRQHandler(void);
extern "C" void DMA1_Stream1_IRQHandler(void);
extern "C" void DMA1_Stream2_IRQHandler(void);
extern "C" void DMA1_Stream3_IRQHandler(void);
extern "C" void DMA1_Stream4_IRQHandler(void);
extern "C" void DMA1_Stream5_IRQHandler(void);
extern "C" void DMA1_Stream6_IRQHandler(void);
extern "C" void ADC_IRQHandler(void);
extern "C" void EXTI9_5_IRQHandler(void);
extern "C" void TIM1_BRK_TIM9_IRQHandler(void);
extern "C" void TIM1_UP_TIM10_IRQHandler(void);
extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(void);
extern "C" void TIM1_CC_IRQHandler(void);
extern "C" void TIM2_IRQHandler(void);
extern "C" void TIM3_IRQHandler(void);
extern "C" void TIM4_IRQHandler(void);
extern "C" void I2C1_EV_IRQHandler(void);
extern "C" void I2C1_ER_IRQHandler(void);
extern "C" void I2C2_EV_IRQHandler(void);
extern "C" void I2C2_ER_IRQHandler(void);
extern "C" void SPI1_IRQHandler(void);
extern "C" void SPI2_IRQHandler(void);
extern "C" void USART1_IRQHandler(void);
extern "C" void USART2_IRQHandler(void);
extern "C" void EXTI15_10_IRQHandler(void);
extern "C" void RTC_Alarm_IRQHandler(void);
extern "C" void OTG_FS_WKUP_IRQHandler(void);
extern "C" void DMA1_Stream7_IRQHandler(void);
extern "C" void FSMC_IRQHandler(void);
extern "C" void SDIO_IRQHandler(void);
extern "C" void TIM5_IRQHandler(void);
extern "C" void SPI3_IRQHandler(void);
extern "C" void DMA2_Stream0_IRQHandler(void);
extern "C" void DMA2_Stream1_IRQHandler(void);
extern "C" void DMA2_Stream2_IRQHandler(void);
extern "C" void DMA2_Stream3_IRQHandler(void);
extern "C" void DMA2_Stream4_IRQHandler(void);
extern "C" void CAN1_TX_IRQHandler(void);
extern "C" void CAN1_RX0_IRQHandler(void);
extern "C" void CAN1_RX1_IRQHandler(void);
extern "C" void CAN1_SCE_IRQHandler(void);
extern "C" void CAN2_TX_IRQHandler(void);
extern "C" void CAN2_RX0_IRQHandler(void);
extern "C" void CAN2_RX1_IRQHandler(void);
extern "C" void CAN2_SCE_IRQHandler(void);
extern "C" void USART3_IRQHandler(void);
extern "C" void TIM8_BRK_TIM12_IRQHandler(void);
extern "C" void TIM8_UP_TIM13_IRQHandler(void);
extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void);
extern "C" void TIM8_CC_IRQHandler(void);
extern "C" void UART4_IRQHandler(void);
extern "C" void UART5_IRQHandler(void);
extern "C" void TIM6_DAC_IRQHandler(void);
extern "C" void TIM7_IRQHandler(void);
extern "C" void OTG_HS_EP1_OUT_IRQHandler(void);
extern "C" void OTG_HS_EP1_IN_IRQHandler(void);
extern "C" void OTG_HS_WKUP_IRQHandler(void) ;
extern "C" void OTG_HS_IRQHandler(void);
extern "C" void HASH_RNG_IRQHandler(void);
extern "C" void ETH_IRQHandler(void);
extern "C" void ETH_WKUP_IRQHandler(void);
extern "C" void DCMI_IRQHandler(void);
extern "C" void OTG_FS_IRQHandler(void);
extern "C" void DMA2_Stream5_IRQHandler(void);
extern "C" void DMA2_Stream6_IRQHandler(void);
extern "C" void DMA2_Stream7_IRQHandler(void);
extern "C" void USART6_IRQHandler(void);
extern "C" void I2C3_EV_IRQHandler(void);
extern "C" void I2C3_ER_IRQHandler(void);
extern "C" void FPU_IRQHandler(void);



#endif /* INTERRUPT_VECTORSF407_H_ */
