/*
 * interruptmanager.cpp
 *
 *  Created on: 23 но€б. 2015 г.
 *      Author: tihonov
 */
#include "interruptmanager.h"
#include "includes.hpp"

constexpr pHandlerPointer_t GenerateArray(void) {
	return InterruptManager::DefaultHandler;
}

// ToDo: заполнение массива compile time

pHandlerPointer_t volatile InterruptManager::IsrVectors[81] { 0 };

extern "C" void WWDG_IRQHandler(void) {
	InterruptManager::Raise(WWDG_IRQn);
}
;
extern "C" void PVD_IRQHandler(void) {
	InterruptManager::Raise(PVD_IRQn);
}
;
extern "C" void TAMP_STAMP_IRQHandler(void) {
	InterruptManager::Raise(TAMP_STAMP_IRQn);
}
;
extern "C" void RTC_WKUP_IRQHandler(void) {
	InterruptManager::Raise(RTC_WKUP_IRQn);
}
;
extern "C" void FLASH_IRQHandler(void) {
	InterruptManager::Raise(FLASH_IRQn);
}
;
extern "C" void RCC_IRQHandler(void) {
	InterruptManager::Raise(RCC_IRQn);
}
;
extern "C" void EXTI0_IRQHandler(void) {
	InterruptManager::Raise(EXTI0_IRQn);
}
;
extern "C" void EXTI1_IRQHandler(void) {
	InterruptManager::Raise(EXTI1_IRQn);
}
;
extern "C" void EXTI2_IRQHandler(void) {
	InterruptManager::Raise(EXTI2_IRQn);
}
;
extern "C" void EXTI3_IRQHandler(void) {
	InterruptManager::Raise(EXTI3_IRQn);
}
;
extern "C" void EXTI4_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}
extern "C" void DMA1_Stream0_IRQHandler(void) {
	InterruptManager::Raise(DMA1_Stream0_IRQn);
}
;
extern "C" void DMA1_Stream1_IRQHandler(void) {
	InterruptManager::Raise(DMA1_Stream1_IRQn);
}
;
extern "C" void DMA1_Stream2_IRQHandler(void) {
	InterruptManager::Raise(DMA1_Stream2_IRQn);
}
;
extern "C" void DMA1_Stream3_IRQHandler(void) {
	InterruptManager::Raise(DMA1_Stream3_IRQn);
}
;
extern "C" void DMA1_Stream4_IRQHandler(void) {
	InterruptManager::Raise(DMA1_Stream4_IRQn);
}
;
extern "C" void DMA1_Stream5_IRQHandler(void) {
	InterruptManager::Raise(DMA1_Stream5_IRQn);
}
;
extern "C" void DMA1_Stream6_IRQHandler(void) {
	InterruptManager::Raise(DMA1_Stream6_IRQn);
}
;
extern "C" void ADC_IRQHandler(void) {
	InterruptManager::Raise(ADC_IRQn);
}
;
extern "C" void EXTI9_5_IRQHandler(void) {
	InterruptManager::Raise(EXTI9_5_IRQn);
}
;
extern "C" void TIM1_BRK_TIM9_IRQHandler(void) {
	InterruptManager::Raise(TIM1_BRK_TIM9_IRQn);
}
;
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
	InterruptManager::Raise(TIM1_UP_TIM10_IRQn);
}
;
extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(void) {
	InterruptManager::Raise(TIM1_TRG_COM_TIM11_IRQn);
}
;
extern "C" void TIM1_CC_IRQHandler(void) {
	InterruptManager::Raise(TIM1_CC_IRQn);
}
;
extern "C" void TIM2_IRQHandler(void) {
	InterruptManager::Raise(TIM2_IRQn);
}
;
extern "C" void TIM3_IRQHandler(void) {
	InterruptManager::Raise(TIM3_IRQn);
	;
}
;
extern "C" void TIM4_IRQHandler(void) {
	InterruptManager::Raise(TIM4_IRQn);
}
;
extern "C" void I2C1_EV_IRQHandler(void) {
	InterruptManager::Raise(I2C1_EV_IRQn);
}
;
extern "C" void I2C1_ER_IRQHandler(void) {
	InterruptManager::Raise(I2C1_ER_IRQn);
}
;
extern "C" void I2C2_EV_IRQHandler(void) {
	InterruptManager::Raise(I2C2_EV_IRQn);
}
;
extern "C" void I2C2_ER_IRQHandler(void) {
	InterruptManager::Raise(I2C2_ER_IRQn);
}
;
extern "C" void SPI1_IRQHandler(void) {
	InterruptManager::Raise(SPI1_IRQn);
}
;
extern "C" void SPI2_IRQHandler(void) {
	InterruptManager::Raise(SPI2_IRQn);
}
;
extern "C" void USART1_IRQHandler(void) {
	InterruptManager::Raise(USART1_IRQn);
}
;
extern "C" void USART2_IRQHandler(void) {
	InterruptManager::Raise(USART2_IRQn);
}
;
extern "C" void EXTI15_10_IRQHandler(void) {
	InterruptManager::Raise(EXTI15_10_IRQn);
}
;

extern "C" void RTC_Alarm_IRQHandler(void) {
	InterruptManager::Raise(RTC_Alarm_IRQn);
}
;
extern "C" void OTG_FS_WKUP_IRQHandler(void) {
	InterruptManager::Raise(OTG_FS_WKUP_IRQn);
}
;
extern "C" void DMA1_Stream7_IRQHandler(void) {
	InterruptManager::Raise(DMA1_Stream7_IRQn);
}
;
extern "C" void FSMC_IRQHandler(void) {
	InterruptManager::Raise(FSMC_IRQn);
}
;
extern "C" void SDIO_IRQHandler(void) {
	InterruptManager::Raise(SDIO_IRQn);
}
;
extern "C" void TIM5_IRQHandler(void) {
	InterruptManager::Raise(TIM5_IRQn);
}
;
extern "C" void SPI3_IRQHandler(void) {
	InterruptManager::Raise(SPI3_IRQn);
}
;
extern "C" void DMA2_Stream0_IRQHandler(void) {
	InterruptManager::Raise(DMA2_Stream0_IRQn);
}
;
extern "C" void DMA2_Stream1_IRQHandler(void) {
	InterruptManager::Raise(DMA2_Stream1_IRQn);
}
;
extern "C" void DMA2_Stream2_IRQHandler(void) {
	InterruptManager::Raise(DMA2_Stream2_IRQn);
}
;
extern "C" void DMA2_Stream3_IRQHandler(void) {
	InterruptManager::Raise(DMA2_Stream3_IRQn);
}
;
extern "C" void DMA2_Stream4_IRQHandler(void) {
	InterruptManager::Raise(DMA2_Stream4_IRQn);
}
;
extern "C" void CAN1_TX_IRQHandler(void) {
	InterruptManager::Raise(CAN1_TX_IRQn);
}
;
extern "C" void CAN1_RX0_IRQHandler(void) {
	InterruptManager::Raise(CAN1_RX0_IRQn);
}
;
extern "C" void CAN1_RX1_IRQHandler(void) {
	InterruptManager::Raise(CAN1_RX1_IRQn);
}
;
extern "C" void CAN1_SCE_IRQHandler(void) {
	InterruptManager::Raise(CAN1_SCE_IRQn);
}
;
extern "C" void CAN2_TX_IRQHandler(void) {
	InterruptManager::Raise(CAN2_TX_IRQn);
}
;
extern "C" void CAN2_RX0_IRQHandler(void) {
	InterruptManager::Raise(CAN2_RX0_IRQn);
}
;
extern "C" void CAN2_RX1_IRQHandler(void) {
	InterruptManager::Raise(CAN2_RX1_IRQn);
}
;
extern "C" void CAN2_SCE_IRQHandler(void) {
	InterruptManager::Raise(CAN2_SCE_IRQn);
}
;
extern "C" void USART3_IRQHandler(void) {
	InterruptManager::Raise(USART3_IRQn);
}
;
extern "C" void TIM8_BRK_TIM12_IRQHandler(void) {
	InterruptManager::Raise(TIM8_BRK_TIM12_IRQn);
}
;
extern "C" void TIM8_UP_TIM13_IRQHandler(void) {
	InterruptManager::Raise(TIM8_UP_TIM13_IRQn);
}
;
extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void) {
	InterruptManager::Raise(TIM8_TRG_COM_TIM14_IRQn);
}
;
extern "C" void TIM8_CC_IRQHandler(void) {
	InterruptManager::Raise(TIM8_CC_IRQn);
}
;
extern "C" void UART4_IRQHandler(void) {
	InterruptManager::Raise(UART4_IRQn);
}
;
extern "C" void UART5_IRQHandler(void) {
	InterruptManager::Raise(UART5_IRQn);
}
;
extern "C" void TIM6_DAC_IRQHandler(void) {
	InterruptManager::Raise(TIM6_DAC_IRQn);
}
;
extern "C" void TIM7_IRQHandler(void) {
	InterruptManager::Raise(TIM7_IRQn);
}
;
extern "C" void OTG_HS_EP1_OUT_IRQHandler(void) {
	InterruptManager::Raise(OTG_HS_EP1_OUT_IRQn);
}
;
extern "C" void OTG_HS_EP1_IN_IRQHandler(void) {
	InterruptManager::Raise(OTG_HS_EP1_IN_IRQn);
}
;
extern "C" void OTG_HS_WKUP_IRQHandler(void) {
	InterruptManager::Raise(OTG_HS_WKUP_IRQn);
}
;
extern "C" void OTG_HS_IRQHandler(void) {
	InterruptManager::Raise(OTG_HS_IRQn);
}
;
extern "C" void HASH_RNG_IRQHandler(void) {
	InterruptManager::Raise(HASH_RNG_IRQn);
}
;
extern "C" void ETH_IRQHandler(void) {
	HAL_ETH_IRQHandler(&EthHandle);
}
;

extern "C" void ETH_WKUP_IRQHandler(void) {
	InterruptManager::Raise(ETH_WKUP_IRQn);
}
;
extern "C" void DCMI_IRQHandler(void) {
	InterruptManager::Raise(DCMI_IRQn);
}
;
extern "C" void OTG_FS_IRQHandler(void) {
	InterruptManager::Raise(OTG_FS_IRQn);
}
;
extern "C" void DMA2_Stream5_IRQHandler(void) {
	InterruptManager::Raise(DMA2_Stream5_IRQn);
}
;
extern "C" void DMA2_Stream6_IRQHandler(void) {
	InterruptManager::Raise(DMA2_Stream6_IRQn);
}
;
extern "C" void DMA2_Stream7_IRQHandler(void) {
	InterruptManager::Raise(DMA2_Stream7_IRQn);
}
;
extern "C" void USART6_IRQHandler(void) {
	InterruptManager::Raise(USART6_IRQn);
}
;
extern "C" void I2C3_EV_IRQHandler(void) {
	InterruptManager::Raise(I2C3_EV_IRQn);
}
;
extern "C" void I2C3_ER_IRQHandler(void) {
	InterruptManager::Raise(I2C3_ER_IRQn);
}
;
extern "C" void FPU_IRQHandler(void) {
	InterruptManager::Raise(FPU_IRQn);
}
;

