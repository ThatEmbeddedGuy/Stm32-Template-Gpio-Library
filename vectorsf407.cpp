/*
 * vectorsf407.cpp
 *
 *  Created on: 23 рту. 2016 у.
 *      Author: tihonov
 */


#include "vectorsf407.h"
#include "interruptmanager.h"

//******************************************************************************************
//Interpritation fo interrupt vectors works only with flash implementation of interrupt manager
//******************************************************************************************

#if INTERRUPTMANAGER_ENABLED==INTERRUPTMANAGER_FLASH

extern "C" void WWDG_IRQHandler(void) {
	INTERRUPTMANAGER::raise(WWDG_IRQn);
}
;
extern "C" void PVD_IRQHandler(void) {
	INTERRUPTMANAGER::raise(PVD_IRQn);
}
;
extern "C" void TAMP_STAMP_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TAMP_STAMP_IRQn);
}
;
extern "C" void RTC_WKUP_IRQHandler(void) {
	INTERRUPTMANAGER::raise(RTC_WKUP_IRQn);
}
;
extern "C" void FLASH_IRQHandler(void) {
	INTERRUPTMANAGER::raise(FLASH_IRQn);
}
;
extern "C" void RCC_IRQHandler(void) {
	INTERRUPTMANAGER::raise(RCC_IRQn);
}
;
extern "C" void EXTI0_IRQHandler(void) {
	INTERRUPTMANAGER::raise(EXTI0_IRQn);
}
;
extern "C" void EXTI1_IRQHandler(void) {
	INTERRUPTMANAGER::raise(EXTI1_IRQn);
}
;
extern "C" void EXTI2_IRQHandler(void) {
	INTERRUPTMANAGER::raise(EXTI2_IRQn);
}
;
extern "C" void EXTI3_IRQHandler(void) {
	INTERRUPTMANAGER::raise(EXTI3_IRQn);
}
;
extern "C" void EXTI4_IRQHandler(void) {
	INTERRUPTMANAGER::raise(EXTI4_IRQn);
}
;
extern "C" void DMA1_Stream0_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA1_Stream0_IRQn);
}
;
extern "C" void DMA1_Stream1_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA1_Stream1_IRQn);
}
;
extern "C" void DMA1_Stream2_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA1_Stream2_IRQn);
}
;
extern "C" void DMA1_Stream3_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA1_Stream3_IRQn);
}
;
extern "C" void DMA1_Stream4_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA1_Stream4_IRQn);
}
;
extern "C" void DMA1_Stream5_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA1_Stream5_IRQn);
}
;
extern "C" void DMA1_Stream6_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA1_Stream6_IRQn);
}
;
extern "C" void ADC_IRQHandler(void) {
	INTERRUPTMANAGER::raise(ADC_IRQn);
}
;
extern "C" void EXTI9_5_IRQHandler(void) {
	INTERRUPTMANAGER::raise(EXTI9_5_IRQn);
}
;
extern "C" void TIM1_BRK_TIM9_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM1_BRK_TIM9_IRQn);
}
;
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM1_UP_TIM10_IRQn);
}
;
extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM1_TRG_COM_TIM11_IRQn);
}
;
extern "C" void TIM1_CC_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM1_CC_IRQn);
}
;
extern "C" void TIM2_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM2_IRQn);
}
;
extern "C" void TIM3_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM3_IRQn);
	;
}
;
extern "C" void TIM4_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM4_IRQn);
}
;
extern "C" void I2C1_EV_IRQHandler(void) {
	INTERRUPTMANAGER::raise(I2C1_EV_IRQn);
}
;
extern "C" void I2C1_ER_IRQHandler(void) {
	INTERRUPTMANAGER::raise(I2C1_ER_IRQn);
}
;
extern "C" void I2C2_EV_IRQHandler(void) {
	INTERRUPTMANAGER::raise(I2C2_EV_IRQn);
}
;
extern "C" void I2C2_ER_IRQHandler(void) {
	INTERRUPTMANAGER::raise(I2C2_ER_IRQn);
}
;
extern "C" void SPI1_IRQHandler(void) {
	INTERRUPTMANAGER::raise(SPI1_IRQn);
}
;
extern "C" void SPI2_IRQHandler(void) {
	INTERRUPTMANAGER::raise(SPI2_IRQn);
}
;
extern "C" void USART1_IRQHandler(void) {
	INTERRUPTMANAGER::raise(USART1_IRQn);
}
;
extern "C" void USART2_IRQHandler(void) {
	INTERRUPTMANAGER::raise(USART2_IRQn);
}
;
extern "C" void EXTI15_10_IRQHandler(void) {
	INTERRUPTMANAGER::raise(EXTI15_10_IRQn);
}
;

extern "C" void RTC_Alarm_IRQHandler(void) {
	INTERRUPTMANAGER::raise(RTC_Alarm_IRQn);
}
;
extern "C" void OTG_FS_WKUP_IRQHandler(void) {
	INTERRUPTMANAGER::raise(OTG_FS_WKUP_IRQn);
}
;
extern "C" void DMA1_Stream7_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA1_Stream7_IRQn);
}
;
extern "C" void FSMC_IRQHandler(void) {
	INTERRUPTMANAGER::raise(FSMC_IRQn);
}
;
extern "C" void SDIO_IRQHandler(void) {
	INTERRUPTMANAGER::raise(SDIO_IRQn);
}
;
extern "C" void TIM5_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM5_IRQn);
}
;
extern "C" void SPI3_IRQHandler(void) {
	INTERRUPTMANAGER::raise(SPI3_IRQn);
}
;
extern "C" void DMA2_Stream0_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA2_Stream0_IRQn);
}
;
extern "C" void DMA2_Stream1_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA2_Stream1_IRQn);
}
;
extern "C" void DMA2_Stream2_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA2_Stream2_IRQn);
}
;
extern "C" void DMA2_Stream3_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA2_Stream3_IRQn);
}
;
extern "C" void DMA2_Stream4_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA2_Stream4_IRQn);
}
;
extern "C" void CAN1_TX_IRQHandler(void) {
	INTERRUPTMANAGER::raise(CAN1_TX_IRQn);
}
;
extern "C" void CAN1_RX0_IRQHandler(void) {
	INTERRUPTMANAGER::raise(CAN1_RX0_IRQn);
}
;
extern "C" void CAN1_RX1_IRQHandler(void) {
	INTERRUPTMANAGER::raise(CAN1_RX1_IRQn);
}
;
extern "C" void CAN1_SCE_IRQHandler(void) {
	INTERRUPTMANAGER::raise(CAN1_SCE_IRQn);
}
;
extern "C" void CAN2_TX_IRQHandler(void) {
	INTERRUPTMANAGER::raise(CAN2_TX_IRQn);
}
;
extern "C" void CAN2_RX0_IRQHandler(void) {
	INTERRUPTMANAGER::raise(CAN2_RX0_IRQn);
}
;
extern "C" void CAN2_RX1_IRQHandler(void) {
	INTERRUPTMANAGER::raise(CAN2_RX1_IRQn);
}
;
extern "C" void CAN2_SCE_IRQHandler(void) {
	INTERRUPTMANAGER::raise(CAN2_SCE_IRQn);
}
;
extern "C" void USART3_IRQHandler(void) {
	INTERRUPTMANAGER::raise(USART3_IRQn);
}
;
extern "C" void TIM8_BRK_TIM12_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM8_BRK_TIM12_IRQn);
}
;
extern "C" void TIM8_UP_TIM13_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM8_UP_TIM13_IRQn);
}
;
extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM8_TRG_COM_TIM14_IRQn);
}
;
extern "C" void TIM8_CC_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM8_CC_IRQn);
}
;
extern "C" void UART4_IRQHandler(void) {
	INTERRUPTMANAGER::raise(UART4_IRQn);
}
;
extern "C" void UART5_IRQHandler(void) {
	INTERRUPTMANAGER::raise(UART5_IRQn);
}
;
extern "C" void TIM6_DAC_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM6_DAC_IRQn);
}
;
extern "C" void TIM7_IRQHandler(void) {
	INTERRUPTMANAGER::raise(TIM7_IRQn);
}
;
extern "C" void OTG_HS_EP1_OUT_IRQHandler(void) {
	INTERRUPTMANAGER::raise(OTG_HS_EP1_OUT_IRQn);
}
;
extern "C" void OTG_HS_EP1_IN_IRQHandler(void) {
	INTERRUPTMANAGER::raise(OTG_HS_EP1_IN_IRQn);
}
;
extern "C" void OTG_HS_WKUP_IRQHandler(void) {
	INTERRUPTMANAGER::raise(OTG_HS_WKUP_IRQn);
}
;
extern "C" void OTG_HS_IRQHandler(void) {
	INTERRUPTMANAGER::raise(OTG_HS_IRQn);
}
;
extern "C" void HASH_RNG_IRQHandler(void) {
	INTERRUPTMANAGER::raise(HASH_RNG_IRQn);
}
;
extern "C" void ETH_IRQHandler(void) {
	INTERRUPTMANAGER::raise(ETH_IRQn);
}
;

extern "C" void ETH_WKUP_IRQHandler(void) {
	INTERRUPTMANAGER::raise(ETH_WKUP_IRQn);
}
;
extern "C" void DCMI_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DCMI_IRQn);
}
;
extern "C" void OTG_FS_IRQHandler(void) {
	INTERRUPTMANAGER::raise(OTG_FS_IRQn);
}
;
extern "C" void DMA2_Stream5_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA2_Stream5_IRQn);
}
;
extern "C" void DMA2_Stream6_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA2_Stream6_IRQn);
}
;
extern "C" void DMA2_Stream7_IRQHandler(void) {
	INTERRUPTMANAGER::raise(DMA2_Stream7_IRQn);
}
;
extern "C" void USART6_IRQHandler(void) {
	INTERRUPTMANAGER::raise(USART6_IRQn);
}
;
extern "C" void I2C3_EV_IRQHandler(void) {
	INTERRUPTMANAGER::raise(I2C3_EV_IRQn);
}
;
extern "C" void I2C3_ER_IRQHandler(void) {
	INTERRUPTMANAGER::raise(I2C3_ER_IRQn);
}
;
extern "C" void FPU_IRQHandler(void) {
	INTERRUPTMANAGER::raise(FPU_IRQn);
}
;
#endif

