/* ****************************************************************************************************
 * lib_ser_init_itf.h within the following project: bld_device_cmake_Nucleo_STM32F401
 *	
 *  compiler:   GNU Tools ARM Embedded (4.7.201xqx)
 *  target:     Cortex Mx
 *  author:		thomas
 * ****************************************************************************************************/

/* ****************************************************************************************************/

/*
 *	******************************* change log *******************************
 *  date			user			comment
 * 	Jun 7, 2018			thomas			- creation of lib_ser_init_itf.h
 *  
 */



#ifndef INT_IO_LIB_SER_SH_LIB_SER_STM32_LIB_SER_INIT_ITF_H_
#define INT_IO_LIB_SER_SH_LIB_SER_STM32_LIB_SER_INIT_ITF_H_

/* *******************************************************************
 * includes
 * ******************************************************************/

/* system */
#ifdef STM32HAL_F1
	#include <stm32f1xx.h>
	#include <stm32f1xx_hal_dma.h>		// Recursively required from tm32f1xx_hal_tim.h
	#include <stm32f1xx_hal_uart.h>		// RCC_* functions
	#include <stm32f4xx_hal_rcc.h>
	#include <stm32f1xx_hal_gpio_ex.h>
#elif STM32HAL_F4
	#include <stm32f4xx.h>
	#include <stm32f4xx_hal_dma.h>		// Recursively required from tm32f1xx_hal_tim.h
	#include <stm32f4xx_hal_uart.h>		// RCC_* functions
	#include <stm32f4xx_hal_rcc.h>
	#include <stm32f4xx_hal_gpio_ex.h>
#else
	#error No Architecture is set at lib_ser
#endif


/* *******************************************************************
 * defines
 * ******************************************************************/
#define M_SER_DEVICE_CFG_UART_DMA(_uart, _dma, _tx_dma_stream, _tx_dma_channel, _rx_dma_stream, _rx_dma_channel) 	\
{																													\
	.uart_device = _uart, 																							\
	.dma = _dma,																									\
	.tx_dma_stream = _dma##_##_tx_dma_stream,																		\
	.tx_dma_channel = _tx_dma_channel,      																		\
	.rx_dma_stream = _dma##_##_rx_dma_stream,        																\
	.rx_dma_channel = _rx_dma_channel,       																		\
	.huart_isr_type = _uart##_IRQn,																					\
	.hdma_tx_isr_type = _dma##_##_tx_dma_stream##_IRQn,																\
    .hdma_rx_isr_type = _dma##_##_rx_dma_stream##_IRQn,																\
}

#define M_SER_DEVICE_CFG_UART_ISR(_uart) 	\
{																													\
	.uart_device = _uart, 																							\
	.dma = NULL,																									\
	.tx_dma_stream = NULL,																							\
	.tx_dma_channel = 0,      																						\
	.rx_dma_stream =  NULL,        																					\
	.rx_dma_channel = 0,       																						\
	.huart_isr_type = _uart##_IRQn,																					\
	.hdma_tx_isr_type = NonMaskableInt_IRQn,																		\
    .hdma_rx_isr_type = NonMaskableInt_IRQn,																		\
}

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/
enum lib_ser_isr_type {
	LIB_SER_ISR_TYPE_uart,
	LIB_SER_ISR_TYPE_dma_tx,
	LIB_SER_ISR_TYPE_dma_rx,
	LIB_SER_ISR_TYPE_CNT
};


struct ser_device_cfg {
	USART_TypeDef const * const uart_device;
	DMA_TypeDef const * dma;
	DMA_Stream_TypeDef const *tx_dma_stream;
	const uint32_t tx_dma_channel;
	DMA_Stream_TypeDef const *rx_dma_stream;
	const uint32_t rx_dma_channel;
	const IRQn_Type huart_isr_type;
	const IRQn_Type hdma_tx_isr_type;
	const IRQn_Type hdma_rx_isr_type;
};



/* *******************************************************************
 * (static) variables declarations
 * ******************************************************************/


/* *******************************************************************
 * static function declarations
 * ******************************************************************/

#endif /* INT_IO_LIB_SER_SH_LIB_SER_STM32_LIB_SER_INIT_ITF_H_ */
