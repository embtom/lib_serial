/*
 * This file is part of the EMBTOM project
 * Copyright (c) 2018-2019 Thomas Willetal 
 * (https://github.com/tom3333)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _LIB_SERIAL_STM32_H_
#define _LIB_SERIAL_STM32_H_

#ifdef __cplusplus
extern "C" {
#endif

/* *******************************************************************
 * includes
 * ******************************************************************/
#include <lib_serial_types.h>
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
	#error No STM32 Architecture at lib_serial STM32 port selected
#endif

/* *******************************************************************
 * defines
 * ******************************************************************/
#define STM32_PORT_CONFIG_DMA(_uart, _dma, _tx_dma_stream, _tx_dma_channel, _rx_dma_stream, _rx_dma_channel) 	\
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

#define STM32_PORT_CONFIG_ISR(_uart) 	\
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

struct port_config {
	const USART_TypeDef *uart_device;
	const DMA_TypeDef *dma;
	const DMA_Stream_TypeDef *tx_dma_stream;
	const uint32_t tx_dma_channel;
	const DMA_Stream_TypeDef *rx_dma_stream;
	const uint32_t rx_dma_channel;
	const IRQn_Type huart_isr_type;
	const IRQn_Type hdma_tx_isr_type;
	const IRQn_Type hdma_rx_isr_type;
};


// #define M_SER_DMA_CHANNEL_MAP_DESCRIPTOR 															  
// {																									  
// 	STM32_PORT_CONFIG_DMA(USART1, DMA2 ,Stream7, DMA_CHANNEL_4, Stream2,DMA_CHANNEL_4), 
// 	STM32_PORT_CONFIG_DMA(USART2, DMA1 ,Stream6, DMA_CHANNEL_4, Stream5,DMA_CHANNEL_4), 
// 	STM32_PORT_CONFIG_DMA(USART6, DMA2 ,Stream6, DMA_CHANNEL_5, Stream1,DMA_CHANNEL_5), 
// }

//#define M_SER_DMA_CHANNEL_MAP_DESCRIPTOR 															   
//{																									   
//	STM32_PORT_CONFIG_DMA(USART1, DMA2 ,Stream7, DMA_CHANNEL_4, Stream2,DMA_CHANNEL_4), 
//	STM32_PORT_CONFIG_ISR(USART2), \
//	STM32_PORT_CONFIG_DMA(USART6, DMA2 ,Stream6, DMA_CHANNEL_5, Stream1,DMA_CHANNEL_5), 
//}

/* Func: lib_serial_create()
 * Desc: Creates a handle to a specific serial port.
 *       On failure the pointer returned is NULL.
 *       The port will not be opened yet.
 *       This is the counter-function to lib_serial_destroy.
 * Param-in port: the port to be used for communication
 *                (NULL: behavior is undefined)
 * Return: the handle to be used for function calls to other API functions
 */
lib_serial_hdl lib_serial_create_stm32(struct port_config *port);

#ifdef __cplusplus
}
#endif

#endif  /* _LIB_SERIAL_STM32_H_ */