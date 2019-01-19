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

#ifndef _SERIAL_ACCESS_STM32_H_
#define _SERIAL_ACCESS_STM32_H_

#ifdef __cplusplus
extern "C" {
#endif

/* *******************************************************************
 * includes
 * ******************************************************************/

/* c-runtime */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* system */
#ifdef STM32HAL_F1
	#include <stm32f1xx.h>
	#include <stm32f1xx_hal_dma.h>		// Recursively required from tm32f1xx_hal_tim.h
 	#include <stm32f1xx_hal_uart.h>		// RCC_* functions
	#include <stm32f1xx_hal_gpio.h>
	#include <stm32f4xx_hal_rcc.h>
 	#include <stm32f1xx_hal_gpio_ex.h>
#elif STM32HAL_F4
	#include <stm32f4xx.h>
	#include <stm32f4xx_hal_dma.h>		// Recursively required from tm32f1xx_hal_tim.h
	#include <stm32f4xx_hal_uart.h>		// RCC_* functions
	#include <stm32f4xx_hal_gpio.h>
	#include <stm32f4xx_hal_rcc.h>
	#include <stm32f4xx_hal_gpio_ex.h>
#else
 	#error No STM32 Architecture at lib_serial STM32 port selected
#endif

/* frame */
#include <lib_convention__macro.h>
#include <FreeRTOS.h>
#include <lib_log.h>
#include <lib_isr.h>
#include <lib_timer.h>
#include <lib_serial_stm32.h>

/* project */
#include <lib_serial_types.h>


/* *******************************************************************
 * defines
 * ******************************************************************/
#define SERIAL_INITIALIZED				0x7FAB7FAB
#define M_LIB_SER__WAIT_INFINITE		0xFFFF

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/

enum receive_mode
{
	RECEIVE_MODE_disable,
	RECEIVE_MODE_enable
};

enum receive_event
{
	RECEIVE_EVENT_overflow,
	RECEIVE_EVENT_timeout
};

struct rx_info
{
	unsigned int number_received_data;			//Number of received characters
	unsigned int number_parity_error;			//Number of parity errors
	unsigned int number_framing_error;			//Number of framing errors
	unsigned int number_noise_error;			//Number of noise errors
};

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/
enum ser_mode {
	SER_MODE_isr,
	SER_MODE_dma
};


typedef struct serial_access_handle serial_access_t;
typedef void (ser_tx_complete_handler_t)(serial_access_t *_hdl);
typedef enum receive_mode (ser_rx_complete_handler_t)(serial_access_t *_hdl, enum receive_event, struct rx_info const * const);

struct serial_access_handle
{
	/* HAL hdl */
	UART_HandleTypeDef huart;
	DMA_HandleTypeDef hdma_tx;
	DMA_HandleTypeDef hdma_rx;
	/* hdl */
	lib_isr_hdl_t isr_uart_hdl;
	lib_isr_hdl_t isr_dma_tx_hdl;
	lib_isr_hdl_t isr_dma_rx_hdl;
	timer_hdl_t rx_timeout_hdl;
	/* parameter */
	unsigned int rx_timeout_interval;
	enum ser_mode mode;
	IRQn_Type huart_isr_type;
	IRQn_Type hdma_tx_isr_type;
	IRQn_Type hdma_rx_isr_type;
	ser_tx_complete_handler_t *tx_complete_handler;
	ser_rx_complete_handler_t *rx_complete_handler;
	unsigned int initialized;
	struct rx_info rx_frame_info;
};


/* ************************************************************************//**
 * \brief  Initialization of the serial driver
 *
 * \param   _hdl* [IN]      	 : serial communication handle
 * \param   _port* [IN] 	     : serial port config
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__config(serial_access_t *_hdl, struct port_config *_port);

/* ************************************************************************//**
 * \brief  open of the serial driver
 *
 * \param   _hdl* [OUT]          : serial communication handle
 * \param   _baudrate            : serial baudrate
 * \param   _format              : serial port parameter setting
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__open(serial_access_t *_hdl, const USART_TypeDef *_uartDevice, enum baudrate _baudrate, enum data_format _format);

/* ************************************************************************//**
 * \brief  open of the serial driver
 *
 * \param   _hdl* [OUT]          : serial communication handle

 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__close(serial_access_t *_hdl);

/* ************************************************************************//**
 * \brief  Cleanup of the serial driver

 * \param   _ser_hdl* [OUT]      : serial communication handle
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__cleanup(serial_access_t *_hdl);

/* ************************************************************************//**
 * \brief  Send of RAW data
 * \description	 A number of bytes (frame) is transmitted by the serial interface.
 * 	
 * \param   _ser_hdl [IN]        : serial communication handle
 * \param   _data_send[IN]       : a pointer to the data to be send
 * \param   _length         	 : number of bytes to send
 * \param   _tx_complete_handler : event handler, which signalizes if a serial frame is transmitted
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__send(serial_access_t *_hdl, const uint8_t * const _data_send, unsigned int _length, ser_tx_complete_handler_t _tx_complete_handler);

/* ************************************************************************//**
 * \brief  Initialization an serial receiver to handle incoming data
 * \description	 This routine is responsible to configure a receiver path
 * 				 for incoming data. The registered event handler is called if a
 * 				 inter-frame timeout is occurred or the prepared receive buffer
 * 				 is overrun. 
 *
 * \param   _ser_hdl [IN]        : serial communication handle
 * \param   _receive_buffer[out] : a pointer to store the received data
 * \param   _length         	 : maximal length of the receive buffer
 * \param   _timeout			 : timeout (ms) for RX cancellation in case of no incoming data. timeout = 0 for disable
 * \param   _rx_complete_handler : event handler, that the maximal buffer length is
 * 								   reached or the adjusted inter-frame timeout eyceeded
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__prepare_receiver(serial_access_t *_hdl, uint8_t *_receive_buffer, unsigned int _length, unsigned int _timeout, ser_rx_complete_handler_t *_rx_complete_handler);

/* ************************************************************************//**
 * \brief  Start/Stop of the serial receiver
 * \description	 This routine starts or stops the receiver. So a new received frame
 * 				 can be processed at first. 
 *
 * \param   _ser_hdl [IN]        : serial communication handle
 * \param   _mode                : sets the receiver on/off
 *
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__set_receiver_on_off(serial_access_t *_hdl, const enum receive_mode _mode);

/* ************************************************************************//**
 * \brief  Configure the destination address of the next receiver frame
 * \description	 This routine reconfigure the destination address and length of
 * 				 the receiver buffer.
 *
 * \param   _ser_hdl [IN]        : serial communication handle
 * \param   _receive_buffer[out] : a pointer to store the next to data to receive
 * \param   _length         	 : maximal length of the new receive buffer
 *
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__set_receiver_frame(serial_access_t *_hdl, uint8_t *_receive_buffer, unsigned int _length);

#ifdef __cplusplus
}
#endif

#endif  /*_SERIAL_ACCESS_STM32_H_*/
