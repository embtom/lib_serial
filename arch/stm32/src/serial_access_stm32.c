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

/* *******************************************************************
 * includes
 * ******************************************************************/

/* c-runtime */
#include <stdlib.h>
#include <string.h>
#include <errno.h>

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
	#error No Architecture is set at lib_clock
#endif

/* frame */
#include <lib_convention__errno.h>
#include <lib_convention__macro.h>
#include <FreeRTOS.h>
#include <lib_isr.h>
#include <lib_timer.h>

/* project */
#include <serial_access_stm32.h>

/* *******************************************************************
 * static data
 * ******************************************************************/
static const char* __attribute__ ((section(".text"))) s_ser_mode_name[] = {"SER_MODE_isr", "SER_MODE_dma"};

/* *******************************************************************
 * static function declarations
 * ******************************************************************/
static void serial_access__uart_event(IRQn_Type _isr_vector, unsigned int _vector, void *_arg);
static void serial_access__uart_rx_timeout(timer_hdl_t _hdl, void* _arg);


/* *******************************************************************
 * function definition
 * ******************************************************************/

int serial_access__config(serial_access_t *_hdl, struct port_config *_port)
{
	int ret;
	if ((_hdl == NULL) || (_port == NULL)) {
		return -EINVAL;
	}
 	ret = lib_isr__attach (&(_hdl->isr_uart_hdl), _port->huart_isr_type, &serial_access__uart_event,  (void*)_hdl);
	if (ret < 0) {
		ret = -EIO;
		goto ERR_1;
	}
 	ret = lib_timer__open(&(_hdl->rx_timeout_hdl), _hdl, &serial_access__uart_rx_timeout);
	if (ret < 0) {
	 	ret = -EIO;
	 	goto ERR_2;
	}
 	if ((_port->rx_dma_stream == NULL) || (_port->rx_dma_stream == NULL))
	{
		_hdl->huart_isr_type = _port->huart_isr_type;
	 	_hdl->isr_dma_tx_hdl = NULL;
	 	_hdl->isr_dma_rx_hdl = NULL;
	 	_hdl->mode = SER_MODE_isr;
 	}
 	else {
	 	if(_port->dma == DMA1) {
 	 		__DMA1_CLK_ENABLE();
 	 	}
 	 	else if (_port->dma == DMA2) {
 	 		__DMA2_CLK_ENABLE();
 	 	}
 	 	else {
 	 	 	ret = -ENODEV;
 	 	 	goto ERR_3;
 	 	}
	     /* USART_TX DMA Init */
 	 	_hdl->hdma_tx.Instance = (DMA_Stream_TypeDef*)_port->tx_dma_stream; //DMA1_Stream6;
 	    _hdl->hdma_tx.Init.Channel = _port->tx_dma_channel; //DMA_CHANNEL_4;
 	    _hdl->hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
 	    _hdl->hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
 	    _hdl->hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
 	    _hdl->hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
 	    _hdl->hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
 	    _hdl->hdma_tx.Init.Mode = DMA_NORMAL;
 	    _hdl->hdma_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
 	    _hdl->hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
 	    ret = HAL_DMA_Init(&_hdl->hdma_tx);
 	    if (ret != HAL_OK) {
 	   		ret = -EIO;
 	   		goto ERR_DMA_TX;
 	    }
 		__HAL_LINKDMA(&(_hdl->huart),hdmatx,(_hdl->hdma_tx));
	    
		/* USART_RX DMA Init */
 	    _hdl->hdma_rx.Instance = (DMA_Stream_TypeDef*)_port->rx_dma_stream;
 	    _hdl->hdma_rx.Init.Channel = _port->rx_dma_channel;
 	    _hdl->hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
 	    _hdl->hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
 	    _hdl->hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
	    _hdl->hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    _hdl->hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    _hdl->hdma_rx.Init.Mode = DMA_NORMAL;
	    _hdl->hdma_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
	    _hdl->hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    ret = HAL_DMA_Init(&_hdl->hdma_rx);
	    if (ret != HAL_OK) {
	 	   ret = -EIO;
	 	   goto ERR_DMA_RX;
	    }
    	__HAL_LINKDMA(&(_hdl->huart),hdmarx,(_hdl->hdma_rx));
    	
		ret = lib_isr__attach (&(_hdl->isr_dma_tx_hdl), _port->hdma_tx_isr_type, &serial_access__uart_event,  (void*)_hdl);
	    if (ret < 0) {
	 	   goto ERR_DMA_ISR_TX;
	    }
    	ret = lib_isr__attach (&(_hdl->isr_dma_rx_hdl), _port->hdma_rx_isr_type, &serial_access__uart_event,  (void*)_hdl);
	    if (ret < 0) {
	 	   goto ERR_DMA_ISR_RX;
	    }
   	 	_hdl->hdma_tx_isr_type = _port->hdma_tx_isr_type;
	    _hdl->hdma_rx_isr_type = _port->hdma_rx_isr_type;
	    _hdl->huart_isr_type = _port->huart_isr_type;
	    _hdl->mode = SER_MODE_dma;
 	}
	return 0;
    
	ERR_DMA_ISR_RX:
		lib_isr__detach(&(_hdl->isr_dma_tx_hdl));
    ERR_DMA_ISR_TX:
		HAL_DMA_DeInit(&(_hdl->hdma_rx));
    ERR_DMA_RX:
		HAL_DMA_DeInit(&(_hdl->hdma_tx));
    ERR_DMA_TX:
		if(_port->dma == DMA1)
	 		__DMA1_CLK_DISABLE();
	 	else if (_port->dma == DMA2)
	  		__DMA2_CLK_DISABLE();
 	ERR_3:
	 	lib_timer__close(&(_hdl->rx_timeout_hdl));
 	ERR_2:
		lib_isr__detach(&(_hdl->isr_uart_hdl));
 	
	ERR_1:
	return ret;
}


/* ************************************************************************//**
 * \brief  Initialization of the serial driver
 *
 * \param   _serHdl* [OUT]      : serial communication handle
 * \param   _baudrate            : serial baudrate
 * \param   _format              : serial port parameter setting
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__open(serial_access_t *_hdl, const USART_TypeDef *_uartDevice, enum baudrate _baudrate, enum data_format _format)
{
	int ret;
	GPIO_InitTypeDef GPIO_InitStruct;

	if (_hdl == NULL) {
		return -EINVAL;
	}

	switch (_format)
	{
		case DATA_FORMAT_8_NONE_2:
		{	/* 8 data bits, no   parity, 2 stop bit   */
			_hdl->huart.Init.WordLength = UART_WORDLENGTH_8B;
			_hdl->huart.Init.StopBits = UART_STOPBITS_2;
			_hdl->huart.Init.Parity = UART_PARITY_NONE;
			break;
		}
		case DATA_FORMAT_8_EVEN_1:
		{	/* 8 data bits, even parity, 1 stop bit   */
			_hdl->huart.Init.WordLength = UART_WORDLENGTH_9B;
			_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			_hdl->huart.Init.Parity = UART_PARITY_EVEN;
			break;
		}
		case DATA_FORMAT_8_ODD_1:
		{	/* 8 data bits, odd  parity, 1 stop bit   */
			_hdl->huart.Init.WordLength = UART_WORDLENGTH_9B;
			_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			_hdl->huart.Init.Parity = UART_PARITY_ODD;
			break;
		}
		case DATA_FORMAT_8_NONE_1:
		{	/* 8 data bits, no   parity, 1 stop bit   */
			_hdl->huart.Init.WordLength = UART_WORDLENGTH_8B;
			_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			_hdl->huart.Init.Parity = UART_PARITY_NONE;
			break;
		}
		case DATA_FORMAT_7_EVEN_1:
		{	/* 7 data bits, even parity, 1 stop bit */
			_hdl->huart.Init.WordLength = UART_WORDLENGTH_8B;
			_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			_hdl->huart.Init.Parity = UART_PARITY_EVEN;
			break;
		}
		case DATA_FORMAT_7_ODD_1:
		{	/* 7 data bits, odd  parity, 1 stop bit */

			_hdl->huart.Init.WordLength = UART_WORDLENGTH_8B;
			_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			_hdl->huart.Init.Parity = UART_PARITY_ODD;
			break;
		}
		default :
		{
			return -EINVAL;
		}
	}

	__HAL_RCC_USART2_CLK_ENABLE();
	_hdl->huart.Instance = (USART_TypeDef*)_uartDevice;
	_hdl->huart.Init.BaudRate = _baudrate;
	_hdl->huart.Init.Mode = UART_MODE_TX_RX;
	_hdl->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	_hdl->huart.Init.OverSampling = UART_OVERSAMPLING_16;
	ret = HAL_UART_Init(&(_hdl->huart));
	if (ret != HAL_OK) {
		return -EIO;	
	}
	SET_BIT(_hdl->huart.Instance->CR1, USART_CR1_IDLEIE);
	_hdl->initialized = SERIAL_INITIALIZED;
	
    return 0;
}

/* ************************************************************************//**
 * \brief  open of the serial driver
 *
 * \param   _hdl* [OUT]          : serial communication handle
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__close(serial_access_t *_hdl)
{
	int ret;
	if (_hdl == NULL) {
		return -EINVAL;
	}

	ret = HAL_UART_DeInit(&(_hdl->huart));
	_hdl->initialized = 0;
	return ret;
}

/* ************************************************************************//**
 * \brief  Cleanup of the serial driver

 * \param   _serHdl* [OUT]      : serial communication handle
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__cleanup(serial_access_t *_hdl)
{
	int line, ret;
	HAL_StatusTypeDef hal_ret;

	if (_hdl == NULL) {
		return -EINVAL;
	}

	if (_hdl->initialized != SERIAL_INITIALIZED) {
		return -EIO;
	}

	if ((_hdl->mode) == SER_MODE_dma) {

		ret = lib_isr__detach(&(_hdl->isr_dma_rx_hdl));
		ret |= lib_isr__detach(&(_hdl->isr_dma_tx_hdl));
		hal_ret = HAL_DMA_DeInit(&(_hdl->hdma_rx));
		if (hal_ret != HAL_OK) {
			ret |= -EIO;
		}

		hal_ret = HAL_DMA_DeInit(&(_hdl->hdma_tx));
		if (hal_ret != HAL_OK) {
			ret |= -EIO;
		}

		__DMA1_CLK_DISABLE();
		__DMA2_CLK_DISABLE();
	}

	ret |= lib_timer__close(&(_hdl->rx_timeout_hdl));
	__HAL_RCC_USART2_CLK_DISABLE();
	lib_isr__detach(&(_hdl->isr_uart_hdl));
    return ret;
}

/* ************************************************************************//**
 * \brief  Send of RAW data
 * \description	 A number of bytes (frame) is transmitted by the serial interface.
 * 	
 * \param   _serHdl [IN]        : serial communication handle
 * \param   _data_send[IN]       : a pointer to the data to be send
 * \param   _length         	 : number of bytes to send
 * \param   _tx_complete_handler : event handler, which signalizes if a serial frame is transmitted
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__send(serial_access_t *_hdl, const uint8_t * const _data_send, unsigned int _length, ser_tx_complete_handler_t _tx_complete_handler)
{
	HAL_StatusTypeDef hal_ret;
	int ret;

	if ((_hdl == NULL) || (_data_send == NULL)) {
		return -EINVAL;
	}

	if (_hdl->initialized != SERIAL_INITIALIZED) {
		return -EINVAL;
	}

	_hdl->tx_complete_handler = _tx_complete_handler;

	switch (_hdl->mode) {

		case SER_MODE_isr:
			hal_ret = HAL_UART_Transmit_IT(&_hdl->huart, (uint8_t*)_data_send, _length);
			break;
		case SER_MODE_dma:
			if (_length == 1) {
				hal_ret = HAL_UART_Transmit_IT(&_hdl->huart, (uint8_t*)_data_send, _length);	
			}
			else {
				hal_ret = HAL_UART_Transmit_DMA(&_hdl->huart, (uint8_t*)_data_send, _length);
			}

			break;
	}

	switch (hal_ret) {
		case HAL_OK     :	ret = EOK; break;
		case HAL_ERROR  :	ret = -EHAL_ERROR; break;
		case HAL_BUSY   :	ret = -ESTD_BUSY; break;
		case HAL_TIMEOUT:	ret = -EEXEC_TO; break;
		default: ret = -ESTD_FAULT; break;
	}

	if (ret < EOK) {
		goto ERR_0;
	}

    return EOK;

	ERR_0:
	return ret;
}

/* ************************************************************************//**
 * \brief  Initialization an serial receiver to handle incoming data
 * \description	 This routine is responsible to configure a receiver path
 * 				 for incoming data. The registered event handler is called if a
 * 				 inter-frame timeout is occurred or the prepared receive buffer
 * 				 is overrun. 
 *
 * \param   _serHdl [IN]        : serial communication handle
 * \param   _receive_buffer[out] : a pointer to store the received data
 * \param   _length         	 : maximal length of the receive buffer
 * \param   _timeout			 : timeout (ms) for RX cancellation in case of no incoming data. timeout = 0 for disable
 * \param   _rx_complete_handler : event handler, that the maximal buffer length is
 * 								   reached or the adjusted inter-frame timeout eyceeded
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__prepare_receiver(serial_access_t *_hdl, uint8_t *_receive_buffer, unsigned int _length, unsigned int _timeout, ser_rx_complete_handler_t *_rx_complete_handler)
{
	int ret;
	HAL_StatusTypeDef hal_ret;

	if (_hdl == NULL) {
		return -EINVAL;
	}

	if (_timeout == 0) {
		return -EINVAL;
	}

	if ((_hdl == NULL) || (_receive_buffer == NULL)) {
		return -EINVAL;
	}

	if (_hdl->initialized != SERIAL_INITIALIZED) {
		return -EIO;
	}

	CLEAR_BITVAL(_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
	memset(&_hdl->rx_frame_info, 0, sizeof(struct rx_info));
	_hdl->rx_complete_handler = _rx_complete_handler;
	_hdl->rx_timeout_interval = _timeout;
	HAL_UART_AbortReceive(&_hdl->huart);

	switch (_hdl->mode) {
		case SER_MODE_isr:
			hal_ret = HAL_UART_Receive_IT(&_hdl->huart,_receive_buffer,_length);
			break;

		case SER_MODE_dma:
			hal_ret = HAL_UART_Receive_DMA(&_hdl->huart,_receive_buffer,_length);
			break;

		default:
			ret = -EFAULT;
			break;
	}

	switch (hal_ret) {
		case HAL_OK     :	ret = EOK; break;
		case HAL_ERROR  :	ret = -EIO; break;
		case HAL_BUSY   :	ret = -ESTD_BUSY; break;
		case HAL_TIMEOUT:	ret = -EEXEC_TO; break;
		default: ret = -EFAULT; break;
	}

	if (ret < EOK) {
		goto ERR_0;
	}
	SET_BITVAL(_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
    return EOK;

	ERR_0:
	return ret;
}

/* ************************************************************************//**
 * \brief  Start/Stop of the serial receiver
 * \description	 This routine starts or stops the receiver. So a new received frame
 * 				 can be processed at first. 
 *
 * \param   _serHdl [IN]        : serial communication handle
 * \param   _mode                : sets the receiver on/off
 *
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__set_receiver_on_off(serial_access_t *_hdl, const enum receive_mode _mode)
{
	int ret;

	if (_hdl == NULL) {
		return -EINVAL;
	}

	if (_hdl->initialized != SERIAL_INITIALIZED) {
		return -EIO;
	}

	switch(_mode)
	{
		case RECEIVE_MODE_disable:
			CLEAR_BITVAL(_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
			break;
		case RECEIVE_MODE_enable:
			SET_BITVAL(_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
			break;
		default: {
			return -EFAULT;
		}
	}
    return EOK;
}

/* ************************************************************************//**
 * \brief  Configure the destination address of the next receiver frame
 * \description	 This routine reconfigure the destination address and length of
 * 				 the receiver buffer.
 *
 * \param   _serHdl [IN]        : serial communication handle
 * \param   _receive_buffer[out] : a pointer to store the next to data to receive
 * \param   _length         	 : maximal length of the new receive buffer
 *
 * \return	EOK if successful
 * ****************************************************************************/
int serial_access__set_receiver_frame(serial_access_t *_hdl, uint8_t *_receive_buffer, unsigned int _length)
{
	int ret;
	HAL_StatusTypeDef hal_ret;

	if ((_hdl == NULL) || (_receive_buffer == NULL)) {
		return -EINVAL;
	}

	if (_hdl->initialized != SERIAL_INITIALIZED) {
		return -EIO;
	}

	CLEAR_BITVAL(_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
	memset(&_hdl->rx_frame_info, 0, sizeof(struct rx_info));
	HAL_UART_AbortReceive(&_hdl->huart);

	switch (_hdl->mode) {
		case SER_MODE_isr:
			hal_ret = HAL_UART_Receive_IT(&_hdl->huart,_receive_buffer,_length);
			break;

		case SER_MODE_dma:
			hal_ret = HAL_UART_Receive_DMA(&_hdl->huart,_receive_buffer,_length);
			break;

		default:
			ret = -ESTD_FAULT;
			break;
	}

	switch (hal_ret) {
		case HAL_OK     :	ret = EOK; break;
		case HAL_ERROR  :	ret = -EIO; break;
		case HAL_BUSY   :	ret = -EBUSY; break;
		case HAL_TIMEOUT:	ret = -EEXEC_TO; break;
		default: ret = -EFAULT; break;
	}

	if (ret < EOK) {
		goto ERR_0;
	}

	SET_BITVAL(_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
	return 0;

	ERR_0:
	return ret;
}

/* *******************************************************************
 * static function definitions
 * ******************************************************************/

static void serial_access__uart_event(IRQn_Type _isr_vector, unsigned int _vector, void *_arg)
{
	int ret;
	unsigned int remaining_bytes;
	enum lib_ser_isr_type isr_type;
	serial_access_t *serHdl = (serial_access_t *)_arg;
	if (_vector >= LIB_SER_ISR_TYPE_CNT) {
		return;
	}

	isr_type = (enum lib_ser_isr_type)_vector;
	switch (isr_type)
	{
		case LIB_SER_ISR_TYPE_uart:

			if(__HAL_UART_GET_FLAG((&(serHdl->huart)), UART_FLAG_IDLE)) {
				volatile uint32_t tmp;
				tmp = serHdl->huart.Instance->DR;
				lib_timer__stop((serHdl->rx_timeout_hdl));
				switch (serHdl->mode)
				{
					case SER_MODE_isr:
						serHdl->rx_frame_info.number_received_data++;
						break;

					case SER_MODE_dma:
						serHdl->rx_frame_info.number_received_data = serHdl->huart.RxXferSize - serHdl->hdma_rx.Instance->NDTR;
						break;
				}

				remaining_bytes = serHdl->huart.RxXferSize - serHdl->hdma_rx.Instance->NDTR;
				__HAL_UART_CLEAR_FLAG((&(serHdl->huart)),UART_FLAG_IDLE);
				lib_timer__start((serHdl->rx_timeout_hdl),serHdl->rx_timeout_interval);
			}
			else {
				HAL_UART_IRQHandler(&(serHdl->huart));
			}
			break;
		case LIB_SER_ISR_TYPE_dma_tx:
			HAL_DMA_IRQHandler(&(serHdl->hdma_tx));
			break;
		case LIB_SER_ISR_TYPE_dma_rx:
			serHdl->rx_frame_info.number_received_data = serHdl->huart.RxXferSize - serHdl->hdma_rx.Instance->NDTR;
			HAL_DMA_IRQHandler(&(serHdl->hdma_rx));
			break;
	}

}

static void serial_access__uart_rx_timeout(timer_hdl_t _hdl, void* _arg)
{
	enum receive_mode receive_mode_ret;
	serial_access_t *serHdl = (serial_access_t *)_arg;

	if (serHdl->rx_complete_handler != NULL) {
		receive_mode_ret = (*serHdl->rx_complete_handler)(serHdl, RECEIVE_EVENT_timeout, &serHdl->rx_frame_info);
	}
	else {
		receive_mode_ret = RECEIVE_MODE_enable;
	}

	switch (receive_mode_ret)
	{
		case RECEIVE_MODE_disable:
			CLEAR_BITVAL(serHdl->huart.Instance->CR1,USART_CR1_RE_Pos);
			break;
		case RECEIVE_MODE_enable:
		default:
			break;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	 serial_access_t *serHdl = (serial_access_t*)GET_CONTAINER_OF(huart,struct serial_access_handle,huart);
	 if (serHdl->tx_complete_handler != NULL) {
	 	(*serHdl->tx_complete_handler)(serHdl);
	 }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 enum receive_mode receive_mode_ret;
	 serial_access_t *serHdl = (serial_access_t*)GET_CONTAINER_OF(huart,struct serial_access_handle,huart);

	 if (serHdl->rx_complete_handler != NULL) {
	 	receive_mode_ret = (*serHdl->rx_complete_handler)(serHdl, RECEIVE_EVENT_overflow, &serHdl->rx_frame_info);
	 }
	 else {
	 	receive_mode_ret = RECEIVE_MODE_enable;
	 }

	 switch (receive_mode_ret)
	 {
	 	case RECEIVE_MODE_disable:
	 		CLEAR_BITVAL(serHdl->huart.Instance->CR1,USART_CR1_RE_Pos);
	 		return;
	 	case RECEIVE_MODE_enable:
	 		HAL_UART_Receive_DMA(&serHdl->huart, serHdl->huart.pRxBuffPtr, serHdl->huart.RxXferSize);
	 		break;
	 }

}


