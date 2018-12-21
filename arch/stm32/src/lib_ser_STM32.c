/* ****************************************************************************************************
 * lib_ser.c within the following project: lib_ser
 *
 *  compiler:   GNU Tools ARM LINUX
 *  target:     armv6
 *  author:	    Tom
 * ****************************************************************************************************/

/* ****************************************************************************************************/

/*
 *	******************************* change log *******************************
 *  date			user			comment
 * 	23 Mai 2018		Tom			- creation of lib_ser.c
 *
 */

/* *******************************************************************
 * includes
 * ******************************************************************/

/* c-runtime */
#include <stdlib.h>
#include <string.h>

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
#include <lib_log.h>
#include <lib_isr.h>
#include <lib_timer.h>

/* project */
#include "lib_ser_init_itf.h"
#include "lib_ser.h"

/* *******************************************************************
 * defines
 * ******************************************************************/
#define M_LIB_SER__MODULE_ID			"LIB_SER"
#define M_LIB_SER__HDL_INITIALIZED		0x7FAB7FAB
#define M_LIB_SER__WAIT_INFINITE		0xFFFF


#define M_SER_DMA_CHANNEL_MAP_DESCRIPTOR 															   \
{																									   \
	M_SER_DEVICE_CFG_UART_DMA(USART1, DMA2 ,Stream7, DMA_CHANNEL_4, Stream2,DMA_CHANNEL_4), \
	M_SER_DEVICE_CFG_UART_DMA(USART2, DMA1 ,Stream6, DMA_CHANNEL_4, Stream5,DMA_CHANNEL_4), \
	M_SER_DEVICE_CFG_UART_DMA(USART6, DMA2 ,Stream6, DMA_CHANNEL_5, Stream1,DMA_CHANNEL_5), \
}

//#define M_SER_DMA_CHANNEL_MAP_DESCRIPTOR 															   \
//{																									   \
//	M_SER_DEVICE_CFG_UART_DMA(USART1, DMA2 ,Stream7, DMA_CHANNEL_4, Stream2,DMA_CHANNEL_4), \
//	M_SER_DEVICE_CFG_UART_ISR(USART2), \
//	M_SER_DEVICE_CFG_UART_DMA(USART6, DMA2 ,Stream6, DMA_CHANNEL_5, Stream1,DMA_CHANNEL_5), \
//}




/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/
enum ser_mode {
	SER_MODE_isr,
	SER_MODE_dma
};

struct ser_hdl_attr
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
	ser_hdl_t *own_loc;
	enum ser_mode mode;
	IRQn_Type huart_isr_type;
	IRQn_Type hdma_tx_isr_type;
	IRQn_Type hdma_rx_isr_type;
	ser_tx_complete_handler_t *tx_complete_handler;
	ser_rx_complete_handler_t *rx_complete_handler;
	unsigned int initialized;
	struct rx_info rx_frame_info;
};


/* *******************************************************************
 * static data
 * ******************************************************************/
static struct ser_device_cfg __attribute__ ((section(".text"))) s_uart_dma_map[] = M_SER_DMA_CHANNEL_MAP_DESCRIPTOR;
static const char* __attribute__ ((section(".text"))) s_ser_mode_name[] = {"SER_MODE_isr", "SER_MODE_dma"};


/* *******************************************************************
 * static function declarations
 * ******************************************************************/
static void lib_ser__uart_event(IRQn_Type _isr_vector, unsigned int _vector, void *_arg);
static void lib_ser__uart_rx_timeout(timer_hdl_t _hdl, void* _arg);
static inline struct ser_device_cfg* lib_ser__get_dma_channel_map(USART_TypeDef *_uart);


/* *******************************************************************
 * function definition
 * ******************************************************************/

/* ************************************************************************//**
 * \brief  Initialization of the serial driver
 *
 * \param   _ser_hdl* [OUT]      : serial communication handle
 * \param   _baudrate            : serial baudrate
 * \param   _format              : serial port parameter setting
 * \return	EOK if successful
 * ****************************************************************************/
int lib_ser__open(ser_hdl_t *_ser_hdl, void *_port, unsigned int _baudrate, enum data_format _format)
{
	int line, ret;
	struct ser_hdl_attr *ser_hdl;
	struct ser_device_cfg* device_cfg_desc;
	GPIO_InitTypeDef GPIO_InitStruct;

	if (_ser_hdl == NULL) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	ser_hdl = (struct ser_hdl_attr*)pvPortMalloc(sizeof(struct ser_hdl_attr));
	if(ser_hdl == NULL) {
		line = __LINE__;
		ret = -ESTD_NOMEM;
		goto ERR_1;
	}

	switch (_format)
	{
		case DATA_FORMAT_8_NONE_2:
		{	/* 8 data bits, no   parity, 2 stop bit   */
			ser_hdl->huart.Init.WordLength = UART_WORDLENGTH_8B;
			ser_hdl->huart.Init.StopBits = UART_STOPBITS_2;
			ser_hdl->huart.Init.Parity = UART_PARITY_NONE;
			break;
		}
		case DATA_FORMAT_8_EVEN_1:
		{	/* 8 data bits, even parity, 1 stop bit   */
			ser_hdl->huart.Init.WordLength = UART_WORDLENGTH_9B;
			ser_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			ser_hdl->huart.Init.Parity = UART_PARITY_EVEN;
			break;
		}
		case DATA_FORMAT_8_ODD_1:
		{	/* 8 data bits, odd  parity, 1 stop bit   */
			ser_hdl->huart.Init.WordLength = UART_WORDLENGTH_9B;
			ser_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			ser_hdl->huart.Init.Parity = UART_PARITY_ODD;
			break;
		}
		case DATA_FORMAT_8_NONE_1:
		{	/* 8 data bits, no   parity, 1 stop bit   */
			ser_hdl->huart.Init.WordLength = UART_WORDLENGTH_8B;
			ser_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			ser_hdl->huart.Init.Parity = UART_PARITY_NONE;
			break;
		}
		case DATA_FORMAT_7_EVEN_1:
		{	/* 7 data bits, even parity, 1 stop bit */
			ser_hdl->huart.Init.WordLength = UART_WORDLENGTH_8B;
			ser_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			ser_hdl->huart.Init.Parity = UART_PARITY_EVEN;
			break;
		}
		case DATA_FORMAT_7_ODD_1:
		{	/* 7 data bits, odd  parity, 1 stop bit */

			ser_hdl->huart.Init.WordLength = UART_WORDLENGTH_8B;
			ser_hdl->huart.Init.StopBits = UART_STOPBITS_1;
			ser_hdl->huart.Init.Parity = UART_PARITY_ODD;
			break;
		}
		default :
		{
			line = __LINE__;
			ret = -ESTD_INVAL;
			goto ERR_1;
		}
	}

	device_cfg_desc = lib_ser__get_dma_channel_map(_port);
	if (device_cfg_desc == NULL) {
		line = __LINE__;
		ret = -ESTD_NODEV;
		goto ERR_1;
	}

	ret = lib_isr__attach (&(ser_hdl->isr_uart_hdl), device_cfg_desc->huart_isr_type, &lib_ser__uart_event,  (void*)ser_hdl);
	if (ret < EOK) {
		line = __LINE__;
	 	goto ERR_1;
	}

	ret = lib_timer__open(&(ser_hdl->rx_timeout_hdl), ser_hdl, &lib_ser__uart_rx_timeout);
	if (ret < EOK) {
		line = __LINE__;
		goto ERR_2;
	}

	if ((device_cfg_desc->rx_dma_stream == NULL) || (device_cfg_desc->rx_dma_stream == NULL))
	{
		ser_hdl->huart_isr_type = device_cfg_desc->huart_isr_type;
		ser_hdl->isr_dma_tx_hdl = NULL;
		ser_hdl->isr_dma_rx_hdl = NULL;
		ser_hdl->mode = SER_MODE_isr;
 	}
 	else {

 		if(device_cfg_desc->dma == DMA1) {
 			__DMA1_CLK_ENABLE();
 		}
 		else if (device_cfg_desc->dma == DMA2) {
 			__DMA2_CLK_ENABLE();
 		}
 		else {
 			line = __LINE__;
 		 	ret = -ESTD_NODEV;
 		 	goto ERR_3;
 		}

 	    /* USART_TX DMA Init */
 		ser_hdl->hdma_tx.Instance = (DMA_Stream_TypeDef*)device_cfg_desc->tx_dma_stream; //DMA1_Stream6;
 	    ser_hdl->hdma_tx.Init.Channel = device_cfg_desc->tx_dma_channel; //DMA_CHANNEL_4;
 	    ser_hdl->hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
 	    ser_hdl->hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
 	    ser_hdl->hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
 	    ser_hdl->hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
 	    ser_hdl->hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
 	    ser_hdl->hdma_tx.Init.Mode = DMA_NORMAL;
 	    ser_hdl->hdma_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
 	    ser_hdl->hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
 	    ret = HAL_DMA_Init(&ser_hdl->hdma_tx);
 	    if (ret != HAL_OK) {
 	    	line = __LINE__;
 	   		ret = -EHAL_ERROR;
 	   		goto ERR_DMA_TX;
 	    }
 	   __HAL_LINKDMA(&(ser_hdl->huart),hdmatx,(ser_hdl->hdma_tx));

 	   /* USART_RX DMA Init */
 	   ser_hdl->hdma_rx.Instance = (DMA_Stream_TypeDef*)device_cfg_desc->rx_dma_stream;
 	   ser_hdl->hdma_rx.Init.Channel = device_cfg_desc->rx_dma_channel;
 	   ser_hdl->hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
 	   ser_hdl->hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
 	   ser_hdl->hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
	   ser_hdl->hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	   ser_hdl->hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	   ser_hdl->hdma_rx.Init.Mode = DMA_NORMAL;
	   ser_hdl->hdma_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
	   ser_hdl->hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	   ret = HAL_DMA_Init(&ser_hdl->hdma_rx);
	   if (ret != HAL_OK) {
		   line = __LINE__;
		   ret = -EHAL_ERROR;
		   goto ERR_DMA_RX;
	   }

	   __HAL_LINKDMA(&(ser_hdl->huart),hdmarx,(ser_hdl->hdma_rx));

	   ret = lib_isr__attach (&(ser_hdl->isr_dma_tx_hdl), device_cfg_desc->hdma_tx_isr_type, &lib_ser__uart_event,  (void*)ser_hdl);
	   if (ret < EOK) {
		   line = __LINE__;
		   goto ERR_DMA_ISR_TX;
	   }

	   ret = lib_isr__attach (&(ser_hdl->isr_dma_rx_hdl), device_cfg_desc->hdma_rx_isr_type, &lib_ser__uart_event,  (void*)ser_hdl);
	   if (ret < EOK) {
		   line = __LINE__;
		   goto ERR_DMA_ISR_RX;
	   }

	   ser_hdl->hdma_tx_isr_type = device_cfg_desc->hdma_tx_isr_type;
	   ser_hdl->hdma_rx_isr_type = device_cfg_desc->hdma_rx_isr_type;
	   ser_hdl->huart_isr_type = device_cfg_desc->huart_isr_type;
	   ser_hdl->mode = SER_MODE_dma;
 	}

	__HAL_RCC_USART2_CLK_ENABLE();

	ser_hdl->huart.Instance = _port;
	ser_hdl->huart.Init.BaudRate = _baudrate;
	ser_hdl->huart.Init.Mode = UART_MODE_TX_RX;
	ser_hdl->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	ser_hdl->huart.Init.OverSampling = UART_OVERSAMPLING_16;
	ret = HAL_UART_Init(&(ser_hdl->huart));
	if (ret != HAL_OK) {
		line = __LINE__;
		ret = -EHAL_ERROR;
		if (ser_hdl->mode == SER_MODE_dma)
			goto ERR_DMA_UART_INIT;
		else
			goto ERR_2;
	}

	SET_BIT(ser_hdl->huart.Instance->CR1, USART_CR1_IDLEIE);


	ser_hdl->initialized = M_LIB_SER__HDL_INITIALIZED;
	ser_hdl->own_loc = _ser_hdl;
	*_ser_hdl = ser_hdl;
	msg (LOG_LEVEL_info, M_LIB_SER__MODULE_ID, "%s(): serial driver initialized at %s mode\n",__func__, s_ser_mode_name[ser_hdl->mode]);
    return EOK;

    ERR_DMA_UART_INIT:
	lib_isr__detach(&(ser_hdl->isr_dma_rx_hdl));

    ERR_DMA_ISR_RX:
	lib_isr__detach(&(ser_hdl->isr_dma_tx_hdl));

    ERR_DMA_ISR_TX:
	HAL_DMA_DeInit(&ser_hdl->hdma_rx);

    ERR_DMA_RX:
	HAL_DMA_DeInit(&ser_hdl->hdma_tx);

    ERR_DMA_TX:
	if(device_cfg_desc->dma == DMA1)
		__DMA1_CLK_DISABLE();
	else if (device_cfg_desc->dma == DMA2)
	 	__DMA2_CLK_DISABLE();

	ERR_3:
	lib_timer__close(&(ser_hdl->rx_timeout_hdl));

	ERR_2:
	__HAL_RCC_USART2_CLK_DISABLE();
	lib_isr__detach(&(ser_hdl->isr_uart_hdl));

    ERR_1:
	vPortFree(ser_hdl);

    ERR_0:
    msg (LOG_LEVEL_error, M_LIB_SER__MODULE_ID, "%s(): failed with retval %i (line %u)\n",__func__, ret, line );
    return ret;
}

/* ************************************************************************//**
 * \brief  Cleanup of the serial driver

 * \param   _ser_hdl* [OUT]      : serial communication handle
 * \return	EOK if successful
 * ****************************************************************************/
int lib_ser__cleanup(ser_hdl_t *_ser_hdl)
{
	int line, ret;
	HAL_StatusTypeDef hal_ret;

	if (_ser_hdl == NULL) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	if (*_ser_hdl == NULL) {
		line = __LINE__;
		ret = -EEXEC_NOINIT;
		goto ERR_0;
	}

	if ((*_ser_hdl)->initialized != M_LIB_SER__HDL_INITIALIZED) {
		line = __LINE__;
		ret = -EEXEC_NOINIT;
		goto ERR_0;
	}

	if (((*_ser_hdl)->mode) == SER_MODE_dma) {

		ret = lib_isr__detach(&((*_ser_hdl)->isr_dma_rx_hdl));
		ret |= lib_isr__detach(&((*_ser_hdl)->isr_dma_tx_hdl));
		hal_ret = HAL_DMA_DeInit(&((*_ser_hdl)->hdma_rx));
		if (hal_ret != HAL_OK) {
			ret |= -EHAL_ERROR;
		}

		hal_ret = HAL_DMA_DeInit(&((*_ser_hdl)->hdma_tx));
		if (hal_ret != HAL_OK) {
			ret |= -EHAL_ERROR;
		}

		__DMA1_CLK_DISABLE();
		__DMA2_CLK_DISABLE();
	}

	ret |= lib_timer__close(&((*_ser_hdl)->rx_timeout_hdl));
	__HAL_RCC_USART2_CLK_DISABLE();
	lib_isr__detach(&((*_ser_hdl)->isr_uart_hdl));
	vPortFree(*_ser_hdl);
	*_ser_hdl = NULL;

    return EOK;

    ERR_0:
	msg (LOG_LEVEL_error, M_LIB_SER__MODULE_ID, "%s(): failed with retval %i (line %u)\n",__func__, ret, line );
	return ret;

}

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
int lib_ser__send(ser_hdl_t _ser_hdl, uint8_t *_data_send, unsigned int _length, ser_tx_complete_handler_t _tx_complete_handler)
{
	HAL_StatusTypeDef hal_ret;
	int line, ret;

	if ((_ser_hdl == NULL) || (_data_send == NULL)) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	if (_ser_hdl->initialized != M_LIB_SER__HDL_INITIALIZED) {
		line = __LINE__;
		ret = -EEXEC_NOINIT;
		goto ERR_0;
	}

	_ser_hdl->tx_complete_handler = _tx_complete_handler;

	switch (_ser_hdl->mode) {

		case SER_MODE_isr:
			hal_ret = HAL_UART_Transmit_IT(&_ser_hdl->huart, _data_send, _length);
			break;
		case SER_MODE_dma:
			if (_length == 1) {
				hal_ret = HAL_UART_Transmit_IT(&_ser_hdl->huart, _data_send, _length);	
			}
			else {
				hal_ret = HAL_UART_Transmit_DMA(&_ser_hdl->huart, _data_send, _length);
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
		line = __LINE__;
		goto ERR_0;
	}

    return EOK;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_SER__MODULE_ID, "%s(): failed with retval %i (line %u)",__func__, ret, line );
	return ret;
}

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
int lib_ser__prepare_receiver(ser_hdl_t _ser_hdl, uint8_t *_receive_buffer, unsigned int _length, unsigned int _timeout, ser_rx_complete_handler_t *_rx_complete_handler)
{
	int ret, line;
	HAL_StatusTypeDef hal_ret;

	if (_ser_hdl == NULL) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	if (_timeout == 0) {
		line = __LINE__;
		ret = -ESTD_INVAL;
		goto ERR_0;
	}

	if ((_ser_hdl == NULL) || (_receive_buffer == NULL)) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	if (_ser_hdl->initialized != M_LIB_SER__HDL_INITIALIZED) {
		line = __LINE__;
		ret = -EEXEC_NOINIT;
		goto ERR_0;
	}

	CLEAR_BITVAL(_ser_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
	memset(&_ser_hdl->rx_frame_info, 0, sizeof(struct rx_info));
	_ser_hdl->rx_complete_handler = _rx_complete_handler;
	_ser_hdl->rx_timeout_interval = _timeout;
	HAL_UART_AbortReceive(&_ser_hdl->huart);

	switch (_ser_hdl->mode) {
		case SER_MODE_isr:
			hal_ret = HAL_UART_Receive_IT(&_ser_hdl->huart,_receive_buffer,_length);
			break;

		case SER_MODE_dma:
			hal_ret = HAL_UART_Receive_DMA(&_ser_hdl->huart,_receive_buffer,_length);
			break;

		default:
			ret = -ESTD_FAULT;
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
		line = __LINE__;
		goto ERR_0;
	}
	SET_BITVAL(_ser_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
    return EOK;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_SER__MODULE_ID, "%s(): failed with retval %i (line %u)",__func__, ret, line );
	return ret;

}

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
int lib_ser__set_receiver_on_off(ser_hdl_t _ser_hdl, const enum receive_mode _mode)
{
	int ret, line;

	if (_ser_hdl == NULL) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	if (_ser_hdl->initialized != M_LIB_SER__HDL_INITIALIZED) {
		line = __LINE__;
		ret = -EEXEC_NOINIT;
		goto ERR_0;
	}

	if (_ser_hdl->initialized != M_LIB_SER__HDL_INITIALIZED) {
		return -EEXEC_NOINIT;
	}

	switch(_mode)
	{
		case RECEIVE_MODE_disable:
			CLEAR_BITVAL(_ser_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
			break;
		case RECEIVE_MODE_enable:
			SET_BITVAL(_ser_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
			break;
		default: {
			line = __LINE__;
			ret = -ESTD_FAULT;
			goto ERR_0;
		}
	}
    return EOK;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_SER__MODULE_ID, "%s(): failed with retval %i (line %u)",__func__, ret, line );
	return ret;
}

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
int lib_ser__set_receiver_frame(ser_hdl_t _ser_hdl, uint8_t *_receive_buffer, unsigned int _length)
{
	int ret, line;
	HAL_StatusTypeDef hal_ret;

	if (_ser_hdl == NULL) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	if ((_ser_hdl == NULL) || (_receive_buffer == NULL)) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	if (_ser_hdl->initialized != M_LIB_SER__HDL_INITIALIZED) {
		line = __LINE__;
		ret = -EEXEC_NOINIT;
		goto ERR_0;
	}

	CLEAR_BITVAL(_ser_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
	memset(&_ser_hdl->rx_frame_info, 0, sizeof(struct rx_info));
	HAL_UART_AbortReceive(&_ser_hdl->huart);

	switch (_ser_hdl->mode) {
		case SER_MODE_isr:
			hal_ret = HAL_UART_Receive_IT(&_ser_hdl->huart,_receive_buffer,_length);
			break;

		case SER_MODE_dma:
			hal_ret = HAL_UART_Receive_DMA(&_ser_hdl->huart,_receive_buffer,_length);
			break;

		default:
			ret = -ESTD_FAULT;
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
		line = __LINE__;
		goto ERR_0;
	}

	SET_BITVAL(_ser_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
	return EOK;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_SER__MODULE_ID, "%s(): failed with retval %i (line %u)",__func__, ret, line );
	return ret;
}


/* *******************************************************************
 * static function definitions
 * ******************************************************************/

static void lib_ser__uart_event(IRQn_Type _isr_vector, unsigned int _vector, void *_arg)
{
	int ret;
	unsigned int remaining_bytes;
	enum lib_ser_isr_type isr_type;
	struct ser_hdl_attr *ser_hdl = (struct ser_hdl_attr *)_arg;
	if (_vector >= LIB_SER_ISR_TYPE_CNT) {
		return;
	}

	isr_type = (enum lib_ser_isr_type)_vector;
	switch (isr_type)
	{
		case LIB_SER_ISR_TYPE_uart:

			if(__HAL_UART_GET_FLAG((&(ser_hdl->huart)), UART_FLAG_IDLE)) {
				volatile uint32_t tmp;
				tmp = ser_hdl->huart.Instance->DR;
				lib_timer__stop((ser_hdl->rx_timeout_hdl));
				switch (ser_hdl->mode)
				{
					case SER_MODE_isr:
						ser_hdl->rx_frame_info.number_received_data++;
						break;

					case SER_MODE_dma:
						ser_hdl->rx_frame_info.number_received_data = ser_hdl->huart.RxXferSize - ser_hdl->hdma_rx.Instance->NDTR;
						break;
				}

				remaining_bytes = ser_hdl->huart.RxXferSize - ser_hdl->hdma_rx.Instance->NDTR;
				__HAL_UART_CLEAR_FLAG((&(ser_hdl->huart)),UART_FLAG_IDLE);
				lib_timer__start((ser_hdl->rx_timeout_hdl),ser_hdl->rx_timeout_interval);
			//	__HAL_DMA_DISABLE(&(ser_hdl->hdma_rx));
			}
			else {
				HAL_UART_IRQHandler(&(ser_hdl->huart));
			}
			break;
		case LIB_SER_ISR_TYPE_dma_tx:
			HAL_DMA_IRQHandler(&(ser_hdl->hdma_tx));
			break;
		case LIB_SER_ISR_TYPE_dma_rx:
			HAL_DMA_IRQHandler(&(ser_hdl->hdma_rx));
			break;
	}

}

static void lib_ser__uart_rx_timeout(timer_hdl_t _hdl, void* _arg)
{
	enum receive_mode receive_mode_ret;
	struct ser_hdl_attr *ser_hdl = (struct ser_hdl_attr *)_arg;

	if (ser_hdl->rx_complete_handler != NULL) {
		receive_mode_ret = (*ser_hdl->rx_complete_handler)(ser_hdl->own_loc, RECEIVE_EVENT_timeout, &ser_hdl->rx_frame_info);
	}
	else {
		receive_mode_ret = RECEIVE_MODE_enable;
	}

//	memset(&ser_hdl->rx_frame_info,0,sizeof(struct rx_info));

	switch (receive_mode_ret)
	{
		case RECEIVE_MODE_disable:
			CLEAR_BITVAL(ser_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
			break;
		case RECEIVE_MODE_enable:
		default:
			break;
	}


}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	struct ser_hdl_attr *ser_hdl = (struct ser_hdl_attr *)GET_CONTAINER_OF(huart,struct ser_hdl_attr,huart);
	if (ser_hdl->tx_complete_handler != NULL) {
		(*ser_hdl->tx_complete_handler)(ser_hdl->own_loc);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	enum receive_mode receive_mode_ret;
	struct ser_hdl_attr *ser_hdl = (struct ser_hdl_attr *)GET_CONTAINER_OF(huart,struct ser_hdl_attr,huart);

	if (ser_hdl->rx_complete_handler != NULL) {
		receive_mode_ret = (*ser_hdl->rx_complete_handler)(ser_hdl->own_loc, RECEIVE_EVENT_overflow, &ser_hdl->rx_frame_info);
	}
	else {
		receive_mode_ret = RECEIVE_MODE_enable;
	}

	//memset(&ser_hdl->rx_frame_info,0,sizeof(struct rx_info));

	switch (receive_mode_ret)
	{
		case RECEIVE_MODE_disable:
			CLEAR_BITVAL(ser_hdl->huart.Instance->CR1,USART_CR1_RE_Pos);
			return;
		case RECEIVE_MODE_enable:
			HAL_UART_Receive_DMA(&ser_hdl->huart, ser_hdl->huart.pRxBuffPtr, ser_hdl->huart.RxXferSize);
			break;
	}

}


static inline struct ser_device_cfg* lib_ser__get_dma_channel_map(USART_TypeDef *_uart)
{
	int iter;
	for(iter =0; iter < sizeof(s_uart_dma_map)/sizeof(*s_uart_dma_map); iter++) {
		if (s_uart_dma_map[iter].uart_device == _uart) {
			return &s_uart_dma_map[iter];
		}
	}
	return NULL;
}

