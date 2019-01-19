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

/*c-runtime*/
#include <stdlib.h>
#include <errno.h> /* ERROR Number Definitions           */
/*frame*/
#include <FreeRTOS.h>
#include <lib_thread.h>
/*project*/
#include <lib_serial.h>
#include <lib_serial_types.h>
#include <serial_access_stm32.h>

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/
// /* the handle definition for this implementation */
struct lib_serial_handle{
	const USART_TypeDef *uartDevice;
	serial_access_t serialAccess;
	sem_hdl_t txSem;
	mutex_hdl_t	txMtx;
	sem_hdl_t rxSem;
	unsigned int rxLen;
	unsigned int initialized;
};

/* *******************************************************************
 * static function declarations
 * ******************************************************************/
static void lib_serial__tx_finished(serial_access_t *_hdl);
static enum receive_mode lib_serial__rx_finished(serial_access_t *_hdl, enum receive_event _type , struct rx_info const * const _info);

/* ****************************************************************************
 * Global Functions
 * ****************************************************************************/
lib_serial_hdl lib_serial_create_stm32(struct port_config *port)
{
	lib_serial_hdl hdl = NULL;
	int ret;

	if (port == NULL) {
		return NULL;
	}

	hdl = (lib_serial_hdl)pvPortMalloc(sizeof(struct lib_serial_handle));
	hdl->uartDevice = port->uart_device;

	ret = serial_access__config(&hdl->serialAccess,port);
	if (ret < 0) {
		vPortFree(hdl);
		return NULL;
	}
	return hdl;
}

int lib_serial_destroy(lib_serial_hdl *hdl)
{	
	int ret;
	/* hdl param check */
	if ((hdl == NULL) || (*hdl == NULL)) {
		return -EINVAL;
	}
	if((*hdl)->serialAccess.initialized == SERIAL_INITIALIZED) {
		serial_access__close(&((*hdl)->serialAccess));
	}
	ret = serial_access__cleanup(&((*hdl)->serialAccess));

	/* free allocated space and declare handle as invalid */
	vPortFree(*hdl);
	*hdl = NULL;
	return ret;
}


int lib_serial_open(lib_serial_hdl hdl, enum baudrate baudrate, enum data_format format)
{
	int ret;
	if (hdl == NULL) {
		return -EINVAL;
	}
	
	if (hdl->initialized == SERIAL_INITIALIZED) {
		return -EAGAIN;
	}	

	ret = serial_access__open(&hdl->serialAccess, hdl->uartDevice, baudrate, format);
	if (ret <0) {
		goto ERR_OPEN;
	}

	ret = lib_thread__sem_init(&hdl->txSem, 0);
	if (ret < 0) {
		goto ERR_TX_SEM;
	}

	ret = lib_thread__mutex_init(&hdl->txMtx);
	if (ret < 0) {
		goto ERR_TX_MTX;
	}

	ret = lib_thread__sem_init(&hdl->rxSem, 0);
	if (ret < 0) {
		goto ERR_RX_SEM;
	}

    hdl->initialized = SERIAL_INITIALIZED;
	return ret;

	ERR_RX_SEM:
	lib_thread__mutex_destroy(&hdl->txMtx);

	ERR_TX_MTX:
	lib_thread__sem_destroy(&hdl->txSem);

	ERR_TX_SEM:
	serial_access__close(&hdl->serialAccess);
	ERR_OPEN:
	return -EIO;
}


int lib_serial_close(lib_serial_hdl _hdl)
{	int ret;
	
	if (_hdl->initialized != SERIAL_INITIALIZED) {
		return -EINVAL;
	}	

	ret = serial_access__close(&_hdl->serialAccess);
	return ret;
}

int lib_serial_write(lib_serial_hdl hdl, const uint8_t * const data, int len)
{
	int ret;

	if (hdl->initialized != SERIAL_INITIALIZED) {
		return -EINVAL;
	}	

	ret = lib_thread__mutex_lock(hdl->txMtx);
	if (ret < 0) {
		goto ERR_MTXLOCK;
	}

	ret = serial_access__send(&hdl->serialAccess,data, len, &lib_serial__tx_finished);
	if (ret < 0) {
		goto ERR_SEND;
	}

	ret = lib_thread__sem_wait(hdl->txSem);
	if (ret < 0) {
		goto ERR_SEND;
	}

	lib_thread__mutex_unlock(hdl->txMtx);
	return ret;

	ERR_SEND:
	lib_thread__mutex_unlock(hdl->txMtx);
	ERR_MTXLOCK:
	return -EIO;
}

int lib_serial_read(lib_serial_hdl hdl, uint8_t * const data, int maxlen, uint32_t frameTimeout)
{
	int ret;
	if ((hdl == NULL) || (data == NULL) || (maxlen == 0)) {
		return -EINVAL;
	}

	if (hdl->initialized != SERIAL_INITIALIZED) {
		return -EINVAL;
	}	

	ret = serial_access__prepare_receiver(&hdl->serialAccess, data, maxlen, frameTimeout, &lib_serial__rx_finished);
	if (ret < 0) {
		return ret;
	}

	ret = lib_thread__sem_wait(hdl->rxSem);
	if (ret < 0) {
		return ret;
	}

	return hdl->rxLen;
}

/* *******************************************************************
 * static function definitions
 * ******************************************************************/
static void lib_serial__tx_finished(serial_access_t *_hdl)
{
	lib_serial_hdl hdl = (lib_serial_hdl)GET_CONTAINER_OF(_hdl, struct lib_serial_handle, serialAccess);
	lib_thread__sem_post(hdl->txSem);
}

static enum receive_mode lib_serial__rx_finished(serial_access_t *_hdl, enum receive_event _type , struct rx_info const * const _info)
{
	lib_serial_hdl hdl = (lib_serial_hdl)GET_CONTAINER_OF(_hdl, struct lib_serial_handle, serialAccess);
	hdl->rxLen = _info->number_received_data;
	lib_thread__sem_post(hdl->rxSem);
	return RECEIVE_MODE_disable;
}
