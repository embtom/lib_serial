/* ****************************************************************************************************
 * lib_ser.h within the following project: lib_ser
 *
 *  compiler:   GNU Tools ARM LINUX
 *  target:     armv6
 *  author:	    Tom
 * ****************************************************************************************************/

/* ****************************************************************************************************/

/*
 *	******************************* change log *******************************
 *  date			user			comment
 * 	23 Mai 2018		Tom			- creation of lib_ser.h
 *
 */


#ifndef _LIB_SER_H_
#define _LIB_SER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* *******************************************************************
 * includes
 * ******************************************************************/

/* c-runtime */
#include <stdint.h>
#include <stdlib.h>

/* system */

/* frame */
//#include <lib_convention__types.h>


/* *******************************************************************
 * defines
 * ******************************************************************/

#define BAUDRATE_115200     115200
#define BAUDRATE_57600      57600
#define BAUDRATE_38400      38400
#define BAUDRATE_19200      19200
#define BAUDRATE_14400      14400
#define BAUDRATE_9600       9600
#define BAUDRATE_4800       4800

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/

enum receive_mode
{
	RECEIVE_MODE_disable,
	RECEIVE_MODE_enable
};

enum data_format
{
	DATA_FORMAT_8_NONE_2,    //8 data bits, no   parity, 2 stop bit
	DATA_FORMAT_8_EVEN_1,	//8 data bits, even parity, 1 stop bit
	DATA_FORMAT_8_ODD_1,	//8 data bits, odd  parity, 1 stop bit
	DATA_FORMAT_8_NONE_1, 	//8 data bits, no   parity, 1 stop bit
	DATA_FORMAT_7_EVEN_1,    //7 data bits, even parity, 1 stop bit
	DATA_FORMAT_7_ODD_1		//7 data bist, odd  parity, 1 stop bit
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


typedef struct ser_hdl_attr *ser_hdl_t;

typedef void (ser_tx_complete_handler_t)(ser_hdl_t *_ser_hdl);
typedef enum receive_mode (ser_rx_complete_handler_t)(ser_hdl_t *_ser_hdl, enum receive_event, struct rx_info const * const);

/* *******************************************************************
 * function declarations
 * ******************************************************************/

/* ************************************************************************//**
 * \brief  Initialization of the serial driver
 *
 * \param   _ser_hdl* [OUT]      : serial communication handle
 * \param   _baudrate            : serial baudrate
 * \param   _format              : serial port parameter setting
 * \return	EOK if successful
 * ****************************************************************************/
int lib_ser__open(ser_hdl_t *_ser_hdl, void *_port, unsigned int _baudrate, enum data_format _format);

/* ************************************************************************//**
 * \brief  Cleanup of the serial driver

 * \param   _ser_hdl* [OUT]      : serial communication handle
 * \return	EOK if successful
 * ****************************************************************************/
int lib_ser__cleanup(ser_hdl_t *_ser_hdl);

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
int lib_ser__send(ser_hdl_t _ser_hdl, uint8_t *_data_send, unsigned int _length, ser_tx_complete_handler_t _tx_complete_handler);

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
int lib_ser__prepare_receiver(ser_hdl_t _ser_hdl, uint8_t *_receive_buffer, unsigned int _length, unsigned int _timeout, ser_rx_complete_handler_t *_rx_complete_handler);

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
int lib_ser__set_receiver_on_off(ser_hdl_t _ser_hdl, const enum receive_mode _mode);

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
int lib_ser__set_receiver_frame(ser_hdl_t _ser_hdl, uint8_t *_receive_buffer, unsigned int _length);

#ifdef __cplusplus
}
#endif

#endif  /* _LIB_SER_EOS_H_ */
