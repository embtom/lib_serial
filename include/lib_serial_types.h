/*
 *  This file is part of the HanoverFlipDot project
 *  (https://github.com/EmbeddedAl/HanoverFlipDot).
 *
 *  HanoverFlipDot is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  HanoverFlipDot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with HanoverFlipDot. If not, see <https://www.gnu.org/licenses/>.
 *
 *  Creation: 2018-12-26, @tom3333
 */

#ifndef _LIB_SERIAL_TYPES_H_
#define _LIB_SERIAL_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/* opaque pointer to a handle for the serial driver */
typedef struct lib_serial_handle *lib_serial_hdl;

enum baudrate {
    BAUDRATE_115200 = 115200,
    BAUDRATE_57600  = 57600,
    BAUDRATE_38400  = 38400,
    BAUDRATE_19200  = 19200,
    BAUDRATE_9600   = 9600,
    BAUDRATE_4800   = 4800
};

enum data_format {
	DATA_FORMAT_8_NONE_2,    //8 data bits, no   parity, 2 stop bit
	DATA_FORMAT_8_EVEN_1,	//8 data bits, even parity, 1 stop bit
	DATA_FORMAT_8_ODD_1,	//8 data bits, odd  parity, 1 stop bit
	DATA_FORMAT_8_NONE_1, 	//8 data bits, no   parity, 1 stop bit
	DATA_FORMAT_7_EVEN_1,    //7 data bits, even parity, 1 stop bit
	DATA_FORMAT_7_ODD_1		//7 data bist, odd  parity, 1 stop bit
};

enum loopback {
    LOOPBACK_off,
    LOOPBACK_on    
};

#ifdef __cplusplus
}
#endif

#endif  /* _LIB_SERIAL_TYPES_H_ */