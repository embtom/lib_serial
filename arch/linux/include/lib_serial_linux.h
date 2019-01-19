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
 *  Creation: 2018-08-07, @EmbeddedAl
 */

#ifndef _LIB_SERIAL_LINUX_H_
#define _LIB_SERIAL_LINUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <lib_serial_types.h>


/* Func: lib_serial_create()
 * Desc: Creates a handle to a specific serial port.
 *       On failure the pointer returned is NULL.
 *       The port will not be opened yet.
 *       This is the counter-function to lib_serial_destroy.
 * Param-in port: the port to be used for communication
 *                (NULL: behavior is undefined)
 * Return: the handle to be used for function calls to other API functions
 */
lib_serial_hdl lib_serial_create_linux(const char *port);

#ifdef __cplusplus
}
#endif

#endif  /* _LIB_SERIAL_LINUX_H_ */