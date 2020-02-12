/*
 * This file is part of the EMBTOM project
 * Copyright (c) 2018-2019 Thomas Willetal 
 * (https://github.com/embtom)
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
