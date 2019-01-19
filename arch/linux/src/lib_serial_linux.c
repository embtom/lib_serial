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

/* includes */
#include <lib_serial.h>
#include <stdlib.h>
#include <errno.h>

#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h> /* ERROR Number Definitions           */

/* the handle definition for this implementation */
struct lib_serial_handle {
	int fd;
};


lib_serial_hdl lib_serial_create_linux(const char *port)
{
	lib_serial_hdl hdl = NULL;

	/* param check */
	if (port == NULL)
		return NULL;

	/* create handle space */
	hdl = malloc(sizeof(struct lib_serial_handle));
	if (hdl == NULL)
		return NULL;

	/* open port and store file descriptor to it */
	hdl->fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);

	/* error checking of open() */
	if(hdl->fd == -1)
	{
		free(hdl);
		return NULL;
	}

	return hdl;
}

int lib_serial_open(lib_serial_hdl hdl, enum baudrate baudrate, enum data_format format)
{
	struct termios portSettings;
	speed_t serialSpeed;

	/* hdl param check */
	if (hdl == NULL) {
		return -EINVAL;
	}

	/* get current settings */
	tcgetattr(hdl->fd, &portSettings);
		
	switch (baudrate)
	{
		case BAUDRATE_115200:
			serialSpeed = B115200;
			break;
		case BAUDRATE_57600:
			serialSpeed = B57600;
			break;	
		case BAUDRATE_38400:
			serialSpeed = B38400;
			break;	
		case BAUDRATE_19200:
			serialSpeed = B19200;
			break;
		case BAUDRATE_9600:
			serialSpeed = B9600;
			break;
		case BAUDRATE_4800:
			serialSpeed = B4800;
			break;
		default:
			return -EINVAL;			
	}

	switch (format)
	{
		case DATA_FORMAT_8_NONE_2:
			/* no parity */
			portSettings.c_cflag &= ~PARENB;
			/* 2 stop bits */
			portSettings.c_cflag |= CSTOPB;
			/* 8 bits */
			portSettings.c_cflag &= ~CSIZE;
			portSettings.c_cflag |=  CS8;
			break;
		case DATA_FORMAT_8_EVEN_1:
			/* even parity */
			portSettings.c_cflag |= PARENB;
			portSettings.c_cflag &= ~PARODD;
			/* 1 stop bit (flag for 2 is cleared) */
			portSettings.c_cflag &= ~CSTOPB;
			/* 8 bits */
			portSettings.c_cflag &= ~CSIZE;
			portSettings.c_cflag |=  CS8;
			break;
		case DATA_FORMAT_8_ODD_1:
			/* even parity */
			portSettings.c_cflag |= PARENB;
			portSettings.c_cflag |= PARODD;
			/* 1 stop bit (flag for 2 is cleared) */
			portSettings.c_cflag &= ~CSTOPB;
			/* 8 bits */
			portSettings.c_cflag &= ~CSIZE;
			portSettings.c_cflag |=  CS8;
			break;
		case DATA_FORMAT_8_NONE_1:
			/* no parity */
			portSettings.c_cflag &= ~PARENB;
			/* 1 stop bit (flag for 2 is cleared) */
			portSettings.c_cflag &= ~CSTOPB;
			/* 8 bits */
			portSettings.c_cflag &= ~CSIZE;
			portSettings.c_cflag |=  CS8;
			break;
		case DATA_FORMAT_7_EVEN_1:
			/* even parity */
			portSettings.c_cflag |= PARENB;
			portSettings.c_cflag &= ~PARODD;
			/* 1 stop bit (flag for 2 is cleared) */
			portSettings.c_cflag &= ~CSTOPB;
			/* 8 bits */
			portSettings.c_cflag &= ~CSIZE;
			portSettings.c_cflag |=  CS7;
			break;
		case DATA_FORMAT_7_ODD_1:
			/* even parity */
			portSettings.c_cflag |= PARENB;
			portSettings.c_cflag |= PARODD;
			/* 1 stop bit (flag for 2 is cleared) */
			portSettings.c_cflag &= ~CSTOPB;
			/* 8 bits */
			portSettings.c_cflag &= ~CSIZE;
			portSettings.c_cflag |=  CS7;
			break;
		default:
			return -EINVAL;

	}

	/* set speed in both directions */
	// todo: map input to constants?
	cfsetispeed(&portSettings,serialSpeed);
	cfsetospeed(&portSettings,serialSpeed);

	/* no flow control */
	portSettings.c_cflag &= ~(CRTSCTS | CMSPAR);
	/* Enable receiver,Ignore Modem Control lines */
	portSettings.c_cflag |= CREAD | CLOCAL | HUPCL;
	
	/* No input processings */ 
	portSettings.c_iflag = 0;
	
	portSettings.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
		
	/* No Output Processing */
	portSettings.c_oflag = 0;

	cfmakeraw(&portSettings);
	/* Set the attributes to the termios structure */
	if((tcsetattr(hdl->fd,TCSANOW,&portSettings)) != 0) {
	    return -EIO;
	}

	/* success */
	return 0;
}


int lib_serial_close(lib_serial_hdl hdl)
{
	/* hdl param check */
	if (hdl == NULL)
		return -EINVAL;

	/* close port */
	close(hdl->fd);

	return 0;
}


int lib_serial_destroy(lib_serial_hdl *hdl)
{
	/* hdl param check */
	if ((hdl == NULL) || (*hdl == NULL)) {
		return -EINVAL;
	}

	/* free allocated space and declare handle as invalid */
	free(*hdl);
	*hdl = NULL;

	/* all good */
	return 0;
}


int lib_serial_write(lib_serial_hdl hdl, const uint8_t * const data, int len)
{
	int ret, bytesWritten;
	
	/* hdl param check */
	if (hdl == NULL) {
		return -EINVAL;
	}

	/* write */
	for(bytesWritten = 0; bytesWritten < len; bytesWritten+=ret) 
	{
		ret = write(hdl->fd, data+bytesWritten, len-bytesWritten);
		if (ret < 0) {
			return -EIO;
		}
	}
	return 0;
}

int lib_serial_read(lib_serial_hdl hdl, uint8_t * const data, int maxlen, uint32_t frameTimeout)
{
	struct termios portSettings;
	int ret;

	/* hdl param check */
	if ((hdl == NULL) || (maxlen == 0)) {
		return -EINVAL;
	}

	/* get current settings */
	tcgetattr(hdl->fd, &portSettings);
	portSettings.c_cc[VMIN] = maxlen;
	/* timeout conversion from milliseconds to deciseconds */
	portSettings.c_cc[VTIME] = frameTimeout / 100;

	if((tcsetattr(hdl->fd,TCSANOW,&portSettings)) != 0) {
	    return -EIO;
	}
	ret = read(hdl->fd, data, maxlen);
	if (ret < 0) {
		return -EIO;
	}
	return ret;
}