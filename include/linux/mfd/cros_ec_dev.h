/*
 * cros_ec_dev - expose the Chrome OS Embedded Controller to userspace
 *
 * Copyright (C) 2013 The Chromium OS Authors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _CROS_EC_DEV_H_
#define _CROS_EC_DEV_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define CROS_EC_DEV_NAME "cros_ec"
#define CROS_EC_DEV_PD_NAME "cros_pd"
#define CROS_EC_DEV_VERSION "1.0.0"


/*
 * @version: Command version number (often 0)
 * @command: Command to send (EC_CMD_...)
 * @outdata: Outgoing data to EC
 * @outsize: Outgoing length in bytes
 * @indata: Where to put the incoming data from EC
 * @insize: Max number of bytes to accept from EC
 * @result: EC's response to the command (separate from communication failure)
 * ioctl returns the number of bytes returned from the EC, negative on error.
 */
struct cros_ec_command {
	uint32_t version;
	uint32_t command;
	uint8_t *outdata;
	uint32_t outsize;
	uint8_t *indata;
	uint32_t insize;
	uint32_t result;
};

/*
 * @offset: within EC_LPC_ADDR_MEMMAP region
 * @bytes: number of bytes to read. zero means "read a string" (including '\0')
 *         (at most only EC_MEMMAP_SIZE bytes can be read)
 * @buffer: where to store the result
 * ioctl returns the number of bytes read, negative on error
 */
struct cros_ec_readmem {
	uint32_t offset;
	uint32_t bytes;
	char *buffer;
};

#define CROS_EC_DEV_IOC              ':'
#define CROS_EC_DEV_IOCXCMD    _IOWR(':', 0, struct cros_ec_command)
#define CROS_EC_DEV_IOCRDMEM   _IOWR(':', 1, struct cros_ec_readmem)

#endif /* _CROS_EC_DEV_H_ */
