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
#define CROS_EC_DEV_VERSION "1.0.0"


/*
 * @version: Command version number (often 0)
 * @command: Command to send (EC_CMD_...)
 * @outdata: Outgoing data to EC
 * @outsize: Outgoing length in bytes
 * @indata: Where to put the incoming data from EC
 * @insize: Incoming length in bytes (filled in by EC)
 * @result: EC's response to the command (separate from communication failure)
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
 * @value: value read from EC
 */
struct cros_ec_read_mem8 {
	uint8_t offset;
	uint8_t value;
};

/*
 * @offset: within EC_LPC_ADDR_MEMMAP region
 * @value: value read from EC
 */
struct cros_ec_read_mem16 {
	uint8_t offset;
	uint16_t value;
};

/*
 * @offset: within EC_LPC_ADDR_MEMMAP region
 * @value: value read from EC
 */
struct cros_ec_read_mem32 {
	uint8_t offset;
	uint32_t value;
};

/*
 * @offset: within EC_LPC_ADDR_MEMMAP region
 * @buffer: where to store the string
 * @length: length of string returned (including any trailing '\0')
 */
struct cros_ec_read_string {
	uint8_t offset;
	char *buffer;
	uint8_t length;
};

#define CROS_EC_DEV_IOC              ':'
#define CROS_EC_DEV_IOCXCMD    _IOWR(':', 0, struct cros_ec_command)
#define CROS_EC_DEV_IOCRDMEM8  _IOWR(':', 1, struct cros_ec_read_mem8)
#define CROS_EC_DEV_IOCRDMEM16 _IOWR(':', 2, struct cros_ec_read_mem8)
#define CROS_EC_DEV_IOCRDMEM32 _IOWR(':', 3, struct cros_ec_read_mem16)
#define CROS_EC_DEV_IOCRDSTR   _IOWR(':', 4, struct cros_ec_read_string)
#define CROS_EC_DEV_IOC_MAXNR 4

#endif /* _CROS_EC_DEV_H_ */
