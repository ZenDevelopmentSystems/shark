/*
 * I2cBus.cpp
 *
 *  Created on: Mar 25, 2014
 *      Author: fabien papleux
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "I2cBus.h"

using namespace std;


/*
 * wiringPiI2C.c:
 *	Simplified I2C access routines
 *	Copyright (c) 2013 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

/*
 * Notes:
 *	The Linux I2C code is actually the same (almost) as the SMBus code.
 *	SMBus is System Management Bus - and in essentially I2C with some
 *	additional functionality added, and stricter controls on the electrical
 *	specifications, etc. however I2C does work well with it and the
 *	protocols work over both.
 *
 *	I'm directly including the SMBus functions here as some Linux distros
 *	lack the correct header files, and also some header files are GPLv2
 *	rather than the LGPL that wiringPi is released under - presumably because
 *	originally no-one expected I2C/SMBus to be used outside the kernel -
 *	however enter the Raspberry Pi with people now taking directly to I2C
 *	devices without going via the kernel...
 *
 *	This may ultimately reduce the flexibility of this code, but it won't be
 *	hard to maintain it and keep it current, should things change.
 *
 *	Information here gained from: kernel/Documentation/i2c/dev-interface
 *	as well as other online resources.
 *********************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>


// I2C definitions

#define I2C_SLAVE	0x0703
#define I2C_SMBUS	0x0720	/* SMBus-level access */

#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

// SMBus transaction types

#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2 
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

// SMBus messages

#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */	
#define I2C_SMBUS_I2C_BLOCK_MAX	32	/* Not specified but we use same structure */

// Structures used in the ioctl() calls

union i2c_smbus_data
{
  uint8_t  byte ;
  uint16_t word ;
  uint8_t  block [I2C_SMBUS_BLOCK_MAX + 2] ;	// block [0] is used for length + one more for PEC
} ;

struct i2c_smbus_ioctl_data
{
  char read_write ;
  uint8_t command ;
  int size ;
  union i2c_smbus_data *data ;
} ;

static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;
  return ioctl (fd, I2C_SMBUS, &args) ;
}


/*
 * wiringPiI2CRead:
 *	Simple device read
 *********************************************************************************
 */

int wiringPiI2CRead (int fd)
{
  union i2c_smbus_data data ;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data))
    return -1 ;
  else
    return data.byte & 0xFF ;
}


/*
 * wiringPiI2CReadReg8: wiringPiI2CReadReg16:
 *	Read an 8 or 16-bit value from a regsiter on the device
 *********************************************************************************
 */

int wiringPiI2CReadReg8 (int fd, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data))
    return -1 ;
  else
    return data.byte & 0xFF ;
}

int wiringPiI2CReadReg16 (int fd, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data))
    return -1 ;
  else
    return data.word & 0xFFFF ;
}


/*
 * wiringPiI2CWrite:
 *	Simple device write
 *********************************************************************************
 */

int wiringPiI2CWrite (int fd, int data)
{
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, data, I2C_SMBUS_BYTE, NULL) ;
}


/*
 * wiringPiI2CWriteReg8: wiringPiI2CWriteReg16:
 *	Write an 8 or 16-bit value to the given register
 *********************************************************************************
 */

int wiringPiI2CWriteReg8 (int fd, int reg, int value)
{
  union i2c_smbus_data data ;

  data.byte = value ;
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data) ;
}

int wiringPiI2CWriteReg16 (int fd, int reg, int value)
{
  union i2c_smbus_data data ;

  data.word = value ;
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, &data) ;
}




/**********************************************************/
I2cBus::I2cBus()
{
	currentSlave = -1;
	fd = -1;
	ready = 0;
	init();
}

/**********************************************************/
I2cBus::~I2cBus()
{
	if (fd) close(fd);
}

/**********************************************************/
int I2cBus::init(void)
{
	ready = 0;
	i2cPath = "/dev/i2c-0";
	if ((fd = open (i2cPath, O_RDWR)) >= 0)
		ready = 1;
	return ready;
}

/**********************************************************/
void I2cBus::setSlave (int address)
{
	 //cout << "Setting new slave as 0x" << hex << address << dec << endl;
	if (! ready) return;
	if (fd < 0) return;
	if (ioctl (fd, I2C_SLAVE, address) >= 0)
		currentSlave = address;
}

/**********************************************************/
int I2cBus::isReady(void)
{
	return ready;
}

/**********************************************************/
int I2cBus::read8 (int address, int reg)
{
	if (! ready) return -1;
	if (fd < 0) return -1;
	if (address != currentSlave) setSlave(address);
	int data = wiringPiI2CReadReg8 (fd, reg);
	 //cout << "Read8 addr=0x" << hex << address << " register=0x" << reg << " value=0x" << data << dec << endl;
	return data;
}

/**********************************************************/
int I2cBus::read16 (int address, int reg)
{
	if (! ready) return -1;
	if (fd < 0) return -1;
	if (address != currentSlave) setSlave(address);
	int data = wiringPiI2CReadReg16 (fd, reg);
	 //cout << "Read16 addr=0x" << hex << address << " register=0x" << reg << " value=0x" << data << dec << endl;
	return data;
}

/**********************************************************/
int I2cBus::write8 (int address, int reg, int data)
{
	 //cout << "Write8 addr=0x" << hex << address << " register=0x" << reg << " value=0x" << data << dec << endl;
	if (! ready) return -1;
	if (fd < 0) return -1;
	if (address != currentSlave) setSlave(address);
	return wiringPiI2CWriteReg8 (fd, reg, data);
}

/**********************************************************/
int I2cBus::write16 (int address, int reg, int data)
{
	 //cout << "Write16 addr=0x" << hex << address << " register=0x" << reg << " value=0x" << data << dec << endl;
	if (! ready) return -1;
	if (fd < 0) return -1;
	if (address != currentSlave) setSlave(address);
	return wiringPiI2CWriteReg8 (fd, reg, data);
}

/**********************************************************/
void I2cBus::printStatus (void)
{
	cout << "I2C BUS Status" << endl;
	cout << "--------------" << endl;
	cout << endl;
	cout << "Is Ready              : " << (isReady() ? "Yes" : "No") << endl;
	cout << "System path           : " << i2cPath << endl;
	cout << "File Descriptor       : " << fd << endl;
	cout << "Current Slave Address : 0x" << hex << currentSlave << dec << endl;
	cout << endl;
}

