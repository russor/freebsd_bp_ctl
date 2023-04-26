/**************************************************************************

Copyright (c) 2006-2017, Silicom
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.

 3. Neither the name of the Silicom nor the names of its
	contributors may be used to endorse or promote products derived from
	this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/



#include <sys/param.h>	/* defines used in kernel.h */
#include <sys/module.h>
#include <sys/ioccom.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/kernel.h> /* types used in module initialization */
#include <sys/conf.h>	/* cdevsw struct */
#include <sys/mman.h>	/* mmap flags */
#include <sys/uio.h>	/* uio struct */
#include <sys/malloc.h>
#include <sys/bus.h>  /* structs, prototypes for pci bus stuff */

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>
#include <machine/clock.h>


#include <dev/pci/pcivar.h> /* For get_pci macros! */
#include <dev/pci/pcireg.h>



#include <sys/sysctl.h>
#include <sys/sockio.h>
#include <sys/socket.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/if_var.h>
#include <sys/types.h>
#include "../include/bp_ioctl.h"
#include "bp_mod.h"
#include "bypass.h"
#include "bp_cmd.h"




#define SUCCESS 0
/*#define BP_MOD_VER  VER_STR_SET*/
#define BP_MOD_VER  "4.0.2"
#define BP_MOD_DESCR "Silicom Bypass Control driver"

struct cdev *bpmod_dev;

struct mtx  mtx;


/*static int major_num=0;*/






#define EM_MMBA                         0x0010 /* Mem base address */


/* Intel Registers */
#define BPCTLI_CTRL          0x00000  
#define BPCTLI_CTRL_SWDPIO0  0x00400000 
#define BPCTLI_CTRL_SWDPIN0  0x00040000




/*static bpctl_dev_t *bpctl_dev_a;
static bpctl_dev_t *bpctl_dev_b;*/
/*bpctl_dev_t *bpctl_dev;*/
static bpctl_dev_t *bpctl_dev_arr;


int device_num;

static bpctl_dev_t *get_status_port_fn(bpctl_dev_t *pbpctl_dev);
static int get_bypass_caps_fn(bpctl_dev_t *pbpctl_dev);


/******************* I2C INTERFACE *****************************************************/
static inline void setsda(bpctl_dev_t *pslcm_dev, unsigned char bit){
	BPCTLI_WRITE_OFFSET(pslcm_dev, 0x88100, 0x3f80010);
	BP40G_RD_REG(pslcm_dev, GPIO_STAT);
	if (bit)
		BPCTLI_WRITE_OFFSET(pslcm_dev, 0x88184, 0x60);
	else
		BPCTLI_WRITE_OFFSET(pslcm_dev, 0x88184, 0x40);
	BP40G_RD_REG(pslcm_dev, GPIO_STAT);
}

static inline void setscl(bpctl_dev_t *pslcm_dev, unsigned char bit){
	BPCTLI_WRITE_OFFSET(pslcm_dev, 0x88104, 0x3f80010);
	BP40G_RD_REG(pslcm_dev, GPIO_STAT);
	if (bit)
		BPCTLI_WRITE_OFFSET(pslcm_dev, 0x88184, 0x61);
	else
		BPCTLI_WRITE_OFFSET(pslcm_dev, 0x88184, 0x41);
	BP40G_RD_REG(pslcm_dev, GPIO_STAT);
}


static inline void sdalo(bpctl_dev_t *pslcm_dev)
{

	setsda(pslcm_dev, 0);
	usec_delay((TIME_CLK + 1) / 2);
}

static inline void sdahi(bpctl_dev_t *pslcm_dev)
{
	setsda(pslcm_dev, 1);
	usec_delay((TIME_CLK + 1) / 2);
}

static inline unsigned char getscl(bpctl_dev_t *pslcm_dev)
{
	unsigned char ctrl_ext;
	BPCTLI_WRITE_OFFSET(pslcm_dev, 0x88104, 0x3f00000);
	BP40G_RD_REG(pslcm_dev, GPIO_STAT);
	ctrl_ext = BP40G_RD_REG(pslcm_dev, GPIO_STAT);
	return( (ctrl_ext&BIT_1)? 1:0);
}

static inline unsigned char getsda(bpctl_dev_t *pslcm_dev)
{
	unsigned char ctrl_ext;
	BPCTLI_WRITE_OFFSET(pslcm_dev, 0x88100, 0x3f00000);
	BP40G_RD_REG(pslcm_dev, GPIO_STAT);
	ctrl_ext = BP40G_RD_REG(pslcm_dev, GPIO_STAT);
	return( (ctrl_ext&BIT_0)? 1:0);
}


/*
 * Raise scl line, and do checking for delays. This is necessary for slower
 * devices.
 */
/* #define time_after(a,b)		\
	(typecheck(unsigned long, a) && \
	 typecheck(unsigned long, b) && \
	 ((long)((b) - (a)) < 0)) */


static int sclhi(bpctl_dev_t *pslcm_dev)
{
	unsigned long start;

	setscl(pslcm_dev, 1);

	/* Not all adapters have scl sense line... !!!*/

	start = ticks;
	while (!getscl(pslcm_dev)) {
		/* This hw knows how to read the clock line, so we wait
		 * until it actually gets high.  This is safer as some
		 * chips may hold it low ("clock stretching") while they
		 * are processing data internally.
		 */
		if ((long)((start+5) - (ticks)) < 0) {
			/* if (time_after(ticks, start + 5)) { */
			/* Test one last time, as we may have been preempted
			 * between last check and timeout test.
			 */
			if (getscl(pslcm_dev))
				break;
			return -1;
		}
		/* cpu_relax(); */
	}

	usec_delay(TIME_CLK);
	return 0;
}

static inline void scllo(bpctl_dev_t *pslcm_dev)
{
	setscl(pslcm_dev, 0);
	usec_delay(TIME_CLK / 2);
}



/* --- other auxiliary functions --------------------------------------	*/
static void i2c_start(bpctl_dev_t *pslcm_dev)
{
	/* assert: scl, sda are high */
	sdahi(pslcm_dev);
	sclhi(pslcm_dev);
	setsda(pslcm_dev, 0);
	usec_delay(TIME_CLK);
	scllo(pslcm_dev);
}

static void i2c_repstart(bpctl_dev_t *pslcm_dev)
{
	/* assert: scl is low */
	sdahi(pslcm_dev);
	sclhi(pslcm_dev);
	setsda(pslcm_dev, 0);
	usec_delay(TIME_CLK);
	scllo(pslcm_dev);
}


static void i2c_stop(bpctl_dev_t *pslcm_dev)
{
	/* assert: scl is low */
	sdalo(pslcm_dev);
	sclhi(pslcm_dev);
	setsda(pslcm_dev, 1);
	usec_delay(TIME_CLK);
}



/* send a byte without start cond., look for arbitration,
   check ackn. from slave */
/* returns:
 * 1 if the device acknowledged
 * 0 if the device did not ack
 * -ETIMEDOUT if an error occurred (while raising the scl line)
 */
static int i2c_outb(bpctl_dev_t *pslcm_dev, unsigned char c)
{
	int i;
	int sb;
	int ack;

	/* assert: scl is low */
	for (i = 7; i >= 0; i--) {
		sb = (c >> i) & 1;
		setsda(pslcm_dev, sb);
		usec_delay((TIME_CLK + 1) / 2);
		if (sclhi(pslcm_dev) < 0) {	/* timed out */
			printf("bpmod: timed out\n");
			return -1;
		}
		/* FIXME do arbitration here:
		 * if (sb && !getsda(adap)) -> ouch! Get out of here.
		 *
		 * Report a unique code, so higher level code can retry
		 * the whole (combined) message and *NOT* issue STOP.
		 */
		scllo(pslcm_dev);
	}
	sdahi(pslcm_dev);
	if (sclhi(pslcm_dev) < 0) {	/* timeout */
		/*printf("bpmod: i2c_outb: 0x%02x, "
			"timeout at ack\n", (int)c);*/
		return -1;
	}

	/* read ack: SDA should be pulled down by slave, or it may
	 * NAK (usually to report problems with the data we wrote).
	 */
	ack = !getsda(pslcm_dev);	 /* ack: sda is pulled low -> success */
	/*printf("bpmod: i2c_outb: 0x%02x %s\n", (int)c,
		ack ? "A" : "NA");*/

	scllo(pslcm_dev);
	return ack;
	/* assert: scl is low (sda undef) */
}


static int i2c_inb(bpctl_dev_t *pslcm_dev)
{
	/* read byte via i2c port, without start/stop sequence	*/
	/* acknowledge is sent in i2c_read.			*/
	int i;
	unsigned char indata = 0;

	/* assert: scl is low */
	sdahi(pslcm_dev);
	for (i = 0; i < 8; i++) {
		if (sclhi(pslcm_dev) < 0) {	/* timeout */
			/*printf("bpmod: i2c_inb: timeout at bit "
				"#%d\n", 7 - i);*/
			return -1;
		}
		indata *= 2;
		if (getsda(pslcm_dev))
			indata |= 0x01;
		setscl(pslcm_dev, 0);
		usec_delay(i == 7 ? TIME_CLK / 2 : TIME_CLK);
	}
	/* assert: scl is low */
	return indata;
}

/*
 * Sanity check for the adapter hardware - check the reaction of
 * the bus lines only if it seems to be idle.
 */
#if 0
static int test_bus(bpctl_dev_t *pslcm_dev)
{

	int scl, sda;

	sda = getsda(pslcm_dev);
	scl = getscl(pslcm_dev);
	if (!scl || !sda) {
		printf("bpmod: bus seems to be busy (scl=%d, sda=%d)\n",
			   scl, sda);
		goto bailout;
	}

	sdalo(pslcm_dev);
	sda = getsda(pslcm_dev);
	scl = getscl(pslcm_dev);
	if (sda) {
		printf("bpmod: SDA stuck high!\n");
		goto bailout;
	}
	if (!scl) {
		printf("bpmod: SCL unexpected low "
			   "while pulling SDA low!\n");
		goto bailout;
	}

	sdahi(pslcm_dev);
	sda = getsda(pslcm_dev);
	scl = getscl(pslcm_dev);
	if (!sda) {
		printf("bpmod: SDA stuck low!\n");
		goto bailout;
	}
	if (!scl) {
		printf("bpmod: SCL unexpected low "
			   "while pulling SDA high!\n");
		goto bailout;
	}

	scllo(pslcm_dev);
	sda = getsda(pslcm_dev);
	scl = getscl(pslcm_dev);
	if (scl) {
		printf("bpmod:  SCL stuck high!\n");
		goto bailout;
	}
	if (!sda) {
		printf("bpmod:  SDA unexpected low "
			   "while pulling SCL low!\n");
		goto bailout;
	}

	sclhi(pslcm_dev);
	sda = getsda(pslcm_dev);
	scl = getscl(pslcm_dev);
	if (!scl) {
		printf(KERN_WARNING "bpmod: SCL stuck low!\n");
		goto bailout;
	}
	if (!sda) {
		printf("bpmod: SDA unexpected low "
			   "while pulling SCL high!\n");
		goto bailout;
	}

	/*printf("bpmod: Test OK\n");*/
	return 0;
	bailout:
	sdahi(pslcm_dev);
	sclhi(pslcm_dev);

	return -ENODEV;
}
#endif

/* ----- Utility functions
 */

/* try_address tries to contact a chip for a number of
 * times before it gives up.
 * return values:
 * 1 chip answered
 * 0 chip did not answer
 * -x transmission error
 */
static int try_address(bpctl_dev_t *pslcm_dev,
					   unsigned char addr, int retries)
{
	int i, ret = 0;

	for (i = 0; i <= retries; i++) {
		ret = i2c_outb(pslcm_dev, addr);
		if (ret == 1 || i == retries)
			break;
		/*printf("bpmod: emitting stop condition\n");*/
		i2c_stop(pslcm_dev);
		usec_delay(TIME_CLK);
		/* yield(); */
		/*printf("bpmod: emitting start condition\n");*/
		i2c_start(pslcm_dev);
	}
	/*if (i && ret)
		printf("bpmod: Used %d tries to %s client at "
			"0x%02x: %s\n", i + 1,
			addr & 1 ? "read from" : "write to", addr >> 1,
			ret == 1 ? "success" : "failed, timeout?");*/
	return ret;
}

static int sendbytes(bpctl_dev_t *pslcm_dev, unsigned char *value,
					 unsigned int num_byte)
{
	int count =0;
	int retval;
	int wrcount = 0;

	do {
		retval = i2c_outb(pslcm_dev, value[count]);

		/* OK/ACK; or ignored NAK */
		if (retval > 0) {
			count++;
			wrcount++;

			/* A slave NAKing the master means the slave didn't like
			 * something about the data it saw.  For example, maybe
			 * the SMBus PEC was wrong.
			 */
		} else if (retval == 0) {
			printf("bpmod: sendbytes: NAK bailout.\n");
			return -1;

			/* Timeout; or (someday) lost arbitration
			 *
			 * FIXME Lost ARB implies retrying the transaction from
			 * the first message, after the "winning" master issues
			 * its STOP.  As a rule, upper layer code has no reason
			 * to know or care about this ... it is *NOT* an error.
			 */
		} else {
			printf("bpmod: sendbytes: error %d\n",
				   retval);
			return retval;
		}
	}while (count < num_byte);
	return wrcount;
}

static int acknak(bpctl_dev_t *pslcm_dev, int is_ack)
{

	/* assert: sda is high */
	if (is_ack)		/* send ack */
		setsda(pslcm_dev, 0);
	usec_delay((TIME_CLK + 1) / 2);
	if (sclhi(pslcm_dev) < 0) {	/* timeout */
		printf("bpmod: readbytes: ack/nak timeout\n");
		return -1;
	}
	scllo(pslcm_dev);
	return 0;
}

static int readbytes(bpctl_dev_t *pslcm_dev, unsigned char *value,
					 unsigned int num_byte)
{
	int inval;
	int rdcount = 0;	/* counts bytes read */
	int count = 0;

	do {
		inval = i2c_inb(pslcm_dev);
		if (inval >= 0) {
			value[count] = inval;
			rdcount++;
		} else {   /* read timed out */
			break;
		}

		count++;


		/*printf("bpmod: readbytes: 0x%02x %s\n",
			inval,
			(count ? "A" : "NA"));*/

		if (count == num_byte)
			inval = acknak(pslcm_dev, 0);
		else
			inval = acknak(pslcm_dev, count);
		if (inval < 0)
			return inval;
	} while (count < num_byte);
	return rdcount;
}

/* doAddress initiates the transfer by generating the start condition (in
 * try_address) and transmits the address in the necessary format to handle
 * reads, writes as well as 10bit-addresses.
 * returns:
 *  0 everything went okay, the chip ack'ed, or IGNORE_NAK flag was set
 * -x an error occurred (like: -ENXIO if the device did not answer, or
 *	-ETIMEDOUT, for example if the lines are stuck...)
 */
static int bit_doAddress(bpctl_dev_t *pslcm_dev, unsigned char rd, unsigned char addr)
{
	int ret, retries;

	retries = 1;

	{	   /* normal 7bit address	*/
		if (rd)
			addr |= 1;
		ret = try_address(pslcm_dev, addr, retries);
		if (ret != 1)
			return -1;
	}

	return 0;
}

static int bit_xfer(bpctl_dev_t *pslcm_dev,
					unsigned char rd,
					unsigned char addr,
					unsigned char *value,
					unsigned int num_byte)
{
	int i, ret, num =1;

	/*printf("bpmod: emitting start condition\n");*/
	i2c_start(pslcm_dev);
	for (i = 0; i < num; i++) {
		if (i) {
			printf("bpmod: emitting "
				   "repeated start condition\n");
			i2c_repstart(pslcm_dev);
		}
		ret = bit_doAddress(pslcm_dev, rd, addr);
		if (ret != 0) {
			printf("bpmod: NAK from "
				   "device addr 0x%02x\n",
				   addr);
			goto bailout;
		}
		if (rd) {
			/* read bytes into buffer*/
			ret = readbytes(pslcm_dev, value, num_byte);
			/*if (ret >= 1)
				printf("bpmod:read %d byte%s\n",
					ret, ret == 1 ? "" : "s");*/
			if (ret < num_byte) {
				if (ret >= 0)
					ret = -1;
				goto bailout;
			}
		} else {
			/* write bytes from buffer */
			ret = sendbytes(pslcm_dev, value, num_byte);
			/*if (ret >= 1)
				printf("bpmod:wrote %d byte%s\n",
					ret, ret == 1 ? "" : "s");*/
			if (ret < num_byte) {
				if (ret >= 0)
					ret = -1;
				goto bailout;
			}
		}
	}
	ret = i;

	bailout:
	/*printf("bpmod:emitting stop condition\n");*/
	i2c_stop(pslcm_dev);
	return ret;
}




static int bp_write_i2c_bytes(bpctl_dev_t *pslcm_dev, unsigned int num_byte, unsigned char *value,
							  unsigned char dev_addr)
{

	return(bit_xfer(pslcm_dev,
					0,
					dev_addr,
					value,
					num_byte));

}




static int bp_read_i2c_bytes(bpctl_dev_t *pslcm_dev, char *value, 
							 unsigned int num_byte,
							 unsigned int dev_addr)
{
	return(bit_xfer(pslcm_dev,
					1,
					dev_addr,
					value,
					num_byte));

}



#if 1 //Pavelk
static int set_bp_bytes_fn(bpctl_dev_t *pbpctl_dev, char *add_param){
	unsigned int ret=0;

	/*test_bus(pbpctl_dev);*/
	ret=bp_write_i2c_bytes(pbpctl_dev, 7, add_param, 0xae);

	return ret;
}

static int get_bp_bytes_fn(bpctl_dev_t *pbpctl_dev, char *add_param){
	char value[6];
	int ret=0;

	memset(value, 0, 6);

	ret=bp_read_i2c_bytes(pbpctl_dev, value, 6, 0xae);

	if (ret!=0) {
		memcpy(&add_param[0], value, 6);

		return 0;
	}

	return -1;
}

#endif  


static int bp_cmd_request(bpctl_dev_t *pbpctl_dev, bp_cmd_t *bp_cmd_buf, bp_cmd_rsp_t *bp_rsp_buf) {
	int ret_val;
	byte data[120]; 
	int try_num = 10;

	pbpctl_dev->wdt_busy=1;
	/*
	 * Send command
	 */
	while (try_num--) {
		memset(bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));

		memcpy(&data[0], bp_cmd_buf, sizeof(bp_cmd_t));
		usec_delay(CMND_INTERVAL);


		if ((ret_val = set_bp_bytes_fn(pbpctl_dev, (char *)data))<0) {
			printf("bp_cmd_request(write): Not supported!\n"); 
			continue;
			// return 0;
		}
		usec_delay(CMND_INTERVAL);
#if 1
		/*
		 * Receive replay
		 */
		memcpy(&data[0], bp_cmd_buf, sizeof(bp_cmd_t));
		if ((ret_val = get_bp_bytes_fn(pbpctl_dev, (char *)data))<0) {
			printf("bp_cmd_request(read): Not supported!\n");
			continue;
			//return 0;	
		}
#endif

		// if (bp_rsp_buf->rsp.rsp_id != BP_ERR_OK)
		//	continue;


		memcpy(bp_rsp_buf, &data[0], sizeof(bp_cmd_rsp_t));
		if (bp_rsp_buf->rsp.rsp_id != BP_ERR_OK)
			continue;

#if 1 
		if (bp_rsp_buf->rsp.rsp_id != BP_ERR_OK)
			printf("bp_cmd_request(got err code or corrupted data!) (%x %x %x %x %x %x)\n",
				   bp_rsp_buf->cmd_bytes[0],bp_rsp_buf->cmd_bytes[1],bp_rsp_buf->cmd_bytes[2],
				   bp_rsp_buf->cmd_bytes[3],bp_rsp_buf->cmd_bytes[4],bp_rsp_buf->cmd_bytes[5]);

#endif

		break;
	}
	pbpctl_dev->wdt_busy=0;

	if (!try_num)
		return 0;
	return 1;
}

/*******************I2C INTERFACE END*****************************************************/





static void write_pulse(bpctl_dev_t *pbpctl_dev, 
						unsigned int ctrl_ext ,
						unsigned char value, 
						unsigned char len){
	unsigned char ctrl_val=0;
	unsigned int i=len;
	unsigned int ctrl= 0;
	bpctl_dev_t *pbpctl_dev_c=NULL;

	if (pbpctl_dev->bp_i80) {
		ctrl= BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	}
	if (pbpctl_dev->bp_540) {
		ctrl= BP10G_READ_REG(pbpctl_dev, ESDP);
	}


	if (pbpctl_dev->bp_10g9) {
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
			return;
		ctrl= BP10G_READ_REG(pbpctl_dev_c, ESDP);
	}

	while (i--) {
		ctrl_val=(value>>i) & 0x1;
		if (ctrl_val) {
			if (pbpctl_dev->bp_10g9) {

				/* To start management : MCLK 1, MDIO 1, output*/
				/* DATA 1 CLK 1*/
				/*BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext|BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9));*/
				BP10G_WRITE_REG(pbpctl_dev, I2CCTL, ctrl_ext|BP10G_MDIO_DATA_OUT9);
				BP10G_WRITE_REG(pbpctl_dev_c, ESDP, (ctrl|BP10G_MCLK_DATA_OUT9|BP10G_MCLK_DIR_OUT9));

			} else if (pbpctl_dev->bp_fiber5) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, (ctrl_ext | 
													  BPCTLI_CTRL_EXT_MCLK_DIR5|
													  BPCTLI_CTRL_EXT_MDIO_DIR5|
													  BPCTLI_CTRL_EXT_MDIO_DATA5 | BPCTLI_CTRL_EXT_MCLK_DATA5));


			} else if (pbpctl_dev->bp_i80) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, (ctrl_ext | 
													  BPCTLI_CTRL_EXT_MDIO_DIR80|
													  BPCTLI_CTRL_EXT_MDIO_DATA80 ));


				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (ctrl | 
														  BPCTLI_CTRL_EXT_MCLK_DIR80|
														  BPCTLI_CTRL_EXT_MCLK_DATA80));


			} else if (pbpctl_dev->bp_540) {
				BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl | 
												   BP540_MDIO_DIR|
												   BP540_MDIO_DATA| BP540_MCLK_DIR|BP540_MCLK_DATA));




			} else if (pbpctl_dev->bp_10gb) {
				BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext | BP10GB_MDIO_SET|
															 BP10GB_MCLK_SET)&~(BP10GB_MCLK_DIR|BP10GB_MDIO_DIR|BP10GB_MDIO_CLR|BP10GB_MCLK_CLR)); 

			} else if (!pbpctl_dev->bp_10g)
				/* To start management : MCLK 1, MDIO 1, output*/
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (ctrl_ext | 
														  BPCTLI_CTRL_EXT_MCLK_DIR|
														  BPCTLI_CTRL_EXT_MDIO_DIR|
														  BPCTLI_CTRL_EXT_MDIO_DATA | BPCTLI_CTRL_EXT_MCLK_DATA));
			else {

/* To start management : MCLK 1, MDIO 1, output*/
				//writel((0x2|0x8), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
				BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext|BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT));
				//BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl | BP10G_MDIO_DATA | BP10G_MDIO_DIR));  

			}


			usec_delay(PULSE_TIME);
			if (pbpctl_dev->bp_10g9) {

				/*BP10G_WRITE_REG(pbpctl_dev, I2CCTL, ((ctrl_ext|BP10G_MDIO_DATA_OUT9)&~(BP10G_MCLK_DATA_OUT9)));*/
				/* DATA 1 CLK 0*/
				BP10G_WRITE_REG(pbpctl_dev, I2CCTL, ctrl_ext|BP10G_MDIO_DATA_OUT9);
				BP10G_WRITE_REG(pbpctl_dev_c, ESDP, (ctrl|BP10G_MCLK_DIR_OUT9)&~BP10G_MCLK_DATA_OUT9);


			} else if (pbpctl_dev->bp_fiber5) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
													   BPCTLI_CTRL_EXT_MCLK_DIR5|
													   BPCTLI_CTRL_EXT_MDIO_DIR5|BPCTLI_CTRL_EXT_MDIO_DATA5)&~(BPCTLI_CTRL_EXT_MCLK_DATA5)));

			} else if (pbpctl_dev->bp_i80) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, (ctrl_ext | 
													  BPCTLI_CTRL_EXT_MDIO_DIR80|BPCTLI_CTRL_EXT_MDIO_DATA80));
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
														   BPCTLI_CTRL_EXT_MCLK_DIR80)&~(BPCTLI_CTRL_EXT_MCLK_DATA80)));

			} else if (pbpctl_dev->bp_540) {
				BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl | BP540_MDIO_DIR|BP540_MDIO_DATA|BP540_MCLK_DIR)&~(BP540_MCLK_DATA));

			} else if (pbpctl_dev->bp_10gb) {

				BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext | BP10GB_MDIO_SET|
															 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR|BP10GB_MDIO_DIR|BP10GB_MDIO_CLR|BP10GB_MCLK_SET));

			} else if (!pbpctl_dev->bp_10g)

				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
														   BPCTLI_CTRL_EXT_MCLK_DIR|
														   BPCTLI_CTRL_EXT_MDIO_DIR|BPCTLI_CTRL_EXT_MDIO_DATA)&~(BPCTLI_CTRL_EXT_MCLK_DATA)));
			else {

				//writel((0x2), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
				BP10G_WRITE_REG(pbpctl_dev, EODSDP, ((ctrl_ext|BP10G_MDIO_DATA_OUT)&~(BP10G_MCLK_DATA_OUT)));
				//  BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl |BP10G_MDIO_DIR|BP10G_MDIO_DATA));
			}

			usec_delay(PULSE_TIME);

		} else {
			if (pbpctl_dev->bp_10g9) {
				/* DATA 0 CLK 1*/
				/*BP10G_WRITE_REG(pbpctl_dev, I2CCTL, ((ctrl_ext|BP10G_MCLK_DATA_OUT9)&~BP10G_MDIO_DATA_OUT9));*/
				BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
				BP10G_WRITE_REG(pbpctl_dev_c, ESDP, (ctrl|BP10G_MCLK_DATA_OUT9|BP10G_MCLK_DIR_OUT9));

			} else if (pbpctl_dev->bp_fiber5) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
													   BPCTLI_CTRL_EXT_MCLK_DIR5|
													   BPCTLI_CTRL_EXT_MDIO_DIR5|BPCTLI_CTRL_EXT_MCLK_DATA5)&~(BPCTLI_CTRL_EXT_MDIO_DATA5)));

			} else if (pbpctl_dev->bp_i80) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
													   BPCTLI_CTRL_EXT_MDIO_DIR80)&~(BPCTLI_CTRL_EXT_MDIO_DATA80)));
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (ctrl | 
														  BPCTLI_CTRL_EXT_MCLK_DIR80|
														  BPCTLI_CTRL_EXT_MCLK_DATA80));

			} else if (pbpctl_dev->bp_540) {
				BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP540_MCLK_DIR| BP540_MCLK_DATA|BP540_MDIO_DIR)&~(BP540_MDIO_DATA)));


			} else if (pbpctl_dev->bp_10gb) {
				BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext | BP10GB_MDIO_CLR|
															 BP10GB_MCLK_SET)&~(BP10GB_MCLK_DIR|BP10GB_MDIO_DIR|BP10GB_MDIO_SET|BP10GB_MCLK_CLR));

			} else if (!pbpctl_dev->bp_10g)

				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
														   BPCTLI_CTRL_EXT_MCLK_DIR|
														   BPCTLI_CTRL_EXT_MDIO_DIR|BPCTLI_CTRL_EXT_MCLK_DATA)&~(BPCTLI_CTRL_EXT_MDIO_DATA)));
			else {

				//    writel((0x8), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
				BP10G_WRITE_REG(pbpctl_dev, EODSDP, ((ctrl_ext|BP10G_MCLK_DATA_OUT)&~BP10G_MDIO_DATA_OUT));
				//  BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));

			}
			usec_delay(PULSE_TIME);
			if (pbpctl_dev->bp_10g9) {
				/* DATA 0 CLK 0 */
				/*BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~(BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9)));*/
				BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
				BP10G_WRITE_REG(pbpctl_dev_c, ESDP, ((ctrl|BP10G_MCLK_DIR_OUT9)&~(BP10G_MCLK_DATA_OUT9)));


			} else if (pbpctl_dev->bp_fiber5) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
													   BPCTLI_CTRL_EXT_MCLK_DIR5|
													   BPCTLI_CTRL_EXT_MDIO_DIR5)&~(BPCTLI_CTRL_EXT_MCLK_DATA5|BPCTLI_CTRL_EXT_MDIO_DATA5)));

			} else if (pbpctl_dev->bp_i80) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
													   BPCTLI_CTRL_EXT_MDIO_DIR80)&~BPCTLI_CTRL_EXT_MDIO_DATA80));
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
														   BPCTLI_CTRL_EXT_MCLK_DIR80)&~(BPCTLI_CTRL_EXT_MCLK_DATA80)));

			} else if (pbpctl_dev->bp_540) {
				BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl | BP540_MCLK_DIR|
													BP540_MDIO_DIR)&~(BP540_MDIO_DATA|BP540_MCLK_DATA)));
			} else if (pbpctl_dev->bp_10gb) {

				BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext | BP10GB_MDIO_CLR|
															 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR|BP10GB_MDIO_DIR|BP10GB_MDIO_SET|BP10GB_MCLK_SET));

			} else if (!pbpctl_dev->bp_10g)
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
														   BPCTLI_CTRL_EXT_MCLK_DIR|
														   BPCTLI_CTRL_EXT_MDIO_DIR)&~(BPCTLI_CTRL_EXT_MCLK_DATA|BPCTLI_CTRL_EXT_MDIO_DATA)));
			else {

				//writel((0x0), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
				BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext&~(BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT)));
				//BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));
			}

			usec_delay(PULSE_TIME);
		} 

	}
}

static int read_pulse(bpctl_dev_t *pbpctl_dev, unsigned int ctrl_ext ,unsigned char len){
	unsigned char ctrl_val=0;
	unsigned int i=len;
	unsigned int ctrl= 0;
	bpctl_dev_t *pbpctl_dev_c=NULL;

	if (pbpctl_dev->bp_i80)
		ctrl= BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	if (pbpctl_dev->bp_540)
		ctrl= BP10G_READ_REG(pbpctl_dev, ESDP);
	if (pbpctl_dev->bp_10g9) {
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
			return -1;
		ctrl= BP10G_READ_REG(pbpctl_dev_c, ESDP);
	}


	//ctrl_ext=BP10G_READ_REG(pbpctl_dev,EODSDP);    

	while (i--) {
		if (pbpctl_dev->bp_10g9) {
			/*BP10G_WRITE_REG(pbpctl_dev, I2CCTL, ((ctrl_ext|BP10G_MDIO_DATA_OUT9)&~BP10G_MCLK_DATA_OUT9));*/
			/* DATA ? CLK 0*/
			BP10G_WRITE_REG(pbpctl_dev_c, ESDP, ((ctrl|BP10G_MCLK_DIR_OUT9)&~(BP10G_MCLK_DATA_OUT9)));

		} else if (pbpctl_dev->bp_fiber5) {
			BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR5)&~(BPCTLI_CTRL_EXT_MDIO_DIR5 | BPCTLI_CTRL_EXT_MCLK_DATA5)));


		} else if (pbpctl_dev->bp_i80) {
			BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, (ctrl_ext &~BPCTLI_CTRL_EXT_MDIO_DIR80 ));
			BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
													   BPCTLI_CTRL_EXT_MCLK_DIR80)&~( BPCTLI_CTRL_EXT_MCLK_DATA80)));



		} else if (pbpctl_dev->bp_540) {
			BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl|BP540_MCLK_DIR) &~(BP540_MDIO_DIR|BP540_MCLK_DATA) ));



		} else if (pbpctl_dev->bp_10gb) {


			BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext | BP10GB_MDIO_DIR|
														 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR|BP10GB_MDIO_CLR| BP10GB_MDIO_SET|BP10GB_MCLK_SET));

		} else if (!pbpctl_dev->bp_10g)
			BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
													   BPCTLI_CTRL_EXT_MCLK_DIR)&~(BPCTLI_CTRL_EXT_MDIO_DIR | BPCTLI_CTRL_EXT_MCLK_DATA)));
		else {

			// writel(( 0/*0x1*/), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
			BP10G_WRITE_REG(pbpctl_dev, EODSDP, ((ctrl_ext|BP10G_MDIO_DATA_OUT)&~BP10G_MCLK_DATA_OUT));	 /* ? */
			//    printf("0x28=0x%x\n",BP10G_READ_REG(pbpctl_dev,EODSDP););
			//BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl &~BP10G_MDIO_DIR));

		}

		usec_delay(PULSE_TIME);
		if (pbpctl_dev->bp_10g9) {
			/*BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext|BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9));*/
			/* DATA ? CLK 1*/
			BP10G_WRITE_REG(pbpctl_dev_c, ESDP, (ctrl|BP10G_MCLK_DATA_OUT9|BP10G_MCLK_DIR_OUT9));



		} else if (pbpctl_dev->bp_fiber5) {
			BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR5|
												   BPCTLI_CTRL_EXT_MCLK_DATA5)&~(BPCTLI_CTRL_EXT_MDIO_DIR5)));



		} else if (pbpctl_dev->bp_i80) {
			BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, (ctrl_ext&~(BPCTLI_CTRL_EXT_MDIO_DIR80)));
			BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (ctrl | 
													  BPCTLI_CTRL_EXT_MCLK_DIR80|
													  BPCTLI_CTRL_EXT_MCLK_DATA80));




		} else if (pbpctl_dev->bp_540) {
			BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl|BP540_MCLK_DIR|BP540_MCLK_DATA)&~(BP540_MDIO_DIR)));




		} else if (pbpctl_dev->bp_10gb) {
			BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext | BP10GB_MDIO_DIR|
														 BP10GB_MCLK_SET)&~(BP10GB_MCLK_DIR|BP10GB_MDIO_CLR| BP10GB_MDIO_SET|BP10GB_MCLK_CLR));


		} else if (!pbpctl_dev->bp_10g)
			BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
													   BPCTLI_CTRL_EXT_MCLK_DIR|
													   BPCTLI_CTRL_EXT_MCLK_DATA)&~(BPCTLI_CTRL_EXT_MDIO_DIR)));
		else {

			// writel((0x8 /*|0x1*/ ), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
			BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext|BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT));
			//BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl &~BP10G_MDIO_DIR));

		}
		if (pbpctl_dev->bp_10g9) {
			ctrl_ext= BP10G_READ_REG(pbpctl_dev,I2CCTL);

		} else if ((pbpctl_dev->bp_fiber5)||(pbpctl_dev->bp_i80)) {
			ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL);
		} else if (pbpctl_dev->bp_540) {
			ctrl_ext = BP10G_READ_REG(pbpctl_dev, ESDP);
		} else if (pbpctl_dev->bp_10gb)
			ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);

		else if (!pbpctl_dev->bp_10g)
			ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
		else
			ctrl_ext= BP10G_READ_REG(pbpctl_dev,EODSDP);
		//ctrl_ext =readl((void *)((pbpctl_dev)->mem_map) + 0x28);


		usec_delay(PULSE_TIME);
		if (pbpctl_dev->bp_10g9) {
			if (ctrl_ext & BP10G_MDIO_DATA_IN9)
				ctrl_val |= 1<<i;

		} else if (pbpctl_dev->bp_fiber5) {
			if (ctrl_ext & BPCTLI_CTRL_EXT_MDIO_DATA5)
				ctrl_val |= 1<<i;
		} else if (pbpctl_dev->bp_i80) {
			if (ctrl_ext & BPCTLI_CTRL_EXT_MDIO_DATA80)
				ctrl_val |= 1<<i;
		} else if (pbpctl_dev->bp_540) {
			if (ctrl_ext & BP540_MDIO_DATA)
				ctrl_val |= 1<<i;
		} else if (pbpctl_dev->bp_10gb) {
			if (ctrl_ext & BP10GB_MDIO_DATA)
				ctrl_val |= 1<<i;

		} else if (!pbpctl_dev->bp_10g) {

			if (ctrl_ext & BPCTLI_CTRL_EXT_MDIO_DATA)
				ctrl_val |= 1<<i;
		} else {

			if (ctrl_ext & BP10G_MDIO_DATA_IN)
				ctrl_val |= 1<<i;
		}

	}

	return ctrl_val;
}

static void write_reg(bpctl_dev_t *pbpctl_dev, unsigned char value, unsigned char addr){
	uint32_t ctrl_ext=0, ctrl=0;
	bpctl_dev_t *pbpctl_dev_c=NULL;
#ifdef BP_SYNC_FLAG
	unsigned long flags;
#endif  
	if (pbpctl_dev->bp_10g9) {
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
			return;
	}
	if ((pbpctl_dev->wdt_status==WDT_STATUS_EN)&&
		(pbpctl_dev->bp_ext_ver<PXG4BPFI_VER))
		wdt_time_left(pbpctl_dev);

#ifdef BP_SYNC_FLAG
	spin_lock_irqsave(&pbpctl_dev->bypass_wr_lock, flags);
#else
	pbpctl_dev->wdt_busy=1;
#endif
	if (pbpctl_dev->bp_10g9) {

		ctrl_ext=BP10G_READ_REG(pbpctl_dev,I2CCTL);
		ctrl= BP10G_READ_REG(pbpctl_dev_c, ESDP);
		/* DATA 0 CLK 0*/
		/* BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~(BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9))); */
		BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
		BP10G_WRITE_REG(pbpctl_dev_c, ESDP, ((ctrl|BP10G_MCLK_DIR_OUT9)&~(BP10G_MCLK_DATA_OUT9)));

	} else if (pbpctl_dev->bp_fiber5) {
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL);
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MCLK_DIR5 |
											   BPCTLI_CTRL_EXT_MDIO_DIR5 )&~(BPCTLI_CTRL_EXT_MDIO_DATA5|BPCTLI_CTRL_EXT_MCLK_DATA5)));
	} else if (pbpctl_dev->bp_i80) {
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL);
		ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MDIO_DIR80 )&~BPCTLI_CTRL_EXT_MDIO_DATA80));
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
												   BPCTLI_CTRL_EXT_MCLK_DIR80 )&~BPCTLI_CTRL_EXT_MCLK_DATA80));

	} else if (pbpctl_dev->bp_540) {
		ctrl=ctrl_ext = BP10G_READ_REG(pbpctl_dev, ESDP);
		BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl | 
											BP540_MDIO_DIR| BP540_MCLK_DIR)&~(BP540_MDIO_DATA|BP540_MCLK_DATA)));

	} else if (pbpctl_dev->bp_10gb) {
		ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);

		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext |  BP10GB_MDIO_CLR|
													 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR| BP10GB_MDIO_DIR| BP10GB_MDIO_SET|BP10GB_MCLK_SET));


	} else if (!pbpctl_dev->bp_10g) {


		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR |
												   BPCTLI_CTRL_EXT_MDIO_DIR )&~(BPCTLI_CTRL_EXT_MDIO_DATA|BPCTLI_CTRL_EXT_MCLK_DATA)));
	} else {
		ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);
		ctrl_ext=BP10G_READ_REG(pbpctl_dev,EODSDP);
		BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext&~(BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT)));
		//BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));
		//writel((0x0), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
	}
	usec_delay(CMND_INTERVAL);

	/*send sync cmd*/
	write_pulse(pbpctl_dev,ctrl_ext,SYNC_CMD_VAL,SYNC_CMD_LEN);
	/*send wr cmd*/
	write_pulse(pbpctl_dev,ctrl_ext,WR_CMD_VAL,WR_CMD_LEN);
	write_pulse(pbpctl_dev,ctrl_ext,addr,ADDR_CMD_LEN);

	/*write data*/
	write_pulse(pbpctl_dev,ctrl_ext,value,WR_DATA_LEN);
	if (pbpctl_dev->bp_10g9) {
		/*BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~(BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9)));*/
		/* DATA 0 CLK 0 */
		BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
		BP10G_WRITE_REG(pbpctl_dev_c, ESDP, ((ctrl|BP10G_MCLK_DIR_OUT9)&~(BP10G_MCLK_DATA_OUT9)));


	} else if (pbpctl_dev->bp_fiber5) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MCLK_DIR5 |
											   BPCTLI_CTRL_EXT_MDIO_DIR5 )&~(BPCTLI_CTRL_EXT_MDIO_DATA5|BPCTLI_CTRL_EXT_MCLK_DATA5)));
	} else if (pbpctl_dev->bp_i80) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MDIO_DIR80 )&~BPCTLI_CTRL_EXT_MDIO_DATA80));
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
												   BPCTLI_CTRL_EXT_MCLK_DIR80 )&~BPCTLI_CTRL_EXT_MCLK_DATA80));
	} else if (pbpctl_dev->bp_540) {
		BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl | 
											BP540_MDIO_DIR| BP540_MCLK_DIR)&~(BP540_MDIO_DATA|BP540_MCLK_DATA)));
	} else if (pbpctl_dev->bp_10gb) {
		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext |  BP10GB_MDIO_CLR|
													 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR| BP10GB_MDIO_DIR| BP10GB_MDIO_SET|BP10GB_MCLK_SET));



	} else if (!pbpctl_dev->bp_10g)

		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR |
												   BPCTLI_CTRL_EXT_MDIO_DIR )&~(BPCTLI_CTRL_EXT_MDIO_DATA|BPCTLI_CTRL_EXT_MCLK_DATA)));
	else {
		BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext&~(BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT)));
		// BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));


		//   writel((0x0), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
	}

	usec_delay(CMND_INTERVAL);


	if ((pbpctl_dev->wdt_status==WDT_STATUS_EN)&&
		(pbpctl_dev->bp_ext_ver<PXG4BPFI_VER)&&
		(addr==CMND_REG_ADDR))
		pbpctl_dev->bypass_wdt_on_time=ticks;
#ifdef BP_SYNC_FLAG
	spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);
#else
	pbpctl_dev->wdt_busy=0;
#endif

}


static void write_data(bpctl_dev_t *pbpctl_dev, unsigned char value){
	write_reg(pbpctl_dev, value, CMND_REG_ADDR);
}

static int read_reg(bpctl_dev_t *pbpctl_dev, unsigned char addr){
	uint32_t ctrl_ext=0, ctrl=0 , ctrl_value=0;
	bpctl_dev_t *pbpctl_dev_c=NULL;



#ifdef BP_SYNC_FLAG
	unsigned long flags;
	spin_lock_irqsave(&pbpctl_dev->bypass_wr_lock, flags);
#else
	pbpctl_dev->wdt_busy=1;
#endif
	if (pbpctl_dev->bp_10g9) {
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
			return -1;
	}


	if (pbpctl_dev->bp_10g9) {
		ctrl_ext=BP10G_READ_REG(pbpctl_dev,I2CCTL);
		ctrl= BP10G_READ_REG(pbpctl_dev_c, ESDP);

		/* BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~(BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9)));*/
		/* DATA 0 CLK 0 */
		BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
		BP10G_WRITE_REG(pbpctl_dev_c, ESDP, ((ctrl|BP10G_MCLK_DIR_OUT9)&~(BP10G_MCLK_DATA_OUT9)));


	} else if (pbpctl_dev->bp_fiber5) {
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL);

		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MCLK_DIR5 |
											   BPCTLI_CTRL_EXT_MDIO_DIR5)&~(BPCTLI_CTRL_EXT_MDIO_DATA5|BPCTLI_CTRL_EXT_MCLK_DATA5)));
	} else if (pbpctl_dev->bp_i80) {
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL);
		ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT); 

		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MDIO_DIR80)&~BPCTLI_CTRL_EXT_MDIO_DATA80));
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
												   BPCTLI_CTRL_EXT_MCLK_DIR80 )&~BPCTLI_CTRL_EXT_MCLK_DATA80));
	} else if (pbpctl_dev->bp_540) {
		ctrl_ext = BP10G_READ_REG(pbpctl_dev, ESDP);
		ctrl = BP10G_READ_REG(pbpctl_dev, ESDP); 

		BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl | BP540_MCLK_DIR|
											BP540_MDIO_DIR)&~(BP540_MDIO_DATA|BP540_MCLK_DATA)));
	} else if (pbpctl_dev->bp_10gb) {
		ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);


		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext |  BP10GB_MDIO_CLR|
													 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR| BP10GB_MDIO_DIR| BP10GB_MDIO_SET|BP10GB_MCLK_SET));
#if 0

		/*BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext | BP10GB_MCLK_DIR | BP10GB_MDIO_DIR|
													 BP10GB_MCLK_CLR|BP10GB_MDIO_CLR));
		ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);
		printf("1reg=%x\n", ctrl_ext);*/

		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, ((ctrl_ext |
													  BP10GB_MCLK_SET|BP10GB_MDIO_CLR))&~(BP10GB_MCLK_CLR|BP10GB_MDIO_SET| BP10GB_MCLK_DIR | BP10GB_MDIO_DIR));


		/*   bnx2x_set_spio(pbpctl_dev, 5, MISC_REGISTERS_SPIO_OUTPUT_LOW);
		   bnx2x_set_spio(pbpctl_dev, 4, MISC_REGISTERS_SPIO_OUTPUT_LOW);
		   bnx2x_set_spio(pbpctl_dev, 4, MISC_REGISTERS_SPIO_INPUT_HI_Z);*/


		ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);

		//printf("2reg=%x\n", ctrl_ext);


#ifdef BP_SYNC_FLAG
		spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);
#else
		pbpctl_dev->wdt_busy=0;
#endif



		return 0;

#endif

	} else if (!pbpctl_dev->bp_10g) {

		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);

		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR |
												   BPCTLI_CTRL_EXT_MDIO_DIR)&~(BPCTLI_CTRL_EXT_MDIO_DATA|BPCTLI_CTRL_EXT_MCLK_DATA)));
	} else {

		//   writel((0x0), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
		ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);
		ctrl_ext=BP10G_READ_REG(pbpctl_dev,EODSDP);
		BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext&~(BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT)));
		//BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));

	}

	usec_delay(CMND_INTERVAL);

	/*send sync cmd*/
	write_pulse(pbpctl_dev,ctrl_ext,SYNC_CMD_VAL,SYNC_CMD_LEN);
	/*send rd cmd*/
	write_pulse(pbpctl_dev,ctrl_ext,RD_CMD_VAL,RD_CMD_LEN);
	/*send addr*/
	write_pulse(pbpctl_dev,ctrl_ext,addr, ADDR_CMD_LEN);
	/*read data*/
	/* zero */
	if (pbpctl_dev->bp_10g9) {
		/* DATA 0 CLK 1*/
		/*BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext|BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9));*/
		BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext|BP10G_MDIO_DATA_OUT9));
		BP10G_WRITE_REG(pbpctl_dev_c, ESDP, (ctrl|BP10G_MCLK_DATA_OUT9|BP10G_MCLK_DIR_OUT9));

	} else if (pbpctl_dev->bp_fiber5) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MCLK_DIR5|BPCTLI_CTRL_EXT_MCLK_DATA5)&~(BPCTLI_CTRL_EXT_MDIO_DIR5|BPCTLI_CTRL_EXT_MDIO_DATA5)));

	} else if (pbpctl_dev->bp_i80) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, (ctrl_ext  &~(BPCTLI_CTRL_EXT_MDIO_DATA80| BPCTLI_CTRL_EXT_MDIO_DIR80)));
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (ctrl | 
												  BPCTLI_CTRL_EXT_MCLK_DIR80|BPCTLI_CTRL_EXT_MCLK_DATA80));

	} else if (pbpctl_dev->bp_540) {
		BP10G_WRITE_REG(pbpctl_dev, ESDP, (((ctrl|BP540_MDIO_DIR|BP540_MCLK_DIR|BP540_MCLK_DATA)&~BP540_MDIO_DATA )));

	} else if (pbpctl_dev->bp_10gb) {

		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext |  BP10GB_MDIO_DIR|
													 BP10GB_MCLK_SET)&~(BP10GB_MCLK_DIR| BP10GB_MDIO_SET|BP10GB_MDIO_CLR| BP10GB_MCLK_CLR));

	} else if (!pbpctl_dev->bp_10g)
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR|BPCTLI_CTRL_EXT_MCLK_DATA)&~(BPCTLI_CTRL_EXT_MDIO_DIR|BPCTLI_CTRL_EXT_MDIO_DATA)));
	else {

		// writel((0x8), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ; 
		BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext|BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT));

		// BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl &~(BP10G_MDIO_DATA|BP10G_MDIO_DIR)));

	}
	usec_delay(PULSE_TIME);

	ctrl_value= read_pulse(pbpctl_dev,ctrl_ext,RD_DATA_LEN);

	if (pbpctl_dev->bp_10g9) {
		ctrl_ext=BP10G_READ_REG(pbpctl_dev,I2CCTL);
		ctrl= BP10G_READ_REG(pbpctl_dev_c, ESDP);

		/* BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~(BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9)));*/
		/* DATA 0 CLK 0 */
		BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
		BP10G_WRITE_REG(pbpctl_dev_c, ESDP, ((ctrl|BP10G_MCLK_DIR_OUT9)&~(BP10G_MCLK_DATA_OUT9))); 

	} else if (pbpctl_dev->bp_fiber5) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MCLK_DIR5 |
											   BPCTLI_CTRL_EXT_MDIO_DIR5)&~(BPCTLI_CTRL_EXT_MDIO_DATA5|BPCTLI_CTRL_EXT_MCLK_DATA5)));
	} else if (pbpctl_dev->bp_i80) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MDIO_DIR80)&~BPCTLI_CTRL_EXT_MDIO_DATA80));
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
												   BPCTLI_CTRL_EXT_MCLK_DIR80 )&~BPCTLI_CTRL_EXT_MCLK_DATA80));

	} else if (pbpctl_dev->bp_10gb) {
		ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);
		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext |  BP10GB_MDIO_CLR|
													 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR| BP10GB_MDIO_DIR| BP10GB_MDIO_SET|BP10GB_MCLK_SET));

	} else if (pbpctl_dev->bp_540) {
		ctrl= BP10G_READ_REG(pbpctl_dev, ESDP);
		BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl | BP540_MCLK_DIR|
											BP540_MDIO_DIR)&~(BP540_MDIO_DATA|BP540_MCLK_DATA)));

	} else if (!pbpctl_dev->bp_10g) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR |
												   BPCTLI_CTRL_EXT_MDIO_DIR)&~(BPCTLI_CTRL_EXT_MDIO_DATA|BPCTLI_CTRL_EXT_MCLK_DATA)));
	} else {

		//writel((0x0), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
		ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);
		ctrl_ext=BP10G_READ_REG(pbpctl_dev,EODSDP);
		BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext&~(BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT)));
		//BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));

	}

	usec_delay(CMND_INTERVAL);
#ifdef BP_SYNC_FLAG
	spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);
#else
	pbpctl_dev->wdt_busy=0;
#endif

	return ctrl_value;
}

static int wdt_pulse(bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0, ctrl=0;
	bpctl_dev_t *pbpctl_dev_c=NULL;
#ifdef BP_SYNC_FLAG
	unsigned long flags;
#endif
	if (!(pbpctl_dev->bp_caps&BP_CAP))
		return -1;

	if ((pbpctl_dev->bp_10g9)||
		(pbpctl_dev->bp_40g)) {
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
			return -1;
	}
#ifdef BP_SYNC_FLAG
	if (pbpctl_dev->bp_40g)
		spin_lock_irqsave(&pbpctl_dev_c->bypass_wr_lock, flags);
	else
		spin_lock_irqsave(&pbpctl_dev->bypass_wr_lock, flags);
#else 
	if (pbpctl_dev->bp_40g) {
		if (pbpctl_dev_c->wdt_busy==1)
			return -1;
	} else {
		if (pbpctl_dev->wdt_busy==1)
			return -1;
	}
#endif


	if (pbpctl_dev->bp_40g) {

		BPCTLI_WRITE_OFFSET(pbpctl_dev_c, 0x88108, 0x3f80010);
		BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
		BPCTLI_WRITE_OFFSET(pbpctl_dev_c, 0x8810c, 0x3f80010);
		BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT); 

		/* DATA 0 CLK 1 */
		if (pbpctl_dev->func == 0) {
			unsigned char a;

			BPCTLI_WRITE_OFFSET(pbpctl_dev_c, 0x88184, 0x62);
			BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			a = BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			/* if(!(a & BIT_2))
					printk("!!SDP2 stuck low! 0x%x %x\n", a, BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT));*/
		} else {
			unsigned char a;

			BPCTLI_WRITE_OFFSET(pbpctl_dev_c, 0x88184, 0x63);
			BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);

			a = BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			/*if(!(a & BIT_3))
					printk("SDP3 stuck low! 0x%x %x\n", a, BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT));*/

		}

	} else if (pbpctl_dev->bp_10g9) {
		ctrl_ext=BP10G_READ_REG(pbpctl_dev,I2CCTL);
		ctrl= BP10G_READ_REG(pbpctl_dev_c, ESDP);

		/* BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~(BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9)));*/
		/* DATA 0 CLK 0 */
		BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
		BP10G_WRITE_REG(pbpctl_dev_c, ESDP, ((ctrl|BP10G_MCLK_DIR_OUT9)&~(BP10G_MCLK_DATA_OUT9))); 

	} else if (pbpctl_dev->bp_fiber5) {
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL);
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MCLK_DIR5 |
											   BPCTLI_CTRL_EXT_MDIO_DIR5)&~(BPCTLI_CTRL_EXT_MDIO_DATA5|BPCTLI_CTRL_EXT_MCLK_DATA5)));
	} else if (pbpctl_dev->bp_i80) {
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL);
		ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MDIO_DIR80)&~BPCTLI_CTRL_EXT_MDIO_DATA80));
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
												   BPCTLI_CTRL_EXT_MCLK_DIR80 )&~BPCTLI_CTRL_EXT_MCLK_DATA80));
	} else if (pbpctl_dev->bp_540) {
		ctrl_ext =ctrl = BP10G_READ_REG(pbpctl_dev, ESDP);
		BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl | BP540_MCLK_DIR|
											BP540_MDIO_DIR)&~(BP540_MDIO_DATA|BP540_MCLK_DATA)));
	} else if (pbpctl_dev->bp_10gb) {
		ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);
		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext |  BP10GB_MDIO_CLR|
													 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR| BP10GB_MDIO_DIR| BP10GB_MDIO_SET|BP10GB_MCLK_SET));

	} else if (!pbpctl_dev->bp_10g) {

		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR |
												   BPCTLI_CTRL_EXT_MDIO_DIR)&~(BPCTLI_CTRL_EXT_MDIO_DATA|BPCTLI_CTRL_EXT_MCLK_DATA)));
	} else {

		// writel((0x0), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
		ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);
		ctrl_ext=BP10G_READ_REG(pbpctl_dev,EODSDP);
		BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext&~(BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT)));
		//BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));

	}
	if (pbpctl_dev->bp_40g) {
		/*struct timeval tv;
		printk("0x%x \n", BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT)); */


		/* DATA 0 CLK 0 */
		if (pbpctl_dev->func == 0) {
			unsigned char a;

			BPCTLI_WRITE_OFFSET(pbpctl_dev_c, 0x88184, 0x42);
			BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			a = BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			/*if(a & BIT_2)
				printk("SDP2 stuck high! 0x%x %x\n", a, BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT));*/

		} else {
			unsigned char a;

			BPCTLI_WRITE_OFFSET(pbpctl_dev_c, 0x88184, 0x43);
			BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			a = BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			/*if(a & BIT_3)
				printk("SDP3 stuck high! 0x%x %x\n", a, BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT));*/

		}
		/*  do_gettimeofday(&tv);
		  printk("0x%x %d\n", BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT), (long)tv.tv_usec);*/


	} else if (pbpctl_dev->bp_10g9) {
		/*   BP10G_WRITE_REG(pbpctl_dev, I2CCTL, ((ctrl_ext|BP10G_MCLK_DATA_OUT9)&~BP10G_MDIO_DATA_OUT9));*/
		/* DATA 0 CLK 1*/
		BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
		BP10G_WRITE_REG(pbpctl_dev_c, ESDP, (ctrl|BP10G_MCLK_DATA_OUT9|BP10G_MCLK_DIR_OUT9));

	} else if (pbpctl_dev->bp_fiber5) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MCLK_DIR5|
											   BPCTLI_CTRL_EXT_MDIO_DIR5 |
											   BPCTLI_CTRL_EXT_MCLK_DATA5)&~(BPCTLI_CTRL_EXT_MDIO_DATA5)));
	} else if (pbpctl_dev->bp_i80) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MDIO_DIR80 )&~BPCTLI_CTRL_EXT_MDIO_DATA80));
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (ctrl | 
												  BPCTLI_CTRL_EXT_MCLK_DIR80|
												  BPCTLI_CTRL_EXT_MCLK_DATA80));

	} else if (pbpctl_dev->bp_540) {
		BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl | 
											BP540_MDIO_DIR |BP540_MCLK_DIR|BP540_MCLK_DATA)&~BP540_MDIO_DATA));

	} else if (pbpctl_dev->bp_10gb) {
		ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);

		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext |  BP10GB_MDIO_CLR|
													 BP10GB_MCLK_SET)&~(BP10GB_MCLK_DIR| BP10GB_MDIO_DIR| BP10GB_MDIO_SET|BP10GB_MCLK_CLR));



	} else if (!pbpctl_dev->bp_10g)
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR|
												   BPCTLI_CTRL_EXT_MDIO_DIR |
												   BPCTLI_CTRL_EXT_MCLK_DATA)&~(BPCTLI_CTRL_EXT_MDIO_DATA)));
	else {

		//writel((0x8), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
		BP10G_WRITE_REG(pbpctl_dev, EODSDP, ((ctrl_ext|BP10G_MCLK_DATA_OUT)&~BP10G_MDIO_DATA_OUT));
		//BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));

	}

	usec_delay(WDT_INTERVAL);
/*{
	struct timeval tv;
	
		
	do_gettimeofday(&tv);
	printk("0x%x %d\n", BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT), (long)tv.tv_usec);
} */
	if (pbpctl_dev->bp_40g) {
		/* DATA 0 CLK 1 */
		if (pbpctl_dev->func == 0) {
			unsigned char a;

			BPCTLI_WRITE_OFFSET(pbpctl_dev_c, 0x88184, 0x62);
			BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			a = BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);

			/*if(! ( a & BIT_2))
				 printk("!!!SDP2 stuck low! 0x%x %x\n", a, BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT));*/

		} else {
			unsigned char a;

			BPCTLI_WRITE_OFFSET(pbpctl_dev_c, 0x88184, 0x63);
			BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);
			a = BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT);

			/* if(! ( a & BIT_3))
				   printk("SDP3 stuck low! 0x%x %x\n", a, BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT));*/

		}
		/* printk("0x%x \n\n", BP40G_RD_REG(pbpctl_dev_c, GPIO_STAT)); */

	} else if (pbpctl_dev->bp_10g9) {
		/* BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~(BP10G_MCLK_DATA_OUT9|BP10G_MDIO_DATA_OUT9)));*/
		/* DATA 0 CLK 0 */
		BP10G_WRITE_REG(pbpctl_dev, I2CCTL, (ctrl_ext&~BP10G_MDIO_DATA_OUT9));
		BP10G_WRITE_REG(pbpctl_dev_c, ESDP, ((ctrl|BP10G_MCLK_DIR_OUT9)&~(BP10G_MCLK_DATA_OUT9))); 


	} else if (pbpctl_dev->bp_fiber5) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MCLK_DIR5|
											   BPCTLI_CTRL_EXT_MDIO_DIR5)&~(BPCTLI_CTRL_EXT_MCLK_DATA5|BPCTLI_CTRL_EXT_MDIO_DATA5)));
	} else if (pbpctl_dev->bp_i80) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_MDIO_DIR80)&~BPCTLI_CTRL_EXT_MDIO_DATA80));
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl | 
												   BPCTLI_CTRL_EXT_MCLK_DIR80)&~BPCTLI_CTRL_EXT_MCLK_DATA80));

	} else if (pbpctl_dev->bp_540) {
		BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP540_MCLK_DIR| 
											BP540_MDIO_DIR)&~(BP540_MDIO_DATA|BP540_MCLK_DATA)));

	} else if (pbpctl_dev->bp_10gb) {
		ctrl_ext = BP10GB_READ_REG(pbpctl_dev, MISC_REG_SPIO);
		BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_SPIO, (ctrl_ext |  BP10GB_MDIO_CLR|
													 BP10GB_MCLK_CLR)&~(BP10GB_MCLK_DIR| BP10GB_MDIO_DIR| BP10GB_MDIO_SET|BP10GB_MCLK_SET));



	} else if (!pbpctl_dev->bp_10g)
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR|
												   BPCTLI_CTRL_EXT_MDIO_DIR)&~(BPCTLI_CTRL_EXT_MCLK_DATA|BPCTLI_CTRL_EXT_MDIO_DATA)));
	else {

		//writel((0x0), (void *)(((pbpctl_dev)->mem_map) + 0x28)) ;
		BP10G_WRITE_REG(pbpctl_dev, EODSDP, (ctrl_ext&~(BP10G_MCLK_DATA_OUT|BP10G_MDIO_DATA_OUT)));
		//BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl |BP10G_MDIO_DIR)&~BP10G_MDIO_DATA));
	}
	if (pbpctl_dev->wdt_status==WDT_STATUS_EN)
		pbpctl_dev->bypass_wdt_on_time=ticks;
#ifdef BP_SYNC_FLAG
	if (pbpctl_dev->bp_40g)
		spin_unlock_irqrestore(&pbpctl_dev_c->bypass_wr_lock, flags);
	else
		spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);
#endif
	/* usec_delay_bp(CMND_INTERVAL);*/
	return 0;
}  

static void data_pulse(bpctl_dev_t *pbpctl_dev, unsigned char value){

	uint32_t ctrl_ext=0;
#ifdef BP_SYNC_FLAG
	unsigned long flags;
#endif  
	wdt_time_left(pbpctl_dev);
#ifdef BP_SYNC_FLAG
	spin_lock_irqsave(&pbpctl_dev->bypass_wr_lock, flags);
#else 
	pbpctl_dev->wdt_busy=1;
#endif

	ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_SDP6_DIR|
											   BPCTLI_CTRL_EXT_SDP7_DIR)&~(BPCTLI_CTRL_EXT_SDP6_DATA|BPCTLI_CTRL_EXT_SDP7_DATA)));

	usec_delay(INIT_CMND_INTERVAL);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
											   BPCTLI_CTRL_EXT_SDP6_DIR|
											   BPCTLI_CTRL_EXT_SDP7_DIR | BPCTLI_CTRL_EXT_SDP6_DATA)&~(BPCTLI_CTRL_EXT_SDP7_DATA)));
	usec_delay(INIT_CMND_INTERVAL);


	while (value) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ctrl_ext |
						   BPCTLI_CTRL_EXT_SDP6_DIR|
						   BPCTLI_CTRL_EXT_SDP7_DIR|
						   BPCTLI_CTRL_EXT_SDP6_DATA|
						   BPCTLI_CTRL_EXT_SDP7_DATA);
		usec_delay(PULSE_INTERVAL);
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (( ctrl_ext| 
													BPCTLI_CTRL_EXT_SDP6_DIR|
													BPCTLI_CTRL_EXT_SDP7_DIR| 
													BPCTLI_CTRL_EXT_SDP6_DATA)&~BPCTLI_CTRL_EXT_SDP7_DATA));
		usec_delay(PULSE_INTERVAL);
		value--;



	}
	usec_delay(INIT_CMND_INTERVAL-PULSE_INTERVAL);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (( ctrl_ext | 
												BPCTLI_CTRL_EXT_SDP6_DIR|
												BPCTLI_CTRL_EXT_SDP7_DIR)&~(BPCTLI_CTRL_EXT_SDP6_DATA|BPCTLI_CTRL_EXT_SDP7_DATA)));
	usec_delay(WDT_TIME_CNT);
	if (pbpctl_dev->wdt_status==WDT_STATUS_EN)
		pbpctl_dev->bypass_wdt_on_time=ticks;
#ifdef BP_SYNC_FLAG
	spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);
#else
	pbpctl_dev->wdt_busy=0;
#endif


}

static int send_wdt_pulse(bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0;

#ifdef BP_SYNC_FLAG
	unsigned long flags;

	spin_lock_irqsave(&pbpctl_dev->bypass_wr_lock, flags);
#else

	if (pbpctl_dev->wdt_busy==1)
		return -1;
#endif
	wdt_time_left(pbpctl_dev);
	ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT); 

	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ctrl_ext |		/* 1 */
					   BPCTLI_CTRL_EXT_SDP7_DIR | 
					   BPCTLI_CTRL_EXT_SDP7_DATA);
	usec_delay(PULSE_INTERVAL);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext |	  /* 0 */
											   BPCTLI_CTRL_EXT_SDP7_DIR)&~BPCTLI_CTRL_EXT_SDP7_DATA));


	usec_delay(PULSE_INTERVAL);
	if (pbpctl_dev->wdt_status==WDT_STATUS_EN)
		pbpctl_dev->bypass_wdt_on_time=ticks;
#ifdef BP_SYNC_FLAG
	spin_unlock_irqrestore(&pbpctl_dev->bypass_wr_lock, flags);
#endif

	return 0;
}  

static void send_bypass_clear_pulse(bpctl_dev_t *pbpctl_dev, unsigned int value){
	uint32_t ctrl_ext=0;

	ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext |	  /* 0 */
											   BPCTLI_CTRL_EXT_SDP6_DIR)&~BPCTLI_CTRL_EXT_SDP6_DATA));

	usec_delay(PULSE_INTERVAL);
	while (value) {
		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ctrl_ext |		/* 1 */
						   BPCTLI_CTRL_EXT_SDP6_DIR | 
						   BPCTLI_CTRL_EXT_SDP6_DATA);
		usec_delay(PULSE_INTERVAL);
		value--;
	}
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext |	  /* 0 */
											   BPCTLI_CTRL_EXT_SDP6_DIR)&~BPCTLI_CTRL_EXT_SDP6_DATA));
	usec_delay(PULSE_INTERVAL);
}
/*  #endif  OLD_FW */
#ifdef BYPASS_DEBUG

int pulse_set_fn (bpctl_dev_t *pbpctl_dev, unsigned int counter){
	uint32_t ctrl_ext=0;

	if (!pbpctl_dev)
		return -1;

	ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	write_pulse_1(pbpctl_dev,ctrl_ext,counter,counter);

	pbpctl_dev->bypass_wdt_status=0;
	if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER) {
		write_pulse_1(pbpctl_dev,ctrl_ext,counter,counter);
	} else {
		wdt_time_left(pbpctl_dev);
		if (pbpctl_dev->wdt_status==WDT_STATUS_EN) {
			pbpctl_dev->wdt_status=0;
			data_pulse(pbpctl_dev,counter);
			pbpctl_dev->wdt_status= WDT_STATUS_EN;
			pbpctl_dev->bypass_wdt_on_time=ticks;

		} else
			data_pulse(pbpctl_dev,counter);
	}

	return 0;
}


int zero_set_fn (bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0, ctrl_value=0;
	if (!pbpctl_dev)
		return -1;

	if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER) {
		printf("zero_set");

		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);

		BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl_ext | 
												   BPCTLI_CTRL_EXT_MCLK_DIR)&~(BPCTLI_CTRL_EXT_MCLK_DATA|BPCTLI_CTRL_EXT_MDIO_DIR|BPCTLI_CTRL_EXT_MDIO_DATA)));

	}
	return ctrl_value;
}


int pulse_get2_fn (bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0, ctrl_value=0;
	if (!pbpctl_dev)
		return -1;

	if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER) {
		printf("pulse_get_fn\n");
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
		ctrl_value=read_pulse_2(pbpctl_dev,ctrl_ext);
		printf("read:%d\n",ctrl_value);
	}
	return ctrl_value;
}

int pulse_get1_fn (bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0, ctrl_value=0;
	if (!pbpctl_dev)
		return -1;
	if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER) {

		printf("pulse_get_fn\n");

		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
		ctrl_value=read_pulse_1(pbpctl_dev,ctrl_ext);
		printf("read:%d\n",ctrl_value);
	}
	return ctrl_value;
}


int gpio6_set_fn (bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0;

	ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ctrl_ext |
					   BPCTLI_CTRL_EXT_SDP6_DIR  |
					   BPCTLI_CTRL_EXT_SDP6_DATA);
	return 0;
}



int gpio7_set_fn (bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0;

	ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ctrl_ext |
					   BPCTLI_CTRL_EXT_SDP7_DIR |
					   BPCTLI_CTRL_EXT_SDP7_DATA);
	return 0;
}

int gpio7_clear_fn (bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0;

	ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (( ctrl_ext |
												BPCTLI_CTRL_EXT_SDP7_DIR) & ~BPCTLI_CTRL_EXT_SDP7_DATA));
	return 0;
}

int gpio6_clear_fn (bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0;

	ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (( ctrl_ext |
												BPCTLI_CTRL_EXT_SDP6_DIR) & ~BPCTLI_CTRL_EXT_SDP6_DATA));
	return 0;
}
#endif /*BYPASS_DEBUG*/

static bpctl_dev_t *get_status40_port_fn(bpctl_dev_t *pbpctl_dev) {
	int idx_dev=0;

	if (pbpctl_dev==NULL)
		return NULL;

	if (pbpctl_dev->func != 0) {
		for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev != NULL)&&(idx_dev < device_num)); idx_dev++) {
			if ((bpctl_dev_arr[idx_dev].bus == pbpctl_dev->bus)&&
				(bpctl_dev_arr[idx_dev].slot == pbpctl_dev->slot)&&
				(bpctl_dev_arr[idx_dev].func == 0)) {

				return(&(bpctl_dev_arr[idx_dev]));
			}
		}
	} else
		return pbpctl_dev;
	return NULL;
} 


static bpctl_dev_t *get_status_port_fn(bpctl_dev_t *pbpctl_dev) {
	int idx_dev=0;

	if (pbpctl_dev==NULL) {
		return NULL;
	}

	if (pbpctl_dev->bp_40g) {
		if (pbpctl_dev->func != 0) {
			for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev != NULL)&&(idx_dev < device_num)); idx_dev++) {
				if ((bpctl_dev_arr[idx_dev].bus == pbpctl_dev->bus)&&
					(bpctl_dev_arr[idx_dev].slot == pbpctl_dev->slot)&&
					(bpctl_dev_arr[idx_dev].func == 0)) {

					return(&(bpctl_dev_arr[idx_dev]));
				}
			}
		} else
			return pbpctl_dev;

	} else { 
		if ((pbpctl_dev->func==0)||(pbpctl_dev->func==2)) {
			for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev!=NULL)&&(idx_dev<device_num)); idx_dev++) {
				if ((bpctl_dev_arr[idx_dev].bus==pbpctl_dev->bus)&&
					(bpctl_dev_arr[idx_dev].slot==pbpctl_dev->slot)&&
					((bpctl_dev_arr[idx_dev].func==1)&&(pbpctl_dev->func==0))) {

					return(&(bpctl_dev_arr[idx_dev]));
				}
				if ((bpctl_dev_arr[idx_dev].bus==pbpctl_dev->bus)&&
					(bpctl_dev_arr[idx_dev].slot==pbpctl_dev->slot)&&
					((bpctl_dev_arr[idx_dev].func==3)&&(pbpctl_dev->func==2))) {

					return(&(bpctl_dev_arr[idx_dev]));
				}
			}
		}
	}
	return NULL;
}



static bpctl_dev_t *get_master_port_fn(bpctl_dev_t *pbpctl_dev) {
	int idx_dev=0;

	if (pbpctl_dev==NULL) {
		return NULL;
	}

	if ((pbpctl_dev->func==1)||(pbpctl_dev->func==3)) {
		for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev!=NULL)&&(idx_dev<device_num)); idx_dev++) {
			if ((bpctl_dev_arr[idx_dev].bus==pbpctl_dev->bus)&&
				(bpctl_dev_arr[idx_dev].slot==pbpctl_dev->slot)&&
				((bpctl_dev_arr[idx_dev].func==0)&&(pbpctl_dev->func==1))) {

				return(&(bpctl_dev_arr[idx_dev]));
			}
			if ((bpctl_dev_arr[idx_dev].bus==pbpctl_dev->bus)&&
				(bpctl_dev_arr[idx_dev].slot==pbpctl_dev->slot)&&
				((bpctl_dev_arr[idx_dev].func==2)&&(pbpctl_dev->func==3))) {

				return(&(bpctl_dev_arr[idx_dev]));
			}
		}
	}
	return NULL;
}




/**************************************/
/**************INTEL API***************/
/**************************************/

static void write_data_port_int(bpctl_dev_t *pbpctl_dev, unsigned char ctrl_value){
	uint32_t value;

	value = BPCTL_READ_REG(pbpctl_dev, CTRL);
/* Make SDP0 Pin Directonality to Output */
	value |= BPCTLI_CTRL_SDP0_DIR;
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, value);

	value &= ~BPCTLI_CTRL_SDP0_DATA;
	value |= ((ctrl_value & 0x1) << BPCTLI_CTRL_SDP0_SHIFT);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, value);

	value = (BPCTL_READ_REG(pbpctl_dev, CTRL_EXT));
/* Make SDP2 Pin Directonality to Output */
	value |= BPCTLI_CTRL_EXT_SDP6_DIR;
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, value);

	value &= ~BPCTLI_CTRL_EXT_SDP6_DATA;
	value |= (((ctrl_value & 0x2) >> 1) << BPCTLI_CTRL_EXT_SDP6_SHIFT);
	BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, value);

}

static int write_data_int(bpctl_dev_t *pbpctl_dev, unsigned char value){
	bpctl_dev_t *pbpctl_dev_b=NULL;

	if (!(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
		return -1;
	pbpctl_dev->wdt_busy=1;
	write_data_port_int(pbpctl_dev, value&0x3);
	write_data_port_int(pbpctl_dev_b,((value & 0xc) >> 2));
	pbpctl_dev->wdt_busy=0;

	return 0;
}   

static int wdt_pulse_int(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->wdt_busy==1)
		return -1;

	if ((write_data_int(pbpctl_dev, RESET_WDT_INT))<0)
		return -1;
	msec_delay_bp(CMND_INTERVAL_INT);
	if ((write_data_int(pbpctl_dev, CMND_OFF_INT))<0)
		return -1;
	msec_delay_bp(CMND_INTERVAL_INT);

	if (pbpctl_dev->wdt_status==WDT_STATUS_EN)
		pbpctl_dev->bypass_wdt_on_time=ticks;

	return 0;
}


/*************************************/
/************* COMMANDS **************/
/*************************************/


/* CMND_ON  0x4 (100)*/
static int cmnd_on(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
			return 0;
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
			write_data(pbpctl_dev,CMND_ON);
		else
			data_pulse(pbpctl_dev,CMND_ON);
		ret=0;
	}
	return ret;
}


/* CMND_OFF  0x2 (10)*/
static int cmnd_off(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			write_data_int(pbpctl_dev,CMND_OFF_INT);
			msec_delay_bp(CMND_INTERVAL_INT);
		} else if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
			write_data(pbpctl_dev,CMND_OFF);
		else
			data_pulse(pbpctl_dev,CMND_OFF);
		ret=0;
	};
	return ret;
}

/* BYPASS_ON (0xa)*/
static int bypass_on(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;

	if (pbpctl_dev->bp_caps&BP_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			write_data_int(pbpctl_dev,BYPASS_ON_INT);
			msec_delay_bp(BYPASS_DELAY_INT);
			pbpctl_dev->bp_status_un=0;
		} else if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			write_data(pbpctl_dev,BYPASS_ON);
			if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
				msec_delay_bp(LATCH_DELAY);
		} else
			data_pulse(pbpctl_dev,BYPASS_ON);
		ret=0;
	};
	return ret;
}

/* BYPASS_OFF (0x8 111)*/
static int bypass_off(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;

	if (pbpctl_dev->bp_caps&BP_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			write_data_int(pbpctl_dev,DIS_BYPASS_CAP_INT);
			msec_delay_bp(BYPASS_DELAY_INT);
			pbpctl_dev->bp_status_un=0;
		} else if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER) {
			write_data(pbpctl_dev,BYPASS_OFF);
			if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
				msec_delay_bp(LATCH_DELAY);
		} else






			data_pulse(pbpctl_dev,BYPASS_OFF);
		ret=0;
	}
	return ret;
}

/* TAP_OFF (0x9)*/
static int tap_off(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;
	if ((pbpctl_dev->bp_caps&TAP_CAP)&&(pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)) {


		write_data(pbpctl_dev,TAP_OFF);
		msec_delay_bp(LATCH_DELAY);
		ret=0;
	};
	return ret;
}

/* TAP_ON (0xb)*/
static int tap_on(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;
	if ((pbpctl_dev->bp_caps&TAP_CAP)&&(pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)) {


		write_data(pbpctl_dev,TAP_ON);
		msec_delay_bp(LATCH_DELAY);
		ret=0;
	};
	return ret;
}

/* DISC_OFF (0x9)*/
static int disc_off(bpctl_dev_t *pbpctl_dev){
	int ret=0;
	if ((pbpctl_dev->bp_caps&DISC_CAP)&&(pbpctl_dev->bp_ext_ver>=0x8)) {


		write_data(pbpctl_dev,DISC_OFF);
		msec_delay_bp(LATCH_DELAY);
	} else	ret=BP_NOT_CAP;


	return ret;
}

/* DISC_ON (0xb)*/
static int disc_on(bpctl_dev_t *pbpctl_dev){
	int ret=0;
	if ((pbpctl_dev->bp_caps&DISC_CAP)&&(pbpctl_dev->bp_ext_ver>=0x8))
	{
		write_data(pbpctl_dev,/*DISC_ON*/0x85);
		msec_delay_bp(LATCH_DELAY);
	} else	ret=BP_NOT_CAP;
	return ret;
}

/*TWO_PORT_LINK_HW_EN (0xe)*/
static int tpl_hw_on (bpctl_dev_t *pbpctl_dev){
	int ret=0, ctrl=0;
	bpctl_dev_t *pbpctl_dev_b=NULL;

	if (!(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
		return BP_NOT_CAP;

	if (pbpctl_dev->bp_caps_ex&TPL2_CAP_EX)
	{
		cmnd_on(pbpctl_dev);
		write_data(pbpctl_dev,TPL2_ON);
		msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
		cmnd_off(pbpctl_dev);
		return ret;
	}

	if (TPL_IF_SERIES(pbpctl_dev->subdevice))
	{
		ctrl = BPCTL_READ_REG(pbpctl_dev_b, CTRL);
		BPCTL_BP_WRITE_REG(pbpctl_dev_b, CTRL, ((ctrl|BPCTLI_CTRL_SWDPIO0)&~BPCTLI_CTRL_SWDPIN0));
	} else ret=BP_NOT_CAP;
	return ret;
}


/*TWO_PORT_LINK_HW_DIS (0xc)*/
static int tpl_hw_off (bpctl_dev_t *pbpctl_dev){
	int ret=0, ctrl=0;
	bpctl_dev_t *pbpctl_dev_b=NULL;

	if (!(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
		return BP_NOT_CAP;
	if (pbpctl_dev->bp_caps_ex&TPL2_CAP_EX)
	{
		cmnd_on(pbpctl_dev);
		write_data(pbpctl_dev,TPL2_OFF);
		msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
		cmnd_off(pbpctl_dev);
		return ret;
	}
	if (TPL_IF_SERIES(pbpctl_dev->subdevice))
	{
		ctrl = BPCTL_READ_REG(pbpctl_dev_b, CTRL);
		BPCTL_BP_WRITE_REG(pbpctl_dev_b, CTRL, (ctrl|BPCTLI_CTRL_SWDPIO0|BPCTLI_CTRL_SWDPIN0));
	} else ret=BP_NOT_CAP;
	return ret;
}



/* WDT_OFF (0x6 110)*/
static int wdt_off(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;

	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			bypass_off(pbpctl_dev);
		} else if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
			write_data(pbpctl_dev,WDT_OFF);
		else
			data_pulse(pbpctl_dev,WDT_OFF);
		pbpctl_dev->wdt_status=WDT_STATUS_DIS;
		ret=0;
	};
	return ret;
}

/* WDT_ON (0x10)*/

/***Global***/
static unsigned int 
wdt_val_array[]={1000, 1500, 2000, 3000, 4000, 8000, 16000, 32000, 0} ;

static int wdt_on(bpctl_dev_t *pbpctl_dev, unsigned int timeout){

	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
	{
		unsigned int pulse=0, temp_value=0, temp_cnt=0;
		pbpctl_dev->wdt_status=0; 

		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			for (;wdt_val_array[temp_cnt];temp_cnt++)
				if (timeout<=wdt_val_array[temp_cnt])
					break;

			if (!wdt_val_array[temp_cnt])
				temp_cnt--;

			timeout=wdt_val_array[temp_cnt];
			temp_cnt+=0x7;

			write_data_int(pbpctl_dev,DIS_BYPASS_CAP_INT);
			msec_delay_bp(BYPASS_DELAY_INT);
			pbpctl_dev->bp_status_un=0;
			write_data_int(pbpctl_dev,temp_cnt);
			pbpctl_dev->bypass_wdt_on_time=ticks;
			msec_delay_bp(CMND_INTERVAL_INT);
			pbpctl_dev->bypass_timer_interval=timeout;  
		} else
		{
			timeout=(timeout<TIMEOUT_UNIT?TIMEOUT_UNIT:(timeout>WDT_TIMEOUT_MAX?WDT_TIMEOUT_MAX:timeout));
			temp_value=timeout/100;
			while ((temp_value>>=1))
				temp_cnt++;
			if (timeout > ((1<<temp_cnt)*100))
				temp_cnt++;
			pbpctl_dev->bypass_wdt_on_time=ticks;
			pulse=(WDT_ON | temp_cnt);
			if (pbpctl_dev->bp_ext_ver==OLD_IF_VER)
				data_pulse(pbpctl_dev,pulse);
			else
				write_data(pbpctl_dev,pulse);
			pbpctl_dev->bypass_timer_interval=(1<<temp_cnt)*100;
		}
		pbpctl_dev->wdt_status=WDT_STATUS_EN;
		return 0;    
	}
	return BP_NOT_CAP;
}

static int32_t bp75_read_phy_reg_mdic(bpctl_dev_t *pbpctl_dev, uint32_t offset, uint16_t *data)
{
	uint32_t i, mdic = 0;
	int32_t ret_val = 0;
	uint32_t phy_addr = 1;


	mdic = ((offset << BPCTLI_MDIC_REG_SHIFT) |
			(phy_addr << BPCTLI_MDIC_PHY_SHIFT) |
			(BPCTLI_MDIC_OP_READ));

	BPCTL_BP_WRITE_REG(pbpctl_dev, MDIC, mdic);

	for (i = 0; i < (BPCTLI_GEN_POLL_TIMEOUT * 3); i++) {
		usec_delay(50);
		mdic = BPCTL_READ_REG(pbpctl_dev, MDIC);
		if (mdic & BPCTLI_MDIC_READY)
			break;
	}
	if (!(mdic & BPCTLI_MDIC_READY)) {
		printf("bpctl_mod: MDI Read did not complete\n");
		ret_val = -1;
		goto out;
	}
	if (mdic & BPCTLI_MDIC_ERROR) {
		printf("bpctl_mod: MDI Error\n");
		ret_val = -1;
		goto out;
	}
	*data = (uint16_t) mdic;

	out:
	return ret_val;
}

static int32_t bp75_write_phy_reg_mdic(bpctl_dev_t *pbpctl_dev, uint32_t offset, uint16_t data)
{
	uint32_t i, mdic = 0;
	int32_t ret_val = 0;
	uint32_t phy_addr = 1;



	mdic = (((uint32_t)data) |
			(offset << BPCTLI_MDIC_REG_SHIFT) |
			(phy_addr << BPCTLI_MDIC_PHY_SHIFT) |
			(BPCTLI_MDIC_OP_WRITE));

	BPCTL_BP_WRITE_REG(pbpctl_dev, MDIC, mdic);

	for (i = 0; i < (BPCTLI_GEN_POLL_TIMEOUT * 3); i++) {
		usec_delay(50);
		mdic = BPCTL_READ_REG(pbpctl_dev, MDIC);
		if (mdic & BPCTLI_MDIC_READY)
			break;
	}
	if (!(mdic & BPCTLI_MDIC_READY)) {
		printf("bpctl_mod: MDI Write did not complete\n");
		ret_val = -1;
		goto out;
	}
	if (mdic & BPCTLI_MDIC_ERROR) {
		printf("bpctl_mod: MDI Error\n");
		ret_val = -1;
		goto out;
	}

	out:
	return ret_val;
}

static void bp75_put_hw_semaphore_generic(bpctl_dev_t *pbpctl_dev)
{
	uint32_t swsm;


	swsm = BPCTL_READ_REG(pbpctl_dev, SWSM);

	swsm &= ~(BPCTLI_SWSM_SMBI | BPCTLI_SWSM_SWESMBI);

	BPCTL_BP_WRITE_REG(pbpctl_dev, SWSM, swsm);
}


static int32_t bp75_get_hw_semaphore_generic(bpctl_dev_t *pbpctl_dev)
{
	uint32_t swsm;
	int32_t ret_val = 0;
	int32_t timeout = 8192 + 1;
	int32_t i = 0;


	/* Get the SW semaphore */
	while (i < timeout) {
		swsm = BPCTL_READ_REG(pbpctl_dev, SWSM);
		if (!(swsm & BPCTLI_SWSM_SMBI))
			break;

		usec_delay(50);
		i++;
	}

	if (i == timeout) {
		printf("bpctl_mod: Driver can't access device - SMBI bit is set.\n");
		ret_val = -1;
		goto out;
	}

	/* Get the FW semaphore. */
	for (i = 0; i < timeout; i++) {
		swsm = BPCTL_READ_REG(pbpctl_dev, SWSM);
		BPCTL_BP_WRITE_REG(pbpctl_dev, SWSM, swsm | BPCTLI_SWSM_SWESMBI);

		/* Semaphore acquired if bit latched */
		if (BPCTL_READ_REG(pbpctl_dev, SWSM) & BPCTLI_SWSM_SWESMBI)
			break;

		usec_delay(50);
	}

	if (i == timeout) {
		/* Release semaphores */
		bp75_put_hw_semaphore_generic(pbpctl_dev);
		printf("bpctl_mod: Driver can't access the NVM\n");
		ret_val = -1;
		goto out;
	}

	out:
	return ret_val;
}


static int32_t bp75_acquire_phy(bpctl_dev_t *pbpctl_dev)
{
	uint16_t mask = BPCTLI_SWFW_PHY0_SM;
	uint32_t swfw_sync;
	uint32_t swmask ;
	uint32_t fwmask ;
	int32_t ret_val = 0;
	int32_t i = 0, timeout = 200; 


	if ((bpctl_dev_arr->func==1)||(bpctl_dev_arr->func==3))
		mask = BPCTLI_SWFW_PHY1_SM;

	swmask = mask;
	fwmask = mask << 16;

	while (i < timeout) {
		if (bp75_get_hw_semaphore_generic(pbpctl_dev)) {
			ret_val = -1;
			goto out;
		}

		swfw_sync = BPCTL_READ_REG(pbpctl_dev, SW_FW_SYNC);
		if (!(swfw_sync & (fwmask | swmask)))
			break;

		bp75_put_hw_semaphore_generic(pbpctl_dev);
		msec_delay_bp(5);
		i++;
	}

	if (i == timeout) {
		printf("bpctl_mod: Driver can't access resource, SW_FW_SYNC timeout.\n");
		ret_val = -1;
		goto out;
	}

	swfw_sync |= swmask;
	BPCTL_BP_WRITE_REG(pbpctl_dev, SW_FW_SYNC, swfw_sync);

	bp75_put_hw_semaphore_generic(pbpctl_dev);

	out:
	return ret_val;
}

static void bp75_release_phy(bpctl_dev_t *pbpctl_dev)
{
	uint16_t mask = BPCTLI_SWFW_PHY0_SM;
	uint32_t swfw_sync;



	if ((bpctl_dev_arr->func==1)||(bpctl_dev_arr->func==3))
		mask = BPCTLI_SWFW_PHY1_SM;

	while (bp75_get_hw_semaphore_generic(pbpctl_dev) != 0);
	/* Empty */

	swfw_sync = BPCTL_READ_REG(pbpctl_dev, SW_FW_SYNC);
	swfw_sync &= ~mask;
	BPCTL_BP_WRITE_REG(pbpctl_dev, SW_FW_SYNC, swfw_sync);

	bp75_put_hw_semaphore_generic(pbpctl_dev);
} 



static int32_t bp75_read_phy_reg( bpctl_dev_t *pbpctl_dev, uint32_t offset, uint16_t *data)
{
	int32_t ret_val = 0;


	ret_val = bp75_acquire_phy(pbpctl_dev);
	if (ret_val)
		goto out;

	if (offset > BPCTLI_MAX_PHY_MULTI_PAGE_REG) {
		ret_val = bp75_write_phy_reg_mdic(pbpctl_dev,
										  BPCTLI_IGP01E1000_PHY_PAGE_SELECT,
										  (uint16_t)offset);
		if (ret_val)
			goto release;
	}

	ret_val = bp75_read_phy_reg_mdic(pbpctl_dev, BPCTLI_MAX_PHY_REG_ADDRESS & offset,
									 data);

	release:
	bp75_release_phy(pbpctl_dev);
	out:
	return ret_val;
}

static int32_t bp75_write_phy_reg(bpctl_dev_t *pbpctl_dev, uint32_t offset, uint16_t data)
{
	int32_t ret_val = 0;


	ret_val = bp75_acquire_phy(pbpctl_dev);
	if (ret_val)
		goto out;

	if (offset > BPCTLI_MAX_PHY_MULTI_PAGE_REG) {
		ret_val = bp75_write_phy_reg_mdic(pbpctl_dev,
										  BPCTLI_IGP01E1000_PHY_PAGE_SELECT,
										  (uint16_t)offset);
		if (ret_val)
			goto release;
	}

	ret_val = bp75_write_phy_reg_mdic(pbpctl_dev, BPCTLI_MAX_PHY_REG_ADDRESS & offset,
									  data);

	release:
	bp75_release_phy(pbpctl_dev);

	out:
	return ret_val;
}



/* SET_TX  (non-Bypass command :)) */

static int set_tx (bpctl_dev_t *pbpctl_dev, int tx_state){
	int ret=0, ctrl=0;  
	bpctl_dev_t *pbpctl_dev_m;
	if ((is_bypass_fn(pbpctl_dev))==1)
		pbpctl_dev_m=pbpctl_dev;
	else
		pbpctl_dev_m=get_master_port_fn(pbpctl_dev);
	if (pbpctl_dev_m==NULL)
		return BP_NOT_CAP;
	if (pbpctl_dev_m->bp_caps_ex&DISC_PORT_CAP_EX) {
		ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL);
		if (!tx_state){
			if (pbpctl_dev->bp_540) {
				ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);
				BP10G_WRITE_REG(pbpctl_dev, ESDP,(ctrl|BP10G_SDP1_DIR|BP10G_SDP1_DATA));

			} else{
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL,(ctrl|BPCTLI_CTRL_SDP1_DIR|BPCTLI_CTRL_SWDPIN1));
			}
		} else{
			if (pbpctl_dev->bp_540) {
				ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);
				BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl|BP10G_SDP1_DIR)&~BP10G_SDP1_DATA));
			} else{
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl|BPCTLI_CTRL_SDP1_DIR)&~BPCTLI_CTRL_SWDPIN1));
				return ret;
			}
			return ret;

		}
	} else if (pbpctl_dev->bp_caps&TX_CTL_CAP) {
		if (PEG5_IF_SERIES(pbpctl_dev->subdevice)) {
			if (tx_state) {
				uint16_t mii_reg;
				if (!(ret=bp75_read_phy_reg(pbpctl_dev, BPCTLI_PHY_CONTROL, &mii_reg))) {
					if (mii_reg & BPCTLI_MII_CR_POWER_DOWN) {
						ret=bp75_write_phy_reg(pbpctl_dev, BPCTLI_PHY_CONTROL, mii_reg&~BPCTLI_MII_CR_POWER_DOWN);
					}
				}
			} else {
				uint16_t mii_reg;
				if (!(ret=bp75_read_phy_reg(pbpctl_dev, BPCTLI_PHY_CONTROL, &mii_reg))) {

					mii_reg |= BPCTLI_MII_CR_POWER_DOWN;
					ret=bp75_write_phy_reg(pbpctl_dev, BPCTLI_PHY_CONTROL, mii_reg);
				}
			}

		}
		if (pbpctl_dev->bp_fiber5) {
			ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);

		} else if (pbpctl_dev->bp_10gb)
			ctrl = BP10GB_READ_REG(pbpctl_dev, MISC_REG_GPIO);


		else if (!pbpctl_dev->bp_10g)
			ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL);
		else
			ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);

		if (!tx_state)
			if (pbpctl_dev->bp_10g9) {
				BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl |BP10G_SDP3_DATA|BP10G_SDP3_DIR));


			} else if (pbpctl_dev->bp_fiber5) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT,(ctrl|BPCTLI_CTRL_EXT_SDP6_DIR|BPCTLI_CTRL_EXT_SDP6_DATA));


			} else if (pbpctl_dev->bp_10gb) {
				if ((pbpctl_dev->func==1)||(pbpctl_dev->func==3))
					BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_GPIO,(ctrl|BP10GB_GPIO0_SET_P1)&~(BP10GB_GPIO0_CLR_P1|BP10GB_GPIO0_OE_P1));
				else
					BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_GPIO,(ctrl|BP10GB_GPIO0_OE_P0|BP10GB_GPIO0_SET_P0));

			} else if (pbpctl_dev->bp_i80) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL,(ctrl|BPCTLI_CTRL_SDP1_DIR|BPCTLI_CTRL_SWDPIN1));

			} else if (pbpctl_dev->bp_540) {
				ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);
				BP10G_WRITE_REG(pbpctl_dev, ESDP,(ctrl|BP10G_SDP1_DIR|BP10G_SDP1_DATA));

			}


			else if (!pbpctl_dev->bp_10g)
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL,(ctrl|BPCTLI_CTRL_SWDPIO0|BPCTLI_CTRL_SWDPIN0));

			else
				BP10G_WRITE_REG(pbpctl_dev, ESDP, (ctrl |BP10G_SDP0_DATA|BP10G_SDP0_DIR));


		else {
			if (pbpctl_dev->bp_10g9) {
				BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl|BP10G_SDP3_DIR) &~BP10G_SDP3_DATA));


			} else if (pbpctl_dev->bp_fiber5) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, ((ctrl|BPCTLI_CTRL_EXT_SDP6_DIR)&~BPCTLI_CTRL_EXT_SDP6_DATA));


			} else if (pbpctl_dev->bp_10gb) {

				if ((bpctl_dev_arr->func==1)||(bpctl_dev_arr->func==3))
					BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_GPIO,(ctrl|BP10GB_GPIO0_CLR_P1)&~(BP10GB_GPIO0_SET_P1|BP10GB_GPIO0_OE_P1));
				else
					BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_GPIO,(ctrl|BP10GB_GPIO0_OE_P0|BP10GB_GPIO0_CLR_P0));




			} else if (pbpctl_dev->bp_i80) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl|BPCTLI_CTRL_SDP1_DIR)&~BPCTLI_CTRL_SWDPIN1));
			} else if (pbpctl_dev->bp_540) {
				ctrl=BP10G_READ_REG(pbpctl_dev,ESDP);
				BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl|BP10G_SDP1_DIR)&~BP10G_SDP1_DATA));
			}


			else if (!pbpctl_dev->bp_10g) {
				BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, ((ctrl|BPCTLI_CTRL_SWDPIO0)&~BPCTLI_CTRL_SWDPIN0));
				if (!PEGF_IF_SERIES(pbpctl_dev->subdevice)) {
					BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL, 
									   (ctrl&~(BPCTLI_CTRL_SDP0_DATA|BPCTLI_CTRL_SDP0_DIR)));
				}
			} else
				BP10G_WRITE_REG(pbpctl_dev, ESDP, ((ctrl|BP10G_SDP0_DIR) &~BP10G_SDP0_DATA));


		}

	} else ret=BP_NOT_CAP;
	return ret;

}


/*RESET_CONT 0x20 */
static int reset_cont (bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
			return BP_NOT_CAP;
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
			write_data(pbpctl_dev,RESET_CONT);
		else
			data_pulse(pbpctl_dev,RESET_CONT);
		ret=0;
	};
	return ret;
}

/*DIS_BYPASS_CAP 0x22 */
static int dis_bypass_cap(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&BP_DIS_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			write_data_int(pbpctl_dev,DIS_BYPASS_CAP_INT);
			msec_delay_bp(BYPASS_DELAY_INT);
		} else
		{
			write_data(pbpctl_dev,BYPASS_OFF);
			msec_delay_bp(LATCH_DELAY);
			write_data(pbpctl_dev,DIS_BYPASS_CAP);
			msec_delay_bp(BYPASS_CAP_DELAY);
		}
		return 0;
	}
	return BP_NOT_CAP;
}


/*EN_BYPASS_CAP 0x24 */
static int en_bypass_cap(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&BP_DIS_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			write_data_int(pbpctl_dev,PWROFF_BYPASS_ON_INT);
			msec_delay_bp(BYPASS_DELAY_INT);
		} else
		{
			write_data(pbpctl_dev,EN_BYPASS_CAP);
			msec_delay_bp(BYPASS_CAP_DELAY);
		}
		return 0;
	}
	return BP_NOT_CAP;
}

/* BYPASS_STATE_PWRON 0x26*/
static int bypass_state_pwron(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&BP_PWUP_CTL_CAP)
	{
		write_data(pbpctl_dev,BYPASS_STATE_PWRON);
		if (pbpctl_dev->bp_ext_ver==PXG2BPI_VER)
			msec_delay_bp(DFLT_PWRON_DELAY);
		else msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
		return 0;
	}
	return BP_NOT_CAP;
}

/* NORMAL_STATE_PWRON 0x28*/
static int normal_state_pwron(bpctl_dev_t *pbpctl_dev){
	if ((pbpctl_dev->bp_caps&BP_PWUP_CTL_CAP)||(pbpctl_dev->bp_caps&TAP_PWUP_CTL_CAP))
	{
		write_data(pbpctl_dev,NORMAL_STATE_PWRON);
		if (pbpctl_dev->bp_ext_ver==PXG2BPI_VER)
			msec_delay_bp(DFLT_PWRON_DELAY);
		else msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
		return 0;
	}
	return BP_NOT_CAP;
}

/* BYPASS_STATE_PWROFF 0x27*/
static int bypass_state_pwroff(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&BP_PWOFF_CTL_CAP)
	{
		write_data(pbpctl_dev,BYPASS_STATE_PWROFF);
		msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
		return 0;
	}
	return BP_NOT_CAP;
}

/* NORMAL_STATE_PWROFF 0x29*/
static int normal_state_pwroff(bpctl_dev_t *pbpctl_dev){
	if ((pbpctl_dev->bp_caps&BP_PWOFF_CTL_CAP))
	{
		write_data(pbpctl_dev,NORMAL_STATE_PWROFF);
		msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
		return 0;
	}
	return BP_NOT_CAP;
}

/*TAP_STATE_PWRON 0x2a*/
static int tap_state_pwron(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&TAP_PWUP_CTL_CAP)
	{
		write_data(pbpctl_dev,TAP_STATE_PWRON);
		msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
		return 0;
	}
	return BP_NOT_CAP;
}

/*DIS_TAP_CAP 0x2c*/
static int dis_tap_cap(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&TAP_DIS_CAP)
	{
		write_data(pbpctl_dev,DIS_TAP_CAP);
		msec_delay_bp(BYPASS_CAP_DELAY);
		return 0;
	}
	return BP_NOT_CAP;
}

/*EN_TAP_CAP 0x2e*/
static int en_tap_cap(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&TAP_DIS_CAP)
	{
		write_data(pbpctl_dev,EN_TAP_CAP);
		msec_delay_bp(BYPASS_CAP_DELAY);
		return 0;
	}
	return BP_NOT_CAP;
}
/*DISC_STATE_PWRON 0x2a*/
static int disc_state_pwron(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&DISC_PWUP_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
		{
			write_data(pbpctl_dev,DISC_STATE_PWRON);
			msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
			return BP_OK;
		}
	}
	return BP_NOT_CAP;
}

/*DIS_DISC_CAP 0x2c*/
static int dis_disc_cap(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&DISC_DIS_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
		{
			write_data(pbpctl_dev,DIS_DISC_CAP);
			msec_delay_bp(BYPASS_CAP_DELAY);
			return BP_OK;
		}
	}
	return BP_NOT_CAP;
}


/*EN_TAP_CAP 0x2e*/
static int en_disc_cap(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&DISC_DIS_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
		{
			write_data(pbpctl_dev,EN_DISC_CAP);
			msec_delay_bp(BYPASS_CAP_DELAY);
			return BP_OK;
		}
	}
	return BP_NOT_CAP;
}


static int std_nic_on(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&STD_NIC_CAP)
	{

		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			write_data_int(pbpctl_dev,DIS_BYPASS_CAP_INT);
			msec_delay_bp(BYPASS_DELAY_INT);
			pbpctl_dev->bp_status_un=0;
			return BP_OK;
		}

		if (pbpctl_dev->bp_ext_ver>=0x8)
		{
			write_data(pbpctl_dev,STD_NIC_ON);
			msec_delay_bp(BYPASS_CAP_DELAY);
			return BP_OK;

		}


		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			wdt_off(pbpctl_dev);

			if (pbpctl_dev->bp_caps&BP_CAP)
			{
				write_data(pbpctl_dev,BYPASS_OFF);
				msec_delay_bp(LATCH_DELAY);
			}

			if (pbpctl_dev->bp_caps&TAP_CAP)
			{
				write_data(pbpctl_dev,TAP_OFF);
				msec_delay_bp(LATCH_DELAY);
			}

			write_data(pbpctl_dev,NORMAL_STATE_PWRON);
			if (pbpctl_dev->bp_ext_ver==PXG2BPI_VER)
				msec_delay_bp(DFLT_PWRON_DELAY);
			else msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);

			if (pbpctl_dev->bp_caps&BP_DIS_CAP)
			{
				write_data(pbpctl_dev,DIS_BYPASS_CAP);
				msec_delay_bp(BYPASS_CAP_DELAY);
			}

			if (pbpctl_dev->bp_caps&TAP_DIS_CAP)
			{
				write_data(pbpctl_dev,DIS_TAP_CAP);
				msec_delay_bp(BYPASS_CAP_DELAY);

			}
			return 0;
		}
	}
	return BP_NOT_CAP;
}

static int std_nic_off(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&STD_NIC_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		{
			write_data_int(pbpctl_dev,PWROFF_BYPASS_ON_INT);
			msec_delay_bp(BYPASS_DELAY_INT);
			return BP_OK;
		}
		if (pbpctl_dev->bp_ext_ver>=0x8)
		{
			write_data(pbpctl_dev,STD_NIC_OFF);
			msec_delay_bp(BYPASS_CAP_DELAY);
			return BP_OK;

		}

		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{

			if (pbpctl_dev->bp_caps&TAP_PWUP_CTL_CAP)
			{
				write_data(pbpctl_dev,TAP_STATE_PWRON);
				msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
			}

			if (pbpctl_dev->bp_caps&BP_PWUP_CTL_CAP)
			{
				write_data(pbpctl_dev,BYPASS_STATE_PWRON);
				if (pbpctl_dev->bp_ext_ver>PXG2BPI_VER)
					msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);
				else
					msec_delay_bp(DFLT_PWRON_DELAY);
			}

			if (pbpctl_dev->bp_caps&TAP_DIS_CAP)
			{
				write_data(pbpctl_dev,EN_TAP_CAP);
				msec_delay_bp(BYPASS_CAP_DELAY);
			}
			if (pbpctl_dev->bp_caps&DISC_DIS_CAP)
			{
				write_data(pbpctl_dev,EN_DISC_CAP);
				msec_delay_bp(BYPASS_CAP_DELAY);
			}


			if (pbpctl_dev->bp_caps&BP_DIS_CAP)
			{
				write_data(pbpctl_dev,EN_BYPASS_CAP);
				msec_delay_bp(BYPASS_CAP_DELAY);
			}

			return 0;
		}
	}
	return BP_NOT_CAP;
}


int wdt_time_left (bpctl_dev_t *pbpctl_dev)
{

	//unsigned long curr_time=((long long)(jiffies*1000))/HZ, delta_time=0,wdt_on_time=((long long)(pbpctl_dev->bypass_wdt_on_time*1000))/HZ;
	unsigned long curr_time=ticks, delta_time=0, wdt_on_time=pbpctl_dev->bypass_wdt_on_time, delta_time_msec=0;
	int time_left = 0;
#if 0	 
	int exp_flag = 0, ctrl_ext = 0;


	bpctl_dev_t *pbpctl_dev_b=NULL;


	if ((pbpctl_dev->bp_40g) &&
		(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))  {
		if (pbpctl_dev->func == 0) {
			ctrl_ext = BP40G_READ_GPIO_CTL(pbpctl_dev_b, 2);
			BP40G_WRITE_GPIO_CTL(pbpctl_dev_b, 2, (ctrl_ext &
												   ~(BP40GB_GPIO_SDP_MODE_MASK | BP40GB_GPIO_OE)));
			exp_flag = ((BP40G_RD_REG(pbpctl_dev_b, GPIO_STAT)) & BIT_2)!=0?0:1;

		} else if (pbpctl_dev->func == 2) {
			ctrl_ext = BP40G_READ_GPIO_CTL(pbpctl_dev_b, 3);
			BP40G_WRITE_GPIO_CTL(pbpctl_dev_b, 3, (ctrl_ext &
												   ~(BP40GB_GPIO_SDP_MODE_MASK | BP40GB_GPIO_OE)));

			exp_flag = ((BP40G_RD_REG(pbpctl_dev_b, GPIO_STAT)) & BIT_3)!=0?0:1;

		}
	}
	if (exp_flag) {
		printf("bpmod: WDT expired!\n");
		pbpctl_dev->wdt_status = WDT_STATUS_EXP;
		return -1;
	}
#endif

	switch (pbpctl_dev->wdt_status) {
	case WDT_STATUS_DIS:
		time_left=0;
		break;
	case WDT_STATUS_EN:
		delta_time=(curr_time>=wdt_on_time)?(curr_time-wdt_on_time):(~wdt_on_time+curr_time);
		delta_time_msec=((long long)(delta_time*1000))/hz;
		time_left= pbpctl_dev->bypass_timer_interval-delta_time_msec;
		if (time_left<0)
		{
			time_left=-1;
			pbpctl_dev->wdt_status=WDT_STATUS_EXP;
		}
		break;
	case WDT_STATUS_EXP:
		time_left=-1;
		break;
	}

	return time_left;
}


static int wdt_timer(bpctl_dev_t *pbpctl_dev, int *time_left){
	int ret=0;
	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			if ((read_reg(pbpctl_dev,STATUS_REG_ADDR))&WDT_EN_MASK)
				pbpctl_dev->wdt_status=WDT_STATUS_EN;
			else  pbpctl_dev->wdt_status=WDT_STATUS_DIS;
		}

		if (pbpctl_dev->bp_ext_ver>=PXG4BPFI_VER)
		{
			if (pbpctl_dev->wdt_status==WDT_STATUS_EN)
			{
				u_int32_t wdt_lo=0, wdt_hi=0;

				if ((read_reg(pbpctl_dev,STATUS_REG_ADDR))&WD_EXP_FLAG_MASK)
					*time_left=-1;
				else
				{
					wdt_lo=read_reg(pbpctl_dev,TMRL_REG_ADDR);
					wdt_hi=read_reg(pbpctl_dev,TMRH_REG_ADDR);

					*time_left=((((wdt_hi&0xff)<<8)|(wdt_lo&0xff)))*100;
				}
			} else
				*time_left=0; /* WDT is disabled */
		} else
		{
			if (pbpctl_dev->wdt_status==WDT_STATUS_UNKNOWN)
				ret=BP_NOT_CAP;
			else
				*time_left=wdt_time_left(pbpctl_dev);
		} 

	} else ret=BP_NOT_CAP;
	return ret;
}


static int wdt_timer_reload(bpctl_dev_t *pbpctl_dev){

	int ret=0;

	if (pbpctl_dev->bp_40g) {
		if (pbpctl_dev->bp_caps&WD_CTL_CAP) {
			wdt_pulse(pbpctl_dev);
			return 1;
		} else return BP_NOT_CAP;
	}

	if ((pbpctl_dev->bp_caps&WD_CTL_CAP)&&
		(pbpctl_dev->wdt_status!=WDT_STATUS_UNKNOWN)) {
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
			ret= wdt_pulse(pbpctl_dev);
		else if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
			ret=wdt_pulse_int(pbpctl_dev);
		else  ret=send_wdt_pulse(pbpctl_dev);
		//if (ret==-1)
		//    mod_timer(&pbpctl_dev->bp_timer, jiffies+1);
		return 1;
	}
	return BP_NOT_CAP; 
}



void wd_reset_timer(void *param){
	bpctl_dev_t *pbpctl_dev= (bpctl_dev_t *) param;

	if ((pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)&&
		(pbpctl_dev->wdt_busy==1))
	{
		callout_reset(&pbpctl_dev->bp_timer, 1, wd_reset_timer, pbpctl_dev);
		return;
	}

	wdt_timer_reload(pbpctl_dev); 

	if (pbpctl_dev->reset_time)
		callout_reset(&pbpctl_dev->bp_timer, (pbpctl_dev->reset_time*hz)/1000, wd_reset_timer, pbpctl_dev);

}




/*WAIT_AT_PWRUP 0x80   */
static int bp_wait_at_pwup_en(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=BP_FW_EXT_VER8)
		{
			write_data(pbpctl_dev,BP_WAIT_AT_PWUP_EN);
			msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);

			return BP_OK;
		}
	}
	return BP_NOT_CAP;
}

/*DIS_WAIT_AT_PWRUP       0x81 */
static int bp_wait_at_pwup_dis(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{

		if (pbpctl_dev->bp_ext_ver>=BP_FW_EXT_VER8)
		{
			write_data(pbpctl_dev,BP_WAIT_AT_PWUP_DIS);
			msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);

			return BP_OK;
		}
	}
	return BP_NOT_CAP;
}

/*EN_HW_RESET  0x82   */

static int bp_hw_reset_en(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=BP_FW_EXT_VER8)
		{
			write_data(pbpctl_dev,BP_HW_RESET_EN);
			msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);

			return BP_OK;
		}
	}
	return BP_NOT_CAP;
}

/*DIS_HW_RESET             0x83   */

static int bp_hw_reset_dis(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=BP_FW_EXT_VER8)
		{
			write_data(pbpctl_dev,BP_HW_RESET_DIS);
			msec_delay_bp(LATCH_DELAY+EEPROM_WR_DELAY);

			return BP_OK;
		}
	}
	return BP_NOT_CAP;
}



static int wdt_exp_mode(bpctl_dev_t *pbpctl_dev, int mode){
	uint32_t status_reg=0, status_reg1=0;

	if ((pbpctl_dev->bp_caps&(TAP_STATUS_CAP|DISC_CAP))&&
		(pbpctl_dev->bp_caps&BP_CAP))
	{
		if (pbpctl_dev->bp_ext_ver>=PXE2TBPI_VER)
		{

			if ((pbpctl_dev->bp_ext_ver>=0x8)&&
				(mode==2)&& 
				(pbpctl_dev->bp_caps&DISC_CAP))
			{
				status_reg1=read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR);
				if (!(status_reg1&WDTE_DISC_BPN_MASK))
					write_reg(pbpctl_dev,status_reg1 | WDTE_DISC_BPN_MASK, STATUS_DISC_REG_ADDR);
				return BP_OK;
			}
		}
		status_reg=read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR);

		if ((mode==0)&&(pbpctl_dev->bp_caps&BP_CAP))
		{
			if (pbpctl_dev->bp_ext_ver>=0x8)
			{
				status_reg1=read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR);
				if (status_reg1&WDTE_DISC_BPN_MASK)
					write_reg(pbpctl_dev,status_reg1 & ~WDTE_DISC_BPN_MASK, STATUS_DISC_REG_ADDR);
			}
			if (status_reg&WDTE_TAP_BPN_MASK)
				write_reg(pbpctl_dev,status_reg & ~WDTE_TAP_BPN_MASK, STATUS_TAP_REG_ADDR);
			return BP_OK;

		} else if ((mode==1)&&(pbpctl_dev->bp_caps&TAP_CAP))
		{
			if (!(status_reg&WDTE_TAP_BPN_MASK))
				write_reg(pbpctl_dev,status_reg | WDTE_TAP_BPN_MASK, STATUS_TAP_REG_ADDR);
			/*else return BP_NOT_CAP;*/
			return BP_OK;
		}

	}
	return BP_NOT_CAP;
}


static int bypass_fw_ver(bpctl_dev_t *pbpctl_dev){
	if (is_bypass_fn(pbpctl_dev)) {

		if (pbpctl_dev->bp_40g) {
			bp_cmd_t bp_cmd_buf;
			bp_cmd_rsp_t bp_rsp_buf;

			bpctl_dev_t *pbpctl_dev_c;
			int ret=-1;

			memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
			memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
			bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
			bp_cmd_buf.cmd.cmd_id=CMD_GET_BYPASS_INFO;
			if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
				return -1;

			if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
				ret = bp_rsp_buf.rsp.rsp_data.bypass_info.fw_ver;
			}
			return ret;

		} else return((read_reg(pbpctl_dev,VER_REG_ADDR)));
	} else return BP_NOT_CAP;
}


static int bypass_sign_check(bpctl_dev_t *pbpctl_dev){

	if (is_bypass_fn(pbpctl_dev))
		return(((read_reg(pbpctl_dev,PIC_SIGN_REG_ADDR))==PIC_SIGN_VALUE)?1:0);
	else return BP_NOT_CAP;
}

static int tx_status (bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl=0;

	bpctl_dev_t *pbpctl_dev_m;
	if ((is_bypass_fn(pbpctl_dev))==1)
		pbpctl_dev_m=pbpctl_dev;
	else
		pbpctl_dev_m=get_master_port_fn(pbpctl_dev);
	if (pbpctl_dev_m==NULL)
		return BP_NOT_CAP;
	if (pbpctl_dev_m->bp_caps_ex&DISC_PORT_CAP_EX) {

		ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL);
		if (pbpctl_dev->bp_i80)
			return((ctrl&BPCTLI_CTRL_SWDPIN1)!=0?0:1);
		if (pbpctl_dev->bp_540){
			ctrl = BP10G_READ_REG(pbpctl_dev, ESDP);

			return((ctrl&BP10G_SDP1_DATA)!=0?0:1);
		}



	}

	if (pbpctl_dev->bp_caps&TX_CTL_CAP) {
		if (PEG5_IF_SERIES(pbpctl_dev->subdevice)) {
			uint16_t mii_reg;
			if (!(bp75_read_phy_reg(pbpctl_dev, BPCTLI_PHY_CONTROL, &mii_reg))) {
				if (mii_reg & BPCTLI_MII_CR_POWER_DOWN)
					return 0;

				else
					return 1;
			}return -1;
		}

		if (pbpctl_dev->bp_10g9) {
			return((BP10G_READ_REG(pbpctl_dev,ESDP)&BP10G_SDP3_DATA)!=0?0:1);

		} else if (pbpctl_dev->bp_fiber5) {
			ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
			if (ctrl&BPCTLI_CTRL_EXT_SDP6_DATA)
				return 0;
			return 1; 
		} else if (pbpctl_dev->bp_10gb) {
			ctrl= BP10GB_READ_REG(pbpctl_dev, MISC_REG_GPIO);
			BP10GB_WRITE_REG(pbpctl_dev, MISC_REG_GPIO,(ctrl|BP10GB_GPIO0_OE_P1)&~(BP10GB_GPIO0_SET_P1|BP10GB_GPIO0_CLR_P1));

			if ((pbpctl_dev->func==1)||(pbpctl_dev->func==3))
				return(((BP10GB_READ_REG(pbpctl_dev, MISC_REG_GPIO)) & BP10GB_GPIO0_P1)!=0?0:1);
			else
				return(((BP10GB_READ_REG(pbpctl_dev, MISC_REG_GPIO)) & BP10GB_GPIO0_P0)!=0?0:1);
		}

		if (!pbpctl_dev->bp_10g) {

			ctrl = BPCTL_READ_REG(pbpctl_dev, CTRL);
			if (pbpctl_dev->bp_i80)
				return((ctrl&BPCTLI_CTRL_SWDPIN1)!=0?0:1);
			if (pbpctl_dev->bp_540){
				ctrl = BP10G_READ_REG(pbpctl_dev, ESDP);

				return((ctrl&BP10G_SDP1_DATA)!=0?0:1);
			}

			return((ctrl&BPCTLI_CTRL_SWDPIN0)!=0?0:1);
		} else
			return((BP10G_READ_REG(pbpctl_dev,ESDP)&BP10G_SDP0_DATA)!=0?0:1);      

	}
	return BP_NOT_CAP;
}

static int bypass_from_last_read(bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0;
	bpctl_dev_t *pbpctl_dev_b=NULL;

	if ((pbpctl_dev->bp_caps&SW_CTL_CAP)&&(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
	{
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev_b, CTRL_EXT);
		BPCTL_BP_WRITE_REG(pbpctl_dev_b, CTRL_EXT, ( ctrl_ext & ~BPCTLI_CTRL_EXT_SDP7_DIR));
		ctrl_ext = BPCTL_READ_REG(pbpctl_dev_b, CTRL_EXT);
		if (ctrl_ext&BPCTLI_CTRL_EXT_SDP7_DATA)
			return 0;
		return 1;
	} else return BP_NOT_CAP;
}

static int bypass_status_clear(bpctl_dev_t *pbpctl_dev){
	bpctl_dev_t *pbpctl_dev_b=NULL;

	if ((pbpctl_dev->bp_caps&SW_CTL_CAP)&&(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
	{

		send_bypass_clear_pulse(pbpctl_dev_b, 1);
		return 0;
	} else
		return BP_NOT_CAP;
}

static int bypass_flag_status(bpctl_dev_t *pbpctl_dev){

	if ((pbpctl_dev->bp_caps&BP_CAP))
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			return((((read_reg(pbpctl_dev,STATUS_REG_ADDR)) & BYPASS_FLAG_MASK)==BYPASS_FLAG_MASK)?1:0);
		}
	}
	return BP_NOT_CAP;
}

static int bypass_flag_status_clear(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&BP_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			uint32_t status_reg=0;
			status_reg=read_reg(pbpctl_dev,STATUS_REG_ADDR);
			write_reg(pbpctl_dev,status_reg & ~BYPASS_FLAG_MASK, STATUS_REG_ADDR);
			return 0;
		}
	}
	return BP_NOT_CAP;
}


static int bypass_change_status(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;

	if (pbpctl_dev->bp_caps&BP_STATUS_CHANGE_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
		{
			ret=bypass_flag_status(pbpctl_dev);
			bypass_flag_status_clear(pbpctl_dev);
		} else if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			ret=bypass_flag_status(pbpctl_dev);
			bypass_flag_status_clear(pbpctl_dev);
		} else
		{
			ret=bypass_from_last_read(pbpctl_dev);
			bypass_status_clear(pbpctl_dev);
		}
	}
	return ret;
}


#if 0
static int bypass_off_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&BP_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			return((((read_reg(pbpctl_dev,STATUS_REG_ADDR)) & BYPASS_OFF_MASK)==BYPASS_OFF_MASK)?1:0);
		}
	}
	return BP_NOT_CAP;
}
#endif


static int bypass_status(bpctl_dev_t *pbpctl_dev){
	unsigned int ctrl_ext=0;
	if (pbpctl_dev->bp_caps&BP_CAP) {

		bpctl_dev_t *pbpctl_dev_b=NULL;

		if (!(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
			return BP_NOT_CAP;




		if (pbpctl_dev->bp_40g) {
#if 0
			if (pbpctl_dev->func == 0) {
				ctrl_ext = BP40G_READ_GPIO_CTL(pbpctl_dev_b, 2);
				BP40G_WRITE_GPIO_CTL(pbpctl_dev_b, 2, (ctrl_ext &
													   ~(BP40GB_GPIO_SDP_MODE_MASK | BP40GB_GPIO_OE)));


				return(((BP40G_RD_REG(pbpctl_dev_b, GPIO_STAT)) & BIT_2)!=0?1:0);

			} else if (pbpctl_dev->func == 2) {
				ctrl_ext = BP40G_READ_GPIO_CTL(pbpctl_dev_b, 3);
				BP40G_WRITE_GPIO_CTL(pbpctl_dev_b, 3, (ctrl_ext &
													   ~(BP40GB_GPIO_SDP_MODE_MASK | BP40GB_GPIO_OE)));


				return(((BP40G_RD_REG(pbpctl_dev_b, GPIO_STAT)) & BIT_3)!=0?1:0);

			} else
				return BP_NOT_CAP;
#else

			bp_cmd_t bp_cmd_buf;
			bp_cmd_rsp_t bp_rsp_buf;

			bpctl_dev_t *pbpctl_dev_c;
			int ret=-1;

			memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
			memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
			bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
			bp_cmd_buf.cmd.cmd_id=CMD_GET_BYPASS;
			if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
				return -1;

			if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
				if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
					if (bp_rsp_buf.rsp.rsp_data.bypass_pwoff==BYPASS_PWOFF_EN)
						ret=1;
					else if (bp_rsp_buf.rsp.rsp_data.bypass_pwoff==BYPASS_PWOFF_DIS)
						ret=0;
				}
			}
			return ret;
#endif

		} else if (INTEL_IF_SERIES(pbpctl_dev->subdevice)) {

			if (!pbpctl_dev->bp_status_un)
				return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL_EXT)) & BPCTLI_CTRL_EXT_SDP7_DATA)!=0?1:0);
			else
				return BP_NOT_CAP;
		}
		if (pbpctl_dev->bp_ext_ver>=0x8) {

			//BPCTL_BP_WRITE_REG(pbpctl_dev, CTRL_EXT, (BPCTL_READ_REG(pbpctl_dev_b, CTRL_EXT))&~BPCTLI_CTRL_EXT_SDP7_DIR);
			if (pbpctl_dev->bp_10g9) {
				ctrl_ext= BP10G_READ_REG(pbpctl_dev_b,I2CCTL);
				BP10G_WRITE_REG(pbpctl_dev_b, I2CCTL, (ctrl_ext|BP10G_I2C_CLK_OUT));
				//return(((readl((void *)((pbpctl_dev)->mem_map) + 0x28))&0x4)!=0?0:1);
				return((BP10G_READ_REG(pbpctl_dev_b,I2CCTL)&BP10G_I2C_CLK_IN)!=0?0:1);


			} else if (pbpctl_dev->bp_540) {
				ctrl_ext= BP10G_READ_REG(pbpctl_dev_b,ESDP);
				BP10G_WRITE_REG(pbpctl_dev_b, ESDP, (ctrl_ext|BIT_11)&~BIT_3);
				return(((BP10G_READ_REG(pbpctl_dev_b, ESDP)) & BP10G_SDP0_DATA)!=0?0:1);
			}



			else if ((pbpctl_dev->bp_fiber5)||(pbpctl_dev->bp_i80)) {
				return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL)) & BPCTLI_CTRL_SWDPIN0)!=0?0:1);
			} else if (pbpctl_dev->bp_10gb) {
				ctrl_ext= BP10GB_READ_REG(pbpctl_dev, MISC_REG_GPIO);
				BP10GB_WRITE_REG(pbpctl_dev,MISC_REG_GPIO, (ctrl_ext| BP10GB_GPIO3_OE_P0)&~(BP10GB_GPIO3_SET_P0|BP10GB_GPIO3_CLR_P0));


				return(((BP10GB_READ_REG(pbpctl_dev, MISC_REG_GPIO)) & BP10GB_GPIO3_P0)!=0?0:1);
			}


			else if (!pbpctl_dev->bp_10g)
				return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL_EXT)) & BPCTLI_CTRL_EXT_SDP7_DATA)!=0?0:1);

			else {
				ctrl_ext= BP10G_READ_REG(pbpctl_dev_b,EODSDP);
				BP10G_WRITE_REG(pbpctl_dev_b, EODSDP, (ctrl_ext|BP10G_SDP7_DATA_OUT));
				//return(((readl((void *)((pbpctl_dev)->mem_map) + 0x28))&0x4)!=0?0:1);
				return((BP10G_READ_REG(pbpctl_dev_b,EODSDP)&BP10G_SDP7_DATA_IN)!=0?0:1);
			}

		} else
			if (pbpctl_dev->media_type == bp_copper) {


			return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL)) & BPCTLI_CTRL_SWDPIN1)!=0?1:0);
		} else {
			if ((bypass_status_clear(pbpctl_dev))>=0)
				return(bypass_from_last_read(pbpctl_dev));
		}    

	}
	return BP_NOT_CAP;
}

static int wd_exp_status(bpctl_dev_t *pbpctl_dev){
	unsigned int ctrl_ext=0;
	if (pbpctl_dev->bp_caps&BP_CAP) {

		bpctl_dev_t *pbpctl_dev_b=NULL;

		if (!(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
			return BP_NOT_CAP;




		if (pbpctl_dev->bp_40g) {
#if 1
			if (pbpctl_dev->func == 0) {
				ctrl_ext = BP40G_READ_GPIO_CTL(pbpctl_dev_b, 2);
				BP40G_WRITE_GPIO_CTL(pbpctl_dev_b, 2, (ctrl_ext &
													   ~(BP40GB_GPIO_SDP_MODE_MASK | BP40GB_GPIO_OE)));

				BP40G_RD_REG(pbpctl_dev_b, GPIO_STAT);
				return(((BP40G_RD_REG(pbpctl_dev_b, GPIO_STAT)) & BIT_2)!=0?0:1);

			} else if (pbpctl_dev->func == 2) {
				ctrl_ext = BP40G_READ_GPIO_CTL(pbpctl_dev_b, 3);
				BP40G_WRITE_GPIO_CTL(pbpctl_dev_b, 3, (ctrl_ext &
													   ~(BP40GB_GPIO_SDP_MODE_MASK | BP40GB_GPIO_OE)));

				BP40G_RD_REG(pbpctl_dev_b, GPIO_STAT);
				return(((BP40G_RD_REG(pbpctl_dev_b, GPIO_STAT)) & BIT_3)!=0?0:1);

			} else
				return BP_NOT_CAP;
#endif



		}
	}
	return BP_NOT_CAP;
}



static int default_pwron_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_caps&BP_PWUP_CTL_CAP)
		{
			if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
			{
				return((((read_reg(pbpctl_dev,STATUS_REG_ADDR)) & DFLT_PWRON_MASK)==DFLT_PWRON_MASK)?0:1);
			}
		} /*else if ((!pbpctl_dev->bp_caps&BP_DIS_CAP)&&
				   (pbpctl_dev->bp_caps&BP_PWUP_ON_CAP))
			return 1;*/
	}
	return BP_NOT_CAP;
}

static int default_pwroff_status(bpctl_dev_t *pbpctl_dev){

	/*if ((!pbpctl_dev->bp_caps&BP_DIS_CAP)&&
		(pbpctl_dev->bp_caps&BP_PWOFF_ON_CAP))
		return 1;*/
	if ((pbpctl_dev->bp_caps&SW_CTL_CAP)&&(pbpctl_dev->bp_caps&BP_PWOFF_CTL_CAP))
	{
		return((((read_reg(pbpctl_dev,STATUS_REG_ADDR)) & DFLT_PWROFF_MASK)==DFLT_PWROFF_MASK)?0:1);
	}
	return BP_NOT_CAP;
}



static int dis_bypass_cap_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&BP_DIS_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			return((((read_reg(pbpctl_dev,STATUS_REG_ADDR)) & DIS_BYPASS_CAP_MASK)==DIS_BYPASS_CAP_MASK)?1:0);
		}
	}
	return BP_NOT_CAP;
}

#if 0
static int cmd_en_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			return((((read_reg(pbpctl_dev,STATUS_REG_ADDR)) & CMND_EN_MASK)==CMND_EN_MASK)?1:0);
		}
	}
	return BP_NOT_CAP;
}
#endif
#if 0
static int wdt_en_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			return((((read_reg(pbpctl_dev,STATUS_REG_ADDR)) & WDT_EN_MASK)==WDT_EN_MASK)?1:0);
		}
	}
	return BP_NOT_CAP;
}
#endif

static int wdt_programmed(bpctl_dev_t *pbpctl_dev, int *timeout){
	int ret=0;
	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			if ((read_reg(pbpctl_dev,STATUS_REG_ADDR))&WDT_EN_MASK)
			{
				u_int8_t wdt_val;
				wdt_val=read_reg(pbpctl_dev,WDT_REG_ADDR);
				*timeout=  (1<<wdt_val)*100;
			} else *timeout=0;
		} else
		{
			int curr_wdt_status= pbpctl_dev->wdt_status;
			if (curr_wdt_status==WDT_STATUS_UNKNOWN)
				*timeout=-1;
			else
				*timeout=curr_wdt_status==0?0:pbpctl_dev->bypass_timer_interval;
		};
	} else ret=BP_NOT_CAP;
	return ret;
}
#if 0
static int bypass_support(bpctl_dev_t *pbpctl_dev){
	int ret=0;

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
		{
			ret=((((read_reg(pbpctl_dev,PRODUCT_CAP_REG_ADDR)) & BYPASS_SUPPORT_MASK)==BYPASS_SUPPORT_MASK)?1:0);
		} else if (pbpctl_dev->bp_ext_ver==PXG2BPI_VER)
			ret=1;
	} else ret=BP_NOT_CAP;
	return ret;
}
#endif
#if 0
static int tap_support(bpctl_dev_t *pbpctl_dev){
	int ret=0;

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
		{
			ret=((((read_reg(pbpctl_dev,PRODUCT_CAP_REG_ADDR)) & TAP_SUPPORT_MASK)==TAP_SUPPORT_MASK)?1:0);
		} else if (pbpctl_dev->bp_ext_ver==PXG2BPI_VER)
			ret=0;
	} else ret=BP_NOT_CAP;
	return ret;
}
#endif

static int normal_support(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
		{
			ret=((((read_reg(pbpctl_dev,PRODUCT_CAP_REG_ADDR)) & NORMAL_UNSUPPORT_MASK)==NORMAL_UNSUPPORT_MASK)?0:1);
		} else
			ret=1;
	};
	return ret;
}
static int get_bp_prod_caps(bpctl_dev_t *pbpctl_dev){
	if ((pbpctl_dev->bp_caps&SW_CTL_CAP)&&
		(pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER))
		return(read_reg(pbpctl_dev,PRODUCT_CAP_REG_ADDR));
	return BP_NOT_CAP;

}


static int tap_flag_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&TAP_STATUS_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
			return((((read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR)) & TAP_FLAG_MASK)==TAP_FLAG_MASK)?1:0);

	}
	return BP_NOT_CAP;
}

static int tap_flag_status_clear(bpctl_dev_t *pbpctl_dev){
	uint32_t status_reg=0;
	if (pbpctl_dev->bp_caps&TAP_STATUS_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
		{
			status_reg=read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR);
			write_reg(pbpctl_dev,status_reg & ~TAP_FLAG_MASK, STATUS_TAP_REG_ADDR);
			return 0;
		}
	}
	return BP_NOT_CAP;
}

static int tap_change_status(bpctl_dev_t *pbpctl_dev){
	int ret= BP_NOT_CAP;
	if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
	{
		if (pbpctl_dev->bp_caps&TAP_CAP)
		{
			if (pbpctl_dev->bp_caps&BP_CAP)
			{
				ret=tap_flag_status(pbpctl_dev);
				tap_flag_status_clear(pbpctl_dev);
			} else
			{
				ret=bypass_from_last_read(pbpctl_dev);
				bypass_status_clear(pbpctl_dev);
			}
		}
	}
	return ret;
}

#if 0
static int tap_off_status(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&TAP_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
			return((((read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR)) & TAP_OFF_MASK)==TAP_OFF_MASK)?1:0);
	}
	return BP_NOT_CAP;
}
#endif
static int tap_status(bpctl_dev_t *pbpctl_dev){
	uint32_t ctrl_ext=0;

	if (pbpctl_dev->bp_caps&TAP_CAP) {
		bpctl_dev_t *pbpctl_dev_b=NULL;

		if (!(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
			return BP_NOT_CAP;

		if (pbpctl_dev->bp_ext_ver>=0x8) {
			if (!pbpctl_dev->bp_10g)
				return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL_EXT)) & BPCTLI_CTRL_EXT_SDP6_DATA)!=0?0:1);
			else {
				ctrl_ext= BP10G_READ_REG(pbpctl_dev_b,EODSDP);
				BP10G_WRITE_REG(pbpctl_dev_b, EODSDP, (ctrl_ext|BP10G_SDP6_DATA_OUT));
				return((BP10G_READ_REG(pbpctl_dev_b,EODSDP)&BP10G_SDP6_DATA_IN)!=0?0:1);
			}


		} else
			if (pbpctl_dev->media_type == bp_copper)
			return(((BPCTL_READ_REG(pbpctl_dev, CTRL)) & BPCTLI_CTRL_SWDPIN0)!=0?1:0);
		else {
			if ((bypass_status_clear(pbpctl_dev))>=0)
				return(bypass_from_last_read(pbpctl_dev));
		}   

	}
	return BP_NOT_CAP;
}




static int default_pwron_tap_status(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&TAP_PWUP_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
			return((((read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR)) & DFLT_PWRON_TAP_MASK)==DFLT_PWRON_TAP_MASK)?1:0);
	}
	return BP_NOT_CAP;
}

static int dis_tap_cap_status(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&TAP_PWUP_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER)
			return((((read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR)) & DIS_TAP_CAP_MASK)==DIS_TAP_CAP_MASK)?1:0);
	}
	return BP_NOT_CAP;
}

static int disc_flag_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&DISC_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
			return((((read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR)) & DISC_FLAG_MASK)==DISC_FLAG_MASK)?1:0);

	}
	return BP_NOT_CAP;
}

static int disc_flag_status_clear(bpctl_dev_t *pbpctl_dev){
	uint32_t status_reg=0;
	if (pbpctl_dev->bp_caps&DISC_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
		{
			status_reg=read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR);
			write_reg(pbpctl_dev,status_reg & ~DISC_FLAG_MASK, STATUS_DISC_REG_ADDR);
			return BP_OK;
		}
	}
	return BP_NOT_CAP;
}

static int disc_change_status(bpctl_dev_t *pbpctl_dev){
	int ret=BP_NOT_CAP;
	if (pbpctl_dev->bp_caps&DISC_CAP)
	{
		ret=disc_flag_status(pbpctl_dev);
		disc_flag_status_clear(pbpctl_dev);
		return ret;
	}
	return BP_NOT_CAP;
}

static int disc_off_status(bpctl_dev_t *pbpctl_dev){
	bpctl_dev_t *pbpctl_dev_b=NULL;
	uint32_t ctrl_ext=0;

	if (pbpctl_dev->bp_caps&DISC_CAP) {
		if (!(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
			return BP_NOT_CAP;
		if (DISCF_IF_SERIES(pbpctl_dev->subdevice))
			return((((read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR)) & DISC_OFF_MASK)==DISC_OFF_MASK)?1:0);

		if (pbpctl_dev->bp_i80) {
			return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL_EXT)) & BPCTLI_CTRL_EXT_SDP6_DATA)!=0?1:0);

		}

		if (pbpctl_dev->bp_540) {
			ctrl_ext= BP10G_READ_REG(pbpctl_dev_b,ESDP);
			//return(((readl((void *)((pbpctl_dev)->mem_map) + 0x28))&0x4)!=0?0:1);
			return((BP10G_READ_REG(pbpctl_dev_b,ESDP)&BP10G_SDP2_DATA)!=0?1:0);

		}

		//if (pbpctl_dev->device==SILICOM_PXG2TBI_SSID) {
		if (pbpctl_dev->media_type == bp_copper) {

#if 0	
			return((((read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR)) & DISC_OFF_MASK)==DISC_OFF_MASK)?1:0);
#endif
			if (!pbpctl_dev->bp_10g)
				return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL)) & BPCTLI_CTRL_SWDPIN1)!=0?1:0);
			else
				return((BP10G_READ_REG(pbpctl_dev_b,ESDP)&BP10G_SDP1_DATA)!=0?1:0);


		} else {
			if (pbpctl_dev->bp_10g9) {
				ctrl_ext= BP10G_READ_REG(pbpctl_dev_b,I2CCTL);
				BP10G_WRITE_REG(pbpctl_dev_b, I2CCTL, (ctrl_ext|BP10G_I2C_DATA_OUT));
				return((BP10G_READ_REG(pbpctl_dev_b,I2CCTL)&BP10G_I2C_DATA_IN)!=0?1:0);

			} else if (pbpctl_dev->bp_fiber5) {
				return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL)) & BPCTLI_CTRL_SWDPIN1)!=0?1:0);
			} else if (pbpctl_dev->bp_10gb) {
				ctrl_ext= BP10GB_READ_REG(pbpctl_dev, MISC_REG_GPIO);
				BP10GB_WRITE_REG(pbpctl_dev,MISC_REG_GPIO, (ctrl_ext| BP10GB_GPIO3_OE_P1)&~(BP10GB_GPIO3_SET_P1|BP10GB_GPIO3_CLR_P1));


				return(((BP10GB_READ_REG(pbpctl_dev, MISC_REG_GPIO))&BP10GB_GPIO3_P1)!=0?1:0);
			}
			if (!pbpctl_dev->bp_10g) {

				return(((BPCTL_READ_REG(pbpctl_dev_b, CTRL_EXT)) & BPCTLI_CTRL_EXT_SDP6_DATA)!=0?1:0);
			} else {
				ctrl_ext= BP10G_READ_REG(pbpctl_dev_b,EODSDP);
				BP10G_WRITE_REG(pbpctl_dev_b, EODSDP, (ctrl_ext|BP10G_SDP6_DATA_OUT));
				return(((BP10G_READ_REG(pbpctl_dev_b,EODSDP))&BP10G_SDP6_DATA_IN)!=0?1:0);
			}

		}
	}
	return BP_NOT_CAP;
}

static int disc_status(bpctl_dev_t *pbpctl_dev){
	int ctrl=0;
	if (pbpctl_dev->bp_caps&DISC_CAP)
	{

		if ((ctrl=disc_off_status(pbpctl_dev))<0)
			return ctrl;
		return((ctrl==0)?1:0);

	}
	return BP_NOT_CAP;
}


static int default_pwron_disc_status(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&DISC_PWUP_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
			return((((read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR)) & DFLT_PWRON_DISC_MASK)==DFLT_PWRON_DISC_MASK)?1:0);
	}
	return BP_NOT_CAP;
}

static int dis_disc_cap_status(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&DIS_DISC_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
			return((((read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR)) & DIS_DISC_CAP_MASK)==DIS_DISC_CAP_MASK)?1:0);
	}
	return BP_NOT_CAP;
}

static int wdt_exp_mode_status(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver<=PXG2BPI_VER)
			return 0;  /* bypass mode */
		else if (pbpctl_dev->bp_ext_ver==PXG2TBPI_VER)
			return 1; /* tap mode */
		else if (pbpctl_dev->bp_ext_ver>=PXE2TBPI_VER)
		{
			if (pbpctl_dev->bp_ext_ver>=0x8)
			{
				if (((read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR)) & WDTE_DISC_BPN_MASK)==WDTE_DISC_BPN_MASK)
					return 2;
			}
			return((((read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR)) & WDTE_TAP_BPN_MASK)==WDTE_TAP_BPN_MASK)?1:0);
		}
	}
	return BP_NOT_CAP;
}

static int tpl2_flag_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps_ex&TPL2_CAP_EX)
	{
		return((((read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR)) & TPL2_FLAG_MASK)==TPL2_FLAG_MASK)?1:0);

	}
	return BP_NOT_CAP;
}

#if 0
static int tpl_hw_status (bpctl_dev_t *pbpctl_dev){
	bpctl_dev_t *pbpctl_dev_b=NULL;

	if (!(pbpctl_dev_b=get_status_port_fn(pbpctl_dev)))
		return BP_NOT_CAP;

	if (TPL_IF_SERIES(pbpctl_dev->subdevice))
		return(((BPCTL_READ_REG(pbpctl_dev, CTRL)) & BPCTLI_CTRL_SWDPIN0)!=0?1:0);
	return BP_NOT_CAP;
}
#endif


static int bp_wait_at_pwup_status(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{
		if (pbpctl_dev->bp_ext_ver>=0x8)
			return((((read_reg(pbpctl_dev,CONT_CONFIG_REG_ADDR)) & WAIT_AT_PWUP_MASK)==WAIT_AT_PWUP_MASK)?1:0);
	}
	return BP_NOT_CAP;
}

static int bp_hw_reset_status(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&SW_CTL_CAP)
	{

		if (pbpctl_dev->bp_ext_ver>=0x8)
			return((((read_reg(pbpctl_dev,CONT_CONFIG_REG_ADDR)) & EN_HW_RESET_MASK)==EN_HW_RESET_MASK)?1:0);
	}
	return BP_NOT_CAP;
}


static int std_nic_status(bpctl_dev_t *pbpctl_dev){
	int status_val=0;

	if (pbpctl_dev->bp_caps&STD_NIC_CAP)
	{
		if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
			return BP_NOT_CAP;
		if (pbpctl_dev->bp_ext_ver>=BP_FW_EXT_VER8)
		{
			return((((read_reg(pbpctl_dev,STATUS_DISC_REG_ADDR)) & STD_NIC_ON_MASK)==STD_NIC_ON_MASK)?1:0);
		}


		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER)
		{
			if (pbpctl_dev->bp_caps&BP_CAP)
			{
				status_val=read_reg(pbpctl_dev,STATUS_REG_ADDR);
				if (((!(status_val&WDT_EN_MASK))&& ((status_val & STD_NIC_MASK)==STD_NIC_MASK)))
					status_val=1;
				else
					return 0;
			}
			if (pbpctl_dev->bp_caps&TAP_CAP)
			{
				status_val=read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR);
				if ((status_val & STD_NIC_TAP_MASK)==STD_NIC_TAP_MASK)
					status_val=1;
				else
					return 0;
			}
			if (pbpctl_dev->bp_caps&TAP_CAP)
			{
				if ((disc_off_status(pbpctl_dev)))
					status_val=1;
				else
					return 0; 
			}

			return status_val;
		}
	}
	return BP_NOT_CAP;
}  




/******************************************************/
/**************SW_INIT*********************************/
/******************************************************/

static void bypass_caps_init (bpctl_dev_t *pbpctl_dev){
	u_int32_t  ctrl_ext=0;
	bpctl_dev_t *pbpctl_dev_m=NULL;


#ifdef BYPASS_DEBUG
	int ret=0;
	if (!(INTEL_IF_SERIES(adapter->bp_device_block.subdevice))) {
		ret=read_reg(pbpctl_dev,VER_REG_ADDR) ;
		printf("VER_REG reg1=%x\n",ret);
		ret=read_reg(pbpctl_dev,PRODUCT_CAP_REG_ADDR) ;
		printf("PRODUCT_CAP reg=%x\n",ret);
		ret=read_reg(pbpctl_dev,STATUS_TAP_REG_ADDR) ;
		printf("STATUS_TAP reg1=%x\n",ret);
		ret=read_reg(pbpctl_dev,0x7) ;
		printf("SIG_REG reg1=%x\n",ret);
		ret=read_reg(pbpctl_dev,STATUS_REG_ADDR);
		printf("STATUS_REG_ADDR=%x\n",ret);
		ret=read_reg(pbpctl_dev,WDT_REG_ADDR);
		printf("WDT_REG_ADDR=%x\n",ret);
		ret=read_reg(pbpctl_dev,TMRL_REG_ADDR);
		printf("TMRL_REG_ADDR=%x\n",ret);
		ret=read_reg(pbpctl_dev,TMRH_REG_ADDR);
		printf("TMRH_REG_ADDR=%x\n",ret);
	}
#endif

	if (pbpctl_dev->bp_40g) {
		pbpctl_dev->bp_caps=get_bypass_caps_fn(pbpctl_dev);
		return;
	}

	if ((pbpctl_dev->bp_fiber5) ||(pbpctl_dev->bp_10g9)) {
		pbpctl_dev->media_type=  bp_fiber;
	} else if (pbpctl_dev->bp_10gb) {
		if (BP10GB_CX4_SERIES(pbpctl_dev->subdevice))
			pbpctl_dev->media_type=  bp_cx4;
		else pbpctl_dev->media_type=  bp_fiber;

	}

	else if ( pbpctl_dev->bp_540)
		pbpctl_dev->media_type=bp_none;
	else if (!pbpctl_dev->bp_10g) {

		ctrl_ext = BPCTL_READ_REG(pbpctl_dev, CTRL_EXT);
		if ((ctrl_ext & BPCTLI_CTRL_EXT_LINK_MODE_MASK) ==0x0)
			pbpctl_dev->media_type=bp_copper;
		else
			pbpctl_dev->media_type=bp_fiber;

	}

	//if (!pbpctl_dev->bp_10g)
	//  pbpctl_dev->media_type=((BPCTL_READ_REG(pbpctl_dev, STATUS))&BPCTLI_STATUS_TBIMODE)?bp_fiber:bp_copper;
	else {
		if (BP10G_CX4_SERIES(pbpctl_dev->subdevice))
			pbpctl_dev->media_type=  bp_cx4;
		else pbpctl_dev->media_type=  bp_fiber;
	}


	//pbpctl_dev->bp_fw_ver=0xa8;
	if (is_bypass_fn(pbpctl_dev)) {

		pbpctl_dev->bp_caps|=BP_PWOFF_ON_CAP;
		if (pbpctl_dev->media_type==bp_fiber)
			pbpctl_dev->bp_caps|=(TX_CTL_CAP| TX_STATUS_CAP|TPL_CAP);


		if (TPL_IF_SERIES(pbpctl_dev->subdevice)) {
			pbpctl_dev->bp_caps|=TPL_CAP;
		}
		if ((pbpctl_dev->subdevice&0xfe0)==0xb40)
			pbpctl_dev->bp_caps&= ~TPL_CAP;
		if (pbpctl_dev->bp_10g9)
			pbpctl_dev->bp_caps&= ~TPL_CAP;

		if (INTEL_IF_SERIES(pbpctl_dev->subdevice)) {
			pbpctl_dev->bp_caps|=(BP_CAP | BP_STATUS_CAP | SW_CTL_CAP |  
								  BP_PWUP_ON_CAP |BP_PWUP_OFF_CAP | 
								  BP_PWOFF_OFF_CAP |
								  WD_CTL_CAP | WD_STATUS_CAP | STD_NIC_CAP |
								  WD_TIMEOUT_CAP);

			pbpctl_dev->bp_ext_ver=OLD_IF_VER;
			return;
		}

		if ((pbpctl_dev->bp_fw_ver==0xff)&&
			OLD_IF_SERIES(pbpctl_dev->subdevice)) {

			pbpctl_dev->bp_caps|=(BP_CAP | BP_STATUS_CAP | BP_STATUS_CHANGE_CAP | SW_CTL_CAP |  
								  BP_PWUP_ON_CAP | WD_CTL_CAP | WD_STATUS_CAP | 
								  WD_TIMEOUT_CAP);

			pbpctl_dev->bp_ext_ver=OLD_IF_VER;
			return; 
		}

		else {
			switch (pbpctl_dev->bp_fw_ver) {
			case BP_FW_VER_A0:
			case BP_FW_VER_A1 :{ 
					pbpctl_dev->bp_ext_ver=(pbpctl_dev->bp_fw_ver & EXT_VER_MASK);
					break;
				}
			default: { 
					if ((bypass_sign_check(pbpctl_dev))!=1) {
						pbpctl_dev->bp_caps=0;
						return;
					}
					pbpctl_dev->bp_ext_ver=(pbpctl_dev->bp_fw_ver & EXT_VER_MASK);
				}
			}
		}
		if (pbpctl_dev->bp_ext_ver>=0x9)
			pbpctl_dev->bp_caps&= ~TPL_CAP;


		if (pbpctl_dev->bp_ext_ver==PXG2BPI_VER)
			pbpctl_dev->bp_caps|=(BP_CAP|BP_STATUS_CAP|BP_STATUS_CHANGE_CAP|SW_CTL_CAP|BP_DIS_CAP|BP_DIS_STATUS_CAP|
								  BP_PWUP_ON_CAP|BP_PWUP_OFF_CAP|BP_PWUP_CTL_CAP|WD_CTL_CAP|
								  STD_NIC_CAP|WD_STATUS_CAP|WD_TIMEOUT_CAP);
		else if (pbpctl_dev->bp_ext_ver>=PXG2TBPI_VER) {
			int cap_reg;

			pbpctl_dev->bp_caps|=(SW_CTL_CAP|WD_CTL_CAP|WD_STATUS_CAP|WD_TIMEOUT_CAP);
			cap_reg=get_bp_prod_caps(pbpctl_dev);

			if ((cap_reg & NORMAL_UNSUPPORT_MASK)==NORMAL_UNSUPPORT_MASK)
				pbpctl_dev->bp_caps|= NIC_CAP_NEG;
			else
				pbpctl_dev->bp_caps|= STD_NIC_CAP;

			if ((normal_support(pbpctl_dev))==1)

				pbpctl_dev->bp_caps|= STD_NIC_CAP;

			else
				pbpctl_dev->bp_caps|= NIC_CAP_NEG;
			if ((cap_reg & BYPASS_SUPPORT_MASK)==BYPASS_SUPPORT_MASK) {
				pbpctl_dev->bp_caps|=(BP_CAP|BP_STATUS_CAP|BP_STATUS_CHANGE_CAP|BP_DIS_CAP|BP_DIS_STATUS_CAP|
									  BP_PWUP_ON_CAP|BP_PWUP_OFF_CAP|BP_PWUP_CTL_CAP);
				if (pbpctl_dev->bp_ext_ver>=BP_FW_EXT_VER7)
					pbpctl_dev->bp_caps|= BP_PWOFF_ON_CAP|BP_PWOFF_OFF_CAP|BP_PWOFF_CTL_CAP;
			}
			if ((cap_reg & TAP_SUPPORT_MASK)==TAP_SUPPORT_MASK) {
				pbpctl_dev->bp_caps|=(TAP_CAP|TAP_STATUS_CAP|TAP_STATUS_CHANGE_CAP|TAP_DIS_CAP|TAP_DIS_STATUS_CAP|
									  TAP_PWUP_ON_CAP|TAP_PWUP_OFF_CAP|TAP_PWUP_CTL_CAP);
			}
			if (pbpctl_dev->bp_ext_ver>=BP_FW_EXT_VER8) {
				if ((cap_reg & DISC_SUPPORT_MASK)==DISC_SUPPORT_MASK)
					pbpctl_dev->bp_caps|=(DISC_CAP|DISC_DIS_CAP|DISC_PWUP_CTL_CAP);
				if ((cap_reg & TPL2_SUPPORT_MASK)==TPL2_SUPPORT_MASK) {
					pbpctl_dev->bp_caps_ex|=TPL2_CAP_EX;
					pbpctl_dev->bp_caps|=TPL_CAP;
					pbpctl_dev->bp_tpl_flag=tpl2_flag_status(pbpctl_dev);
				}

			}

			if (pbpctl_dev->bp_ext_ver>=BP_FW_EXT_VER9) {
				if ((cap_reg & DISC_PORT_SUPPORT_MASK)==DISC_PORT_SUPPORT_MASK) {
					pbpctl_dev->bp_caps_ex|=DISC_PORT_CAP_EX;
					pbpctl_dev->bp_caps|= (TX_CTL_CAP| TX_STATUS_CAP);
				}

			}

		}

		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER) {
			int sta = read_reg(pbpctl_dev,STATUS_REG_ADDR);
			if (sta & WD_EXP_FLAG_MASK)
				pbpctl_dev->wdt_status=WDT_STATUS_EXP;
			else if (sta & WDT_EN_MASK)

				pbpctl_dev->wdt_status=WDT_STATUS_EN;
			else  pbpctl_dev->wdt_status=WDT_STATUS_DIS;
		}
		if (pbpctl_dev->bp_ext_ver>=PXG2BPI_VER) {
			if ((read_reg(pbpctl_dev,STATUS_REG_ADDR))&WDT_EN_MASK)
				pbpctl_dev->wdt_status=WDT_STATUS_EN;
			else  pbpctl_dev->wdt_status=WDT_STATUS_DIS;
		}

		if ((PEG5_IF_SERIES(pbpctl_dev->subdevice)) ||
			(PEG80_IF_SERIES(pbpctl_dev->subdevice)))
			pbpctl_dev->bp_caps |= TPL_CAP;


	} else if ((P2BPFI_IF_SERIES(pbpctl_dev->subdevice))||
			   (PEGF5_IF_SERIES(pbpctl_dev->subdevice))||
			   (PEGF80_IF_SERIES(pbpctl_dev->subdevice))||
			   (BP10G9_IF_SERIES(pbpctl_dev->subdevice))) {
		pbpctl_dev->bp_caps|= (TX_CTL_CAP| TX_STATUS_CAP);
	}
	if ((pbpctl_dev->subdevice&0xfe0)==0xaa0)
		pbpctl_dev->bp_caps|= (TX_CTL_CAP| TX_STATUS_CAP);
	if (PEG5_IF_SERIES(pbpctl_dev->subdevice))
		pbpctl_dev->bp_caps|= (TX_CTL_CAP| TX_STATUS_CAP);
	if (pbpctl_dev->bp_fiber5)
		pbpctl_dev->bp_caps|= (TX_CTL_CAP| TX_STATUS_CAP);

	if (BP10GB_IF_SERIES  (pbpctl_dev->subdevice)) {
		pbpctl_dev->bp_caps&= ~(TX_CTL_CAP| TX_STATUS_CAP);
	}
	pbpctl_dev_m=get_master_port_fn(pbpctl_dev);
	if (pbpctl_dev_m!=NULL) {
		int cap_reg=0;
		if (pbpctl_dev_m->bp_ext_ver>=0x9) {
			cap_reg=get_bp_prod_caps(pbpctl_dev_m);
			if ((cap_reg & DISC_PORT_SUPPORT_MASK)==DISC_PORT_SUPPORT_MASK)
				pbpctl_dev->bp_caps|= (TX_CTL_CAP| TX_STATUS_CAP);
			pbpctl_dev->bp_caps_ex|= DISC_PORT_CAP_EX;
		}
	}
}


#if 0
static int bypass_off_init(bpctl_dev_t *pbpctl_dev){
	int ret=0;

	if ((ret=cmnd_on(pbpctl_dev))<0)
		return ret;
	if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
		return(dis_bypass_cap(pbpctl_dev));
	wdt_off(pbpctl_dev);
	if (pbpctl_dev->bp_caps&BP_CAP)
		bypass_off(pbpctl_dev);
	if (pbpctl_dev->bp_caps&TAP_CAP)
		tap_off(pbpctl_dev);
	cmnd_off(pbpctl_dev);
	return 0;
}
#endif


static void remove_bypass_wd_auto(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
		callout_drain(&pbpctl_dev->bp_timer);
}

static int init_bypass_wd_auto(bpctl_dev_t *pbpctl_dev){
	callout_init(&pbpctl_dev->bp_timer, 1);
	return 1; 
}



static int set_bypass_wd_auto(bpctl_dev_t *pbpctl_dev, unsigned int param){
	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
	{
		if (pbpctl_dev->reset_time!=param)
		{
			if (INTEL_IF_SERIES(pbpctl_dev->subdevice))
				pbpctl_dev->reset_time=(param<WDT_AUTO_MIN_INT)?WDT_AUTO_MIN_INT:param;
			else pbpctl_dev->reset_time=param;
			if (param)
				wd_reset_timer((void *)pbpctl_dev);
		}
		return 0;
	}
	return BP_NOT_CAP; 
}


static int get_bypass_wd_auto(bpctl_dev_t *pbpctl_dev){

	if (pbpctl_dev->bp_caps&WD_CTL_CAP)
	{
		return pbpctl_dev->reset_time;
	}
	return BP_NOT_CAP; 
}




/**************************************************************/
/************************* API ********************************/
/**************************************************************/


int is_bypass_fn(bpctl_dev_t *pbpctl_dev){
	return(((pbpctl_dev->func==0)||(pbpctl_dev->func==2))?1:0);
}

static int set_bypass_fn (bpctl_dev_t *pbpctl_dev, int bypass_mode){
	int ret=0;

	if (!(pbpctl_dev->bp_caps & BP_CAP))
		return BP_NOT_CAP;


	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;

		bpctl_dev_t *pbpctl_dev_c;
		int ret=-1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id=CMD_SET_BYPASS;
		bp_cmd_buf.cmd.cmd_data.bypass_mode=bypass_mode?1:0;

		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
			return -1;

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret=0;
			}
		}
		msec_delay_bp(LATCH_DELAY);  
		return ret;

	} else {

		if ((ret=cmnd_on(pbpctl_dev))<0)
			return ret;
		if (!bypass_mode)
			ret=bypass_off(pbpctl_dev);
		else
			ret=bypass_on(pbpctl_dev);
		cmnd_off(pbpctl_dev);
	}

	return ret;
}


static int get_wd_expire_fn (bpctl_dev_t *pbpctl_dev){
	return(wd_exp_status(pbpctl_dev));
}

static int get_bypass_fn (bpctl_dev_t *pbpctl_dev){

	return(bypass_status(pbpctl_dev));
}


static int get_bypass_change_fn(bpctl_dev_t *pbpctl_dev){
	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & BP_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;

		bpctl_dev_t *pbpctl_dev_c;
		int ret=-1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;

		bp_cmd_buf.cmd.cmd_id=CMD_GET_BYPASS_CHANGE;
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev)))
			return -1;

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_data.bypass_change==BYPASS_CHANGED)
				ret=1;
			else if (bp_rsp_buf.rsp.rsp_data.bypass_change==BYPASS_NOT_CHANGED)
				ret=0;
		}
		return ret;

	} else {
		return(bypass_change_status(pbpctl_dev));
	}
}

static int set_dis_bypass_fn(bpctl_dev_t *pbpctl_dev, int dis_bypass){
	int ret=0;

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & BP_DIS_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;

		bpctl_dev_t *pbpctl_dev_c;
		int ret=-1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id=CMD_SET_DIS_BYPASS;
		bp_cmd_buf.cmd.cmd_data.dis_bypass=dis_bypass?DIS_BYPASS_DISABLE:DIS_BYPASS_ENABLE;

		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret=0;
			}
		}
		msec_delay_bp(BYPASS_CAP_DELAY);
		return ret;

	} else {
		if (!(pbpctl_dev->bp_caps & BP_DIS_CAP)) {
			return BP_NOT_CAP;
		}
		if ((ret=cmnd_on(pbpctl_dev))<0) {
			return ret;
		}
		if (dis_bypass) {
			ret=dis_bypass_cap(pbpctl_dev);
		} else {
			ret=en_bypass_cap(pbpctl_dev);
		}
		cmnd_off(pbpctl_dev);
		return ret;
	}
}

static int get_dis_bypass_fn(bpctl_dev_t *pbpctl_dev){

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & BP_DIS_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;

		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;

		bp_cmd_buf.cmd.cmd_id=CMD_GET_DIS_BYPASS;
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_data.dis_bypass==DIS_BYPASS_DISABLE) {
				ret=1;
			} else if (bp_rsp_buf.rsp.rsp_data.dis_bypass==DIS_BYPASS_ENABLE) {
				ret=0;
			}
		}
		return ret;

	} else
		return(dis_bypass_cap_status(pbpctl_dev));
}

static int set_bypass_pwoff_fn (bpctl_dev_t *pbpctl_dev, int bypass_mode){
	int ret=0;

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & BP_PWOFF_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		int ret = -1;
		bpctl_dev_t *pbpctl_dev_c;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id=CMD_SET_BYPASS_PWOFF;
		bp_cmd_buf.cmd.cmd_data.bypass_pwoff=bypass_mode?BYPASS_PWOFF_EN:BYPASS_PWOFF_DIS;

		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret=0;
			}
		}
		msec_delay_bp(EEPROM_WR_DELAY);
		return ret;
	} else {
		if ((ret=cmnd_on(pbpctl_dev))<0) {
			return ret;
		}
		if (bypass_mode) {
			ret=bypass_state_pwroff(pbpctl_dev);
		} else {
			ret=normal_state_pwroff(pbpctl_dev);
		}
		cmnd_off(pbpctl_dev);
		return ret;
	}     
}

static int get_bypass_pwoff_fn(bpctl_dev_t *pbpctl_dev){
	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & BP_PWOFF_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_BYPASS_PWOFF;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				if (bp_rsp_buf.rsp.rsp_data.bypass_pwoff==BYPASS_PWOFF_EN) {
					ret=1;
				} else if (bp_rsp_buf.rsp.rsp_data.bypass_pwoff==BYPASS_PWOFF_DIS) {
					ret=0;
				}
			}
		}

		return ret;

	} else {
		return(default_pwroff_status(pbpctl_dev));
	}
}


static int set_bypass_pwup_fn(bpctl_dev_t *pbpctl_dev, int bypass_mode){
	int ret=0;
	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & BP_PWUP_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;

		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id=CMD_SET_BYPASS_PWUP;
		bp_cmd_buf.cmd.cmd_data.bypass_pwup=bypass_mode?BYPASS_PWUP_EN:BYPASS_PWUP_DIS;

		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret=0;
			}
		}
		msec_delay_bp(EEPROM_WR_DELAY);
		return ret;

	} else {
		if ((ret=cmnd_on(pbpctl_dev))<0) {
			return ret;
		}
		if (bypass_mode) {
			ret=bypass_state_pwron(pbpctl_dev);
		} else {
			ret=normal_state_pwron(pbpctl_dev);
		}
		cmnd_off(pbpctl_dev);

		return ret;
	}
}

static int get_bypass_pwup_fn(bpctl_dev_t *pbpctl_dev){

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & BP_PWUP_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id=CMD_GET_BYPASS_PWUP;
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				if (bp_rsp_buf.rsp.rsp_data.bypass_pwoff==BYPASS_PWOFF_EN) {
					ret=1;
				} else if (bp_rsp_buf.rsp.rsp_data.bypass_pwoff==BYPASS_PWOFF_DIS) {
					ret=0;
				}
			}
		}
		return ret;

	} else
		return(default_pwron_status(pbpctl_dev));
}

static int set_bypass_wd_fn(bpctl_dev_t *pbpctl_dev, int timeout){
	int ret=0;

	if (!pbpctl_dev) {
		return -1;
	}
	if (!(pbpctl_dev->bp_caps & WD_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_BYPASS_WD;
		bp_cmd_buf.cmd.cmd_data.timeout = htonl(timeout);

		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = ntohl(bp_rsp_buf.rsp.rsp_data.timeout_set);
				if (ret) {
					pbpctl_dev->wdt_status = WDT_STATUS_EN;
				}
			}
		}
		return ret;

	} else {
		if ((ret=cmnd_on(pbpctl_dev)) < 0) {
			return ret;
		}
		if (!timeout) {
			ret=wdt_off(pbpctl_dev);
		} else {
			wdt_on(pbpctl_dev,timeout);
			ret = pbpctl_dev->bypass_timer_interval;
		}
		cmnd_off(pbpctl_dev);
		return ret;
	}
}

static int get_bypass_wd_fn(bpctl_dev_t *pbpctl_dev, int *timeout){
	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & WD_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;

		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));      
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id=CMD_GET_BYPASS_WD;
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				*timeout = ntohl(bp_rsp_buf.rsp.rsp_data.timeout_set);
				if (*timeout) {
					pbpctl_dev->wdt_status = WDT_STATUS_EN;
				}
				ret=0;
			}
		}
		return ret;

	} else {
		return wdt_programmed(pbpctl_dev, timeout);
	}
}

static int get_wd_expire_time_fn(bpctl_dev_t *pbpctl_dev, int *time_left) {
#if 0
	bpctl_dev_t *pbpctl_dev_b = NULL;
#endif

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & WD_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_WD_EXPIRE_TIME;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				*time_left = ntohl(bp_rsp_buf.rsp.rsp_data.time_left);
				if (*time_left==0) {
					ret=0;
				} else {
					ret=1;
				}
			}
		}
		return ret;

	} else {
		return(wdt_timer(pbpctl_dev, time_left));
	}
}


static int reset_bypass_wd_timer_fn(bpctl_dev_t *pbpctl_dev){

	return(wdt_timer_reload(pbpctl_dev));
}

static int get_wd_set_caps_fn(bpctl_dev_t *pbpctl_dev){
	int bp_status = 0;    
	unsigned int step_value = TIMEOUT_MAX_STEP + 1, bit_cnt = 0;

	if (!pbpctl_dev) {
		return -1;
	}
	if (INTEL_IF_SERIES(pbpctl_dev->subdevice)) {
		return BP_NOT_CAP;
	}

	if (!(pbpctl_dev->bp_caps & WD_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_WD_SET_CAPS;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK)
					ret=ntohl(bp_rsp_buf.rsp.rsp_data.wd_set_caps);

			}
		}
		return ret;

	} else {
		while ((step_value>>=1)) {
			bit_cnt++;
		}

		if (is_bypass_fn(pbpctl_dev)) {
			bp_status= WD_STEP_COUNT_MASK(bit_cnt)|WDT_STEP_TIME|WD_MIN_TIME_MASK(TIMEOUT_UNIT/100);
		} else {
			return -1;
		}

		return bp_status;
	}
}

static int set_std_nic_fn(bpctl_dev_t *pbpctl_dev, int nic_mode){
	int ret = 0;

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & STD_NIC_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;

		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_STD_NIC;
		bp_cmd_buf.cmd.cmd_data.std_nic = nic_mode?STD_NIC_EN:STD_NIC_DIS;

		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		msec_delay_bp(BYPASS_CAP_DELAY);
		return ret;

	} else {
		if ((ret = cmnd_on(pbpctl_dev)) < 0) {      
			return ret;
		}
		if (nic_mode) {
			ret = std_nic_on(pbpctl_dev);
		} else {
			ret = std_nic_off(pbpctl_dev);
		}
		cmnd_off(pbpctl_dev);
		return ret;
	}
}

static int get_std_nic_fn(bpctl_dev_t *pbpctl_dev) {
	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps&STD_NIC_CAP)) {
		return -1;
	}
	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num= (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id=CMD_GET_STD_NIC;
		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				if (bp_rsp_buf.rsp.rsp_data.std_nic==STD_NIC_EN) {
					ret = 1;
				} else if (bp_rsp_buf.rsp.rsp_data.std_nic==STD_NIC_DIS) {
					ret = 0;
				}
			}
		}
		return ret;

	} else {
		return(std_nic_status(pbpctl_dev));
	}
}

static int set_tap_fn (bpctl_dev_t *pbpctl_dev, int tap_mode){

	if ((pbpctl_dev->bp_caps & TAP_CAP)&&((cmnd_on(pbpctl_dev)) >= 0))
	{
		if (!tap_mode) {
			tap_off(pbpctl_dev);
		} else {
			tap_on(pbpctl_dev);
		}
		cmnd_off(pbpctl_dev);
		return 0;
	}
	return BP_NOT_CAP;
}

static int get_tap_fn (bpctl_dev_t *pbpctl_dev) {

	return(tap_status(pbpctl_dev));
}

static int set_tap_pwup_fn(bpctl_dev_t *pbpctl_dev, int tap_mode) {
	int ret = 0;

	if ((pbpctl_dev->bp_caps & TAP_PWUP_CTL_CAP)&&((cmnd_on(pbpctl_dev))>=0))
	{
		if (tap_mode) {
			ret = tap_state_pwron(pbpctl_dev);
		} else {
			ret = normal_state_pwron(pbpctl_dev);
		}
		cmnd_off(pbpctl_dev);
	} else ret = BP_NOT_CAP;
	return ret;
}

static int get_tap_pwup_fn(bpctl_dev_t *pbpctl_dev) {
	int ret = 0;

	if ((ret=default_pwron_tap_status(pbpctl_dev)) < 0) {
		return ret;
	}
	return((ret==0) ? 1 : 0);
}

static int get_tap_change_fn(bpctl_dev_t *pbpctl_dev) {

	return(tap_change_status(pbpctl_dev));
}

static int set_dis_tap_fn(bpctl_dev_t *pbpctl_dev, int dis_param){
	int ret = 0;

	if ((pbpctl_dev->bp_caps & TAP_DIS_CAP) && ((cmnd_on(pbpctl_dev)) >= 0))
	{
		if (dis_param) {
			ret = dis_tap_cap(pbpctl_dev);
		} else {
			ret = en_tap_cap(pbpctl_dev);
		}
		cmnd_off(pbpctl_dev);
		return ret;
	} else {
		return BP_NOT_CAP;
	}    
}

static int get_dis_tap_fn(bpctl_dev_t *pbpctl_dev){

	return(dis_tap_cap_status(pbpctl_dev));
}

static int set_disc_fn (bpctl_dev_t *pbpctl_dev, int disc_mode){
	if (!pbpctl_dev) {
		return -1;
	}
	if (!(pbpctl_dev->bp_caps & DISC_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_DISC;
		bp_cmd_buf.cmd.cmd_data.disc_mode = disc_mode ? 1 : 0;

		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}
		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		msec_delay_bp(LATCH_DELAY);

		return ret;

	} else {     
		if ((cmnd_on(pbpctl_dev) >= 0)) {
			if (!disc_mode) {
				disc_off(pbpctl_dev);
			} else {
				disc_on(pbpctl_dev);
			}
			cmnd_off(pbpctl_dev);

			return BP_OK;
		}
		return BP_NOT_CAP;
	}
}

static int get_disc_fn (bpctl_dev_t *pbpctl_dev) {
	int ret=0;
	if (!pbpctl_dev) {
		return -1;
	}
	if (!(pbpctl_dev->bp_caps&DISC_CAP)) {
		return -1;
	}
	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_DISC;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev)))
			return -1;

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				if (bp_rsp_buf.rsp.rsp_data.disc_mode == 1) {
					ret = 1;
				} else if (bp_rsp_buf.rsp.rsp_data.disc_mode == 0) {
					ret = 0;
				}
			}
		}
		return ret;

	} else {
		ret = disc_status(pbpctl_dev);
	}

	return ret;
}

static int set_disc_pwup_fn(bpctl_dev_t *pbpctl_dev, int disc_mode) {
	int ret = 0;

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & DISC_PWUP_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_DISC_PWUP;
		bp_cmd_buf.cmd.cmd_data.disc_pwup = disc_mode?DISC_PWUP_EN:DISC_PWUP_DIS;

		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		msec_delay_bp(EEPROM_WR_DELAY);
		return ret;

	} else {
		if ((cmnd_on(pbpctl_dev) >= 0)) {
			if (disc_mode) {
				ret = disc_state_pwron(pbpctl_dev);
			} else {
				ret = normal_state_pwron(pbpctl_dev);
			}
			cmnd_off(pbpctl_dev);
		} else {
			ret = BP_NOT_CAP;
		}

		return ret;
	}
}

static int get_disc_pwup_fn(bpctl_dev_t *pbpctl_dev){
	int ret = 0;

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & DISC_PWUP_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_DISC_PWUP;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				if (bp_rsp_buf.rsp.rsp_data.disc_pwup == DISC_PWUP_EN) {
					ret = 1;
				} else if (bp_rsp_buf.rsp.rsp_data.disc_pwup == DISC_PWUP_DIS) {
					ret = 0;
				}
			}
		}
		return ret;

	} else  {

		ret=default_pwron_disc_status(pbpctl_dev);
		return(ret == 0 ? 1 : (ret < 0 ? BP_NOT_CAP : 0));
	}
}

static int get_disc_change_fn(bpctl_dev_t *pbpctl_dev) {
	int ret=0;

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & DISC_CAP)) {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;

		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0) ? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_DISC_CHANGE;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}
		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				if (bp_rsp_buf.rsp.rsp_data.disc_change == DISC_CHANGED) {
					ret = 1;
				} else if (bp_rsp_buf.rsp.rsp_data.disc_change == DISC_NOT_CHANGED) {
					ret = 0;
				}
			}
		}
		return ret;

	} else  {
		ret = disc_change_status(pbpctl_dev);
		return ret;
	}
}

static int set_dis_disc_fn(bpctl_dev_t *pbpctl_dev, int dis_param) {
	int ret = 0;

	if (!pbpctl_dev) {
		return -1;
	}
	if (!(pbpctl_dev->bp_caps & DISC_DIS_CAP)) {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0) ? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_DIS_DISC;
		bp_cmd_buf.cmd.cmd_data.dis_disc = dis_param?DIS_DISC_DISABLE:DIS_DISC_ENABLE;

		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		msec_delay_bp(BYPASS_CAP_DELAY);
		return ret;

	} else {
		if ((cmnd_on(pbpctl_dev) >= 0)) {
			if (dis_param) {
				ret = dis_disc_cap(pbpctl_dev);
			} else {
				ret = en_disc_cap(pbpctl_dev);
			}
			cmnd_off(pbpctl_dev);
			return ret;
		} else {
			return BP_NOT_CAP;
		}
	}
} 

static int get_dis_disc_fn(bpctl_dev_t *pbpctl_dev){
	int ret = 0;

	if (!pbpctl_dev) {
		return -1;
	}
	if ((pbpctl_dev->bp_40g) &&
		(pbpctl_dev->bp_caps&DISC_DIS_CAP)) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0) ? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_DIS_DISC;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}
		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				if (bp_rsp_buf.rsp.rsp_data.dis_disc==DIS_DISC_DISABLE) {
					ret = 1;
				} else if (bp_rsp_buf.rsp.rsp_data.dis_disc==DIS_DISC_ENABLE) {
					ret = 0;
				}
			}
		}
		return ret;

	} else  {
		ret = dis_disc_cap_status(pbpctl_dev);
		return ret;
	}
}


static int get_wd_exp_mode_fn(bpctl_dev_t *pbpctl_dev){
	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & WD_CTL_CAP)) {
		return BP_NOT_CAP;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_WD_EXP_MODE;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret=bp_rsp_buf.rsp.rsp_data.wd_exp_mode;
			}
		}
		return ret;

	} else
		return(wdt_exp_mode_status(pbpctl_dev));
}  

static int set_wd_exp_mode_fn(bpctl_dev_t *pbpctl_dev, int param){
	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps & WD_CTL_CAP)) {
		return BP_NOT_CAP;
	}
	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0) ? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_WD_EXP_MODE;
		bp_cmd_buf.cmd.cmd_data.wd_exp_mode = param;

		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		msec_delay_bp(BYPASS_CAP_DELAY);
		return ret;
	} else {
		return(wdt_exp_mode(pbpctl_dev,param));
	}
} 

static int set_tx_fn(bpctl_dev_t *pbpctl_dev, int tx_state) {
	bpctl_dev_t *pbpctl_dev_b = NULL;

	if (!pbpctl_dev) {
		return -1;
	}

	if (!(pbpctl_dev->bp_caps&TX_CTL_CAP)) {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c, *pbpctl_dev_m;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		if (is_bypass_fn(pbpctl_dev)) {
			pbpctl_dev_m = pbpctl_dev;
		} else { 
			pbpctl_dev_m = get_master_port_fn(pbpctl_dev);
		}

		if (!pbpctl_dev_m) {
			return -1;
		}
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev_m->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_TX;
		bp_cmd_buf.cmd.cmd_data.tx_dis.mode = tx_state ? TX_OFF : TX_ON;
		bp_cmd_buf.cmd.cmd_data.tx_dis.port_num =
		(pbpctl_dev_m->func == 0)?pbpctl_dev->func:(pbpctl_dev->func - 2);

		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		return ret;

	} else {
		if ((pbpctl_dev->bp_caps & TPL_CAP)&&
			(pbpctl_dev->bp_caps & SW_CTL_CAP) ) {
			if ((pbpctl_dev->bp_tpl_flag)) {    
				return BP_NOT_CAP;
			}
		} else if ((pbpctl_dev_b = get_master_port_fn(pbpctl_dev))) {
			if ((pbpctl_dev_b->bp_caps & TPL_CAP)&&
				(pbpctl_dev_b->bp_tpl_flag)) {
				return BP_NOT_CAP;
			}
		}
		return(set_tx(pbpctl_dev,tx_state));
	}
}   


static int set_wd_autoreset_fn(bpctl_dev_t *pbpctl_dev, int param) {
	return(set_bypass_wd_auto(pbpctl_dev, param));
}

static int get_wd_autoreset_fn(bpctl_dev_t *pbpctl_dev) {
	return(get_bypass_wd_auto(pbpctl_dev));
}

#ifdef BP_SELF_TEST
static int set_bp_self_test_fn(int dev_num, int param) {
	static bpctl_dev_t *bpctl_dev_curr;

	if ((dev_num < 0) || (dev_num > device_num) || (bpctl_dev_arr[dev_num].pdev == NULL)) {
		return -1;
	}
	bpctl_dev_curr = &bpctl_dev_arr[dev_num];

	return(set_bp_self_test(bpctl_dev_curr, param));
}

static int get_bp_self_test_fn(int dev_num) {
	static bpctl_dev_t *bpctl_dev_curr;

	if ((dev_num < 0) || (dev_num > device_num) || (bpctl_dev_arr[dev_num].pdev == NULL)) {
		return -1;
	}
	bpctl_dev_curr = &bpctl_dev_arr[dev_num];

	return(get_bp_self_test(bpctl_dev_curr));
}

#endif


static int get_bypass_caps_fn(bpctl_dev_t *pbpctl_dev) {
	if (!pbpctl_dev) {
		return -1;
	}
	if (pbpctl_dev->bp_40g) {       
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c, *pbpctl_dev_m;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));

		if (is_bypass_fn(pbpctl_dev)) {
			pbpctl_dev_m = pbpctl_dev;
		} else  {
			pbpctl_dev_m = get_master_port_fn(pbpctl_dev);
		}

		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev_m->func == 0)? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_BYPASS_CAPS;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}
		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret=(ntohl(bp_rsp_buf.rsp.rsp_data.bypass_caps));
				if ((is_bypass_fn(pbpctl_dev)) != 1) {
					int ret1 = 0;
					if (ret&TX_CTL_CAP) {
						ret1=(TX_CTL_CAP|TX_STATUS_CAP);
					}
					return ret1;

				}
			}
		}

		return ret;

	} else {
		return(pbpctl_dev->bp_caps);
	}
}


static int get_bypass_slave_fn(bpctl_dev_t *pbpctl_dev,bpctl_dev_t **pbpctl_dev_out){
	int idx_dev=0;

	if (!pbpctl_dev) {
		return -1;
	}
	if ((pbpctl_dev->func == 0) || (pbpctl_dev->func == 2)) {
		for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev != NULL) && (idx_dev<device_num)); idx_dev++) {
			if ((bpctl_dev_arr[idx_dev].bus == pbpctl_dev->bus) &&
				(bpctl_dev_arr[idx_dev].slot==pbpctl_dev->slot)) {
				if ((pbpctl_dev->func==0) &&
					(bpctl_dev_arr[idx_dev].func==1)) {
					*pbpctl_dev_out=&bpctl_dev_arr[idx_dev];
					return 1;
				}
				if ((pbpctl_dev->func == 2) &&
					(bpctl_dev_arr[idx_dev].func == 3)) {
					*pbpctl_dev_out=&bpctl_dev_arr[idx_dev];
					return 1;
				}
			}
		}
		return -1;
	} else {
		return 0;
	}   
}

static int is_bypass(bpctl_dev_t *pbpctl_dev) {

	if ((pbpctl_dev->func == 0) || (pbpctl_dev->func == 2)) {
		return 1;
	} else {
		return 0;
	}
}

static int get_tx_fn(bpctl_dev_t *pbpctl_dev) {
	bpctl_dev_t *pbpctl_dev_b=NULL;

	if (!pbpctl_dev) {
		return -1;
	}
	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c, *pbpctl_dev_m;
		int ret = -1;

		if (!(pbpctl_dev->bp_caps & TX_CTL_CAP)) {      
			return -1;
		}

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		if (is_bypass_fn(pbpctl_dev)) {
			pbpctl_dev_m = pbpctl_dev;
		} else { 
			pbpctl_dev_m = get_master_port_fn(pbpctl_dev);
		}

		if (!pbpctl_dev_m) {
			return -1;
		}
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev_m->func == 0) ? 0 : 1;
		bp_cmd_buf.cmd.cmd_data.tx_dis.port_num =
		(pbpctl_dev_m->func == 0)?pbpctl_dev->func:(pbpctl_dev->func-2);

		bp_cmd_buf.cmd.cmd_id = CMD_GET_TX;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) { 
				if (bp_rsp_buf.rsp.rsp_data.tx_dis.mode == TX_ON) {
					ret = 0;
				} else if (bp_rsp_buf.rsp.rsp_data.tx_dis.mode==TX_OFF) {
					ret = 1;
				}
			} else {
				return -1;
			}
		}
		return ret;

	} else {
		if ((pbpctl_dev->bp_caps & TPL_CAP) &&
			(pbpctl_dev->bp_caps & SW_CTL_CAP)) {
			if ((pbpctl_dev->bp_tpl_flag)) {
				return BP_NOT_CAP;
			}
		} else if ((pbpctl_dev_b = get_master_port_fn(pbpctl_dev))) {
			if ((pbpctl_dev_b->bp_caps&TPL_CAP) &&
				(pbpctl_dev_b->bp_tpl_flag)) {
				return BP_NOT_CAP;
			}
		}
		return(tx_status(pbpctl_dev));
	}
}


static int get_bypass_link_status (bpctl_dev_t *pbpctl_dev) {
	if (pbpctl_dev->media_type == bp_fiber) {
		return((BPCTL_READ_REG(pbpctl_dev, CTRL) & BPCTLI_CTRL_SWDPIN1));
	} else {
		return((BPCTL_READ_REG(pbpctl_dev, STATUS) & BPCTLI_STATUS_LU));
	}

}


static void bp_tpl_timer_fn(void *param){
	bpctl_dev_t *pbpctl_dev = (bpctl_dev_t *) param;
	uint32_t link1, link2;
	bpctl_dev_t *pbpctl_dev_b = NULL;

	if (!(pbpctl_dev_b = get_status_port_fn(pbpctl_dev))) {
		return;
	}

	if (!pbpctl_dev->bp_tpl_flag) {
		set_tx(pbpctl_dev_b, 1);
		set_tx(pbpctl_dev, 1);
		return;
	}
	link1 = get_bypass_link_status(pbpctl_dev);
	link2 = get_bypass_link_status(pbpctl_dev_b);
	if ((link1)&&(tx_status(pbpctl_dev))) {
		if ((!link2)&&(tx_status(pbpctl_dev_b))) {
			set_tx(pbpctl_dev, 0);
		} else if (!tx_status(pbpctl_dev_b)) {
			set_tx(pbpctl_dev_b, 1);
		}
	} else if ((!link1)&&(tx_status(pbpctl_dev))) {
		if ((link2)&&(tx_status(pbpctl_dev_b))) {
			set_tx(pbpctl_dev_b, 0);
		}
	} else if ((link1)&&(!tx_status(pbpctl_dev))) {
		if ((link2)&&(tx_status(pbpctl_dev_b))) {
			set_tx(pbpctl_dev, 1);
		}
	} else if ((!link1)&&(!tx_status(pbpctl_dev))) {
		if ((link2)&&(tx_status(pbpctl_dev_b))) {
			set_tx(pbpctl_dev, 1);  
		}
	}
	callout_reset(&pbpctl_dev->bp_tpl_timer, BP_LINK_MON_DELAY*hz, bp_tpl_timer_fn, pbpctl_dev);
}


static void remove_bypass_tpl_auto(bpctl_dev_t *pbpctl_dev) {
	bpctl_dev_t *pbpctl_dev_b = NULL;

	pbpctl_dev_b = get_status_port_fn(pbpctl_dev);

	if (pbpctl_dev->bp_caps&TPL_CAP) {
		callout_drain(&pbpctl_dev->bp_tpl_timer);
		pbpctl_dev->bp_tpl_flag = 0;
		pbpctl_dev_b=get_status_port_fn(pbpctl_dev);
		if (pbpctl_dev_b) {
			set_tx(pbpctl_dev_b, 1);
		}
		set_tx(pbpctl_dev, 1);
	}
	return;    
}
static int init_bypass_tpl_auto(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&TPL_CAP)
	{
		callout_init(&pbpctl_dev->bp_tpl_timer, 1);
		return BP_OK;
	}
	return BP_NOT_CAP; 
}

static int set_bypass_tpl_auto(bpctl_dev_t *pbpctl_dev, unsigned int param) {
	if (pbpctl_dev->bp_caps&TPL_CAP) {
		if ((param)&&(!pbpctl_dev->bp_tpl_flag)) {
			pbpctl_dev->bp_tpl_flag=param;
			callout_reset(&pbpctl_dev->bp_tpl_timer, 1, bp_tpl_timer_fn, pbpctl_dev);
			return BP_OK;
		};
		if ((!param)&&(pbpctl_dev->bp_tpl_flag)) {
			remove_bypass_tpl_auto(pbpctl_dev);
		}
		return BP_OK;
	}
	return BP_NOT_CAP; 
}
#if 0
static int get_bypass_tpl_auto(bpctl_dev_t *pbpctl_dev){
	if (pbpctl_dev->bp_caps&TPL_CAP)
	{
		return pbpctl_dev->bp_tpl_flag;
	}
	return BP_NOT_CAP; 
}
#endif
static int set_tpl_fn(bpctl_dev_t *pbpctl_dev, int tpl_mode) {

	bpctl_dev_t *pbpctl_dev_b = NULL;

	if (!pbpctl_dev) {
		return -1;
	}
	if (pbpctl_dev->bp_40g)  {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		if (!(pbpctl_dev->bp_caps & TPL_CAP))
			return -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_TPL;
		bp_cmd_buf.cmd.cmd_data.tpl_mode = tpl_mode?TPL_ON:TPL_OFF;

		if (!(pbpctl_dev_c=get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret=0;
			}
		}
		return ret;

	} else {
		pbpctl_dev_b = get_status_port_fn(pbpctl_dev);

		if (pbpctl_dev->bp_caps & TPL_CAP) {
			if (tpl_mode) {
				if ((pbpctl_dev_b = get_status_port_fn(pbpctl_dev))) {
					set_tx(pbpctl_dev_b,1);
				}
				set_tx(pbpctl_dev, 1);
			}
			if ((TPL_IF_SERIES(pbpctl_dev->subdevice)) ||
				(pbpctl_dev->bp_caps_ex&TPL2_CAP_EX)) {
				pbpctl_dev->bp_tpl_flag = tpl_mode;
				if (!tpl_mode) {
					tpl_hw_off(pbpctl_dev);
				} else {
					tpl_hw_on(pbpctl_dev);
				}
			} else {
				set_bypass_tpl_auto(pbpctl_dev, tpl_mode);
			}
			return 0;
		}
		return BP_NOT_CAP;
	}
}

static int get_tpl_fn(bpctl_dev_t *pbpctl_dev) {
	int ret = BP_NOT_CAP;

	if (!pbpctl_dev) {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		if (!(pbpctl_dev->bp_caps & TPL_CAP)) {
			return -1;
		}

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_TPL;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}
		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_data.tpl_mode == TPL_ON) {
				ret=1;
			} else if (bp_rsp_buf.rsp.rsp_data.tpl_mode == TPL_OFF) {
				ret=0;
			}
		}
		return ret;

	} else {
		if (pbpctl_dev->bp_caps & TPL_CAP) {
			if (pbpctl_dev->bp_caps_ex & TPL2_CAP_EX) {
				return(tpl2_flag_status(pbpctl_dev));
			}
			ret = pbpctl_dev->bp_tpl_flag;
		}
	}
	return ret;
}

static int set_bp_wait_at_pwup_fn(bpctl_dev_t *pbpctl_dev, int bp_wait_at_pwup) {
	if (!pbpctl_dev)  {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		if (!(pbpctl_dev->bp_caps & BP_CAP)) {
			return -1;
		}

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_BP_WAIT_AT_PWUP;
		bp_cmd_buf.cmd.cmd_data.bp_wait_at_pwup = bp_wait_at_pwup?1:0;

		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		msec_delay_bp(EEPROM_WR_DELAY);
		return ret;

	} else {
		if (pbpctl_dev->bp_caps & SW_CTL_CAP) {
			//bp_lock(pbp_device_block);
			cmnd_on(pbpctl_dev);
			if (!bp_wait_at_pwup) {
				bp_wait_at_pwup_dis(pbpctl_dev);
			} else {
				bp_wait_at_pwup_en(pbpctl_dev);
			}
			cmnd_off(pbpctl_dev);
			// bp_unlock(pbp_device_block);
			return BP_OK;
		}
		return BP_NOT_CAP;
	}
}

static int get_bp_wait_at_pwup_fn(bpctl_dev_t *pbpctl_dev) {
	int ret = 0;

	if (!pbpctl_dev) {
		return -1;
	}
	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		if (!(pbpctl_dev->bp_caps&BP_CAP)) {
			return -1;
		}

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0)? 0:1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_BP_WAIT_AT_PWUP;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_data.bp_wait_at_pwup==1) {
				ret=1;
			} else if (bp_rsp_buf.rsp.rsp_data.bp_wait_at_pwup==0) {
				ret=0;
			}
		}
		return ret;

	} else {
		// bp_lock(pbp_device_block);
		ret=bp_wait_at_pwup_status(pbpctl_dev);
		// bp_unlock(pbp_device_block);
		return ret;
	}
}

static int set_bp_hw_reset_fn(bpctl_dev_t *pbpctl_dev, int bp_hw_reset) {
	if (!pbpctl_dev) {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		if (!(pbpctl_dev->bp_caps&BP_CAP)) {
			return -1;
		}

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0) ? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_BP_HW_RESET;
		bp_cmd_buf.cmd.cmd_data.bp_hw_reset = bp_hw_reset ? 1 : 0;

		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		msec_delay_bp(EEPROM_WR_DELAY);
		return ret;

	} else {
		if (pbpctl_dev->bp_caps & SW_CTL_CAP) {
			//   bp_lock(pbp_device_block);
			cmnd_on(pbpctl_dev);

			if (!bp_hw_reset) {
				bp_hw_reset_dis(pbpctl_dev);
			} else {
				bp_hw_reset_en(pbpctl_dev);
			}
			cmnd_off(pbpctl_dev);
			//    bp_unlock(pbp_device_block);
			return BP_OK;
		}
		return BP_NOT_CAP;
	}
}

static int get_bp_hw_reset_fn(bpctl_dev_t *pbpctl_dev) {
	int ret = 0;

	if (!pbpctl_dev) {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		ret = -1;

		if (!(pbpctl_dev->bp_caps&BP_CAP)) {
			return -1;
		}

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0) ? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_BP_HW_RESET;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_data.bp_hw_reset == 1) {
				ret = 1;
			} else if (bp_rsp_buf.rsp.rsp_data.bp_hw_reset == 0) {
				ret = 0;
			}
		}
		return ret;

	} else {

		//bp_lock(pbp_device_block);
		ret = bp_hw_reset_status(pbpctl_dev);

		//bp_unlock(pbp_device_block);

		return ret;
	}
}



static int get_bypass_info_fn(bpctl_dev_t *pbpctl_dev, char *dev_name, char *add_param) {
	if (!pbpctl_dev) {
		return -1;
	}
	if (!is_bypass_fn(pbpctl_dev)) {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;
		int ret = -1;

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0) ? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_GET_BYPASS_INFO;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev))) {
			return -1;
		}

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			strcpy(dev_name, pbpctl_dev->name);
			*add_param = bp_rsp_buf.rsp.rsp_data.bypass_info.fw_ver;
			ret = 0;
		}
		return ret;

	} else {
		strcpy(dev_name,pbpctl_dev->name);
		*add_param = pbpctl_dev->bp_fw_ver;
		return 0;
	}
}

static int get_dev_idx(int ifindex) {
	int idx_dev=0;

	for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev != NULL) && (idx_dev<device_num)); idx_dev++)
	{
		if (ifindex == bpctl_dev_arr[idx_dev].ifindex)
			return idx_dev;
	}
	return -1;
}

static int get_pair_idx(int dev_num) {
	int idx_dev;
	int func_pair;
	static bpctl_dev_t *bpctl_dev_curr;

	bpctl_dev_curr=&bpctl_dev_arr[dev_num];
	func_pair = (bpctl_dev_curr->func == 0) ? 1 : 0;

	for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev != NULL) && (idx_dev < device_num)); idx_dev++)
	{
		if ((bpctl_dev_arr[idx_dev].bus == bpctl_dev_curr->bus)&&
			(bpctl_dev_arr[idx_dev].slot == bpctl_dev_curr->slot)&&
			(bpctl_dev_arr[idx_dev].func == func_pair))
		{
			return idx_dev;
		}
	}
	return -1;
}

static int set_bp_manuf_fn(bpctl_dev_t *pbpctl_dev) {
	int ret = -1;

	if (!pbpctl_dev) {
		return -1;
	}

	if (pbpctl_dev->bp_40g) {
		bp_cmd_t bp_cmd_buf;
		bp_cmd_rsp_t bp_rsp_buf;
		bpctl_dev_t *pbpctl_dev_c;

		ret = -1;

		if (!(pbpctl_dev->bp_caps & BP_CAP)) {
			return -1;
		}

		memset(&bp_cmd_buf, 0, sizeof(bp_cmd_buf));
		memset(&bp_rsp_buf, 0, sizeof(bp_cmd_rsp_t));
		bp_cmd_buf.cmd.cmd_dev_num = (pbpctl_dev->func == 0) ? 0 : 1;
		bp_cmd_buf.cmd.cmd_id = CMD_SET_BP_MANUF;
		if (!(pbpctl_dev_c = get_status_port_fn(pbpctl_dev)))
			return -1;

		if (bp_cmd_request(pbpctl_dev_c, &bp_cmd_buf, &bp_rsp_buf)) {
			if (bp_rsp_buf.rsp.rsp_id == BP_ERR_OK) {
				ret = 0;
			}
		}
		msec_delay_bp(BYPASS_CAP_DELAY);
		return ret;

	} else {
		//bp_lock(pbp_device_block);

		//bp_unlock(pbp_device_block);

		return ret;
	}
}


static int
bpmod_open(struct cdev *dev, int flag, int otyp, struct thread *td)
{
	return(0);
}

static int
bpmod_close(struct cdev *dev, int flag, int otyp, struct thread *td)
{
	return(0);
}


static int get_dev_idx_bsf(int bus, int slot, int func) {
	int idx_dev=0;
	for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev!=NULL)&&(idx_dev<device_num)); idx_dev++) {
		if ((bus==bpctl_dev_arr[idx_dev].bus) &&
			(slot==bpctl_dev_arr[idx_dev].slot) &&
			(func==bpctl_dev_arr[idx_dev].func) ) {
			return idx_dev;
		}
	}
	return -1;
}


static   int
bpmod_ioctl(struct cdev *dev, u_long cmd, caddr_t arg, int mode,
			struct thread *td)
{


	struct bpctl_cmd *bpctl_cmd;
	int dev_idx = 0;
	bpctl_dev_t *pbpctl_dev_out = 0;
	bpctl_dev_t *pbpctl_dev = 0;


	bpctl_cmd = (struct bpctl_cmd *)arg;
	if (bpctl_cmd == NULL) {
		return EPERM;
	}


/*
* Switch according to the ioctl called
*/
	if (cmd == IOCTL_TX_MSG(IF_SCAN)) {    
		return SUCCESS;
	}
	if (cmd == IOCTL_TX_MSG(GET_DEV_NUM)) {    
		bpctl_cmd->out_param[0] = device_num;
		return SUCCESS;
	}


	mtx_lock(&mtx);
	if ((bpctl_cmd->in_param[5])||
		(bpctl_cmd->in_param[6])||
		(bpctl_cmd->in_param[7])) {
		dev_idx = get_dev_idx_bsf(bpctl_cmd->in_param[5],
								  bpctl_cmd->in_param[6],
								  bpctl_cmd->in_param[7]);
	} else {
		dev_idx = (bpctl_cmd->in_param[1] == 0)?bpctl_cmd->in_param[0]:get_dev_idx(bpctl_cmd->in_param[1]);
	}
	if (dev_idx<0||dev_idx>device_num) {
		mtx_unlock(&mtx);
		return -EOPNOTSUPP;
	}

	bpctl_cmd->out_param[0]= bpctl_dev_arr[dev_idx].bus;
	bpctl_cmd->out_param[1]= bpctl_dev_arr[dev_idx].slot;
	bpctl_cmd->out_param[2]= bpctl_dev_arr[dev_idx].func;
	bpctl_cmd->out_param[3]= bpctl_dev_arr[dev_idx].ifindex;


	if (bpctl_dev_arr[dev_idx].bp_10gb) {       
		int pair_idx;

		if (!(device_is_attached(bpctl_dev_arr[dev_idx].pdev))) {
			printf("Please load network driver for %s adapter!\n",bpctl_dev_arr[dev_idx].name);
			bpctl_cmd->status=-1;
			goto exit_ioctl;
		}
		if (bpctl_dev_arr[dev_idx].ndev) {
			if (!(bpctl_dev_arr[dev_idx].ndev->if_flags & IFF_UP)) {
				printf("Please bring up network interfaces for %s adapter!\n",
					   bpctl_dev_arr[dev_idx].name);
				bpctl_cmd->status = -1;
				goto exit_ioctl;
			}
		}
		if ((pair_idx = get_pair_idx(dev_idx)) != -1) {
			if (bpctl_dev_arr[pair_idx].ndev) {
				if (!(bpctl_dev_arr[pair_idx].ndev->if_flags & IFF_UP)) {
					printf("Please bring up network interfaces for %s adapter!\n",
						   bpctl_dev_arr[pair_idx].name);
					bpctl_cmd->status = -1;
					goto exit_ioctl;
				}
			}
		} else {
			printf("Error: No pair device found!!!\nPlease reload bpmod driver.\n" );
			bpctl_cmd->status = -1;
			goto exit_ioctl;
		}
	}
	pbpctl_dev = &bpctl_dev_arr[dev_idx];
	switch (cmd) {
	case IOCTL_TX_MSG(SET_BYPASS_PWOFF) :
		bpctl_cmd->status = set_bypass_pwoff_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;                              

	case IOCTL_TX_MSG(GET_BYPASS_PWOFF) :
		bpctl_cmd->status = get_bypass_pwoff_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_BYPASS_PWUP) :
		bpctl_cmd->status = set_bypass_pwup_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_BYPASS_PWUP) :
		bpctl_cmd->status = get_bypass_pwup_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_BYPASS_WD) :
		bpctl_cmd->status = set_bypass_wd_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_BYPASS_WD) :
		bpctl_cmd->status = get_bypass_wd_fn(pbpctl_dev, (int *)&(bpctl_cmd->data[0]));
		break;

	case IOCTL_TX_MSG(GET_WD_EXPIRE_TIME) :
		bpctl_cmd->status = get_wd_expire_time_fn(pbpctl_dev, (int *)&(bpctl_cmd->data[0]));
		break;

	case IOCTL_TX_MSG(RESET_BYPASS_WD_TIMER) :
		bpctl_cmd->status = reset_bypass_wd_timer_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(GET_WD_SET_CAPS) :
		bpctl_cmd->status = get_wd_set_caps_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_STD_NIC) :
		bpctl_cmd->status= set_std_nic_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_STD_NIC) :
		bpctl_cmd->status= get_std_nic_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_TAP) :
		bpctl_cmd->status= set_tap_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_TAP) :
		bpctl_cmd->status= get_tap_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(GET_TAP_CHANGE) :
		bpctl_cmd->status= get_tap_change_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_DIS_TAP) :
		bpctl_cmd->status= set_dis_tap_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_DIS_TAP) :
		bpctl_cmd->status= get_dis_tap_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_TAP_PWUP) :
		bpctl_cmd->status= set_tap_pwup_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_TAP_PWUP) :
		bpctl_cmd->status= get_tap_pwup_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_WD_EXP_MODE):
		bpctl_cmd->status= set_wd_exp_mode_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_WD_EXP_MODE):
		bpctl_cmd->status= get_wd_exp_mode_fn(pbpctl_dev);
		break;

	case  IOCTL_TX_MSG(GET_DIS_BYPASS):
		bpctl_cmd->status= get_dis_bypass_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_DIS_BYPASS):
		bpctl_cmd->status= set_dis_bypass_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_BYPASS_CHANGE):
		bpctl_cmd->status= get_bypass_change_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(GET_BYPASS):
		bpctl_cmd->status= get_bypass_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_BYPASS):
		bpctl_cmd->status= set_bypass_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_BYPASS_CAPS):
		bpctl_cmd->status= get_bypass_caps_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(GET_BYPASS_SLAVE):
		bpctl_cmd->status= get_bypass_slave_fn(pbpctl_dev, &pbpctl_dev_out);
		if (bpctl_cmd->status==1)
		{
			bpctl_cmd->out_param[4]= pbpctl_dev_out->bus;
			bpctl_cmd->out_param[5]= pbpctl_dev_out->slot;
			bpctl_cmd->out_param[6]= pbpctl_dev_out->func;
			bpctl_cmd->out_param[7]= pbpctl_dev_out->ifindex;
		}
		break;

	case IOCTL_TX_MSG(IS_BYPASS):
		bpctl_cmd->status= is_bypass(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_TX):
		bpctl_cmd->status= set_tx_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_TX):
		bpctl_cmd->status= get_tx_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_WD_AUTORESET):
		bpctl_cmd->status= set_wd_autoreset_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_WD_AUTORESET):
		bpctl_cmd->status= get_wd_autoreset_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_DISC) :
		bpctl_cmd->status=set_disc_fn(pbpctl_dev,bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_DISC) :
		bpctl_cmd->status=get_disc_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(GET_DISC_CHANGE) :
		bpctl_cmd->status=get_disc_change_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_DIS_DISC) :
		bpctl_cmd->status=set_dis_disc_fn(pbpctl_dev,bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_DIS_DISC) :
		bpctl_cmd->status=get_dis_disc_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_DISC_PWUP) :
		bpctl_cmd->status=set_disc_pwup_fn(pbpctl_dev,bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_DISC_PWUP) :
		bpctl_cmd->status=get_disc_pwup_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(GET_BYPASS_INFO):
		bpctl_cmd->status= get_bypass_info_fn(pbpctl_dev, (char *)&bpctl_cmd->data, (char *)&bpctl_cmd->out_param[4]);
		break;

	case IOCTL_TX_MSG(SET_TPL) :
		bpctl_cmd->status= set_tpl_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_TPL) :
		bpctl_cmd->status= get_tpl_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_BP_WAIT_AT_PWUP) :
		bpctl_cmd->status= set_bp_wait_at_pwup_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_BP_WAIT_AT_PWUP) :
		bpctl_cmd->status= get_bp_wait_at_pwup_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_BP_HW_RESET) :
		bpctl_cmd->status= set_bp_hw_reset_fn(pbpctl_dev, bpctl_cmd->in_param[2]);
		break;

	case IOCTL_TX_MSG(GET_BP_HW_RESET) :
		bpctl_cmd->status= get_bp_hw_reset_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(SET_BP_MANUF) :
		bpctl_cmd->status= set_bp_manuf_fn(pbpctl_dev);
		break;

	case IOCTL_TX_MSG(GET_WD_EXPIRE):
		bpctl_cmd->status= get_wd_expire_fn(pbpctl_dev);
		break;

	default:
		mtx_unlock(&mtx);
		return -EOPNOTSUPP;
	}


	exit_ioctl:
	mtx_unlock(&mtx);
	return SUCCESS;
}

static struct cdevsw bpmod_devsw = {
	  .d_version = D_VERSION,
		.d_open = bpmod_open,
		.d_close = bpmod_close,
		.d_ioctl = bpmod_ioctl,
		.d_name = "bpmod",
}; 



#ifndef PCI_DEVICE
	#define PCI_DEVICE(vend,dev) \
	.vendor = (vend), .device = (dev), \
	.subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID
#endif



#define SILICOM_E1000BP_ETHERNET_DEVICE(device_id) {\
	PCI_DEVICE(SILICOM_VID, device_id)}

typedef enum {
	PXG2BPFI,
	PXG2BPFIL,
	PXG2BPFILX,
	PXG2BPFILLX,
	PXGBPI,
	PXGBPIG,
	PXG2TBFI,
	PXG4BPI,
	PXG4BPFI,
	PEG4BPI,
	PEG2BPI,
	PEG4BPIN,
	PEG2BPFI,
	PEG2BPFILX,
	PMCXG2BPFI,
	PMCXG2BPFIN,
	PEG4BPII,
	PEG4BPFII,
	PXG4BPFILX,
	PMCXG2BPIN,
	PMCXG4BPIN,
	PXG2BISC1,
	PEG2TBFI,
	PXG2TBI,
	PXG4BPFID,
	PEG4BPFI,
	PEG4BPIPT,
	PXG6BPI,
	PEG4BPIL,
	PMCXG2BPIN2,
	PMCXG4BPIN2,
	PMCX2BPI,
	PEG2BPFID,
	PEG2BPFIDLX,
	PMCX4BPI,
	MEG2BPFILN,
	MEG2BPFINX,
	PEG4BPFILX,
	PE10G2BPISR,
	PE10G2BPILR,
	MHIO8AD,
	PE10G2BPICX4,
	PEG2BPI5,
	PEG6BPI,
	PEG4BPFI5,
	PEG4BPFI5LX,
	MEG2BPFILXLN,
	PEG2BPIX1,
	MEG2BPFILXNX,
	XE10G2BPIT,
	XE10G2BPICX4,
	XE10G2BPISR,
	XE10G2BPILR,
	PEG4BPIIO,
	XE10G2BPIXR,
	PE10GDBISR,
	PE10GDBILR,
	PEG2BISC6,
	PEG6BPIFC,
	PE10G2BPTCX4,
	PE10G2BPTSR,
	PE10G2BPTLR,
	PE10G2BPTT,
	PEG4BPI6,
	PEG4BPFI6,
	PEG4BPFI6LX,
	PEG4BPFI6ZX,
	PEG4BPFI6CS,
	PEG2BPI6,
	PEG2BPFI6,
	PEG2BPFI6LX,
	PEG2BPFI6ZX,

	PEG2BPFI6FLXM,
	PEG2BPFI6FLXMRB2,

	PEG4BPI6FC,
	PEG4BPFI6FC,
	PEG4BPFI6FCLX,
	PEG4BPFI6FCZX,
	PEG6BPI6,
	PEG2BPI6SC6,
	MEG2BPI6,
	XEG2BPI6,
	MEG4BPI6,
	PEG2BPFI5,
	PEG2BPFI5LX,
	PXEG4BPFI,
	M1EG2BPI6,
	M1EG2BPFI6,
	M1EG2BPFI6LX,
	M1EG2BPFI6ZX,
	M1EG4BPI6,
	M1EG4BPFI6,
	M1EG4BPFI6LX,
	M1EG4BPFI6ZX,
	M1EG6BPI6,
	M1E2G4BPi80,
	M1E2G4BPFi80,
	M1E2G4BPFi80LX,
	M1E2G4BPFi80ZX,
	PE210G2SPI9,
	M1E10G2BPI9CX4, 
	M1E10G2BPI9SR, 
	M1E10G2BPI9LR, 
	PE210G2BPI9CX4,
	PE210G2BPI9SR,
	PE210G2BPI9LR,
	PE210G2BPI9SRD,
	PE210G2BPI9LRD,
	PE210G2BPI9T,
	M1E210G2BPI9SRDJP,
	M1E210G2BPI9SRDJP1,
	M1E210G2BPI9LRDJP,
	M1E210G2BPI9LRDJP1,
	M2EG2BPFI6,
	M2EG2BPFI6LX,
	M2EG2BPFI6ZX,
	M2EG4BPI6,
	M2EG4BPFI6,
	M2EG4BPFI6LX,
	M2EG4BPFI6ZX,
	M2EG6BPI6,
	PEG2DBI6,   
	PEG2DBFI6,  
	PEG2DBFI6LX,
	PEG2DBFI6ZX,
	PE2G4BPi80, 
	PE2G4BPFi80, 
	PE2G4BPFi80LX,
	PE2G4BPFi80ZX,
	PE2G4BPi80L,
	M6E2G8BPi80A,

	PE2G2BPi35,
	PAC1200BPi35,
	PE2G2BPFi35,
	PE2G2BPFi35ARB2,
	PE2G2BPFi35LX,
	PE2G2BPFi35ALXRB2,
	PE2G2BPFi35ZX,
	PE2G4BPi35,
	PE2G4BPi35L,
	PE2G4BPi35ALRB2,
	PE2G4BPFi35,
	PE2G4BPFi35ARB2,
	PE2G4BPFi35CS,
	PE2G4BPFi35LX,
	PE2G4BPFi35ALXRB2,
	PE2G4BPFi35ZX,

	PE2G6BPi35,
	PE2G6BPi35CX,



	PE2G2BPi80, 
	PE2G2BPFi80, 
	PE2G2BPFi80LX,
	PE2G2BPFi80ZX,
	M2E10G2BPI9CX4, 
	M2E10G2BPI9SR, 
	M2E10G2BPI9LR, 
	M2E10G2BPI9T,
	M6E2G8BPi80,
	PE210G2DBi9SR,
	PE210G2DBi9SRRB2,

	PE210G2DBi9LR,

	PE210G2DBi9LRRB2,
	PE310G4DBi940SR,
	PE310G4DBi940SRRB2,

	PE310G4DBi940LR,
	PE310G4DBi940LRRB2,

	PE310G4DBi940T,
	PE310G4DBi940TRB2,

	PE310G4BPi9T,
	PE310G4BPi9SR,
	PE310G4BPi9LR,
	PE310G4BPi9SRD,
	PE310G4BPi9LRD,

	PE210G2BPi40,
	PE310G4BPi40,
	M1E210G2BPI40T,
	M6E310G4BPi9SR,
	M6E310G4BPi9LR,
	PE2G6BPI6CS,
	PE2G6BPI6,

	M1E2G4BPi35,
	M1E2G4BPFi35,
	M1E2G4BPFi35LX,
	M1E2G4BPFi35ZX,

	M1E2G4BPi35JP,
	M1E2G4BPi35JP1,

	PE310G4DBi9T,

	PE310G4BPI71,
	PE340G2BPI71,
	PE310G4BPI71SRD,
	PE310G4BPI71LRD,
	PE310G2BPI71SRD,
	PE310G2BPI71LRD,
	PE340G2BPI71QS4,
	PE340G2BPI71QL4,
} board_t;


typedef struct _bpmod_info_t {
	unsigned int vendor;
	unsigned int device;
	unsigned int subvendor;
	unsigned int subdevice;
	unsigned int index;
	char *bp_name;

} bpmod_info_t;


typedef struct _dev_desc {
	char *name;
} dev_desc_t;

dev_desc_t dev_desc[]={
	{"Silicom Bypass PXG2BPFI-SD series adapter"},
	{"Silicom Bypass PXG2BPFIL-SD series adapter"}, 
	{"Silicom Bypass PXG2BPFILX-SD series adapter"},
	{"Silicom Bypass PXG2BPFILLX-SD series adapter"},
	{"Silicom Bypass PXG2BPI-SD series adapter"},    
	{"Silicom Bypass PXG2BPIG-SD series adapter"},   
	{"Silicom Bypass PXG2TBFI-SD series adapter"},  
	{"Silicom Bypass PXG4BPI-SD series adapter"},   
	{"Silicom Bypass PXG4BPFI-SD series adapter"},   
	{"Silicom Bypass PEG4BPI-SD series adapter"},
	{"Silicom Bypass PEG2BPI-SD series adapter"},
	{"Silicom Bypass PEG4BPIN-SD series adapter"},
	{"Silicom Bypass PEG2BPFI-SD series adapter"},
	{"Silicom Bypass PEG2BPFI-LX-SD series adapter"},
	{"Silicom Bypass PMCX2BPFI-SD series adapter"},
	{"Silicom Bypass PMCX2BPFI-N series adapter"},  
	{"Intel Bypass PEG2BPII series adapter"},
	{"Intel Bypass PEG2BPFII series adapter"},
	{"Silicom Bypass PXG4BPFILX-SD series adapter"},
	{"Silicom Bypass PMCX2BPI-N series adapter"},
	{"Silicom Bypass PMCX4BPI-N series adapter"},
	{"Silicom Bypass PXG2BISC1-SD series adapter"},
	{"Silicom Bypass PEG2TBFI-SD series adapter"},
	{"Silicom Bypass PXG2TBI-SD series adapter"},
	{"Silicom Bypass PXG4BPFID-SD series adapter"},
	{"Silicom Bypass PEG4BPFI-SD series adapter"},
	{"Silicom Bypass PEG4BPIPT-SD series adapter"},
	{"Silicom Bypass PXG6BPI-SD series adapter"},
	{"Silicom Bypass PEG4BPIL-SD series adapter"},
	{"Silicom Bypass PMCX2BPI-N2 series adapter"},
	{"Silicom Bypass PMCX4BPI-N2 series adapter"},
	{"Silicom Bypass PMCX2BPI-SD series adapter"}, 
	{"Silicom Bypass PEG2BPFID-SD series adapter"},
	{"Silicom Bypass PEG2BPFIDLX-SD series adapter"},
	{"Silicom Bypass PMCX4BPI-SD series adapter"}, 
	{"Silicom Bypass MEG2BPFILN-SD series adapter"},
	{"Silicom Bypass MEG2BPFINX-SD series adapter"},
	{"Silicom Bypass PEG4BPFILX-SD series adapter"},
	{"Silicom Bypass PE10G2BPISR-SD series adapter"},
	{"Silicom Bypass PE10G2BPILR-SD series adapter"},
	{"Silicom Bypass MHIO8AD-SD series adapter"},
	{"Silicom Bypass PE10G2BPICX4-SD series adapter"},
	{"Silicom Bypass PEG2BPI5-SD series adapter"},
	{"Silicom Bypass PEG6BPI5-SD series adapter"},
	{"Silicom Bypass PEG4BPFI5-SD series adapter"},
	{"Silicom Bypass PEG4BPFI5LX-SD series adapter"},
	{"Silicom Bypass MEG2BPFILXLN-SD series adapter"},
	{"Silicom Bypass PEG2BPIX1-SD series adapter"},
	{"Silicom Bypass MEG2BPFILXNX-SD series adapter"},
	{"Silicom Bypass XE10G2BPIT-SD series adapter"},
	{"Silicom Bypass XE10G2BPICX4-SD series adapter"}, 
	{"Silicom Bypass XE10G2BPISR-SD series adapter"},
	{"Silicom Bypass XE10G2BPILR-SD series adapter"},
	{"Intel Bypass PEG2BPFII0 series adapter"},
	{"Silicom Bypass XE10G2BPIXR series adapter"},
	{"Silicom Bypass PE10G2DBISR series adapter"},
	{"Silicom Bypass PEG2BI5SC6 series adapter"},
	{"Silicom Bypass PEG6BPI5FC series adapter"},

	{"Silicom Bypass PE10G2BPTCX4 series adapter"},
	{"Silicom Bypass PE10G2BPTSR series adapter"},
	{"Silicom Bypass PE10G2BPTLR series adapter"},
	{"Silicom Bypass PE10G2BPTT series adapter"},
	{"Silicom Bypass PEG4BPI6 series adapter"},
	{"Silicom Bypass PEG4BPFI6 series adapter"},
	{"Silicom Bypass PEG4BPFI6LX series adapter"},
	{"Silicom Bypass PEG4BPFI6ZX series adapter"},
	{"Silicom Bypass PEG4BPFI6CS series adapter"},
	{"Silicom Bypass PEG2BPI6 series adapter"},
	{"Silicom Bypass PEG2BPFI6 series adapter"},
	{"Silicom Bypass PEG2BPFI6LX series adapter"},
	{"Silicom Bypass PEG2BPFI6ZX series adapter"},

	{"Silicom Bypass PEG2BPFI6FLXM series adapter"},
	{"Silicom Bypass PEG2BPFI6FLXMRB2 series adapter"},


	{"Silicom Bypass PEG4BPI6FC series adapter"},
	{"Silicom Bypass PEG4BPFI6FC series adapter"},
	{"Silicom Bypass PEG4BPFI6FCLX series adapter"},
	{"Silicom Bypass PEG4BPFI6FCZX series adapter"},
	{"Silicom Bypass PEG6BPI6 series adapter"},
	{"Silicom Bypass PEG2BPI6SC6 series adapter"},
	{"Silicom Bypass MEG2BPI6 series adapter"},
	{"Silicom Bypass XEG2BPI6 series adapter"},
	{"Silicom Bypass MEG4BPI6 series adapter"},
	{"Silicom Bypass PEG2BPFI5-SD series adapter"},
	{"Silicom Bypass PEG2BPFI5LX-SD series adapter"},
	{"Silicom Bypass PXEG4BPFI-SD series adapter"},
	{"Silicom Bypass MxEG2BPI6 series adapter"},
	{"Silicom Bypass MxEG2BPFI6 series adapter"},
	{"Silicom Bypass MxEG2BPFI6LX series adapter"},
	{"Silicom Bypass MxEG2BPFI6ZX series adapter"},
	{"Silicom Bypass MxEG4BPI6 series adapter"},
	{"Silicom Bypass MxEG4BPFI6 series adapter"},
	{"Silicom Bypass MxEG4BPFI6LX series adapter"},
	{"Silicom Bypass MxEG4BPFI6ZX series adapter"},
	{"Silicom Bypass MxEG6BPI6 series adapter"},
	{"Silicom Bypass MxE2G4BPi80 series adapter"},
	{"Silicom Bypass MxE2G4BPFi80 series adapter"},
	{"Silicom Bypass MxE2G4BPFi80LX series adapter"},
	{"Silicom Bypass MxE2G4BPFi80ZX series adapter"},


	{"Silicom Bypass PE210G2SPI9 series adapter"},


	{"Silicom Bypass MxE210G2BPI9CX4 series adapter"},
	{"Silicom Bypass MxE210G2BPI9SR series adapter"},
	{"Silicom Bypass MxE210G2BPI9LR series adapter"},
	{"Silicom Bypass MxE210G2BPI9T series adapter"},

	{"Silicom Bypass PE210G2BPI9CX4 series adapter"},
	{"Silicom Bypass PE210G2BPI9SR series adapter"},
	{"Silicom Bypass PE210G2BPI9LR series adapter"},
	{"Silicom Bypass PE210G2BPI9SRD series adapter"},
	{"Silicom Bypass PE210G2BPI9LRD series adapter"},

	{"Silicom Bypass PE210G2BPI9T series adapter"},
	{"Silicom Bypass M1E210G2BPI9SRDJP series adapter"},
	{"Silicom Bypass M1E210G2BPI9SRDJP1 series adapter"},
	{"Silicom Bypass M1E210G2BPI9LRDJP series adapter"},
	{"Silicom Bypass M1E210G2BPI9LRDJP1 series adapter"},

	{"Silicom Bypass M2EG2BPFI6 series adapter"},
	{"Silicom Bypass M2EG2BPFI6LX series adapter"},
	{"Silicom Bypass M2EG2BPFI6ZX series adapter"},
	{"Silicom Bypass M2EG4BPI6 series adapter"},
	{"Silicom Bypass M2EG4BPFI6 series adapter"},
	{"Silicom Bypass M2EG4BPFI6LX series adapter"},
	{"Silicom Bypass M2EG4BPFI6ZX series adapter"},
	{"Silicom Bypass M2EG6BPI6 series adapter"},



	{"Silicom Bypass PEG2DBI6    series adapter"},
	{"Silicom Bypass PEG2DBFI6   series adapter"},
	{"Silicom Bypass PEG2DBFI6LX series adapter"},
	{"Silicom Bypass PEG2DBFI6ZX series adapter"},


	{"Silicom Bypass PE2G4BPi80 series adapter"}, 
	{"Silicom Bypass PE2G4BPFi80 series adapter"},
	{"Silicom Bypass PE2G4BPFi80LX series adapter"},
	{"Silicom Bypass PE2G4BPFi80ZX series adapter"},

	{"Silicom Bypass PE2G4BPi80L series adapter"},
	{"Silicom Bypass MxE2G8BPi80A series adapter"},




	{"Silicom Bypass PE2G2BPi35 series adapter"},
	{"Silicom Bypass PAC1200BPi35 series adapter"},
	{"Silicom Bypass PE2G2BPFi35 series adapter"},
	{"Silicom Bypass PE2G2BPFi35ARB2 series adapter"},
	{"Silicom Bypass PE2G2BPFi35LX series adapter"},
	{"Silicom Bypass PE2G2BPFi35ALXRB2 series adapter"},
	{"Silicom Bypass PE2G2BPFi35ZX series adapter"},



	{"Silicom Bypass PE2G4BPi35 series adapter"},
	{"Silicom Bypass PE2G4BPi35L series adapter"},
	{"Silicom Bypass PE2G4BPi35ALRB2 series adapter"},
	{"Silicom Bypass PE2G4BPFi35 series adapter"},
	{"Silicom Bypass PE2G4BPFi35ARB2 series adapter"},
	{"Silicom Bypass PE2G4BPFi35CS series adapter"},
	{"Silicom Bypass PE2G4BPFi35LX series adapter"},
	{"Silicom Bypass PE2G4BPFi35ALXRB2 series adapter"},
	{"Silicom Bypass PE2G4BPFi35ZX series adapter"},

	{"Silicom Bypass PE2G6BPi35 series adapter"},
	{"Silicom Bypass PE2G6BPi35CX series adapter"},


	{"Silicom Bypass PE2G2BPi80 series adapter"}, 
	{"Silicom Bypass PE2G2BPFi80 series adapter"},
	{"Silicom Bypass PE2G2BPFi80LX series adapter"},
	{"Silicom Bypass PE2G2BPFi80ZX series adapter"},


	{"Silicom Bypass M2E10G2BPI9CX4 series adapter"},
	{"Silicom Bypass M2E10G2BPI9SR series adapter"},
	{"Silicom Bypass M2E10G2BPI9LR series adapter"},
	{"Silicom Bypass M2E10G2BPI9T series adapter"},
	{"Silicom Bypass MxE2G8BPi80 series adapter"},
	{"Silicom Bypass PE210G2DBi9SR series adapter"},  
	{"Silicom Bypass PE210G2DBi9SRRB2 series adapter"},
	{"Silicom Bypass PE210G2DBi9LR series adapter"},  
	{"Silicom Bypass PE210G2DBi9LRRB2 series adapter"},
	{"Silicom Bypass PE310G4DBi9-SR series adapter"},
	{"Silicom Bypass PE310G4DBi9-SRRB2 series adapter"},
	{"Silicom Bypass PE310G4BPi9T series adapter"},
	{"Silicom Bypass PE310G4BPi9SR series adapter"},
	{"Silicom Bypass PE310G4BPi9LR series adapter"},
	{"Silicom Bypass PE310G4BPi9SRD series adapter"},
	{"Silicom Bypass PE310G4BPi9LRD series adapter"},

	{"Silicom Bypass PE210G2BPi40T series adapter"},
	{"Silicom Bypass PE310G4BPi40T series adapter"},
	{"Silicom Bypass M1E210G2BPI40T series adapter"},
	{"Silicom Bypass M6E310G4BPi9SR series adapter"},
	{"Silicom Bypass M6E310G4BPi9LR series adapter"},
	{"Silicom Bypass PE2G6BPI6CS series adapter"},
	{"Silicom Bypass PE2G6BPI6 series adapter"},

	{"Silicom Bypass M1E2G4BPi35 series adapter"},
	{"Silicom Bypass M1E2G4BPFi35 series adapter"},
	{"Silicom Bypass M1E2G4BPFi35LX series adapter"},
	{"Silicom Bypass M1E2G4BPFi35ZX series adapter"},
	{"Silicom Bypass M1E2G4BPi35JP series adapter"},
	{"Silicom Bypass M1E2G4BPi35JP1 series adapter"},

	{"Silicom Bypass PE310G4DBi9T series adapter"},

	{"Silicom Bypass PE310G4BPI71 series adapter"},
	{"Silicom Bypass PE340G2BPI71 series adapter"},
	{"Silicom Bypass PE310G4BPI71SRD series adapter"},
	{"Silicom Bypass PE310G4BPI71LRD series adapter"},
	{"Silicom Bypass PE310G2BPI71SRD series adapter"},
	{"Silicom Bypass PE310G2BPI71LRD series adapter"},
	{"Silicom Bypass PE340G2BPI71QS4 series adapter"},
	{"Silicom Bypass PE340G2BPI71QL4 series adapter"},

	{0},
};

static bpmod_info_t tx_ctl_pci_tbl[] = {
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PXG2BPFI_SSID, PXG2BPFI, "PXG2BPFI-SD"},
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PXG2BPFIL_SSID, PXG2BPFIL, "PXG2BPFIL-SD"},
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PXG2BPFILX_SSID, PXG2BPFILX, "PXG2BPFILX-SD"},
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PXG2BPFILLX_SSID, PXG2BPFILLX, "PXG2BPFILLXSD"},
	{0x8086, 0x1010, SILICOM_SVID, SILICOM_PXGBPI_SSID, PXGBPI, "PXG2BPI-SD"},
	{0x8086, 0x1079, SILICOM_SVID, SILICOM_PXGBPIG_SSID, PXGBPIG, "PXG2BPIG-SD"},
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PXG2TBFI_SSID, PXG2TBFI, "PXG2TBFI-SD"},
	{0x8086, 0x1079, SILICOM_SVID, SILICOM_PXG4BPI_SSID, PXG4BPI, "PXG4BPI-SD"},
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PXG4BPFI_SSID, PXG4BPFI, "PXG4BPFI-SD"},
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PXG4BPFILX_SSID, PXG4BPFILX, "PXG4BPFILX-SD"},
	{0x8086, 0x1079, SILICOM_SVID, SILICOM_PEG4BPI_SSID, PEG4BPI, "PEXG4BPI-SD"},
	{0x8086, 0x105e, SILICOM_SVID, SILICOM_PEG2BPI_SSID, PEG2BPI, "PEG2BPI-SD"},
	{0x8086, 0x105e, SILICOM_SVID, SILICOM_PEG4BPIN_SSID, PEG4BPIN, "PEG4BPI-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_PEG2BPFI_SSID, PEG2BPFI, "PEG2BPFI-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_PEG2BPFILX_SSID, PEG2BPFILX, "PEG2BPFILX-SD"},
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PMCXG2BPFI_SSID, PMCXG2BPFI, "PMCX2BPFI-SD"},    
	{0x8086, 0x107a, NOKIA_PMCXG2BPFIN_SVID, NOKIA_PMCXG2BPFIN_SSID, PMCXG2BPFIN, "PMCX2BPFI-N"},    
	{0x8086, INTEL_PEG4BPII_PID,  0x8086, INTEL_PEG4BPII_SSID, PEG4BPII, "PEG4BPII"},
	{0x8086, INTEL_PEG4BPIIO_PID,  0x8086, INTEL_PEG4BPIIO_SSID, PEG4BPIIO, "PEG4BPII0"},
	{0x8086, INTEL_PEG4BPFII_PID, 0x8086, INTEL_PEG4BPFII_SSID, PEG4BPFII, "PEG4BPFII"},    
	{0x8086, 0x1079, NOKIA_PMCXG2BPFIN_SVID, NOKIA_PMCXG2BPIN_SSID, PMCXG2BPIN, "PMCX2BPI-N"},    
	{0x8086, 0x1079, NOKIA_PMCXG2BPFIN_SVID, NOKIA_PMCXG4BPIN_SSID, PMCXG4BPIN, "PMCX4BPI-N"},    
	{0x8086, 0x1079, SILICOM_SVID,SILICOM_PXG2BISC1_SSID, PXG2BISC1, "PXG2BISC1-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_PEG2TBFI_SSID, PEG2TBFI, "PEG2TBFI-SD"},
	{0x8086, 0x1079, SILICOM_SVID,SILICOM_PXG2TBI_SSID, PXG2TBI, "PXG2TBI-SD"},
	{0x8086, 0x107a, SILICOM_SVID, SILICOM_PXG4BPFID_SSID, PXG4BPFID, "PXG4BPFID-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_PEG4BPFI_SSID, PEG4BPFI, "PEG4BPFI-SD"},
	{0x8086, 0x105e, SILICOM_SVID, SILICOM_PEG4BPIPT_SSID, PEG4BPIPT, "PEG4BPIPT-SD"},   
	{0x8086, 0x1079, SILICOM_SVID, SILICOM_PXG6BPI_SSID, PXG6BPI, "PXG6BPI-SD"},
	{0x8086, 0x10a7, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPIL_SSID	/*PCI_ANY_ID*/, PEG4BPIL, "PEG4BPIL-SD"},
	{0x8086, 0x1079, NOKIA_PMCXG2BPFIN_SVID, NOKIA_PMCXG2BPIN2_SSID, PMCXG2BPIN2, "PMCX2BPI-N2"},    
	{0x8086, 0x1079, NOKIA_PMCXG2BPFIN_SVID, NOKIA_PMCXG4BPIN2_SSID, PMCXG4BPIN2, "PMCX4BPI-N2"},    
	{0x8086, 0x1079, SILICOM_SVID, SILICOM_PMCX2BPI_SSID, PMCX2BPI, "PMCX2BPI-SD"},
	{0x8086, 0x1079, SILICOM_SVID, SILICOM_PMCX4BPI_SSID, PMCX4BPI, "PMCX4BPI-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_PEG2BPFID_SSID, PEG2BPFID, "PEG2BPFID-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_PEG2BPFIDLX_SSID, PEG2BPFIDLX, "PEG2BPFIDLXSD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_MEG2BPFILN_SSID, MEG2BPFILN, "MEG2BPFILN-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_MEG2BPFINX_SSID, MEG2BPFINX, "MEG2BPFINX-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_PEG4BPFILX_SSID, PEG4BPFILX, "PEG4BPFILX-SD"},
	{0x8086, 0x10C6, SILICOM_SVID, SILICOM_PE10G2BPISR_SSID, PE10G2BPISR, "PE10G2BPISR"},
	{0x8086, 0x10C6, SILICOM_SVID, SILICOM_PE10G2BPILR_SSID, PE10G2BPILR, "PE10G2BPILR"},
	{0x8086, 0x10a9, SILICOM_SVID , SILICOM_MHIO8AD_SSID , MHIO8AD, "MHIO8AD-SD"},
	{0x8086, 0x10DD, SILICOM_SVID, SILICOM_PE10G2BPICX4_SSID, PE10G2BPISR, "PE10G2BPICX4"},
	{0x8086, 0x10a7, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2BPI5_SSID	/*PCI_ANY_ID*/, PEG2BPI5, "PEG2BPI5-SD"},
	{0x8086, 0x10a7, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG6BPI_SSID /*PCI_ANY_ID*/, PEG6BPI, "PEG6BPI5"},
	{0x8086, 0x10a9, SILICOM_SVID /*PCI_ANY_ID*/,SILICOM_PEG4BPFI5_SSID, PEG4BPFI5, "PEG4BPFI5"},
	{0x8086, 0x10a9, SILICOM_SVID /*PCI_ANY_ID*/,SILICOM_PEG4BPFI5LX_SSID, PEG4BPFI5LX, "PEG4BPFI5LX"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_MEG2BPFILXLN_SSID, MEG2BPFILXLN, "MEG2BPFILXLN"},
	{0x8086, 0x105e, SILICOM_SVID, SILICOM_PEG2BPIX1_SSID, PEG2BPIX1, "PEG2BPIX1-SD"},
	{0x8086, 0x105f, SILICOM_SVID, SILICOM_MEG2BPFILXNX_SSID, MEG2BPFILXNX, "MEG2BPFILXNX"},
	{0x8086, 0x10C8, SILICOM_SVID, SILICOM_XE10G2BPIT_SSID, XE10G2BPIT, "XE10G2BPIT"},
	{0x8086, 0x10DD, SILICOM_SVID, SILICOM_XE10G2BPICX4_SSID, XE10G2BPICX4, "XE10G2BPICX4"},
	{0x8086, 0x10C6, SILICOM_SVID, SILICOM_XE10G2BPISR_SSID, XE10G2BPISR, "XE10G2BPISR"},
	{0x8086, 0x10C6, SILICOM_SVID, SILICOM_XE10G2BPILR_SSID, XE10G2BPILR, "XE10G2BPILR"},
	{0x8086, 0x10C6, NOKIA_XE10G2BPIXR_SVID, NOKIA_XE10G2BPIXR_SSID, XE10G2BPIXR, "XE10G2BPIXR"},
	{0x8086, 0x10C6, SILICOM_SVID,SILICOM_PE10GDBISR_SSID, PE10GDBISR, "PE10G2DBISR"},
	{0x8086, 0x10C6, SILICOM_SVID,SILICOM_PE10GDBILR_SSID, PE10GDBILR, "PE10G2DBILR"},
	{0x8086, 0x10a7, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2BISC6_SSID /*PCI_ANY_ID*/, PEG2BISC6, "PEG2BI5SC6"},
	{0x8086, 0x10a7, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG6BPIFC_SSID /*PCI_ANY_ID*/, PEG6BPIFC, "PEG6BPI5FC"},

	{BROADCOM_VID, BROADCOM_PE10G2_PID, SILICOM_SVID, SILICOM_PE10G2BPTCX4_SSID, PE10G2BPTCX4, "PE10G2BPTCX4"},
	{BROADCOM_VID, BROADCOM_PE10G2_PID, SILICOM_SVID, SILICOM_PE10G2BPTSR_SSID, PE10G2BPTSR, "PE10G2BPTSR"},
	{BROADCOM_VID, BROADCOM_PE10G2_PID, SILICOM_SVID, SILICOM_PE10G2BPTLR_SSID, PE10G2BPTLR, "PE10G2BPTLR"},
	{BROADCOM_VID, BROADCOM_PE10G2_PID, SILICOM_SVID, SILICOM_PE10G2BPTT_SSID, PE10G2BPTT, "PE10G2BPTT"},

	//{BROADCOM_VID, BROADCOM_PE10G2_PID, PCI_ANY_ID, PCI_ANY_ID, PE10G2BPTCX4, "PE10G2BPTCX4"},

	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPI6_SSID	/*PCI_ANY_ID*/, PEG4BPI6, "PEG4BPI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPFI6_SSID /*PCI_ANY_ID*/, PEG4BPFI6, "PEG4BPFI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPFI6LX_SSID /*PCI_ANY_ID*/, PEG4BPFI6LX, "PEG4BPFI6LX"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPFI6ZX_SSID /*PCI_ANY_ID*/, PEG4BPFI6ZX, "PEG4BPFI6ZX"},
	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2BPI6_SSID	/*PCI_ANY_ID*/, PEG2BPI6, "PEG2BPI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2BPFI6_SSID /*PCI_ANY_ID*/, PEG2BPFI6, "PEG2BPFI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2BPFI6LX_SSID /*PCI_ANY_ID*/, PEG2BPFI6LX, "PEG2BPFI6LX"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2BPFI6ZX_SSID /*PCI_ANY_ID*/, PEG2BPFI6ZX, "PEG2BPFI6ZX"},

	{0x8086, 0x10e7, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2BPFI6FLXM_SSID /*PCI_ANY_ID*/, PEG2BPFI6FLXM, "PEG2BPFI6FLXM"},
	{0x8086, 0x10e7, 0x1b2e	/*PCI_ANY_ID*/, SILICOM_PEG2BPFI6FLXM_SSID /*PCI_ANY_ID*/, PEG2BPFI6FLXMRB2, "PEG2BPFI6FLXMRB2"},


	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPI6FC_SSID /*PCI_ANY_ID*/, PEG4BPI6FC, "PEG4BPI6FC"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPFI6FC_SSID /*PCI_ANY_ID*/, PEG4BPFI6FC, "PEG4BPFI6FC"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPFI6FCLX_SSID /*PCI_ANY_ID*/, PEG4BPFI6FCLX, "PEG4BPFI6FCLX"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG4BPFI6FCZX_SSID /*PCI_ANY_ID*/, PEG4BPFI6FCZX, "PEG4BPFI6FCZX"},
	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG6BPI6_SSID	/*PCI_ANY_ID*/, PEG6BPI6, "PEG6BPI6"},
	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2BPI6SC6_SSID /*PCI_ANY_ID*/, PEG2BPI6SC6, "PEG6BPI62SC6"},
	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_MEG2BPI6_SSID	/*PCI_ANY_ID*/, MEG2BPI6, "MEG2BPI6"},
	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_XEG2BPI6_SSID	/*PCI_ANY_ID*/, XEG2BPI6, "XEG2BPI6"},
	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_MEG4BPI6_SSID	/*PCI_ANY_ID*/, MEG4BPI6, "MEG4BPI6"},

	{0x8086, 0x10a9, SILICOM_SVID /*PCI_ANY_ID*/,SILICOM_PEG2BPFI5_SSID, PEG2BPFI5, "PEG2BPFI5"},
	{0x8086, 0x10a9, SILICOM_SVID /*PCI_ANY_ID*/,SILICOM_PEG2BPFI5LX_SSID, PEG2BPFI5LX, "PEG2BPFI5LX"},

	{0x8086, 0x105f, SILICOM_SVID, SILICOM_PXEG4BPFI_SSID, PXEG4BPFI, "PXEG4BPFI-SD"},

	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG2BPI6_SSID /*PCI_ANY_ID*/, M1EG2BPI6, "MxEG2BPI6"},

	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG2BPFI6_SSID /*PCI_ANY_ID*/, M1EG2BPFI6, "MxEG2BPFI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG2BPFI6LX_SSID	/*PCI_ANY_ID*/, M1EG2BPFI6LX, "MxEG2BPFI6LX"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG2BPFI6ZX_SSID	/*PCI_ANY_ID*/, M1EG2BPFI6ZX, "MxEG2BPFI6ZX"},

	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG4BPI6_SSID /*PCI_ANY_ID*/, M1EG4BPI6, "MxEG4BPI6"},

	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG4BPFI6_SSID /*PCI_ANY_ID*/, M1EG4BPFI6, "MxEG4BPFI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG4BPFI6LX_SSID	/*PCI_ANY_ID*/, M1EG4BPFI6LX, "MxEG4BPFI6LX"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG4BPFI6ZX_SSID	/*PCI_ANY_ID*/, M1EG4BPFI6ZX, "MxEG4BPFI6ZX"},

	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1EG6BPI6_SSID /*PCI_ANY_ID*/, M1EG6BPI6, "MxEG6BPI6"},


	{0x8086, 0x150e, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E2G4BPi80_SSID /*PCI_ANY_ID*/, M1E2G4BPi80, "MxE2G4BPi80"},
	{0x8086, 0x150f, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E2G4BPFi80_SSID	/*PCI_ANY_ID*/, M1E2G4BPFi80, "MxE2G4BPFi80"},
	{0x8086, 0x150f, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E2G4BPFi80LX_SSID /*PCI_ANY_ID*/, M1E2G4BPFi80LX, "MxE2G4BPFi80LX"},
	{0x8086, 0x150f, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E2G4BPFi80ZX_SSID /*PCI_ANY_ID*/, M1E2G4BPFi80ZX, "MxE2G4BPFi80ZX"},






	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2EG2BPFI6_SSID /*PCI_ANY_ID*/, M2EG2BPFI6, "M2EG2BPFI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2EG2BPFI6LX_SSID	/*PCI_ANY_ID*/, M2EG2BPFI6LX, "M2EG2BPFI6LX"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2EG2BPFI6ZX_SSID	/*PCI_ANY_ID*/, M2EG2BPFI6ZX, "M2EG2BPFI6ZX"},

	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2EG4BPI6_SSID /*PCI_ANY_ID*/, M2EG4BPI6, "M2EG4BPI6"},

	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2EG4BPFI6_SSID /*PCI_ANY_ID*/, M2EG4BPFI6, "M2EG4BPFI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2EG4BPFI6LX_SSID	/*PCI_ANY_ID*/, M2EG4BPFI6LX, "M2EG4BPFI6LX"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2EG4BPFI6ZX_SSID	/*PCI_ANY_ID*/, M2EG4BPFI6ZX, "M2EG4BPFI6ZX"},

	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2EG6BPI6_SSID /*PCI_ANY_ID*/, M2EG6BPI6, "M2EG6BPI6"},


	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2DBI6_SSID	/*PCI_ANY_ID*/, PEG2DBI6, "PEG2DBI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2DBFI6_SSID /*PCI_ANY_ID*/, PEG2DBFI6, "PEG2DBFI6"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2DBFI6LX_SSID /*PCI_ANY_ID*/, PEG2DBFI6LX, "PEG2DBFI6LX"},
	{0x8086, 0x10e6, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PEG2DBFI6ZX_SSID /*PCI_ANY_ID*/, PEG2DBFI6ZX, "PEG2DBFI6ZX"},

	{0x8086, 0x10F9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE210G2DBi9SR_SSID,    PE210G2DBi9SR,   "PE210G2DBi9SR"},



	{0x8086, 0x10F9, 0X1B2E	/*PCI_ANY_ID*/, SILICOM_PE210G2DBi9SRRB_SSID , PE210G2DBi9SRRB2, "PE210G2DBi9SRRB2"}, 
	{0x8086, 0x10F9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE210G2DBi9LR_SSID  ,  PE210G2DBi9LR,   "PE210G2DBi9LR"},  
	{0x8086, 0x10F9, 0X1B2E	/*PCI_ANY_ID*/, SILICOM_PE210G2DBi9LRRB_SSID , PE210G2DBi9LRRB2, "PE210G2DBi9LRRB2"},

	{0x8086, 0x10F9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4DBi940SR_SSID , PE310G4DBi940SR, "PE310G4DBi9SR"},
	{0x8086, 0x10F9, 0X1B2E	/*PCI_ANY_ID*/, SILICOM_PE310G4DBi940SR_SSID , PE310G4DBi940SRRB2, "PE310G4DBi9SRRB2"},

	{0x8086, 0x10F9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4DBi940LR_SSID , PE310G4DBi940LR, "PE310G4DBi9LR"},
	{0x8086, 0x10F9, 0X1B2E	/*PCI_ANY_ID*/, SILICOM_PE310G4DBi940LR_SSID , PE310G4DBi940LRRB2, "PE310G4DBi9LRRB2"},

	{0x8086, 0x10F9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4DBi940T_SSID , PE310G4DBi940T, "PE310G4DBi9T"},
	{0x8086, 0x10F9, 0X1B2E	/*PCI_ANY_ID*/, SILICOM_PE310G4DBi940T_SSID , PE310G4DBi940TRB2, "PE310G4DBi9TRB2"},





	{0x8086, 0x10F9, SILICOM_SVID /*PCI_ANY_ID*/,SILICOM_PE310G4DBi9T_SSID , PE310G4DBi9T, "PE310G4DBi9T"},



	{0x8086, 0x10Fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4BPi9T_SSID,  PE310G4BPi9T, "PE310G4BPi9T"},
	{0x8086, 0x10Fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4BPi9SR_SSID, PE310G4BPi9SR, "PE310G4BPi9SR"},
	{0x8086, 0x10Fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4BPi9LR_SSID, PE310G4BPi9LR, "PE310G4BPi9LR"},
	{0x8086, 0x10Fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4BPi9SRD_SSID, PE310G4BPi9SRD, "PE310G4BPi9SRD"},
	{0x8086, 0x10Fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4BPi9LRD_SSID, PE310G4BPi9LRD, "PE310G4BPi9LRD"},

	{0x8086, 0x10Fb, SILICOM_SVID,  SILICOM_M6E310G4BPi9LR_SSID, M6E310G4BPi9LR, "M6E310G4BPi9LR"},
	{0x8086, 0x10Fb, SILICOM_SVID,  SILICOM_M6E310G4BPi9SR_SSID, M6E310G4BPi9SR, "M6E310G4BPi9SR"},





	{0x8086, 0x150e, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPi80_SSID /*PCI_ANY_ID*/,    PE2G4BPi80, "PE2G4BPi80"},
	{0x8086, 0x150f, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPFi80_SSID /*PCI_ANY_ID*/,   PE2G4BPFi80, "PE2G4BPFi80"},
	{0x8086, 0x150f, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPFi80LX_SSID /*PCI_ANY_ID*/, PE2G4BPFi80LX, "PE2G4BPFi80LX"},
	{0x8086, 0x150f, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPFi80ZX_SSID /*PCI_ANY_ID*/, PE2G4BPFi80ZX, "PE2G4BPFi80ZX"},

	{0x8086, 0x150e, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPi80L_SSID /*PCI_ANY_ID*/,    PE2G4BPi80L, "PE2G4BPi80L"},

	{0x8086, 0x150e, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M6E2G8BPi80A_SSID	/*PCI_ANY_ID*/,    M6E2G8BPi80A, "MxE2G8BPi80A"},



	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G2BPi35_SSID /*PCI_ANY_ID*/,    PE2G2BPi35, "PE2G2BPi35"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PAC1200BPi35_SSID	/*PCI_ANY_ID*/,    PAC1200BPi35, "PAC1200BPi35"},

	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G2BPFi35_SSID /*PCI_ANY_ID*/,    PE2G2BPFi35, "PE2G2BPFi35"},
	{0x8086, 0x1522, 0x1B2E	/*PCI_ANY_ID*/, SILICOM_PE2G2BPFi35_SSID /*PCI_ANY_ID*/,    PE2G2BPFi35ARB2, "PE2G2BPFi35ARB2"},

	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G2BPFi35LX_SSID /*PCI_ANY_ID*/,    PE2G2BPFi35LX, "PE2G2BPFi35LX"},
	{0x8086, 0x1522, 0x1B2E	/*PCI_ANY_ID*/, SILICOM_PE2G2BPFi35LX_SSID /*PCI_ANY_ID*/,    PE2G2BPFi35ALXRB2, "PE2G2BPFi35ALXRB2"},


	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G2BPFi35ZX_SSID /*PCI_ANY_ID*/,    PE2G2BPFi35ZX, "PE2G2BPFi35ZX"},

	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPi35_SSID /*PCI_ANY_ID*/,    PE2G4BPi35, "PE2G4BPi35"},


	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPi35L_SSID /*PCI_ANY_ID*/,    PE2G4BPi35L, "PE2G4BPi35L"},
	{0x8086, 0x1521, 0x1B2E	/*PCI_ANY_ID*/, SILICOM_PE2G4BPi35L_SSID /*PCI_ANY_ID*/,    PE2G4BPi35ALRB2, "PE2G4BPi35ALRB2"},


	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPFi35_SSID /*PCI_ANY_ID*/,    PE2G4BPFi35, "PE2G4BPFi35"},
	{0x8086, 0x1522, 0x1B2E	/*PCI_ANY_ID*/, SILICOM_PE2G4BPFi35_SSID /*PCI_ANY_ID*/,    PE2G4BPFi35ARB2, "PE2G4BPFi35ARB2"},




	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPFi35CS_SSID /*PCI_ANY_ID*/,    PE2G4BPFi35CS, "PE2G4BPFi35CS"},

	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPFi35LX_SSID /*PCI_ANY_ID*/,    PE2G4BPFi35LX, "PE2G4BPFi35LX"},
	{0x8086, 0x1522, 0x1B2E	/*PCI_ANY_ID*/, SILICOM_PE2G4BPFi35LX_SSID /*PCI_ANY_ID*/,    PE2G4BPFi35ALXRB2, "PE2G4BPFi35ALXRB2"},


	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G4BPFi35ZX_SSID /*PCI_ANY_ID*/,    PE2G4BPFi35ZX, "PE2G4BPFi35ZX"},


	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E2G4BPi35_SSID /*PCI_ANY_ID*/,    M1E2G4BPi35, "M1E2G4BPi35"},
	{0x8086, 0x1521, 0x1304	/*PCI_ANY_ID*/, SILICOM_M1E2G4BPi35JP_SSID /*PCI_ANY_ID*/,    M1E2G4BPi35JP, "M1E2G4BPi35JP"},
	{0x8086, 0x1521, 0x1304	/*PCI_ANY_ID*/, SILICOM_M1E2G4BPi35JP1_SSID	/*PCI_ANY_ID*/,    M1E2G4BPi35JP1, "M1E2G4BPi35JP1"},


	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E2G4BPFi35_SSID	/*PCI_ANY_ID*/,    M1E2G4BPFi35, "M1E2G4BPFi35"},
	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E2G4BPFi35LX_SSID /*PCI_ANY_ID*/,    M1E2G4BPFi35LX, "M1E2G4BPFi35LX"},
	{0x8086, 0x1522, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E2G4BPFi35ZX_SSID /*PCI_ANY_ID*/,    M1E2G4BPFi35ZX, "M1E2G4BPFi35ZX"},


	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G6BPi35_SSID /*PCI_ANY_ID*/,    PE2G6BPi35, "PE2G6BPi35"},

	// {0x8086, PCI_ANY_ID, SILICOM_SVID /*PCI_ANY_ID*/,0xaa0,PE2G6BPi35CX,"PE2G6BPi35CX"},
	// {0x8086, PCI_ANY_ID, SILICOM_SVID /*PCI_ANY_ID*/,0xaa1,PE2G6BPi35CX,"PE2G6BPi35CX"},
	// {0x8086, PCI_ANY_ID, SILICOM_SVID /*PCI_ANY_ID*/,0xaa2,PE2G6BPi35CX,"PE2G6BPi35CX"},

	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B40,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B41,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B42,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B43,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B44,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B45,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B46,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B47,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B48,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B49,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B4a,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B4b,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B4c,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B4d,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B4e,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B4F,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B50,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B51,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B52,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B53,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B54,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B55,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B56,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B57,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B58,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B59,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B5A,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B5B,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B5C,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B5D,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B5E,PE2G6BPI6CS,"PE2G6BPI6CS"},
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,0x0B5F,PE2G6BPI6CS,"PE2G6BPI6CS"},

	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B60,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B61,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B62,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B63,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B64,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B65,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B66,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B67,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B68,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B69,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B6a,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B6b,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B6c,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B6d,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B6e,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B6f,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B71,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B72,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B73,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B74,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B75,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B76,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B77,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B78,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B79,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B7a,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B7b,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B7c,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B7d,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B7e,PEG4BPFI6CS,"PEG4BPFI6CS"},
	{0x8086, 0x10E6, SILICOM_SVID /*PCI_ANY_ID*/,0x0B7f,PEG4BPFI6CS,"PEG4BPFI6CS"},



	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/,SILICOM_PE2G6BPI6_SSID,PE2G6BPI6,"PE2G6BPI6"},


	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa0,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa1,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa2,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa3,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa4,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa5,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa6,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa7,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa8,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaa9,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaaa,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaab,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaac,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaad,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaae,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaaf,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab0,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab1,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab2,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab3,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab4,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab5,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab6,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab7,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab8,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xab9,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xaba,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xabb,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xabc,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xabd,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xabe,PE2G6BPi35CX,"PE2G6BPi35CX"},
	{0x8086, 0x1521, SILICOM_SVID /*PCI_ANY_ID*/,0xabf,PE2G6BPi35CX,"PE2G6BPi35CX"},

	{0x8086, PCI_ANY_ID, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G2BPi80_SSID /*PCI_ANY_ID*/,    PE2G2BPi80, "PE2G2BPi80"},
	{0x8086, PCI_ANY_ID, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G2BPFi80_SSID /*PCI_ANY_ID*/,   PE2G2BPFi80, "PE2G2BPFi80"},
	{0x8086, PCI_ANY_ID, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G2BPFi80LX_SSID /*PCI_ANY_ID*/, PE2G2BPFi80LX, "PE2G2BPFi80LX"},
	{0x8086, PCI_ANY_ID, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE2G2BPFi80ZX_SSID /*PCI_ANY_ID*/, PE2G2BPFi80ZX, "PE2G2BPFi80ZX"},

	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_MEG2BPI6_SSID	/*PCI_ANY_ID*/, MEG2BPI6, "MEG2BPI6"},
	{0x8086, 0x10c9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_XEG2BPI6_SSID	/*PCI_ANY_ID*/, XEG2BPI6, "XEG2BPI6"},



#if 0
	{0x8086, 0x10fb, 0x8086, INTEL_PE210G2SPI9_SSID, PE210G2SPI9, "PE210G2SPI9"},
#endif
	{0x8086, 0x10fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E10G2BPI9CX4_SSID /*PCI_ANY_ID*/, M1E10G2BPI9CX4, "MxE210G2BPI9CX4"},
	{0x8086, 0x10fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E10G2BPI9SR_SSID /*PCI_ANY_ID*/, M1E10G2BPI9SR, "MxE210G2BPI9SR"},
	{0x8086, 0x10fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E10G2BPI9LR_SSID /*PCI_ANY_ID*/, M1E10G2BPI9LR, "MxE210G2BPI9LR"},

	{0x8086, 0x10fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2E10G2BPI9CX4_SSID /*PCI_ANY_ID*/, M2E10G2BPI9CX4, "M2E10G2BPI9CX4"},
	{0x8086, 0x10fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2E10G2BPI9SR_SSID /*PCI_ANY_ID*/, M2E10G2BPI9SR, "M2E10G2BPI9SR"},
	{0x8086, 0x10fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2E10G2BPI9LR_SSID /*PCI_ANY_ID*/, M2E10G2BPI9LR, "M2E10G2BPI9LR"},
	{0x8086, 0x10fb, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M2E10G2BPI9T_SSID	/*PCI_ANY_ID*/, M2E10G2BPI9T, "M2E10G2BPI9T"},



	{0x8086, 0x10fb, SILICOM_SVID, SILICOM_PE210G2BPI9CX4_SSID, PE210G2BPI9CX4, "PE210G2BPI9CX4"},
	{0x8086, 0x10fb, SILICOM_SVID, SILICOM_PE210G2BPI9SR_SSID,  PE210G2BPI9SR, "PE210G2BPI9SR"},
	{0x8086, 0x10fb, SILICOM_SVID, SILICOM_PE210G2BPI9LR_SSID,  PE210G2BPI9LR, "PE210G2BPI9LR"},
	{0x8086, 0x10fb, SILICOM_SVID, SILICOM_PE210G2BPI9SRD_SSID,  PE210G2BPI9SRD, "PE210G2BPI9SRD"},
	{0x8086, 0x10fb, SILICOM_SVID, SILICOM_PE210G2BPI9LRD_SSID,  PE210G2BPI9LRD, "PE210G2BPI9LRD"},
	{0x8086, 0x10fb, SILICOM_SVID, SILICOM_PE210G2BPI9T_SSID,   PE210G2BPI9T, "PE210G2BPI9T"},

	{0x8086, 0x10fb, SILICOM_SVID, SILICOM_PE210G2BPI9T_SSID,   PE210G2BPI9T, "PE210G2BPI9T"},
	{0x8086, 0x10fb, 0x1304, SILICOM_M1E210G2BPI9SRDJP_SSID,   M1E210G2BPI9SRDJP, "M1E210G2BPI9SRDJP"},
	{0x8086, 0x10fb, 0x1304, SILICOM_M1E210G2BPI9SRDJP1_SSID,   M1E210G2BPI9SRDJP1, "M1E210G2BPI9SRDJP1"},
	{0x8086, 0x10fb, 0x1304, SILICOM_M1E210G2BPI9LRDJP_SSID,   M1E210G2BPI9LRDJP, "M1E210G2BPI9LRDJP"},
	{0x8086, 0x10fb, 0x1304, SILICOM_M1E210G2BPI9LRDJP1_SSID,   M1E210G2BPI9LRDJP1, "M1E210G2BPI9LRDJP1"},

#if 0
	{0x1374, 0x2c, SILICOM_SVID, SILICOM_PXG4BPI_SSID, PXG4BPI, "PXG4BPI-SD"},

	{0x1374, 0x2d, SILICOM_SVID, SILICOM_PXG4BPFI_SSID, PXG4BPFI, "PXG4BPFI-SD"},


	{0x1374, 0x3f, SILICOM_SVID,SILICOM_PXG2TBI_SSID, PXG2TBI, "PXG2TBI-SD"},

	{0x1374, 0x3d, SILICOM_SVID,SILICOM_PXG2BISC1_SSID, PXG2BISC1, "PXG2BISC1-SD"},


	{0x1374, 0x40, SILICOM_SVID, SILICOM_PEG4BPFI_SSID, PEG4BPFI, "PEG4BPFI-SD"},



#ifdef BP_SELF_TEST
	{0x1374, 0x28, SILICOM_SVID,0x28,  PXGBPI, "PXG2BPI-SD"},
#endif
#endif
	{0x8086, 0x10C9, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M6E2G8BPi80_SSID /*PCI_ANY_ID*/,    M6E2G8BPi80, "MxE2G8BPi80"},
	{0x8086, 0x1528, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE210G2BPi40_SSID	/*PCI_ANY_ID*/,    PE210G2BPi40, "PE210G2BPi40T"},
	{0x8086, 0x1528, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_PE310G4BPi40_SSID	/*PCI_ANY_ID*/,    PE310G4BPi40, "PE310G4BPi40T"},

	{0x8086, 0x1528, SILICOM_SVID /*PCI_ANY_ID*/, SILICOM_M1E210G2BPI40T_SSID /*PCI_ANY_ID*/,    M1E210G2BPI40T, "M1E210G2BPi40T"},

	{0x8086, 0x1572, SILICOM_SVID, SILICOM_PE310G4BPI71SRD_SSID, PE310G4BPI71SRD, "PE310G4BPI71SRD"},
	{0x8086, 0x1572, SILICOM_SVID, SILICOM_PE310G4BPI71LRD_SSID, PE310G4BPI71LRD, "PE310G4BPI71LRD"},

	{0x8086, 0x1572, SILICOM_SVID, SILICOM_PE310G2BPI71SRD_SSID, PE310G2BPI71SRD, "PE310G2BPI71SRD"},
	{0x8086, 0x1572, SILICOM_SVID, SILICOM_PE310G2BPI71LRD_SSID, PE310G2BPI71LRD, "PE310G2BPI71LRD"},

	{0x8086, 0x1572, SILICOM_SVID, 0x0, PE310G4BPI71, "PE310G4BPI71"},

	{0x8086, 0x1583, SILICOM_SVID, SILICOM_PE340G2BPI71QS4_SSID, PE340G2BPI71QS4, "PE340G2BPI71QS4"},
	{0x8086, 0x1583, SILICOM_SVID, SILICOM_PE340G2BPI71QL4_SSID, PE340G2BPI71QL4, "PE340G2BPI71QL4"},

	{0x8086, 0x1583, SILICOM_SVID, 0x0, PE340G2BPI71, "PE340G2BPI71"},


	/* required last entry */
	{0,}
}; 

/*                                          
* Initialize the module - Register the character device
*/

static int bpmod_init(void)
{   
	printf(BP_MOD_DESCR" v"BP_MOD_VER"\n");

	mtx_init(&mtx, "bpmod", MTX_NETWORK_LOCK, MTX_DEF);

	device_num=bpmod_find_devices();

	if (!device_num)
	{
		printf("No such device\n"); 
		return ENODEV;
	}

	bpctl_dev_arr=malloc ((device_num)  * sizeof (bpctl_dev_t), M_DEVBUF, M_NOWAIT|M_ZERO);

	if (!bpctl_dev_arr)
	{
		printf("Allocation error\n"); 
		return ENOMEM;
	}

	return(bpmod_alloc_devices());

}
/*
* Cleanup - unregister the appropriate file from /proc
*/
static void bpmod_clean(void)
{
	int i ;        

	/* unmap all devices */
	for (i = 0; i < device_num; i++)
	{
		remove_bypass_wd_auto(&bpctl_dev_arr[i]);
		bpctl_dev_arr[i].reset_time=0;
#ifdef aaa
		iounmap ((void *)(bpctl_dev_arr[i].mem_map));
#endif
		remove_bypass_tpl_auto(&bpctl_dev_arr[i]);
		if (bpctl_dev_arr[i].ndev) {
			if_rele(bpctl_dev_arr[i].ndev);
		}

	}

	/* free all devices space */
	if (bpctl_dev_arr)

		free (bpctl_dev_arr, M_DEVBUF);
	mtx_destroy(&mtx); 
}

static int
bpmod_load(module_t mod, int cmd, void *arg)
{
	int  err = 0;

	switch (cmd)
	{
	case MOD_LOAD:
		err = bpmod_init();
		bpmod_dev = make_dev(&bpmod_devsw, 0, UID_ROOT, GID_WHEEL, 0644, "bpmod");
		break;      

	case MOD_UNLOAD:
		bpmod_clean();
		destroy_dev(bpmod_dev);
		break;		/* Success*/

	default:	/* we only understand load/unload*/
		err = EINVAL;
		break;
	}

	return(err);
}

int
bpmod_find_devices(void){
	devclass_t      pci_devclass;
	device_t        *pci_devices;
	int         pci_count = 0;
	device_t        *pci_children;
	int         pci_childcount = 0;
	device_t        *busp, *childp;
	int         i, j, idx,idx_dev=0;

	if ((pci_devclass = devclass_find("pci")) == NULL)
		return 0;

	devclass_get_devices(pci_devclass, &pci_devices, &pci_count);

	for (i = 0, busp = pci_devices; i < pci_count; i++, busp++)
	{
		pci_childcount = 0;
		device_get_children(*busp, &pci_children, &pci_childcount);
		for (j = 0, childp = pci_children;
			j < pci_childcount; j++, childp++)
		{

			for (idx = 0; tx_ctl_pci_tbl[idx].vendor; idx++)
				if ( ((tx_ctl_pci_tbl[idx].vendor == PCI_ANY_ID) ||
					  (pci_get_vendor(*childp) == tx_ctl_pci_tbl[idx].vendor)) &&
					 ((tx_ctl_pci_tbl[idx].device == PCI_ANY_ID) ||
					  (pci_get_device(*childp) == tx_ctl_pci_tbl[idx].device)) &&
					 ((tx_ctl_pci_tbl[idx].subvendor == PCI_ANY_ID) ||
					  (pci_get_subvendor(*childp) == tx_ctl_pci_tbl[idx].subvendor)) &&
					 ((tx_ctl_pci_tbl[idx].subdevice == PCI_ANY_ID) ||
					  (pci_get_subdevice(*childp) == tx_ctl_pci_tbl[idx].subdevice))
				   )
					idx_dev++;

		} 
	}
	return idx_dev;
}


int
bpmod_alloc_devices(void)
{
	devclass_t      pci_devclass;
	device_t        *pci_devices;
	int         pci_count = 0;
	device_t        *pci_children;
	int         pci_childcount = 0;
	device_t        *busp, *childp;
	device_t        child = NULL;
	int         i, j, idx, rid, idx_dev=0;

	if ((pci_devclass = devclass_find("pci")) == NULL)
		return ENODEV;

	devclass_get_devices(pci_devclass, &pci_devices, &pci_count);

	for (i = 0, busp = pci_devices; i < pci_count; i++, busp++)
	{
		pci_childcount = 0;
		device_get_children(*busp, &pci_children, &pci_childcount);
		for (j = 0, childp = pci_children;
			j < pci_childcount; j++, childp++)
		{
			for (idx = 0; tx_ctl_pci_tbl[idx].vendor; idx++)
			{
				if ( ((tx_ctl_pci_tbl[idx].vendor == PCI_ANY_ID) ||
					  (pci_get_vendor(*childp) == tx_ctl_pci_tbl[idx].vendor)) &&
					 ((tx_ctl_pci_tbl[idx].device == PCI_ANY_ID) ||
					  (pci_get_device(*childp) == tx_ctl_pci_tbl[idx].device)) &&
					 ((tx_ctl_pci_tbl[idx].subvendor == PCI_ANY_ID) ||
					  (pci_get_subvendor(*childp) == tx_ctl_pci_tbl[idx].subvendor)) &&
					 ((tx_ctl_pci_tbl[idx].subdevice == PCI_ANY_ID) ||
					  (pci_get_subdevice(*childp) == tx_ctl_pci_tbl[idx].subdevice))
				   )
				{
					child= *childp;
					rid = EM_MMBA;

					{
						struct pci_devinfo
						{
							STAILQ_ENTRY(pci_devinfo) pci_links;
							struct resource_list resources;         
						}* dinfo = (struct pci_devinfo *)device_get_ivars(*childp);
						struct resource_list *rl = &dinfo->resources;
						struct resource_list_entry *rle=resource_list_find(rl,SYS_RES_MEMORY,rid);

						if ((!rle)||(!(rle->res)))
						{
							printf("Error: Network driver is not loaded!!!\nPlease load the network driver and after that reload bpmod driver.\n" );
							free(pci_devices, M_TEMP);
							free(pci_children, M_TEMP);
							return ENOMEM;
						}
						/*bpctl_dev[idx_dev].res_memory = bus_alloc_resource(*childp, SYS_RES_MEMORY,
																		   &rid, 0, ~0, 1,
																		   0x20);*/
						bpctl_dev_arr[idx_dev].res_memory=rle->res;                                    
					}

					if (!(bpctl_dev_arr[idx_dev].res_memory))
					{
						printf("Unable to allocate bus resource: memory\n" );
						free(pci_devices, M_TEMP);
						free(pci_children, M_TEMP);
						return ENOMEM;
					}
					bpctl_dev_arr[idx_dev].osdep.mem_bus_space_tag = 
					rman_get_bustag(bpctl_dev_arr[idx_dev].res_memory);
					bpctl_dev_arr[idx_dev].osdep.mem_bus_space_handle = 
					rman_get_bushandle(bpctl_dev_arr[idx_dev].res_memory);

					bpctl_dev_arr[idx_dev].back = & bpctl_dev_arr[idx_dev].osdep;
					bpctl_dev_arr[idx_dev].desc=dev_desc[tx_ctl_pci_tbl[idx].index].name;
					bpctl_dev_arr[idx_dev].name=tx_ctl_pci_tbl[idx].bp_name;
					bpctl_dev_arr[idx_dev].bus=pci_get_bus(child);
					bpctl_dev_arr[idx_dev].func= pci_get_function(child);
					bpctl_dev_arr[idx_dev].slot =pci_get_slot(child);
					bpctl_dev_arr[idx_dev].pdev=child;
					bpctl_dev_arr[idx_dev].device=tx_ctl_pci_tbl[idx].device;
					bpctl_dev_arr[idx_dev].vendor=tx_ctl_pci_tbl[idx].vendor;
					bpctl_dev_arr[idx_dev].subdevice=tx_ctl_pci_tbl[idx].subdevice;
					bpctl_dev_arr[idx_dev].subvendor=tx_ctl_pci_tbl[idx].subvendor;
					if (BP10G9_IF_SERIES(bpctl_dev_arr[idx_dev].subdevice))
						bpctl_dev_arr[idx_dev].bp_10g9=1;
					if (BP10G_IF_SERIES(bpctl_dev_arr[idx_dev].subdevice))
						bpctl_dev_arr[idx_dev].bp_10g=1;
					if (PEG540_IF_SERIES(bpctl_dev_arr[idx_dev].subdevice)) {

						bpctl_dev_arr[idx_dev].bp_540=1;
					}
					if (PEGF5_IF_SERIES(bpctl_dev_arr[idx_dev].subdevice))
						bpctl_dev_arr[idx_dev].bp_fiber5=1;
					if (PEG80_IF_SERIES(bpctl_dev_arr[idx_dev].subdevice))
						bpctl_dev_arr[idx_dev].bp_i80=1;
					if (PEGF80_IF_SERIES(bpctl_dev_arr[idx_dev].subdevice))
						bpctl_dev_arr[idx_dev].bp_i80=1;
					if ((bpctl_dev_arr[idx_dev].subdevice&0xfe0)==0xb60)
						bpctl_dev_arr[idx_dev].bp_fiber5=1;
					if ((bpctl_dev_arr[idx_dev].subdevice&0xfc0)==0xb40)
						bpctl_dev_arr[idx_dev].bp_fiber5=1;
					if ((bpctl_dev_arr[idx_dev].subdevice&0xfe0)==0xaa0)
						bpctl_dev_arr[idx_dev].bp_i80=1;
					if (BP40_IF_SERIES(bpctl_dev_arr[idx_dev].subdevice)) {
						bpctl_dev_arr[idx_dev].bp_40g=1;
					}

					if (BP10GB_IF_SERIES(bpctl_dev_arr[idx_dev].subdevice)) {
						if (bpctl_dev_arr[idx_dev].ifindex==0) {
							printf("Please load network driver for %s adapter!\n",bpctl_dev_arr[idx_dev].name);
							free(pci_devices, M_TEMP);
							free(pci_children, M_TEMP);
							return ENODEV;
						}

						if (bpctl_dev_arr[idx_dev].ndev) {
							if (!(bpctl_dev_arr[idx_dev].ndev->if_flags&IFF_UP)) {
								printf("Please bring up network interfaces for %s adapter!\n",
									   bpctl_dev_arr[idx_dev].name);
								free(pci_devices, M_TEMP);
								free(pci_children, M_TEMP);
								return ENODEV;

							}
						}
						bpctl_dev_arr[idx_dev].bp_10gb=1;
					}


					if ((!bpctl_dev_arr[idx_dev].bp_10g9) &&
						(!bpctl_dev_arr[idx_dev].bp_40g))  {
						if (is_bypass_fn(&bpctl_dev_arr[idx_dev]))
						{
							printf("%s found, ", bpctl_dev_arr[idx_dev].name);

							bpctl_dev_arr[idx_dev].bp_fw_ver=bypass_fw_ver(&bpctl_dev_arr[idx_dev]);

							if ((bpctl_dev_arr[idx_dev].bp_10gb==1)&&                                                  
								(bpctl_dev_arr[idx_dev].bp_fw_ver==0xff)) {                                            
								int cnt=100;                                                                           
								while (cnt--) {                                                                        
									/*iounmap ((void *)(bpctl_dev_arr[idx_dev].mem_map));*/                            

									child= *childp;                                                                    
									rid = EM_MMBA;                                                                     

									bpctl_dev_arr[idx_dev].res_memory = bus_alloc_resource_any(*childp, SYS_RES_MEMORY,
																							   &rid, RF_ACTIVE);               
									if (!(bpctl_dev_arr[idx_dev].res_memory))
									{                                                                                  
										printf("Unable to allocate bus resource: memory\n" );                          
										free(pci_devices, M_TEMP);
										free(pci_children, M_TEMP);
										return ENOMEM;                                                                 
									}
									bpctl_dev_arr[idx_dev].osdep.mem_bus_space_tag =                                   
									rman_get_bustag(bpctl_dev_arr[idx_dev].res_memory);                            
									bpctl_dev_arr[idx_dev].osdep.mem_bus_space_handle =                                
									rman_get_bushandle(bpctl_dev_arr[idx_dev].res_memory);                         


									bpctl_dev_arr[idx_dev].bp_fw_ver=bypass_fw_ver(&bpctl_dev_arr[idx_dev]);           
									if (bpctl_dev_arr[idx_dev].bp_fw_ver==0xa8)
										break;

								}                                                                                      
							}
							printf("firmware version: 0x%x\n",bpctl_dev_arr[idx_dev].bp_fw_ver);
						}


						bpctl_dev_arr[idx_dev].wdt_status=WDT_STATUS_UNKNOWN;
						bpctl_dev_arr[idx_dev].reset_time=0;
						bpctl_dev_arr[idx_dev].wdt_busy=0;
						bpctl_dev_arr[idx_dev].bp_status_un=1;


						bypass_caps_init(&bpctl_dev_arr[idx_dev]);
						init_bypass_wd_auto(&bpctl_dev_arr[idx_dev]);
						init_bypass_tpl_auto(&bpctl_dev_arr[idx_dev]);
					} /*!bpctl_dev_arr[idx_dev].bp_10g9*/
					if (device_get_nameunit(*childp)!=NULL)
					{
						/* struct softc_ex *softc=NULL; */
						struct ifnet *ifp=NULL;

						snprintf(bpctl_dev_arr[idx_dev].nameunit, IFNAMSIZ, "%s", device_get_nameunit(*childp));
						ifp = ifunit_ref(bpctl_dev_arr[idx_dev].nameunit);
						if (ifp) {
							bpctl_dev_arr[idx_dev].ndev=ifp;
							bpctl_dev_arr[idx_dev].ifindex=ifp->if_index;
						}
					}

					if (NOKIA_SERIES(bpctl_dev_arr[idx_dev].subdevice))
						reset_cont(&bpctl_dev_arr[idx_dev]) ;

					idx_dev++;

				}
			}
		}
	}

	for (idx_dev = 0; ((bpctl_dev_arr[idx_dev].pdev!=NULL)&&(idx_dev<device_num)); idx_dev++) {
		
		{
			bpctl_dev_t *pbpctl_dev_c=get_status40_port_fn(&bpctl_dev_arr[idx_dev]);


			if ((bpctl_dev_arr[idx_dev].func!=0)&&(pbpctl_dev_c)) {

				if (pbpctl_dev_c->bp_40g) {
					bpctl_dev_arr[idx_dev].bp_40g=1;
					bpctl_dev_arr[idx_dev].desc=pbpctl_dev_c->desc; 
					bpctl_dev_arr[idx_dev].name=pbpctl_dev_c->name;
				}
			}
		}

		if ((bpctl_dev_arr[idx_dev].bp_10g9) || (bpctl_dev_arr[idx_dev].bp_40g)){
			if (is_bypass_fn(&bpctl_dev_arr[idx_dev])) {
				printf("%s found, ", bpctl_dev_arr[idx_dev].name);
				bpctl_dev_arr[idx_dev].bp_fw_ver=bypass_fw_ver(&bpctl_dev_arr[idx_dev]);
				printf("firmware version: 0x%x\n",bpctl_dev_arr[idx_dev].bp_fw_ver);

			}

			bpctl_dev_arr[idx_dev].wdt_status=WDT_STATUS_UNKNOWN;
			bpctl_dev_arr[idx_dev].reset_time=0;
			bpctl_dev_arr[idx_dev].wdt_busy=0;
			bpctl_dev_arr[idx_dev].bp_status_un=1;


			bypass_caps_init(&bpctl_dev_arr[idx_dev]);

			init_bypass_wd_auto(&bpctl_dev_arr[idx_dev]);
			init_bypass_tpl_auto(&bpctl_dev_arr[idx_dev]);

		}

	}


	free(pci_devices, M_TEMP);
	free(pci_children, M_TEMP);
	return 0;
}

static moduledata_t bpmod_mod = {
	"bpmod",
	bpmod_load,
	NULL
};
DECLARE_MODULE(bpmod, bpmod_mod, SI_SUB_PROTO_IF, SI_ORDER_ANY);












