/**************************************************************************

Copyright (c) 2006, Silicom
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

#ifndef BYPASS_H
#define BYPASS_H

#include "bp_mod.h"
          
/* Bypass related */

#define SYNC_CMD_VAL               2      /* 10b */
#define SYNC_CMD_LEN               2

#define WR_CMD_VAL                 2      /* 10b */
#define WR_CMD_LEN                 2 

#define RD_CMD_VAL                 1      /* 10b */
#define RD_CMD_LEN                 2 

#define ADDR_CMD_LEN               4 

#define WR_DATA_LEN                8
#define RD_DATA_LEN                8


#define PIC_SIGN_REG_ADDR          0x7
    #define PIC_SIGN_VALUE         0xcd


#define STATUS_REG_ADDR           0
    #define WDT_EN_MASK            0x01    /* BIT_0 */
    #define CMND_EN_MASK           0x02    /* BIT_1 */
    #define DIS_BYPASS_CAP_MASK    0x04    /* BIT_2 Bypass Cap is disable */
    #define DFLT_PWRON_MASK        0x08    /* BIT_3 */
    #define BYPASS_OFF_MASK        0x10    /* BIT_4 */
    #define BYPASS_FLAG_MASK       0x20    /* BIT_5 */
    #define STD_NIC_MASK           (DIS_BYPASS_CAP_MASK | BYPASS_OFF_MASK | DFLT_PWRON_MASK)
    #define WD_EXP_FLAG_MASK       0x40    /* BIT_6 */
    #define DFLT_PWROFF_MASK       0x80    /*BIT_7 */
    #define STD_NIC_PWOFF_MASK     (DIS_BYPASS_CAP_MASK | BYPASS_OFF_MASK | DFLT_PWRON_MASK | DFLT_PWROFF_MASK)
    
    
#define PRODUCT_CAP_REG_ADDR   0x5
    #define BYPASS_SUPPORT_MASK    0x01    /* BIT_0 */
    #define TAP_SUPPORT_MASK       0x02    /* BIT_1 */
    #define NORMAL_UNSUPPORT_MASK  0x04    /* BIT_2 */
    #define DISC_SUPPORT_MASK      0x08    /* BIT_3 */
    #define TPL2_SUPPORT_MASK      0x10    /* BIT_4 */
#define DISC_PORT_SUPPORT_MASK 0x20    //BIT_5
    

#define STATUS_TAP_REG_ADDR    0x6
    #define WDTE_TAP_BPN_MASK      0x01    /* BIT_1  1 when wdt expired -> TAP, 0 - Bypass */
    #define DIS_TAP_CAP_MASK       0x04    /* BIT_2 TAP Cap is disable*/
    #define DFLT_PWRON_TAP_MASK    0x08    /* BIT_3 */
    #define TAP_OFF_MASK           0x10    /* BIT_4 */
    #define TAP_FLAG_MASK          0x20    /* BIT_5 */
#define TX_DISA_MASK            0x40										    
#define TX_DISB_MASK            0x80 
    #define STD_NIC_TAP_MASK       (DIS_TAP_CAP_MASK | TAP_OFF_MASK | DFLT_PWRON_TAP_MASK)
    
#define STATUS_DISC_REG_ADDR    13
    #define WDTE_DISC_BPN_MASK      0x01    /* BIT_0  */  /* 1 when wdt expired -> TAP, 0 - Bypass */
    #define STD_NIC_ON_MASK         0x02    /* BIT_1  */ 
    #define DIS_DISC_CAP_MASK       0x04    /* BIT_2  */  /* TAP Cap is disable*/
    #define DFLT_PWRON_DISC_MASK    0x08    /* BIT_3   */ 
    #define DISC_OFF_MASK           0x10    /* BIT_4   */ 
    #define DISC_FLAG_MASK          0x20    /* BIT_5   */ 
    #define TPL2_FLAG_MASK          0x40    /* BIT_6   */
    #define STD_NIC_DISC_MASK       DIS_DISC_CAP_MASK
    
#define CONT_CONFIG_REG_ADDR    12
    #define EN_HW_RESET_MASK       0x2  /* BIT_1 */
    #define WAIT_AT_PWUP_MASK      0x1  /* BIT_0 */
    

#define VER_REG_ADDR               0x1
    #define BP_FW_VER_A0           0xa0
    #define BP_FW_VER_A1           0xa1
    #define INT_VER_MASK           0xf0
    #define EXT_VER_MASK           0xf
    /* */
    #define PXG2BPI_VER            0x0 
    #define PXG2TBPI_VER           0x1
    #define PXE2TBPI_VER           0x2 
    #define PXG4BPFI_VER           0x4
    #define BP_FW_EXT_VER7         0x6
    #define BP_FW_EXT_VER8         0x8
    #define BP_FW_EXT_VER9         0x9

    #define OLD_IF_VER              -1
    
#define CMND_REG_ADDR              10     /* 1010b */
#define WDT_REG_ADDR               4
#define TMRL_REG_ADDR              2
#define TMRH_REG_ADDR              3

/* NEW_FW */
#define WDT_INTERVAL               1   
#define WDT_CMND_INTERVAL          50 
#define CMND_INTERVAL              100  /* usec */
#define PULSE_TIME                 100  

/* OLD_FW */
#define INIT_CMND_INTERVAL         40
#define PULSE_INTERVAL             5
#define WDT_TIME_CNT               3

/* Intel Commands */

#define CMND_OFF_INT               0xf
#define PWROFF_BYPASS_ON_INT       0x5
#define BYPASS_ON_INT              0x6
#define DIS_BYPASS_CAP_INT         0x4
#define RESET_WDT_INT              0x1

/* Intel timing */

#define BYPASS_DELAY_INT           4     /* msec */
#define CMND_INTERVAL_INT          2     /* msec */
             
/* Silicom Commands */



/* Commands */
#define CMND_ON                    0x4
#define CMND_OFF                   0x2 
#define BYPASS_ON                  0xa
#define BYPASS_OFF                 0x8
#define PORT_LINK_EN               0xe
#define PORT_LINK_DIS              0xc
#define WDT_ON                     0x10                /* 0x1f (11111) - max*/
#define TIMEOUT_UNIT           100
#define TIMEOUT_MAX_STEP       15
#define WDT_TIMEOUT_MIN        100                  /*  msec */
#define WDT_TIMEOUT_MAX        3276800              /*  msec */
#define WDT_AUTO_MIN_INT           500

#define WDT_TIMEOUT_DEF        WDT_TIMEOUT_MIN
#define WDT_OFF                    0x6
#define WDT_RELOAD                 0x9
#define RESET_CONT                 0x20
#define DIS_BYPASS_CAP             0x22
#define EN_BYPASS_CAP              0x24
#define BYPASS_STATE_PWRON         0x26
#define NORMAL_STATE_PWRON         0x28
#define BYPASS_STATE_PWROFF        0x27
#define NORMAL_STATE_PWROFF        0x29
#define TAP_ON                     0xb
#define TAP_OFF                    0x9
#define TAP_STATE_PWRON            0x2a
#define DIS_TAP_CAP                0x2c
#define EN_TAP_CAP                 0x2e
#define STD_NIC_OFF       0x86
#define STD_NIC_ON       0x84
#define DISC_ON           0x85    
#define DISC_OFF          0x8a    
#define DISC_STATE_PWRON  0x87    
#define DIS_DISC_CAP      0x88    
#define EN_DISC_CAP       0x89   
#define TPL2_ON                    0x8c
#define TPL2_OFF                   0x8b
#define BP_WAIT_AT_PWUP_EN        0x80    
#define BP_WAIT_AT_PWUP_DIS       0x81    
#define BP_HW_RESET_EN             0x82    
#define BP_HW_RESET_DIS            0x83 

#define TX_DISA                0x8d    
#define TX_DISB                0x8e    
#define TX_ENA                 0xA0    
#define TX_ENB                 0xA1    

#define TX_DISA_PWRUP          0xA2    
#define TX_DISB_PWRUP          0xA3    
#define TX_ENA_PWRUP           0xA4    
#define TX_ENB_PWRUP           0xA5    

#define BYPASS_CAP_DELAY           21     /* msec */
#define DFLT_PWRON_DELAY           10     /* msec */
#define LATCH_DELAY                13      /* msec */
#define EEPROM_WR_DELAY             8     /* msec */

#define BP_LINK_MON_DELAY          4 /* sec */


#define BP_FW_EXT_VER0                 0xa0
#define BP_FW_EXT_VER1                 0xa1
#define BP_FW_EXT_VER2                0xb1 

#define BP_OK        0
#define BP_NOT_CAP  -1
#define WDT_STATUS_EXP -2
#define WDT_STATUS_UNKNOWN -1
#define WDT_STATUS_EN 1
#define WDT_STATUS_DIS 0

struct softc_ex {
    struct ifnet   *ifp;
};

struct bpctl_osdep {
    bus_space_tag_t    mem_bus_space_tag;
	bus_space_handle_t mem_bus_space_handle;
	struct device     *dev;
};

/* Media Types */
typedef enum {
    bp_copper = 0,
    bp_fiber,
    bp_cx4,
    bp_none,
} bp_media_type;


typedef struct _bpctl_dev {
    char *name;
    char *desc;
    char nameunit[IFNAMSIZ];
    device_t pdev;  /* PCI device */
    struct ifnet *ndev;
    void *back;
    unsigned long mem_map;
    u_int8_t  bus;
    u_int8_t  slot;
    u_int8_t  func;
    u_int32_t  device;
    u_int32_t  vendor;
    u_int32_t  subvendor;
    u_int32_t  subdevice;
    int      ifindex;
    u_int32_t bp_caps;
    u_int32_t bp_caps_ex;
    u_int8_t  bp_fw_ver;
    int  bp_ext_ver;
    int wdt_status;
    unsigned long bypass_wdt_on_time;
    u_int32_t bypass_timer_interval;
    struct callout bp_timer;
    u_int32_t reset_time;
    int      wdt_busy;
    uint8_t   bp_status_un;
    int   bp_10g;
    int   bp_10gb;
    int   bp_fiber5;
    int   bp_10g9;
    int   bp_i80; 
    int   bp_540;
	int   bp_40g;
    struct callout bp_tpl_timer;
    int   bp_tpl_flag;
    bp_media_type media_type;

    struct bpctl_osdep osdep;
    struct resource *res_memory;


} bpctl_dev_t;

int is_bypass_fn(bpctl_dev_t *pbpctl_dev);
int wdt_time_left(bpctl_dev_t *pbpctl_dev);
void wd_reset_timer(void *param);


#endif    /* BYPASS_H*/


