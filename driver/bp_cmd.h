#ifndef __BP_CMD_H__
#define __BP_CMD_H__

//#define FW_HEADER_DEF  1

#ifndef FW_HEADER_DEF
	typedef unsigned int	       	U32_T;			/* 32-bit unsigned */
	typedef unsigned short int		U16_T;    		/* 16-bit  unsigned */
	typedef unsigned char       	U8_T;			/* 8-bit unsigned */
#endif
	typedef U8_T			       	byte;


#ifndef FW_HEADER_DEF
	#pragma pack(push)  /* push current alignment to stack */
	#pragma pack(1)     /* set alignment to 1 byte boundary */
#endif


/******************************************************\
*                                                      * 
*      COMMUNICATION PROTOCOL SPECIFICATION            *
*                                                      * 
*	  <COMMAND_ID><COMMAND_LEN><COMMAND_DATA>          *
*                                                      *
\******************************************************/

/*
 * BP cmd IDs
 */
#define	CMD_GET_BYPASS_CAPS				1
#define CMD_GET_WD_SET_CAPS				2
#define CMD_SET_BYPASS					3
#define CMD_GET_BYPASS					4
#define CMD_GET_BYPASS_CHANGE			5
#define CMD_SET_BYPASS_WD				6
#define CMD_GET_BYPASS_WD				7
#define CMD_GET_WD_EXPIRE_TIME			8
#define CMD_RESET_BYPASS_WD_TIMER		9
#define CMD_SET_DIS_BYPASS				10
#define CMD_GET_DIS_BYPASS				11
#define CMD_SET_BYPASS_PWOFF			12
#define CMD_GET_BYPASS_PWOFF			13
#define CMD_SET_BYPASS_PWUP				14
#define CMD_GET_BYPASS_PWUP				15
#define CMD_SET_STD_NIC					16
#define CMD_GET_STD_NIC					17
#define CMD_SET_TAP						18
#define CMD_GET_TAP						19        
#define CMD_GET_TAP_CHANGE				20
#define CMD_SET_DIS_TAP					21
#define CMD_GET_DIS_TAP					22
#define CMD_SET_TAP_PWUP				23 
#define CMD_GET_TAP_PWUP				24
#define CMD_SET_WD_EXP_MODE				25                       
#define CMD_GET_WD_EXP_MODE				26
#define CMD_SET_DISC					27
#define CMD_GET_DISC					28         
#define CMD_GET_DISC_CHANGE				29
#define CMD_SET_DIS_DISC				30
#define CMD_GET_DIS_DISC				31
#define CMD_SET_DISC_PWUP				32 
#define CMD_GET_DISC_PWUP				33
#define CMD_SET_DISC_PWOFF				34 
#define CMD_GET_DISC_PWOFF				35
#define CMD_GET_BYPASS_CAPS_EX			36
#define CMD_SET_HOST_PWOFF_MODE			37
#define CMD_GET_HOST_PWOFF_MODE			38
#define CMD_SET_HOST_PWUP_MODE			39
#define CMD_GET_HOST_PWUP_MODE			40
#define CMD_SET_DISC_PORT				41
#define CMD_GET_DISC_PORT				42
#define CMD_SET_DISC_PORT_PWUP 			43
#define CMD_GET_DISC_PORT_PWUP			44
#define CMD_SET_WD_RST_RESTORE_MODE		45
#define CMD_GET_WD_RST_RESTORE_MODE		46
#define CMD_SET_TPL						47
#define CMD_GET_TPL						48
#define CMD_SET_TX						49
#define CMD_GET_TX						50


#define CMD_GET_BYPASS_INFO				100
#define CMD_GET_BP_WAIT_AT_PWUP			101
#define CMD_SET_BP_WAIT_AT_PWUP			102
#define CMD_GET_BP_HW_RESET				103
#define CMD_SET_BP_HW_RESET				104
#define CMD_SET_BP_MANUF				105
//#define CMD_RESERVED				131-132
#define CMD_GET_DEV_INFO				133
//#define CMD_RESERVED				132-139


typedef U8_T cmd_id_t;



/*
 * BP cmd response codes
 */
#define BP_ERR_OK						1
#define BP_ERR_NOT_CAP					2
#define BP_ERR_DEV_BUSY					3
#define BP_ERR_INVAL_DEV_NUM			4 
#define BP_ERR_INVAL_STATE				5 
#define BP_ERR_INVAL_PARAM				6 
#define BP_ERR_UNSUPPORTED_PARAM		7 
#define BP_ERR_INVAL_CMD				8 
#define BP_ERR_UNSUPPORTED_CMD			9 
#define BP_ERR_INTERNAL					10
#define BP_ERR_UNKNOWN					11

typedef U8_T cmd_rsp_id_t;

typedef U8_T cmd_bp_board_t;



/*
 * BP device info
 */
typedef struct _cmd_bp_info_t{
	cmd_bp_board_t dev_id;
	byte fw_ver;
}cmd_bp_info_t;


/*
 * Disc port
 */
typedef struct _disc_port_t{
	U8_T mode;
	U8_T port_num;
}disc_port_t;


/*
 * Tx disable
 */
typedef struct _tx_dis_t{
	U8_T mode;
	U8_T port_num;
}tx_dis_t;


/*
 * Device info
 */
typedef union _cmd_dev_info_t{
	U8_T info_id;
    struct {
		U8_T dev_id;
		U8_T dev_rev;
		U8_T eeprom_ver;
		U8_T eeprom_rev;
    }dev_ver;
	U8_T bds_rev[4];
	U8_T fw_ver[4];
	U8_T hw_ver[4];
	U32_T fw_build;
    union {
		U8_T part0[4];
		U8_T part1[2];
    }mac_addr;
    union {
		U8_T sn0[4];
		U8_T sn1[4];
		U8_T sn2[4];
		U8_T sn3[4];
    }sn;
}cmd_dev_info_t;


/*
 * BP command device number
 */
#define  DEV_NUM_MAX 4
typedef U8_T cmd_dev_num_t;


/*
 * BP command data lenth type
 */
typedef U8_T cmd_data_len_t;


/*
 * BP command data structure
 */
typedef union _cmd_data_t{
#if 0
	/* 
	 * CMD_GET_BYPASS_CAPS
	 */
	/* 
	 * CMD_GET_BYPASS_CAPS_EX (1st double word)
	 */
#define BP_CAP                   BIT0
#define BP_STATUS_CAP            BIT1
#define BP_STATUS_CHANGE_CAP     BIT2
#define SW_CTL_CAP               BIT3
#define BP_DIS_CAP               BIT4
#define BP_DIS_STATUS_CAP        BIT5
#define STD_NIC_CAP              BIT6
#define BP_PWOFF_ON_CAP          BIT7
#define BP_PWOFF_OFF_CAP         BIT8
#define BP_PWOFF_CTL_CAP         BIT9
#define BP_PWUP_ON_CAP           BIT10
#define BP_PWUP_OFF_CAP          BIT11
#define BP_PWUP_CTL_CAP          BIT12
#define WD_CTL_CAP               BIT13
#define WD_STATUS_CAP            BIT14
#define WD_TIMEOUT_CAP           BIT15
#define TX_CTL_CAP               BIT16  
#define TX_STATUS_CAP            BIT17  
#define TAP_CAP                  BIT18 
#define TAP_STATUS_CAP           BIT19 
#define TAP_STATUS_CHANGE_CAP    BIT20 
#define TAP_DIS_CAP              BIT21
#define TAP_DIS_STATUS_CAP       BIT22
#define TAP_PWUP_ON_CAP          BIT23
#define TAP_PWUP_OFF_CAP         BIT24
#define TAP_PWUP_CTL_CAP         BIT25
#define NIC_CAP_NEG              BIT26
#define TPL_CAP                  BIT27
#define DISC_CAP                 BIT28
#define DISC_DIS_CAP             BIT29
#define DISC_PWUP_CTL_CAP        BIT30
#define BP_CAPS_EX_CAP           BIT31
#endif
	U32_T bypass_caps;


	/* 
	 * CMD_GET_BYPASS_CAPS_EX (2nd double word)
	 */
#if 0
#define DISC_PWOFF_CTL_CAP        BIT0
#define BP_HOST_PWOFF_CTL_CAP     BIT1
#define BP_HOST_PWUP_CTL_CAP      BIT2
#define DISC_HOST_PWOFF_CTL_CAP   BIT3
#define DISC_HOST_PWUP_CTL_CAP    BIT4
#define NORMAL_HOST_PWOFF_CTL_CAP BIT5
#define NORMAL_HOST_PWUP_CTL_CAP  BIT6
#define TAP_HOST_PWOFF_CTL_CAP    BIT7
#define TAP_HOST_PWUP_CTL_CAP     BIT8
#define DISC_PORT_CAP             BIT9
#define DISC_PORT_PWUP_CTL_CAP    BIT10
#endif

	U32_T bypass_caps_ex;


	/* 
	 * CMD_GET_WD_SET_CAPS
	 */

	/* --------------------------------------------------------------------------------------
	 * Bit	feature			description									Products
	 * ---------------------------------------------------------------------------------------
	 * 0-3	WD_MIN_TIME		The interface WD minimal time period in		PXG2/4BP - 5 (500mS)
	 *						100mS units									All the other
	 *																	products - 1(100mS)
	 * ---------------------------------------------------------------------------------------
	 * 4 	WD_STEP_TIME 	The steps of the WD timer in				PXG2/4BP - 0
	 *						0 - for linear steps (WD_MIN_TIME * X)		All the other
	 *						1 - for multiply by 2 from previous step	products - 1
	 *						(WD_MIN_TIME * 2^X)
	 * ---------------------------------------------------------------------------------------
	 * 5-8 	WD_STEP_COUNT 	Number of register bits available for		PXG2/4BP -7
	 *						configuring the WD timer. From that the		All the other
	 *						number of steps the WD timer will have is	products - 4
	 *						2^X (X number of bits available for
	 *						defining the value)
	 * ---------------------------------------------------------------------------------------
	 * 9-31					RESERVED Reserved, should be ignored.
	 * -------------------------------------------------------------------------------------*/

	U32_T wd_set_caps;


	/* 
	 * CMD_SET_BYPASS
	 */
	/* 
	 * CMD_GET_BYPASS
	 */
/* #define BYPASS_OFF             0 */
/* #define BYPASS_ON              1 */

	U8_T bypass_mode;


	/* 
	 * CMD_GET_BYPASS_CHANGE
	 */
#define BYPASS_NOT_CHANGED     0
#define BYPASS_CHANGED         1

	U8_T bypass_change;


	/* 
	 * CMD_SET_BYPASS_WD
	 */
	/* 
	 * CMD_GET_BYPASS_WD
	 */

	U32_T timeout;
	U32_T timeout_set;


	/* 
	 * CMD_GET_WD_EXPIRE_TIME
	 */

	U32_T time_left;


	/* 
	 * CMD_SET_DIS_BYPASS
	 */
	/* 
	 * CMD_GET_DIS_BYPASS
	 */
#define DIS_BYPASS_ENABLE      0
#define DIS_BYPASS_DISABLE     1

	U8_T dis_bypass;


	/* 
	 * CMD_SET_BYPASS_PWOFF
	 */
	/* 
	 * CMD_GET_BYPASS_PWOFF
	 */
#define BYPASS_PWOFF_DIS       0
#define BYPASS_PWOFF_EN        1

	U8_T bypass_pwoff;


	/* 
	 * CMD_SET_BYPASS_PWUP
	 */
	/* 
	 * CMD_GET_BYPASS_PWUP
	 */
#define BYPASS_PWUP_DIS        0
#define BYPASS_PWUP_EN         1

	U8_T bypass_pwup;


	/* 
	 * CMD_SET_HOST_PWOFF_MODE
	 */
	/* 
	 * CMD_GET_HOST_PWOFF_MODE
	 */
#define HOST_PWOFF_MODE_BYPASS    0
#define HOST_PWOFF_MODE_TAP       1
#define HOST_PWOFF_MODE_DISC      2
#define HOST_PWOFF_MODE_NORMAL    3

	U8_T host_pwoff_mode;


	/* 
	 * CMD_SET_HOST_PWUP_MODE
	 */
	/* 
	 * CMD_GET_HOST_PWUP_MODE
	 */
#define HOST_PWUP_MODE_BYPASS     0
#define HOST_PWUP_MODE_TAP        1
#define HOST_PWUP_MODE_DISC       2
#define HOST_PWUP_MODE_NORMAL     3

	U8_T host_pwup_mode;


	/* 
	 * CMD_SET_STD_NIC
	 */
	/* 
	 * CMD_GET_STD_NIC
	 */
#define STD_NIC_DIS            0
#define STD_NIC_EN             1

	U8_T std_nic;


	/* 
	 * CMD_SET_TAP
	 */
	/* 
	 * CMD_GET_TAP
	 */
/* #define TAP_OFF                0 */
/* #define TAP_ON                 1 */

	U8_T tap_mode;


	/* 
	 * CMD_GET_TAP_CHANGE
	 */
#define TAP_NOT_CHANGED        0
#define TAP_CHANGED            1

	U8_T tap_change;


	/* 
	 * CMD_SET_DIS_TAP
	 */
	/* 
	 * CMD_GET_DIS_TAP
	 */
#define DIS_TAP_ENABLE         0
#define DIS_TAP_DISABLE        1

	U8_T dis_tap;


	/* 
	 * CMD_SET_TAP_PWUP
	 */
	/* 
	 * CMD_GET_TAP_PWUP
	 */
#define TAP_PWUP_DIS           0
#define TAP_PWUP_EN            1

	U8_T tap_pwup;


	/* 
	 * CMD_SET_WD_EXP_MODE
	 */
	/* 
	 * CMD_GET_WD_EXP_MODE
	 */
#define WD_EXP_MODE_BYPASS     0
#define WD_EXP_MODE_TAP        1
#define WD_EXP_MODE_DISC       2
#define WD_EXP_MODE_NORMAL     3

	U8_T wd_exp_mode;


	/* 
	 * CMD_SET_WD_RST_RESTORE_MODE
	 */
	/* 
	 * CMD_GET_WD_RST_RESTORE_MODE
	 */
#define WD_RST_RESTORE_MODE_BYPASS     0
#define WD_RST_RESTORE_MODE_TAP        1
#define WD_RST_RESTORE_MODE_DISC       2
#define WD_RST_RESTORE_MODE_NORMAL     3
#define WD_RST_RESTORE_MODE_LAST_STATE 4
#define WD_RST_RESTORE_MODE_STAY_OFF   5

	U8_T wd_rst_restore_mode;


	/* 
	 * CMD_SET_DISC
	 */
	/* 
	 * CMD_GET_DISC
	 */
/* #define DISC_OFF               0 */
/* #define DISC_ON                1 */

	U8_T disc_mode;


	/* 
	 * CMD_GET_DISC_CHANGE
	 */
#define DISC_NOT_CHANGED       0
#define DISC_CHANGED           1

	U8_T disc_change;


	/* 
	 * CMD_SET_DIS_DISC
	 */
	/* 
	 * CMD_GET_DIS_DISC
	 */
#define DIS_DISC_ENABLE        0
#define DIS_DISC_DISABLE       1

	U8_T dis_disc;


	/* 
	 * CMD_SET_DISC_PWUP
	 */
	/* 
	 * CMD_GET_DISC_PWUP
	 */
#define DISC_PWUP_DIS        0
#define DISC_PWUP_EN         1

	U8_T disc_pwup;


	/* 
	 * CMD_SET_DISC_PWOFF
	 */
	/* 
	 * CMD_GET_DISC_PWOFF
	 */
#define DISC_PWOFF_DIS        0
#define DISC_PWOFF_EN         1

	U8_T disc_pwoff;


	/* 
	 * CMD_SET_DISC_PORT
	 */
	/* 
	 * CMD_GET_DISC_PORT
	 */
#define DISC_PORT_OFF          0
#define DISC_PORT_ON           1

	disc_port_t disc_port;


	/* 
	 * CMD_SET_DISC_PORT_PWUP
	 */
	/* 
	 * CMD_GET_DISC_PORT_PWUP
	 */
#define DISC_PORT_PWUP_DIS       0
#define DISC_PORT_PWUP_EN        1

	disc_port_t disc_port_pwup;


	/* 
	 * CMD_SET_TPL
	 */
	/* 
	 * CMD_GET_TPL
	 */
#define TPL_OFF                0
#define TPL_ON                 1

	U8_T tpl_mode;


	/* 
	 * CMD_SET_TX
	 */
	/* 
	 * CMD_GET_TX
	 */
#define TX_OFF                 0
#define TX_ON                  1

	tx_dis_t tx_dis;


	/* 
	 * CMD_GET_BYPASS_INFO
	 */
	cmd_bp_info_t bypass_info;


	/* 
	 * CMD_GET_DEV_INFO
	 */
#define DEV_INFO_DEV_VER      0
#define DEV_INFO_FW_VER       1
#define DEV_INFO_MAC_PART0    2
#define DEV_INFO_MAC_PART1    3
#define DEV_INFO_SN_PART0     4
#define DEV_INFO_SN_PART1     5
#define DEV_INFO_SN_PART2     6
#define DEV_INFO_SN_PART3     7
#define DEV_INFO_FW_BUILD     8
#define DEV_INFO_BDS_REV      9
#define DEV_INFO_HW_REV      10

	cmd_dev_info_t dev_info;


	/* 
	 * CMD_GET_BP_WAIT_AT_PWUP
	 */
	/* 
	 * CMD_SET_BP_WAIT_AT_PWUP
	 */
/* #define BP_WAIT_AT_PWUP_DIS    0 */
/* #define BP_WAIT_AT_PWUP_EN     1 */
	U8_T bp_wait_at_pwup;


	/* 
	 * CMD_GET_BP_HW_RESET
	 */
	/* 
	 * CMD_SET_BP_HW_RESET
	 */
/* #define BP_HW_RESET_DIS        0 */
/* #define BP_HW_RESET_EN         1 */

	U8_T bp_hw_reset;

}cmd_data_t;


/******************************************************\
*                                                      * 
*   <DEV_NUM><COMMAND_ID><COMMAND_LEN><COMMAND_DATA>   *
*                                                      *
\******************************************************/
#define BP_CMD_PACKET_SIZE ( sizeof(cmd_dev_num_t)  \
                           + sizeof(cmd_id_t)       \
                           + sizeof(cmd_data_len_t) \
                           + sizeof(cmd_data_t) )

typedef union _bp_cmd_t
{
    byte cmd_bytes[BP_CMD_PACKET_SIZE]; //	For Byte Access

    struct {
		cmd_dev_num_t 	cmd_dev_num;
		cmd_id_t		cmd_id;
		cmd_data_len_t	cmd_data_len;
		cmd_data_t 		cmd_data;
    }cmd;

}bp_cmd_t;


/******************************************************\
*                                                      * 
*       <RET_CODE><COMMAND_LEN><COMMAND_DATA>          *
*                                                      *
\******************************************************/
#define BP_RSP_PACKET_SIZE ( sizeof(cmd_rsp_id_t)    \
                           + sizeof(cmd_data_len_t)  \
                           + sizeof(cmd_data_t) )

typedef union _bp_cmd_rsp_t
{
    byte cmd_bytes[BP_RSP_PACKET_SIZE]; //	For Byte Access

    struct {
		cmd_rsp_id_t	rsp_id;
		cmd_data_len_t 	rsp_data_len;
		cmd_data_t 		rsp_data;
    }rsp;

}bp_cmd_rsp_t;

#ifndef FW_HEADER_DEF
	#pragma pack(pop)  /* push current alignment to stack */
#endif

#endif /* End of __BP_CMD_H__ */
