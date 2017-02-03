////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_mtk.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/hwmsen_helper.h>
//#include <linux/hw_module_info.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/namei.h>
#include <linux/vmalloc.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

#include <cust_eint.h>

#include "tpd.h"
#include "cust_gpio_usage.h"

#include "mstar_drv_platform_interface.h"

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "msg2xxx" //"msg21xxA" or "msg22xx" or "msg26xxM" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */
#define I2C_BUS_ID   (1)       // i2c bus id : 0 or 1

#define TPD_OK (0)

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
extern const int g_TpVirtualKey[];

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
extern const int g_TpVirtualKeyDimLocal[][4];
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

extern struct tpd_device *tpd;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

/*
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT] = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8] = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif
*/
struct i2c_client *g_I2cClient = NULL;

//static int boot_mode = 0;

/*=============================================================*/
// FUNCTION DECLARATION
/*=============================================================*/

/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

///lc mike_zhu 20150525 for devinfo
#define  SLT_DEVINFO_CTP
#ifdef SLT_DEVINFO_CTP
#include<linux/dev_info.h>
#define CTP_PROC_FILE "tp_info"
#define CTP_PROC_FILE_VENDER "ctp_vender"
#define  LCT_ADD_TP_VERSION
unsigned char CFG_VER_buff[12]={0};	//read the version name
typedef enum msg2238_ctp
{
	msg2238_ctp_dj_kuantie = 0,	//default module
	msg2238_ctp_dj_quantie = 1,
	msg2238_ctp_txd_quantie = 2,
	msg2238_ctp_txd_kuantie=3,
}MSG2238_CTP_MODULE;
MSG2238_CTP_MODULE msg2238_ctp_module_name = msg2238_ctp_dj_quantie;//ctm
struct devinfo_struct *s_DEVINFO_ctp = NULL;
extern struct tpd_device *tpd;
#if 0
extern void _DrvFwCtrlMsg22xxCheckFirmwareUpdateBySwId(void);
static void get_sw_version(void)
{
printk("msg2238 get_sw_version");
///printk("msg2238  ----0-----------CFG_VER_buff=%s\n",CFG_VER_buff);	
_DrvFwCtrlMsg22xxCheckFirmwareUpdateBySwId(); 
///printk("msg2238  ----2-----------CFG_VER_buff=%s\n",CFG_VER_buff);	
}
#endif
static void devinfo_ctp_regchar(char *module,char * vendor,char *version,char *used)
{
 	s_DEVINFO_ctp =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);	
	s_DEVINFO_ctp->device_type="CTP";
	s_DEVINFO_ctp->device_module=module;
	s_DEVINFO_ctp->device_vendor=vendor;
	s_DEVINFO_ctp->device_ic="MSG2238";
	s_DEVINFO_ctp->device_info=DEVINFO_NULL;
	s_DEVINFO_ctp->device_version=version;
	s_DEVINFO_ctp->device_used=used;
    devinfo_check_add_device(s_DEVINFO_ctp);
}
 #ifdef  LCT_ADD_TP_VERSION
static int ctp_proc_read_show (struct seq_file* m, void* data)
{
	char temp[30] = {0};
	//get_sw_version();
	sprintf(temp, "pid:%s\n", CFG_VER_buff); 
	seq_printf(m, "%s\n", temp);
	return 0;
}

static ctp_proc_open (struct inode* inode, struct file* file) 
{
    return single_open(file, ctp_proc_read_show, inode->i_private);
}

static const struct file_operations g_ctp_proc = 
{
    .open = ctp_proc_open,
    .read = seq_read,
};

//add for ctp_vendor proc
static int tpvendor_proc_show(struct seq_file *m, void *unused)
{
  ///  get_sw_version();
    seq_printf(m, " [Vendor]DJ,[fw]%s,[ic]msg2238\n",CFG_VER_buff);
	return 0;
}
static int tpvendor_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tpvendor_proc_show, NULL);
}
static const struct file_operations ctp_vendor_proc_fops = {
    .open       = tpvendor_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release	= single_release,
};


#endif


#endif

#if 0///def SLT_DEVINFO_CTP
#define  LCT_ADD_TP_VERSION
unsigned char CFG_VER_buff[12]={0};	//read the version name
extern void _DrvFwCtrlMsg22xxCheckFirmwareUpdateBySwId(void);
#endif
#if 0///def SLT_DEVINFO_CTP/// 
#include <linux/dev_info.h>
static struct devinfo_struct *devinfo_tp = NULL;
static unsigned char config_id[4];
//extern unsigned char my_fw_id;
static unsigned char v_info[20];
///static struct proc_dir_entry *g_ctp_proc = NULL;
//static struct proc_dir_entry *g_ctp_vender = NULL;
#define CTP_PROC_FILE "tp_info"

#ifdef LCT_ADD_TP_VERSION
///unsigned char CFG_Module_buff[12]={0};//read the module name
static int ctp_proc_read_show (struct seq_file* m, void* data)
{
	char temp[30] = {0};
	sprintf(temp, "pid:%s\n", CFG_VER_buff); //changed by cao
	seq_printf(m, "%s\n", temp);
	return 0;
}
#if 0

static ctp_proc_open (struct inode* inode, struct file* file) 
{
    return single_open(file, ctp_proc_read_show, inode->i_private);
}
static const struct file_operations g_ctp_proc = 
{
    .open = ctp_proc_open,
    .read = seq_read,
};
#endif
//add for ctp_vendor proc
static int tpvendor_proc_show(struct seq_file *m, void *unused)
{
 //   get_sw_version();
    seq_printf(m, " tp vender:msg2238 , FW version:%s\n",CFG_VER_buff);
	return 0;
}
static int tpvendor_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tpvendor_proc_show, NULL);
}
static const struct file_operations ctp_vendor_proc_fops = {
    .open       = tpvendor_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release	= single_release,
};
static int tpinfo_proc_show(struct seq_file *m, void *unused)
{
    get_sw_version();
    seq_printf(m, "pid:0x%2x\n",config_id[3]);
	return 0;
}
static int tpinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tpinfo_proc_show, NULL);
}
static const struct file_operations ctp_info_proc_fops = {
    .open       = tpinfo_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release	= single_release,
};
#endif
static void tp_devinfo_init()
{
	get_sw_version();
	sprintf(v_info,"id:0x%02x %02x %02x %02x",config_id[0],config_id[1],config_id[2],config_id[3]);
	//sprintf(v_info,"fw_%02x",my_fw_id);
	devinfo_tp =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
	if(NULL != devinfo_tp)
	{
		devinfo_tp->device_type = "TP";
		devinfo_tp->device_ic = "msg2238";
		devinfo_tp->device_used = DEVINFO_USED;
		devinfo_tp->device_module = "unknow";
		devinfo_tp->device_vendor = "DJ";
		devinfo_tp->device_version = v_info;
		devinfo_tp->device_info = "480*854";
		devinfo_check_add_device(devinfo_tp);
	}
}
#endif

/***************** swf add start ****************************/
 void power_on_self(void)
{
	int ret = 0;
	tpd->reg=regulator_get(tpd->tpd_dev,"VGP2"); // get pointer to regulator structure
	ret=regulator_set_voltage(tpd->reg, 2800000, 2800000);  // set 2.8v
	if (ret)
		{
		printk(" TDP regulator_set_voltage() VGP2 failed!\n");
		return -1;
		}
	ret=regulator_enable(tpd->reg);  //enable regulator
	if (ret)
		printk(" regulator_enable() VGP2 failed!\n");

}

 void power_on_self_mstar(void)
{
	int ret = 0;
	//tpd->reg=regulator_get(tpd->tpd_dev,TPD_POWER_SOURCE_CUSTOM); // get pointer to regulator structure
	tpd->reg=regulator_get(tpd->tpd_dev,"VGP1");
	//tpd->reg=regulator_get(tpd->tpd_dev,"CAP_TOUCH_VDD");
	///tpd->reg=regulator_get(tpd->tpd_dev,MT6325_POWER_LDO_VGP1); // get pointer to regulator structure
	ret=regulator_set_voltage(tpd->reg, 2800000, 2800000);  // set 2.8v
	if (ret)
		{
		printk(" TDP regulator_set_voltage() VGP1 failed!\n");
		return -1;
		}
	ret=regulator_enable(tpd->reg);  //enable regulator
	if (ret)
		printk(" regulator_enable() failed!\n");
}
/***************** swf add end ****************************/

/* probe function is used for matching and initializing input device */
static int /*__devinit*/ tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    TPD_DMESG("TPD probe\n");   
    
    if (client == NULL)
    {
        TPD_DMESG("i2c client is NULL\n");
        return -1;
    }
    g_I2cClient = client;
    
    MsDrvInterfaceTouchDeviceSetIicDataRate(g_I2cClient, 100000); // 100 KHZ
    power_on_self_mstar();//swf 20150701 add for power on
    MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);

    tpd_load_status = 1;

    TPD_DMESG("TPD probe done\n");
   	#if 0///def SLT_DEVINFO_CTP
    	tp_devinfo_init();
		proc_create("tp_info", 0444, NULL, &ctp_info_proc_fops);
		proc_create("ctp_vender", 0444, NULL, &ctp_vendor_proc_fops);
	#endif
 #ifdef SLT_DEVINFO_CTP
 #ifdef LCT_ADD_TP_VERSION
	if(proc_create(CTP_PROC_FILE, 0444, NULL, &g_ctp_proc)== NULL)
	{
		printk("create_proc_entry msg2238 failed\n");
	}
	if(proc_create(CTP_PROC_FILE_VENDER, 0444, NULL, &ctp_vendor_proc_fops)== NULL)
	{
		printk("create_proc_entry msg2238 ctp_vendor_proc_fops  failed\n");
	}
#endif   

	switch(msg2238_ctp_module_name)
	{
		case 0:
			devinfo_ctp_regchar("msg2238", "dj_kuantie", CFG_VER_buff, DEVINFO_USED);
		break;		
		case 1: 
			devinfo_ctp_regchar("msg2238", "dj_quantie", CFG_VER_buff, DEVINFO_USED);
		break;
		case 2: 
			devinfo_ctp_regchar("msg2238", "txd_quantie", CFG_VER_buff, DEVINFO_USED);
		break;	
		case 3: 
			devinfo_ctp_regchar("msg2238", "txd_kuantie", CFG_VER_buff, DEVINFO_USED);
		break;			
		default:
			devinfo_ctp_regchar("unknown", "unknown", "unknown", DEVINFO_USED);
		break;
	}
#endif

    return TPD_OK;   
}

static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info) 
{
    strcpy(info->type, TPD_DEVICE);    
//    strcpy(info->type, MSG_TP_IC_NAME);
    
    return TPD_OK;
}

static int /*__devexit*/ tpd_remove(struct i2c_client *client)
{   
    TPD_DEBUG("TPD removed\n");
    
    MsDrvInterfaceTouchDeviceRemove(client);
    
    return TPD_OK;
}

static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(MSG_TP_IC_NAME, (0x4C>>1))};

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id tpd_device_id[] =
{
    {MSG_TP_IC_NAME, 0},
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, tpd_device_id);

static struct i2c_driver tpd_i2c_driver = {
    .driver = {
        .name = MSG_TP_IC_NAME,
    },
    .probe = tpd_probe,
    //.remove = __devexit_p(tpd_remove),
    .remove = tpd_remove,
    .id_table = tpd_device_id,
    .detect = tpd_detect,
};

static int tpd_local_init(void)
{  
    TPD_DMESG("TPD init device driver (Built %s @ %s)\n", __DATE__, __TIME__);
/*
    // Software reset mode will be treated as normal boot
    boot_mode = get_boot_mode();
    if (boot_mode == 3) 
    {
        boot_mode = NORMAL_BOOT;    
    }
*/
    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        TPD_DMESG("unable to add i2c driver.\n");
         
        return -1;
    }
    
    if (tpd_load_status == 0) 
    {
        TPD_DMESG("add error touch panel driver.\n");

        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#ifdef CONFIG_TP_HAVE_KEY
#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE     
    // initialize tpd button data
    tpd_button_setting(4, g_TpVirtualKey, g_TpVirtualKeyDimLocal); //MAX_KEY_NUM
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE  
#endif //CONFIG_TP_HAVE_KEY  

/*
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);    
#endif  
*/
    TPD_DMESG("TPD init done %s, %d\n", __FUNCTION__, __LINE__);  
        
    return TPD_OK; 
}

static void tpd_resume(struct early_suspend *h)
{
    TPD_DMESG("TPD wake up\n");
    
    MsDrvInterfaceTouchDeviceResume(h);
    
    TPD_DMESG("TPD wake up done\n");
}

static void tpd_suspend(struct early_suspend *h)
{
    TPD_DMESG("TPD enter sleep\n");

    MsDrvInterfaceTouchDeviceSuspend(h);

    TPD_DMESG("TPD enter sleep done\n");
} 

static struct tpd_driver_t tpd_device_driver = {
     .tpd_device_name = MSG_TP_IC_NAME,
     .tpd_local_init = tpd_local_init,
     .suspend = tpd_suspend,
     .resume = tpd_resume,
#ifdef CONFIG_TP_HAVE_KEY
#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
     .tpd_have_button = 1,
#else
     .tpd_have_button = 0,
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE        
#endif //CONFIG_TP_HAVE_KEY        
};

static int __init tpd_driver_init(void) 
{
    TPD_DMESG("touch panel driver init\n");

    i2c_register_board_info(I2C_BUS_ID, &i2c_tpd, 1);
    if (tpd_driver_add(&tpd_device_driver) < 0)
    {
        TPD_DMESG("TPD add driver failed\n");
    }
     
    return 0;
}
 
static void __exit tpd_driver_exit(void) 
{
    TPD_DMESG("touch panel driver exit\n");
    
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
MODULE_LICENSE("GPL");
