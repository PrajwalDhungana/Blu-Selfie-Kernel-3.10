
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif
//lc mike_zhu 20150708 for front led
#define GPIO_FRONT_FLASHLIGHT_EN   GPIO_FLASHLIGHT_F_EN
int SUB_FL_Torch_Mode_FLAG =0;
static DEFINE_SPINLOCK(sub_g_strobeSMPLock); /* cotta-- SMP proection */
static u32 sub_strobe_Res = 0;
static u32 sub_strobe_Timeus = 0;
static BOOL sub_g_strobe_On = 0;
static int sub_g_duty=-1;
static int sub_g_timeOutTimeMs=0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(sub_g_strobeSem);
#else
static DECLARE_MUTEX(sub_g_strobeSem);
#endif
static struct work_struct sub_workTimeOut;
static void sub_work_timeOutFunc(struct work_struct *data);
int sub_FL_Enable(void)
{
printk("sub_strobe.c Sub_FL_Enable\n");
mt_set_gpio_out(GPIO_FRONT_FLASHLIGHT_EN,GPIO_OUT_ONE);
    return 0;
}
int sub_FL_Disable(void)
{
printk(" sub_strobe.c Sub_FL_Disable\n");
	mt_set_gpio_out(GPIO_FRONT_FLASHLIGHT_EN,GPIO_OUT_ZERO);
    return 0;
}


int sub_FL_Init(void)
{
printk(" \n");
printk("sub_strobe.c Sub_FL_Init\n");
    if(mt_set_gpio_mode(GPIO_FRONT_FLASHLIGHT_EN,GPIO_MODE_00)){printk("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_FRONT_FLASHLIGHT_EN,GPIO_DIR_OUT)){printk("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_FRONT_FLASHLIGHT_EN,GPIO_OUT_ZERO)){printk("[constant_flashlight] set gpio failed!! \n");}
    INIT_WORK(&sub_workTimeOut, sub_work_timeOutFunc);
    return 0;
}

int sub_FL_Uninit(void)
{
printk(" sub_strobe.c Sub_FL_Uninit\n");
    if(mt_set_gpio_mode(GPIO_FRONT_FLASHLIGHT_EN,GPIO_MODE_00)){printk("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_FRONT_FLASHLIGHT_EN,GPIO_DIR_OUT)){printk("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_FRONT_FLASHLIGHT_EN,GPIO_OUT_ZERO)){printk("[constant_flashlight] set gpio failed!! \n");}
	return 0;
}

void sub_FL_Flash_Mode(void)
{
printk(" sub_strobe.c Sub_FL_Flash_Mode\n");
    if(mt_set_gpio_mode(GPIO_FRONT_FLASHLIGHT_EN,GPIO_MODE_00)){printk("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_FRONT_FLASHLIGHT_EN,GPIO_DIR_OUT)){printk("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_FRONT_FLASHLIGHT_EN,GPIO_OUT_ONE)){printk("[constant_flashlight] set gpio failed!! \n");}
}
void sub_FL_Torch_Mode(void)
{
printk(" leds_strobe.c Sub_FL_Torch_Mode\n");
    if(mt_set_gpio_mode(GPIO_FRONT_FLASHLIGHT_EN,GPIO_MODE_00)){printk("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_FRONT_FLASHLIGHT_EN,GPIO_DIR_OUT)){printk("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_FRONT_FLASHLIGHT_EN,GPIO_OUT_ONE)){printk("[constant_flashlight] set gpio failed!! \n");}
}
static void sub_work_timeOutFunc(struct work_struct *data)
{
   // FL_Disable();
    printk("sub_work_timeOutFunc ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}

enum hrtimer_restart sub_ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&sub_workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer sub_g_timeOutTimer;
void sub_timerInit(void)
{
	sub_g_timeOutTimeMs=99999;///1000; //1s
	hrtimer_init( &sub_g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	sub_g_timeOutTimer.function=sub_ledTimeOutCallback;

}


static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	//	PK_DBG("sub dummy ioctl");
	//printk("sub_strobe_ioctl()  ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%ul\n", ior_shift, iow_shift, iowr_shift, arg);
	printk("sub_strobe_ioctl cmd=%d \n",cmd);
	switch(cmd)
    {
		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			printk("sub_strobe_ioctl FLASH_IOC_SET_TIME_OUT_TIME_MS\n");
			sub_g_timeOutTimeMs=arg;
		break;
    	case FLASH_IOC_SET_DUTY :
    		printk("sub_strobe_ioctl FLASHLIGHT_DUTY\n");
			if (arg != 1)
				{
				sub_FL_Flash_Mode();
				SUB_FL_Torch_Mode_FLAG =0;
				}
			else
				{
				SUB_FL_Torch_Mode_FLAG=1;
				sub_FL_Torch_Mode();
    		//            FL_dim_duty(arg);
				}
    		break;
    	case FLASH_IOC_SET_STEP:
    		printk("sub_strobe_ioctl FLASH_IOC_SET_STEP: \n");
    		break;
    	case FLASH_IOC_SET_ONOFF :
    		printk("sub_strobe_ioctl FLASHLIGHT_ONOFF:\n");
		//if(FL_Torch_Mode_FLAG !=1)
		//{
    		if(arg==1)
    		{
		if(sub_g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
			ktime = ktime_set( 0, sub_g_timeOutTimeMs*1000000 );
			hrtimer_start( &sub_g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			sub_FL_Enable();
    		}
    		else
    		{
    			sub_FL_Disable();
			hrtimer_cancel( &sub_g_timeOutTimer );
    		}
		//}
    		break;
		default :
    		printk("sub_strobe_ioctl  No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
    int i4RetValue = 0;
    ///PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
    PK_DBG("sub_strobe_open  sub dummy open");
	if (0 == sub_strobe_Res)
	{
	    sub_FL_Init();
		sub_timerInit();
	}
////	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&sub_g_strobeSMPLock);


    if(sub_strobe_Res)
    {
        PK_ERR(" sub_strobe_open busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        sub_strobe_Res += 1;
    }
    spin_unlock_irq(&sub_g_strobeSMPLock);

    return i4RetValue;


}

static int sub_strobe_release(void *pArg)
{
    printk("sub dummy release");

    if (sub_strobe_Res)
    {
        spin_lock_irq(&sub_g_strobeSMPLock);

        sub_strobe_Res = 0;
        sub_strobe_Timeus = 0;

        /* LED On Status */
        sub_g_strobe_On = FALSE;

        spin_unlock_irq(&sub_g_strobeSMPLock);

    	sub_FL_Uninit();
    }

    printk("sub_strobe_release Done\n");

    return 0;

}

FLASHLIGHT_FUNCTION_STRUCT	subStrobeFunc=
{
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &subStrobeFunc;
    }
    return 0;
}





