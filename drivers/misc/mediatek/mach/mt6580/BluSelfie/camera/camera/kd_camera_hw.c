#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include <linux/regulator/consumer.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(PFX fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)   pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   pr_debug(PFX fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

/*
#ifndef BOOL
typedef unsigned char BOOL;
#endif
*/

struct regulator *regVCAMA = NULL;
struct regulator *regVCAMD = NULL;
struct regulator *regVCAMIO = NULL;
struct regulator *regVCAMAF = NULL;
struct regulator *regVCAMD_SUB = NULL;






/* Mark: need to verify whether ISP_MCLK1_EN is required in here //Jessy @2014/06/04*/
extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);

//extern struct platform_device *camerahw_platform_device;
extern struct device *sensor_device;
//lc mike_zhu 20150530
static MS2R_CAMERA_MODE g_eCameramode = MS2R_MAIN_CAMERA;
void MS2R_Set_CameraMode(MS2R_CAMERA_MODE mode)
{
	g_eCameramode = mode;
}
u8 MS2R_Get_CameraMode(void)
{
	return g_eCameramode;
}
//static DEFINE_SPINLOCK(camera_drv_lock);

bool _hwPowerOn(char * powerId, int powerVolt, struct regulator **regVCAM){
    bool ret = false;
	struct regulator * temp = NULL;
		PK_DBG("[_hwPowerOn]before get, powerId:%s, regVCAM: %p\n", powerId, *regVCAM );
	if(*regVCAM == NULL)
		*regVCAM = regulator_get(sensor_device,powerId);
	temp = *regVCAM;

	PK_DBG("[_hwPowerOn]after get, powerId:%s, regVCAM: %p\n", powerId, temp );


 	if(!IS_ERR(temp)){
        if(powerId != CAMERA_POWER_VCAM_IO && regulator_set_voltage(temp , powerVolt,powerVolt)!=0 ){
            PK_DBG("[_hwPowerOn]fail to regulator_set_voltage, powerVolt:%d, powerID: %s\n", powerVolt, powerId);
        }
        if(regulator_enable(temp)!= 0) {
            PK_DBG("[_hwPowerOn]fail to regulator_enable, powerVolt:%d, powerID: %s\n", powerVolt, powerId);
            //regulator_put(regVCAM);
            //regVCAM = NULL;
            return ret;
        }
        ret = true;
    }else
   		 PK_DBG("[_hwPowerOn]IS_ERR_OR_NULL regVCAM %s\n",powerId);

    return ret;
}

bool _hwPowerDown(char * powerId, struct regulator **regVCAM){
    bool ret = false;
	struct regulator * temp = *regVCAM;
    if(!IS_ERR(temp)){
		#if 1
        if(regulator_is_enabled(temp)) {
            PK_DBG("[_hwPowerDown]before disable %s is enabled\n", powerId);
        }
		#endif
		if(regulator_disable(temp)!=0)
			 PK_DBG("[_hwPowerDown]fail to regulator_disable, powerID: %s\n", powerId);
		//for SMT stage, put seems always fail?
       // regulator_put(regVCAM);
       // regVCAM = NULL;
        ret = true;
    } else {
        PK_DBG("[_hwPowerDown]%s fail to power down  due to regVCAM == NULL regVCAM 0x%p\n", powerId,temp );
    }
    return ret;
}


int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000


u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN, // The reset pin of main sensor uses GPIO10 of mt6306, please call mt6306 API to set
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for main_2 sensor
                     {  GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,   /* mode */
                        GPIO_OUT_ONE,               /* ON state */
                        GPIO_OUT_ZERO,              /* OFF state */
                        GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     }
                   };



    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
	MS2R_Set_CameraMode(MS2R_MAIN_CAMERA);
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
	MS2R_Set_CameraMode(MS2R_SUB_CAMERA);
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

    //power ON
    if (On) {
		if(pinSetIdx == 0)
			ISP_MCLK1_EN(1);
		else
			ISP_MCLK2_EN(1);


        printk("kd_camera_hw.c [PowerON]pinSetIdx:%d, currSensorName: %s On=%d SensorIdx =%d\n", pinSetIdx, currSensorName,On,SensorIdx);

        if ((currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw")))||
            (currSensorName && (0 == strcmp(currSensorName,"imx220mipiraw"))))
        {
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }


            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            //AF_VCC
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, &regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(1);

            //VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(1);

            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,&regVCAMD))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %s \n", CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            mdelay(1);

            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(2);


            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

        }
 else  if((currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW, currSensorName)))&&( 1== SensorIdx))
		{
    printk(" kd_camera_hw.c SENSOR_DRVNAME_OV5670_MIPI_RAW [PowerON]SensorIdx :%d ,pinSetIdx:%d, currSensorName: %s\n", SensorIdx,pinSetIdx, currSensorName);
			//First Power Pin low and Reset Pin Low
          ///  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
           {
		if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		}
            mdelay(50);
           // if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
               // if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		//mdelay(1);
		//if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
		}
            //AF_VCC
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, &regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_A & VCAM_IO use same source VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
            //VCAMD
            if(TRUE != _hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1200, &regVCAMD))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", SUB_CAMERA_POWER_VCAM_D);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
           /// if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		}
              //mdelay(50);
            //enable active sensor
          ///  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) 
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
		 }
		mdelay(50);
 }
 else  if((currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI542MIPI_RAW, currSensorName)))&&( 1== SensorIdx))
{
    printk(" kd_camera_hw.c SENSOR_DRVNAME_HI542MIPI_RAW [PowerON]SensorIdx :%d ,pinSetIdx:%d, currSensorName: %s\n", SensorIdx,pinSetIdx, currSensorName);
			//First Power Pin low and Reset Pin Low
    //spin_lock(&camera_drv_lock);
			
          ///  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
           {
		if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		}
            mdelay(10);
           // if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
               // if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		//mdelay(1);
		//if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(10);
		}
///NC front af en
		if(mt_set_gpio_mode(GPIO_AF_F_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_F_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_F_EN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio failed!! (CMPDN)\n");}
		   ///af en
		if(mt_set_gpio_mode(GPIO_AF_R_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_R_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_R_EN,GPIO_OUT_ONE)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio failed!! (CMPDN)\n");}
		
            //AF_VCC
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, &regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
            //VCAM_A & VCAM_IO use same source VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
            //VCAMD
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200, &regVCAMD))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", SUB_CAMERA_POWER_VCAM_D);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
           /// if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		}
              //mdelay(50);
            //enable active sensor
          ///  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) 
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
		 }
    //spin_unlock(&camera_drv_lock);

		mdelay(50);
 }
 else  if((currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI542SUBMIPI_RAW, currSensorName)))&&( 2== SensorIdx))
 {
    printk(" kd_camera_hw.c SENSOR_DRVNAME_HI542SUBMIPI_RAW [PowerON]SensorIdx :%d ,pinSetIdx:%d, currSensorName: %s\n", SensorIdx,pinSetIdx, currSensorName);
			//First Power Pin low and Reset Pin Low
   /// spin_lock(&camera_drv_lock);
           /// if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
           {
		if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		//if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		//mdelay(1);
		//if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		mdelay(10);
		}
           /// if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
               // if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		//mdelay(1);
		//if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(10);
		}

  ///nc rear af en
		if(mt_set_gpio_mode(GPIO_AF_R_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_R_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_R_EN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio failed!! (CMPDN)\n");}
		   
///af en
		if(mt_set_gpio_mode(GPIO_AF_F_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_F_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_F_EN,GPIO_OUT_ONE)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio failed!! (CMPDN)\n");}
		
            //AF_VCC
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, &regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
            //VCAM_A & VCAM_IO use same source VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
            //VCAMD
            if(TRUE != _hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1200, &regVCAMD_SUB))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", SUB_CAMERA_POWER_VCAM_D);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(10);
         ///   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
		{
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		 }
              //mdelay(50);
            //enable active sensor
          //  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) 
		{
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
		mdelay(50);
		if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
	       }
    ///spin_unlock(&camera_drv_lock);
		mdelay(50);
 }
 else  if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2680_MIPI_RAW, currSensorName))&&(2 == SensorIdx))
{
   printk("mike_zhu kd_camera_hw.c  SENSOR_DRVNAME_OV2680_MIPI_RAW  \n");
    printk("mike_zhu kd_camera_hw.c OV2680 [PowerON]SensorIdx :%d ,pinSetIdx:%d, currSensorName: %s\n", SensorIdx,pinSetIdx, currSensorName);
if (pinSetIdx ==1)
{
			//First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
           {
		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		//if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		//mdelay(1);
		//if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		//mdelay(1);
		}
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
               // if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		//mdelay(1);
		//if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(1);
		}
            //VCAM_A & VCAM_IO use same source VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
            //VCAMD
            if(TRUE != _hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1800, &regVCAMD))
            {
                printk("mike_zhu [CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", SUB_CAMERA_POWER_VCAM_D);
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
		{
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		}
              //mdelay(50);
            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) 
		{
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
	       }
		mdelay(5);
}
 }	
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5648_MIPI_RAW, currSensorName)))
        {
        #if 0
            mt_set_gpio_mode(GPIO_SPI_MOSI_PIN,GPIO_MODE_00);
            mt_set_gpio_dir(GPIO_SPI_MOSI_PIN,GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_SPI_MOSI_PIN,GPIO_OUT_ONE);
        #endif
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }


            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(1);

            //VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(1);

            if(TRUE != _hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1500,&regVCAMD))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %s \n", SUB_CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            mdelay(5);

            //AF_VCC
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,&regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }


            mdelay(1);


            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }


            mdelay(2);


            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}

            }

            mdelay(20);
        }
        else  if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW, currSensorName)))
        {
        #if 0
            mt_set_gpio_mode(GPIO_SPI_MOSI_PIN,GPIO_MODE_00);
            mt_set_gpio_dir(GPIO_SPI_MOSI_PIN,GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_SPI_MOSI_PIN,GPIO_OUT_ONE);
        #endif
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            mdelay(50);

            //VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(10);

            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(10);

            if(TRUE != _hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1500,&regVCAMD))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %s \n", SUB_CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            mdelay(10);

            //AF_VCC
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,&regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }


            mdelay(50);

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
                mdelay(5);
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}

            }
            mdelay(5);
            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
                mdelay(5);
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            mdelay(5);
        }
        else  if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC0310_YUV, currSensorName)))
        {
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            mdelay(50);
            //VCAM_A & VCAM_IO use same source VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(10);

            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(10);

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
            mdelay(5);
            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                mdelay(5);
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
                mdelay(5);
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}
                mdelay(5);
            }
            mdelay(5);
        }
        else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4ECGX_MIPI_YUV, currSensorName)))
        {
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            //VCAM_D

                if(pinSetIdx == 1 && TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,&regVCAMD))
                {
                     PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                     goto _kdCISModulePowerOn_exit_;
                }
 //VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
             mdelay(5);

            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

	 mdelay(5);


             //AF_VCC
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,&regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(5);

            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            mdelay(1);

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
			 mdelay(5);
        }
        else
        	{
printk("kd_camera_hw power on-------none------- \n");
		}
	#if 0
        {
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,&regVCAMA))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_D
            if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K2P8_MIPI_RAW, currSensorName)))
            {
                if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,&regVCAMD))
                {
                     PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                     goto _kdCISModulePowerOn_exit_;
                }
            }
			else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX179_MIPI_RAW, currSensorName )||
				0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW, currSensorName ))){
					if(pinSetIdx == 0 && TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,&regVCAMD)) {
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						goto _kdCISModulePowerOn_exit_;
					} else if (pinSetIdx == 1 && TRUE != _hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1200,&regVCAMD)) {
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						goto _kdCISModulePowerOn_exit_;
					}
			}
            else if(currSensorName &&(0 == strcmp(SENSOR_DRVNAME_IMX219_MIPI_RAW, currSensorName )))
            {
                if(pinSetIdx == 0 && TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,&regVCAMD))
                {
                     PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                     goto _kdCISModulePowerOn_exit_;
                }
            }
            else { // Main VCAMD max 1.5V
                PK_DBG("[CAMERA SENSOR] before vcamd power on\n");
                if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,&regVCAMD))
                {
                     PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                     goto _kdCISModulePowerOn_exit_;
                }
				PK_DBG("[CAMERA SENSOR] after before vcamd power on regVCAMD=%p\n",&regVCAMD);
            }


             //AF_VCC
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,&regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(5);

            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            mdelay(1);

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
        }
	#endif
    }
    else {//power OFF

        PK_DBG("[PowerOFF]pinSetIdx:%d\n", pinSetIdx);
        if(pinSetIdx == 0)
			ISP_MCLK1_EN(0);
		else
			ISP_MCLK2_EN(0);

        if ((currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw")))||
            (currSensorName && (0 == strcmp(currSensorName,"imx220mipiraw")))) {


            //Set Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            //Set Reset Pin Low
             if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            //AF_VCC
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF,&regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_D,&regVCAMD))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %s \n",CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

        }
 else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI542MIPI_RAW, currSensorName)))&&(1 == SensorIdx))
		{
	#if 1
        printk("SENSOR_DRVNAME_HI542MIPI_RAW  PowerOFF]pinSetIdx:%d\n", pinSetIdx);
    ///spin_lock(&camera_drv_lock);

            //Set Power Pin low and Reset Pin Low
          //  if (GPIO_CAMERA_INVALID != pinSet[0][IDX_PS_CMPDN])
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		mdelay(50);
			}
           /// if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
		   }

		   ///af en 
		if(mt_set_gpio_mode(GPIO_AF_F_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_F_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_F_EN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio failed!! (CMPDN)\n");}
   
		if(mt_set_gpio_mode(GPIO_AF_R_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_R_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_R_EN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio failed!! (CMPDN)\n");}
		
            //AF_VCC
		if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF,&regVCAMAF)) {
		PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_AF), power id= %s \n", CAMERA_POWER_VCAM_AF);
		goto _kdCISModulePowerOn_exit_;
		}
		mdelay(50);
            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(50);
            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(50);
            //VCAM_D
            if(TRUE != _hwPowerDown(SUB_CAMERA_POWER_VCAM_D, &regVCAMD)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
    ///spin_unlock(&camera_drv_lock);
		mdelay(50);
#endif
		}
 else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW, currSensorName)))&&(1 == SensorIdx))
{
	#if 1
        printk("SENSOR_DRVNAME_OV5670_MIPI_RAW  PowerOFF]pinSetIdx:%d\n", pinSetIdx);
            //Set Power Pin low and Reset Pin Low
    ///spin_lock(&camera_drv_lock);
          //  if (GPIO_CAMERA_INVALID != pinSet[0][IDX_PS_CMPDN])
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		mdelay(50);
			}
           /// if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(50);
		   }
            //AF_VCC
		if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF,&regVCAMAF)) {
		PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_AF), power id= %s \n", CAMERA_POWER_VCAM_AF);
		goto _kdCISModulePowerOn_exit_;
		}
		mdelay(50);
            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(50);
            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(50);
            //VCAM_D
            if(TRUE != _hwPowerDown(SUB_CAMERA_POWER_VCAM_D, &regVCAMD)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
   /// spin_unlock(&camera_drv_lock);  
		mdelay(50);
#endif
		}
  else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI542SUBMIPI_RAW, currSensorName)))&&(2 == SensorIdx))
	{
	#if 1
        printk("SENSOR_DRVNAME_HI542SUBMIPI_RAW  PowerOFF]pinSetIdx:%d\n", pinSetIdx);

            //Set Power Pin low and Reset Pin Low
          ///  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
		{
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		mdelay(10);
		}
            ///if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(10);
			}
///af en
		if(mt_set_gpio_mode(GPIO_AF_F_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_F_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_F_EN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio failed!! (CMPDN)\n");}

		if(mt_set_gpio_mode(GPIO_AF_R_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_R_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_R_EN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA GPIO_AF_R_EN] set gpio failed!! (CMPDN)\n");}
		   
            //AF_VCC
		if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF,&regVCAMAF)) {
		PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_AF), power id= %s \n", CAMERA_POWER_VCAM_AF);
		goto _kdCISModulePowerOn_exit_;
		}	
		mdelay(10);
            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(10);
            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(10);
            //VCAM_D
            if(TRUE != _hwPowerDown(SUB_CAMERA_POWER_VCAM_D, &regVCAMD)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(50);
#endif
		}
 else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2680_MIPI_RAW, currSensorName)))&&(2 == SensorIdx))
	{
	#if 1
        printk("mike_zhu [kd_camera_hw.c OV2680 PowerOFF]pinSetIdx:%d\n", pinSetIdx);

            //Set Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
		{
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_D
            if(TRUE != _hwPowerDown(SUB_CAMERA_POWER_VCAM_D, &regVCAMD)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
	mdelay(100);

#endif
		}
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5648_MIPI_RAW, currSensorName)))
        {
        #if 0
            mt_set_gpio_out(GPIO_SPI_MOSI_PIN,GPIO_OUT_ZERO);
        #endif
            //Set Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }

            if(TRUE != _hwPowerDown(SUB_CAMERA_POWER_VCAM_D,&regVCAMD))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %s \n",SUB_CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //AF_VCC
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF,&regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

        }        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW, currSensorName))){
#if 0
            mt_set_gpio_out(GPIO_SPI_MOSI_PIN,GPIO_OUT_ZERO);
#endif
            //Set Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }


            if(TRUE != _hwPowerDown(SUB_CAMERA_POWER_VCAM_D,&regVCAMD))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %s \n",SUB_CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //AF_VCC
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF,&regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

	      } else  if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC0310_YUV, currSensorName))){
            //Set Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
		}
		  else 
		{

printk("kd_camera_hw power off ---------------------no -- \n");
	#if 0///mike_zhu for_ctm_luodian 20150716
        printk("[  PowerOFF]pinSetIdx:%d\n", pinSetIdx);

            //Set Power Pin low and Reset Pin Low

		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		//mdelay(50);
			}
            ///if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(5);
			}
          ///  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
		{
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
		mdelay(5);
			}
            ///if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
                if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
		mdelay(5);
			}
			///af en
		if(mt_set_gpio_mode(GPIO_AF_F_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_F_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_F_EN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio failed!! (CMPDN)\n");}
		if(mt_set_gpio_mode(GPIO_AF_R_EN,GPIO_MODE_00)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio mode failed!! (CMPDN)\n");}
		if(mt_set_gpio_dir(GPIO_AF_R_EN,GPIO_DIR_OUT)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio dir failed!! (CMPDN)\n");}
		if(mt_set_gpio_out(GPIO_AF_R_EN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA GPIO_AF_F_EN] set gpio failed!! (CMPDN)\n");}
		
            //AF_VCC
		if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF,&regVCAMAF)) {
		PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_AF), power id= %s \n", CAMERA_POWER_VCAM_AF);
		goto _kdCISModulePowerOn_exit_;
		}	
		mdelay(5);
            //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(5);
            //VCAM_IO
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
		mdelay(5);
            //VCAM_D
            if(TRUE != _hwPowerDown(SUB_CAMERA_POWER_VCAM_D, &regVCAMD)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
#endif

		}
#if 0
		{
			//Set Power Pin low and Reset Pin Low
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}
	        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			}
			if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX219_MIPI_RAW, currSensorName))){
				if(pinSetIdx == 0 && TRUE != _hwPowerDown(CAMERA_POWER_VCAM_D, &regVCAMD))
				{
					PK_DBG("[CAMERA SENSOR] main imx220 Fail to OFF core power (VCAM_D), power id = %s \n",CAMERA_POWER_VCAM_D);
					goto _kdCISModulePowerOn_exit_;
				}
			}
			else if(currSensorName &&(0 == strcmp(SENSOR_DRVNAME_IMX179_MIPI_RAW, currSensorName )||
				0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW, currSensorName ))) {
				if(pinSetIdx == 0 && TRUE != _hwPowerDown(CAMERA_POWER_VCAM_D,&regVCAMD)){
					PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
					goto _kdCISModulePowerOn_exit_;
				} else if (pinSetIdx == 1 && TRUE != _hwPowerDown(SUB_CAMERA_POWER_VCAM_D,&regVCAMD)){
					PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
					goto _kdCISModulePowerOn_exit_;
				}
			}
			else  if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4ECGX_MIPI_YUV, currSensorName))){
				if(pinSetIdx == 1 && TRUE != _hwPowerDown(CAMERA_POWER_VCAM_D, &regVCAMD)) {
					PK_DBG("[CAMERA SENSOR] sub imx219 Fail to OFF core power (VCAM_D), power id = %s \n",CAMERA_POWER_VCAM_D);
					goto _kdCISModulePowerOn_exit_;
				}
			} else {
			    if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_D,&regVCAMD))
				{
					PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %s \n",CAMERA_POWER_VCAM_D);
					goto _kdCISModulePowerOn_exit_;
				}
			}
			//VCAM_A
			if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A,&regVCAMA)){
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= %s \n", CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}
			//VCAM_IO
			if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)){
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
	            goto _kdCISModulePowerOn_exit_;
			}
			//AF_VCC
			if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF,&regVCAMAF)){
				PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
				goto _kdCISModulePowerOn_exit_;
			}
		}
		#endif

    }

    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);

//!--
//


