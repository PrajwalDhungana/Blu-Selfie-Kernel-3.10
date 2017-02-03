#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#include <platform/upmu_common.h>
#include <platform/upmu_hw.h>
#include <platform/mt_i2c.h> 
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/string.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include "ddp_hal.h"
#endif

#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
///extern void upmu_set_rg_vgp2_ndis_en(kal_uint32 val);
#ifndef BUILD_LK
///extern  void power_on_self(void);
extern void DSI_clk_HS_mode(DISP_MODULE_ENUM module, void* cmdq, bool enter);
//void power_on_0(void);

#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
// pixel
#define FRAME_WIDTH  			(480)
#define FRAME_HEIGHT 			(854)
// physical dimension
#define PHYSICAL_WIDTH        (68)
#define PHYSICAL_HIGHT         (121)


#define LCM_ID_LI9806E_A       0x98
#define LCM_ID_LI9806E_B       0x06
#define LCM_ID_LI9806E_C       0x04

#define LCM_DSI_CMD_MODE		0

#define REGFLAG_DELAY 0xFE
#define REGFLAG_END_OF_TABLE 0xFFF // END OF REGISTERS MARKER
///lc mike_zhu 20150511 
///extern kal_uint16 mt6350_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);
/*
DSV power +5V,-5v
*/
#if 0///ndef GPIO_DSV_AVDD_EN
#define GPIO_DSV_AVDD_EN (GPIO41 | 0x80000000)
#define GPIO_DSV_AVDD_EN_M_GPIO GPIO_MODE_00
#define GPIO_DSV_AVDD_EN_M_KROW GPIO_MODE_06
#define GPIO_DSV_AVDD_EN_M_PWM GPIO_MODE_05
#endif

#if 0///ndef GPIO_DSV_AVEE_EN
#define GPIO_DSV_AVEE_EN (GPIO42 | 0x80000000)
#define GPIO_DSV_AVEE_EN_M_GPIO GPIO_MODE_00
#define GPIO_DSV_AVEE_EN_M_KROW GPIO_MODE_06
#define GPIO_DSV_AVEE_EN_M_PWM GPIO_MODE_05
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V3(para_tbl, size, force_update)   	lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static unsigned int need_set_lcm_addr = 1;

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static struct LCM_setting_table lcm_initialization_setting[] = 
{
//lc mike_zhu 20150615
{0XFF,5, {0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 1
{0x08,1, {0x10}},                // output SDA
{0x21,1, {0x01}},                // DE = 1 Active  Display Function Control
{0x30,1, {0x01}},                // 480 X 854
{0x31,1, {0x00}},              //  inversion
{0x50,1, {0xA0}},               // VGMP
{0x51,1, {0xA0}},                 // VGMN
{0x60,1, {0x07}},                // SDTI
{0x61,1, {0x00}},               // CRTI
{0x62,1, {0x07}},                // EQTI
{0x63,1, {0x00}},               // PCTI
{0x40,1, {0x15}},                // BT 
{0x41,1, {0x55}},                 // DDVDH/DDVDL  Clamp
{0x42,1, {0x03}},                // VGH/VGL 
{0x43,1, {0x0A}},                 // VGH Clamp       
{0x44,1, {0x06}},                 // VGL Clamp  
{0x46,1, {0x55}},                
{0x53,1, {0x0C}},                //Flicker
{0x55,1, {0x0C}},               
{0x57,1, {0x50}},
//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
{0XFF,5, {0xFF,0x98,0x06,0x04,0x01}},      // Change to Page 1
{0xA0,1, {0x0A}}, // Gamma 255 
{0xA1,1, {0x18}},   // Gamma 251
{0xA2,1, {0x20}},   // Gamma 247
{0xA3,1, {0x10}},   // Gamma 239
{0xA4,1, {0x08}},   // Gamma 231
{0xA5,1, {0x0E}},   // Gamma 203
{0xA6,1, {0x08}},   // Gamma 175
{0xA7,1, {0x05}},   // Gamma 147
{0xA8,1, {0x08}},   // Gamma 108
{0xA9,1, {0x0C}},   // Gamma 80
{0xAA,1, {0x0F}},   // Gamma 52
{0xAB,1, {0x08}},   // Gamma 24
{0xAC,1, {0x11}},   // Gamma 16
{0xAD,1, {0x14}},  // Gamma 8
{0xAE,1, {0x0E}},   // Gamma 4
{0xAF,1, {0x03}},   // Gamma 0
///==1, {============Nagitive
{0xC0,1, {0x0F}},   // Gamma 0 
{0xC1,1, {0x18}},   // Gamma 4
{0xC2,1, {0x20}},   // Gamma 8
{0xC3,1, {0x10}},   // Gamma 16
{0xC4,1, {0x08}},   // Gamma 24
{0xC5,1, {0x0E}},   // Gamma 52
{0xC6,1, {0x08}},   // Gamma 80
{0xC7,1, {0x05}},   // Gamma 108
{0xC8,1, {0x08}},  // Gamma 147
{0xC9,1, {0x0C}},   // Gamma 175
{0xCA,1, {0x0F}},   // Gamma 203
{0xCB,1, {0x08}},   // Gamma 231
{0xCC,1, {0x11}},   // Gamma 239
{0xCD,1, {0x14}},   // Gamma 247
{0xCE,1, {0x0E}},   // Gamma 251
{0xCF,1, {0x00}},   // Gamma 255
// Pag1, {e 6 Command 
{0XFF,5, {0xFF,0x98,0x06,0x04,0x06}},      // Change to Page 6
{0x00,1, {0x21}}, 
{0x01,1, {0x06}}, 
{0x02,1, {0xA0}},     
{0x03,1, {0x02}}, 
{0x04,1, {0x01}}, 
{0x05,1, {0x01}}, 
{0x06,1, {0x80}},     
{0x07,1, {0x03}},   //04
{0x08,1, {0x00}}, 
{0x09,1, {0x80}},     
{0x0A,1, {0x00}},     
{0x0B,1, {0x00}},     
{0x0C,1, {0x20}}, 
{0x0D,1, {0x20}}, 
{0x0E,1, {0x09}}, 
{0x0F,1, {0x00}}, 
{0x10,1, {0xFF}}, 
{0x11,1, {0xE0}}, 
{0x12,1, {0x00}}, 
{0x13,1, {0x00}}, 
{0x14,1, {0x00}}, 
{0x15,1, {0xC0}}, 
{0x16,1, {0x08}}, 
{0x17,1, {0x00}}, 
{0x18,1, {0x00}}, 
{0x19,1, {0x00}}, 
{0x1A,1, {0x00}}, 
{0x1B,1, {0x00}},                               
{0x1C,1, {0x00}}, 
{0x1D,1, {0x00}}, 
{0x20,1, {0x01}}, 
{0x21,1, {0x23}}, 
{0x22,1, {0x45}}, 
{0x23,1, {0x67}}, 
{0x24,1, {0x01}}, 
{0x25,1, {0x23}}, 
{0x26,1, {0x45}}, 
{0x27,1, {0x67}},                              
{0x30,1, {0x12}}, 
{0x31,1, {0x22}}, 
{0x32,1, {0x22}}, 
{0x33,1, {0x22}}, 
{0x34,1, {0x87}}, 
{0x35,1, {0x96}}, 
{0x36,1, {0xAA}}, 
{0x37,1, {0xDB}}, 
{0x38,1, {0xCC}}, 
{0x39,1, {0xBD}}, 
{0x3A,1, {0x78}}, 
{0x3B,1, {0x69}}, 
{0x3C,1, {0x22}}, 
{0x3D,1, {0x22}}, 
{0x3E,1, {0x22}}, 
{0x3F,1, {0x22}}, 
{0x40,1, {0x22}}, 
{0x54,1, {0x13}},                  
//***1, {*************************************************************************//
{0XFF,5, {0xFF,0x98,0x06,0x04,0x07}},      // Change to Page 7
{0x02,1, {0x77}},           
{0x18,1, {0x1D}},         
{0xE1,1, {0x79}},                                 
{0x17,1, {0x22}},              
{0xB3,1, {0x10}}, 
//***1, {*************************************************************************//
{0XFF, 5, {0xFF,0x98,0x06,0x04,0x00}},      // Change to Page 0
{0x11, 1, {00}},
{REGFLAG_DELAY, 120, {}},
{0x29, 1, {00}},
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {
		unsigned cmd;
		
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
	LCM_PRINT("[LCD] push_table \n");
}
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy((void*)&lcm_util, (void*)util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params) 
#if 1///lc mike_zhu 20150526 
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
//add by zwp
#ifdef SLT_DEVINFO_LCM
		params->module="ilitek";
		params->vendor="DJ";
		params->ic="ili9806e";
		params->version="DJN_15_22251_55221";
		params->info="480*854";
#endif
		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;//LCM_TWO_LANE
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 4;	//5;
		params->dsi.vertical_backporch					= 20;	///5;
		params->dsi.vertical_frontporch					= 20;	///	5;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 120;
		params->dsi.horizontal_frontporch				= 120;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.PLL_CLOCK=239;	////239 241 225 215  200 180
		
		///params->dsi.dsi_clock=;
		params->dsi.ssc_disable=0;
		params->dsi.ssc_range=4;
		//unsigned int		dsi_clock;
		// unsigned int		ssc_disable;
		//unsigned int		ssc_range;
		// Bit rate calculation
//		params->dsi.pll_div1=34;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
//		params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
		///params->dsi.clk_lp_per_line_enable = 0;
#if 1///mike_zhu 20150605
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
#endif
}
#else
{ 
	memset(params, 0, sizeof(LCM_PARAMS)); 

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

   	//params->physical_width=PHYSICAL_WIDTH;
   	//params->physical_height=PHYSICAL_HIGHT;

	// enable tearing-free
	params->dbi.te_mode 				=LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine. 

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;	
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST; 
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB; 
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888; 

	// Highly depends on LCD driver capability. 
	params->dsi.packet_size = 256; 
	// Video mode setting 
	params->dsi.intermediat_buffer_num = 2; 
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888; 

	params->dsi.vertical_sync_active = 2; 
	params->dsi.vertical_backporch = 20; 
	params->dsi.vertical_frontporch = 20; 
	params->dsi.vertical_active_line = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 60;
	params->dsi.horizontal_frontporch				= 200;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

	// Bit rate calculation
	//params->dsi.pll_div1=35;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)

	/* ESD or noise interference recovery For video mode LCM only. */
	// Send TE packet to LCM in a period of n frames and check the response.
	//params->dsi.lcm_int_te_monitor = FALSE;
	//params->dsi.lcm_int_te_period = 1;		// Unit : frames

	// Need longer FP for more opportunity to do int. TE monitor applicably.
	//if(params->dsi.lcm_int_te_monitor)
	//	params->dsi.vertical_frontporch *= 2;

	// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.)
	//params->dsi.lcm_ext_te_monitor = FALSE;
	// Non-continuous clock
	//params->dsi.noncont_clock = TRUE;
	//params->dsi.noncont_clock_period = 2;	// Unit : frames

	// DSI MIPI Spec parameters setting
	/*params->dsi.HS_TRAIL = 6;
	params->dsi.HS_ZERO = 9;
	params->dsi.HS_PRPR = 5;
	params->dsi.LPX = 4;
	params->dsi.TA_SACK = 1;
	params->dsi.TA_GET = 20;
	params->dsi.TA_SURE = 6;
	params->dsi.TA_GO = 16;
	params->dsi.CLK_TRAIL = 5;
	params->dsi.CLK_ZERO = 18;
	params->dsi.LPX_WAIT = 1;
	params->dsi.CONT_DET = 0;
	params->dsi.CLK_HS_PRPR = 4;*/
	// Bit rate calculation
	params->dsi.PLL_CLOCK = 241;

	//LCM_PRINT("[LCD] lcm_get_params \n");

}
#endif
static void init_lcm_registers(void)
{
push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}
#if 0//mike_zhu
void ldo_2v8vgp2_on(void)	//mike_zhu 
{
#ifdef BUILD_LK
	///pmic_set_register_value(PMIC_RG_VGP2_EN,1);
#else
	printk("ili9806e_dsi_vdo_fwvga_dj.c ldo_2v8vgp2_on\n");
	power_on_self();
	//hwPowerOn(MT6325_POWER_LDO_VGP2, VOL_2800, "2V8_LCD_VCC_MTK_S");
#endif
}
void ldo_2v8vgp2_off(void)	
{
#ifdef BUILD_LK
//	pmic_set_register_value(PMIC_RG_VGP2_EN,0);
#else
printk("ili9806e_dsi_vdo_fwvga_dj.c ldo_2v8vgp2_off\n");
//power_on_0();
///hwPowerDown(MT6325_POWER_LDO_VGP2, "2V8_LCD_VCC_MTK_S");      
#endif
}
#endif
static void reset_lcd_module(unsigned char reset)
{
	mt_set_gpio_mode(GPIO_LCM_RST, GPIO_LCM_RST_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCM_RST, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);

   if(reset){
   	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);	
   }else{
   	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
   }
}  
static void lcm_init(void)
{
#if defined(BUILD_LK) 	
printf("ili9806e_dsi_vdo_fwvga_dj.c  lcm_init     ----- lk \n");
	///ldo_p5m5_dsv_3v0_off();
#else
printk("ili9806e_dsi_vdo_fwvga_dj.c  lcm_init   ------kernel \n");
#endif
	////ldo_2v8vgp2_on();
	MDELAY(50);
	/* 
	/ reset pin High > delay 1ms > Low > delay 10ms > High > delay 120ms
	/ LCD RESET PIN 
	*/
	MDELAY(1);
	SET_RESET_PIN(1);///reset_lcd_module(1);
	MDELAY(1);
	SET_RESET_PIN(0);///reset_lcd_module(0);
	MDELAY(30);
	SET_RESET_PIN(1);///reset_lcd_module(1);
	MDELAY(150);
	init_lcm_registers();	//SET EXTC ~ sleep out register
	//need_set_lcm_addr = 1;
}

static void lcm_suspend(void)
{
	unsigned int data_array[1];
	data_array[0] = 0x00280500;	//Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);	
	data_array[0] = 0x00100500;	//Sleep in
	dsi_set_cmdq(data_array, 1, 1);
	//reset_lcd_module(0); //reset pin low
	MDELAY(120);
	LCM_PRINT("[ili9806e_dsi_vdo_fwvga_dj.c] lcm_suspend \n");
}

static void lcm_resume(void)
{
#if 1
	unsigned int data_array[1];
	data_array[0] = 0x00110500;	//Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);	
	data_array[0] = 0x00290500;	//Sleep in
	dsi_set_cmdq(data_array, 1, 1);
	//reset_lcd_module(0); //reset pin low
	MDELAY(120);
	#else
	lcm_init();
	#endif
    need_set_lcm_addr = 1;
	LCM_PRINT("[ili9806e_dsi_vdo_fwvga_dj.c] lcm_resume \n");
}
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char  buffer[6];
	int   array[4];
	unsigned int tmp;
	
	//set_HS_read();
	DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, 0);
    array[0] = 0x00013708;
    //array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);	
	read_reg_v2(0x0A, &buffer[0], 1);
	array[0] = 0x00013708;
	//restore_HS_read();	
	DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, 1);
	LCM_PRINT("[ili9806e_dsi_vdo_fwvga_dj.c] esdcheck buffer[0]=0x%x \n",buffer[0]);
	if(0x9C == buffer[0])
	{
		return FALSE;
	}
	else
	{
		return TRUE;
	}
#endif	
}

static void lcm_esd_recover(void)
{
	lcm_init();
	LCM_PRINT("[ili9806e_dsi_vdo_fwvga_dj.c] lcm_esd_recover \n");
}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	// need update at the first time
	if(need_set_lcm_addr)
	{
		data_array[0]= 0x00053902;
		data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
		data_array[2]= (x1_LSB);
		dsi_set_cmdq(data_array, 3, 1);
		
		data_array[0]= 0x00053902;
		data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
		data_array[2]= (y1_LSB);
		dsi_set_cmdq(data_array, 3, 1);		
		need_set_lcm_addr = 0;
	}
	
	data_array[0]= 0x002c3909;
   dsi_set_cmdq(data_array, 1, 0);
	LCM_PRINT("[ili9806e_dsi_vdo_fwvga_dj.c] lcm_update \n");	
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[4];
	unsigned int array[16];  
	unsigned char params[5] = {0xFF,0x98,0x06,0x04,0x01};
LCM_PRINT("[ili9806e_dsi_vdo_fwvga_dj.c] lcm_compare_id \n");
	#if 1///lc mike_zhu 20150527
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(10);//Must over 6 ms
/*
	array[0]=0x00063902;
	array[1]=0x0698FFFF;// page enable
	array[2]=0x00000104;
	dsi_set_cmdq(array, 3, 1);
	MDELAY(10);

	array[0] = 0x00033700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);

	read_reg_v2(0x00, buffer, 3);

*/
	dsi_set_cmdq_V2(0xFF, 5, params, 1);//enable to page1
	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x00, &buffer[0], 1);

	array[0] = 0x00033700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x01, &buffer[1], 1);

	array[0] = 0x00033700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x02, &buffer[2], 1);
	
#ifdef BUILD_LK
	printf("zwp...%s,ili9806e..lcm_compare_id:0x%2x,0x%2x,0x%2x\n", __func__,buffer[0],buffer[1],buffer[2]);
#else
	printk("zwp...%s,ili9806e..lcm_compare_id:0x%2x,0x%2x,0x%2x\n", __func__,buffer[0],buffer[1],buffer[2]);
#endif
    if((LCM_ID_LI9806E_A == buffer[0])&&(LCM_ID_LI9806E_B == buffer[1])&&(LCM_ID_LI9806E_C == buffer[2]))
	    return 1;
    else
	    return 0;
#else
   return 1;
#endif	
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_dsi_vdo_fwvga_dj_drv = {
	.name = "ili9806e_dsi_vdo_fwvga_dj",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
#if (!defined(BUILD_UBOOT) && !defined(BUILD_LK))
	///.esd_check= lcm_esd_check,
	.esd_recover = lcm_esd_recover,
#endif
};
