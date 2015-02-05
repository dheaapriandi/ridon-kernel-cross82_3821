#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID_OTM1283 (0x1283)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
static unsigned int is_lcm_connected = FALSE;

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF  // END OF REGISTERS MARKER


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define   LCM_DSI_CMD_MODE							0



// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
    {0x00,1,{0x00}}, 
    {0xFF,3,{0x12,0x83,0x01}}, 
    {0x00,1,{0x80}}, 
    {0xFF,2,{0x12,0x83}}, 
    {0x00,1,{0xA2}}, 
    {0xC1,1,{0x08}}, 
    {0x00,1,{0xA4}}, 
    {0xC1,1,{0xF0}}, 
    {0x00,1,{0x80}}, 
    {0xC4,1,{0x30}}, 
    {0x00,1,{0x8A}}, 
    {0xC4,1,{0x40}}, 
    {0x00,1,{0x80}}, 
    {0xc0,9,{0x00,0x64,0x00,0x0f,0x11,0x00,0x64,0x0f,0x11}}, 
    {0x00,1,{0x90}}, 
    {0xC0,6,{0x00,0x55,0x00,0x01,0x00,0x04}}, 
    {0x00,1,{0xA4}}, 
    {0xC0,1,{0x00}}, 
    {0x00,1,{0xB3}}, 
    {0xC0,2,{0x00,0x50}}, 
    {0x00,1,{0x81}}, 
    {0xC1,1,{0x66}}, 
    {0x00,1,{0xA0}}, 
    {0xC1,1,{0x02}}, 
    {0x00,1,{0x80}}, 
    {0xC4,1,{0x30}}, 
    {0x00,1,{0x81}}, 
    {0xC4,1,{0x83}}, 
    {0x00,1,{0x82}}, 
    {0xC4,1,{0x02}}, 
    {0x00,1,{0x90}}, 
    {0xC4,1,{0x49}}, 
    {0x00,1,{0xB9}}, 
    {0xB0,1,{0x51}}, 
    {0x00,1,{0xC6}}, 
    {0xB0,1,{0x03}}, 
    {0x00,1,{0xA4}}, 
    {0xC0,1,{0x00}}, 
    {0x00,1,{0x87}}, 
    {0xC4,1,{0x18}}, 
    {0x00,1,{0xB0}}, 
    {0xC6,1,{0x03}}, 
    {0x00,1,{0x90}}, 
    {0xF5,4,{0x02,0x11,0x02,0x11}}, 
    {0x00,1,{0x90}}, 
    {0xC5,1,{0x50}}, 
    {0x00,1,{0x94}}, 
    {0xC5,1,{0x66}}, 
    {0x00,1,{0xB2}}, 
    {0xF5,2,{0x00,0x00}},
    {0x00,1,{0xB4}}, 
    {0xF5,2,{0x00,0x00}}, 
    {0x00,1,{0xB6}}, 
    {0xF5,2,{0x00,0x00}}, 
    {0x00,1,{0xB8}}, 
    {0xF5,2,{0x00,0x00}}, 
    {0x00,1,{0x94}}, 
    {0xF5,1,{0x02}}, 
    {0x00,1,{0xBA}}, 
    {0xF5,1,{0x03}}, 
    {0x00,1,{0xB4}}, 
    {0xC5,1,{0xC0}}, 
    {0x00,1,{0xA0}},
    {0xC4,14,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}}, 
    {0x00,1,{0xB0}}, 
    {0xC4,2,{0x66,0x66}},
    {0x00,1,{0x91}}, 
    {0xC5,2,{0x19,0x50}}, 
    {0x00,1,{0xB0}}, 
    {0xC5,2,{0x04,0xB8}}, 
    {0x00,1,{0xB5}}, 
    {0xC5,6,{0x03,0xE8,0x40,0x03,0xE8,0x40}}, 
    {0x00,1,{0xBB}}, 
    {0xC5,2,{0x80,0x00}}, 
    {0x00,1,{0x80}}, 
    {0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0x90}}, 
    {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0xff,0x00}}, 
    {0x00,1,{0xA0}}, 
    {0xcb,15,{0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0xB0}}, 
    {0xCB,15,{0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0xC0}}, 
    {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x05,0x05}}, 
    {0x00,1,{0xD0}}, 
    {0xCB,15,{0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0xE0}}, 
    {0xCB,14,{0x00,0x00,0x00,0x05,0x00,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0xF0}}, 
    {0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}}, 
    {0x00,1,{0x80}}, 
    {0xCC,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x10,0x0E}}, 
    {0x00,1,{0x90}}, 
    {0xCC,15,{0x0C,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0xA0}}, 
    {0xCC,14,{0x00,0x00,0x00,0x09,0x00,0x0F,0x0D,0x0B,0x01,0x03,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0xB0}}, 
    {0xCC,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x10,0x0E}}, 
    {0x00,1,{0xC0}}, 
    {0xCC,15,{0x0C,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0xD0}}, 
    {0xCC,14,{0x00,0x00,0x00,0x09,0x00,0x0F,0x0D,0x0B,0x01,0x03,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0x80}}, 
    {0xCE,12,{0x87,0x03,0x06,0x86,0x03,0x06,0x85,0x03,0x06,0x84,0x03,0x06}}, 
    {0x00,1,{0x90}}, 
    {0xCE,14,{0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00}}, 
    {0x00,1,{0xA0}}, 
    {0xCE,14,{0x38,0x05,0x84,0xFE,0x00,0x06,0x00,0x38,0x04,0x84,0xFF,0x00,0x06,0x00}}, 
    {0x00,1,{0xB0}}, 
    {0xCE,14,{0x38,0x03,0x85,0x00,0x00,0x06,0x00,0x38,0x02,0x85,0x01,0x00,0x06,0x00}}, 
    {0x00,1,{0xC0}}, 
    {0xCE,14,{0x38,0x01,0x85,0x02,0x00,0x06,0x00,0x38,0x00,0x85,0x03,0x00,0x06,0x00}}, 
    {0x00,1,{0xD0}}, 
    {0xCE,14,{0x30,0x00,0x85,0x04,0x00,0x06,0x00,0x30,0x01,0x85,0x05,0x00,0x06,0x00}}, 
    {0x00,1,{0x80}}, 
    {0xCF,14,{0x70,0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}}, 
    {0x00,1,{0x90}}, 
    {0xCF,14,{0x70,0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}}, 
    {0x00,1,{0xA0}}, 
    {0xCF,14,{0x70,0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}}, 
    {0x00,1,{0xB0}}, 
    {0xCF,14,{0x70,0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}}, 
    {0x00,1,{0xC0}}, 
    {0xCF,11,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x80,0x00,0x03,0x08}}, 
    {0x00,1,{0x00}}, 
    {0xD8,2,{0xBC,0xBC}}, 
    {0x00,1,{0x00}}, 
    {0xD9,1,{0xa2}}, 
    {0x00,1,{0x00}}, 
    {0xE1,16,{0x04,0x09,0x0e,0x0D,0x06,0x0F,0x0A,0x09,0x04,0x07,0x0d,0x07,0x0E,0x16,0x10,0x0A}}, 
    {0x00,1,{0x00}}, 
    {0xE2,16,{0x04,0x09,0x0f,0x0D,0x06,0x0F,0x0A,0x09,0x04,0x07,0x0d,0x07,0x0E,0x16,0x10,0x0A}}, 
    {0x35,1,{0x00}}, 
    {REGFLAG_DELAY, 10, {}},
    {0x51,1,{0x00}}, 
    {0x53,1,{0x24}}, 
    {0x55,1,{0x00}}, 
    {REGFLAG_DELAY, 10, {}},
    {0x00,1,{0x00}}, 
    {0xFF,3,{0xFF,0xFF,0xFF}}, 
    {0x11,1,{0x00}}, 
    {REGFLAG_DELAY, 200, {}}, 
    {0x29,1,{0x00}}, 
    {REGFLAG_DELAY, 50, {}}, 
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xF0,	5,	{0x55, 0xaa, 0x52,0x08,0x00}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

//edit by Magnum 2012-12-18
static void dsi_send_cmdq_tinno(unsigned cmd, unsigned char count, unsigned char *para_list, unsigned char force_update)
{
	unsigned int item[16];
	unsigned char dsi_cmd = (unsigned char)cmd;
	unsigned char dc;
	int index = 0, length = 0;
	
	memset(item,0,sizeof(item));
	if(count+1 > 60)
	{
		//LCM_DBG("Exceed 16 entry\n");
		return;
	}
/*
	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05
*/	
	if(count == 0)
	{
		item[0] = 0x0500 | (dsi_cmd<<16);
		length = 1;
	}
	else if(count == 1)
	{
		item[0] = 0x1500 | (dsi_cmd<<16) | (para_list[0]<<24);
		length = 1;
	}
	else
	{
		item[0] = 0x3902 | ((count+1)<<16);//Count include command.
		++length;
		while(1)
		{
			if (index == count+1)
				break;
			if ( 0 == index ){
				dc = cmd;
			}else{
				dc = para_list[index-1];
			}
			// an item make up of 4data. 
			item[index/4+1] |= (dc<<(8*(index%4)));  
			if ( index%4 == 0 ) ++length;
			++index;
		}
	}
	
	dsi_set_cmdq(&item, length, force_update);

}


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_send_cmdq_tinno(cmd, table[i].count, table[i].para_list, force_update);
		//	dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
	//	params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
	//	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 2;// 3    2
    	params->dsi.vertical_backporch					= 14;// 20   1
		params->dsi.vertical_frontporch					= 16; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 6;// 50  2
   		params->dsi.horizontal_backporch				= 42;
		params->dsi.horizontal_frontporch				= 44;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
	//	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	//	params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
	//arams->dsi.fbk_div =14;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	

        
		params->dsi.PLL_CLOCK=208;

		

}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(130);

   // lcm_init_register();
   push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}



static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);      
}


static void lcm_resume(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00110500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);      
	
	data_array[0] = 0x00290500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);      
}
         
#if (LCM_DSI_CMD_MODE)
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

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
    unsigned int id=0,id2=0;
	unsigned char buffer[5],buffer2[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
    MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(25);
	SET_RESET_PIN(1);
	MDELAY(50);

	array[0]=0x00043902;
	array[1]=0x018312ff; 
	dsi_set_cmdq(array, 2, 1); //{0xff, 3 ,{0x80,0x09,0x01}}, // Command2 Enable
	
	array[0]=0x80001500;
	dsi_set_cmdq(array, 1, 1); //{0x00, 1 ,{0x80}},
	
	array[0]=0x00033902;
	array[1]=0x008312ff;
	dsi_set_cmdq(array, 2, 1); //{0xff, 2 ,{0x80,0x09}}, // Orise Mode Enable
	
	array[0]=0xC5001500;
	dsi_set_cmdq(array, 1, 1); //{0x00, 1 ,{0xC6}},
	
	array[0]=0x03B01500;
	dsi_set_cmdq(array, 1, 1); //{0xB0, 1 ,{0x03}}, // Read Contention Error
	MDELAY(10);

	array[0] = 0x00023700;// set return byte number
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, buffer, 4); // Read Register 0xA1 : 0x01,0x8B,0x80,0x09 (OTM8009A/OTM8018B);

	id = buffer[2]<<8 | buffer[3]; 
#ifndef BUILD_LK
	printk("[LSQ] -- otm1283a 0x%x , 0x%x , 0x%x \n",buffer[2],buffer[3],id);
#endif
    if(id == LCM_ID_OTM1283)
    {
        is_lcm_connected = TRUE;
    	return 1;
    }
    else
        return 0;
}


static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
   unsigned char buffer[1];
   unsigned int array[16];

    if(is_lcm_connected == FALSE)
        return FALSE;

#if defined(BUILD_LK)
	printf("[cabc] otm1283a: lcm_esd_check enter\n");
#else
	printk("[cabc] otm1283a: lcm_esd_check enter\n");
#endif

//////////////////////
   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0A, buffer, 1);
#if defined(BUILD_LK)
    printf("lcm_esd_check  0x0A = %x\n",buffer[0]);
#else
    printk("lcm_esd_check  0x0A = %x\n",buffer[0]);
#endif
   if(buffer[0] != 0x9C)//0x9C
   {
      return 1;
   }

  /////////////// 
  /* array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0B, buffer, 1);
#if defined(BUILD_LK)
    printf("lcm_esd_check  0x0B = %x\n",buffer[0]);
#else
    printk("lcm_esd_check  0x0B = %x\n",buffer[0]);
#endif
   if(buffer[0] != 0x00)//0x88 ???????????????????????????????
   {
      return 1;
   }*/

   //////////////////////

/*
   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0C, buffer, 1);
#if defined(BUILD_LK)
    printf("lcm_esd_check  0x0C =%x\n",buffer[0]);
#else
    printk("lcm_esd_check  0x0C =%x\n",buffer[0]);
#endif
   if(buffer[0] != 0x07)//0x07
   {
      return 1;
   }

*/
/////////////////////////////
   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0D, buffer, 1);
#if defined(BUILD_LK)
    printf("lcm_esd_check 0x0D =%x\n",buffer[0]);
#else
    printk("lcm_esd_check 0x0D =%x\n",buffer[0]);
#endif
   if(buffer[0] != 0x00)//0x00
   {
      return 1;
   }

   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
      //    id = read_reg(0xF4);
   read_reg_v2(0x0E, buffer, 1);
#if defined(BUILD_LK)
    printf("lcm_esd_check  0x0E = %x\n",buffer[0]);
#else
    printk("lcm_esd_check  0x0E = %x\n",buffer[0]);
#endif
    if(buffer[0] != 0x80)
    {
        return 1;
    }
 #if defined(BUILD_LK)
	printf("[cabc] otm1283a: lcm_esd_check exit\n");
#else
	printk("[cabc] otm1283a: lcm_esd_check exit\n");
#endif
/////////////////
   return 0;


#endif

}

static unsigned int lcm_esd_recover(void)
{
	unsigned int data_array[16];

#if defined(BUILD_LK)
    printf("lcm_esd_recover enter");
#else
    printk("lcm_esd_recover enter");
#endif

    lcm_init();
 	data_array[0]=0x00110500;
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(50);
	
	data_array[0]=0x00290500;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0]= 0x00023902;
	data_array[1]= 0xFF51;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

    return TRUE;
}



LCM_DRIVER otm1283a_hd720_dsi_vdo_lcm_drv = 
{
    .name			= "otm1283a_hd720_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
