/* BMA2XX motion sensor driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <mach/mt_gpio.h>

//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "bma2xx.h"
#include <linux/hwmsen_helper.h>

//add by liuhuan
#if defined(MIKI_YUANZHENG_SUPPORT)
#include <mach/mt_pwm.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

#endif
//end
#include "cust_gpio_usage.h"
#include "cust_eint.h"


/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_BMA2XX_LOWPASS   /*apply low pass filter on output*/       
#define SW_CALIBRATION

/*----------------------------------------------------------------------------*/
#define BMA2XX_AXIS_X		0
#define BMA2XX_AXIS_Y		1
#define BMA2XX_AXIS_Z		2
#define BMA2XX_AXES_NUM		3
#define BMA2XX_DATA_LEN		6
#define BMA2XX_DEV_NAME		"BMA2XX"

#define BMA2XX_RETRY_MAX	10



/*----------------------------------------------------------------------------*/
extern s32 mt_set_gpio_pull_select_ext(u32 pin, u32 select);
extern s32 mt_set_gpio_mode_ext(u32 pin, u32 mode);
extern s32 mt_set_gpio_dir_ext(u32 pin, u32 dir);
extern s32 mt_set_gpio_pull_enable_ext(u32 pin, u32 enable);
extern s32 mt_set_gpio_out_ext(u32 pin, u32 output);

extern s32 mt_get_gpio_mode_ext(u32 pin);
extern s32 mt_get_gpio_dir_ext(u32 pin);
extern s32 mt_get_gpio_pull_enable_ext(u32 pin);
extern s32 mt_get_gpio_pull_select_ext(u32 pin);
extern s32 mt_get_gpio_out_ext(u32 pin);



extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);

extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);

extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en,
			  unsigned int pol, void (EINT_FUNC_PTR) (void),
			  unsigned int is_auto_umask);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id bma2xx_i2c_id[] = {{BMA2XX_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_BMA2XX={ I2C_BOARD_INFO(BMA2XX_DEV_NAME, (BMA2XX_I2C_SLAVE_WRITE_ADDR>>1))};

/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int bma2xx_i2c_remove(struct i2c_client *client);
static int bma2xx_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

static int bma2xx_local_init(void);
static int bma2xx_remove(void);


static int bma2xx_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
static int bma2xx_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);


static int bma2xx_init_flag =0;

static struct sensor_init_info bma2xx_init_info = {
		.name = "bma2xx",
		.init = bma2xx_local_init,
		.uninit = bma2xx_remove,
};
/*----------------------------------------------------------------------------*/
typedef enum {
	BMA_TRC_FILTER  = 0x01,
	BMA_TRC_RAWDATA = 0x02,
	BMA_TRC_IOCTL   = 0x04,
	BMA_TRC_CALI	= 0X08,
	BMA_TRC_INFO	= 0X10,
} BMA_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
	u8  whole;
	u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][BMA2XX_AXES_NUM];
	int sum[BMA2XX_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct bma2xx_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert   cvt;

	/*misc*/
	struct data_resolution *reso;
	atomic_t                trace;
	atomic_t                suspend;
	atomic_t                selftest;
	atomic_t				filter;
	atomic_t				sensor_power;
	s16                     cali_sw[BMA2XX_AXES_NUM+1];

	/*data*/
	s8                      offset[BMA2XX_AXES_NUM+1];  /*+1: for 4-byte alignment*/
	s16                     data[BMA2XX_AXES_NUM+1];

#if defined(CONFIG_BMA2XX_LOWPASS)
	atomic_t                firlen;
	atomic_t                fir_en;
	struct data_filter      fir;
#endif 
	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver bma2xx_i2c_driver = {
	.driver = {
		.name           = BMA2XX_DEV_NAME,
	},
	.probe      		= bma2xx_i2c_probe,
	.remove    			= bma2xx_i2c_remove,
	.detect				= bma2xx_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
	.suspend            = bma2xx_suspend,
	.resume             = bma2xx_resume,
#endif
	.id_table = bma2xx_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *bma2xx_i2c_client = NULL;
//static struct platform_driver bma2xx_gsensor_driver;
static struct bma2xx_i2c_data *obj_i2c_data = NULL;
static GSENSOR_VECTOR3D gsensor_gain;

static struct mutex i2c_data_mutex;


/*----------------------------------------------------------------------------*/
#define GSE_TAG_FUN                  "\n\n===>>[@miki_bma2xx_fun]- "
#define GSE_TAG_INFO                  "\n\n===>>[@miki_bma2xx_info]- "
#define GSE_TAG_ERR                  "\n\n===>>[@miki_bma2xx_err]- "

#define GSE_FUN(f) 
//#define GSE_ERR(fmt, args...) 
#define GSE_LOG(fmt, args...) 

//#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG_FUN"--%s:\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG_ERR"--%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
//#define GSE_LOG(fmt, args...)    printk(GSE_TAG_INFO "--%d :"fmt, __LINE__,##args)


#define MIKI_DBG_TAG                  "\n\n==>>[@miki_bma2xx_log@v_v@]---"
#define MIKI_FUN_TAG                  "\n\n==>>[@miki_bma2xx_fun@v_v@]---"
#define MIKI_ERR_TAG                  "\n\n==>>[@miki_bma2xx_err@v_v@]---"


#define miki_gs_fun()
#define miki_gs_log(fmt, args...)
//#define miki_gs_err(fmt, args...)

//#define miki_gs_log(fmt, args...)  printk(MIKI_DBG_TAG "line = %d,"fmt, __LINE__, ##args)
//#define miki_gs_fun()  printk(MIKI_FUN_TAG "line = %d, fun call:%s---\n\n",__LINE__, __FUNCTION__)
#define miki_gs_err(fmt, args...)  printk(MIKI_ERR_TAG "line = %d,"fmt, __LINE__, ##args)

int bma250_flag = 0;
/*----------------------------------------------------------------------------*/
static struct data_resolution bma2xx_data_resolution[1] = {
	/* combination by {FULL_RES,RANGE}*/
	{{ 15, 6}, 64},   // dataformat +/-2g  in 8-bit resolution;  { 15, 6} = 15.6= (2*2*1000)/(2^8);  64 = (2^8)/(2*2)          
};
static struct data_resolution bma250_data_resolution[1] = {
	/* combination by {FULL_RES,RANGE}*/
	{{ 3, 9}, 256},   // dataformat +/-2g  in 8-bit resolution;  { 15, 6} = 15.6= (2*2*1000)/(2^8);  64 = (2^8)/(2*2)          
};
/*----------------------------------------------------------------------------*/
static struct data_resolution bma2xx_offset_resolution = {{15, 6}, 64};
static struct data_resolution bma250_offset_resolution = {{3, 9}, 256};
/*--------------------BMA2XX power control function----------------------------------*/
static void BMA2XX_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "BMA2XX"))
			{
				miki_gs_err("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "BMA2XX"))
			{
				miki_gs_err("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int BMA2XX_SetDataResolution(struct bma2xx_i2c_data *obj)
{

	/*set g sensor dataresolution here*/

	/*BMA2XX only can set to 10-bit dataresolution, so do nothing in bma2xx driver here*/

	/*end of set dataresolution*/

	/*we set measure range from -2g to +2g in BMA2XX_SetDataFormat(client, BMA2XX_RANGE_2G), 
	  and set 10-bit dataresolution BMA2XX_SetDataResolution()*/

	/*so bma2xx_data_resolution[0] set value as {{ 3, 9}, 256} when declaration, and assign the value to obj->reso here*/  

	if(bma250_flag==1)
	{
	obj->reso = &bma250_data_resolution[0];
	}
	else
	{
	obj->reso = &bma2xx_data_resolution[0];
	}
	return 0;

	/*if you changed the measure range, for example call: BMA2XX_SetDataFormat(client, BMA2XX_RANGE_4G), 
	  you must set the right value to bma2xx_data_resolution*/

}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadData(struct i2c_client *client, s16 data[BMA2XX_AXES_NUM])
{
    if(bma250_flag==1)
    {
#ifdef CONFIG_BMA2XX_LOWPASS
       struct bma2xx_i2c_data *priv = i2c_get_clientdata(client);        
#endif        
	u8 addr = 0x02;
	u8 buf[BMA2XX_DATA_LEN] = {0};
	int err = 0;
	int i;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = bma2xx_i2c_read_block(client, addr, buf, 0x06))<0)
	{
		miki_gs_err("error: %d\n", err);
	}
	else
	{
		data[BMA2XX_AXIS_X] = (s16)((buf[BMA2XX_AXIS_X*2] >> 6) |
		         (buf[BMA2XX_AXIS_X*2+1] << 2));
		data[BMA2XX_AXIS_Y] = (s16)((buf[BMA2XX_AXIS_Y*2] >> 6) |
		         (buf[BMA2XX_AXIS_Y*2+1] << 2));
		data[BMA2XX_AXIS_Z] = (s16)((buf[BMA2XX_AXIS_Z*2] >> 6) |
		         (buf[BMA2XX_AXIS_Z*2+1] << 2));

		for(i=0;i<3;i++)				
		{								//because the data is store in binary complement number formation in computer system
			if ( data[i] == 0x0200 )	//so we want to calculate actual number here
				data[i]= -512;			//10bit resolution, 512= 2^(10-1)
			else if ( data[i] & 0x0200 )//transfor format
			{							//printk("data 0 step %x \n",data[i]);
				data[i] -= 0x1;			//printk("data 1 step %x \n",data[i]);
				data[i] = ~data[i];		//printk("data 2 step %x \n",data[i]);
				data[i] &= 0x01ff;		//printk("data 3 step %x \n\n",data[i]);
				data[i] = -data[i];		
			}
		}	

		//if(1)//if(atomic_read(&priv->trace) & BMA_TRC_RAWDATA)
		//{
			//GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z],
			//		data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
		//}
#ifdef CONFIG_BMA2XX_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_X] = data[BMA2XX_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Y] = data[BMA2XX_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Z] = data[BMA2XX_AXIS_Z];
					priv->fir.sum[BMA2XX_AXIS_X] += data[BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] += data[BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] += data[BMA2XX_AXIS_Z];
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
								priv->fir.raw[priv->fir.num][BMA2XX_AXIS_X], priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Y], priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Z],
								priv->fir.sum[BMA2XX_AXIS_X], priv->fir.sum[BMA2XX_AXIS_Y], priv->fir.sum[BMA2XX_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA2XX_AXIS_X] -= priv->fir.raw[idx][BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] -= priv->fir.raw[idx][BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] -= priv->fir.raw[idx][BMA2XX_AXIS_Z];
					priv->fir.raw[idx][BMA2XX_AXIS_X] = data[BMA2XX_AXIS_X];
					priv->fir.raw[idx][BMA2XX_AXIS_Y] = data[BMA2XX_AXIS_Y];
					priv->fir.raw[idx][BMA2XX_AXIS_Z] = data[BMA2XX_AXIS_Z];
					priv->fir.sum[BMA2XX_AXIS_X] += data[BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] += data[BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] += data[BMA2XX_AXIS_Z];
					priv->fir.idx++;
					data[BMA2XX_AXIS_X] = priv->fir.sum[BMA2XX_AXIS_X]/firlen;
					data[BMA2XX_AXIS_Y] = priv->fir.sum[BMA2XX_AXIS_Y]/firlen;
					data[BMA2XX_AXIS_Z] = priv->fir.sum[BMA2XX_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
								priv->fir.raw[idx][BMA2XX_AXIS_X], priv->fir.raw[idx][BMA2XX_AXIS_Y], priv->fir.raw[idx][BMA2XX_AXIS_Z],
								priv->fir.sum[BMA2XX_AXIS_X], priv->fir.sum[BMA2XX_AXIS_Y], priv->fir.sum[BMA2XX_AXIS_Z],
								data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
    }
    else
    {
	struct bma2xx_i2c_data *priv = i2c_get_clientdata(client);        
	u8 addr = BMA2XX_REG_DATAXLOW;
	u8 buf[BMA2XX_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = bma2xx_i2c_read_block(client, addr, buf, 0x05))<0)
	{
		miki_gs_err("error: %d\n", err);
	}
	else
	{
		data[BMA2XX_AXIS_X] = (s16)buf[BMA2XX_AXIS_X*2] ;
		data[BMA2XX_AXIS_Y] = (s16)buf[BMA2XX_AXIS_Y*2];
		data[BMA2XX_AXIS_Z] = (s16)buf[BMA2XX_AXIS_Z*2] ;
		if(atomic_read(&priv->trace) & BMA_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] before\n", data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z],
					data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
		}

		if(data[BMA2XX_AXIS_X]&0x80)
		{
			data[BMA2XX_AXIS_X] = ~data[BMA2XX_AXIS_X];
			data[BMA2XX_AXIS_X] &= 0xff;
			data[BMA2XX_AXIS_X]+=1;
			data[BMA2XX_AXIS_X] = -data[BMA2XX_AXIS_X];
		}
		if(data[BMA2XX_AXIS_Y]&0x80)
		{
			data[BMA2XX_AXIS_Y] = ~data[BMA2XX_AXIS_Y];
			data[BMA2XX_AXIS_Y] &= 0xff;
			data[BMA2XX_AXIS_Y]+=1;
			data[BMA2XX_AXIS_Y] = -data[BMA2XX_AXIS_Y];
		}
		if(data[BMA2XX_AXIS_Z]&0x80)
		{
			data[BMA2XX_AXIS_Z] = ~data[BMA2XX_AXIS_Z];
			data[BMA2XX_AXIS_Z] &= 0xff;
			data[BMA2XX_AXIS_Z]+=1;
			data[BMA2XX_AXIS_Z] = -data[BMA2XX_AXIS_Z];
		}

		if(atomic_read(&priv->trace) & BMA_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z],
					data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
		}
#ifdef CONFIG_BMA2XX_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_X] = data[BMA2XX_AXIS_X];
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Y] = data[BMA2XX_AXIS_Y];
					priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Z] = data[BMA2XX_AXIS_Z];
					priv->fir.sum[BMA2XX_AXIS_X] += data[BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] += data[BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] += data[BMA2XX_AXIS_Z];
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
								priv->fir.raw[priv->fir.num][BMA2XX_AXIS_X], priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Y], priv->fir.raw[priv->fir.num][BMA2XX_AXIS_Z],
								priv->fir.sum[BMA2XX_AXIS_X], priv->fir.sum[BMA2XX_AXIS_Y], priv->fir.sum[BMA2XX_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[BMA2XX_AXIS_X] -= priv->fir.raw[idx][BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] -= priv->fir.raw[idx][BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] -= priv->fir.raw[idx][BMA2XX_AXIS_Z];
					priv->fir.raw[idx][BMA2XX_AXIS_X] = data[BMA2XX_AXIS_X];
					priv->fir.raw[idx][BMA2XX_AXIS_Y] = data[BMA2XX_AXIS_Y];
					priv->fir.raw[idx][BMA2XX_AXIS_Z] = data[BMA2XX_AXIS_Z];
					priv->fir.sum[BMA2XX_AXIS_X] += data[BMA2XX_AXIS_X];
					priv->fir.sum[BMA2XX_AXIS_Y] += data[BMA2XX_AXIS_Y];
					priv->fir.sum[BMA2XX_AXIS_Z] += data[BMA2XX_AXIS_Z];
					priv->fir.idx++;
					data[BMA2XX_AXIS_X] = priv->fir.sum[BMA2XX_AXIS_X]/firlen;
					data[BMA2XX_AXIS_Y] = priv->fir.sum[BMA2XX_AXIS_Y]/firlen;
					data[BMA2XX_AXIS_Z] = priv->fir.sum[BMA2XX_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & BMA_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
								priv->fir.raw[idx][BMA2XX_AXIS_X], priv->fir.raw[idx][BMA2XX_AXIS_Y], priv->fir.raw[idx][BMA2XX_AXIS_Z],
								priv->fir.sum[BMA2XX_AXIS_X], priv->fir.sum[BMA2XX_AXIS_Y], priv->fir.sum[BMA2XX_AXIS_Z],
								data[BMA2XX_AXIS_X], data[BMA2XX_AXIS_Y], data[BMA2XX_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
    }
}
/*----------------------------------------------------------------------------*/

static int BMA2XX_ReadOffset(struct i2c_client *client, s8 ofs[BMA2XX_AXES_NUM])
{    
	int err;
	err = 0;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#else
	if(err = hwmsen_read_block(client, BMA2XX_REG_OFSX, ofs, BMA2XX_AXES_NUM))
	{
		GSE_ERR("error: %d\n", err);
	}
#endif
	//printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);

	return err;    
}

/*----------------------------------------------------------------------------*/
static int BMA2XX_ResetCalibration(struct i2c_client *client)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	//u8 ofs[4]={0,0,0,0};
	int err;
	err = 0;

#ifdef SW_CALIBRATION

#else
	if(err = hwmsen_write_block(client, BMA2XX_REG_OFSX, ofs, 4))
	{
		GSE_ERR("error: %d\n", err);
	}
#endif

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadCalibration(struct i2c_client *client, int dat[BMA2XX_AXES_NUM])
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int  err = 0;
	int mul;

#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	if ((err = BMA2XX_ReadOffset(client, obj->offset))) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	} 
	if(bma250_flag==1)   
	{
	mul = obj->reso->sensitivity/bma250_offset_resolution.sensitivity;
	}
	else
	mul = obj->reso->sensitivity/bma2xx_offset_resolution.sensitivity;
#endif

	dat[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*(obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X]);
	dat[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*(obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y]);
	dat[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*(obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z]);                        

	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadCalibrationEx(struct i2c_client *client, int act[BMA2XX_AXES_NUM], int raw[BMA2XX_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	err = 0;

#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	if(err = BMA2XX_ReadOffset(client, obj->offset))
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}  
	if(bma250_flag==1)   
	{
	mul = obj->reso->sensitivity/bma250_offset_resolution.sensitivity;
	}
	else 
	mul = obj->reso->sensitivity/bma2xx_offset_resolution.sensitivity;
#endif

	raw[BMA2XX_AXIS_X] = obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X];
	raw[BMA2XX_AXIS_Y] = obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y];
	raw[BMA2XX_AXIS_Z] = obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z];

	act[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*raw[BMA2XX_AXIS_X];
	act[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*raw[BMA2XX_AXIS_Y];
	act[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*raw[BMA2XX_AXIS_Z];                        

	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_WriteCalibration(struct i2c_client *client, int dat[BMA2XX_AXES_NUM])
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[BMA2XX_AXES_NUM], raw[BMA2XX_AXES_NUM];
	int lsb; 

	if(bma250_flag==1)   
	{
	lsb = bma250_offset_resolution.sensitivity;
	}
	else
	lsb = bma2xx_offset_resolution.sensitivity;
	//int divisor = obj->reso->sensitivity/lsb;

	if((err = BMA2XX_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		miki_gs_err("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
			raw[BMA2XX_AXIS_X], raw[BMA2XX_AXIS_Y], raw[BMA2XX_AXIS_Z],
			obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
			obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[BMA2XX_AXIS_X] += dat[BMA2XX_AXIS_X];
	cali[BMA2XX_AXIS_Y] += dat[BMA2XX_AXIS_Y];
	cali[BMA2XX_AXIS_Z] += dat[BMA2XX_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
			dat[BMA2XX_AXIS_X], dat[BMA2XX_AXIS_Y], dat[BMA2XX_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[BMA2XX_AXIS_X] = obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]]);
	obj->cali_sw[BMA2XX_AXIS_Y] = obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]]);
	obj->cali_sw[BMA2XX_AXIS_Z] = obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]]);	
#else
	int divisor = obj->reso->sensitivity/lsb;//modified
	obj->offset[BMA2XX_AXIS_X] = (s8)(obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]])/(divisor));
	obj->offset[BMA2XX_AXIS_Y] = (s8)(obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]])/(divisor));
	obj->offset[BMA2XX_AXIS_Z] = (s8)(obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[BMA2XX_AXIS_X] = obj->cvt.sign[BMA2XX_AXIS_X]*(cali[obj->cvt.map[BMA2XX_AXIS_X]])%(divisor);
	obj->cali_sw[BMA2XX_AXIS_Y] = obj->cvt.sign[BMA2XX_AXIS_Y]*(cali[obj->cvt.map[BMA2XX_AXIS_Y]])%(divisor);
	obj->cali_sw[BMA2XX_AXIS_Z] = obj->cvt.sign[BMA2XX_AXIS_Z]*(cali[obj->cvt.map[BMA2XX_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
			obj->offset[BMA2XX_AXIS_X]*divisor + obj->cali_sw[BMA2XX_AXIS_X], 
			obj->offset[BMA2XX_AXIS_Y]*divisor + obj->cali_sw[BMA2XX_AXIS_Y], 
			obj->offset[BMA2XX_AXIS_Z]*divisor + obj->cali_sw[BMA2XX_AXIS_Z], 
			obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
			obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

	if(err = hwmsen_write_block(obj->client, BMA2XX_REG_OFSX, obj->offset, BMA2XX_AXES_NUM))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_CheckDeviceID(struct i2c_client *client)
{
    u8 databuf[2];    
    int res = 0;
    int i = 0;
    u8 devids[] = BMA2XX_FIXED_DEVID;
    int devidlen = sizeof(devids) / sizeof(u8);

    memset(databuf, 0, sizeof(u8)*2);    
    databuf[0] = BMA2XX_REG_DEVID;    

    res = bma2xx_i2c_read_block(client, BMA2XX_REG_DEVID,databuf, 0x01);
    if(res < 0)
    {
        goto exit_BMA2XX_CheckDeviceID;
    }

    for(i = 0; i < devidlen; i++)
    {
        if(databuf[0] == devids[i])
        {
            if(databuf[0]==BMA250_FIXED_DEVID)
            {
                bma250_flag=1;
            }
            miki_gs_log("BMA2XX_CheckDeviceID,chip id = 0x%02x pass!\n ", databuf[0]);
            break;
        }
    }

    if(i >= devidlen)
    {
        miki_gs_err("BMA2XX_CheckDeviceID, chip id = 0x%02x failt!----\n ", databuf[0]);
        return BMA2XX_ERR_IDENTIFICATION;
    }

exit_BMA2XX_CheckDeviceID:
    if (res < 0)
    {
        miki_gs_err("EXIT_BMA2XX_CheckDeviceID with error---\n");
        return BMA2XX_ERR_I2C;
    }

    return BMA2XX_SUCCESS;
}

/*----------------------------------------------------------------------------*/
// enable = false, suspend mode.
// enable = true, resume mode.
static int BMA2XX_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = BMA2XX_SUCCESS;
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int retry = BMA2XX_RETRY_MAX;
	
       miki_gs_fun();

	do
	{
		if(enable == ((bool)atomic_read(&obj->sensor_power)) )
		{
			miki_gs_log("Sensor power status is newest!\n");
			return BMA2XX_SUCCESS;
		}

		if(bma2xx_i2c_read_block(client, BMA2XX_REG_POWER_CTL, databuf, 0x01)<0)
		{
			miki_gs_err("read power ctl register err!\n");
			res = BMA2XX_ERR_I2C;
			continue;
		}

		miki_gs_log("[set power mode] read from reg[0x11] = 0x%02x, databuf[1] = 0x%02x, enable = %d---\n", databuf[0], databuf[1], enable);
		
		if(enable == TRUE)
		{
			if(bma250_flag==1)
			{
				databuf[0] &= ~BMA2XX_MEASURE_MODE;
			}
			else
			{
                           //databuf[0] &= ~BMA2XX_MEASURE_MODE;
    	                    databuf[0] &= ~BMA2XX_NORM_MODE;  //liuhuan
			}
		}
		else
		{
			if(bma250_flag==1)
			{
	                    databuf[0] |= BMA2XX_MEASURE_MODE;
			}
			else
			{
				databuf[0] &= ~BMA2XX_SUSPEND_MODE;//liuhuan
				databuf[0] |= BMA2XX_MEASURE_MODE;
			}
		}
		
		databuf[1] = databuf[0];
		databuf[0] = BMA2XX_REG_POWER_CTL;
		
		miki_gs_log("[set power mode] write to databuf[0] = 0x%02x, databuf[1]=0x%02x---\n\n", databuf[0], databuf[1]);

		//res = i2c_master_send(client, databuf, 0x2);
		
		res = bma2xx_i2c_write_block(client,BMA2XX_REG_POWER_CTL , &databuf[1], 0x01);
             //miki_gs_log("i2c_master_send ret result = %d----.\n\n", res);
		if(res <0)
		{
			miki_gs_err("set power mode failed!\n\n");
			res = BMA2XX_ERR_I2C;
			continue;
		}
		else if(atomic_read(&obj->trace) & BMA_TRC_INFO)
		{
			miki_gs_log("set power mode ok  data = %d!\n\n", databuf[1]);
		}
		
             //miki_gs_log("to be confirm the setting.\n\n");
             
		/* confirm the setting */
		databuf[0] = 0xff;
		bma2xx_i2c_read_block(client, BMA2XX_REG_POWER_CTL, databuf, 0x01);

             miki_gs_log("read block databuf[0] = 0x%02x, databuf[1] = 0x%02x----.\n\n", databuf[0], databuf[1]);
		
		if (databuf[0] != databuf[1])
		{
			miki_gs_err("fail to set power mode!\n\n");
			res = BMA2XX_ERR_SETUP_FAILURE;
			continue;
		}

		miki_gs_log("BMA2XX_SetPowerMode ok!\n\n");
		atomic_set(&obj->sensor_power, enable);
		res = BMA2XX_SUCCESS;
		mdelay(20);
	} while (((retry--) > 0) && (res != BMA2XX_SUCCESS));

	miki_gs_log("res = %d by retry %d times!\n\n\n", res, (BMA2XX_RETRY_MAX - retry));

	return res;    
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[2] = {0};    
	int res = BMA2XX_SUCCESS;
	int retry = BMA2XX_RETRY_MAX;

	do
	{
		if(bma2xx_i2c_read_block(client, BMA2XX_REG_DATA_FORMAT, databuf, 0x01)<0)
		{
			miki_gs_err("bma2xx read Dataformat failed \n");
			res = BMA2XX_ERR_I2C;
			continue;
		}

		databuf[0] &= ~BMA2XX_RANGE_MASK;
		databuf[0] |= dataformat;
		if(bma250_flag==1)
		{
        		databuf[1] = databuf[0];
		}
		else
		{
        		databuf[1] = databuf[0] & BMA2XX_RANGE_MASK;//modified by liuhuan;;
		}
		
		databuf[0] = BMA2XX_REG_DATA_FORMAT;


		//res = i2c_master_send(client, databuf, 0x2);
		res = bma2xx_i2c_write_block(client, BMA2XX_REG_DATA_FORMAT, &databuf[1], 0x1);
		if(res < 0)
		{
                miki_gs_err("bma i2c write error.---\n\n");
                res = BMA2XX_ERR_I2C;
                continue;
		}

		/* confirm the setting */
		databuf[0] = 0xff;
		bma2xx_i2c_read_block(client, BMA2XX_REG_DATA_FORMAT, databuf, 0x01);
		if (databuf[0] != databuf[1])
		{
			miki_gs_err("fail to set data format!\n");
			res = BMA2XX_ERR_SETUP_FAILURE;
			continue;
		}

		res = BMA2XX_SUCCESS;
		//printk("BMA2XX_SetDataFormat OK! \n");
		mdelay(1);
	} while ((retry-- > 0) && (res != BMA2XX_SUCCESS));

	GSE_LOG("res = %d by retry %d times!\n", res, (BMA2XX_RETRY_MAX - retry));

	if (res == BMA2XX_SUCCESS)
	{
		return BMA2XX_SetDataResolution(obj);    
	}
	else
	{
		return res;
	}
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetBWRate(struct i2c_client *client, u8 bwrate)
{
        u8 databuf[2] = {0};    
        int res = BMA2XX_SUCCESS;
        int retry = BMA2XX_RETRY_MAX;
        miki_gs_fun();
        do
        {
            if(bma2xx_i2c_read_block(client, BMA2XX_REG_BW_RATE, databuf, 0x01)<0)
            {
                miki_gs_err("bma2xx read rate failed \n");
                res = BMA2XX_ERR_I2C;
                continue;
            }

            databuf[0] &= ~BMA2XX_BW_MASK;
            databuf[0] |= bwrate;
            if(bma250_flag==1)
            {
                databuf[1] = databuf[0];
            }
            else
            {
                databuf[1] = databuf[0] & BMA2XX_BW_MASK;//modified by liuhuan;;
            }
            
            databuf[0] = BMA2XX_REG_BW_RATE;

            //res = i2c_master_send(client, databuf, 0x2);
            res = bma2xx_i2c_write_block(client, BMA2XX_REG_BW_RATE, &databuf[1], 0x1);
            
            miki_gs_log("SetBWRate,i2c_master_send() return = %d\n\n", res);
            if(res < 0)
            {
                miki_gs_err("bma i2c write error.---\n\n");
                res = BMA2XX_ERR_I2C;
                continue;
            }

            /* confirm the setting */
            databuf[0] = 0xff;
            bma2xx_i2c_read_block(client, BMA2XX_REG_BW_RATE, databuf, 0x01);
            
            miki_gs_log("SetBWRate, config the setting, databuf[0] = 0x%02x, databuf[1] = 0x%02x---\n\n", databuf[0], databuf[1]);

            if (databuf[0] != databuf[1])
            {
                miki_gs_err("fail to set bandwidth!\n");
                res = BMA2XX_ERR_SETUP_FAILURE;
                continue;
            }

            res = BMA2XX_SUCCESS;
            miki_gs_log("BMA2XX_SetBWRate OK! \n\n");
            mdelay(1);
        } while ((retry-- > 0) && (res != BMA2XX_SUCCESS));

	miki_gs_log("res = %d by retry %d times!\n\n", res, (BMA2XX_RETRY_MAX - retry));

	return res;    
}
#if defined(MIKI_YUANZHENG_SUPPORT)
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetIntMode(struct i2c_client *client, u8 mode)
{
    u8 data=0;
	int res =0;	
	res = bma2xx_i2c_read_block(client, BMA2X2_INT_CTRL_REG ,&data, 0x01);
    if(res < 0)
    {
   		printk("BMA2XX_SetIntMode read err\n"); 
		return res;
	}

	//data |=0x01; //temporary 250ms
	data = (data&(~0x0f))|(mode&0x0f);

	res = bma2xx_i2c_write_block(client, BMA2X2_INT_CTRL_REG, &data, 0x01);
    if(res < 0) 
    {
        miki_gs_err("i2c write error\n");
        return res;
    }
	printk("BMA2XX_SetIntMode ok\n");

	return res;
}
#endif

/*----------------------------------------------------------------------------*/
static int BMA2XX_SetIntEnable(struct i2c_client *client, u8 intenable)
{
    int res = 0;
    u8 data = 0x00;
#if defined(MIKI_YUANZHENG_SUPPORT)
    data = 0x03;
	res = bma2xx_i2c_write_block(client, BMA2XX_INT_REG_1, &data, 0x01);
    if(res < 0) 
    {
        miki_gs_err("i2c write error\n");
        return res;
    }
	printk("liuhuan --------- set reg:0x16 val:0x03 enable x y z INT func---\n");
#else

    res = bma2xx_i2c_write_block(client, BMA2XX_INT_REG_1, &data, 0x01);
    if(res < 0) 
    {
        miki_gs_err("i2c write error\n");
        return res;
    }
#endif
    data=0x00;
	res = bma2xx_i2c_write_block(client, BMA2XX_INT_REG_2, &data, 0x01);
    if(res < 0 ) 
    {
        miki_gs_err("i2c write error\n");
        return res;
    }

    printk("BMA2XX disable interrupt  success...\n");

    /*for disable interrupt function*/

    return BMA2XX_SUCCESS;	  
}

#if defined(MIKI_YUANZHENG_SUPPORT)
/*----------------------------------------------------------------------------*/
static int BMA2XX_SetInt2_SlopeEnable(struct i2c_client *client, u8 int2sel)
{
    u8 data=0;
	int res = 0;	
	res = bma2xx_i2c_read_block(client, BMA2X2_INT1_PAD_SEL_REG,&data, 0x01);
    if(res < 0)
    {
   		printk("BMA2XX_SetIntMode read err\n"); 
		return res;
	}

	data = (data&(~0x04))|((int2sel<<2)&0x04);

	res = bma2xx_i2c_write_block(client, BMA2X2_INT1_PAD_SEL_REG, &data, 0x01);
    if(res < 0) 
    {
        miki_gs_err("i2c write error\n");
        return res;
    }
	printk("BMA2XX_SetIntMode ok\n");
}
/*----------------------------------------------------------------------------*/
static int bma2x2_set_slope_threshold(struct i2c_client *client, u8 threshold)
{
    u8 data=0;
	int res = 0;	
	res = bma2xx_i2c_read_block(client,BMA2X2_SLOPE_THRES_REG,&data, 0x01);
    if(res < 0)
    {
   		printk("bma2x2_set_slope_threshold read err\n"); 
		return res;
	}

	printk("liuhuan--------read slope threshold: %d   \n",data);

	res = bma2xx_i2c_write_block(client,BMA2X2_SLOPE_THRES_REG, &threshold, 0x01);
    if(res < 0) 
    {
        miki_gs_err("i2c write error\n");
        return res;
    }
	printk("bma2x2_set_slope_threshold ok\n");
	return res;

}
/*----------------------------------------------------------------------------*/
static int bma2x2_set_slope_duration(struct i2c_client *client, u8 duration)
{
    u8 data=0;
	int res = 0;	
	res = bma2xx_i2c_read_block(client,BMA2X2_SLOPE_DURN_REG,&data, 0x01);
    if(res < 0)
    {
   		printk("bma2x2_set_slope_duration read err\n"); 
		return res;
	}

	printk("liuhuan--------read slope duration: %d   \n",data);
	data = (data&~0x03)|(duration&0x03);
	res = bma2xx_i2c_write_block(client,BMA2X2_SLOPE_DURN_REG, &data, 0x01);
    if(res < 0) 
    {
        miki_gs_err("i2c write error\n");
        return res;
    }
	printk("bma2x2_set_slope_duration ok\n");
}
#endif
/*----------------------------------------------------------------------------*/
static int bma2xx_init_client(struct i2c_client *client, int reset_cali)
{
    struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
    int res = 0;
    
    res = BMA2XX_CheckDeviceID(client); 
    if(res != BMA2XX_SUCCESS)
    {
        miki_gs_err("BMA2XX_CheckDeviceID error ----\n");
        return res;
    }
    else
    {
        miki_gs_log("BMA2XX_CheckDeviceID ok ---\n");
    }
    
    res = BMA2XX_SetBWRate(client, BMA2XX_BW_25HZ);
    if(res != BMA2XX_SUCCESS ) 
    {
        miki_gs_err("BMA2XX_SetBWRate error!---\n");
        return res;
    }
    else
    {
        miki_gs_log("BMA2XX_SetBWRate OK!---\n");
    }
    
    res = BMA2XX_SetDataFormat(client, BMA2XX_RANGE_2G);
    if(res != BMA2XX_SUCCESS) 
    {
        miki_gs_err("BMA2XX_SetDataFormat error!---\n");
        return res;
    }
    else
    {
        miki_gs_log("BMA2XX_SetDataFormat OK!---\n");
    }
    
    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

#if defined(MIKI_YUANZHENG_SUPPORT) 
	res=bma2x2_set_slope_threshold(client, 0x5);
    if(res != BMA2XX_SUCCESS)
    {
        miki_gs_err("BMA2XX bma2x2_set_slope_threshold error---\n");
        return res;
    }
	
	res=bma2x2_set_slope_duration(client, 0x01);
    if(res != BMA2XX_SUCCESS)
    {
        miki_gs_err("BMA2XX bma2x2_set_slope_duration error---\n");
        return res;
    }

	res=BMA2XX_SetIntMode(client, 0x00);//250ms
    if(res != BMA2XX_SUCCESS)
    {
        miki_gs_err("BMA2XX BMA2XX_SetIntMode error---\n");
        return res;
    }

	res=BMA2XX_SetInt2_SlopeEnable(client, 0x01);
    if(res != BMA2XX_SUCCESS)
    {
        miki_gs_err("BMA2XX BMA2XX_SetInt2_SlopeEnable error---\n");
        return res;
    }
#endif
	
	res = BMA2XX_SetIntEnable(client, 0x00);        
    if(res != BMA2XX_SUCCESS)
    {
        miki_gs_err("BMA2XX disable interrupt function error---\n");
        return res;
    }
    else
    {
        miki_gs_log("BMA2XX disable interrupt function OK---\n");
    }
    
    res = BMA2XX_SetPowerMode(client, false);
    if(res != BMA2XX_SUCCESS)
    {
        miki_gs_err("BMA2XX_SetPowerMode OK!\n");
        return res;
    }
    else
    {
        miki_gs_log("BMA2XX_SetPowerMode OK!\n");
    }
    
    if(0 != reset_cali)
    { 
        /*reset calibration only in power on*/
        res = BMA2XX_ResetCalibration(client);
        if(res != BMA2XX_SUCCESS)
        {
            miki_gs_err("BMA2XX_ResetCalibration OK!\n");
            return res;
        }
    }
    
    printk("bma2xx_init_client OK!\n");
#ifdef CONFIG_BMA2XX_LOWPASS
    memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

    mdelay(20);

    return BMA2XX_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
    int res = 0;
    u8 chip_id = 0;
    
    if((NULL == buf)||(bufsize<=30))
    {
        return -1;
    }    
    
    if(NULL == client)
    {
        *buf = 0;
        return -2;
    }    
    
    res = bma2xx_i2c_read_block(client, BMA2XX_REG_DEVID, &chip_id, 0x01);

    if (res<0)
    {
        miki_gs_err("i2c read error.\n\n");
        return -3;
    }

    if(chip_id==0xF8) // bma222e
    {
        sprintf(buf, "WFJ BMA222E Chip");

    }
    else if(chip_id==0x03) // bma250
    {
        sprintf(buf, "BMA250 Chip");

    }  
    else if(chip_id==0xF9) // bma250e
    {
        sprintf(buf, "BMA250E Chip");

    }
    else 
    {
        sprintf(buf, "BMA2XX Chip");
    }
    
    return 0;
}

/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[BMA2XX_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(((bool)atomic_read(&obj->sensor_power)) == FALSE)
	{
		res = BMA2XX_SetPowerMode(client, true);
		if(res)
		{
			miki_gs_err("Power on bma2xx error %d!\n", res);
		}
	}

	if((res = BMA2XX_ReadData(client, obj->data))!=0)
	{        
		miki_gs_err("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//printk("raw data x=%d, y=%d, z=%d \n",obj->data[BMA2XX_AXIS_X],obj->data[BMA2XX_AXIS_Y],obj->data[BMA2XX_AXIS_Z]);
		obj->data[BMA2XX_AXIS_X] += obj->cali_sw[BMA2XX_AXIS_X];
		obj->data[BMA2XX_AXIS_Y] += obj->cali_sw[BMA2XX_AXIS_Y];
		obj->data[BMA2XX_AXIS_Z] += obj->cali_sw[BMA2XX_AXIS_Z];

		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[BMA2XX_AXIS_X],obj->cali_sw[BMA2XX_AXIS_Y],obj->cali_sw[BMA2XX_AXIS_Z]);

		/*remap coordinate*/
		acc[obj->cvt.map[BMA2XX_AXIS_X]] = obj->cvt.sign[BMA2XX_AXIS_X]*obj->data[BMA2XX_AXIS_X];
		acc[obj->cvt.map[BMA2XX_AXIS_Y]] = obj->cvt.sign[BMA2XX_AXIS_Y]*obj->data[BMA2XX_AXIS_Y];
		acc[obj->cvt.map[BMA2XX_AXIS_Z]] = obj->cvt.sign[BMA2XX_AXIS_Z]*obj->data[BMA2XX_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[BMA2XX_AXIS_X],obj->cvt.sign[BMA2XX_AXIS_Y],obj->cvt.sign[BMA2XX_AXIS_Z]);

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[BMA2XX_AXIS_X], acc[BMA2XX_AXIS_Y], acc[BMA2XX_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[BMA2XX_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[BMA2XX_AXIS_X] = acc[BMA2XX_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA2XX_AXIS_Y] = acc[BMA2XX_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[BMA2XX_AXIS_Z] = acc[BMA2XX_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		

		sprintf(buf, "%04x %04x %04x", acc[BMA2XX_AXIS_X], acc[BMA2XX_AXIS_Y], acc[BMA2XX_AXIS_Z]);
		if(atomic_read(&obj->trace) & BMA_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int BMA2XX_ReadRawData(struct i2c_client *client, char *buf)
{
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}

	if((res = BMA2XX_ReadData(client, obj->data)))
	{        
		miki_gs_err("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "BMA2XX_ReadRawData %04x %04x %04x", obj->data[BMA2XX_AXIS_X], 
				obj->data[BMA2XX_AXIS_Y], obj->data[BMA2XX_AXIS_Z]);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma2xx_i2c_client;
	char strbuf[BMA2XX_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	BMA2XX_ReadChipInfo(client, strbuf, BMA2XX_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma2xx_i2c_client;
	char strbuf[BMA2XX_BUFSIZE];

	if(NULL == client)
	{
		miki_gs_err("i2c client is null!!\n");
		return 0;
	}
	BMA2XX_ReadSensorData(client, strbuf, BMA2XX_BUFSIZE);
	//BMA2XX_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_regs_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma2xx_i2c_client;

#if 0
	int err = 0, i;
	u8 regsbuf[0x14];
	char strbuf[BMA2XX_BUFSIZE] = {0};

	err = bma2xx_i2c_read_block(client, 0x00, regsbuf, 0x014);
	if (err <0)
	{
		miki_gs_err("fail to read regs!\n");
		return 0;
	}
	
	for (i = 0; i < 0x14; i++)
	{
		sprintf(strbuf + strlen(strbuf), "%02X ", regsbuf[i]);
	}
	
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
#endif


//add by liuhuan
#if 1
    u8 test_buf[64]={0};
    int test_err = 0;
    int j,k;
    for(k=0;k<8;k++)
    {
        if((test_err = bma2xx_i2c_read_block(client, k*8, test_buf, 8))<0)
        {
            printk("ERROR : %d \n", test_err);
        }
        mdelay(1);
        for(j=0;j<8;j++)
        {
            printk("REG:[0x%x]------->VAL:[0x%x]\n", (k*8+j), test_buf[j]);
        }
    }
#endif    
//add end

}

/*----------------------------------------------------------------------------*/
#if 1
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = bma2xx_i2c_client;
	struct bma2xx_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[BMA2XX_AXES_NUM];

	if(NULL == client)
	{
		miki_gs_err("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	if((err = BMA2XX_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if((err = BMA2XX_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		if(bma250_flag==1)   
		{
		mul = obj->reso->sensitivity/bma250_offset_resolution.sensitivity;
		}
		else
		mul = obj->reso->sensitivity/bma2xx_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
				obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z],
				obj->offset[BMA2XX_AXIS_X], obj->offset[BMA2XX_AXIS_Y], obj->offset[BMA2XX_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
				obj->cali_sw[BMA2XX_AXIS_X], obj->cali_sw[BMA2XX_AXIS_Y], obj->cali_sw[BMA2XX_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
				obj->offset[BMA2XX_AXIS_X]*mul + obj->cali_sw[BMA2XX_AXIS_X],
				obj->offset[BMA2XX_AXIS_Y]*mul + obj->cali_sw[BMA2XX_AXIS_Y],
				obj->offset[BMA2XX_AXIS_Z]*mul + obj->cali_sw[BMA2XX_AXIS_Z],
				tmp[BMA2XX_AXIS_X], tmp[BMA2XX_AXIS_Y], tmp[BMA2XX_AXIS_Z]);

		return len;
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma2xx_i2c_client;  
	int err, x, y, z;
	int dat[BMA2XX_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if((err = BMA2XX_ResetCalibration(client)))
		{
			miki_gs_err("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[BMA2XX_AXIS_X] = x;
		dat[BMA2XX_AXIS_Y] = y;
		dat[BMA2XX_AXIS_Z] = z;
		if((err = BMA2XX_WriteCalibration(client, dat)))
		{
			miki_gs_err("write calibration err = %d\n", err);
		}		
	}
	else
	{
		miki_gs_err("invalid format\n");
	}

	return count;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_BMA2XX_LOWPASS
	struct i2c_client *client = bma2xx_i2c_client;
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][BMA2XX_AXIS_X], obj->fir.raw[idx][BMA2XX_AXIS_Y], obj->fir.raw[idx][BMA2XX_AXIS_Z]);
		}

		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[BMA2XX_AXIS_X], obj->fir.sum[BMA2XX_AXIS_Y], obj->fir.sum[BMA2XX_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[BMA2XX_AXIS_X]/len, obj->fir.sum[BMA2XX_AXIS_Y]/len, obj->fir.sum[BMA2XX_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

#if defined(MIKI_YUANZHENG_SUPPORT)
/*----------------------------------------------------------------------------*/
static ssize_t store_slope_threshold_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = bma2xx_i2c_client;  
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	unsigned char threshold;
	unsigned char data;
	int err;
	if(1 != sscanf(buf, "%d", &threshold))
	{
		GSE_ERR("invallid format\n");
	}
	
	err = bma2x2_set_slope_threshold(client, threshold);	
	if(err <0)
	{
		printk("store_slope_threshold_value err\n");
			}
		
	err = bma2xx_i2c_read_block(client,BMA2X2_SLOPE_THRES_REG,&data, 0x01);
    if(err < 0)
    {
   		printk("bma2x2_set_slope_threshold read err\n"); 
	}
	printk("liuhuan-------read the slope threshold: %d ---\n",data);	
	
	return count;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_BMA2XX_LOWPASS
	struct i2c_client *client = bma2xx_i2c_client;  
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		miki_gs_err("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		miki_gs_err("i2c_data obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		miki_gs_err("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;    
}
/*-----------------------------------------------------------------------------*/
//add by liuhuan for test
static ssize_t bma250_softreset_init(struct device_driver *ddri, char *buf, size_t count)
{ 
    struct i2c_client *client = bma2xx_i2c_client;
    int err=0; 
    u8 rst_val=0xB6;
    if(!strncmp(buf, "rst", 3))  
    {
        if ((err = bma2xx_i2c_write_block(client, BMA2XX_SOFT_RESET , &rst_val, 1)) <0)
        {
            miki_gs_err("bma250 soft reset error: %d \n", err); 
            return err; 
        }

        GSE_LOG("bma250 soft reset write fuction return %d \n", err);


        mdelay(10);//add by liuhuan

        struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);        
        BMA2XX_power(obj->hw, 1);
                
        if((err = bma2xx_init_client(client, 0)))
        {
            miki_gs_err("initialize client fail!!\n");
            return 0;        
        }
    
        atomic_set(&obj->suspend, 0);    

        GSE_LOG("bma250 soft reset bma2xx init ok  %d \n", err);
    }

    return count;
}
//add end

static ssize_t bma250_softreset(struct device_driver *ddri, char *buf, size_t count)
{ 
    struct i2c_client *client = bma2xx_i2c_client;
    int err=0; 
    u8 rst_val=0xB6;
    if(!strncmp(buf, "rst", 3))  
    {
        if((err = bma2xx_i2c_write_block(client,BMA2XX_SOFT_RESET ,&rst_val, 1))< 0) 
        {
            miki_gs_err("bma250 soft reset error: %d \n", err); 
            return err; 
        }
    
        GSE_LOG("bma250 soft reset write fuction return %d \n", err);
    }

    return 0;
}  
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct bma2xx_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	

	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
				obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	struct bma2xx_i2c_data *obj = obj_i2c_data;

	if(atomic_read(&obj->sensor_power))
		printk("G sensor is in work mode, sensor_power = %d\n", atomic_read(&obj->sensor_power));
	else
		printk("G sensor is in standby mode, sensor_power = %d\n", atomic_read(&obj->sensor_power));

	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_power_mode_value(struct device_driver *ddri, char *buf, size_t count)
{
	unsigned long data;
	int error;
	u8 reg_value;
	struct bma2xx_i2c_data *obj = obj_i2c_data;

	if (obj == NULL)
	{
		miki_gs_err("i2c_data obj is null!!\n");
		return 0;
	}	

	error = strict_strtoul(buf, 10, &data);
	if(error)
		return error;

	if((data == 0) ||(data == 1)){
		if(bma2xx_i2c_read_block(obj->client, BMA2XX_REG_POWER_CTL, &reg_value, 0x01)<0)
		{
			miki_gs_err("read power ctl register err!\n");
			return BMA2XX_ERR_I2C;
		}

		if(data == 1)/*Normal Mode*/
		{
			reg_value &= ~BMA2XX_MEASURE_MODE;
		}
		else/*Suspend Mode*/
		{
			reg_value |= BMA2XX_MEASURE_MODE;
		}

		if(bma2xx_i2c_write_block(obj->client, BMA2XX_REG_POWER_CTL, &reg_value, 0x01)<0)
		{
			miki_gs_err("write power ctl register err!\n");
			return BMA2XX_ERR_I2C;
		}

		atomic_set(&obj->sensor_power, data);
		mdelay(20);
	}

	return count;
}

//add by liuhuan for test
#if defined(MIKI_YUANZHENG_SUPPORT)
static ssize_t upmu_test(struct device_driver *ddri, char *buf, size_t count)
{
	int temp;
	if(!strncmp(buf, "uart1", 5))  
	{
		mt_set_gpio_mode(101,GPIO_MODE_01);
		mt_set_gpio_dir(101,GPIO_DIR_IN);
		//mt_set_gpio_out(GPIO83,GPIO_OUT_ZERO);		
		mt_set_gpio_pull_enable(101, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(101, GPIO_PULL_UP); 

		mt_set_gpio_mode(102,GPIO_MODE_01);
		mt_set_gpio_dir(102,GPIO_DIR_OUT);
		mt_set_gpio_out(102,GPIO_OUT_ONE);		
		mt_set_gpio_pull_enable(102, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(102, GPIO_PULL_UP); 

	}

	if(!strncmp(buf, "urtest", 6))  
	{
		temp=mt_get_gpio_mode(101);
		printk("<0>""liuhuan--get-mode: %d\n",temp);
		temp=mt_get_gpio_dir(101);
		printk("<0>""liuhuan--get-dir: %d\n",temp);
		temp=mt_get_gpio_pull_enable(101);
		printk("<0>""liuhuan--get-pull-enable: %d\n",temp);
		temp=mt_get_gpio_pull_select(101);
		printk("<0>""liuhuan--get-select: %d\n",temp);
		temp=mt_get_gpio_out(101);
		printk("<0>""liuhuan--get-out: %d\n",temp);
			
		temp=mt_get_gpio_mode(102);
		printk("<0>""liuhuan--get-mode: %d\n",temp);
		temp=mt_get_gpio_dir(102);
		printk("<0>""liuhuan--get-dir: %d\n",temp);
		temp=mt_get_gpio_pull_enable(102);
		printk("<0>""liuhuan--get-pull-enable: %d\n",temp);
		temp=mt_get_gpio_pull_select(102);
		printk("<0>""liuhuan--get-select: %d\n",temp);
		temp=mt_get_gpio_out(102);
		printk("<0>""liuhuan--get-out: %d\n",temp);
	}
	
	
	
	if(!strncmp(buf, "poff", 4))  
	{
            upmu_set_isink_ch0_en(0x0); // Turn on ISINK Channel 0
            upmu_set_isink_ch1_en(0x0); // Turn on ISINK Channel 0
            upmu_set_isink_ch2_en(0x0); // Turn on ISINK Channel 0
            upmu_set_isink_ch3_en(0x0); // Turn on ISINK Channel 0
			}
	if(!strncmp(buf, "test0", 5))  
	{
            upmu_set_rg_isink0_ck_pdn(0x0); // Disable power down    
            upmu_set_rg_isink0_ck_sel(0x0); // Freq = 32KHz for Indicator     
            upmu_set_isink_dim0_duty(15); // 16/32
			upmu_set_isink_ch0_mode(1);
            upmu_set_isink_dim0_fsel(0); // 1K = 32000 / (0 + 1) / 32
            upmu_set_isink_ch0_step(0x0); // 4mA
            upmu_set_isink_sfstr0_tc(0x0); // 0.5us
            upmu_set_isink_sfstr0_en(0x0); // Disable soft start
			upmu_set_isink_breath0_trf_sel(0x04); // 0.926s
			upmu_set_isink_breath0_ton_sel(0x02); // 0.523s
			upmu_set_isink_breath0_toff_sel(0x03); // 1.417s
			upmu_set_rg_isink0_double_en(0x0); // Disable double current
			upmu_set_isink_phase0_dly_en(0x0); // Disable phase delay
            upmu_set_isink_chop0_en(0x0); // Disable CHOP clk
            upmu_set_isink_ch0_en(0x1); // Turn on ISINK Channel 0
			
			}
	if(!strncmp(buf, "test1", 5))  
	{
            upmu_set_rg_isink1_ck_pdn(0x0); // Disable power down    
            upmu_set_rg_isink1_ck_sel(0x0); // Freq = 32KHz for Indicator            
            upmu_set_isink_dim1_duty(15); // 16/32
			upmu_set_isink_ch1_mode(1);
            upmu_set_isink_dim1_fsel(0); // 1K = 32000 / (0 + 1) / 32
            upmu_set_isink_ch1_step(0x0); // 4mA
            upmu_set_isink_sfstr1_tc(0x0); // 0.5us
            upmu_set_isink_sfstr1_en(0x0); // Disable soft start
			upmu_set_isink_breath1_trf_sel(0x04); // 0.926s
			upmu_set_isink_breath1_ton_sel(0x02); // 0.523s
			upmu_set_isink_breath1_toff_sel(0x03); // 1.417s
			upmu_set_rg_isink1_double_en(0x0); // Disable double current
			upmu_set_isink_phase1_dly_en(0x0); // Disable phase delay
            upmu_set_isink_chop1_en(0x0); // Disable CHOP clk
            upmu_set_isink_ch1_en(0x1); // Turn on ISINK Channel 1        
			
			}
	if(!strncmp(buf, "test2", 5))  
	{
            upmu_set_rg_isink2_ck_pdn(0x0); // Disable power down    
            upmu_set_rg_isink2_ck_sel(0x0); // Freq = 32KHz for Indicator            
            upmu_set_isink_dim2_duty(15); // 16/32
			upmu_set_isink_ch2_mode(1);
            upmu_set_isink_dim2_fsel(0); // 1K = 32000 / (0 + 1) / 32
            upmu_set_isink_ch2_step(0x0); // 4mA
            upmu_set_isink_sfstr2_tc(0x0); // 0.5us
            upmu_set_isink_sfstr2_en(0x0); // Disable soft start
			upmu_set_isink_breath2_trf_sel(0x04); // 0.926s
			upmu_set_isink_breath2_ton_sel(0x02); // 0.523s
			upmu_set_isink_breath2_toff_sel(0x03); // 1.417s
			upmu_set_rg_isink2_double_en(0x0); // Disable double current
			upmu_set_isink_phase2_dly_en(0x0); // Disable phase delay
            upmu_set_isink_chop2_en(0x0); // Disable CHOP clk
            upmu_set_isink_ch2_en(0x1); // Turn on ISINK Channel 2

	}
	if(!strncmp(buf, "test3", 5))  
	{
            upmu_set_rg_isink3_ck_pdn(0x0); // Disable power down    
            upmu_set_rg_isink3_ck_sel(0x0); // Freq = 32KHz for Indicator            
            upmu_set_isink_dim3_duty(15); // 16/32
			upmu_set_isink_ch3_mode(1);
            upmu_set_isink_dim3_fsel(0); // 1K = 32000 / (0 + 1) / 32
            upmu_set_isink_ch3_step(0x0); // 4mA
            upmu_set_isink_sfstr3_tc(0x0); // 0.5us
            upmu_set_isink_sfstr3_en(0x0); // Disable soft start
			upmu_set_isink_breath3_trf_sel(0x04); // 0.926s
			upmu_set_isink_breath3_ton_sel(0x02); // 0.523s
			upmu_set_isink_breath3_toff_sel(0x03); // 1.417s
			upmu_set_rg_isink3_double_en(0x0); // Disable double current
			upmu_set_isink_phase3_dly_en(0x0); // Disable phase delay
            upmu_set_isink_chop3_en(0x0); // Disable CHOP clk
            upmu_set_isink_ch3_en(0x1); // Turn on ISINK Channel 3
			
			}
	if(!strncmp(buf, "pwm2", 4))  
	{
            upmu_set_rg_isink2_ck_pdn(0x0); // Disable power down    
            upmu_set_rg_isink2_ck_sel(0x0); // Freq = 32KHz for Indicator            
            upmu_set_isink_dim2_duty(15); // 16/32
			upmu_set_isink_ch2_mode(0);
            upmu_set_isink_dim2_fsel(0); // 1K = 32000 / (0 + 1) / 32
            upmu_set_isink_ch2_step(0x0); // 4mA
            upmu_set_isink_sfstr2_tc(0x0); // 0.5us
            upmu_set_isink_sfstr2_en(0x0); // Disable soft start
			upmu_set_isink_breath2_trf_sel(0x04); // 0.926s
			upmu_set_isink_breath2_ton_sel(0x02); // 0.523s
			upmu_set_isink_breath2_toff_sel(0x03); // 1.417s
			upmu_set_rg_isink2_double_en(0x0); // Disable double current
			upmu_set_isink_phase2_dly_en(0x0); // Disable phase delay
            upmu_set_isink_chop2_en(0x0); // Disable CHOP clk
            upmu_set_isink_ch2_en(0x1); // Turn on ISINK Channel 2

	}
	if(!strncmp(buf, "reg3", 4))  
	{
            upmu_set_rg_isink3_ck_pdn(0x0); // Disable power down    
            upmu_set_rg_isink3_ck_sel(0x0); // Freq = 32KHz for Indicator            
            upmu_set_isink_dim3_duty(15); // 16/32
			upmu_set_isink_ch3_mode(2);
            upmu_set_isink_dim3_fsel(0); // 1K = 32000 / (0 + 1) / 32
            upmu_set_isink_ch3_step(0x0); // 4mA
            upmu_set_isink_sfstr3_tc(0x0); // 0.5us
            upmu_set_isink_sfstr3_en(0x0); // Disable soft start
			upmu_set_isink_breath3_trf_sel(0x04); // 0.926s
			upmu_set_isink_breath3_ton_sel(0x02); // 0.523s
			upmu_set_isink_breath3_toff_sel(0x03); // 1.417s
			upmu_set_rg_isink3_double_en(0x0); // Disable double current
			upmu_set_isink_phase3_dly_en(0x0); // Disable phase delay
            upmu_set_isink_chop3_en(0x0); // Disable CHOP clk
            upmu_set_isink_ch3_en(0x1); // Turn on ISINK Channel 3
			
	}
}
#endif

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(regs, S_IWUSR | S_IRUGO, show_regs_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);
static DRIVER_ATTR(power_mode,               S_IWUSR, NULL,        store_power_mode_value);
static DRIVER_ATTR(softreset,  S_IWUSR | S_IRUGO, NULL,    bma250_softreset);
static DRIVER_ATTR(softreset_init, S_IWUSR | S_IRUGO, NULL, bma250_softreset_init);
#if defined(MIKI_YUANZHENG_SUPPORT)
static DRIVER_ATTR(upmu_test, S_IWUSR | S_IRUGO, NULL, upmu_test);
static DRIVER_ATTR(threshold, S_IWUSR | S_IRUGO, NULL, store_slope_threshold_value);
#endif
/*----------------------------------------------------------------------------*/
static struct driver_attribute *bma2xx_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_regs,
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_power_mode,/*ONLY for debug usage*/
    &driver_attr_softreset, /*soft reset*/
    &driver_attr_softreset_init,
#if defined(MIKI_YUANZHENG_SUPPORT)
    &driver_attr_upmu_test,
    &driver_attr_threshold,
#endif
};
/*----------------------------------------------------------------------------*/
static int bma2xx_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(bma2xx_attr_list)/sizeof(bma2xx_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, bma2xx_attr_list[idx])))
		{            
			miki_gs_err("driver_create_file (%s) = %d\n", bma2xx_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(bma2xx_attr_list)/sizeof(bma2xx_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, bma2xx_attr_list[idx]);
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct bma2xx_i2c_data *priv = (struct bma2xx_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[BMA2XX_BUFSIZE];
	bool power_mode;

	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				miki_gs_err("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					sample_delay = BMA2XX_BW_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = BMA2XX_BW_100HZ;
				}
				else
				{
					sample_delay = BMA2XX_BW_50HZ;
				}

				err = BMA2XX_SetBWRate(priv->client, sample_delay);
				if(err != BMA2XX_SUCCESS ) //0x2C->BW=100Hz
				{
					miki_gs_err("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{	
#if defined(CONFIG_BMA2XX_LOWPASS)
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[BMA2XX_AXIS_X] = 0;
					priv->fir.sum[BMA2XX_AXIS_Y] = 0;
					priv->fir.sum[BMA2XX_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
#endif
				}
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				miki_gs_err("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				power_mode = (bool)atomic_read(&priv->sensor_power);
				if(((value == 0) && (power_mode == false)) ||((value == 1) && (power_mode == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
				#if defined(MIKI_YUANZHENG_SUPPORT)
				#else					
					err = BMA2XX_SetPowerMode( priv->client, !power_mode);
				#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				miki_gs_err("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				BMA2XX_ReadSensorData(priv->client, buff, BMA2XX_BUFSIZE);
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
						&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
			}
			break;
		default:
			miki_gs_err("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/****************************************************************************** 
 * Function Configuration
 ******************************************************************************/
static int bma2xx_open(struct inode *inode, struct file *file)
{
	file->private_data = bma2xx_i2c_client;

	if(file->private_data == NULL)
	{
		miki_gs_err("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int bma2xx_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int bma2xx_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long bma2xx_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct bma2xx_i2c_data *obj = (struct bma2xx_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[BMA2XX_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	#if defined(MIKI_YUANZHENG_SUPPORT)
	unsigned char slope_threshold;
	#endif

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		miki_gs_err("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		#if defined(MIKI_YUANZHENG_SUPPORT)
		case GSENSOR_IOCTL_SET_SLOPE_THRESHOLD:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&slope_threshold, data, sizeof(unsigned char)))
			{
				err = -EFAULT;
				printk("liuhuan -------GSENSOR_IOCTL_SET_SLOPE_THRESHOLD--copy_from_user ---err\n");
				break;	  
			}
		
			err=bma2x2_set_slope_threshold(client, slope_threshold);
		    if(err < 0)
		    {
        		miki_gs_err("BMA2XX disable interrupt function error---\n");
		    	break;
			}
		
			printk("liuhuan--------------GSENSOR_IOCTL_SET_SLOPE_THRESHOLD--ok\n");	
			
			break;
		
		
		case GSENSOR_IOCTL_GET_SLOPE_THRESHOLD:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			err = bma2xx_i2c_read_block(client,BMA2X2_SLOPE_THRES_REG,&slope_threshold, 0x01);
		    if(err < 0)
		    {
		   		printk("bma2x2_set_slope_threshold read err\n"); 
				break;
			}
			
			printk("liuhuan--------GSENSOR_IOCTL_GET_SLOPE_THRESHOLD----threshold: %d--\n",slope_threshold);

			if(copy_to_user(data, &slope_threshold, sizeof(unsigned char)))
			{
				err = -EFAULT;
				printk("liuhuan-------GSENSOR_IOCTL_GET_SLOPE_THRESHOLD---err\n");
				break;
			}				 
			
			printk("liuhuan-------GSENSOR_IOCTL_GET_SLOPE_THRESHOLD---ok\n");

			break;	  
		
		
		
		#endif
		case GSENSOR_IOCTL_INIT:
			bma2xx_init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			BMA2XX_ReadChipInfo(client, strbuf, BMA2XX_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			BMA2XX_ReadSensorData(client, strbuf, BMA2XX_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			

			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			BMA2XX_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				miki_gs_err("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[BMA2XX_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA2XX_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[BMA2XX_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = BMA2XX_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = BMA2XX_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if((err = BMA2XX_ReadCalibration(client, cali)))
			{
				break;
			}

			sensor_data.x = cali[BMA2XX_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[BMA2XX_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[BMA2XX_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;

		default:
			miki_gs_err("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;

	}

	return err;
}

/*----------------------------------------------------------------------------*/
static struct file_operations bma2xx_fops = {
	.owner = THIS_MODULE,
	.open = bma2xx_open,
	.release = bma2xx_release,
	.unlocked_ioctl = bma2xx_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice bma2xx_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &bma2xx_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int bma2xx_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	GSE_FUN();    
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
		
		if((err = BMA2XX_SetPowerMode(obj->client, false)))
		{
			GSE_ERR("write power control fail!!\n");
			return;
		}       
		BMA2XX_power(obj->hw, 0);
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_resume(struct i2c_client *client)
{
	struct bma2xx_i2c_data *obj = i2c_get_clientdata(client);        
	int err;

	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	BMA2XX_power(obj->hw, 1);
	        
	if((err = bma2xx_init_client(client, 0)))
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void bma2xx_early_suspend(struct early_suspend *h) 
{
#if defined(MIKI_YUANZHENG_SUPPORT)
	//printk("bma2xx_early_suspend---1\n");
//	mt65xx_eint_unmask(CUST_EINT_GSE_2_NUM);
#else
	struct bma2xx_i2c_data *obj = container_of(h, struct bma2xx_i2c_data, early_drv);   
	int err;
//	printk("bma2xx_early_suspend---2\n");
	GSE_FUN();   
 
	if(obj == NULL)
	{
		miki_gs_err("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	
	if((err = BMA2XX_SetPowerMode(obj->client, false)))
	{
		miki_gs_err("write power control fail!!\n");
		return;
	}

	atomic_set(&obj->sensor_power, 0);

	BMA2XX_power(obj->hw, 0);
#endif
}
/*----------------------------------------------------------------------------*/
static void bma2xx_late_resume(struct early_suspend *h)
{
#if defined(MIKI_YUANZHENG_SUPPORT)
//	printk("bma2xx_late_resume--------\n");
//	mt65xx_eint_mask(CUST_EINT_GSE_2_NUM);		
#else
	struct bma2xx_i2c_data *obj = container_of(h, struct bma2xx_i2c_data, early_drv);         
	int err;
	
	GSE_FUN();
	
	
	if(obj == NULL)
	{
		miki_gs_err("null pointer!!\n");
		return;
	}
	
	BMA2XX_power(obj->hw, 1);
	if((err = bma2xx_init_client(obj->client, 0)))
	{
		miki_gs_err("initialize client fail!!\n");
		return;        
	}
	atomic_set(&obj->suspend, 0);    
#endif
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, BMA2XX_DEV_NAME);
	return 0;
}
#if defined(MIKI_YUANZHENG_SUPPORT)
void gsensor_interrupt_handler(void)
{
	printk("liuhuan -----gsensor INT triggered--------\n");
	mt65xx_eint_mask(CUST_EINT_GSE_2_NUM);
	
}
#endif

/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct bma2xx_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;

	GSE_FUN();
	
	mutex_init(&i2c_data_mutex);

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct bma2xx_i2c_data));

	obj->hw = bma2xx_get_cust_acc_hw();

	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		miki_gs_err("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	atomic_set(&obj->sensor_power,  1);

#ifdef CONFIG_BMA2XX_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}

	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}

#endif

	bma2xx_i2c_client = new_client;	

	if((err = bma2xx_init_client(new_client, 1)))
	{
		goto exit_init_failed;
	}


	if((err = misc_register(&bma2xx_device)))
	{
		miki_gs_err("bma2xx_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = bma2xx_create_attr(&(bma2xx_init_info.platform_diver_addr->driver)))<0)
	{
		miki_gs_err("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	sobj.self = obj;
	sobj.polling = 1;
	sobj.sensor_operate = gsensor_operate;
	if((err = hwmsen_attach(ID_ACCELEROMETER, &sobj)))
	{
		miki_gs_err("attach fail = %d\n", err);
		goto exit_kfree;
	}




#if defined(MIKI_YUANZHENG_SUPPORT)
	mt_set_gpio_mode(GPIO_GSE_2_EINT_PIN, GPIO_GSE_2_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_GSE_2_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GSE_2_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_GSE_2_EINT_PIN, GPIO_PULL_DOWN);

	mt65xx_eint_set_sens(CUST_EINT_GSE_2_NUM, CUST_EINT_GSE_2_SENSITIVE);
	mt65xx_eint_registration(CUST_EINT_GSE_2_NUM,CUST_EINT_GSE_2_DEBOUNCE_EN,CUST_EINT_GSE_2_POLARITY, gsensor_interrupt_handler, 0);
	mt65xx_eint_unmask(CUST_EINT_GSE_2_NUM);
#endif





#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
		obj->early_drv.suspend  = bma2xx_early_suspend,
		obj->early_drv.resume   = bma2xx_late_resume,    
		register_early_suspend(&obj->early_drv);
#endif 

	GSE_LOG("%s: OK\n", __func__);    
    bma2xx_init_flag = 0;
	return 0;

exit_create_attr_failed:
	misc_deregister(&bma2xx_device);
exit_misc_device_register_failed:
exit_init_failed:
	//i2c_detach_client(new_client);
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
    bma2xx_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int bma2xx_local_init(void) 
{
	struct acc_hw *hw = bma2xx_get_cust_acc_hw();
	GSE_FUN();

	BMA2XX_power(hw, 1);
//	kxtj2_force[0] = hw->i2c_num;
	if(i2c_add_driver(&bma2xx_i2c_driver))
	{
		miki_gs_err("add driver error\n");
		return -1;
	}
	if(-1 == bma2xx_init_flag)
	{
	   return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_i2c_remove(struct i2c_client *client)
{
    int err = 0;	

    if((err = bma2xx_delete_attr(&(bma2xx_init_info.platform_diver_addr->driver)))<0)
    {
        miki_gs_err("bma150_delete_attr fail: %d\n", err);
    }

    if((err = misc_deregister(&bma2xx_device))<0)
    {
        miki_gs_err("misc_deregister fail: %d\n", err);
    }

    if((err = hwmsen_detach(ID_ACCELEROMETER))<0)
    {
        bma2xx_i2c_client = NULL;
    }
    
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*----------------------------------------------------------------------------*/
static int bma2xx_remove(void)
{
    struct acc_hw *hw = bma2xx_get_cust_acc_hw();

    GSE_FUN();    
    BMA2XX_power(hw, 0);    
    i2c_del_driver(&bma2xx_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
/*static struct platform_driver bma2xx_gsensor_driver = {
	.probe      = bma2xx_probe,
	.remove     = bma2xx_remove,    
	.driver     = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};
*/
/*----------------------------------------------------------------------------*/
static int __init bma2xx_init(void)
{
	GSE_FUN();
	struct acc_hw *hw = bma2xx_get_cust_acc_hw();
	GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_BMA2XX, 1);
	hwmsen_gsensor_add(&bma2xx_init_info);
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit bma2xx_exit(void)
{
	GSE_FUN();
}


// 0-->success
// negative value-->fail
static int bma2xx_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    int err = 0;

    if (NULL==client)
    {
        return -1;
    }
    
    mutex_lock(&i2c_data_mutex);
    err = hwmsen_read_block(client, addr, data, len);
    mutex_unlock(&i2c_data_mutex);
    
    return err;
}

// 0-->success
// negative value-->fail
static int bma2xx_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    int err = 0;

    if (NULL==client)
    {
        return -1;
    }
    
    mutex_lock(&i2c_data_mutex);
    err = hwmsen_write_block(client, addr, data, len);
    mutex_unlock(&i2c_data_mutex);
    
    return err;
}

/*----------------------------------------------------------------------------*/
module_init(bma2xx_init);
module_exit(bma2xx_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMA2XX I2C driver");
MODULE_AUTHOR("Hongji.Zhou@cn.bosch.com");
