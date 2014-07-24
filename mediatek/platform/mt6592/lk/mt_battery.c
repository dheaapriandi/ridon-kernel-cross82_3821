#include <target/board.h>
#ifdef MTK_KERNEL_POWER_OFF_CHARGING
#define CFG_POWER_CHARGING
#endif
#ifdef CFG_POWER_CHARGING
#include <platform/mt_typedefs.h>
#include <platform/mt_reg_base.h>
#include <platform/mt_pmic.h>
#include <platform/boot_mode.h>
#include <platform/mt_gpt.h>
#include <platform/mt_rtc.h>
#include <platform/mt_disp_drv.h>
#include <platform/mtk_wdt.h>
#include <platform/mtk_key.h>
#include <platform/mt_logo.h>
#include <platform/mt_leds.h>
#include <printf.h>
#include <sys/types.h>
#include <target/cust_battery.h>

//#if 0 
#ifdef SUPPORT_TINNO_BATTERY_COMPENSATE	   
#include <platform/mt_gpio.h>
#include <cust_gpio_usage.h>
int gpio_number_temp   = GPIO_SWCHARGER_EN_PIN; 
int gpio_off_mode_temp = GPIO_SWCHARGER_EN_PIN_M_GPIO;
int gpio_on_mode_temp  = GPIO_SWCHARGER_EN_PIN_M_GPIO;

int gpio_off_dir_temp  = GPIO_DIR_OUT;
int gpio_off_out_temp  = GPIO_OUT_ONE;
int gpio_on_dir_temp   = GPIO_DIR_OUT;
int gpio_on_out_temp   = GPIO_OUT_ZERO;
#endif

#undef printf


/*****************************************************************************
 *  Type define
 ****************************************************************************/
#define BATTERY_LOWVOL_THRESOLD             3450


/*****************************************************************************
 *  Global Variable
 ****************************************************************************/
bool g_boot_reason_change = false;


/*****************************************************************************
 *  Externl Variable
 ****************************************************************************/
extern bool g_boot_menu;

#ifdef MTK_FAN5405_SUPPORT
extern void fan5405_hw_init(void);
extern void fan5405_turn_on_charging(void);
extern void fan5405_dump_register(void);
#endif

#ifdef MTK_BQ24158_SUPPORT
extern void bq24158_hw_init(void);
extern void bq24158_turn_on_charging(void);
extern void bq24158_dump_register(void);
//add by alik
extern UINT32 g_battery_voltage;
//add end

#if 0
//#ifdef SUPPORT_TINNO_BATTERY_COMPENSATE	   
//add by alik
UINT32 g_battery_voltage=4000;
#define AVR_TIMES  10
#define  LK_LOW_CHARGE_VOL    3300
#define  LK_0_PERCENT_VOL    3450
extern void pchr_turn_off_charging_bq24158(void);
extern unsigned int bq24158_config_interface_reg (unsigned char RegNum, unsigned char val);

//add end
#endif

#endif
#ifdef MTK_BQ24196_SUPPORT
extern void bq24196_hw_init(void);
extern void bq24196_charging_enable(kal_uint32 bEnable);
extern void bq24196_dump_register(void);
extern kal_uint32 bq24196_get_chrg_stat(void);
#endif

void kick_charger_wdt(void)
{
    upmu_set_rg_chrwdt_td(0x0);           // CHRWDT_TD, 4s
    upmu_set_rg_chrwdt_wr(1); 			  // CHRWDT_WR
    upmu_set_rg_chrwdt_int_en(1);         // CHRWDT_INT_EN
    upmu_set_rg_chrwdt_en(1);             // CHRWDT_EN
    upmu_set_rg_chrwdt_flag_wr(1);        // CHRWDT_WR
}

kal_bool is_low_battery(kal_uint32 val)
{
    #ifdef MTK_BQ24196_SUPPORT
    kal_uint32 bq24196_chrg_status;
    
    if(0 == val)
        val = get_i_sense_volt(5);
    #endif
    
    if (val < BATTERY_LOWVOL_THRESOLD)
    {
        dprintf(INFO, "%s, TRUE\n", __FUNCTION__);
        return KAL_TRUE;
    }
    else
    {
        #ifdef MTK_BQ24196_SUPPORT
        bq24196_chrg_status = bq24196_get_chrg_stat();
        dprintf(INFO, "bq24196_chrg_status = %d\n", bq24196_chrg_status);
    
        if(bq24196_chrg_status == 0x1) //Pre-charge
        {
            dprintf(INFO, "%s, battery protect TRUE\n", __FUNCTION__);
            return KAL_TRUE;
        }  
        #endif
    }
    
    dprintf(INFO, "%s, FALSE\n", __FUNCTION__);
    return KAL_FALSE;
}

void pchr_turn_on_charging (void)
{
	upmu_set_rg_usbdl_set(0);        //force leave USBDL mode
	upmu_set_rg_usbdl_rst(1);		//force leave USBDL mode
	
	kick_charger_wdt();
	
	upmu_set_rg_cs_vth(0xC);    	// CS_VTH, 450mA            
	upmu_set_rg_csdac_en(1);                // CSDAC_EN
	upmu_set_rg_chr_en(1);                  // CHR_EN  

#ifdef MTK_FAN5405_SUPPORT
	fan5405_hw_init();
	fan5405_turn_on_charging();
	fan5405_dump_register();
#endif

#ifdef MTK_BQ24158_SUPPORT
    bq24158_hw_init();
    bq24158_turn_on_charging();
    bq24158_dump_register();
#endif

#ifdef MTK_BQ24196_SUPPORT
	bq24196_hw_init();
	bq24196_charging_enable(0);  //disable charging with power path
	bq24196_dump_register();
#endif
}



#if 0
//#ifdef SUPPORT_TINNO_BATTERY_COMPENSATE	   

void Low_battery_chg()
{
#ifndef GPT_TIMER
	long tmo;
	long tmo2;
#endif

	bq24158_hw_init();
	mt_set_gpio_mode(gpio_number_temp,gpio_on_mode_temp);
	mt_set_gpio_dir(gpio_number_temp,gpio_on_dir_temp);
	mt_set_gpio_out(gpio_number_temp,gpio_on_out_temp);
	while(1)
	{
		kick_charger_wdt();
		bq24158_config_interface_reg(0x01,0x38|0x40);
		 //if battery is low ,we charging 2 senconds at first.
		#ifdef GPT_TIMER                        
		mtk_sleep(2000, KAL_TRUE);
		#else
		tmo2 = get_timer(0);            
		while(get_timer(tmo2) <= 2000 /* ms */);                    
		#endif 
		if ((upmu_is_chr_det() != KAL_TRUE)) /*power off */
		{
#ifndef NO_POWER_OFF
			mt6575_power_off();
#endif			
			while(1)
			{
				printf("If you see the log, please check with RTC power off API\n\r");
			}
		}
		else
		{
			if(get_bat_sense_volt(1)>=LK_LOW_CHARGE_VOL)
			{
				return;
			}
		}
			
	}
}
#endif

void mt65xx_bat_init(void)
{    
		kal_int32 bat_vol;

#if 0		
//#ifdef SUPPORT_TINNO_BATTERY_COMPENSATE
#ifndef GPT_TIMER
	long tmo;
	long tmo2;
#endif

    int i=0;
    int temp_voltage=0;
    int ret =0;
    // Low Battery Safety Booting
       bat_vol = get_bat_sense_volt(1);
     if(bat_vol<LK_LOW_CHARGE_VOL)
     { 
		if ((upmu_is_chr_det() == KAL_TRUE))
		{
			Low_battery_chg();
		}	
      }
     if((upmu_is_chr_det() == KAL_TRUE))
     	{
	       bat_vol = get_bat_sense_volt(1);
	     if(bat_vol>LK_0_PERCENT_VOL)
	     {
		 bq24158_hw_init(); 
		 pchr_turn_off_charging_bq24158();
		#ifdef GPT_TIMER                        
		mtk_sleep(1000, KAL_TRUE);
		#else
		tmo2 = get_timer(0);            
		while(get_timer(tmo2) <= 1000 /* ms */);                    
		#endif 
		
		for(i=0;i<AVR_TIMES;i++)
		{
			temp_voltage=temp_voltage+get_bat_sense_volt(2);
		}
		g_battery_voltage=temp_voltage/AVR_TIMES;
	     }else{
			g_battery_voltage= LK_0_PERCENT_VOL;
		 }
        }else{
			for(i=0;i<AVR_TIMES;i++)
			{
				temp_voltage=temp_voltage+get_bat_sense_volt(2);
			}
			g_battery_voltage=temp_voltage/AVR_TIMES;
		}

dprintf(INFO,"check VBAT=%d mV with %d  g_battery_voltage= %d mV\n", bat_vol, BATTERY_LOWVOL_THRESOLD,g_battery_voltage);
		
#endif
//{=========add by alik begin============
if(g_boot_mode!=KERNEL_POWER_OFF_CHARGING_BOOT)
{
	  if((upmu_is_chr_det() == KAL_TRUE))
	  {
		g_battery_voltage=g_battery_voltage-10;
	  }
}
//add end
		// Low Battery Safety Booting
		
		bat_vol = get_bat_sense_volt(1);

		#ifdef MTK_BQ24196_SUPPORT
		bat_vol = get_i_sense_volt(5);
		#endif

		dprintf(INFO, "[mt65xx_bat_init] check VBAT=%d mV with %d mV\n", bat_vol, BATTERY_LOWVOL_THRESOLD);
		
		pchr_turn_on_charging();

		if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT && (upmu_get_pwrkey_deb()==0) ) {
				dprintf(INFO, "[mt65xx_bat_init] KPOC+PWRKEY => change boot mode\n");		
		
				g_boot_reason_change = true;
		}
		rtc_boot_check(false);

	#ifndef MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
    //if (bat_vol < BATTERY_LOWVOL_THRESOLD)
    if (is_low_battery(bat_vol))
    {
        if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT && upmu_is_chr_det() == KAL_TRUE)
        {
            dprintf(INFO, "[%s] Kernel Low Battery Power Off Charging Mode\n", __func__);
            g_boot_mode = LOW_POWER_OFF_CHARGING_BOOT;
            return;
        }
        else
        {
            dprintf(INFO, "[BATTERY] battery voltage(%dmV) <= CLV ! Can not Boot Linux Kernel !! \n\r",bat_vol);
#ifndef NO_POWER_OFF
            mt6575_power_off();
#endif			
            while(1)
            {
                dprintf(INFO, "If you see the log, please check with RTC power off API\n\r");
            }
        }
    }
	#endif
    return;
}

#else

#include <platform/mt_typedefs.h>
#include <platform/mt_reg_base.h>
#include <printf.h>

void mt65xx_bat_init(void)
{
    dprintf(INFO, "[BATTERY] Skip mt65xx_bat_init !!\n\r");
    dprintf(INFO, "[BATTERY] If you want to enable power off charging, \n\r");
    dprintf(INFO, "[BATTERY] Please #define CFG_POWER_CHARGING!!\n\r");
}

#endif
