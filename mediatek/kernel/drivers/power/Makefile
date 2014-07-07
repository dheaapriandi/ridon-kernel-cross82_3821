ifeq ($(PROJECT),rio_5)
EXTRA_CFLAGS += -DRIO_5_CHARGING_TEMPERATURE_PLOICY
EXTRA_CFLAGS += -DBATT_ID_CHECK_SUPPORT
EXTRA_CFLAGS += -DJRD_FUEL_GAUGE_OPTIMIZE
endif
ifeq ($(TARGET_BUILD_MMITEST),true)
	EXTRA_CFLAGS += -DCHARGING_META
endif

ifeq ($(TARGET_BUILD_MMITEST),true)
EXTRA_CFLAGS += -DCHARGING_IOCHARGRE_RESET
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter.o battery_common.o

ifeq ($(MTK_FAN5402_SUPPORT),yes)
    obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
else ifeq ($(MTK_FAN5405_SUPPORT),yes)
        obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o 
else
    ifeq ($(MTK_SWCHR_SUPPORT),yes)
        obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
    else
        ifeq ($(MTK_BQ24158_SUPPORT),yes)
            obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o 
        else
            ifeq ($(MTK_BQ24196_SUPPORT),yes)
                obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
            else
                ifeq ($(MTK_BQ24296_SUPPORT),yes)
                    obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
                else
                    ifeq ($(MTK_NCP1851_SUPPORT),yes)
                        obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
                    else
                        ifeq ($(MTK_NCP1854_SUPPORT),yes)
                            obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
                        else
                            ifeq ($(MTK_BQ24160_SUPPORT),yes)
                                obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o
                            else
                                obj-$(CONFIG_MTK_SMART_BATTERY) += linear_charging.o
                            endif        
                        endif
                   endif
                endif
            endif
        endif  
    endif    
endif
