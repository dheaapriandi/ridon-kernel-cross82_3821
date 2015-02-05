#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>   //proc file use
#include <linux/dma-mapping.h>
#include <linux/xlog.h>

#include "../camera/kd_camera_hw.h"
#include <asm/system.h>


#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_imgsensor_errcode.h"

#include "kd_sensorlist.h"

static DEFINE_SPINLOCK(kdsensor_drv_lock);

#define SUPPORT_I2C_BUS_NUM1          0
//#define SUPPORT_I2C_BUS_NUM2        2


#define CAMERA_HW_DRVNAME1  "kd_camera_hw"
//#define CAMERA_HW_DRVNAME2  "kd_camera_hw_bus2"

static struct i2c_board_info __initdata i2c_devs1={I2C_BOARD_INFO(CAMERA_HW_DRVNAME1, 0xfe>>1)};
//static struct i2c_board_info __initdata i2c_devs2={I2C_BOARD_INFO(CAMERA_HW_DRVNAME2, 0xfe>>1)};




/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_sensorlist]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG , PFX, fmt, ##arg)


#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERROR , PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                    xlog_printk(ANDROID_LOG_DEBUG, PFX, fmt, ##args); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(a...)

#endif

/*******************************************************************************
* Proifling
********************************************************************************/
#define PROFILE 1
#if PROFILE
static struct timeval tv1, tv2;
/*******************************************************************************
*
********************************************************************************/
inline void KD_IMGSENSOR_PROFILE_INIT(void)
{
    do_gettimeofday(&tv1);
}

/*******************************************************************************
*
********************************************************************************/
inline void KD_IMGSENSOR_PROFILE(char *tag)
{
    unsigned long TimeIntervalUS;

	spin_lock(&kdsensor_drv_lock);

    do_gettimeofday(&tv2);
    TimeIntervalUS = (tv2.tv_sec - tv1.tv_sec) * 1000000 + (tv2.tv_usec - tv1.tv_usec);
    tv1 = tv2;

	spin_unlock(&kdsensor_drv_lock);
    PK_DBG("[%s]Profile = %lu\n",tag, TimeIntervalUS);
}
#else
inline static void KD_IMGSENSOR_PROFILE_INIT() {}
inline static void KD_IMGSENSOR_PROFILE(char *tag) {}
#endif

/*******************************************************************************
*
********************************************************************************/
extern int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName,BOOL On, char* mode_name);
extern ssize_t strobe_VDIrq(void);  //cotta : add for high current solution

/*******************************************************************************
*
********************************************************************************/

static struct i2c_client * g_pstI2Cclient = NULL;
//static struct i2c_client * g_pstI2Cclient2= NULL;

//81 is used for V4L driver
static dev_t g_CAMERA_HWdevno = MKDEV(250,0);
//static dev_t g_CAMERA_HWdevno2;
static struct cdev * g_pCAMERA_HW_CharDrv = NULL;
//static struct cdev * g_pCAMERA_HW_CharDrv2 = NULL;
static struct class *sensor_class = NULL;
//static struct class *sensor2_class = NULL;

static atomic_t g_CamHWOpend;
//static atomic_t g_CamHWOpend2;
static atomic_t g_CamHWOpening;
static atomic_t g_CamDrvOpenCnt;
//static atomic_t g_CamDrvOpenCnt2;

//static u32 gCurrI2CBusEnableFlag = 0;
//static u32 gI2CBusNum=SUPPORT_I2C_BUS_NUM1;

//#define SET_I2CBUS_FLAG(_x_)        ((1<<_x_)|(gCurrI2CBusEnableFlag))
//#define CLEAN_I2CBUS_FLAG(_x_)      ((~(1<<_x_))&(gCurrI2CBusEnableFlag))

static DEFINE_MUTEX(kdCam_Mutex);
static BOOL bSesnorVsyncFlag = FALSE;
static ACDK_KD_SENSOR_SYNC_STRUCT g_NewSensorExpGain = {128, 128, 128, 128, 1000, 640, 0xFF, 0xFF, 0xFF, 0};


extern MULTI_SENSOR_FUNCTION_STRUCT kd_MultiSensorFunc;
static MULTI_SENSOR_FUNCTION_STRUCT *g_pSensorFunc = &kd_MultiSensorFunc;;
static SENSOR_FUNCTION_STRUCT *g_pInvokeSensorFunc[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {NULL,NULL};
static BOOL g_bEnableDriver[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {FALSE,FALSE};
static CAMERA_DUAL_CAMERA_SENSOR_ENUM g_invokeSocketIdx[KDIMGSENSOR_MAX_INVOKE_DRIVERS] = {DUAL_CAMERA_NONE_SENSOR,DUAL_CAMERA_NONE_SENSOR};
static char g_invokeSensorNameStr[KDIMGSENSOR_MAX_INVOKE_DRIVERS][32] = {KDIMGSENSOR_NOSENSOR,KDIMGSENSOR_NOSENSOR};

static CAMERA_DUAL_CAMERA_SENSOR_ENUM g_CurrentInvokeCam = DUAL_CAMERA_NONE_SENSOR;

/*=============================================================================

=============================================================================*/
/*******************************************************************************
* i2c relative start
* migrate new style i2c driver interfaces required by Kirby 20100827
********************************************************************************/
static const struct i2c_device_id CAMERA_HW_i2c_id[] = {{CAMERA_HW_DRVNAME1,0},{}};
//static const struct i2c_device_id CAMERA_HW_i2c_id2[] = {{CAMERA_HW_DRVNAME2,0},{}};

/*******************************************************************************
* general camera image sensor kernel driver
*******************************************************************************/
UINT32 kdGetSensorInitFuncList(ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT **ppSensorList)
{
	if (NULL == ppSensorList)
	{
		PK_DBG("[kdGetSensorInitFuncList]ERROR: NULL ppSensorList\n");
		return 1;
	}
	*ppSensorList = &kdSensorList[0];
	return 0;
} // kdGetSensorInitFuncList()


/*******************************************************************************
*
********************************************************************************/



/*******************************************************************************
  * CAMERA_HW_DumpReg_To_Proc() 
  * Used to dump some critical sensor register 
  ********************************************************************************/
static int  CAMERA_HW_DumpReg_To_Proc(char *page, char **start, off_t off,
                                                                                       int count, int *eof, void *data)
{
    return count;
}

/*******************************************************************************
  * CAMERA_HW_Reg_Debug()
  * Used for sensor register read/write by proc file
  ********************************************************************************/
static int  CAMERA_HW_Reg_Debug( struct file *file, const char *buffer, unsigned long count,
                                                                     void *data)
{
    char regBuf[64] = {'\0'};
    u32 u4CopyBufSize = (count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);

    MSDK_SENSOR_REG_INFO_STRUCT sensorReg;
    memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

    if (copy_from_user(regBuf, buffer, u4CopyBufSize))
        return -EFAULT;

    if (sscanf(regBuf, "%x %x",  &sensorReg.RegAddr, &sensorReg.RegData) == 2) {
        if (g_pSensorFunc != NULL) {
            g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_SENSOR, SENSOR_FEATURE_SET_REGISTER, (MUINT8*)&sensorReg, (MUINT32*)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
            g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8*)&sensorReg, (MUINT32*)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
            PK_DBG("write addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
        }
    }
    else if (sscanf(regBuf, "%x", &sensorReg.RegAddr) == 1) {
        if (g_pSensorFunc != NULL) {
            g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_MAIN_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8*)&sensorReg, (MUINT32*)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
            PK_DBG("read addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
        }
    }

    return count;
}


static int  CAMERA_HW_Reg_Debug2( struct file *file, const char *buffer, unsigned long count,
                                                                     void *data)
{
    char regBuf[64] = {'\0'};
    u32 u4CopyBufSize = (count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);

    MSDK_SENSOR_REG_INFO_STRUCT sensorReg;
    memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

    if (copy_from_user(regBuf, buffer, u4CopyBufSize))
        return -EFAULT;

    if (sscanf(regBuf, "%x %x",  &sensorReg.RegAddr, &sensorReg.RegData) == 2) {
        if (g_pSensorFunc != NULL) {
            g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_SUB_SENSOR, SENSOR_FEATURE_SET_REGISTER, (MUINT8*)&sensorReg, (MUINT32*)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
            g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_SUB_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8*)&sensorReg, (MUINT32*)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
            PK_DBG("write addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
        }
    }
    else if (sscanf(regBuf, "%x", &sensorReg.RegAddr) == 1) {
        if (g_pSensorFunc != NULL) {
            g_pSensorFunc->SensorFeatureControl(DUAL_CAMERA_SUB_SENSOR, SENSOR_FEATURE_GET_REGISTER, (MUINT8*)&sensorReg, (MUINT32*)sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
            PK_DBG("read addr = 0x%08x, data = 0x%08x\n", sensorReg.RegAddr, sensorReg.RegData);
        }
    }

    return count;
}


/*=======================================================================
  * platform driver
  *=======================================================================*/
static struct platform_driver g_stCAMERA_HW_Driver = {
    .probe		= CAMERA_HW_probe,
    .remove	    = CAMERA_HW_remove,
    .suspend	= CAMERA_HW_suspend,
    .resume	    = CAMERA_HW_resume,
    .driver		= {
        .name	= "image_sensor",
        .owner	= THIS_MODULE,
    }
};

/*=======================================================================
  * CAMERA_HW_i2C_init()
  *=======================================================================*/
static int __init CAMERA_HW_i2C_init(void)
{
    struct proc_dir_entry *prEntry;

	//i2c_register_board_info(CAMERA_I2C_BUSNUM, &kd_camera_dev, 1);
    i2c_register_board_info(SUPPORT_I2C_BUS_NUM1, &i2c_devs1, 1);
    //i2c_register_board_info(SUPPORT_I2C_BUS_NUM2, &i2c_devs2, 1);


    if(platform_driver_register(&g_stCAMERA_HW_Driver)){
        PK_ERR("failed to register CAMERA_HW driver\n");
        return -ENODEV;
    }
    //if(platform_driver_register(&g_stCAMERA_HW_Driver2)){
    //    PK_ERR("failed to register CAMERA_HW driver\n");
    //    return -ENODEV;
    //}

    //Register proc file for main sensor register debug
    prEntry = create_proc_entry("driver/camsensor", 0, NULL);
    if (prEntry) {
        prEntry->read_proc = CAMERA_HW_DumpReg_To_Proc;
        prEntry->write_proc = CAMERA_HW_Reg_Debug;
    }
    else {
        PK_ERR("add /proc/driver/camsensor entry fail \n");
    }

    //Register proc file for sub sensor register debug
    prEntry = create_proc_entry("driver/camsensor2", 0, NULL);
    if (prEntry) {
        prEntry->read_proc = CAMERA_HW_DumpReg_To_Proc;
        prEntry->write_proc = CAMERA_HW_Reg_Debug2;
    }
    else {
        PK_ERR("add /proc/driver/camsensor2 entry fail \n");
    }
    atomic_set(&g_CamHWOpend, 0); 
    //atomic_set(&g_CamHWOpend2, 0);
    atomic_set(&g_CamDrvOpenCnt, 0);
    //atomic_set(&g_CamDrvOpenCnt2, 0);
    atomic_set(&g_CamHWOpening, 0);
    return 0;
}

/*=======================================================================
  * CAMERA_HW_i2C_exit()
  *=======================================================================*/
static void __exit CAMERA_HW_i2C_exit(void)
{
    platform_driver_unregister(&g_stCAMERA_HW_Driver);
    //platform_driver_unregister(&g_stCAMERA_HW_Driver2);
}


EXPORT_SYMBOL(kdSetSensorSyncFlag);
EXPORT_SYMBOL(kdSensorSyncFunctionPtr);
EXPORT_SYMBOL(kdGetRawGainInfoPtr);

module_init(CAMERA_HW_i2C_init);
module_exit(CAMERA_HW_i2C_exit);

MODULE_DESCRIPTION("CAMERA_HW driver");
MODULE_AUTHOR("Jackie Su <jackie.su@Mediatek.com>");
MODULE_LICENSE("GPL");





