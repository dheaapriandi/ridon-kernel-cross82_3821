/*
 * MD218A voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "FM50AF.h"
#include "../camera/kd_camera_hw.h"

#define LENS_I2C_BUSNUM 1
//static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO("FM50AF", 0x18)};
static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO("FM50AF", 0x22)};


#define FM50AF_DRVNAME "FM50AF"
#define FM50AF_VCM_WRITE_ID           0x18

#define FM50AF_DEBUG
#ifdef FM50AF_DEBUG
#define FM50AFDB printk
#else
#define FM50AFDB(x,...)
#endif

#define AF_DLC_MODE

// #define AF_JUMP_BACK_SLOWLY      // LINE Use this if you can listen to the back jump hit
#define AF_JUMP_BACK_STEP    50  // LINE <> <DATE20131217> <S8513A:avoid imx111 af noise.> wupingzhou
#define AF_JUMP_BACK_WAIT_MS 20  // LINE <> <DATE20131217> <S8513A:avoid imx111 af noise.> wupingzhou
#define AF_JUMP_BACK_CURRENT 250 //  <S8513A:avoid imx111 af noise.> wupingzhou
#define AF_FIRST_MOVE_CNT    5   // LINE <> <DATE20131224> <9320:avoid hit when open Camera> Jiangde
static int g_s_iFirstMove = 0; // LINE <> <DATE20131224> <9320:avoid hit when open Camera> Jiangde

static spinlock_t g_FM50AF_SpinLock;

static struct i2c_client * g_pstFM50AF_I2Cclient = NULL;

static dev_t g_FM50AF_devno;
static struct cdev * g_pFM50AF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int  g_s4FM50AF_Opened = 0;
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4FM50AF_INF = 0;
static unsigned long g_u4FM50AF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition   = 0;
static  bool releaseflag = 0;

static int g_sr = 5;

#if 0
extern s32 mt_set_gpio_mode(u32 u4Pin, u32 u4Mode);
extern s32 mt_set_gpio_out(u32 u4Pin, u32 u4PinOut);
extern s32 mt_set_gpio_dir(u32 u4Pin, u32 u4Dir);
#endif

static int s4FM50AF_ReadReg(unsigned short * a_pu2Result)
{
    int  i4RetValue = 0;
    char pBuff[2];

    i4RetValue = i2c_master_recv(g_pstFM50AF_I2Cclient, pBuff , 2);

    if (i4RetValue < 0) 
    {
        FM50AFDB("[FM50AF] I2C read failed!! \n");
        return -1;
    }

    *a_pu2Result = (((u16)pBuff[0]) << 4) + (pBuff[1] >> 4);

    return 0;
}

static int s4FM50AF_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;

    char puSendCmd[2] = {(char)(a_u2Data >> 4) , (char)(((a_u2Data & 0xF) << 4)+g_sr)};

    FM50AFDB("HJDDbg-MTK, [FM50AF] s4FM50AF_WriteReg, g_s4FM50AF_Opened=%d, g_sr=%d, write pos=%d \n",g_s4FM50AF_Opened, g_sr, a_u2Data);
    g_pstFM50AF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSendCmd, 2);
	
    if (i4RetValue < 0) 
    {
        FM50AFDB("[FM50AF] I2C send failed!! \n");
        return -1;
    }

    return 0;
}

inline static int getFM50AFInfo(__user stFM50AF_MotorInfo * pstMotorInfo)
{
    stFM50AF_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4FM50AF_MACRO;
    stMotorInfo.u4InfPosition     = g_u4FM50AF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = TRUE;

    FM50AFDB("HJDDbg-MTK, [FM50AF], u4CurrentPosition=%d \n", stMotorInfo.u4CurrentPosition);
	if (g_i4MotorStatus == 1)	{stMotorInfo.bIsMotorMoving = 1;}
	else						{stMotorInfo.bIsMotorMoving = 0;}

	if (g_s4FM50AF_Opened >= 1)	{stMotorInfo.bIsMotorOpen = 1;}
	else						{stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stFM50AF_MotorInfo)))
    {
        FM50AFDB("[FM50AF] copy to user failed when getting motor information \n");
    }

    FM50AFDB("HJDDbg-MTK, stMotorInfo.u4CurrentPosition = %d \n", stMotorInfo.u4CurrentPosition); //////?¡§??a??
    return 0;
}

#ifdef LensdrvCM3
inline static int getFM50AFMETA(__user stFM50AF_MotorMETAInfo * pstMotorMETAInfo)
{
    stFM50AF_MotorMETAInfo stMotorMETAInfo;
    stMotorMETAInfo.Aperture=2.8;      //fn
	stMotorMETAInfo.Facing=1;   
	stMotorMETAInfo.FilterDensity=1;   //X
	stMotorMETAInfo.FocalDistance=1.0;  //diopters
	stMotorMETAInfo.FocalLength=34.0;  //mm
	stMotorMETAInfo.FocusRange=1.0;    //diopters
	stMotorMETAInfo.InfoAvalibleApertures=2.8;
	stMotorMETAInfo.InfoAvalibleFilterDensity=1;
	stMotorMETAInfo.InfoAvalibleFocalLength=34.0;
	stMotorMETAInfo.InfoAvalibleHypeDistance=1.0;
	stMotorMETAInfo.InfoAvalibleMinFocusDistance=1.0;
	stMotorMETAInfo.InfoAvalibleOptStabilization=0;
	stMotorMETAInfo.OpticalAxisAng[0]=0.0;
	stMotorMETAInfo.OpticalAxisAng[1]=0.0;
	stMotorMETAInfo.Position[0]=0.0;
	stMotorMETAInfo.Position[1]=0.0;
	stMotorMETAInfo.Position[2]=0.0;
	stMotorMETAInfo.State=0;
	stMotorMETAInfo.u4OIS_Mode=0;
	
	if(copy_to_user(pstMotorMETAInfo , &stMotorMETAInfo , sizeof(stFM50AF_MotorMETAInfo)))
	{
		FM50AFDB("[FM50AF] copy to user failed when getting motor information \n");
	}

    return 0;
}
#endif


//BEGIN <> <DATE20131217> <S8513A:avoid imx111 af noise.> wupingzhou
inline static int moveFM50AF_int(unsigned long a_u4Position)
{
    FM50AFDB("HJDDbg, [FM50AF] moveFM50AF_int=%d \n", a_u4Position);
    if(s4FM50AF_WriteReg((unsigned short)a_u4Position) == 0)
    {
        spin_lock(&g_FM50AF_SpinLock);		
        g_u4CurrPosition = (unsigned long)a_u4Position;
        spin_unlock(&g_FM50AF_SpinLock);				
    }
    else
    {
        FM50AFDB("[FM50AF] set I2C failed when moving the motor \n");			
        spin_lock(&g_FM50AF_SpinLock);
        g_i4MotorStatus = -1;
        spin_unlock(&g_FM50AF_SpinLock);				
    }
}
//END <> <DATE20131217> <S8513A:avoid imx111 af noise.> wupingzhou


inline static int moveFM50AF(unsigned long a_u4Position)
{
    int           ret = 0;
    unsigned long temp_Position = 0; //LINE <> <DATE20131217> <S8513A:avoid imx111 af noise.> wupingzhou

    FM50AFDB("HJDDbg-MTK, moveFM50AF, a_u4Position = %d \n", a_u4Position);
    if((a_u4Position > g_u4FM50AF_MACRO) || (a_u4Position < g_u4FM50AF_INF))
    {
        FM50AFDB("[FM50AF] out of range \n");
        return -EINVAL;
    }

#if 1
    if(releaseflag ==1)
    {
        if(a_u4Position>=g_u4FM50AF_MACRO/4)
        {
            s4FM50AF_WriteReg(a_u4Position/4);
            s4FM50AF_WriteReg(a_u4Position/4+a_u4Position/6);
            s4FM50AF_WriteReg(a_u4Position/4+a_u4Position/6+a_u4Position/6);
            s4FM50AF_WriteReg(a_u4Position/4+a_u4Position/6+a_u4Position/6+a_u4Position/6);
            s4FM50AF_WriteReg(a_u4Position/4+a_u4Position/6+a_u4Position/6+a_u4Position/6+a_u4Position/6);
            s4FM50AF_WriteReg(a_u4Position/4+a_u4Position/6+a_u4Position/6+a_u4Position/6+a_u4Position/6+a_u4Position/6);
            s4FM50AF_WriteReg(a_u4Position);
            releaseflag = 0;
            return 0;
        }
        releaseflag = 0;
    }
#endif

    if (g_s4FM50AF_Opened == 1)
    {
        unsigned short InitPos;
        ret = s4FM50AF_ReadReg(&InitPos);
	    
        spin_lock(&g_FM50AF_SpinLock);
        if(ret == 0)
        {
            FM50AFDB("[FM50AF] Init Pos %6d \n", InitPos);
            g_u4CurrPosition = (unsigned long)InitPos;
        }
        else
        {		
            g_u4CurrPosition = 0;
        }
        g_s4FM50AF_Opened = 2;
        spin_unlock(&g_FM50AF_SpinLock);
    }

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(&g_FM50AF_SpinLock);	
        g_i4Dir = 1;
        spin_unlock(&g_FM50AF_SpinLock);	
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(&g_FM50AF_SpinLock);	
        g_i4Dir = -1;
        spin_unlock(&g_FM50AF_SpinLock);			
    }
    else										{return 0;}

    spin_lock(&g_FM50AF_SpinLock);    
    g_u4TargetPosition = a_u4Position;
    spin_unlock(&g_FM50AF_SpinLock);	

    //FM50AFDB("[FM50AF] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);

    spin_lock(&g_FM50AF_SpinLock);
    //g_sr = 3;
    g_i4MotorStatus = 0;
    spin_unlock(&g_FM50AF_SpinLock);	

#ifdef AF_JUMP_BACK_SLOWLY //LINE <> <DATE20131217> <S8513A:avoid imx111 af noise.> wupingzhou
    if( g_s_iFirstMove < AF_FIRST_MOVE_CNT
       && g_u4CurrPosition > AF_JUMP_BACK_CURRENT
       && g_u4TargetPosition < g_u4CurrPosition 
       && g_u4CurrPosition - g_u4TargetPosition > AF_JUMP_BACK_STEP)
    {
        FM50AFDB("HJDDbg, [FM50AF] AF_JUMP_BACK_SLOWLY, current=%d, target=%d \n", g_u4CurrPosition, g_u4TargetPosition);
        g_s_iFirstMove++;
            
        do
        {
            temp_Position = g_u4CurrPosition - AF_JUMP_BACK_STEP;
            FM50AFDB("HJDDbg, [FM50AF] AF_JUMP_BACK_SLOWLY, temp_Position=%d \n", temp_Position);
            
            moveFM50AF_int(temp_Position);
            mdelay(AF_JUMP_BACK_WAIT_MS);
            FM50AFDB("HJDDbg, [FM50AF] AF_JUMP_BACK_SLOWLY, after delay %d ms \n", AF_JUMP_BACK_WAIT_MS);
        } while(temp_Position - g_u4TargetPosition > AF_JUMP_BACK_STEP);
        
        if(temp_Position != g_u4TargetPosition)
        {
            moveFM50AF_int(g_u4TargetPosition);
        }            
    }
    else
    {
        moveFM50AF_int(g_u4TargetPosition);
    }
#else
    if(s4FM50AF_WriteReg((unsigned short)g_u4TargetPosition) == 0)
    {
        spin_lock(&g_FM50AF_SpinLock);		
        g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
        spin_unlock(&g_FM50AF_SpinLock);				
    }
    else
    {
        FM50AFDB("[FM50AF] set I2C failed when moving the motor \n");			
        spin_lock(&g_FM50AF_SpinLock);
        g_i4MotorStatus = -1;
        spin_unlock(&g_FM50AF_SpinLock);				
    }
#endif

    return 0;
}

inline static int setFM50AFInf(unsigned long a_u4Position)
{
    spin_lock(&g_FM50AF_SpinLock);
    g_u4FM50AF_INF = a_u4Position;
    FM50AFDB("HJDDbg-MTK, setFM50AFInf, g_u4FM50AF_INF = %d \n", g_u4FM50AF_INF);
    spin_unlock(&g_FM50AF_SpinLock);	
    return 0;
}

inline static int setFM50AFMacro(unsigned long a_u4Position)
{
    spin_lock(&g_FM50AF_SpinLock);
    g_u4FM50AF_MACRO = a_u4Position;
    FM50AFDB("HJDDbg-MTK, setFM50AFMacro, g_u4FM50AF_MACRO = %d \n", g_u4FM50AF_MACRO);
    spin_unlock(&g_FM50AF_SpinLock);	
    return 0;	
}

////////////////////////////////////////////////////////////////
static long FM50AF_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case FM50AFIOC_G_MOTORINFO :
            i4RetValue = getFM50AFInfo((__user stFM50AF_MotorInfo *)(a_u4Param));
        break;
#ifdef LensdrvCM3
        case FM50AFIOC_G_MOTORMETAINFO :
            i4RetValue = getFM50AFMETA((__user stFM50AF_MotorMETAInfo *)(a_u4Param));
        break;
#endif
        case FM50AFIOC_T_MOVETO :
            i4RetValue = moveFM50AF(a_u4Param);
        break;
 
        case FM50AFIOC_T_SETINFPOS :
            i4RetValue = setFM50AFInf(a_u4Param);
        break;

        case FM50AFIOC_T_SETMACROPOS :
            i4RetValue = setFM50AFMacro(a_u4Param);
        break;
		
        default :
      	    FM50AFDB("[FM50AF] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    return i4RetValue;
}

//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
// 3.Update f_op pointer.
// 4.Fill data structures into private_data
//CAM_RESET
static int FM50AF_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    FM50AFDB("[FM50AF] FM50AF_Open - Start\n");
    long i4RetValue = 0;
    spin_lock(&g_FM50AF_SpinLock);

    if(g_s4FM50AF_Opened)
    {
        spin_unlock(&g_FM50AF_SpinLock);
        FM50AFDB("[FM50AF] the device is opened \n");
        return -EBUSY;
    }
    g_s4FM50AF_Opened = 1;
		
    spin_unlock(&g_FM50AF_SpinLock);

    #ifdef AF_DLC_MODE
    /*char puSuspendCmd[2] = {(char)(0xEC), (char)(0xA3)};
    i4RetValue = i2c_master_send(g_pstIMX135AF_I2Cclient, puSuspendCmd, 2);

    char puSuspendCmd2[2] = {(char)(0xA1), (char)(0x05)};//0x0D
    i4RetValue = i2c_master_send(g_pstIMX135AF_I2Cclient, puSuspendCmd2, 2);

    char puSuspendCmd3[2] = {(char)(0xF2), (char)(0xF8)};//0xE8
    i4RetValue = i2c_master_send(g_pstIMX135AF_I2Cclient, puSuspendCmd3, 2);

    char puSuspendCmd4[2] = {(char)(0xDC), (char)(0x51)};
    i4RetValue = i2c_master_send(g_pstIMX135AF_I2Cclient, puSuspendCmd4, 2);*/

    char puSuspendCmd[2] = {(char)(0xEC), (char)(0xA3)};
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd, 2);

    char puSuspendCmd1[2] = {(char)(0xA1), (char)(0x0D)};//--05
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd1, 2);

    char puSuspendCmd2[2] = {(char)(0xF2), (char)(0x10)}; // 0x10(2013.12.23) LINE <><20131212><vibration period> Jiangde,  0xE8-->0x38
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd2, 2);

    char puSuspendCmd3[2] = {(char)(0xDC), (char)(0x51)};
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd3, 2);

    #else //AF_LSC_MODE

    char puSuspendCmd[2] = {(char)(0xEC), (char)(0xA3)};
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd, 2);

    char puSuspendCmd1[2] = {(char)(0xF2), (char)(0x80)};
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd1, 2);

    char puSuspendCmd2[2] = {(char)(0xDC), (char)(0x51)};
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd2, 2);
 
    #endif


    FM50AFDB("[FM50AF] FM50AF_Open - End\n");

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int FM50AF_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    long i4RetValue = 0;
    
    FM50AFDB("HJDDbg, [FM50AF] FM50AF_Release - Start\n");

    if (!g_s4FM50AF_Opened)
    {
        FM50AFDB("[FM50AF] not opened \n");
        return;
    }

    char puSuspendCmd[2] = {(char)(0xEC), (char)(0xA3)};
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd, 2);

    char puSuspendCmd1[2] = {(char)(0xA1), (char)(0x05)};// 05:LSC 0D:DLC
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd1, 2);

    char puSuspendCmd2[2] = {(char)(0xF2), (char)(0xE8)}; // LINE <><20131212><vibration period> Jiangde,  0xE8-->0x38
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd2, 2);

    char puSuspendCmd3[2] = {(char)(0xDC), (char)(0x51)};
    i4RetValue = i2c_master_send(g_pstFM50AF_I2Cclient, puSuspendCmd3, 2);
    
    
    FM50AFDB("[FM50AF] free \n");
    g_sr = 5; // Jiangde, 5-->4 

    FM50AFDB("HJDDbg, [FM50AF] free, g_sr=%d \n", g_sr);
    s4FM50AF_WriteReg(300);
    msleep(70);

    s4FM50AF_WriteReg(200);
    msleep(20);

    s4FM50AF_WriteReg(100);
    msleep(20);  

    spin_lock(&g_FM50AF_SpinLock);
    g_s4FM50AF_Opened = 0;
    g_s_iFirstMove = 0; // LINE <> <DATE20131224> <9320:avoid hit when open Camera> Jiangde
    // g_u4CurrPosition = g_u4FM50AF_INF; // MTK
    spin_unlock(&g_FM50AF_SpinLock);

    FM50AFDB("HJDDbg-MTK, [FM50AF] FM50AF_Release - End, g_u4CurrPosition=%d\n", g_u4CurrPosition);
	releaseflag = 1; // MTK

    return 0;
}

static const struct file_operations g_stFM50AF_fops = 
{
    .owner = THIS_MODULE,
    .open = FM50AF_Open,
    .release = FM50AF_Release,
    .unlocked_ioctl = FM50AF_Ioctl
};

inline static int Register_FM50AF_CharDrv(void)
{
    struct device* vcm_device = NULL;

    FM50AFDB("[FM50AF] Register_FM50AF_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_FM50AF_devno, 0, 1,FM50AF_DRVNAME) )
    {
        FM50AFDB("[FM50AF] Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pFM50AF_CharDrv = cdev_alloc();

    if(NULL == g_pFM50AF_CharDrv)
    {
        unregister_chrdev_region(g_FM50AF_devno, 1);

        FM50AFDB("[FM50AF] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pFM50AF_CharDrv, &g_stFM50AF_fops);

    g_pFM50AF_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pFM50AF_CharDrv, g_FM50AF_devno, 1))
    {
        FM50AFDB("[FM50AF] Attatch file operation failed\n");

        unregister_chrdev_region(g_FM50AF_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, "actuatordrv2");
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        FM50AFDB("Unable to create class, err = %d\n", ret);
        return ret;            
    }

    vcm_device = device_create(actuator_class, NULL, g_FM50AF_devno, NULL, FM50AF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    FM50AFDB("[FM50AF] Register_FM50AF_CharDrv - End\n");    
    return 0;
}

inline static void Unregister_FM50AF_CharDrv(void)
{
    FM50AFDB("[FM50AF] Unregister_FM50AF_CharDrv - Start\n");

    //Release char driver
    cdev_del(g_pFM50AF_CharDrv);

    unregister_chrdev_region(g_FM50AF_devno, 1);
    
    device_destroy(actuator_class, g_FM50AF_devno);

    class_destroy(actuator_class);

    FM50AFDB("[FM50AF] Unregister_FM50AF_CharDrv - End\n");    
}

//////////////////////////////////////////////////////////////////////

static int FM50AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int FM50AF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id FM50AF_i2c_id[] = {{FM50AF_DRVNAME,0},{}};   
struct i2c_driver FM50AF_i2c_driver = {                       
    .probe = FM50AF_i2c_probe,                                   
    .remove = FM50AF_i2c_remove,                           
    .driver.name = FM50AF_DRVNAME,                 
    .id_table = FM50AF_i2c_id,                             
};  

#if 0 
static int FM50AF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, FM50AF_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int FM50AF_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int FM50AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    FM50AFDB("[FM50AF] FM50AF_i2c_probe\n");

    /* Kirby: add new-style driver { */
    g_pstFM50AF_I2Cclient = client;
    
    //g_pstFM50AF_I2Cclient->addr = g_pstFM50AF_I2Cclient->addr >> 1;
    g_pstFM50AF_I2Cclient->addr = FM50AF_VCM_WRITE_ID>>1;
    //Register char driver
    i4RetValue = Register_FM50AF_CharDrv();

    if(i4RetValue){

        FM50AFDB("[FM50AF] register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(&g_FM50AF_SpinLock);

    FM50AFDB("[FM50AF] Attached!! \n");

    return 0;
}

static int FM50AF_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&FM50AF_i2c_driver);
}

static int FM50AF_remove(struct platform_device *pdev)
{
    i2c_del_driver(&FM50AF_i2c_driver);
    return 0;
}

static int FM50AF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int FM50AF_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stFM50AF_Driver = {
    .probe		= FM50AF_probe,
    .remove	= FM50AF_remove,
    .suspend	= FM50AF_suspend,
    .resume	= FM50AF_resume,
    .driver		= {
        .name	= "lens_actuator2",
        .owner	= THIS_MODULE,
    }
};

static int __init FM50AF_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
	
    if(platform_driver_register(&g_stFM50AF_Driver)){
        FM50AFDB("failed to register FM50AF driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit FM50AF_i2C_exit(void)
{
	platform_driver_unregister(&g_stFM50AF_Driver);
}

module_init(FM50AF_i2C_init);
module_exit(FM50AF_i2C_exit);

MODULE_DESCRIPTION("FM50AF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");


