#ifndef __CUST_KEY_H__
#define __CUST_KEY_H__

#include<cust_kpd.h>

#define MT65XX_RECOVERY_KEY	32
#define MT65XX_META_KEY		42	/* KEY_2 */
#define MT65XX_PMIC_RST_KEY	MT65XX_RECOVERY_KEY
#define MT_CAMERA_KEY 		10
#define MT65XX_FACTORY_KEY         (KPD_NUM_KEYS+1000)

#define MT65XX_BOOT_MENU_KEY       32   /* KEY_VOLUMEUP */
#define MT65XX_MENU_SELECT_KEY     MT65XX_BOOT_MENU_KEY   
#define MT65XX_MENU_OK_KEY         0 /* KEY_VOLUMEDOWN */

#endif /* __CUST_KEY_H__ */
