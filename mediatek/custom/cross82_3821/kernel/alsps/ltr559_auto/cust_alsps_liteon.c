/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */
#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
//	.polling_mode = 1,
// yanggy 2012-07-17 modify for interrupt mode begin
//#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE 
    .polling_mode_ps = 0,
//#else
//    .polling_mode_ps = 1,
//#endif
// yanggy 2012-07-17 modify for interrupt mode end
    .polling_mode_als = 1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    //.als_level  = {5, 7,  9,  13,  16,  21,  25,  40,  120,  400, 1600,  3000, 6000,  10000, 65535},
    //.als_value  = { 1, 2,  4,   9,  16,  39,  80, 150, 300,  650,  1500,  3500,  8500, 15000,  45000, 100000},      //------the old data
    .als_level  = { 2, 4, 6,  8, 10,  15,  20,  50,  100, 1600,  10000,  65535},
    .als_value  = { 40, 60, 75,  90, 160, 225, 640, 1280,  2600, 6400, 60000,  100000},

    //.als_value  = { 1, 40,   90,  160,  225,  320, 640, 1280,  2600, 10240,  20000,  40000, 60000,  80000, 100000}, //-----the new data
//sxt 2011-8-19 add for threshold begin
#if defined (GN_MTK_BSP)
	.ps_threshold =PS_THRESHOLD,
#else
	.ps_threshold =60,
#endif
	.ps_threshold_high = 25,
	.ps_threshold_low = 20,


//gionee sxt 2011-8-19 add for threshold end
};

struct alsps_hw *LTR559_get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
