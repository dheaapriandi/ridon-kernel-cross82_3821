/*
 *
 * BMA2XX driver for MT65xx
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef BMA2XX_H
#define BMA2XX_H
	 
#include <linux/ioctl.h>
 
#define BMA2XX_I2C_SLAVE_WRITE_ADDR		0x30

//#define BMA222E_FIXED_DEVID			0xF8
#define BMA250_FIXED_DEVID			0x03
//#define BMA250E_FIXED_DEVID                   0xF9
#define BMA2XX_FIXED_DEVID			{0xF8, 0x03, 0xF9}

 /* BMA2XX Register Map  (Please refer to BMA2XX Specifications) */
#define BMA2XX_REG_DEVID			0x00
#define BMA2XX_REG_OFSX				0x16
#define BMA2XX_REG_OFSX_HIGH		0x1A
#define BMA2XX_REG_BW_RATE			0x10
#define BMA2XX_BW_MASK				0x1f
#define BMA2XX_BW_200HZ				0x0d
#define BMA2XX_BW_100HZ				0x0c
#define BMA2XX_BW_50HZ				0x0b
#define BMA2XX_BW_25HZ				0x0a
#define BMA2XX_REG_POWER_CTL		0x11		
#define BMA2XX_REG_DATA_FORMAT		0x0f
#define BMA2XX_RANGE_MASK			0x0f
#define BMA2XX_RANGE_2G				0x03
#define BMA2XX_RANGE_4G				0x05
#define BMA2XX_RANGE_8G				0x08
#define BMA2XX_REG_DATAXLOW			0x03
#define BMA2XX_REG_DATA_RESOLUTION	0x14
#define BMA2XX_MEASURE_MODE			0x80	
#define BMA2XX_SELF_TEST           	0x32
#define BMA2XX_SELF_TEST_AXIS_X		0x01
#define BMA2XX_SELF_TEST_AXIS_Y		0x02
#define BMA2XX_SELF_TEST_AXIS_Z		0x03
#define BMA2XX_SELF_TEST_POSITIVE	0x00
#define BMA2XX_SELF_TEST_NEGATIVE	0x04
#define BMA2XX_INT_REG_1           	0x16
#define BMA2XX_INT_REG_2           	0x17

#define BMA2XX_SOFT_RESET          0x14
#define BMA2XX_PMU_LOW_NOISE       0x12
#define BMA2XX_NORM_MODE           0xe0
#define BMA2XX_SUSPEND_MODE        0x60

#define BMA2XX_SUCCESS				0
#define BMA2XX_ERR_I2C				-1
#define BMA2XX_ERR_STATUS			-3
#define BMA2XX_ERR_SETUP_FAILURE	-4
#define BMA2XX_ERR_GETGSENSORDATA	-5
#define BMA2XX_ERR_IDENTIFICATION	-6	 
	 
#define BMA2XX_BUFSIZE				256

#define BMA2X2_INT1_PAD_SEL_REG   0x19
#define BMA2X2_INT2_PAD_SEL_REG   0x1B
#define BMA2X2_INT_CTRL_REG       0x21 
#define BMA2X2_SLOPE_DURN_REG     0x27
#define BMA2X2_SLOPE_THRES_REG    0x28


extern struct acc_hw* bma2xx_get_cust_acc_hw(void);

#endif

