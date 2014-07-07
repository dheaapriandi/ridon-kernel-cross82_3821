#ifndef L3GD20_H
#define L3GD20_H
	 
#include <linux/ioctl.h>
	 
#define L3GD20_I2C_SLAVE_ADDR		0xD6     //SD0 high--->D6    SD0 low ----->D4
#define L3GD20_FIXED_DEVID			0xD4


/* L3GD20 Register Map  (Please refer to L3GD20 Specifications) */
#define L3GD20_REG_DEVID			         0x0F

#define L3GD20_REG_GYRO_XH			0x29
#define L3GD20_REG_GYRO_XL			0x28
//#define L3GD20_REG_GYRO_YH			0x1F
//#define L3GD20_REG_GYRO_YL			0x20
//#define L3GD20_REG_GYRO_ZH			0x21
//#define L3GD20_REG_GYRO_ZL			0x22
#define L3GD20_CTL_REG1			0x20 
#define L3GD20_FIFO_CTL			0x2E 

#define L3GD20_CTL_REG4			0x23 
#define L3GD20_FIFO_SRC_REG		0x2F 
#define L3GD20_STATUS_REG		    0x27 



	 

/*L3GD20 Register Bit definitions*/ 

#define L3GD20_FS_250_LSB			131	// LSB/(o/s)
#define L3GD20_FS_500_LSB			66 
#define L3GD20_FS_2000_LSB		16  
#define L3GD20_OUT_MAGNIFY		131
#define L3GD20_RANGE_250		    0x00
#define L3GD20_RANGE_500		    0x10
#define L3GD20_RANGE_2000		    0x30

#define L3GD20_FIFO_MODE_BYPASS   0x00
#define L3GD20_FIFO_MODE_FIFO     0x20
#define L3GD20_FIFO_MODE_STREAM   0x40

#define AUTO_INCREMENT 0x80










#define L3GD20_SAM_RATE_MASK		    0x07	//set sample rate and low padd filter configuration
#define L3GD20_RATE_8K_LPFB_256HZ 	0x00
#define L3GD20_RATE_1K_LPFB_188HZ	0x01
#define L3GD20_RATE_1K_LPFB_98HZ 	0x02
#define L3GD20_RATE_1K_LPFB_42HZ 	0x03
#define L3GD20_RATE_1K_LPFB_20HZ 	0x04
#define L3GD20_RATE_1K_LPFB_10HZ 	0x05
#define L3GD20_RATE_1K_LPFB_5HZ 	0x06


#define L3GD20_POWER_ON			0x08	
#define L3GD20_100HZ 0x00 
#define L3GD20_200HZ 0x40 
#define L3GD20_400HZ 0x80
#define L3GD20_800HZ 0xC0 


	 
#define L3GD20_SUCCESS		       0
#define L3GD20_ERR_I2C		      -1
#define L3GD20_ERR_STATUS			  -3
#define L3GD20_ERR_SETUP_FAILURE	  -4
#define L3GD20_ERR_GETGSENSORDATA  -5
#define L3GD20_ERR_IDENTIFICATION	  -6

#define L3GD20_BUFSIZE 60

// 1 rad = 180/PI degree, L3GD20_OUT_MAGNIFY = 131,
// 180*131/PI = 7506
#define DEGREE_TO_RAD	7506
	 
#endif //L3GD20_H

