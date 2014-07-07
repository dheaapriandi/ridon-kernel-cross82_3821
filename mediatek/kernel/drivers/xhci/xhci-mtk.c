#include <xhci.h>
#include <linux/xhci/xhci-mtk.h>
#include <linux/xhci/xhci-mtk-power.h>
#include <linux/xhci/xhci-mtk-scheduler.h>
#include <linux/mu3phy/mtk-phy.h>
#include <linux/mu3phy/mtk-phy-c60802.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

void setInitialReg(){
	__u32 __iomem *addr;
	u32 temp;

    if(g_num_u3_port ){
    	//set MAC reference clock speed
    	addr = SSUSB_U3_MAC_BASE+U3_UX_EXIT_LFPS_TIMING_PAR;
    	temp = readl(addr);
    	temp &= ~(0xff << U3_RX_UX_EXIT_LFPS_REF_OFFSET);
    	temp |= (U3_RX_UX_EXIT_LFPS_REF << U3_RX_UX_EXIT_LFPS_REF_OFFSET);
    	writel(temp, addr);
    	addr = SSUSB_U3_MAC_BASE+U3_REF_CK_PAR;
    	temp = readl(addr);
    	temp &= ~(0xff);
    	temp |= U3_REF_CK_VAL;
    	writel(temp, addr);

    	//set SYS_CK
    	addr = SSUSB_U3_SYS_BASE+U3_TIMING_PULSE_CTRL;
    	temp = readl(addr);
    	temp &= ~(0xff);
    	temp |= CNT_1US_VALUE;
    	writel(temp, addr);
    }

	addr = SSUSB_U2_SYS_BASE+USB20_TIMING_PARAMETER;
	temp &= ~(0xff);
	temp |= TIME_VALUE_1US;
	writel(temp, addr);

    if(g_num_u3_port){
    	//set LINK_PM_TIMER=3
    	addr = SSUSB_U3_SYS_BASE+LINK_PM_TIMER;
    	temp = readl(addr);
    	temp &= ~(0xf);
    	temp |= PM_LC_TIMEOUT_VALUE;
    	writel(temp, addr);
    }
}


void setLatchSel(void){
	__u32 __iomem *latch_sel_addr;
	u32 latch_sel_value;

    if(g_num_u3_port <= 0)
        return;

	latch_sel_addr = U3_PIPE_LATCH_SEL_ADD;
	latch_sel_value = ((U3_PIPE_LATCH_TX)<<2) | (U3_PIPE_LATCH_RX);
	writel(latch_sel_value, latch_sel_addr);
}

void reinitIP(void){
	__u32 __iomem *ip_reset_addr;
	u32 ip_reset_value;

	enableAllClockPower();

	setLatchSel();
	mtk_xhci_scheduler_init();
#if PERF_PROBE
	mtk_probe_init(0x38383838);
	mtk_probe_out(0x0);
#endif
#if WEB_CAM_PROBE
	mtk_probe_init(0x70707070);
	mtk_probe_out(0x0);
#endif
}

///////////////////////////////////////////////////////////////////////////////

#define RET_SUCCESS 0
#define RET_FAIL 1

int dbg_u3init(int argc, char**argv)
{
	int ret;
	ret = u3phy_init();

	if(u3phy_ops->u2_slew_rate_calibration){
		u3phy_ops->u2_slew_rate_calibration(u3phy);
	}
	else{
		printk(KERN_ERR "WARN: PHY doesn't implement u2 slew rate calibration function\n");
	}
	if(u3phy_ops->init(u3phy) == PHY_TRUE){
        printk(KERN_ERR "phy registers and operations initial done\n");
		return RET_SUCCESS;
    }
	return RET_FAIL;
}

void dbg_setU1U2(int argc, char**argv){
	struct xhci_hcd *xhci;
	int u1_value;
	int u2_value;
	u32 port_id, temp;
	u32 __iomem *addr;

	if (argc<3)
    {
        printk(KERN_ERR "Arg: u1value u2value\n");
        return RET_FAIL;
    }

	u1_value = (int)simple_strtol(argv[1], &argv[1], 10);
	u2_value = (int)simple_strtol(argv[2], &argv[2], 10);
	addr = 	(SSUSB_U3_XHCI_BASE+0x424);		//0xf0040424
	temp = readl(addr);
	temp = temp & (~(0x0000ffff));
	temp = temp | u1_value | (u2_value<<8);
	writel(temp, addr);
}


