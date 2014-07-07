#include <linux/xhci/xhci-mtk-power.h>
#include <linux/xhci/xhci-mtk.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>
#include <linux/delay.h>
#include "xhci.h"

/* set 1 to PORT_POWER of PORT_STATUS register of each port */
void enableXhciAllPortPower(struct xhci_hcd *xhci){
	int i;
	u32 port_id, temp;
	u32 __iomem *addr;

	g_num_u3_port = SSUSB_U3_PORT_NUM(readl((void __iomem *)SSUSB_IP_CAP));
	g_num_u2_port = SSUSB_U2_PORT_NUM(readl((void __iomem *)SSUSB_IP_CAP));

	for(i=1; i<=g_num_u3_port; i++){
		port_id=i;
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS*((port_id-1) & 0xff);
		temp = xhci_readl(xhci, addr);
		temp = xhci_port_state_to_neutral(temp);
		temp |= PORT_POWER;
		xhci_writel(xhci, temp, addr);
	}
	for(i=1; i<=g_num_u2_port; i++){
		port_id=i+g_num_u3_port;
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS*((port_id-1) & 0xff);
		temp = xhci_readl(xhci, addr);
		temp = xhci_port_state_to_neutral(temp);
		temp |= PORT_POWER;
		xhci_writel(xhci, temp, addr);
	}
}

void enableAllClockPower(){
	int i;
	u32 temp;

	g_num_u3_port = SSUSB_U3_PORT_NUM(readl((void __iomem *)SSUSB_IP_CAP));
	g_num_u2_port = SSUSB_U2_PORT_NUM(readl((void __iomem *)SSUSB_IP_CAP));

	//2.	Enable xHC
	writel(readl((void __iomem *)SSUSB_IP_PW_CTRL) | (SSUSB_IP_SW_RST), (void __iomem *)SSUSB_IP_PW_CTRL);
	writel(readl((void __iomem *)SSUSB_IP_PW_CTRL) & (~SSUSB_IP_SW_RST), (void __iomem *)SSUSB_IP_PW_CTRL);
	writel(readl((void __iomem *)SSUSB_IP_PW_CTRL_1) & (~SSUSB_IP_PDN), (void __iomem *)SSUSB_IP_PW_CTRL_1);


	//1.	Enable target ports
	for(i=0; i<g_num_u3_port; i++){
		temp = readl((void __iomem *)SSUSB_U3_CTRL(i));
		temp = temp & (~SSUSB_U3_PORT_PDN) & (~SSUSB_U3_PORT_DIS);

		writel(temp, (void __iomem *)SSUSB_U3_CTRL(i));
	}

    /*
     * FIXME: clock is the correct 30MHz, if the U3 device is enabled
     */
    temp = readl(SSUSB_U3_CTRL(0));
    temp = temp & (~SSUSB_U3_PORT_PDN) & (~SSUSB_U3_PORT_DIS) & (~SSUSB_U3_PORT_HOST_SEL);
    writel(temp, SSUSB_U3_CTRL(0));

	for(i=0; i<g_num_u2_port; i++){
		temp = readl((void __iomem *)SSUSB_U2_CTRL(i));
		temp = temp & (~SSUSB_U2_PORT_PDN) & (~SSUSB_U2_PORT_DIS);
		writel(temp, (void __iomem *)SSUSB_U2_CTRL(i));
	}
	msleep(100);
}


//(X)disable clock/power of a port
//(X)if all ports are disabled, disable IP ctrl power
//disable all ports and IP clock/power, this is just mention HW that the power/clock of port
//and IP could be disable if suspended.
//If doesn't not disable all ports at first, the IP clock/power will never be disabled
//(some U2 and U3 ports are binded to the same connection, that is, they will never enter suspend at the same time
//port_index: port number
//port_rev: 0x2 - USB2.0, 0x3 - USB3.0 (SuperSpeed)
void disablePortClockPower(int port_index, int port_rev){
	u32 temp;
	int real_index;

	real_index = port_index;

	if(port_rev == 0x3){
		temp = readl((void __iomem *)SSUSB_U3_CTRL(real_index));
		temp = temp | (SSUSB_U3_PORT_PDN);
		writel(temp, (void __iomem *)SSUSB_U3_CTRL(real_index));
	}
	else if(port_rev == 0x2){
		temp = readl((void __iomem *)SSUSB_U2_CTRL(real_index));
		temp = temp | (SSUSB_U2_PORT_PDN);
		writel(temp, (void __iomem *)SSUSB_U2_CTRL(real_index));
	}
	writel(readl((void __iomem *)SSUSB_IP_PW_CTRL_1) | (SSUSB_IP_PDN), (void __iomem *)SSUSB_IP_PW_CTRL_1);
}

//if IP ctrl power is disabled, enable it
//enable clock/power of a port
//port_index: port number
//port_rev: 0x2 - USB2.0, 0x3 - USB3.0 (SuperSpeed)
void enablePortClockPower(int port_index, int port_rev){
	u32 temp;
	int real_index;

	real_index = port_index;

	writel(readl((void __iomem *)SSUSB_IP_PW_CTRL_1) & (~SSUSB_IP_PDN), (void __iomem *)SSUSB_IP_PW_CTRL_1);

	if(port_rev == 0x3){
		temp = readl((void __iomem *)SSUSB_U3_CTRL(real_index));
		temp = temp & (~SSUSB_U3_PORT_PDN);
		writel(temp, (void __iomem *)SSUSB_U3_CTRL(real_index));
	}
	else if(port_rev == 0x2){
		temp = readl((void __iomem *)SSUSB_U2_CTRL(real_index));
		temp = temp & (~SSUSB_U2_PORT_PDN);
		writel(temp, (void __iomem *)SSUSB_U2_CTRL(real_index));
	}
}
