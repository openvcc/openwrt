#include <linux/module.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/phy.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>


#include "rtk_types.h"
#include "rtl8367c_asicdrv.h"
#include "rtl8367c_asicdrv_port.h"
#include "rtk_switch.h"
#include "vlan.h"
#include "smi.h"
#include "port.h"
#include "led.h"
#include "cpu.h"
#include "stat.h"

#define MAX_SW_NUM 10

#define RTL8367C_SW_CPU_PORT    6
 //RTL8367C_PHY_PORT_NUM + ext0 + ext1
#define RTL8367C_NUM_PORTS 7 

#define RTL8363NB_SW_CPU_PORT   2
//PHY1 PHY3 EXT0
#define RTL8363NB_NUM_PORTS 3

#define RTL8367C_NUM_VIDS  4096   


static inline unsigned int rtl8363_sw_to_phy_port(int port);
static inline unsigned int rtl8367c_sw_to_phy_port(int port);

struct sw_chip_info
{
	char *name;
	unsigned int ports;
	unsigned int vlans;
	unsigned int (*sw_port_to_phy_port)(int port);
};

struct sw_chip_info rtl8363 =
{
	.name = "RTL8363",
	.ports = RTL8363NB_NUM_PORTS,
	.vlans = RTL8367C_NUM_VIDS,
	.sw_port_to_phy_port = rtl8363_sw_to_phy_port,
};

struct sw_chip_info rtl8367 =
{
	.name = "RTL8367",
	.ports = RTL8367C_NUM_PORTS,
	.vlans = RTL8367C_NUM_VIDS,
	.sw_port_to_phy_port = rtl8367c_sw_to_phy_port,
};


extern int gsw_debug_proc_init(void);
extern void gsw_debug_proc_exit(void);


#define debug_line() printk("%s():%d\n",__FUNCTION__,__LINE__)

DEFINE_MUTEX(api_mutex);

#define RTL8363_MAGIC 	0x8363

typedef unsigned int (*sw_port_to_phy_port_func_t)(int port);
struct priv
{
	struct mii_bus * bus;	
	int vlanen;
	int ports;
	int reset_pin;
	unsigned int (*sw_port_to_phy_port)(int port);
};


static struct priv gMii[MAX_SW_NUM] ;
static int mii_cnt = 0;
static struct mii_bus *curr_mii;
static DEFINE_MUTEX(mii_mutex);
#define MII_LOCK() mutex_lock(&mii_mutex)
#define MII_UNLOCK() mutex_unlock(&mii_mutex)

static int rxdelay = 6; // 0~7
static int txdelay = 1; // 0~1

static void set_curr_mii(struct mii_bus *mii)
{	
	curr_mii = mii;
}

void MDIO_WRITE(int regID,int data)
{
	if(!curr_mii)
	{
		return;
	}

	//gMii.bus->write(gMii.bus,gMii.phyaddr,regID,data);
	curr_mii->write(curr_mii,MDC_MDIO_PHY_ID,regID,data);	
}

void MDIO_READ(int regID,int* pData)
{
	if(!curr_mii)
	{
		return;
	}
	//*pData = gMii.bus->read(gMii.bus,gMii.phyaddr,regID);
	*pData = curr_mii->read(curr_mii,MDC_MDIO_PHY_ID,regID);	
}

void rtlglue_drvMutexLock(void)
{
	//mutex_lock(&gMii.bus->mdio_lock);
}
void rtlglue_drvMutexUnlock(void)
{
	//mutex_unlock(&gMii.bus->mdio_lock);
}


void _write(struct mii_bus *bus, int phyaddr,int mAddrs ,int rData)
{
	 /* Write address control code to register 31 */
    mdiobus_write(bus, phyaddr, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

    /* Write address to register 23 */
    mdiobus_write(bus, phyaddr, MDC_MDIO_ADDRESS_REG, mAddrs);

    /* Write data to register 24 */
    mdiobus_write(bus, phyaddr, MDC_MDIO_DATA_WRITE_REG, rData);

    /* Write data control code to register 21 */
    mdiobus_write(bus, phyaddr, MDC_MDIO_CTRL1_REG, MDC_MDIO_WRITE_OP);
}

int _read(struct mii_bus *bus, int phyaddr,int mAddrs )
{
	 /* Write address control code to register 31 */
    mdiobus_write(bus, phyaddr, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

    /* Write address to register 23 */
    mdiobus_write(bus, phyaddr, MDC_MDIO_ADDRESS_REG, mAddrs);

    /* Write read control code to register 21 */
    mdiobus_write(bus, phyaddr, MDC_MDIO_CTRL1_REG, MDC_MDIO_READ_OP);

    /* Read data from register 25 */
    return mdiobus_read(bus, phyaddr, MDC_MDIO_DATA_READ_REG);
  
}

static int rtl8363_fixup(struct phy_device * pdev)
{
	u16 chipid;
	int retVal,data,regValue;

	/* Attach to primary LAN port and WAN port */
	if (pdev->mdio.addr != 0 && pdev->mdio.addr != 29)
		return 0;	

	_write(pdev->mdio.bus, pdev->mdio.addr, 0x13C2, 0x0249);
		
	data = _read(pdev->mdio.bus, pdev->mdio.addr, 0x1300);
	regValue = _read(pdev->mdio.bus, pdev->mdio.addr, 0x1301);
	
	_write(pdev->mdio.bus, pdev->mdio.addr, 0x13C2, 0x0000);

	printk("data=%04x,regValue=%04x \n",data,regValue);

	switch (data)
    {
        case 0x0276:
        case 0x0597:
        case 0x6367:
            chipid = CHIP_RTL8367C;
            //halCtrl = &rtl8367c_hal_Ctrl;
            break;
        case 0x0652:
        case 0x6368:
            chipid = CHIP_RTL8370B;
            //halCtrl = &rtl8370b_hal_Ctrl;
            break;
        case 0x0801:
        case 0x6511:
            if( (regValue & 0x00F0) == 0x0080)
            {
                chipid = CHIP_RTL8363SC_VB;
                //halCtrl = &rtl8363sc_vb_hal_Ctrl;
            }
            else
            {
                chipid = CHIP_RTL8364B;
                //halCtrl = &rtl8364b_hal_Ctrl;
            }
            break;
        default:
            return RT_ERR_FAILED;
    }
	

	if (chipid == CHIP_RTL8364B)
		pdev->phy_id = RTL8363_MAGIC;

	return 0;
}




struct rtl8367_priv {
	struct switch_dev	swdev;
	bool			global_vlan_enable;
};

struct rtl8367_mib_counter {	
	const char *name;
};

struct rtl8367_vlan_info {
	unsigned short	vid;
	unsigned int	untag;
	unsigned int	member;
	unsigned char		fid;
};


unsigned int rtl8367c_port_id[RTL8367C_NUM_PORTS]={0,1,2,3,4,EXT_PORT0,EXT_PORT1};
unsigned int rtl8363nb_port_id[RTL8363NB_NUM_PORTS]={1,3,EXT_PORT0};

/*rtl8367c  proprietary switch API wrapper */
static inline unsigned int rtl8363_sw_to_phy_port(int port)
{
	return rtl8363nb_port_id[port];
}

static inline unsigned int rtl8367c_sw_to_phy_port(int port)
{
	return rtl8367c_port_id[port];
}

void (*rtl8367_switch_reset_func)(void)=NULL;

static  struct rtl8367_mib_counter  rtl8367c_mib_counters[] = {
	{"ifInOctets"},
	{"dot3StatsFCSErrors"},
	{"dot3StatsSymbolErrors"},
	{"dot3InPauseFrames"},
	{"dot3ControlInUnknownOpcodes"},
	{"etherStatsFragments"},
	{"etherStatsJabbers"},
	{"ifInUcastPkts"},
	{"etherStatsDropEvents"},
	{"etherStatsOctets"},
	{"etherStatsUndersizePkts"},
	{"etherStatsOversizePkts"},
	{"etherStatsPkts64Octets"},
	{"etherStatsPkts65to127Octets"},
	{"etherStatsPkts128to255Octets"},
	{"etherStatsPkts256to511Octets"},
	{"etherStatsPkts512to1023Octets"},
	{"etherStatsPkts1024toMaxOctets"},
	{"etherStatsMcastPkts"}, 
	{"etherStatsBcastPkts"},
	{"ifOutOctets"},
	{"dot3StatsSingleCollisionFrames"},
	{"dot3StatsMultipleCollisionFrames"},
	{"dot3StatsDeferredTransmissions"},
	{"dot3StatsLateCollisions"}, 
	{"etherStatsCollisions"},
	{"dot3StatsExcessiveCollisions"},
	{"dot3OutPauseFrames"},
	{"dot1dBasePortDelayExceededDiscards"},
	{"dot1dTpPortInDiscards"},
	{"ifOutUcastPkts"},
	{"ifOutMulticastPkts"},
	{"ifOutBrocastPkts"},
	{"outOampduPkts"},
	{"inOampduPkts"},
	{"pktgenPkts"},
	{"inMldChecksumError"},
	{"inIgmpChecksumError"},
	{"inMldSpecificQuery"},
	{"inMldGeneralQuery"},
	{"inIgmpSpecificQuery"},
	{"inIgmpGeneralQuery"},
	{"inMldLeaves"},
	{"inIgmpLeaves"},
	{"inIgmpJoinsSuccess"},
	{"inIgmpJoinsFail"},
	{"inMldJoinsSuccess"},
	{"inMldJoinsFail"},
	{"inReportSuppressionDrop"},
	{"inLeaveSuppressionDrop"},
	{"outIgmpReports"},
	{"outIgmpLeaves"},
	{"outIgmpGeneralQuery"},
	{"outIgmpSpecificQuery"},
	{"outMldReports"},
	{"outMldLeaves"},
	{"outMldGeneralQuery"},
	{"outMldSpecificQuery"},
	{"inKnownMulticastPkts"},
	{"ifInMulticastPkts"},
	{"ifInBroadcastPkts"},
	{"ifOutDiscards"}
};



static inline unsigned int rtl8367c_portmask_phy_to_sw(rtk_portmask_t phy_portmask)
{
	int i;
	for (i = 0; i < 32; i++) {
		if(RTK_PORTMASK_IS_PORT_SET(phy_portmask,(i))) {
			RTK_PORTMASK_PORT_CLEAR(phy_portmask,(i));
			RTK_PORTMASK_PORT_SET(phy_portmask,i);
		}		

	}
	return (unsigned int)phy_portmask.bits[0];
}

static int rtl8367c_reset_mibs(void)
{
	return rtk_stat_global_reset();
}

static int rtl8367c_reset_port_mibs(int port)
{

	return rtk_stat_port_reset(rtl8367c_sw_to_phy_port(port));
}

static int rtl8367c_get_mibs_num(void)
{
	return ARRAY_SIZE(rtl8367c_mib_counters);
}

static const char *rtl8367c_get_mib_name(int idx)
{
	
	return rtl8367c_mib_counters[idx].name;
}

static int rtl8367c_get_port_mib_counter(int idx, int port, unsigned long long *counter)
{
	return rtk_stat_port_get(port, idx, counter);
}

static int rtl8367c_is_vlan_valid(unsigned int vlan)
{
	unsigned max = RTL8367C_NUM_VIDS;	

	if (vlan == 0 || vlan >= max)
		return 0;

	return 1;
}

static int rtl8367c_get_vlan( unsigned short vid, struct rtl8367_vlan_info *vlan)
{	
	rtk_vlan_cfg_t vlan_cfg;

	memset(vlan, '\0', sizeof(struct rtl8367_vlan_info));

	if (vid >= RTL8367C_NUM_VIDS)
		return -EINVAL;	

	if(rtk_vlan_get(vid,&vlan_cfg))
       	return -EINVAL;		
	
	vlan->vid = vid;
	vlan->member = rtl8367c_portmask_phy_to_sw(vlan_cfg.mbr);	
	vlan->untag = rtl8367c_portmask_phy_to_sw(vlan_cfg.untag);	
	vlan->fid = vlan_cfg.fid_msti;

	return 0;
}

static int rtl8367c_set_vlan( unsigned short vid, u32 mbr, u32 untag, u8 fid)
{	
	rtk_vlan_cfg_t vlan_cfg;
	int i;

	memset(&vlan_cfg, 0x00, sizeof(rtk_vlan_cfg_t));	

	for (i = 0; i < 32; i++) {
		if (mbr & (1 << i)) {
			RTK_PORTMASK_PORT_SET(vlan_cfg.mbr, i);
			if(untag & (1 << i))
				RTK_PORTMASK_PORT_SET(vlan_cfg.untag, i);			
		}
	}
	vlan_cfg.fid_msti=fid;
	vlan_cfg.ivl_en = 1;
	return rtk_vlan_set(vid, &vlan_cfg);
}


static int rtl8367c_get_pvid( int port, int *pvid)
{
	u32 prio=0;

	return rtk_vlan_portPvid_get(port,pvid,&prio);
}


static int rtl8367c_set_pvid( int port, int pvid)
{
	u32 prio=0;

	return rtk_vlan_portPvid_set(port,pvid,prio);
}

static int rtl8367c_get_port_link(int port, int *link, int *speed, int *duplex)
{
	
	if(rtk_port_phyStatus_get(port,(rtk_port_linkStatus_t *)link,
					(rtk_port_speed_t *)speed,(rtk_port_duplex_t *)duplex))
		return -EINVAL;

	return 0;
}



/*common rtl8367 swconfig entry API*/

static int
rtl8367_sw_set_vlan_enable(struct switch_dev *dev,
			   const struct switch_attr *attr,
			   struct switch_val *val)
{
	struct rtl8367_priv *priv = container_of(dev, struct rtl8367_priv, swdev);	

	priv->global_vlan_enable = val->value.i ;

	return 0;
}

static int
rtl8367_sw_get_vlan_enable(struct switch_dev *dev,
			   const struct switch_attr *attr,
			   struct switch_val *val)
{
	struct rtl8367_priv *priv = container_of(dev, struct rtl8367_priv, swdev);

	val->value.i = priv->global_vlan_enable;

	return 0;
}

static int rtl8367_sw_reset_mibs(struct switch_dev *dev,
				  const struct switch_attr *attr,
				  struct switch_val *val)
{
	int ret;
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	
	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);	
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;	
	
	ret = rtl8367c_reset_mibs();
	
	MII_UNLOCK();
	
	return ret;
}


static int rtl8367_sw_reset_port_mibs(struct switch_dev *dev,
				       const struct switch_attr *attr,
				       struct switch_val *val)
{
	int port,ret;
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	
	port = val->port_vlan;
	if (port >= dev->ports)
		return -EINVAL;

	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);	
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;
	
	ret = rtl8367c_reset_port_mibs(port);
	
	MII_UNLOCK();
	return ret;
}

static int rtl8367_sw_get_port_mib(struct switch_dev *dev,
			    const struct switch_attr *attr,
			    struct switch_val *val)
{	
	int i, len = 0;
	unsigned long long counter = 0;
	static char mib_buf[4096];
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	
	if (val->port_vlan >= dev->ports)
		return -EINVAL;

	
	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);	
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;
	
	len += snprintf(mib_buf + len, sizeof(mib_buf) - len,
			"Port %d MIB counters\n",
			val->port_vlan);	

	for (i = 0; i <rtl8367c_get_mibs_num(); ++i) {
		len += snprintf(mib_buf + len, sizeof(mib_buf) - len,
				"%-36s: ",rtl8367c_get_mib_name(i));
		if (!rtl8367c_get_port_mib_counter(i, sw_port_to_phy_port(val->port_vlan),
					       &counter))
			len += snprintf(mib_buf + len, sizeof(mib_buf) - len,
					"%llu\n", counter);
		else
			len += snprintf(mib_buf + len, sizeof(mib_buf) - len,
					"%s\n", "N/A");
	}	
	
	val->value.s = mib_buf;
	val->len = len;

	MII_UNLOCK();
	
	return 0;
}


static int rtl8367_sw_get_vlan_info(struct switch_dev *dev,
			     const struct switch_attr *attr,
			     struct switch_val *val)
{	
	int i;
	u32 len = 0;
	struct rtl8367_vlan_info vlan;
	static char vlan_buf[256];
	int err;
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	
	if (!rtl8367c_is_vlan_valid(val->port_vlan))
		return -EINVAL;

	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);	
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;
	
	memset(vlan_buf, '\0', sizeof(vlan_buf));

	err = rtl8367c_get_vlan(val->port_vlan, &vlan);
	if (err)
	{
		MII_UNLOCK();
		return err;
	}
	
	MII_UNLOCK();

	len += snprintf(vlan_buf + len, sizeof(vlan_buf) - len,
			"VLAN %d: Ports: '", vlan.vid);

	for (i = 0; i < dev->ports; i++) {
		if (!(vlan.member & (1 << i)))
			continue;

		len += snprintf(vlan_buf + len, sizeof(vlan_buf) - len, "%d%s", i,
				(vlan.untag & (1 << i)) ? "" : "t");
	}

	len += snprintf(vlan_buf + len, sizeof(vlan_buf) - len,
			"', members=%04x, untag=%04x, fid=%u",
			vlan.member, vlan.untag, vlan.fid);

	val->value.s = vlan_buf;
	val->len = len;

	
	
	return 0;
}


static int rtl8367_sw_get_vlan_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct switch_port *port;
	struct rtl8367_vlan_info vlan;
	int i;	
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	
	if (!rtl8367c_is_vlan_valid(val->port_vlan))
		return -EINVAL;
	
	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;

	if(rtl8367c_get_vlan(val->port_vlan, &vlan))
	{
		MII_UNLOCK();
		return -EINVAL;
	}
	MII_UNLOCK();
	
	port = &val->value.ports[0];
	val->len = 0;
	for (i = 0; i < dev->ports ; i++) {
		if (!(vlan.member & BIT(sw_port_to_phy_port(i))))
			continue;

		port->id = i;
		port->flags = (vlan.untag & BIT(sw_port_to_phy_port(i))) ?
					0 : BIT(SWITCH_PORT_FLAG_TAGGED);
		val->len++;
		port++;
	}
	return 0;
}


static int rtl8367_sw_set_vlan_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct switch_port *port;
	u32 member = 0;
	u32 untag = 0;
	u8 fid=0;
	int err;
	int i;	
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	
	if (!rtl8367c_is_vlan_valid(val->port_vlan))
		return -EINVAL;
	
	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;

	port = &val->value.ports[0];
	for (i = 0; i < val->len; i++, port++)
	{
		int pvid = 0;
		member |= BIT(sw_port_to_phy_port(port->id));

		if (!(port->flags & BIT(SWITCH_PORT_FLAG_TAGGED)))
			untag |= BIT(sw_port_to_phy_port(port->id));
		
	}

	//pr_info("[%s] vid=%d , mem=%x,untag=%x,fid=%d \n",__func__,val->port_vlan,member,untag,fid);

	err = rtl8367c_set_vlan(val->port_vlan, member, untag, fid);
	if(err)
	{
		MII_UNLOCK();
		return -EINVAL;
	}

	port = &val->value.ports[0];
	
	for (i = 0; i < val->len; i++, port++)
	{
		int pvid = 0;		
		
		/*let cpu_port PVID = 0*/
		if (port->id == dev->cpu_port)	
			continue;
		/*
		 * To ensure that we have a valid MC entry for this VLAN,
		 * initialize the port VLAN ID here.
		 */
		
		err = rtl8367c_set_pvid(sw_port_to_phy_port(port->id), val->port_vlan);
		if (err < 0)
		{
			MII_UNLOCK();
			return err;
		}

	}
	MII_UNLOCK();

	return 0;
}


static int rtl8367_sw_get_port_pvid(struct switch_dev *dev, int port, int *val)
{
	int ret = 0;
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;
	
	ret = rtl8367c_get_pvid(sw_port_to_phy_port(port), val);

	MII_UNLOCK();
	return ret;
}


static int rtl8367_sw_set_port_pvid(struct switch_dev *dev, int port, int val)
{
	int ret = 0;
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;
	
	ret = rtl8367c_set_pvid(sw_port_to_phy_port(port), val);
	MII_UNLOCK();
	return ret;
}


static int rtl8367_sw_reset_switch(struct switch_dev *dev)
{
	if(rtl8367_switch_reset_func)
		rtl8367_switch_reset_func();
	else
		printk("rest switch is not supported\n");

	return 0;
}

static int rtl8367_sw_get_port_link(struct switch_dev *dev, int port,
				    struct switch_port_link *link)
{	
	int speed;	
	sw_port_to_phy_port_func_t sw_port_to_phy_port;
	
	MII_LOCK();
	set_curr_mii(gMii[dev->id - 1].bus);
	sw_port_to_phy_port = gMii[dev->id - 1].sw_port_to_phy_port;

	if(rtl8367c_get_port_link(sw_port_to_phy_port(port),(int *)&link->link,(int *)&speed,(int *)&link->duplex))
	{
		MII_UNLOCK();
		return -EINVAL;		
	}

	MII_UNLOCK();

	if (!link->link)
		return 0;	

	switch (speed) {
	case 0:
		link->speed = SWITCH_PORT_SPEED_10;
		break;
	case 1:
		link->speed = SWITCH_PORT_SPEED_100;
		break;
	case 2:
		link->speed = SWITCH_PORT_SPEED_1000;
		break;
	default:
		link->speed = SWITCH_PORT_SPEED_UNKNOWN;
		break;
	}

	return 0;
}


static struct switch_attr rtl8367_globals[] = {
	{
		.type = SWITCH_TYPE_INT,
		.name = "enable_vlan",
		.description = "Enable VLAN mode",
		.set = rtl8367_sw_set_vlan_enable,
		.get = rtl8367_sw_get_vlan_enable,
		.max = 1,		
	}, {		
		.type = SWITCH_TYPE_NOVAL,
		.name = "reset_mibs",
		.description = "Reset all MIB counters",
		.set = rtl8367_sw_reset_mibs,
	}
};

static struct switch_attr rtl8367_port[] = {
	{
		.type = SWITCH_TYPE_NOVAL,
		.name = "reset_mib",
		.description = "Reset single port MIB counters",
		.set = rtl8367_sw_reset_port_mibs,
	},
/*
	{
		.type = SWITCH_TYPE_STRING,
		.name = "mib",
		.description = "Get MIB counters for port",
		//.max = 33,
		.set = NULL,
		.get = NULL,//rtl8367_sw_get_port_mib,
	},
*/	
};

static struct switch_attr rtl8367_vlan[] = {
	{
		.type = SWITCH_TYPE_STRING,
		.name = "info",
		.description = "Get vlan information",
		.max = 1,
		.set = NULL,
		.get = rtl8367_sw_get_vlan_info,
	},
};

static const struct switch_dev_ops rtl8367_sw_ops = {
	.attr_global = {
		.attr = rtl8367_globals,
		.n_attr = ARRAY_SIZE(rtl8367_globals),
	},
	.attr_port = {
		.attr = rtl8367_port,
		.n_attr = ARRAY_SIZE(rtl8367_port),
	},
	.attr_vlan = {
		.attr = rtl8367_vlan,
		.n_attr = ARRAY_SIZE(rtl8367_vlan),
	},

	.get_vlan_ports = rtl8367_sw_get_vlan_ports,
	.set_vlan_ports = rtl8367_sw_set_vlan_ports,
	.get_port_pvid = rtl8367_sw_get_port_pvid,
	.set_port_pvid = rtl8367_sw_set_port_pvid,
	.reset_switch = rtl8367_sw_reset_switch,
	.get_port_link = rtl8367_sw_get_port_link,
};

int rtl8367s_swconfig_init(void (*reset_func)(void))
{	
	rtl8367_switch_reset_func = reset_func ;
	
	return 0;
}

static int rtl8367s_hw_reset(int x)
{
	
	int ret;

	if (gMii[x].reset_pin < 0)
		return 0;

	ret = gpio_request( gMii[x].reset_pin, "gsw,reset-pin");

	if (ret)
        printk("fail to devm_gpio_request\n");

	gpio_direction_output(gMii[x].reset_pin, 0);

	mdelay(1000);	

	gpio_set_value(gMii[x].reset_pin, 1);

	mdelay(100);	

	//gpio_free( gMii.reset_pin);

	return 0;
	
}


static int rtl8367s_hw_init(void)
{	
	if(rtk_switch_init())
	        return -1;

	mdelay(500);

	if (rtk_vlan_reset())
	        return -1;

	if (rtk_vlan_init())
	        return -1;

	return 0;
}

static void set_rtl8367s_rgmii(void)
{
	rtk_port_mac_ability_t mac_cfg;
	rtk_mode_ext_t mode;

	mode = MODE_EXT_RGMII;
	mac_cfg.forcemode = MAC_FORCE;
	mac_cfg.speed = PORT_SPEED_1000M;
	mac_cfg.duplex = PORT_FULL_DUPLEX;
	mac_cfg.link = PORT_LINKUP;
	mac_cfg.nway = DISABLED;
	mac_cfg.txpause = ENABLED;
	mac_cfg.rxpause = ENABLED;
	rtk_port_macForceLinkExt_set(EXT_PORT0, mode, &mac_cfg);
	rtk_port_rgmiiDelayExt_set(EXT_PORT0, txdelay ? 1 : 0, rxdelay%8);
	rtk_port_phyEnableAll_set(ENABLED);	
}

void init_gsw(void)
{
	rtk_portmask_t portmask;
	
	rtl8367s_hw_init();	
	set_rtl8367s_rgmii();
	
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT0);
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT1);
	rtk_led_enable_set(LED_GROUP_0, &portmask);
	rtk_led_enable_set(LED_GROUP_1, &portmask);
	rtk_led_operation_set(LED_OP_PARALLEL);
	rtk_led_groupConfig_set(LED_GROUP_0, LED_CONFIG_LINK_ACT);
	rtk_led_groupConfig_set(LED_GROUP_1, LED_CONFIG_DUPCOL);
}


static const struct of_device_id rtk_gsw_match[] = {
	{ .compatible = "rtk,rtk8363", .data = (void*)&rtl8363 },
	{ .compatible = "rtk,rtk8367", .data = (void*)&rtl8367 },
	{},
};

MODULE_DEVICE_TABLE(of, rtk_gsw_match);

static int rtk_gsw_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *mdio;
	struct mii_bus *mdio_bus;
	struct sw_chip_info *sw_chip = NULL;
	struct switch_dev *swdev = NULL;	
	const char *pm;
	u32 cpu_port = 0;
	
	of_property_read_u32(np, "cpuport", &cpu_port);
	
	mdio = of_parse_phandle(np, "mdio", 0);

	if (!mdio)
		return -EINVAL;

	mdio_bus = of_mdio_find_bus(mdio);

	if (!mdio_bus)
		return -EPROBE_DEFER;
	
	sw_chip = (struct sw_chip_info *)of_device_get_match_data(&pdev->dev);

	gMii[mii_cnt].bus = mdio_bus;
	gMii[mii_cnt].vlanen = 0;
	gMii[mii_cnt].ports = sw_chip->ports;
	gMii[mii_cnt].sw_port_to_phy_port = sw_chip->sw_port_to_phy_port;	
	gMii[mii_cnt].reset_pin = of_get_named_gpio(np, "gsw,reset-pin", 0);	

	MII_LOCK();
	set_curr_mii(mdio_bus);
	
	rtl8367s_hw_reset(mii_cnt);
	
	init_gsw();

	MII_UNLOCK();

	swdev = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	
	if(!swdev)
	{
		return -1;
	}
	
	platform_set_drvdata(pdev,swdev);

	swdev->cpu_port = cpu_port;
	swdev->ports = sw_chip->ports;
	swdev->vlans = sw_chip->vlans;
	swdev->ops = &rtl8367_sw_ops;
	swdev->name = sw_chip->name;
	swdev->alias = sw_chip->name;	
	register_switch(swdev,NULL);

	mii_cnt ++;
	
	gsw_debug_proc_init();

	debug_line();
	return 0;
	
}

static int rtk_gsw_remove(struct platform_device *pdev)
{	
	struct switch_dev * swdev = (struct switch_dev *)platform_get_drvdata(pdev);

	debug_line();
	printk("remove swdev->id = %d \n",swdev->id);
	
	

	mii_cnt --;

	if(mii_cnt == 0)
	{
		int i ;
		for( i = 0 ; i < MAX_SW_NUM; i++ )
		{
			if( gMii[swdev->id - 1].reset_pin > 0)
			{
				gpio_free( gMii[swdev->id - 1].reset_pin);		
			}
		}
	}
	
	unregister_switch(swdev);

	kfree(swdev);
	platform_set_drvdata(pdev, NULL);	
	gsw_debug_proc_exit();

	return 0;
}


static struct platform_driver gsw_driver = {
	.probe = rtk_gsw_probe,
	.remove = rtk_gsw_remove,
	.driver = {
		.name = "rtk-gsw",
		.owner = THIS_MODULE,
		.of_match_table = rtk_gsw_match,
	},
};

module_platform_driver(gsw_driver);

MODULE_LICENSE("GPL");


module_param(rxdelay, int, 0600);
module_param(txdelay, int, 0600);


