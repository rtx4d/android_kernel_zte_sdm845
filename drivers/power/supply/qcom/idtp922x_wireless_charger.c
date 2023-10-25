/**
 * @file   p922x_wireless_charger.c
 * @author	<sun.shaojie@zte.com.cn>
 */

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/pinctrl/consumer.h>
#include <linux/qpnp/qpnp-revid.h>

#include "idtp922x_wireless_charger.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/alarmtimer.h>

static int p922x_debug_mask = 0xff;
module_param_named(
	debug_mask, p922x_debug_mask, int, 0600
);

#define p922x_err(idt, fmt, ...)		\
	pr_err("%s: %s: " fmt, idt->name,	\
		__func__, ##__VA_ARGS__)	\

#define p922x_dbg(idt, reason, fmt, ...)			\
	do {							\
		if (p922x_debug_mask & (reason))		\
			pr_err("%s: %s: " fmt, idt->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_err("%s: %s: " fmt, idt->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)


struct p922x_dev *idt;

struct idtp9220_access_func {
	int (*read)(struct p922x_dev *di, u16 reg, u8 *val);
	int (*write)(struct p922x_dev *di, u16 reg, u8 val);
	int (*read_buf)(struct p922x_dev *di,
					u16 reg, u8 *buf, u32 size);
	int (*write_buf)(struct p922x_dev *di,
					 u16 reg, u8 *buf, u32 size);
};

#define VENDOR_NAME_SIZE 12
struct p922x_dev {
	char				*name;
	struct i2c_client	 *client;
	struct device		*dev;
	struct regmap		*regmap;
	struct idtp9220_access_func bus;
	struct mutex		irq_complete;
	struct mutex		write_lock;
	struct mutex		send_pkg_lock;
	bool				resume_completed;
	bool				irq_waiting;
	int int_pin;
	int power_good_pin;
	bool power_good;
	bool id_auth_success;
	bool device_auth_success;
	bool fod_5v_enabled;
	bool fod_9v_enabled;
	struct pinctrl		 *p922x_gpio_pinctrl;
	struct pinctrl_state *p922x_gpio_state;
	struct pinctrl		 *p922x_en_pin_pinctrl;
	struct pinctrl_state *p922x_en_pin_active;
	struct pinctrl_state *p922x_en_pin_suspend;
	struct dentry		 *debug_root;
	struct alarm		 fod_voltage_check_alarm;
	struct delayed_work  fod_voltage_check_work;
	struct delayed_work  device_key_auth_work;
	struct delayed_work  get_tx_adapter_work;
	struct delayed_work  e_trans_show_work;
	struct power_supply *idtp922x_psy;
	int idt_adapter_type;
	int idt_tx_iin;
	int idt_tx_vin;
	int idt_rx_iout;
	int idt_rx_vout;
	int idt_tx_freq;
	int idt_fast_charging_voltage;
	int idt_package_send_success;
	int idt_tx_data_recive;
	int trans_efficiency;
	int thermal_state;
	int signal_strength_good_threshold;
	u8 signal_strength;
	int board_id;
	char coil_vendor_name[VENDOR_NAME_SIZE];
};

enum {
	THERMAL_GOOD_STATE = 0,
	THERMAL_WARM_STATE,
	THERMAL_HOT_STATE,
};

enum {
	IDT_REGULATE_5V_RX_VOUT = 0,
	IDT_REGULATE_9V_RX_VOUT,
};

static int p922x_irq_clr(struct p922x_dev *chip);
static int p922x_irq_disable(struct p922x_dev *chip);
static int p922x_irq_enable(struct p922x_dev *chip);

int idtp9220_read(struct p922x_dev *di, u16 reg, u8 *val)
{
	unsigned int temp;
	int rc;

	mutex_lock(&di->write_lock);

	rc = regmap_read(di->regmap, reg, &temp);
	if (rc >= 0)
		*val = (u8)temp;
	else
		dev_err(di->dev, "idtp9220 read error: %d\n", rc);
	mutex_unlock(&di->write_lock);

	return rc;
}

int idtp9220_write(struct p922x_dev *di, u16 reg, u8 val)
{
	int rc = 0;

	mutex_lock(&di->write_lock);
	rc = regmap_write(di->regmap, reg, val);
	if (rc < 0)
		dev_err(di->dev, "idtp9220 write error: %d\n", rc);
	mutex_unlock(&di->write_lock);

	return rc;
}

static int idtp9220_masked_write(struct p922x_dev *chip, u16 addr, u8 mask, u8 val)
{
	int rc;

	mutex_lock(&chip->write_lock);
	rc = regmap_update_bits(chip->regmap, addr, mask, val);
	mutex_unlock(&chip->write_lock);
	return rc;
}

int idtp9220_read_buffer(struct p922x_dev *di, u16 reg, u8 *buf, u32 size)
{
	int ret;

	mutex_lock(&di->write_lock);
	ret = regmap_bulk_read(di->regmap, reg, buf, size);
	mutex_unlock(&di->write_lock);
	return ret;
}

int idtp9220_write_buffer(struct p922x_dev *di, u16 reg, u8 *buf, u32 size)
{
	int rc = 0;

	while (size--) {
		rc = di->bus.write(di, reg++, *buf++);
		if (rc < 0) {
			dev_err(di->dev, "write error: %d\n", rc);
			return rc;
		}
	}
	return rc;
}

int ExtractPacketSize(u8 hdr)
{
	if (hdr < 0x20)
		return 1;
	if (hdr < 0x80)
		return (2 + ((hdr - 0x20) >> 4));
	if (hdr < 0xe0)
		return (8 + ((hdr - 0x80) >> 3));
	return (20 + ((hdr - 0xe0) >> 2));
}

void clritr(u16 s)
{
	idt->bus.write_buf(idt, REG_INT_CLEAR, (u8 *)&s, 2);
	idt->bus.write(idt, REG_COMMAND, CLRINT);
}

int checkitr(u16 s)
{
	u8 buf[2];
	u16 itr;
	int i = 0;

	for (i = 0; i < 10; i++) {
		idt->bus.read_buf(idt, REG_INTR, buf, 2);
		itr = buf[0]|(buf[1]<<8);
		if (itr & s) {
			p922x_dbg(idt, PR_DEBUG, "return true\n");
			return true;
		}
		p922x_dbg(idt, PR_DEBUG, "not true, retry.\n");
		msleep(200);
	}
	return false;
}

#define TIMEOUT_COUNT 100
bool sendPkt(struct p922x_dev *chip, ProPkt_Type *pkt)
{
	int length = ExtractPacketSize(pkt->header)+1;
	int i;
	bool ret = true;

	mutex_lock(&chip->send_pkg_lock);
	p922x_dbg(chip, PR_DEBUG, "length:%d, keep send_pkg_lock\n", length);
	/* write data into proprietary packet buffer*/
	chip->bus.write_buf(chip, REG_PROPPKT_ADDR, (u8 *)pkt, length);
	/* send proprietary packet*/
	chip->bus.write(chip, REG_COMMAND, SENDPROPP);
	chip->idt_tx_data_recive = DATA_RCV_SEND;

	for (i = 0; i < TIMEOUT_COUNT; i++) {
		if (chip->idt_tx_data_recive != DATA_RCV_SEND) {
			break;
		}
		msleep(20);
	}
	if (i == TIMEOUT_COUNT) {
		p922x_irq_clr(chip);
		p922x_dbg(chip, PR_DEBUG, "no int recived, try again\n");
		chip->bus.write_buf(chip, REG_PROPPKT_ADDR, (u8 *)pkt, length);
		chip->bus.write(chip, REG_COMMAND, SENDPROPP);
		for (i = 0; i < TIMEOUT_COUNT; i++) {
			if (chip->idt_tx_data_recive != DATA_RCV_SEND) {
				break;
			}
			msleep(20);
		}
		if (i == TIMEOUT_COUNT) {
			chip->idt_tx_data_recive = DATA_RCV_NO_INT;
		}
	}
	p922x_dbg(chip, PR_DEBUG, "chip->idt_tx_data_recive:%d\n", chip->idt_tx_data_recive);
	if (chip->idt_tx_data_recive == DATA_RCV_WAIT_SUCCESS) {
		p922x_dbg(chip, PR_DEBUG, "send package successful\n");
		ret = true;
		goto out;
	} else {
		p922x_dbg(chip, PR_DEBUG, "send package failed\n");
		ret = false;
		goto out;
	}

out:
	p922x_dbg(chip, PR_DEBUG, "release send_pkg_lock\n");
	mutex_unlock(&chip->send_pkg_lock);
	return ret;
}

int receivePkt(u8 *data)
{
	u8 header;
	u8 length;

	if (checkitr(TXDATARCVD)) {
		idt->bus.read(idt, REG_BCHEADER_ADDR, &header);
		length = ExtractPacketSize(header);
		idt->bus.read_buf(idt, REG_BCDATA_ADDR, data, length);
		clritr(TXDATARCVD);
		return true;
	}
	return false;
}

int p922x_get_received_data(struct p922x_dev *chip, u8 *data)
{
	u8 header;
	u8 length;
	int rc;

	rc = chip->bus.read(chip, REG_BCHEADER_ADDR, &header);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n",
				REG_BCHEADER_ADDR, rc);
		return rc;
	}
	length = ExtractPacketSize(header);
	rc = chip->bus.read_buf(chip, REG_BCDATA_ADDR, data, length);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n",
				REG_BCDATA_ADDR, rc);
	}
	return rc;
}

ssize_t p922x_get_firmware_ver(struct p922x_dev *chip)
{
	int i = 0;

	u8 id[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, ver[4] = {0xff, 0xff, 0xff, 0xff};

	chip->bus.read_buf(chip, REG_CHIP_ID, id, 8);
	chip->bus.read_buf(chip, REG_CHIP_REV, ver, 4);

	for (i = 0; i < 8 ; i++) {
		p922x_dbg(chip, PR_DEBUG, "id[%d]=0x%02x\n", i, id[i]);
	}
	p922x_dbg(chip, PR_DEBUG, "IDT ChipID:%04x\nFWVer:%02x.%02x.%02x.%02x\n",
	id[4]|(id[0]<<8), ver[3], ver[2], ver[1], ver[0]);

	return 0;
}

int p922x_get_system_mode(void)
{
	u8 mode;

	idt->bus.read(idt, REG_MODE_ADDR, &mode);
	p922x_dbg(idt, PR_DEBUG, "system_mode:0x%x\n", mode);
	return mode;
}

void p922x_get_rx_firmware_version(void)
{
	u8 ver_data[4];
	int mode;

	idt->bus.read_buf(idt, REG_OTPFWVER_ADDR, ver_data, 4);
	p922x_dbg(idt, PR_DEBUG, "FWVer:%02x.%02x.%02x.%02x\n",
		ver_data[3], ver_data[2], ver_data[1], ver_data[0]);

	idt->bus.read_buf(idt, REG_CHIP_REV, ver_data, 4);
	mode = p922x_get_system_mode();
	if (mode & EEPROMSUPPORT) {
		p922x_dbg(idt, PR_DEBUG, "EEPROM:%02x.%02x.%02x.%02x\n",
			ver_data[3], ver_data[2], ver_data[1], ver_data[0]);
	} else if (mode & RAMPROGRAM) {
		p922x_dbg(idt, PR_DEBUG, "SRAM:%02x.%02x.%02x.%02x\n",
			ver_data[3], ver_data[2], ver_data[1], ver_data[0]);
	} else {
		p922x_dbg(idt, PR_DEBUG, "APP:%02x.%02x.%02x.%02x\n",
			ver_data[3], ver_data[2], ver_data[1], ver_data[0]);
	}
}

int p922x_get_rx_frequency(void)
{
	u8 ver_data[2];

	idt->bus.read_buf(idt, REG_FREQ_ADDR, ver_data, 2);
	p922x_dbg(idt, PR_DEBUG, "rx_frequency:%d\n", ver_data[0] | ver_data[1]<<8);
	return (ver_data[0] | ver_data[1]<<8);
}

int p922x_get_rx_vrect(void)
{
	u8 ver_data[2];

	idt->bus.read_buf(idt, REG_VRECT_ADDR, ver_data, 2);
	p922x_dbg(idt, PR_DEBUG, "RX Vrect:%dmV\n", (ver_data[0] | ver_data[1] << 8) * 21 * 1000 / 4095);
	return ((ver_data[0] | ver_data[1] << 8) * 21 * 1000 / 4095);
}

int p922x_get_rx_iout(struct p922x_dev *chip)
{
	u8 ver_data[2];

	chip->bus.read_buf(chip, REG_RX_LOUT, ver_data, 2);
	p922x_dbg(chip, PR_DEBUG, "Iout:%dmA\n", (ver_data[0] | ver_data[1]<<8));
	return (ver_data[0] | ver_data[1]<<8);
}

int p922x_get_rx_vout(void)
{
	u8 ver_data[2];
	int vout = 0;

	idt->bus.read_buf(idt, REG_ADC_VOUT, ver_data, 2);
	vout = ((ver_data[0] | ver_data[1]<<8)*6*21*1000/40950);
	p922x_dbg(idt, PR_DEBUG, "Vout:%dmV\n", vout);
	return vout;
}

void p922x_set_rx_vout(ushort vol)
{
	int val;

	val = vol*10 - 35;
	idt->bus.write(idt, REG_VOUT_SET, val);
}

int p922x_get_rx_ilimit(void)
{
	u8 data;
	int ilimit = 0;

	idt->bus.read(idt, REG_ILIM_SET, &data);
	ilimit = (data+1)/10;
	p922x_dbg(idt, PR_DEBUG, "ilimt:%d\n", ilimit);
	return ilimit;
}

void p922x_get_tx_atapter_type(struct p922x_dev *chip)
{
	ProPkt_Type proPkt;

	proPkt.header = PROPRIETARY18;
	proPkt.cmd	  = BC_ADAPTER_TYPE;
	sendPkt(chip, &proPkt);
}

#define RX_OUTPUT_CURRENT_UA_LOW		250000
#define RX_OUTPUT_CURRENT_UA_MEDIUM	750000
#define RX_OUTPUT_CURRENT_UA_HIGH	1100000
int p922x_get_rx_output_current_max(struct p922x_dev *chip)
{
	int output_current = RX_OUTPUT_CURRENT_UA_MEDIUM;

	if (chip->id_auth_success && chip->device_auth_success
		&& chip->idt_adapter_type >= ADAPTER_QC20) {
		output_current = RX_OUTPUT_CURRENT_UA_HIGH;
	} else if (chip->id_auth_success && chip->device_auth_success
		&& chip->idt_adapter_type == ADAPTER_SDP) {
		output_current = RX_OUTPUT_CURRENT_UA_LOW;
	}

	p922x_dbg(chip, PR_DEBUG, "output_current = %d\n", output_current);
	return output_current;
}

#define RX_VOUT_5V 0xf
#define RX_VOUT_9V 0x37
#define QC_MIN_VOLTAGE 5000
#define QC_MAX_VOLTAGE 9000

void p922x_set_fast_charging_voltage(struct p922x_dev *chip, ushort mv)
{
	u8 val = 0;

	chip->idt_fast_charging_voltage = mv;
	chip->bus.write_buf(chip, REG_FC_VOLTAGE, (u8 *)&mv, 2);
	chip->bus.write(chip, REG_COMMAND, VSWITCH);

	msleep(20);
	idt->bus.read(idt, REG_VOUT_SET, &val);
	if ((val == RX_VOUT_5V && mv == QC_MIN_VOLTAGE)
		|| (val == RX_VOUT_9V && mv == QC_MAX_VOLTAGE)) {
		p922x_dbg(chip, PR_INTERRUPT, "set voltage: %dmv\n", mv);
	} else {
		p922x_dbg(chip, PR_INTERRUPT, "set voltage error\n");
	}
}

void p922x_toggle_ldo(void)
{
	idt->bus.write(idt, REG_COMMAND, LDOTGL);
}

void p922x_get_tx_firmware_version(void)
{
	ProPkt_Type proPkt;

	proPkt.header = PROPRIETARY18;
	proPkt.cmd	  = BC_READ_FW_VER;
	sendPkt(idt, &proPkt);
}

int p922x_get_tx_iin(void)
{
	ProPkt_Type proPkt;

	proPkt.header = PROPRIETARY18;
	proPkt.cmd	  = BC_READ_Iin;
	sendPkt(idt, &proPkt);
	return idt->idt_tx_iin;
}

int p922x_get_tx_vin(struct p922x_dev *chip)
{
	ProPkt_Type proPkt;

	proPkt.header = PROPRIETARY18;
	proPkt.cmd	  = BC_READ_Vin;
	sendPkt(chip, &proPkt);
	return chip->idt_tx_vin;
}

void p922x_set_tx_vin(int mv)
{
	ProPkt_Type proPkt;

	proPkt.header  = PROPRIETARY38;
	proPkt.cmd	   = BC_SET_Vin;
	proPkt.data[0] = mv&0xff;
	proPkt.data[1] = (mv>>8)&0xff;
	sendPkt(idt, &proPkt);
	idt->idt_tx_vin = mv;
	p922x_dbg(idt, PR_DEBUG, "set vin:%d\n", mv);
}

void p922x_set_tx_frequency(int freq)
{
	ProPkt_Type proPkt;

	proPkt.header  = PROPRIETARY38;
	proPkt.cmd	   = BC_SET_FREQ;
	proPkt.data[0] = freq&0xff;
	proPkt.data[1] = (freq>>8)&0xff;
	sendPkt(idt, &proPkt);
	idt->idt_tx_freq = freq;
	p922x_dbg(idt, PR_DEBUG, "set freq:%d\n", freq);
}

int p922x_get_tx_frequency(struct p922x_dev *chip)
{
	ProPkt_Type proPkt;

	proPkt.header  = PROPRIETARY18;
	proPkt.cmd	   = BC_GET_FREQ;
	sendPkt(chip, &proPkt);
	return chip->idt_tx_freq;
}

void p922x_toggle_loopmode(void)
{
	ProPkt_Type proPkt;

	proPkt.header  = PROPRIETARY18;
	proPkt.cmd	   = BC_TOGGLE_LOOPMODE;
	sendPkt(idt, &proPkt);
}

void p922x_system_reset(struct p922x_dev *chip)
{
	ProPkt_Type proPkt;

	proPkt.header  = PROPRIETARY18;
	proPkt.cmd	   = BC_RESET;
	sendPkt(chip, &proPkt);

	/* Reset RX*/
	chip->bus.write(chip, 0x3000, 0x5a);
	chip->bus.write(chip, 0x3040, 0x10);
	msleep(2);

	chip->bus.write(chip, 0x3000, 0x5a);
	chip->bus.write(chip, 0x3048, 0x00);
	chip->bus.write(chip, 0x3040, 0x80);

	p922x_dbg(idt, PR_DEBUG, "reset idt system\n");
}

static int program_bootloader(struct p922x_dev *di)
{
	int i, rc = 0;
	int len;

	len = sizeof(bootloader);

	for (i = 0; i < len; i++) {
		rc = di->bus.write(di, 0x1c00+i, bootloader[i]);
		if (rc)
			return rc;
	}

	return 0;
}

int program_fw(struct p922x_dev *di, u16 destAddr, u8 *src, u32 size)
{
	int i, j;
	u8 data = 0;

	/*=== Step-1 ===
	 Transfer 9220 boot loader code "OTPBootloader" to 9220 SRAM
	 - Setup 9220 registers before transferring the boot loader code
	 - Transfer the boot loader code to 9220 SRAM
	 - Reset 9220 => 9220 M0 runs the boot loader
	*/
	di->bus.read(di, 0x5870, &data);
	p922x_dbg(di, PR_DEBUG, "0x5870 :%02x\n", data);
	di->bus.read(di, 0x5874, &data);
	p922x_dbg(di, PR_DEBUG, "0x5874 :%02x\n", data);
	/*configure the system*/
	if (di->bus.write(di, 0x3000, 0x5a))
		return false;		 /*write key*/
	if (di->bus.write(di, 0x3040, 0x10))
		return false;		 /* halt M0 execution*/
	if (program_bootloader(di))
		return false;
	if (di->bus.write(di, 0x3048, 0x80))
		return false;		 /* map RAM to OTP*/

	/* ignoreNAK */
	di->bus.write(di, 0x3040, 0x80);		/* reset chip and run the bootloader*/
	mdelay(100);

	/* === Step-2 ===
	 Program OTP image data to 9220 OTP memory
	*/
	for (i = destAddr; i < destAddr+size; i += 128) {		 /* program pages of 128 bytes*/
		/* Build a packet*/
		char sBuf[136]; /* 136=8+128 --- 8-byte header plus 128-byte data*/
		u16 StartAddr = (u16)i;
		u16 CheckSum = StartAddr;
		u16 CodeLength = 128;
		int retry_cnt = 0;

		memset(sBuf, 0, 136);

		/*(1) Copy the 128 bytes of the OTP image data to the packet data buffer
		  Array.Copy(srcData, i + srcOffs, sBuf, 8, 128);// Copy 128 bytes from srcData (starting at i+srcOffs)
		Copy 128 bytes from srcData (starting at i+srcOffs)*/
		memcpy(sBuf+8, src, 128);
		src += 128;
		/*(2) Calculate the packet checksum of the 128-byte data, StartAddr, and CodeLength*/
		/* find the 1st non zero value byte from the end of the sBuf[] buffer*/
		for (j = 127; j >= 0; j--) {
			if (sBuf[j + 8] != 0)
				break;
			CodeLength--;
		}
		if (CodeLength == 0)
			continue;			 /* skip programming if nothing to program*/

		for (; j >= 0; j--)
			CheckSum += sBuf[j + 8];	/* add the nonzero values*/
		CheckSum += CodeLength; /* finish calculation of the check sum*/

		/*(3) Fill up StartAddr, CodeLength, CheckSum of the current packet.*/
		memcpy(sBuf+2, &StartAddr, 2);
		memcpy(sBuf+4, &CodeLength, 2);
		memcpy(sBuf+6, &CheckSum, 2);

		/* Send the current packet to 9220 SRAM via I2C*/
		/* read status is guaranteed to be != 1 at this point*/
		for (j = 0; j < CodeLength+8; j++) {
			if (di->bus.write(di, 0x400+j, sBuf[j])) {
				p922x_dbg(di, PR_DEBUG, "ERROR: on writing to OTP buffer");
				return false;
			}
		}

		/*Write 1 to the Status in the SRAM. This informs the 9220 to start programming the new packet
		 from SRAM to OTP memory*/
		if (di->bus.write(di, 0x400, 1))	{
			p922x_dbg(di, PR_DEBUG, "ERROR: on OTP buffer validation");
			return false;
		}

		/*
		 Wait for 9220 bootloader to complete programming the current packet image data from SRAM to the OTP.
		 The boot loader will update the Status in the SRAM as follows:
			 Status:
			 "0" - reset value (from AP)
			 "1" - buffer validated / busy (from AP)
			 "2" - finish "OK" (from the boot loader)
			 "4" - programming error (from the boot loader)
			 "8" - wrong check sum (from the boot loader)
			 "16"- programming not possible (try to write "0" to bit location already programmed to "1")
				 (from the boot loader)*/

		/*		  DateTime startT = DateTime.Now;*/
		do {
			mdelay(100);
			di->bus.read(di, 0x400, sBuf);
			if (sBuf[0] == 1) {
				p922x_dbg(di, PR_DEBUG, "Programming OTP buffer status sBuf:%02x i:%d\n",
					sBuf[0], i);
			}
			if (retry_cnt++ > 5)
				break;
		} while (sBuf[0] == 1); /*check if OTP programming finishes "OK"*/

		if (sBuf[0] != 2) { /* not OK*/
			p922x_dbg(di, PR_DEBUG, "ERROR: buffer write to OTP returned status:%d :%s\n",
				sBuf[0], "X4");
			return false;
		}
		p922x_dbg(di, PR_DEBUG, "Program OTP 0x%04x\n", i);
	}

	/* === Step-3 ===
	 Restore system (Need to reset or power cycle 9220 to run the OTP code)
	*/
	if (di->bus.write(di, 0x3000, 0x5a))
		return false;/* write key*/
	if (di->bus.write(di, 0x3048, 0x00))
		return false;/* remove code remapping*/
	return true;
}

static const struct of_device_id match_table[] = {
	{.compatible = "IDT,idt_wireless_power",},
	{ },
};

static const struct i2c_device_id p922x_dev_id[] = {
	{"idt_wireless_power", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, p922x_dev_id);

static int p922x_remove(struct i2c_client *client)
{
	return 0;
}

/* first step: define regmap_config*/
static const struct regmap_config p922x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};
/*debug interface start*/
static int addr = -1;
static int p922x_register_set_addr(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}

	p922x_dbg(idt, PR_DEBUG, "p922x_reg_addr:0x%x\n", addr);
	return 0;
}

static int p922x_register_get_addr(char *val, struct kernel_param *kp)
{
	p922x_dbg(idt, PR_DEBUG, "p922x_reg_addr:0x%x\n", addr);

	return	sprintf(val, "%x", addr);
}

module_param_call(addr, p922x_register_set_addr, p922x_register_get_addr,
					&addr, 0644);

static int data = -1;
static int p922x_register_set_val(const char *val, struct kernel_param *kp)
{
	int rc;
	u8 value;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}
	value = data;
	idt->bus.write(idt, addr, value);

	p922x_dbg(idt, PR_DEBUG, "p922x_reg_val:0x%x\n", data);
	return 0;
}

static int p922x_register_get_val(char *val, struct kernel_param *kp)
{
	u8 value;

	idt->bus.read(idt, addr, &value);
	data = value;
	p922x_dbg(idt, PR_DEBUG, "read addr:0x%x[0x%x]\n", addr, data);

	return	sprintf(val, "%x", data);
}

module_param_call(data, p922x_register_set_val, p922x_register_get_val,
					&data, 0644);

static int count = 0;
static int p922x_register_set_count(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting count %d\n", rc);
		return rc;
	}
	if (count == 0x5a) {
		p922x_system_reset(idt);
	}
	p922x_dbg(idt, PR_DEBUG, "count:0x%x\n", count);
	return 0;
}

static int p922x_register_get_count(char *val, struct kernel_param *kp)
{
	p922x_dbg(idt, PR_DEBUG, "count:0x%x\n", count);
	p922x_get_rx_firmware_version();
	p922x_get_tx_firmware_version();

	return	sprintf(val, "%x", count);
}

module_param_call(count, p922x_register_set_count, p922x_register_get_count,
					&count, 0644);

static int rx_fw_otp_write = 0;
static int p922x_rx_fw_otp_write(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error %d\n", rc);
		return rc;
	}
	if (rx_fw_otp_write) {
		p922x_irq_clr(idt);
		p922x_irq_disable(idt);
		alarm_cancel(&idt->fod_voltage_check_alarm);
		cancel_delayed_work(&idt->fod_voltage_check_work);
		cancel_delayed_work_sync(&idt->device_key_auth_work);
		cancel_delayed_work_sync(&idt->get_tx_adapter_work);
		cancel_delayed_work_sync(&idt->e_trans_show_work);
		if (!program_fw(idt, 0x0000, idtp9220_rx_fw_a129, sizeof(idtp9220_rx_fw_a129))) {
			p922x_dbg(idt, PR_DEBUG, "download_wireless_charger_firmware failed.\n");
		} else {
			p922x_dbg(idt, PR_DEBUG, "download_wireless_charger_firmware success.\n");
		}
		p922x_irq_enable(idt);
	}

	rx_fw_otp_write = 0;

	return 0;
}

module_param_call(rx_fw_otp_write, p922x_rx_fw_otp_write, NULL,
					&rx_fw_otp_write, 0644);

static int data_buf = -1;
static int p922x_register_set_buf(const char *val, struct kernel_param *kp)
{
	int rc;
	int value;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}
	value = data_buf;
	idt->bus.write_buf(idt, addr, (u8 *)&value, 2);

	p922x_dbg(idt, PR_DEBUG, "p922x_reg_val:0x%x\n", data_buf);
	return 0;
}

static int p922x_register_get_buf(char *val, struct kernel_param *kp)
{
	u8 value[2];
	int i;

	if (count) {
		for (i = 0; i < count; i++) {
			addr = addr + i*2;
			idt->bus.read_buf(idt, addr, value, 2);
			data_buf = value[0] | (value[1]<<8);
			p922x_dbg(idt, PR_DEBUG, "read addr:0x%x = 0x%x ,addr:0x%x = 0x%x\n",
				addr+1, value[1], addr, value[0]);
		}
	} else {
		idt->bus.read_buf(idt, addr, value, 2);
		data_buf = value[0] | (value[1]<<8);
		p922x_dbg(idt, PR_DEBUG, "read addr:0x%x = 0x%x ,addr:0x%x = 0x%x\n",
			addr+1, value[1], addr, value[0]);
	}
	return	sprintf(val, "%x", data_buf);
}

module_param_call(data_buf, p922x_register_set_buf, p922x_register_get_buf,
					&data_buf, 0644);

static int system_mode = -1;
static int p922x_get_system_mode_node(char *val, struct kernel_param *kp)
{
	system_mode = p922x_get_system_mode();
	return	sprintf(val, "%d", system_mode);
}

module_param_call(system_mode, NULL, p922x_get_system_mode_node,
					&system_mode, 0644);

static int adapter_type = ADAPTER_UNKNOWN;
static int p922x_get_tx_adapter_type_node(char *val, struct kernel_param *kp)
{
	return sprintf(val, "%d", adapter_type);
}

module_param_call(adapter_type, NULL, p922x_get_tx_adapter_type_node,
					&adapter_type, 0644);

static int rx_vrect = -1;
static int p922x_get_rx_vrect_node(char *val, struct kernel_param *kp)
{
	rx_vrect = p922x_get_rx_vrect();
	return	sprintf(val, "%d", rx_vrect);
}

module_param_call(rx_vrect, NULL, p922x_get_rx_vrect_node,
					&rx_vrect, 0644);

static int rx_iout = -1;
static int p922x_get_rx_iout_node(char *val, struct kernel_param *kp)
{
	rx_iout = p922x_get_rx_iout(idt);
	return	sprintf(val, "%d", rx_iout);
}

module_param_call(rx_iout, NULL, p922x_get_rx_iout_node,
					&rx_iout, 0644);

static int rx_vout = -1;
static int p922x_set_rx_vout_node(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}
	p922x_set_rx_vout(rx_vout);

	p922x_dbg(idt, PR_DEBUG, "rx_vout:%d\n", rx_vout);
	return 0;
}

static int p922x_get_rx_vout_node(char *val, struct kernel_param *kp)
{
	rx_vout = p922x_get_rx_vout();
	return	sprintf(val, "%d", rx_vout);
}

module_param_call(rx_vout, p922x_set_rx_vout_node, p922x_get_rx_vout_node,
					&rx_vout, 0644);

static int rx_ilimit = -1;
static int p922x_get_rx_ilimit_node(char *val, struct kernel_param *kp)
{
	rx_ilimit = p922x_get_rx_ilimit();
	return	sprintf(val, "%d", rx_ilimit);
}

module_param_call(rx_ilimit, NULL, p922x_get_rx_ilimit_node,
					&rx_ilimit, 0644);

static int fast_charging_voltage = -1;
static int p922x_set_fast_charging_voltage_node(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}
	p922x_set_fast_charging_voltage(idt, fast_charging_voltage);

	p922x_dbg(idt, PR_DEBUG, "fast_charging_voltage:%d\n", fast_charging_voltage);
	return 0;
}

module_param_call(fast_charging_voltage, p922x_set_fast_charging_voltage_node, NULL,
					&fast_charging_voltage, 0644);

static int tx_iin = -1;
static int p922x_get_tx_iin_node(char *val, struct kernel_param *kp)
{
	tx_iin = p922x_get_tx_iin();
	return	sprintf(val, "%d", tx_iin);
}

module_param_call(tx_iin, NULL, p922x_get_tx_iin_node,
					&tx_iin, 0644);

static int tx_vin = -1;
static int p922x_set_tx_vin_node(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}
	p922x_set_tx_vin(tx_vin);

	p922x_dbg(idt, PR_DEBUG, "tx_vin:%d\n", tx_vin);
	return 0;
}

static int p922x_get_tx_vin_node(char *val, struct kernel_param *kp)
{
	tx_vin = p922x_get_tx_vin(idt);
	return	sprintf(val, "%d", tx_vin);
}

module_param_call(tx_vin, p922x_set_tx_vin_node, p922x_get_tx_vin_node,
					&tx_vin, 0644);

static int tx_freq = -1;
static int p922x_set_tx_frequency_node(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}
	p922x_set_tx_frequency(tx_freq);

	p922x_dbg(idt, PR_DEBUG, "tx_freq:%d\n", tx_freq);
	return 0;
}

static int p922x_get_tx_frequency_node(char *val, struct kernel_param *kp)
{
	tx_freq = p922x_get_tx_frequency(idt);
	return	sprintf(val, "%d", tx_freq);
}

module_param_call(tx_freq, p922x_set_tx_frequency_node, p922x_get_tx_frequency_node,
					&tx_freq, 0644);

static bool use_fod_5v_dbg_enable = false;
static bool use_fod_9v_dbg_enable = false;
static unsigned int fod_5v_dbg = 0;
static unsigned int fod_9v_dbg = 0;
static unsigned char idtp9220_rx_fod_5v_dbg[12] = {0, };
static unsigned char idtp9220_rx_fod_9v_dbg[12] = {0, };
#define FOD_DBG_ARR_MAX 11
static int p922x_init_fod_dbg_node(const char *val, struct kernel_param *kp)
{
	int rc, i;
	static int position_5v = 0;
	static int position_9v = 0;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}

	p922x_dbg(idt, PR_DEBUG, "p922x_set_fod_5v_dbg_node: %d\n", fod_5v_dbg);
	if (fod_5v_dbg == 0) {
		for (i = 0; i <= FOD_DBG_ARR_MAX; i++) {
			idtp9220_rx_fod_5v_dbg[i] = 0;
		}
		use_fod_5v_dbg_enable = false;
		position_5v = 0;
	} else {
		switch (position_5v) {
		case 0:
		case 3:
		case 6:
		case 9:
			for (i = position_5v + 2; i >= position_5v; i--) {
				idtp9220_rx_fod_5v_dbg[i] = fod_5v_dbg % 1000;
				fod_5v_dbg = fod_5v_dbg / 1000;
				if (i == 9) {
					use_fod_5v_dbg_enable = true;
					for (i = 0; i <= FOD_DBG_ARR_MAX; i++) {
						p922x_dbg(idt, PR_DEBUG, "%d--0x%x\n",
							idtp9220_rx_fod_5v_dbg[i], idtp9220_rx_fod_5v_dbg[i]);
					}
					position_5v = 0;
					break;
				}
			}
			position_5v += 3;
			break;
		}
	}

	p922x_dbg(idt, PR_DEBUG, "p922x_set_fod_9v_dbg_node: %d\n", fod_9v_dbg);
	if (fod_9v_dbg == 0) {
		for (i = 0; i <= FOD_DBG_ARR_MAX; i++) {
			idtp9220_rx_fod_9v_dbg[i] = 0;
		}
		use_fod_9v_dbg_enable = false;
		position_9v = 0;
	} else {
		switch (position_9v) {
		case 0:
		case 3:
		case 6:
		case 9:
			for (i = position_9v + 2; i >= position_9v; i--) {
				idtp9220_rx_fod_9v_dbg[i] = fod_9v_dbg % 1000;
				fod_9v_dbg = fod_9v_dbg / 1000;
				if (i == 9) {
					use_fod_9v_dbg_enable = true;
					for (i = 0; i <= FOD_DBG_ARR_MAX; i++) {
						p922x_dbg(idt, PR_DEBUG, "%d--0x%x\n",
							idtp9220_rx_fod_9v_dbg[i], idtp9220_rx_fod_9v_dbg[i]);
					}
					position_9v = 0;
					break;
				}
			}
			position_9v += 3;
			break;
		}
	}

	return 0;
}

module_param_call(fod_5v_dbg, p922x_init_fod_dbg_node, NULL,
					&fod_5v_dbg, 0644);
module_param_call(fod_9v_dbg, p922x_init_fod_dbg_node, NULL,
					&fod_9v_dbg, 0644);

#define FOD_REGSTERS_NUM 12
#define TRY_MAX 5
static int p922x_fod_parameters_check(struct p922x_dev *chip, bool high_voltage)
{
	int rc = 0;
	int i = 0;
	u8 val[FOD_REGSTERS_NUM] = {0, };

	idt->bus.read_buf(idt, REG_FOD_START_ADDR, val, FOD_REGSTERS_NUM);
	for (i = 0; i < FOD_REGSTERS_NUM; i++) {
		if (high_voltage) {
			if (val[i] != idtp9220_rx_fod_9v[i]) {
				p922x_dbg(chip, PR_INTERRUPT, "fod 9v parameters is wrong\n");
				rc = -EINVAL;
				break;
			}
		} else {
			if (val[i] != idtp9220_rx_fod_5v[i]) {
				p922x_dbg(chip, PR_INTERRUPT, "fod 5v parameters is wrong\n");
				rc = -EINVAL;
				break;
			}
		}
	}

	return rc;
}

static int p922x_set_fod_5v(struct p922x_dev *chip)
{
	int rc = 0;
	int tries = 0;

	p922x_dbg(chip, PR_INTERRUPT, "fod dbg %s\n", use_fod_5v_dbg_enable ? "yes" : "no");

	for (tries = 0; tries < TRY_MAX; tries++) {
		if (!chip->fod_5v_enabled) {
			if (use_fod_5v_dbg_enable) {
				chip->bus.write_buf(chip, REG_FOD_START_ADDR, (u8 *)idtp9220_rx_fod_5v_dbg, FOD_REGSTERS_NUM);
				chip->fod_5v_enabled = true;
				chip->fod_9v_enabled = false;
				break;
			} else {
				chip->bus.write_buf(chip, REG_FOD_START_ADDR, (u8 *)idtp9220_rx_fod_5v, FOD_REGSTERS_NUM);
				rc = p922x_fod_parameters_check(chip, false);
				if (!rc) {
					chip->fod_5v_enabled = true;
					chip->fod_9v_enabled = false;
					break;
				}
			}
		}
		msleep(10);
	}

	return rc;
}

static int p922x_set_fod_9v(struct p922x_dev *chip)
{
	int rc = 0;
	int tries = 0;

	p922x_dbg(chip, PR_INTERRUPT, "fod dbg %s\n", use_fod_9v_dbg_enable ? "yes" : "no");

	for (tries = 0; tries < TRY_MAX; tries++) {
		if (!chip->fod_9v_enabled) {
			if (use_fod_9v_dbg_enable) {
				chip->bus.write_buf(chip, REG_FOD_START_ADDR, (u8 *)idtp9220_rx_fod_9v_dbg, FOD_REGSTERS_NUM);
				chip->fod_5v_enabled = false;
				chip->fod_9v_enabled = true;
				break;
			} else {
				chip->bus.write_buf(chip, REG_FOD_START_ADDR, (u8 *)idtp9220_rx_fod_9v, FOD_REGSTERS_NUM);
				rc = p922x_fod_parameters_check(chip, true);
				if (!rc) {
					chip->fod_5v_enabled = false;
					chip->fod_9v_enabled = true;
					break;
				}
			}
		}
		msleep(10);
	}

	return rc;
}

/* if lab_test node is set, 9V/5V switcher will ignore thermal-engine */
static int lab_test_mode = 0;
static int p922x_set_lab_test_node(const char *val, struct kernel_param *kp)
{
	int rc;

	rc = param_set_int(val, kp);
	if (rc) {
		p922x_dbg(idt, PR_DEBUG, "error setting value %d\n", rc);
		return rc;
	}

	cancel_delayed_work(&idt->fod_voltage_check_work);
	schedule_delayed_work(&idt->fod_voltage_check_work, msecs_to_jiffies(0));

	p922x_dbg(idt, PR_DEBUG, "lab_test_mode:%d\n", lab_test_mode);
	return 0;
}

module_param_call(lab_test_mode, p922x_set_lab_test_node, NULL,
					&lab_test_mode, 0644);

#define WIRELESS_CHARGING_SIGNAL_GOOD_THRESHOLD_DEFAULT 115
extern int wireless_charging_signal_good;
static void p922x_get_signal_strength(struct p922x_dev *chip)
{
	if (idt->power_good) {
		idt->bus.read(chip, REG_SIGNAL_STRENGTH, &idt->signal_strength);
		if (idt->signal_strength >= chip->signal_strength_good_threshold) {
			wireless_charging_signal_good = 1;
		} else {
			wireless_charging_signal_good = 0;
		}
	} else {
		idt->signal_strength = 0;
		wireless_charging_signal_good = 0;
	}

	p922x_dbg(idt, PR_DEBUG, "signal_strength:%d, signal_good:%d\n",
		idt->signal_strength, wireless_charging_signal_good);
}

/*debug interface end*/

static int over_curr_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_DEBUG, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int over_volt_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_DEBUG, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int over_temp_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_DEBUG, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int rx_ready_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_DEBUG, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int tx_data_rcvd_handler(struct p922x_dev *chip, u8 rt_stat)
{
#if 0
	ProPkt_Type proPkt;

	chip->bus.read(chip, REG_BCCMD_ADDR, &proPkt.cmd);
	p922x_get_received_data(chip, proPkt.data);
	p922x_dbg(chip, PR_DEBUG, "proPkt.cmd:0x%x,data_list:0x%x,%x,%x,%x\n",
		proPkt.cmd, proPkt.data[0], proPkt.data[1], proPkt.data[2], proPkt.data[3]);
	switch (proPkt.cmd) {
	case BC_NONE:
		break;
	case BC_SET_FREQ:
		p922x_dbg(chip, PR_DEBUG, "set idt_tx_freq%d success\n", chip->idt_tx_freq);
		break;
	case BC_GET_FREQ:
		chip->idt_tx_freq = (proPkt.data[0] | proPkt.data[1]<<8);
		p922x_dbg(chip, PR_DEBUG, "get idt_tx_freq%d\n", chip->idt_tx_freq);
		break;
	case BC_READ_FW_VER:
		p922x_dbg(chip, PR_DEBUG, "TX Version is: %d.%d.%d.%d\n",
			proPkt.data[3], proPkt.data[2], proPkt.data[1], proPkt.data[0]);
		break;
	case BC_READ_Iin:
		chip->idt_tx_iin = (proPkt.data[0] | proPkt.data[1]<<8);
		p922x_dbg(chip, PR_DEBUG, "chip->idt_tx_iin%d\n", chip->idt_tx_iin);
		break;
	case BC_READ_Vin:
		chip->idt_tx_vin = (proPkt.data[0] | proPkt.data[1]<<8);
		p922x_dbg(chip, PR_DEBUG, "In voltage:%d\n", chip->idt_tx_vin);
		break;
	case BC_SET_Vin:
		p922x_dbg(chip, PR_DEBUG, "set in voltage:%d success\n", chip->idt_tx_vin);
		break;
	case BC_ADAPTER_TYPE:
		chip->idt_adapter_type = proPkt.data[0];
		p922x_dbg(chip, PR_DEBUG, "adapter type:%d\n", chip->idt_adapter_type);
		break;
	case BC_RESET:
		break;
	case BC_READ_I2C:
		break;
	case BC_WRITE_I2C:
		break;
	case BC_VI2C_INIT:
		break;
	case BC_TOGGLE_LOOPMODE:
		p922x_dbg(chip, PR_DEBUG, "LOOPMODE success\n");
		break;
	default:
		p922x_dbg(chip, PR_DEBUG, "error type\n");
	}
	chip->idt_tx_data_recive = DATA_RCV_WAIT_SUCCESS;
#endif
	return 0;
}

static int mode_changed_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_DEBUG, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int ldo_on_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_DEBUG, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int ldo_off_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_INTERRUPT, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int device_auth_failed_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_INTERRUPT, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

#define FOD_VOLTAGE_CHECK_ALARM_PERIOD_NS 60000000000 /*60s*/
static int device_auth_success_handler(struct p922x_dev *chip, u8 rt_stat)
{
	chip->power_good = gpio_get_value(chip->power_good_pin);
	p922x_dbg(chip, PR_INTERRUPT, "rt_stat = 0x%02x\n", rt_stat);

	chip->device_auth_success = true;
	if (chip->power_good) {
		alarm_start_relative(&chip->fod_voltage_check_alarm, ns_to_ktime(0));
	}
	return 0;
}

static int send_pkt_timeout_handler(struct p922x_dev *chip, u8 rt_stat)
{
	ProPkt_Type proPkt;

	p922x_dbg(chip, PR_INTERRUPT, "rt_stat = 0x%02x\n", rt_stat);

	chip->bus.read(chip, REG_BCCMD_ADDR, &proPkt.cmd);
	p922x_get_received_data(chip, proPkt.data);
	p922x_dbg(chip, PR_INTERRUPT, "proPkt.cmd:0x%x,data_list:0x%x,%x,%x,%x\n",
		proPkt.cmd, proPkt.data[0], proPkt.data[1], proPkt.data[2], proPkt.data[3]);
	switch (proPkt.cmd) {
	case BC_NONE:
		break;
	case BC_SET_FREQ:
		p922x_dbg(chip, PR_INTERRUPT, "set idt_tx_freq%d success\n", chip->idt_tx_freq);
		break;
	case BC_GET_FREQ:
		p922x_dbg(chip, PR_INTERRUPT, "get idt_tx_freq%d\n", chip->idt_tx_freq);
		break;
	case BC_READ_FW_VER:
		p922x_dbg(chip, PR_INTERRUPT, "TX Version is: %d.%d.%d.%d\n",
			proPkt.data[3], proPkt.data[2], proPkt.data[1], proPkt.data[0]);
		break;
	case BC_READ_Iin:
		p922x_dbg(chip, PR_INTERRUPT, "chip->idt_tx_iin%d\n", chip->idt_tx_iin);
		break;
	case BC_READ_Vin:
		p922x_dbg(chip, PR_INTERRUPT, "In voltage:%d\n", chip->idt_tx_vin);
		break;
	case BC_SET_Vin:
		p922x_dbg(chip, PR_INTERRUPT, "set in voltage:%d success\n", chip->idt_tx_vin);
		break;
	case BC_ADAPTER_TYPE:
		p922x_get_tx_atapter_type(chip);
		p922x_dbg(chip, PR_INTERRUPT, "send adapter type pkg failed, try again\n");
		break;
	case BC_RESET:
		break;
	case BC_READ_I2C:
		break;
	case BC_WRITE_I2C:
		break;
	case BC_VI2C_INIT:
		break;
	case BC_TOGGLE_LOOPMODE:
		p922x_dbg(chip, PR_INTERRUPT, "LOOPMODE success\n");
		break;
	default:
		p922x_dbg(chip, PR_INTERRUPT, "error type\n");
	}
	chip->idt_tx_data_recive = DATA_RCV_WAIT_TIMEOUT;
	return 0;
}

static int send_pkt_success_handler(struct p922x_dev *chip, u8 rt_stat)
{
	ProPkt_Type proPkt;

	chip->bus.read(chip, REG_BCCMD_ADDR, &proPkt.cmd);
	p922x_get_received_data(chip, proPkt.data);
	p922x_dbg(chip, PR_INTERRUPT, "proPkt.cmd:0x%x,data_list:0x%x,%x,%x,%x\n",
		proPkt.cmd, proPkt.data[0], proPkt.data[1], proPkt.data[2], proPkt.data[3]);
	switch (proPkt.cmd) {
	case BC_NONE:
		break;
	case BC_SET_FREQ:
		break;
	case BC_GET_FREQ:
		chip->idt_tx_freq = (proPkt.data[0] | proPkt.data[1]<<8);
		p922x_dbg(chip, PR_INTERRUPT, "get idt_tx_freq%d\n", chip->idt_tx_freq);
		break;
	case BC_READ_FW_VER:
		p922x_dbg(chip, PR_INTERRUPT, "TX Version is: %d.%d.%d.%d\n",
			proPkt.data[3], proPkt.data[2], proPkt.data[1], proPkt.data[0]);
		break;
	case BC_READ_Iin:
		chip->idt_tx_iin = (proPkt.data[0] | proPkt.data[1]<<8);
		p922x_dbg(chip, PR_INTERRUPT, "Iin:%dmA\n", chip->idt_tx_iin);
		break;
	case BC_READ_Vin:
		chip->idt_tx_vin = (proPkt.data[0] | proPkt.data[1]<<8);
		p922x_dbg(chip, PR_INTERRUPT, "Vin:%dmV\n", chip->idt_tx_vin);
		break;
	case BC_SET_Vin:
		break;
	case BC_ADAPTER_TYPE:
		chip->idt_adapter_type = proPkt.data[0];
		adapter_type = chip->idt_adapter_type;
		p922x_dbg(chip, PR_INTERRUPT, "adapter type:%d\n", chip->idt_adapter_type);
		break;
	case BC_RESET:
		break;
	case BC_READ_I2C:
		break;
	case BC_WRITE_I2C:
		break;
	case BC_VI2C_INIT:
		break;
	case BC_TOGGLE_LOOPMODE:
		break;
	default:
		p922x_dbg(chip, PR_INTERRUPT, "error type\n");
	}
	chip->idt_tx_data_recive = DATA_RCV_WAIT_SUCCESS;

	return 0;
}

static int id_auth_failed_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_INTERRUPT, "rt_stat = 0x%02x\n", rt_stat);
	chip->id_auth_success = false;
	return 0;
}

static int id_auth_success_handler(struct p922x_dev *chip, u8 rt_stat)
{
	chip->power_good = gpio_get_value(chip->power_good_pin);
	p922x_dbg(chip, PR_INTERRUPT, "rt_stat = 0x%02x\n", rt_stat);
	p922x_dbg(chip, PR_INTERRUPT, "power %s, old id auth %s\n",
		chip->power_good ? "good" : "not good",
		chip->id_auth_success ? "success" : "fail");

	chip->id_auth_success = true;
	if (chip->power_good) {
		cancel_delayed_work_sync(&chip->device_key_auth_work);
		schedule_delayed_work(&chip->device_key_auth_work, msecs_to_jiffies(500));
	}
	p922x_dbg(chip, PR_INTERRUPT, "power %s, now id auth %s\n",
		chip->power_good ? "good" : "not good",
		chip->id_auth_success ? "success" : "fail");
	return 0;
}

static int sleep_mode_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_INTERRUPT, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int power_on_handler(struct p922x_dev *chip, u8 rt_stat)
{
	p922x_dbg(chip, PR_INTERRUPT, "rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

struct p922x_irq_info {
	const char		*name;
	int (*p922x_irq)(struct p922x_dev *chip,
							u8 rt_stat);
	int	high;
	int	low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct p922x_irq_info	irq_info[8];
};

static struct irq_handler_info handlers[] = {
	{REG_INTR, 0, 0,
		{
			{
				.name		= "over_curr",
				.p922x_irq	= over_curr_handler,
			},
			{
				.name		= "over_volt",
				.p922x_irq	= over_volt_handler,
			},
			{
				.name		= "over_temp",
				.p922x_irq	= over_temp_handler,
			},
			{
				.name		= "rx_ready",
				.p922x_irq	= rx_ready_handler,
			},
			{
				.name		= "tx_data_rcvd",
				.p922x_irq	= tx_data_rcvd_handler,
			},
			{
				.name		= "mode_changed",
				.p922x_irq	= mode_changed_handler,
			},
			{
				.name		= "ldo_on",
				.p922x_irq	= ldo_on_handler,
			},
			{
				.name		= "ldo_off",
				.p922x_irq	= ldo_off_handler,
			},
		},
	},
	{REG_INTR + 1, 0, 0,
		{
			{
				.name		= "device_auth_failed",
				.p922x_irq	= device_auth_failed_handler,
			},
			{
				.name		= "device_auth_success",
				.p922x_irq	= device_auth_success_handler,
			},
			{
				.name		= "send_pkt_timeout",
				.p922x_irq	= send_pkt_timeout_handler,
			},
			{
				.name		= "send_pkt_success",
				.p922x_irq	= send_pkt_success_handler,
			},
			{
				.name		= "id_auth_failed",
				.p922x_irq	= id_auth_failed_handler,
			},
			{
				.name		= "id_auth_success",
				.p922x_irq	= id_auth_success_handler,
			},
			{
				.name		= "sleep_mode",
				.p922x_irq	= sleep_mode_handler,
			},
			{
				.name		= "power_on",
				.p922x_irq	= power_on_handler,
			},
		},
	},
};

static int p922x_irq_disable(struct p922x_dev *chip)
{
	int rc;
	int value;

	value = 0x0;
	rc = chip->bus.write_buf(chip, REG_INTR_EN, (u8 *)&value, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write %d rc = %d\n",
					REG_INT_CLEAR, rc);
	}
	return rc;
}

static int p922x_irq_enable(struct p922x_dev *chip)
{
	int rc;
	int value;

	value = 0xffff;
	rc = chip->bus.write_buf(chip, REG_INTR_EN, (u8 *)&value, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write %d rc = %d\n",
					REG_INT_CLEAR, rc);
	}
	return rc;
}

static int p922x_irq_clr(struct p922x_dev *chip)
{
	int rc;
	int value;

	value = 0xffff;
	rc = chip->bus.write_buf(chip, REG_INT_CLEAR, (u8 *)&value, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write %d rc = %d\n",
					REG_INT_CLEAR, rc);
		goto clr_int_end;
	}
	rc = idtp9220_masked_write(chip, REG_COMMAND, CLRINT, CLRINT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write %d rc = %d\n",
					REG_COMMAND, rc);
		goto clr_int_end;
	}

	mdelay(5);
clr_int_end:
	return rc;
}

#define WIRELESS_THERMAL_DAEMON_VOTER "WIRELESS_THERMAL_DAEMON_VOTER"
static irqreturn_t p922x_power_good_handler(int irq, void *dev_id)
{
	struct p922x_dev *chip = dev_id;

	chip->power_good = gpio_get_value(chip->power_good_pin);
	p922x_dbg(chip, PR_INTERRUPT, "power %s\n",
		chip->power_good ? "good" : "not good");

	alarm_cancel(&chip->fod_voltage_check_alarm);
	cancel_delayed_work(&chip->fod_voltage_check_work);
	cancel_delayed_work_sync(&chip->device_key_auth_work);
	cancel_delayed_work_sync(&chip->get_tx_adapter_work);
	cancel_delayed_work_sync(&chip->e_trans_show_work);

	chip->id_auth_success = false;
	chip->device_auth_success = false;
	chip->fod_5v_enabled = false;
	chip->fod_9v_enabled = false;
	idt->idt_tx_vin = 0;
	idt->idt_tx_iin = 0;
	idt->idt_rx_vout = 0;
	idt->idt_rx_iout = 0;
	idt->trans_efficiency = 0;
	idt->signal_strength = 0;
	wireless_charging_signal_good = 0;

	if (chip->power_good) {
		if(p922x_set_fod_5v(chip)) {
			p922x_dbg(chip, PR_INTERRUPT, "set fod 5v fail.\n");
		}
		schedule_delayed_work(&chip->e_trans_show_work, msecs_to_jiffies(1000));
	}
	return IRQ_HANDLED;
}

#define IRQ_STATUS_MASK	0x01
static irqreturn_t p922x_stat_handler(int irq, void *dev_id)
{
	struct p922x_dev *chip = dev_id;
	int i, j;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	p922x_dbg(chip, PR_INTERRUPT, "p922x_stat_handler start\n");
	if (rx_fw_otp_write) {
		return IRQ_HANDLED;
	}
	/* pr_info("enter..\n"); */
	mutex_lock(&chip->irq_complete);
	msleep(2);

	p922x_get_signal_strength(chip);

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = idtp9220_read(chip, handlers[i].stat_reg,
					&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}
	}

	p922x_irq_clr(chip);

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		p922x_dbg(chip, PR_INTERRUPT, "[%d]reg=0x%x val=0x%x prev_val=0x%x\n",
				i, handlers[i].stat_reg, handlers[i].val, handlers[i].prev_val);

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << j);
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << j);
			changed = prev_rt_stat ^ rt_stat;
			changed = rt_stat;
			if (changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if (changed && handlers[i].irq_info[j].p922x_irq != NULL) {
				handler_count++;
				p922x_dbg(chip, PR_INTERRUPT, "call %pf, handler_count=%d\n",
					handlers[i].irq_info[j].p922x_irq, handler_count);
				rc = handlers[i].irq_info[j].p922x_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	p922x_dbg(chip, PR_INTERRUPT, "handler count = %d\n", handler_count);

#if 0
	if (handler_count) {
		cancel_delayed_work(&chip->update_heartbeat_work);
		schedule_delayed_work(&chip->update_heartbeat_work, 0);

		cancel_delayed_work(&chip->charger_eoc_work);
		schedule_delayed_work(&chip->charger_eoc_work, 0);
	}
#endif

	mutex_unlock(&chip->irq_complete);
	p922x_dbg(chip, PR_INTERRUPT, "p922x_stat_handler end\n");

	return IRQ_HANDLED;
}


/* set hall gpio input and no pull*/
static int p922x_set_gpio_state(struct p922x_dev *chip)
{
	int error = 0;

	chip->p922x_gpio_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->p922x_gpio_pinctrl)) {
		p922x_dbg(chip, PR_DEBUG, "Can not get p922x_gpio_pinctrl\n");
		error = PTR_ERR(chip->p922x_gpio_pinctrl);
		return error;
	}
	chip->p922x_gpio_state = pinctrl_lookup_state(chip->p922x_gpio_pinctrl, "idt_int_default");
	if (IS_ERR_OR_NULL(chip->p922x_gpio_state)) {
		p922x_dbg(chip, PR_DEBUG, "Can not get p922x_gpio_state\n");
		error = PTR_ERR(chip->p922x_gpio_state);
		return error;
	}

	error = pinctrl_select_state(chip->p922x_gpio_pinctrl, chip->p922x_gpio_state);
	if (error) {
		p922x_dbg(chip, PR_DEBUG, "can not set hall_gpio pins to zte_hall_gpio_active states\n");
	} else {
		p922x_dbg(chip, PR_DEBUG, "set_p922x_gpio_state success.\n");
	}
	return error;
}

/*
static int p922x_en_pin_pinctrl_deinit(struct p922x_dev *chip)
{
	int rc = 0;

	devm_pinctrl_put(chip->p922x_en_pin_pinctrl);

	return rc;
}
*/
static int p922x_en_pin_pinctrl_init(struct p922x_dev *chip)
{
	int rc = 0;

	chip->p922x_en_pin_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->p922x_en_pin_pinctrl)) {
		rc = PTR_ERR(chip->p922x_en_pin_pinctrl);
		pr_err("failed to get pinctrl, rc=%d\n", rc);
		return rc;
	}

	chip->p922x_en_pin_active = pinctrl_lookup_state(chip->p922x_en_pin_pinctrl, "idt_en_pin_active");
	if (IS_ERR_OR_NULL(chip->p922x_en_pin_active)) {
		rc = PTR_ERR(chip->p922x_en_pin_active);
		pr_err("failed to get pinctrl active state, rc=%d\n", rc);
		return rc;
	}

	chip->p922x_en_pin_suspend = pinctrl_lookup_state(chip->p922x_en_pin_pinctrl, "idt_en_pin_suspend");
	if (IS_ERR_OR_NULL(chip->p922x_en_pin_suspend)) {
		rc = PTR_ERR(chip->p922x_en_pin_suspend);
		pr_err("failed to get pinctrl suspend state, rc=%d\n", rc);
		return rc;
	}

	p922x_dbg(chip, PR_DEBUG, "done.\n");
	return rc;
}

static int p922x_set_en_pin_pinctrl_state(struct p922x_dev *chip, bool enable)
{
	int rc = 0;
	struct pinctrl_state *state;

	p922x_dbg(chip, PR_INTERRUPT, "enable = %d\n", enable);

	if (enable) {
		state = chip->p922x_en_pin_active;
	} else {
		state = chip->p922x_en_pin_suspend;
	}

	rc = pinctrl_select_state(chip->p922x_en_pin_pinctrl, state);
	if (rc) {
		pr_err("failed to set pin state, rc=%d\n", rc);
	}

	return rc;
}

static int idtp922x_set_enable(struct p922x_dev *chip, bool enable)
{
	int rc = 0;

	rc = p922x_set_en_pin_pinctrl_state(chip, enable);
/*
	if (!gpio_is_valid(chip->en_pin)) {
		pr_err("en pin is invalid.\n");
	} else {
		if (gpio_request(chip->en_pin, "idt-en-pin")) {
			pr_err("unable to request idt en pin [%d]\n", chip->en_pin);
		} else {
			if (gpio_direction_output(chip->en_pin, !enable)) {
				pr_err("cannot set direction for idt en pin [%d]\n", chip->en_pin);
			} else {
				p922x_dbg(chip, PR_INTERRUPT, "set enable success\n");
			}
		}
	}
*/
	return rc;
}

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 8; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct p922x_dev *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static enum alarmtimer_restart fod_voltage_check_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct p922x_dev *chip = container_of(alarm, struct p922x_dev,
											fod_voltage_check_alarm);

	p922x_dbg(chip, PR_DEBUG, "fod_voltage_check_alarm_cb\n");

	alarm_forward_now(&chip->fod_voltage_check_alarm, ns_to_ktime(FOD_VOLTAGE_CHECK_ALARM_PERIOD_NS));
	schedule_delayed_work(&chip->fod_voltage_check_work, msecs_to_jiffies(0));

	return ALARMTIMER_RESTART;
}

static void fod_voltage_check_work_cb(struct work_struct *work)
{
	struct p922x_dev *chip = container_of(work, struct p922x_dev,
						fod_voltage_check_work.work);
	int vout = 0;
	int i = 0, timeout = 100, delay_ms = 20;

	schedule_delayed_work(&chip->get_tx_adapter_work, msecs_to_jiffies(0));

	for (i = 0; i < timeout; i++) {
		if (chip->idt_adapter_type != ADAPTER_UNKNOWN) {
			p922x_dbg(chip, PR_INTERRUPT, "i : %d\n", i);
			break;
		}
		msleep(delay_ms);
	}

	p922x_dbg(chip, PR_INTERRUPT, "adapter type:%d, thermal state = %d\n",
		chip->idt_adapter_type, chip->thermal_state);

	if (chip->idt_adapter_type == ADAPTER_QC20 || chip->idt_adapter_type == ADAPTER_QC30) {
		if (chip->thermal_state == THERMAL_GOOD_STATE || lab_test_mode) {
			p922x_set_fast_charging_voltage(chip, QC_MAX_VOLTAGE);
		} else if (chip->thermal_state == THERMAL_WARM_STATE) {
			p922x_set_fast_charging_voltage(chip, QC_MIN_VOLTAGE);
		}
		msleep(1000);
		vout = p922x_get_rx_vout();
		if (vout > 8000 && vout < 9500) {
			if (p922x_set_fod_9v(chip)) {
				p922x_dbg(chip, PR_INTERRUPT, "set fod 9v fail.\n");
			}
		} else if (vout > 4000 && vout < 5500) {
			if (p922x_set_fod_5v(chip)) {
				p922x_dbg(chip, PR_INTERRUPT, "set fod 5v fail.\n");
			}
		}
		p922x_dbg(chip, PR_INTERRUPT, "power %s, rx vout %dmV\n",
			gpio_get_value(chip->power_good_pin) ? "good" : "not good", vout);
	} else {
		alarm_cancel(&chip->fod_voltage_check_alarm);
	}

	p922x_dbg(chip, PR_INTERRUPT, "fod_voltage_check_work\n");
}

enum {
	CUR_LEGACY = 0,
	VOL_LEGACY,
	EFF_LEGACY,
};

#define CURRENT_MIN 0
#define CURRENT_MAX 2500
#define VOLTAGE_MIN 0
#define VOLTAGE_MAX 9500
#define EFFICIENCY_MIN 0
#define EFFICIENCY_MAX 100

static int in_range(int val, int min, int max)
{
	int ret = 0;

	if (val >= min && val <= max) {
		ret = val;
	} else if (val < min) {
		ret = min;
	} else if (val > max) {
		ret = max;
	}

	return ret;
}

static int get_legal_val(int val, int type)
{
	int ret = 0;

	if (type == CUR_LEGACY) {
		ret = in_range(val, CURRENT_MIN, CURRENT_MAX);
	} else if (type == VOL_LEGACY) {
		ret = in_range(val, VOLTAGE_MIN, VOLTAGE_MAX);
	} else if (type == EFF_LEGACY) {
		ret = in_range(val, EFFICIENCY_MIN, EFFICIENCY_MAX);
	}

	return ret;
}

static void tx_adapter_work(struct work_struct *work)
{
	struct p922x_dev *chip = container_of(work, struct p922x_dev,
						get_tx_adapter_work.work);

	p922x_dbg(chip, PR_DEBUG, "fod_voltage_check_work\n");
	p922x_get_tx_atapter_type(chip);
}

static void e_trans_work(struct work_struct *work)
{
	struct p922x_dev *chip = container_of(work, struct p922x_dev,
						e_trans_show_work.work);

	if (chip->power_good && chip->id_auth_success && chip->device_auth_success) {
		idt->idt_tx_vin = get_legal_val(p922x_get_tx_vin(chip), VOL_LEGACY);
		idt->idt_tx_iin = get_legal_val(p922x_get_tx_iin(), CUR_LEGACY);
		idt->idt_rx_vout = get_legal_val(p922x_get_rx_vout(), VOL_LEGACY);
		idt->idt_rx_iout = get_legal_val(p922x_get_rx_iout(chip), CUR_LEGACY);
		if (idt->idt_tx_vin * idt->idt_tx_iin != 0) {
			chip->trans_efficiency =
				100 * (idt->idt_rx_vout * idt->idt_rx_iout) / (idt->idt_tx_vin * idt->idt_tx_iin);
			chip->trans_efficiency = get_legal_val(chip->trans_efficiency, EFF_LEGACY);
		}
		p922x_dbg(chip, PR_DEBUG, "power%s, ss:%d, efficiency:%d, RX:%dmV, %dmA, TX:%dmV, %dmA\n",
			chip->power_good ? "on" : "off", idt->signal_strength,
			idt->trans_efficiency, idt->idt_rx_vout, idt->idt_rx_iout, idt->idt_tx_vin, idt->idt_tx_iin);
	} else if (chip->power_good) {
		idt->idt_tx_vin = 0;
		idt->idt_tx_iin = 0;
		idt->idt_rx_vout = get_legal_val(p922x_get_rx_vout(), VOL_LEGACY);
		idt->idt_rx_iout = get_legal_val(p922x_get_rx_iout(chip), CUR_LEGACY);
		idt->trans_efficiency = 0;
		p922x_dbg(chip, PR_DEBUG, "power%s or id auth %s or device auth %s, ss:%d, RX:%dmV, %dmA\n",
			chip->power_good ? "on" : "off", chip->id_auth_success ? "success" : "fail", chip->device_auth_success ? "success" : "fail",
			idt->signal_strength, idt->idt_rx_vout, idt->idt_rx_iout);
	} else if (!chip->power_good) {
		idt->idt_tx_vin = 0;
		idt->idt_tx_iin = 0;
		idt->idt_rx_vout = 0;
		idt->idt_rx_iout = 0;
		idt->trans_efficiency = 0;
	}

	schedule_delayed_work(&chip->e_trans_show_work, msecs_to_jiffies(2000));
}

static void key_auth_work(struct work_struct *work)
{
	struct p922x_dev *chip = container_of(work, struct p922x_dev,
						device_key_auth_work.work);

	if (chip->power_good && chip->id_auth_success) {
		p922x_dbg(chip, PR_INTERRUPT, "SENDDEVICEAUTH CMD\n");
		idt->bus.write(idt, REG_COMMAND, SENDDEVICEAUTH);
	}
}

#define EMODE_SHOW_SIZE 160
static int tx_rx_trans_efficiency = 0;
static int p922x_tx_rx_trans_efficiency_node(char *val, struct kernel_param *kp)
{
	static bool rx_fw_gotten = false;
	static u8 ver_data[4];

	p922x_dbg(idt, PR_DEBUG, "efficiency:%d\n",
		idt->trans_efficiency);

	if (!rx_fw_gotten) {
		idt->bus.read_buf(idt, REG_CHIP_REV, ver_data, 4);
		if (ver_data[3] == 0x0a) {
			rx_fw_gotten = true;
		}
	}

	return snprintf(val, EMODE_SHOW_SIZE,
		"ID:%2d, Coil:%8s\tFW:%02x.%02x.%02x.%02x,SS:%2d,E:%2d,RX:%4dmV,%4dmA,TX:%4dmV,%4dmA",
		idt->board_id, idt->coil_vendor_name, ver_data[3], ver_data[2], ver_data[1], ver_data[0],
		idt->signal_strength, idt->trans_efficiency,
		idt->idt_rx_vout, idt->idt_rx_iout, idt->idt_tx_vin, idt->idt_tx_iin);
}

module_param_call(tx_rx_trans_efficiency, NULL, p922x_tx_rx_trans_efficiency_node,
					&tx_rx_trans_efficiency, 0644);

static void __iomem *vendor_imem_info_addr;
static int vendor_imem_info_parse_dt(const char *compatible)
{
	struct device_node *np;
	int val = -EINVAL;

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np) {
		pr_err("unable to find DT imem %s node\n", compatible);
	} else {
		vendor_imem_info_addr = of_iomap(np, 0);
		if (!vendor_imem_info_addr) {
			pr_err("unable to map imem %s offset\n", compatible);
		} else {
			val = __raw_readl(vendor_imem_info_addr);
			pr_info("%s: %d\n", compatible, val);
		}
	}
	return val;
}

static int read_board_id(void)
{
	int id = 0;

	id = vendor_imem_info_parse_dt("qcom,msm-imem-board-id");

	return id;
}

void copy_array(const char *p_src, char *p_dest, int size)
{
	int num = 0;

	for (num = 0; num <= size - 1; num++) {
		p_dest[num] = p_src[num];
	}
}

static enum power_supply_property idtp922x_properties[] = {
	POWER_SUPPLY_PROP_PIN_ENABLED,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
};

static int idtp922x_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct p922x_dev *chip = power_supply_get_drvdata(psy);

	if (chip) {
		switch (psp) {
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
			val->intval = p922x_get_rx_output_current_max(chip);
		case POWER_SUPPLY_PROP_PIN_ENABLED:
			break;
		case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
			if (chip->power_good) {
				if (chip->fod_5v_enabled)
					val->intval = IDT_REGULATE_5V_RX_VOUT;
				else if (chip->fod_9v_enabled)
					val->intval = IDT_REGULATE_9V_RX_VOUT;
			} else {
				val->intval = IDT_REGULATE_5V_RX_VOUT;
			}
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static int idtp922x_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct p922x_dev *chip = power_supply_get_drvdata(psy);

	if (chip) {
		switch (psp) {
		case POWER_SUPPLY_PROP_PIN_ENABLED:
			idtp922x_set_enable(chip, val->intval);
			break;
		case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
			if (val->intval) {
				chip->thermal_state = THERMAL_GOOD_STATE;
			} else {
				chip->thermal_state = THERMAL_WARM_STATE;
			}
			p922x_dbg(chip, PR_INTERRUPT, "the thermal_state is %d.\n", chip->thermal_state);
			cancel_delayed_work(&chip->fod_voltage_check_work);
			schedule_delayed_work(&chip->fod_voltage_check_work, msecs_to_jiffies(0));
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static int idtp922x_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PIN_ENABLED:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}

static const struct power_supply_desc idtp922x_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.get_property = idtp922x_get_property,
	.set_property = idtp922x_set_property,
	.properties = idtp922x_properties,
	.property_is_writeable = idtp922x_property_is_writeable,
	.num_properties = ARRAY_SIZE(idtp922x_properties),
};

static int p922x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct p922x_dev *chip;
	int ret = 0;
	struct power_supply_config idtp922x_cfg = {};

	pr_info("IDTP922x probe\n");

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->board_id = read_board_id();
	pr_info("board_id = %d\n", chip->board_id);
	if (chip->board_id != BOARD_ID_NPI_VERSION_A
		&& chip->board_id != BOARD_ID_NPI_VERSION_B) {
		pr_info("IDTP922x update fod parameters for old coil.\n");
		copy_array(idtp9220_rx_fod_5v_Amphenol_Coil, idtp9220_rx_fod_5v, FOD_REGSTERS_NUM);
		copy_array(idtp9220_rx_fod_9v_Amphenol_Coil, idtp9220_rx_fod_9v, FOD_REGSTERS_NUM);
		strncpy(chip->coil_vendor_name, "Amphenol", strlen("Amphenol"));
	} else {
		pr_info("IDTP922x update fod parameters for new coil.\n");
		copy_array(idtp9220_rx_fod_5v_Amotech_Coil, idtp9220_rx_fod_5v, FOD_REGSTERS_NUM);
		copy_array(idtp9220_rx_fod_9v_Amotech_Coil, idtp9220_rx_fod_9v, FOD_REGSTERS_NUM);
		strncpy(chip->coil_vendor_name, "Amotech", strlen("Amotech"));
	}
	pr_info("coil vendor name is %s\n", chip->coil_vendor_name);

	idt = chip;
	chip->dev = &client->dev;
	chip->regmap = devm_regmap_init_i2c(client, &p922x_regmap_config);
	if (!chip->regmap) {
		pr_err("parent regmap is missing\n");
		return -EINVAL;
	}
	i2c_set_clientdata(client, chip);
	chip->client = client;
	chip->dev = &client->dev;

	chip->bus.read = idtp9220_read;
	chip->bus.write = idtp9220_write;
	chip->bus.read_buf = idtp9220_read_buffer;
	chip->bus.write_buf = idtp9220_write_buffer;
	chip->name = "IDT";
	device_init_wakeup(chip->dev, true);

	mutex_init(&chip->write_lock);
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->send_pkg_lock);

	INIT_DELAYED_WORK(&chip->fod_voltage_check_work, fod_voltage_check_work_cb);
	INIT_DELAYED_WORK(&chip->get_tx_adapter_work, tx_adapter_work);
	INIT_DELAYED_WORK(&chip->e_trans_show_work, e_trans_work);
	INIT_DELAYED_WORK(&chip->device_key_auth_work, key_auth_work);
	alarm_init(&chip->fod_voltage_check_alarm, ALARM_BOOTTIME,
			fod_voltage_check_alarm_cb);

	chip->debug_root = debugfs_create_dir("p922x", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");
	if (chip->debug_root) {
		struct dentry *ent;
#if 0
		ent = debugfs_create_file("registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cnfg debug file\n");
		ent = debugfs_create_u8("address", S_IRUSR | S_IWUSR,
					chip->debug_root,
					&chip->reg_addr);
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create address debug file\n");
		}

		ent = debugfs_create_file("data",  S_IRUSR | S_IWUSR,
					chip->debug_root, chip,
					&ti2419x_debug_data_fops);
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create data debug file\n");
		}

		ent = debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_writes));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file\n");

		ent = debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_reads));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file\n");
#endif
		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create count debug file\n");
	}

	chip->fod_5v_enabled = false;
	chip->fod_9v_enabled = false;

	ret = of_property_read_u32(chip->dev->of_node, "qcom,idt-ss-good-thr", &chip->signal_strength_good_threshold);
	if (ret < 0) {
		pr_err("get signal_strength_good_threshold failed\n");
		chip->signal_strength_good_threshold = WIRELESS_CHARGING_SIGNAL_GOOD_THRESHOLD_DEFAULT;
	}

	ret = p922x_en_pin_pinctrl_init(chip);
	if (ret < 0) {
		p922x_dbg(chip, PR_DEBUG, "p922x_en_pin_pinctrl_init failed.\n");
		goto release;
	} else {
		ret = p922x_set_en_pin_pinctrl_state(chip, true);
		if (ret < 0) {
			p922x_dbg(chip, PR_DEBUG, "p922x_set_en_pin_pinctrl_state failed.\n");
			goto release;
		} else {
			p922x_dbg(chip, PR_DEBUG, "p922x_set_en_pin_pinctrl_state success.\n");
		}
	}

	chip->int_pin = of_get_named_gpio(chip->dev->of_node, "qcom,idt-int-pin", 0);
	if (!gpio_is_valid(chip->int_pin)) {
		pr_err("int pin is invalid.\n");
		ret = -EINVAL;
		goto release;
	}

	chip->power_good_pin = of_get_named_gpio(chip->dev->of_node, "qcom,idt-power-good", 0);
	if (!gpio_is_valid(chip->power_good_pin)) {
		pr_err("power good pin is invalid.\n");
		ret = -EINVAL;
		goto release;
	}

/*
	chip->en_pin = of_get_named_gpio(chip->dev->of_node, "qcom,idt-en-pin", 0);
	if (!gpio_is_valid(chip->en_pin)) {
		pr_err("en pin is invalid.\n");
	} else {
		if (gpio_request(chip->en_pin, "idt-en-pin")) {
			pr_err("unable to request idt en pin [%d]\n", chip->en_pin);
		} else {
			if (gpio_direction_output(chip->en_pin, 0)) {
				pr_err("cannot set direction for idt en pin [%d]\n", chip->en_pin);
			} else {
				p922x_dbg(chip, PR_DEBUG, "set direction for idt en pin success.\n");
			}
		}
	}
*/

	ret = p922x_set_gpio_state(chip);
	if (ret < 0) {
		p922x_dbg(chip, PR_DEBUG, "p922x_set_gpio_state failed.\n");
		goto release;
	}
	gpio_direction_input(chip->power_good_pin);
	if (gpio_get_value(chip->power_good_pin)) {
		p922x_dbg(chip, PR_DEBUG, "p922x_irq_disable.\n");
		p922x_irq_disable(chip);
	}
	ret = request_threaded_irq(gpio_to_irq(chip->int_pin),
			NULL, p922x_stat_handler,
			(IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT),
			"p922x-handler", chip);
	if (ret < 0) {
		p922x_dbg(chip, PR_DEBUG, "request irq failed\n");
		gpio_free(chip->int_pin);
		goto release;
	} else {
		p922x_dbg(chip, PR_DEBUG, "request irq success\n");
	}

	ret = request_threaded_irq(gpio_to_irq(chip->power_good_pin),
			NULL, p922x_power_good_handler,
			(IRQ_TYPE_EDGE_BOTH | IRQF_ONESHOT),
			"p922x-power_good_handler", chip);
	if (ret < 0) {
		p922x_dbg(chip, PR_DEBUG, "request power_good irq failed\n");
		gpio_free(chip->power_good_pin);
		goto release;
	} else {
		p922x_dbg(chip, PR_DEBUG, "request power_good irq success\n");
	}

	if (gpio_get_value(chip->power_good_pin)) {
		p922x_dbg(chip, PR_DEBUG, "p922x_stat_handler.\n");
		p922x_stat_handler(gpio_to_irq(chip->int_pin), chip);
		p922x_irq_enable(chip);
		alarm_forward_now(&chip->fod_voltage_check_alarm, ns_to_ktime(5000000000));
	}

	idtp922x_cfg.drv_data = chip;
	chip->idtp922x_psy = power_supply_register(chip->dev,
			&idtp922x_psy_desc,
			&idtp922x_cfg);

	chip->thermal_state = THERMAL_GOOD_STATE;

	p922x_dbg(chip, PR_DEBUG, "IDTP922x probed successfully chip is power %s\n",
		(gpio_get_value(chip->power_good_pin)) ? "ON" : "OFF");
	return 0;

release:
	p922x_dbg(chip, PR_DEBUG, "IDTP922x init is failed");
	i2c_set_clientdata(client, NULL);

	return ret;
}

static struct i2c_driver p922x_driver = {
	.driver   = {
		.name			= "idt_wireless_power",
		.owner			= THIS_MODULE,
		.of_match_table = match_table,
	},
	.probe	  = p922x_probe,
	.remove   = p922x_remove,
	.id_table = p922x_dev_id,
};
module_i2c_driver(p922x_driver);

MODULE_AUTHOR("sun.shaojie@zte.com.cn");
MODULE_DESCRIPTION("P922x Wireless Power Charger Monitor driver");
MODULE_LICENSE("GPL v2");
