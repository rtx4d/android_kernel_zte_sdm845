/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>

#include "cam_actuator_dev.h"

#include "cam_sensor_io.h"
#include "cam_sensor_i2c.h"
#include "zte_camera_actuator_util.h"


#define CONFIG_ZTE_ACTUATOR_UTIL_DEBUG

#undef CDBG
#ifdef CONFIG_ZTE_CAMERA_UTIL_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


typedef struct {
	int gyro_cal_success;
	uint16_t reg1_cal_val;
	uint16_t reg2_cal_val;
} msm_cal_info_t;

typedef struct {
	struct cam_actuator_ctrl_t *s_ctrl;
	enum camera_sensor_i2c_type msm_actuator_reg_data_type;
	enum camera_sensor_i2c_type msm_actuator_reg_addr_type;
	uint64_t address;
	msm_cal_info_t cal_info;
} msm_actuator_debug_info_t;


static int actuator_debugfs_datatype_s(void *data, u64 val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;

	if (val < CAMERA_SENSOR_I2C_TYPE_MAX
		&& val >= CAMERA_SENSOR_I2C_TYPE_BYTE)
		ptr->msm_actuator_reg_data_type = val;

	CAM_DBG(CAM_ACTUATOR, "%s:%d: msm_actuator_reg_data_type = %d",
		__func__, __LINE__, ptr->msm_actuator_reg_data_type);

	return 0;
}

static int actuator_debugfs_datatype_g(void *data, u64 *val)

{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;

	*val = ptr->msm_actuator_reg_data_type;
	CDBG(CAM_ACTUATOR, "%s:%d: msm_actuator_reg_data_type = %d",
		__func__, __LINE__, ptr->msm_actuator_reg_data_type);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(actuator_debugfs_datatype, actuator_debugfs_datatype_g,
			actuator_debugfs_datatype_s, "%llx\n");

static int actuator_debugfs_addrtype_s(void *data, u64 val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;

	if (val < CAMERA_SENSOR_I2C_TYPE_MAX
		&& val >= CAMERA_SENSOR_I2C_TYPE_BYTE)
		ptr->msm_actuator_reg_addr_type = val;

	CAM_DBG(CAM_SENSOR, "%s:%d: msm_sensor_reg_data_type = %d",
		__func__, __LINE__, ptr->msm_actuator_reg_data_type);

	return 0;
}

static int actuator_debugfs_addrtype_g(void *data, u64 *val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;

	*val = ptr->msm_actuator_reg_addr_type;

	CAM_DBG(CAM_SENSOR, "%s:%d: msm_sensor_reg_addr_type = %d",
		__func__, __LINE__, ptr->msm_actuator_reg_addr_type);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(actuator_debugfs_addrtype, actuator_debugfs_addrtype_g,
			actuator_debugfs_addrtype_s, "%llx\n");

static int actuator_debugfs_setaddr(void *data, u64 val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;

	ptr->address = val;
	CDBG(CAM_ACTUATOR, "%s:%d: address = 0x%llx",
		__func__, __LINE__, ptr->address);

	return 0;
}

static int actuator_debugfs_getaddr(void *data, u64 *val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;

	*val = ptr->address;
	CDBG(CAM_ACTUATOR, "%s:%d: address = 0x%llx",
		__func__, __LINE__, ptr->address);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(actuator_debugfs_address, actuator_debugfs_getaddr,

			actuator_debugfs_setaddr, "%llx\n");


static int actuator_debugfs_setvalue(void *data, u64 val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;
	int32_t rc = 0;

	CDBG("%s:%d: address = 0x%llx  value = 0x%llx",
		__func__, __LINE__, ptr->address, val);

	rc = cam_cci_i2c_write(&(ptr->s_ctrl->io_master_info),
			ptr->address, val,
			ptr->msm_actuator_reg_addr_type,
			ptr->msm_actuator_reg_data_type);

	if (rc < 0) {
		pr_err("%s:%d: i2c write %llx failed", __func__, __LINE__, val);
		return rc;
	}

	return 0;
}

static int actuator_debugfs_getvalue(void *data, u64 *val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;
	int32_t rc = 0;
	uint32_t temp;

	rc = camera_io_dev_read(
			&(ptr->s_ctrl->io_master_info),
			ptr->address, &temp,
			ptr->msm_actuator_reg_addr_type,
			ptr->msm_actuator_reg_data_type);

	if (rc < 0) {
		pr_err("%s:%d: i2c read %x failed", __func__, __LINE__, temp);
		return rc;
	}

	*val = temp;

	CDBG("%s:%d: address = 0x%llx  value = 0x%x", __func__, __LINE__,
		ptr->address, temp);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(actuator_debugfs_value, actuator_debugfs_getvalue,
			actuator_debugfs_setvalue, "%llx\n");

#define GYRO_CAL_TIMES 16
#define GYRO_CAL_THRES_MIN  -1444
#define GYRO_CAL_THRES_MAX   1444

#if 0
static int actuator_debugfs_gyrocal_s(void *data, u64 val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;
	enum camera_sensor_i2c_type reg_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	enum camera_sensor_i2c_type reg_addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	int32_t rc = 0;
	int i;
	uint32_t temp = 0;
	uint32_t reg1_addr = 0x8455;
	uint32_t reg2_addr = 0x8456;
	int16_t reg1_val[GYRO_CAL_TIMES];
	int32_t reg1_cal_sum = 0;
	int16_t reg2_val[GYRO_CAL_TIMES];
	int32_t reg2_cal_sum = 0;

	ptr->cal_info.gyro_cal_success = 0;
	for (i = 0; i < GYRO_CAL_TIMES; i++) {
		rc = camera_io_dev_read(&(ptr->s_ctrl->io_master_info),
			reg1_addr, &(temp), reg_addr_type, reg_data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c read addr[%d] = 0x%x  failed",
				__func__, __LINE__, i, reg1_addr);
			break;
		}
		if (temp & 0x8000)
			reg1_val[i] = (int16_t)(temp | 0xffff0000);
		else
			reg1_val[i] =  (int16_t)temp;
		if (reg1_val[i] < GYRO_CAL_THRES_MIN || reg1_val[i] > GYRO_CAL_THRES_MAX) {
			pr_err("%s:%d: i2c read 0x%x[%d] = 0x%x  %d outside error",
				__func__, __LINE__, reg1_addr, i, temp, reg1_val[i]);
			break;
		}
		reg1_cal_sum += reg1_val[i];
		pr_info("%s:%d: i2c read 0x%x[%d] = 0x%x  %d",
			__func__, __LINE__, reg1_addr, i, temp, reg1_val[i]);

		rc = camera_io_dev_read(&(ptr->s_ctrl->io_master_info),
			reg2_addr, &(temp), reg_addr_type, reg_data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c read addr[%d] = 0x%x  failed",
				__func__, __LINE__, i, reg2_val[i]);
			break;
		}
		if (temp & 0x8000)
			reg2_val[i] = (int16_t)(temp | 0xffff0000);
		else
			reg2_val[i] =  (int16_t)temp;
		if (reg2_val[i] < GYRO_CAL_THRES_MIN || reg2_val[i] > GYRO_CAL_THRES_MAX) {
			pr_err("%s:%d: i2c read 0x%x[%d] = 0x%x  %d outside error",
				__func__, __LINE__, reg2_addr, i, temp, reg2_val[i]);
			break;
		}
		reg2_cal_sum += reg2_val[i];
		pr_info("%s:%d: i2c read 0x%x[%d] = 0x%x  %d",
			__func__, __LINE__, reg2_addr, i, temp, reg2_val[i]);

		if (i == (GYRO_CAL_TIMES - 1))
			ptr->cal_info.gyro_cal_success = 1;
	}
	if (ptr->cal_info.gyro_cal_success == 1) {
		for (i = 0; i < GYRO_CAL_TIMES - 1; i++) {
			if (reg1_val[i] != reg1_val[GYRO_CAL_TIMES - 1])
				break;
			if (i == (GYRO_CAL_TIMES - 2)) {
				ptr->cal_info.gyro_cal_success = 0;
				pr_err("%s:%d: 0x%x reg1 all value same, error",
					__func__, __LINE__, reg1_val[i]);
			}
		}
		for (i = 0; i < GYRO_CAL_TIMES - 1; i++) {
			if (reg2_val[i] != reg2_val[GYRO_CAL_TIMES - 1])
				break;
			if (i == (GYRO_CAL_TIMES - 2)) {
				ptr->cal_info.gyro_cal_success = 0;
				pr_err("%s:%d: 0x%x reg1 all value same, error",
					__func__, __LINE__, reg1_val[i]);
			}
		}
	}

	ptr->cal_info.reg1_cal_val  = (uint16_t)(reg1_cal_sum / GYRO_CAL_TIMES);
	ptr->cal_info.reg2_cal_val  = (uint16_t)(reg2_cal_sum / GYRO_CAL_TIMES);
	pr_info("%s:%d: gyro_cal_success = %d reg1_cal_val = 0x%x  reg2_cal_val = 0x%x",
		__func__, __LINE__, ptr->cal_info.gyro_cal_success, ptr->cal_info.reg1_cal_val,
		ptr->cal_info.reg2_cal_val);

	return 0;
}
#endif

static int actuator_debugfs_gyrocal_g(void *data, u64 *val)
{
	msm_actuator_debug_info_t *ptr = (msm_actuator_debug_info_t *) data;

	enum camera_sensor_i2c_type reg_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	enum camera_sensor_i2c_type reg_addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	int32_t rc = 0;
	int i;
	uint32_t temp = 0;
	uint32_t reg1_addr = 0x8455;
	uint32_t reg2_addr = 0x8456;
	int16_t reg1_val[GYRO_CAL_TIMES];
	int32_t reg1_cal_sum = 0;
	int16_t reg2_val[GYRO_CAL_TIMES];
	int32_t reg2_cal_sum = 0;

	ptr->cal_info.gyro_cal_success = 0;
	for (i = 0; i < GYRO_CAL_TIMES; i++) {
		rc = camera_io_dev_read(&(ptr->s_ctrl->io_master_info),
			reg1_addr, &(temp), reg_addr_type, reg_data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c read addr[%d] = 0x%x  failed",
				__func__, __LINE__, i, reg1_addr);
			break;
		}
		if (temp & 0x8000)
			reg1_val[i] = (int16_t)(temp | 0xffff0000);
		else
			reg1_val[i] =  (int16_t)temp;
		if (reg1_val[i] < GYRO_CAL_THRES_MIN || reg1_val[i] > GYRO_CAL_THRES_MAX) {
			pr_err("%s:%d: i2c read 0x%x[%d] = 0x%x  %d outside error",
				__func__, __LINE__, reg1_addr, i, temp, reg1_val[i]);
			break;
		}
		reg1_cal_sum += reg1_val[i];
		pr_info("%s:%d: i2c read 0x%x[%d] = 0x%x  %d",
			__func__, __LINE__, reg1_addr, i, temp, reg1_val[i]);

		rc = camera_io_dev_read(&(ptr->s_ctrl->io_master_info),
			reg2_addr, &(temp), reg_addr_type, reg_data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c read addr[%d] = 0x%x  failed",
				__func__, __LINE__, i, reg2_val[i]);
			break;
		}
		if (temp & 0x8000)
			reg2_val[i] = (int16_t)(temp | 0xffff0000);
		else
			reg2_val[i] =  (int16_t)temp;
		if (reg2_val[i] < GYRO_CAL_THRES_MIN || reg2_val[i] > GYRO_CAL_THRES_MAX) {
			pr_err("%s:%d: i2c read 0x%x[%d] = 0x%x  %d outside error",
				__func__, __LINE__, reg2_addr, i, temp, reg2_val[i]);
			break;
		}
		reg2_cal_sum += reg2_val[i];
		pr_info("%s:%d: i2c read 0x%x[%d] = 0x%x  %d",
			__func__, __LINE__, reg2_addr, i, temp, reg2_val[i]);

		if (i == (GYRO_CAL_TIMES - 1))
			ptr->cal_info.gyro_cal_success = 1;
	}
	if (ptr->cal_info.gyro_cal_success == 1) {
		for (i = 0; i < GYRO_CAL_TIMES - 1; i++) {
			if (reg1_val[i] != reg1_val[GYRO_CAL_TIMES - 1])
				break;
			if (i == (GYRO_CAL_TIMES - 2)) {
				ptr->cal_info.gyro_cal_success = 0;
				pr_err("%s:%d: 0x%x reg1 all value same, error",
					__func__, __LINE__, reg1_val[i]);
			}
		}
		for (i = 0; i < GYRO_CAL_TIMES - 1; i++) {
			if (reg2_val[i] != reg2_val[GYRO_CAL_TIMES - 1])
				break;
			if (i == (GYRO_CAL_TIMES - 2)) {
				ptr->cal_info.gyro_cal_success = 0;
				pr_err("%s:%d: 0x%x reg1 all value same, error",
					__func__, __LINE__, reg1_val[i]);
			}
		}
	}

	ptr->cal_info.reg1_cal_val  = (uint16_t)(reg1_cal_sum / GYRO_CAL_TIMES);
	ptr->cal_info.reg2_cal_val  = (uint16_t)(reg2_cal_sum / GYRO_CAL_TIMES);
	pr_info("%s:%d: gyro_cal_success = %d reg1_cal_val = 0x%x  reg2_cal_val = 0x%x",
		__func__, __LINE__, ptr->cal_info.gyro_cal_success, ptr->cal_info.reg1_cal_val,
		ptr->cal_info.reg2_cal_val);

	if (ptr->cal_info.gyro_cal_success == 1) {
		*val = ptr->cal_info.reg1_cal_val;
		*val = (*val << 16);
		*val |= ptr->cal_info.reg2_cal_val;
		*val |= 0x100000000;
	} else {
		*val = 0;
	}
	pr_info("%s:%d: gyro_cal_success = %d  0x%llx",
		__func__, __LINE__, ptr->cal_info.gyro_cal_success, *val);
	return 0;
}

#if 0
DEFINE_SIMPLE_ATTRIBUTE(actuator_debugfs_gyrocal, actuator_debugfs_gyrocal_g,
			actuator_debugfs_gyrocal_s, "%llx\n");
#else
DEFINE_SIMPLE_ATTRIBUTE(actuator_debugfs_gyrocal, actuator_debugfs_gyrocal_g,
			NULL, "%llx\n");
#endif


struct dentry *actuator_debugfs_base = NULL;
int actuator_probe = 0;
#define BUF_SIZE 15

void msm_actuator_creat_debugfs(void)
{
	if (!actuator_debugfs_base) {
		actuator_debugfs_base = debugfs_create_dir("zte_actuator", NULL);
		if (!actuator_debugfs_base) {
			pr_err(": zte_actuator dir creat fail");
		}
	}
}

int msm_actuator_enable_debugfs(struct cam_actuator_ctrl_t *s_ctrl)
{
	struct dentry  *actuator_dir;
	msm_actuator_debug_info_t *debug_ptr = NULL;
	char buf[BUF_SIZE];

	CDBG("%s:%d:  E", __func__, __LINE__);
	if (!actuator_debugfs_base) {
		actuator_debugfs_base = debugfs_create_dir("zte_actuator", NULL);
		if (!actuator_debugfs_base) {
			pr_err("%s:%d: exit", __func__, __LINE__);
			return -ENOMEM;
		}
	}

	if (actuator_probe & (1 << s_ctrl->soc_info.index)) {
		pr_err(": debug dir(sensor-%d) had creat before return ", s_ctrl->soc_info.index);
		return -ENOMEM;
	}

	debug_ptr = kzalloc(sizeof(msm_actuator_debug_info_t), GFP_KERNEL);
	if (!debug_ptr) {
		pr_err("failed: no memory s_ctrl %p", debug_ptr);
		return -ENOMEM;
	}

	memset(buf, 0, sizeof(buf));
	snprintf(buf, BUF_SIZE, "actuator-%d", s_ctrl->soc_info.index);
	actuator_dir = debugfs_create_dir(buf, actuator_debugfs_base);

	if (!actuator_dir) {
		pr_err("%s:%d: exit", __func__, __LINE__);
		goto debug_ptr_free;
	}

	debug_ptr->s_ctrl = s_ctrl;
	debug_ptr->msm_actuator_reg_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	debug_ptr->cal_info.gyro_cal_success = 0;

	if (!debugfs_create_file("datatype", S_IRUGO | S_IWUSR, actuator_dir,
			(void *) debug_ptr, &actuator_debugfs_datatype)) {

		pr_err("%s:%d: exit", __func__, __LINE__);
		goto failed_create_file;
	}

	if (!debugfs_create_file("addrtype", S_IRUGO | S_IWUSR, actuator_dir,
			(void *) debug_ptr, &actuator_debugfs_addrtype))
		goto failed_create_file;

	if (!debugfs_create_file("address", S_IRUGO | S_IWUSR, actuator_dir,
			(void *) debug_ptr, &actuator_debugfs_address)) {
		pr_err("%s:%d: exit", __func__, __LINE__);
		goto failed_create_file;
	}

	if (!debugfs_create_file("value", S_IRUGO | S_IWUSR, actuator_dir,
			(void *) debug_ptr, &actuator_debugfs_value)) {

		pr_err("%s:%d: exit", __func__, __LINE__);
		goto failed_create_file;
	}

#if 0
	if (!debugfs_create_file("gyrocal", S_IRUGO | S_IWUSR, actuator_dir,
			(void *) debug_ptr, &actuator_debugfs_gyrocal)) {
#else
	if (!debugfs_create_file("gyrocal", 0444, actuator_dir,
			(void *) debug_ptr, &actuator_debugfs_gyrocal)) {
#endif
		pr_err("%s:%d: exit", __func__, __LINE__);
		goto failed_create_file;
	}

	actuator_probe |= (1 << s_ctrl->soc_info.index);

	CDBG("%s:%d: X", __func__, __LINE__);

	return 0;

failed_create_file:
	debugfs_remove_recursive(actuator_dir);
	actuator_dir = NULL;

debug_ptr_free:
	kfree(debug_ptr);
	return -ENOMEM;
}


