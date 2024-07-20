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

#include "cam_ois_dev.h"

#include "cam_sensor_io.h"
#include "cam_sensor_i2c.h"
#include "zte_camera_ois_util.h"


#define CONFIG_ZTE_OIS_UTIL_DEBUG

#undef CDBG
#ifdef CONFIG_ZTE_CAMERA_UTIL_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif



typedef struct {
	struct cam_ois_ctrl_t *s_ctrl;
	enum camera_sensor_i2c_type msm_ois_reg_data_type;
	enum camera_sensor_i2c_type msm_ois_reg_addr_type;
	uint64_t address;
} msm_ois_debug_info_t;

static int ois_debugfs_datatype_s(void *data, u64 val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;

	if (val < CAMERA_SENSOR_I2C_TYPE_MAX
		&& val >= CAMERA_SENSOR_I2C_TYPE_BYTE)
		ptr->msm_ois_reg_data_type = val;

	CAM_DBG(CAM_OIS, "%s:%d: msm_ois_reg_data_type = %d",
		__func__, __LINE__, ptr->msm_ois_reg_data_type);

	return 0;
}

static int ois_debugfs_datatype_g(void *data, u64 *val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;

	*val = ptr->msm_ois_reg_data_type;
	CDBG(CAM_OIS, "%s:%d: msm_ois_reg_data_type = %d",
		__func__, __LINE__, ptr->msm_ois_reg_data_type);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ois_debugfs_datatype, ois_debugfs_datatype_g,
			ois_debugfs_datatype_s, "%llx\n");

static int ois_debugfs_addrtype_s(void *data, u64 val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;

	if (val < CAMERA_SENSOR_I2C_TYPE_MAX
		&& val >= CAMERA_SENSOR_I2C_TYPE_BYTE)
		ptr->msm_ois_reg_addr_type = val;

	CAM_DBG(CAM_SENSOR, "%s:%d: msm_sensor_reg_data_type = %d",
		__func__, __LINE__, ptr->msm_ois_reg_data_type);
	return 0;
}

static int ois_debugfs_addrtype_g(void *data, u64 *val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;

	*val = ptr->msm_ois_reg_addr_type;

	CAM_DBG(CAM_SENSOR, "%s:%d: msm_sensor_reg_addr_type = %d",
		__func__, __LINE__, ptr->msm_ois_reg_addr_type);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ois_debugfs_addrtype, ois_debugfs_addrtype_g,
			ois_debugfs_addrtype_s, "%llx\n");

static int ois_debugfs_setaddr(void *data, u64 val)
{

	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;

	ptr->address = val;

	CDBG(CAM_OIS, "%s:%d: address = 0x%llx",
		__func__, __LINE__, ptr->address);

	return 0;
}

static int ois_debugfs_getaddr(void *data, u64 *val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;

	*val = ptr->address;

	CDBG(CAM_OIS, "%s:%d: address = 0x%llx",
		__func__, __LINE__, ptr->address);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ois_debugfs_address, ois_debugfs_getaddr,
			ois_debugfs_setaddr, "%llx\n");

static int ois_debugfs_setvalue(void *data, u64 val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;
	int32_t rc = 0;

	CDBG("%s:%d: address = 0x%llx  value = 0x%llx",
		__func__, __LINE__, ptr->address, val);

	rc = cam_cci_i2c_write(&(ptr->s_ctrl->io_master_info),
			ptr->address, val,
			ptr->msm_ois_reg_addr_type,
			ptr->msm_ois_reg_data_type);
	if (rc < 0) {
		pr_err("%s:%d: i2c write %llx failed", __func__, __LINE__, val);
		return rc;
	}

	return 0;
}

static int ois_debugfs_getvalue(void *data, u64 *val)
{
	msm_ois_debug_info_t *ptr = (msm_ois_debug_info_t *) data;
	int32_t rc = 0;
	uint32_t temp;



	rc = camera_io_dev_read(
			&(ptr->s_ctrl->io_master_info),
			ptr->address, &temp,
			ptr->msm_ois_reg_addr_type,

			ptr->msm_ois_reg_data_type);


	if (rc < 0) {
		pr_err("%s:%d: i2c read %x failed", __func__, __LINE__, temp);
		return rc;
	}

	*val = temp;

	CDBG("%s:%d: address = 0x%llx  value = 0x%x", __func__, __LINE__,
		ptr->address, temp);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ois_debugfs_value, ois_debugfs_getvalue,
			ois_debugfs_setvalue, "%llx\n");

struct dentry *ois_debugfs_base = NULL;

int ois_probe = 0;

#define BUF_SIZE 15


void msm_ois_creat_debugfs(void)
{
	if (!ois_debugfs_base) {
		ois_debugfs_base = debugfs_create_dir("zte_ois", NULL);
		if (!ois_debugfs_base) {
			pr_err(": zte_ois dir creat fail");
		}
	}
}

int msm_ois_enable_debugfs(struct cam_ois_ctrl_t *s_ctrl)

{
	struct dentry  *ois_dir;
	msm_ois_debug_info_t *debug_ptr = NULL;
	char buf[BUF_SIZE];



	CDBG("%s:%d:  E", __func__, __LINE__);

	if (!ois_debugfs_base) {
		ois_debugfs_base = debugfs_create_dir("zte_ois", NULL);
		if (!ois_debugfs_base) {
			pr_err("%s:%d: exit", __func__, __LINE__);
			return -ENOMEM;
		}
	}


	if (ois_probe & (1 << s_ctrl->soc_info.index)) {

		pr_err(": debug dir(sensor-%d) had creat before return ", s_ctrl->soc_info.index);
		return -ENOMEM;
	}



	debug_ptr = kzalloc(sizeof(msm_ois_debug_info_t), GFP_KERNEL);
	if (!debug_ptr) {
		pr_err("failed: no memory s_ctrl %p", debug_ptr);
		return -ENOMEM;
	}


	memset(buf, 0, sizeof(buf));
	snprintf(buf, BUF_SIZE, "ois-%d", s_ctrl->soc_info.index);

	ois_dir = debugfs_create_dir(buf, ois_debugfs_base);

	if (!ois_dir) {
		pr_err("%s:%d: exit", __func__, __LINE__);
		goto debug_ptr_free;
	}

	debug_ptr->s_ctrl = s_ctrl;
	debug_ptr->msm_ois_reg_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;


	if (!debugfs_create_file("datatype", S_IRUGO | S_IWUSR, ois_dir,
			(void *) debug_ptr, &ois_debugfs_datatype)) {
		pr_err("%s:%d: exit", __func__, __LINE__);
		goto failed_create_file;
	}


	if (!debugfs_create_file("addrtype", S_IRUGO | S_IWUSR, ois_dir,

			(void *) debug_ptr, &ois_debugfs_addrtype))

		goto failed_create_file;



	if (!debugfs_create_file("address", S_IRUGO | S_IWUSR, ois_dir,
			(void *) debug_ptr, &ois_debugfs_address)) {
		pr_err("%s:%d: exit", __func__, __LINE__);
		goto failed_create_file;
	}

	if (!debugfs_create_file("value", S_IRUGO | S_IWUSR, ois_dir,
			(void *) debug_ptr, &ois_debugfs_value)) {
		pr_err("%s:%d: exit", __func__, __LINE__);
		goto failed_create_file;
	}


	ois_probe |= (1 << s_ctrl->soc_info.index);

	CDBG("%s:%d: X", __func__, __LINE__);

	return 0;

failed_create_file:
	debugfs_remove_recursive(ois_dir);
	ois_dir = NULL;
debug_ptr_free:
	kfree(debug_ptr);
	return -ENOMEM;
}


