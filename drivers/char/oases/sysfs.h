#ifndef _OASES_SYSFS_H
#define _OASES_SYSFS_H

struct oases_patch_info;

int oases_sysfs_add_patch(struct oases_patch_info* info);
void oases_sysfs_init_patch(struct oases_patch_info *info);
void oases_sysfs_del_patch(struct oases_patch_info *info);

int oases_sysfs_init(void);
void oases_sysfs_destroy(void);

#endif /* _OASES_SYSFS_H */
