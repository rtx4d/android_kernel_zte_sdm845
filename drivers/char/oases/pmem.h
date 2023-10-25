
#ifndef _OASES_PMEM_H_
#define _OASES_PMEM_H_

#define OASES_PMEM_MAGIC "OASESPMEM~MAGIC~"

struct oases_pmem {
    unsigned char magic[16];
/* bitwise boot flags */
#define OASES_KDUMP_PANIC 0x00000001
    unsigned int boot;
};

extern struct oases_pmem oases_pmem;

#endif /* _OASES_PMEM_H_ */

