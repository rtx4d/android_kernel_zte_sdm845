/*
 * inlinehook.c - inlinehook for functions
 *
 * Copyright (C) 2016 Baidu, Inc. All Rights Reserved.
 *
 * You should have received a copy of license along with this program;
 * if not, ask for it from Baidu, Inc.
 *
 */

#include <linux/gfp.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kallsyms.h>
#include <linux/version.h>
#include <asm/sections.h>
#include <asm/cacheflush.h>
#include "inlinehook.h"
#include "hook_insn.h"
#include "util.h"

#if defined(__aarch64__)

/* any insn with bit[28:27] = 00 kprobes use 0x07F001F8U */
#define OASES_ARM_UNDEF_INSN 0x07F02016U

/*
 * make_jump_insn() - generate a B instruction to jump from @addr to @dst
 *
 * Return: 0 for success, also set insn, otherwise return -1 when failed
 */
int oases_make_jump_insn(u32 *addr, u32 *dst, u32 *insn)
{
	u32 jump_insn;
	unsigned long offset = 0;
	unsigned long forward_max = 0;
	unsigned long backward_min = 0;

	offset = (unsigned long) dst - (unsigned long) addr;
	/*
	 * Forward offset: 0 - 0x7FFFFFCUL
	 * Backward offset: 0xFFFFFFFFF8000000UL - 0xFFFFFFFFFFFFFFFCUL
	 */
	forward_max = 0x7FFFFFCUL;
	backward_min = 0xFFFFFFFFF8000000UL;
	if (offset > forward_max && offset < backward_min) {
		return -1;
	}

	/* offset = SignExtend(imm26:'00', 64) */
	offset = (offset & 0xFFFFFFFUL) >> 2;
	jump_insn = (u32) 0x14000000UL + (u32) offset;
	*insn = jump_insn;
	return 0;
}

/* location dependent instructions */

/* ADR  <Xd>, <label> */
#define ARM64_ADR       1
/* ADRP <Xd>, <label> */
#define ARM64_ADRP      2
/* B.cond <label> */
#define ARM64_BCOND     3
/* B <label> */
#define ARM64_B         4
/* BL <label> */
#define ARM64_BL        5
/* BLR <Xn> */
#define ARM64_BLR       6
/*CB(N)Z <label>*/
#define ARM64_CBNZ      7
#define ARM64_CBZ       8
/* LDR <Xt>, <label>*/
#define ARM64_LDR_32    9
/* LDR <Wt>, <label>*/
#define ARM64_LDR_64    10
/* LDRSW <Xt>, <label>*/
#define ARM64_LDRSW     11
/* TB(N)Z <R><t>, #<imm>, <label> */
#define ARM64_TBNZ      12
#define ARM64_TBZ       13
/* PRFM (prfop|#<imm5>), <label> */
#define ARM64_PRFM		14
/* BR <Xn> */
#define ARM64_BR        15
/* TODO: */
#define ARM64_UNDEF     99

static int get_insn_type(u32 instruction)
{
	if ((instruction & (u32) 0x9F000000UL) == (u32) 0x10000000UL)
		return ARM64_ADR;

	if ((instruction & (u32) 0x9F000000UL) == (u32) 0x90000000UL)
		return ARM64_ADRP;

	if ((instruction & (u32) 0xFF000010UL) == (u32) 0x54000000UL)
		return ARM64_BCOND;

	if ((instruction & (u32) 0xFC000000UL) == (u32) 0x14000000UL)
		return ARM64_B;

	if ((instruction & (u32) 0xFC000000UL) == (u32) 0x94000000UL)
		return ARM64_BL;

	if ((instruction & (u32) 0xFFFFFC1FUL) == (u32) 0xD63F0000UL)
		return ARM64_BLR;

	if ((instruction & (u32) 0xFF000000UL) == (u32) 0xB5000000UL)
		return ARM64_CBNZ;

	if ((instruction & (u32) 0xFF000000UL) == (u32) 0xB4000000UL)
		return ARM64_CBZ;

	if ((instruction & (u32) 0x7F000000UL) == (u32) 0x37000000UL)
		return ARM64_TBNZ;

	if ((instruction & (u32) 0x7F000000UL) == (u32) 0x36000000UL)
		return ARM64_TBZ;

	if ((instruction & (u32) 0xFF000000UL) == (u32) 0x58000000UL)
		return ARM64_LDR_64;

	if ((instruction & (u32) 0xFF000000UL) == (u32) 0x18000000UL)
		return ARM64_LDR_32;

	if ((instruction & (u32) 0xFF000000UL) == (u32) 0x98000000UL)
		return ARM64_LDRSW;

	if ((instruction & (u32) 0xFF000000UL) == (u32) 0xD8000000UL)
		return ARM64_PRFM;

	if ((instruction & (u32) 0xFFFFFC1FUL) == (u32) 0xD61F0000UL)
		return ARM64_BR;

	return ARM64_UNDEF;
}

/*
 * LDR X16, <label>
 * BLR X16
 * label: DCQ
 */
static void trampoline_setup_thunk_lr(u32 *tramp_insn, u64 *tramp_label, u64 target)
{
	u32 ldr_off = (u32 *)tramp_label - tramp_insn;

	*tramp_insn++ = (u32)0x58000000UL + (ldr_off << 5) + 16;
	*tramp_insn++ = (u32)0xD63F0000UL + (16 << 5);
	*tramp_label++ = target;
}

/*
 * LDR X16, <label>
 * BR X16
 * label: DCQ
 */
static void trampoline_setup_thunk(u32 *from, u64 *label, u64 target)
{
	u32 ldr_off = (u32 *)label - from;

	*from++ = (u32)0x58000000UL + (ldr_off << 5) + 16;
	*from++ = (u32)0xD61F0000UL + (16 << 5);
	*label++ = target;
}

/*
 * oases_relocate_insn() - relocate instruction to trampoline
 *
 * Return: 0 if relocate operation succeed, otherwise -1 and cause inlinehook fail
 */
int oases_relocate_insn(struct oases_insn *info, int off)
{
	int ret = 0;
	u32 insn;
	u32 ldr_off;
	u64 offset;
	u32 rd;
	u32 opc;
	u64 adr_immhi;
	u64 adr_immlo;

	u32 *tramp_insn = info->trampoline + off;
	u32 *tramp_thunk = info->trampoline + TRAMPOLINE_THUNK_OFF;
	u32 *tramp_data = info->trampoline + TRAMPOLINE_DATA_OFF;
	u64 *tramp_label = info->trampoline + TRAMPOLINE_LABEL_OFF;

	insn = *((u32 *)info->address);
	switch (get_insn_type(insn)) {
		case ARM64_ADR:
			/*
			 * ADR <Xd>, <label> (+/-1MB)
			 *
			 * Caculate the Xd value of ADR and store it in label of LDR
			 *
			 * LDR <Xd>, <label>
			 * label: PC[] + imm
			*/
			adr_immhi = insn >> 5 & 0x7FFFFUL; /* 19 bits */
			adr_immlo = insn >> 29 & 0x3UL; /* 2 bits */

			/* imm = SignExtend(immhi:immlo, 64) */
			offset = (adr_immhi << 2) + adr_immlo;
			if (offset & 0x100000UL) {
				offset += 0xFFFFFFFFFFE00000UL;
			}

			rd = insn & 0x1FUL;
			ldr_off = (u32 *)tramp_label - tramp_insn;
			*tramp_insn++ = (u32)0x58000000UL + (ldr_off << 5) + rd;
			*tramp_label++ = (u64)info->address + offset;
			break;
		case ARM64_ADRP:
			/*
			 * ADRP <Xd>, <label> (+/-4GB)
			 *
			 * Caculate the Xd value of ADRP and store it in label of LDR
			 *
			 * LDR <Xd>, <label>
			 * label: PC[](PC[11:0] = ZERO(12)) + imm
			*/
			adr_immhi = insn >> 5 & 0x7FFFFUL; /* 19 bits */
			adr_immlo = insn >> 29 & 0x3UL; /* 2 bits */

			/* imm = SignExtend(immhi:immlow:Zeros(12), 64) */
			offset = ((adr_immhi << 2) + adr_immlo) << 12;
			if (offset & 0x100000000UL) {
				offset += 0xFFFFFFFE00000000UL;
			}

			rd = insn & 0x1FUL;
			ldr_off = (u32 *)tramp_label - tramp_insn;
			*tramp_insn++ = (u32)0x58000000UL + (ldr_off << 5) + rd;
			*tramp_label++ = ((u64)info->address & 0xFFFFFFFFFFFFF000UL) + offset;
			break;
		case ARM64_BCOND:
			/*
			 * B.<cond> <label> (+/-1MB)
			 *
			 * copy and modify the insn to branch to thunk which jump to original target
			 */
			ldr_off = tramp_thunk - tramp_insn;
			/* B.<cond> thunk */
			*tramp_insn++ = (insn & (u32) 0xFF00001FUL) + (ldr_off << 5);

			/* imm = SignExtend(imm19:'00',64) */
			offset = (insn >> 5 & 0x7FFFFUL) << 2;
			if (offset & 0x100000UL) {
				offset += 0xFFFFFFFFFFE00000UL;
			}
			trampoline_setup_thunk(tramp_thunk, tramp_label, offset + (u64)info->address);
			break;
		case ARM64_B:
			/*
			 * B <label> (+/-128MB)
			 *
			 * Calculate the target address of B instruction, and store it in LDR label.
			 *
			 * LDR Xd, <label>; BR/RET Xd;
			 */
			offset = (insn & (u32) 0x3FFFFFFUL) << 2; /* 28 bits */
			if (offset & 0x8000000UL) {
				offset += 0xFFFFFFFFF0000000UL;
			}
			trampoline_setup_thunk(tramp_insn, tramp_label, offset + (u64)info->address);
			break;
		case ARM64_BL:
			/* BL <label> */
			offset = (insn & (u32) 0x3FFFFFFUL) << 2; /* 28 bits */
			if (offset & 0x8000000UL) {
				offset += 0xFFFFFFFFF0000000UL;
			}
			trampoline_setup_thunk_lr(tramp_insn, tramp_label, offset + (u64)info->address);
			break;
		case ARM64_BR:
		case ARM64_BLR:
			/*
			 * B(L)R <Xn>
			 *
			 * for kernel with CFI enabled, we can't hook funcs with BLR as the first insn
			 */
#if OASES_ENABLE_CFI
			ret = -1;
#else
			rd = (insn >> 5 ) & 0x1FUL;
			if (rd < 8 ) {
				*tramp_insn++ = insn;
			} else {
				ret = -1;
			}
#endif
			break;
		case ARM64_CBNZ:
		case ARM64_CBZ:
			/*
			 * CB(N)Z Rt, <label> (+/-1MB)
			 *
			 * CB(N)Z THUNK
			 */
			ldr_off = tramp_thunk - tramp_insn;
			*tramp_insn++ = (insn & (u32) 0xFF00001FUL) + (ldr_off << 5);
			offset = ((insn >> 5) & (u32) 0x7FFFFUL) << 2; /* imm19 */
			if (offset & 0x100000UL) {
				offset += 0xFFFFFFFFFFE00000UL;
			}
			trampoline_setup_thunk(tramp_thunk, tramp_label, offset + (u64)info->address);
			break;
		case ARM64_LDR_32:
		case ARM64_LDR_64:
		case ARM64_LDRSW:
			/*
			 * LDR <W|X>t, <label> +/-1MB
			 * LDRSW Xt, <label> +/-1MB
			 */
			offset = ((insn >> 5) & (u32) 0x7FFFFUL) << 2;
			if (offset & 0x100000UL) {
				offset += 0xFFFFFFFFFFE00000UL;
			}

			/*
			 * opc=00, LDR, size=4;
			 * opc=01, LDR, size=8;
			 * opc=10, LDRSW, size=4, signed=true;
			 * opc=11, prefetch, size=8???
			 */
			opc = insn >> 30;
			if (opc == 1 || opc == 3) {
				ldr_off = (u32 *)tramp_label - tramp_insn;
				*tramp_insn++ = (insn & (u32) 0xFF00001FUL) + (ldr_off << 5);
				*tramp_label++ = *((u64 *)(offset + (u64)info->address));
			} else {
				ldr_off = tramp_data - tramp_insn;
				*tramp_insn++ = (insn & (u32) 0xFF00001FUL) + (ldr_off << 5);
				*tramp_data++ = *(u32 *)(offset + (u64)info->address);
			}
			/* another way: LDR Xd, <label>; LDR Xd, <Xd>; label save the target address */
			break;
		case ARM64_TBNZ:
		case ARM64_TBZ:
			/*
			 * TB(N)Z Rt, #<imm>, <label> (+/-32KB)
			 *
			 * TB(N)Z THUNK
			 */
			ldr_off = tramp_thunk - tramp_insn;
			*tramp_insn++ = (insn & (u32) 0xFFF8001FUL) + (ldr_off << 5);
			offset = ((insn >> 5) & (u32) 0x3FFFUL) << 2; /* imm 14 bits */
			if (offset & 0x8000UL) {
				offset += 0xFFFFFFFFFFFF0000UL;
			}
			trampoline_setup_thunk(tramp_thunk, tramp_label, offset + (u64)info->address);
			break;
		case ARM64_PRFM:
			/*
			 * PRFM #<imm5>, <label>
			 *
			 * LDR X16, <label>; PRFM #<imm5>, [X16]
			 */
			offset = (insn >> 5 & 0x7FFFFUL) << 2;
			if (offset & 0x100000UL) {
				offset += 0xFFFFFFFFFFE00000UL;
			}

			ldr_off = (u32 *)tramp_label - tramp_insn;
			*tramp_insn++ = (u32) 0x58000000UL + (ldr_off << 5) + 16;
			*tramp_label++ = offset + (u64)info->address;
			/* PRFM(immediate) */
			*tramp_insn++ = (u32) 0xF9800000UL + (16 << 5) + (insn & 0x1FUL);
			break;
		default:
			*tramp_insn++ = insn;
			break;
	}

	return ret;
}

#elif defined(__arm__)

int oases_make_jump_insn(u32 *addr, u32 *dst, u32 *insn)
{
	u32 jump_insn;
	unsigned long offset = (unsigned long) dst - (unsigned long) addr - 8;
	/* imm24 */
	unsigned long forward_max = 0x7fffffUL << 2, backward_min = 0xff800000UL << 2;

	if (offset > forward_max && offset < backward_min)
		return -1;
	jump_insn = (u32) 0xea000000UL | (u32)((offset >> 2) & 0x00ffffffUL);
	*insn = jump_insn;
	return 0;
}

#define ARM_REG_LR 14
#define ARM_REG_PC 15

enum {
	ARM_ADR_FORWARD,
	ARM_ADR_BACKWARD,

	ARM_B_IMM,

	ARM_BL_IMM,
	/*ARM_BLX_IMM, kenel is arm-only*/

	/* ARM_BLX_REG, ARM_LDR_IMM */

	ARM_BX_REG,

	ARM_LDR_LITERAL,
	ARM_LDR_REG,

	ARM_LDRB_LITERAL,
	ARM_LDRB_REG,

	ARM_LDRH_LITERAL,
	ARM_LDRH_REG,

	ARM_LDRSB_LITERAL,
	ARM_LDRSB_REG,

	ARM_LDRSH_LITERAL,
	ARM_LDRSH_REG,

	ARM_LDRD_LITERAL,
	ARM_LDRD_REG,

	ARM_MOVE_REG,

	ARM_ADD_REG,
	/*
	rare cases(instructions below usually don't use PC,
	so the relocation is not necessary)

	ARM_PLD_LITERAL,
	ARM_PLD_REG,

	ARM_PLI_LITERAL,
	ARM_PLI_REG,

	ARM_PUSH,
	ARM_STM,
	ARM_STMDA,
	ARM_STMDB,
	ARM_STMIA,
	*/
	ARM_INSN_UNDEF
};

static int get_insn_type(u32 insn)
{
	if ((insn & 0x0FFF0000UL) == 0x028F0000UL)
		return ARM_ADR_FORWARD;

	if ((insn & 0x0FFF0000UL) == 0x024F0000UL)
		return ARM_ADR_BACKWARD;

	if ((insn & 0x0F000000UL) == 0x0A000000UL)
		return ARM_B_IMM;

	if ((insn & 0x0F000000UL) == 0x0B000000UL)
		return ARM_BL_IMM;
	/*

	if ((insn & 0xFE000000UL) == 0xFA000000UL)
		return ARM_BLX_IMM;

	if ((insn & 0x0FFFFFF0UL) == 0x012FFF30UL)
		return ARM_BLX_REG;

	if ((insn & 0x0E300000UL) == 0x04100000UL)
		return ARM_LDR_IMM;
	*/

	if ((insn & 0x0FFFFFF0UL) == 0x012FFF10UL)
		return ARM_BX_REG;

	if ((insn & 0x0F7F0000UL) == 0x051F0000UL)
		return ARM_LDR_LITERAL;

	if (((insn & 0x0E500010UL) == 0x06100000UL) &&
		(((insn & 0x01000000UL) == 0x01000000UL) || ((insn & 0x00200000UL) == 0x00000000UL)))
		return ARM_LDR_REG;

	if ((insn & 0x0F7F0000UL) == 0x055F0000UL)
		return ARM_LDRB_LITERAL;

	if (((insn & 0x0E500010UL) == 0x06500000UL) &&
		(((insn & 0x01000000UL) == 0x01000000UL) || ((insn & 0x00200000UL) == 0x00000000UL)))
		return ARM_LDRB_REG;

	if ((insn & 0x0F7F00F0UL) == 0x015F00B0UL)
		return ARM_LDRH_LITERAL;

	if (((insn & 0x0E500FF0UL) == 0x001000B0UL) &&
		(((insn & 0x01000000UL) == 0x01000000UL) || ((insn & 0x00200000UL) == 0x00000000UL)))
		return ARM_LDRH_REG;

	if ((insn & 0x0F7F00F0UL) == 0x015F00D0UL)
		return ARM_LDRSB_LITERAL;

	if (((insn & 0x0E500FF0UL) == 0x001000D0UL) &&
		(((insn & 0x01000000UL) == 0x01000000UL) || ((insn & 0x00200000UL) == 0x00000000UL)))
		return ARM_LDRSB_REG;

	if ((insn & 0x0F7F00F0UL) == 0x015F00F0UL)
		return ARM_LDRSH_LITERAL;

	if (((insn & 0x0E500FF0UL) == 0x001000F0UL) &&
		(((insn & 0x01000000UL) == 0x01000000UL) || ((insn & 0x00200000UL) == 0x00000000UL)))
		return ARM_LDRSH_REG;

	if ((insn & 0x0F7F00F0UL) == 0x014F00D0UL)
		return ARM_LDRD_LITERAL;

	if ((insn & 0x0E500FF0UL) == 0x000000D0)
		return ARM_LDRD_REG;

	if ((insn & 0x0FEF0FF0UL) == 0x01A00000UL)
		return ARM_MOVE_REG;

	if ((insn & 0x0FE00010UL) == 0x00800000UL)
		return ARM_ADD_REG;

	return ARM_INSN_UNDEF;
}

int oases_relocate_insn(struct oases_insn *info, int off)
{
	int type, forward;
	u32 insn, ldr_off, data;
	u32 cond, rd, rt, rm, rn, imm12, imm24, imm32;
	u32 rx;

	u32 *tramp_insn = info->trampoline + off;
	//u32 *tramp_thunk = info->trampoline + TRAMPOLINE_THUNK_OFF;
	//u32 *tramp_data = info->trampoline + TRAMPOLINE_DATA_OFF;
	u32 *tramp_label = info->trampoline + TRAMPOLINE_LABEL_OFF;

	insn = *((u32 *)info->address);
	type = get_insn_type(insn);

	switch(type) {
	case ARM_ADR_FORWARD:
	case ARM_ADR_BACKWARD:
		/*
		 * ADR<c> <Rd>, <label>
		 *
		 * LDR<c> <Rd>, <label>
		 * <label>: ADR target
		 */
		forward = 0;
		if (((insn >> 20) & 0x0F) == 8)
			forward = 1;
		rd = (insn >> 12) & 0x0F;
		imm12 = insn & 0x0FFF;
		cond = (insn >> 28) & 0x0F;
		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (rd << 12) + ldr_off;
		if (forward)
			*tramp_label++ = (u32)info->address + 8 + imm12;
		else
			*tramp_label++ = (u32)info->address + 8 - imm12;
		break;

	case ARM_B_IMM:
		/*
		 * B<c> <label>
		 *
		 * => LDR<c> PC, <label>
		 * label: B target
		 */
		cond = (insn >> 28) & 0x0F;
		imm24 = (insn & 0x00FFFFFF) << 2;
		if (imm24 & 0x2000000)
			imm24 += 0xFC000000;

		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (ARM_REG_PC << 12) + ldr_off;
		*tramp_label++ = (u32)info->address + 8 + imm24;
		break;

	case ARM_BX_REG:
		/*
		 * BX<c> Rm
		 *
		 * => LDR<c> PC, <label>
		 * label: BX target
		 */
		rm = insn & 0x0F;
		if (rm == ARM_REG_PC) {
			cond = (insn >> 28) & 0x0F;
			ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
			*tramp_insn++ = (cond << 28) + 0x059F0000 + (ARM_REG_PC << 12) + ldr_off;
			*tramp_label++ = (u32)info->address + 8;
		} else {
			*tramp_insn++ = insn;
		}
		break;

	case ARM_BL_IMM:
		/*
		 * BL<c> <label>
		 *
		 * => LDR<c> LR, <label1>
		 *    LDR<c> PC, <label2>
		 */
		cond = (insn >> 28) & 0x0F;
		imm24 = (insn & 0x00FFFFFF) << 2;
		if (imm24 & 0x2000000)
			imm24 += 0xFC000000;

		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (ARM_REG_LR << 12) + ldr_off;
		*tramp_label++ = (u32)info->address + 4;

		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (ARM_REG_PC << 12) + ldr_off;
		*tramp_label++ = (u32)info->address + 8 + imm24;
		break;

	/* case ARM_LDRD_LITERAL: */
	case ARM_LDR_LITERAL:
		/*
		 * LDR<c> <Rt>, <lable>
		 *
		 * => LDR Rt, label
		 * label: ori_data
		 */
		forward = 0;
		data = 0;
		if ((insn >> 23) & 1)
			forward = 1;
		imm12 = insn & 0x0FFF;
		if (forward) {
			data = *((u32 *)(info->address + 8 + imm12));
		} else {
			data = *((u32 *)(info->address + 8 - imm12));
		}
		rt = (insn >> 12) & 0x0F;
		cond = (insn >> 28) & 0x0F;
		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (rt << 12) + ldr_off;
		*tramp_label++ = data;
		break;

	case ARM_LDRB_LITERAL:
		/*
		 * LDRB<c> <Rt>, <lable>
		 *
		 * => LDR Rt, label
		 * label: ori_data & 0xFF
		 */
		forward = 0;
		data = 0;
		if ((insn >> 23) & 1)
			forward = 1;
		imm12 = insn & 0x0FFF;
		if (forward) {
			data = *((u32 *)(info->address + 8 + imm12));
		} else {
			data = *((u32 *)(info->address + 8 - imm12));
		}
		data &= 0xFF;
		rt = (insn >> 12) & 0x0F;
		cond = (insn >> 28) & 0x0F;
		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (rt << 12) + ldr_off;
		*tramp_label++ = data;
		break;

	case ARM_LDRH_LITERAL:
		/*
		 * LDRH<c> <Rt>, <lable>
		 *
		 * => LDR Rt, label
		 * label: ori_data & 0xFFFF
		 */
		forward = 0;
		data = 0;
		if ((insn >> 23) & 1)
			forward = 1;
		imm32 = (((insn >> 8) & 0x0F) << 4) + (insn & 0x0F);
		if (forward) {
			data = *((u32 *)(info->address + 8 + imm32));
		} else {
			data = *((u32 *)(info->address + 8 - imm32));
		}
		data &= 0xFFFF;
		rt = (insn >> 12) & 0x0F;
		cond = (insn >> 28) & 0x0F;
		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (rt << 12) + ldr_off;
		*tramp_label++ = data;
		break;

	case ARM_LDRSB_LITERAL:
		/*
		 * LDRSB<c> <Rt>, <lable>
		 *
		 * => LDR Rt, label
		 * label: sign_extend(ori_data & 0xFF)
		 */
		forward = 0;
		data = 0;
		if ((insn >> 23) & 1)
			forward = 1;
		imm32 = (((insn >> 8) & 0x0F) << 4) + (insn & 0x0F);
		if (forward) {
			data = *((u32 *)(info->address + 8 + imm32));
		} else {
			data = *((u32 *)(info->address + 8 - imm32));
		}
		if (data & 0x80)
			data |= 0xFFFFFF00;
		else
			data &= 0xFF;
		rt = (insn >> 12) & 0x0F;
		cond = (insn >> 28) & 0x0F;
		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (rt << 12) + ldr_off;
		*tramp_label++ = data;
		break;

	case ARM_LDRSH_LITERAL:
		/*
		 * LDRSH<c> <Rt>, <lable>
		 *
		 * => LDR Rt, label
		 * label: sign_extend(ori_data & 0xFFFF)
		 */
		forward = 0;
		data = 0;
		if ((insn >> 23) & 1)
			forward = 1;
		imm32 = (((insn >> 8) & 0x0F) << 4) + (insn & 0x0F);
		if (forward) {
			data = *((u32 *)(info->address + 8 + imm32));
		} else {
			data = *((u32 *)(info->address + 8 - imm32));
		}
		if (data & 0x8000)
			data |= 0xFFFF0000;
		else
			data &= 0xFFFF;
		rt = (insn >> 12) & 0x0F;
		cond = (insn >> 28) & 0x0F;
		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (rt << 12) + ldr_off;
		*tramp_label++ = data;
		break;

	case ARM_LDRD_LITERAL:
		/*
		*LDRD<c> <Rt>, <Rt2>, <label>
		*
		*=> LDR<c> <Rt>, label
		*      LDR<c> <Rt2>, label + 4
		*label: ori_data1
		*         ori_data2
		*/
		forward = 0;
		data = 0;

		if ((insn >> 23) & 1)
			forward = 1;
		imm32 = (((insn >> 8) & 0x0F) << 4) + (insn & 0x0F);
		if (forward) {
			data = *((u32 *)(info->address + 8 + imm32));
		} else {
			data = *((u32 *)(info->address + 8 - imm32));
		}
		rt = (insn >> 12) & 0x0F;
		cond = (insn >> 28) & 0x0F;
		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (rt << 12) + ldr_off;
		*tramp_label++ = data;

		if (forward) {
			data = *((u32 *)(info->address + 8 + imm32 + 4));
		} else {
			data = *((u32 *)(info->address + 8 - imm32 + 4));
		}
		rt++;
		ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
		*tramp_insn++ = (cond << 28) + 0x059F0000 + (rt << 12) + ldr_off;
		*tramp_label++ = data;
		break;

	case ARM_LDRSH_REG:
	case ARM_LDRSB_REG:
	case ARM_LDRH_REG:
	case ARM_LDRB_REG:
	case ARM_LDR_REG:
		/*
		*
		*LDRX<c> <Rt>, [<PC>,+/-<Rm>{, <shift>}]  (no wirte back)
		*
		*=> PUSH<c> {Rx}
		*     LDR<c> Rx, label
		*	LDRX<c> <Rt>, [<Rx>,+/-<Rm>{, <shift>}]
		*     POP<c> {Rx}
		*label: PC
		*/
		rn = (insn >> 16) & 0x0F;
		if (rn == ARM_REG_PC) {
			cond = (insn >> 28) & 0x0F;
			rt = (insn >> 12) & 0x0F;
			rm = insn & 0x0F;
			for (rx = 0; rx < 14; rx++) {
				if ((rx != rt) && (rx != rm))
					break;
			}
			*tramp_insn++ = (cond << 28) + 0x052D0004 + (rx << 12);
			ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
			*tramp_insn++ = (cond << 28) + 0x059F0000 + (rx << 12) + ldr_off;
			*tramp_label++ = (u32)info->address + 8;

			insn &= 0xFFF0FFFF;
			insn |= rx << 16;
			*tramp_insn++ = insn;
			*tramp_insn++ = (cond << 28) + 0x049D0004 + (rx << 12);
		} else {
			*tramp_insn++ = insn;
		}
		break;

	case ARM_LDRD_REG:
		/*
		*
		*LDRD<c> <Rt>, <Rt2>, [<PC>,+/-<Rm>{, <shift>}]  (no wirte back)
		*
		*=> PUSH<c> {Rx}
		*     LDR<c> Rx, label
		*	LDRD<c> <Rt>, <Rt2>, [<Rx>,+/-<Rm>{, <shift>}]
		*     POP<c> {Rx}
		*label: PC
		*/
		rn = (insn >> 16) & 0x0F;
		if (rn == ARM_REG_PC) {
			cond = (insn >> 28) & 0x0F;
			rt = (insn >> 12) & 0x0F;
			rm = insn & 0x0F;
			for (rx = 0; rx < 14; rx++) {
				if ((rx != rt) && (rx != rm) && (rx != rt + 1))
					break;
			}
			*tramp_insn++ = (cond << 28) + 0x052D0004 + (rx << 12);
			ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
			*tramp_insn++ = (cond << 28) + 0x059F0000 + (rx << 12) + ldr_off;
			*tramp_label++ = (u32)info->address + 8;

			insn &= 0xFFF0FFFF;
			insn |= rx << 16;
			*tramp_insn++ = insn;
			*tramp_insn++ = (cond << 28) + 0x049D0004 + (rx << 12);
		} else {
			*tramp_insn++ = insn;
		}
		break;

	case ARM_MOVE_REG:
		/*
		 * MOV{S}<c> <Rd>, <PC>
		 *
		 * => LDR<c> Rd, label
		 *label: PC
		 */
		rm = insn & 0x0F;
		rd = (insn >> 12) & 0x0F;
		cond = (insn >> 28) & 0x0F;
		if (rm == ARM_REG_PC) {
			ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
			*tramp_insn++ = (cond << 28) + 0x059F0000 + (rd << 12) + ldr_off;
			*tramp_label++ = (u32)info->address + 8;
		} else {
			*tramp_insn++ = insn;
		}
		break;

	case ARM_ADD_REG:
		/*
		*ADD{S}<c> <Rd>, <PC>, <Rm>{, <shift>}
		*
		*=> PUSH {Rx}
		*      LDR<c> Rx, label
		*	 ADD{S}<c> <Rd>, <Rx>, <Rm>{, <shift>}
		*	 POP {Rx}
		*label: PC
		*
		*
		*ADD{S}<c> <Rd>, <Rn>, <PC>{, <shift>}
		*
		*=> PUSH {Rx}
		*	LDR<c> Rx, label
		*	ADD{S}<c> <Rd>, <Rn>, <Rx>{, <shift>}
		*     POP {Rx}
		*label:PC
		*/
		rm = insn &0x0F;
		rd = (insn >> 12) & 0x0F;
		rn = (insn >> 16) & 0x0F;
		cond = (insn >> 28) & 0x0F;
		if ((rm == ARM_REG_PC) && (rn == ARM_REG_PC))
			return -1;
		if (rn == ARM_REG_PC) {
			for (rx = 0; rx < 14; rx++) {
				if ((rx != rd) && (rx != rm))
					break;
			}
			*tramp_insn++ = (cond << 28) + 0x052D0004 + (rx << 12);
			ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
			*tramp_insn++ = (cond << 28) + 0x059F0000 + (rx << 12) + ldr_off;
			*tramp_label++ = (u32)info->address + 8;
			insn &= 0xFFF0FFFF;
			insn |= rx << 16;
			*tramp_insn++ = insn;
			*tramp_insn++ = (cond << 28) + 0x049D0004 + (rx << 12);
		} else if (rm == ARM_REG_PC) {
			for (rx = 0; rx < 14; rx++) {
				if ((rx != rd) && (rx != rn))
					break;
			}
			*tramp_insn++ = (cond << 28) + 0x052D0004 + (rx << 12);
			ldr_off = (u32)tramp_label - (u32)tramp_insn - 8;
			*tramp_insn++ = (cond << 28) + 0x059F0000 + (rx << 12) + ldr_off;
			*tramp_label++ = (u32)info->address + 8;
			insn &= 0xFFFFFFF0;
			insn |= rx;
			*tramp_insn++ = insn;
			*tramp_insn++ = (cond << 28) + 0x049D0004 + (rx << 12);
		} else {
			*tramp_insn++ = insn;
		}
		break;

	default:
		*tramp_insn++ = insn;
		break;
	}

	return 0;
}

#endif

