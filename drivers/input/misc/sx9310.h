/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#ifndef SX9310_H
#define SX9310_H

/*
 *  I2C Registers
 */
#define SX9310_IRQSTAT_REG    0x00
#define SX9310_TOUCH_BIT 0x6
#define SX9310_REASE_BIT 0x5
#define SX9310_STAT0_REG    0x01
#define SX9310_STAT1_REG    0x02
#define SX9310_IRQ_ENABLE_REG 0x03
#define SX9310_IRQFUNC_REG 0x04

#define SX9310_CPS_CTRL0_REG    0x10
#define SX9310_CPS_CTRL1_REG    0x11
#define SX9310_CPS_CTRL2_REG    0x12
#define SX9310_CPS_CTRL3_REG    0x13
#define SX9310_CPS_CTRL4_REG    0x14
#define SX9310_CPS_CTRL5_REG    0x15
#define SX9310_CPS_CTRL6_REG    0x16
#define SX9310_CPS_CTRL7_REG    0x17
#define SX9310_CPS_CTRL8_REG    0x18
#define SX9310_CPS_CTRL9_REG   0x19
#define SX9310_CPS_CTRL10_REG   0x1A
#define SX9310_CPS_CTRL11_REG   0x1B
#define SX9310_CPS_CTRL12_REG   0x1C
#define SX9310_CPS_CTRL13_REG   0x1D
#define SX9310_CPS_CTRL14_REG   0x1E
#define SX9310_CPS_CTRL15_REG   0x1F
#define SX9310_CPS_CTRL16_REG   0x20
#define SX9310_CPS_CTRL17_REG   0x21
#define SX9310_CPS_CTRL18_REG   0x22
#define SX9310_CPS_CTRL19_REG   0x23
#define SX9310_SAR_CTRL0_REG   0x2A
#define SX9310_SAR_CTRL1_REG   0x2B
#define SX9310_SAR_CTRL2_REG   0x2C

#define SX9310_SOFTRESET_REG  0x7F

/*      Sensor Readback */
#define SX9310_CPSRD          0x30

#define SX9310_USEMSB         0x31
#define SX9310_USELSB         0x32

#define SX9310_AVGMSB         0x33
#define SX9310_AVGLSB         0x34

#define SX9310_DIFFMSB        0x35
#define SX9310_DIFFLSB        0x36
#define SX9310_OFFSETMSB      0x37
#define SX9310_OFFSETLSB      0x38

#define SX9310_SARMSB         0x39
#define SX9310_SARLSB         0x3A

#define SX9310_WHOAMI_REG         0x42

/*      IrqStat 0:Inactive 1:Active     */
#define SX9310_IRQSTAT_RESET_FLAG      0x80
#define SX9310_IRQSTAT_TOUCH_FLAG      0x40
#define SX9310_IRQSTAT_RELEASE_FLAG    0x20
#define SX9310_IRQSTAT_COMPDONE_FLAG   0x10
#define SX9310_IRQSTAT_CONV_FLAG       0x08
#define SX9310_IRQSTAT_CLOSEALL_FLAG   0x04
#define SX9310_IRQSTAT_FARALL_FLAG     0x02
#define SX9310_IRQSTAT_SMARTSAR_FLAG   0x01


/* CpsStat  */
#define SX9310_TCHCMPSTAT_TCHCOMB_FLAG    0x08
#define SX9310_TCHCMPSTAT_TCHSTAT2_FLAG   0x04
#define SX9310_TCHCMPSTAT_TCHSTAT1_FLAG   0x02
#define SX9310_TCHCMPSTAT_TCHSTAT0_FLAG   0x01


/*      SoftReset */
#define SX9310_SOFTRESET  0xDE

#define BITGET(byte, bit) (byte & (1 << bit))

struct smtc_reg_data {
unsigned char reg;
unsigned char val;
};

struct _buttonInfo {
  /*! The Key to send to the input */
int keycode;
  /*! Mask to look for on Touch Status */
int mask;
  /*! Current state of button. */
int state;
};

#endif
