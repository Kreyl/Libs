/*
 * File:   acc_mma.h
 * Author: Kreyl
 *
 * Created on June 4, 2011, 11:56 AM
 */

#pragma once

#include "kl_lib.h"
#include "shell.h"
#include "board.h"
#include "kl_i2c.h"

#define MOTION_BY_IRQ           FALSE //TRUE

#if MOTION_BY_IRQ
#define ACC_MOTION_TRESHOLD     17      // 1...127. The threshold resolution is 0.063g/LSB.
#else
#define MOTION_THRESHOLD_TOP    1152
#define MOTION_THRESHOLD_BOTTOM 900
#endif

#define ACC_ACCELERATIONS_NEEDED    (!MOTION_BY_IRQ)

#define ACC_I2C_ADDR            0x1C

// Registers addresses
#define ACC_REG_STATUS          0x00
#define ACC_INT_SRC             0x0C
#define ACC_REG_WHO_AM_I        0x0D
#define ACC_REG_XYZ_DATA_CFG    0x0E
#define ACC_FF_MT_CFG           0x15
#define ACC_FF_MT_SRC           0x16
#define ACC_FF_MT_THS           0x17
#define ACC_FF_MT_COUNT         0x18
#define ACC_REG_CONTROL1        0x2A
#define ACC_REG_CONTROL2        0x2B
#define ACC_REG_CONTROL3        0x2C
#define ACC_REG_CONTROL4        0x2D
#define ACC_REG_CONTROL5        0x2E

#if ACC_ACCELERATIONS_NEEDED
struct Accelerations_t {
    uint8_t Status;     // Read to reset latched data
    int16_t a[3];
} __packed;
#define ACCELERATIONS_SIZE     sizeof(Accelerations_t)
#endif

extern i2c_t Acc_i2c;

class Acc_t {
private:
    void IClearIrq() { // Dummy read
        uint8_t RegAddr = ACC_FF_MT_SRC, Dummy;
        Acc_i2c.WriteRead(ACC_I2C_ADDR, &RegAddr, 1, &Dummy, 1);
    }
    void IWriteReg(uint8_t AAddr, uint8_t AValue) {
        uint8_t RegAddr = AAddr, RegValue = AValue;
        uint8_t r = Acc_i2c.WriteWrite(ACC_I2C_ADDR, &RegAddr, 1, &RegValue, 1);
        if(r != retvOk) Printf("WriteReg: %u\r", r);
    }
    void IReadReg(uint8_t AAddr, uint8_t *PValue) {
        uint8_t r = Acc_i2c.WriteRead(ACC_I2C_ADDR, &AAddr, 1, PValue, 1);
        if(r != retvOk) Printf("ReadReg: %u\r", r);
    }
public:
#if ACC_ACCELERATIONS_NEEDED
    Accelerations_t Accelerations;
    void ReadAccelerations() {
        uint8_t RegAddr = ACC_REG_STATUS;
        Acc_i2c.WriteRead(ACC_I2C_ADDR, &RegAddr, 1, (uint8_t*)&Accelerations, ACCELERATIONS_SIZE);
        Accelerations.a[0] = __REVSH(Accelerations.a[0]);
        Accelerations.a[1] = __REVSH(Accelerations.a[1]);
        Accelerations.a[2] = __REVSH(Accelerations.a[2]);
    }
    int32_t ThresholdStable = 540;
#endif
    void Init();
    void Task();
};

extern Acc_t Acc;
