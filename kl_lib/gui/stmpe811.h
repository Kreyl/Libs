/*
 * stmpe811.h
 *
 *  Created on: 14 ??? 2016 ?.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "board.h"

#define STMPE811_I2C_ADDR   0x41

class STMPE811_t {
private:
    uint8_t Write(uint8_t Addr, uint8_t Data);
    uint8_t Read(uint8_t Addr, uint8_t *PData);
    uint8_t Read(uint8_t Addr, uint8_t *PData, uint32_t Len);
    uint8_t Read(uint8_t Addr, uint16_t *PData);
public:
    void Init();
    bool IsTouched();
    uint8_t ReadData();
    void DiscardData();
    int32_t X, Y;
};

extern STMPE811_t Touch;

// Calibration
#define CLBR_X_LEFT     3760
#define CLBR_X_RIGHT    210
#define CLBR_Y_TOP      240
#define CLBR_Y_BOTTOM   3840
// Common formulas
#define CLBR_DIV
#define CLBR_A(in0, in1, out0, out1)    (((out1) - (out0)) / ((in1) - (in0)))
#define CLBR_B(in0, in1, out0, out1)    ((out0) - ((in0) * ((out1) - (out0))) / ((in1) - (in0)))
// Display formulas
#define CLBR_AX     CLBR_A((float)CLBR_X_LEFT, (float)CLBR_X_RIGHT,  0, (float)LCD_W)
#define CLBR_BX     CLBR_B((float)CLBR_X_LEFT, (float)CLBR_X_RIGHT,  0, (float)LCD_W)
#define CLBR_AY     CLBR_A((float)CLBR_Y_TOP,  (float)CLBR_Y_BOTTOM, 0, (float)LCD_H)
#define CLBR_BY     CLBR_B((float)CLBR_Y_TOP,  (float)CLBR_Y_BOTTOM, 0, (float)LCD_H)

#define CLBR_X(x)   (CLBR_AX * ((float)x) + CLBR_BX)
#define CLBR_Y(y)   (CLBR_AY * ((float)y) + CLBR_BY)
