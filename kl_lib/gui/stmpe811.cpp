/*
 * stmpe811.cpp
 *
 *  Created on: 14 ??? 2016 ?.
 *      Author: Kreyl
 */

#include "ch.h"
#include "stmpe811.h"
#include "i2cL476.h"
#include "uart.h"
#include "math.h"

#if 1 // ============================ Definitions ==============================
// Identification registers
#define STMPE811_CHIP_ID                0x00    //STMPE811 Device identification
#define STMPE811_ID_VER                 0x02    //STMPE811 Revision number; 0x01 for engineering sample; 0x03 for final silicon
#define STMPE811_SYS_CTRL1              0x03    //Reset control
#define STMPE811_SYS_CTRL2              0x04    //Clock control
#define STMPE811_SPI_CFG                0x08    //SPI interface configuration
#define STMPE811_INT_CTRL               0x09    //Interrupt control register
#define STMPE811_INT_EN                 0x0A    //Interrupt enable register
#define STMPE811_INT_STA                0x0B    //Interrupt status register
#define STMPE811_GPIO_EN                0x0C    //GPIO interrupt enable register
#define STMPE811_GPIO_INT_STA           0x0D    //GPIO interrupt status register
#define STMPE811_ADC_INT_EN             0x0E    //ADC interrupt enable register
#define STMPE811_ADC_INT_STA            0x0F    //ADC interface status register
#define STMPE811_GPIO_SET_PIN           0x10    //GPIO set pin register
#define STMPE811_GPIO_CLR_PIN           0x11    //GPIO clear pin register
#define STMPE811_MP_STA                 0x12    //GPIO monitor pin state register
#define STMPE811_GPIO_DIR               0x13    //GPIO direction register
#define STMPE811_GPIO_ED                0x14    //GPIO edge detect register
#define STMPE811_GPIO_RE                0x15    //GPIO rising edge register
#define STMPE811_GPIO_FE                0x16    //GPIO falling edge register
#define STMPE811_GPIO_AF                0x17    //alternate function register
#define STMPE811_ADC_CTRL1              0x20    //ADC control
#define STMPE811_ADC_CTRL2              0x21    //ADC control
#define STMPE811_ADC_CAPT               0x22    //To initiate ADC data acquisition
#define STMPE811_ADC_DATA_CHO           0x30    //ADC channel 0
#define STMPE811_ADC_DATA_CH1           0x32    //ADC channel 1
#define STMPE811_ADC_DATA_CH2           0x34    //ADC channel 2
#define STMPE811_ADC_DATA_CH3           0x36    //ADC channel 3
#define STMPE811_ADC_DATA_CH4           0x38    //ADC channel 4
#define STMPE811_ADC_DATA_CH5           0x3A    //ADC channel 5
#define STMPE811_ADC_DATA_CH6           0x3C    //ADC channel 6
#define STMPE811_ADC_DATA_CH7           0x3E    //ADC channel 7
#define STMPE811_TSC_CTRL               0x40    //4-wire touchscreen controller setup
#define STMPE811_TSC_CFG                0x41    //Touchscreen controller configuration
#define STMPE811_WDW_TR_X               0x42    //Window setup for top right X
#define STMPE811_WDW_TR_Y               0x44    //Window setup for top right Y
#define STMPE811_WDW_BL_X               0x46    //Window setup for bottom left X
#define STMPE811_WDW_BL_Y               0x48    //Window setup for bottom left Y
#define STMPE811_FIFO_TH                0x4A    //FIFO level to generate interrupt
#define STMPE811_FIFO_STA               0x4B    //Current status of FIFO
#define STMPE811_FIFO_SIZE              0x4C    //Current filled level of FIFO
#define STMPE811_TSC_DATA_X             0x4D    //Data port for touchscreen controller data access
#define STMPE811_TSC_DATA_Y             0x4F    //Data port for touchscreen controller data access
#define STMPE811_TSC_DATA_Z             0x51    //Data port for touchscreen controller data access
#define STMPE811_TSC_DATA_XYZ           0x52    //Data port for touchscreen controller data access
#define STMPE811_TSC_FRACTION_Z         0x56    //Touchscreen controller FRACTION_Z
#define STMPE811_TSC_DATA               0x57    //Data port for touchscreen controller data access
#define STMPE811_TSC_I_DRIVE            0x58    //Touchscreen controller drivel
#define STMPE811_TSC_SHIELD             0x59    //Touchscreen controller shield
#define STMPE811_TEMP_CTRL              0x60    //Temperature sensor setup
#define STMPE811_TEMP_DATA              0x61    //Temperature data access port
#define STMPE811_TEMP_TH                0x62    //Threshold for temperature controlled interrupt

#endif

STMPE811_t Touch;

void STMPE811_t::Init() {
    // IRQ pin
    PinSetupInput(TOUCH_INT_GPIO, TOUCH_INT_PIN, pudPullDown);
    // Reset
    if(Write(STMPE811_SYS_CTRL1, 0x02) != OK) return;
    chThdSleepMilliseconds(5);
    Write(STMPE811_SYS_CTRL1, 0x00);
    chThdSleepMilliseconds(2);
//    uint16_t Data;
//    if(Read(STMPE811_CHIP_ID, &Data) != OK) return;
//    Uart.Printf("tch: %X\r", Data);
    // Temperature sensor clock off, GPIO clock off, touch clock on, ADC clock on
    Write(STMPE811_SYS_CTRL2, 0x0C);
    Write(STMPE811_INT_EN, 0x00);       // Interrupts disable
    // ADC conversion time = 80 clock ticks, 12-bit ADC, internal voltage refernce
    Write(STMPE811_ADC_CTRL1, 0x49);
    chThdSleepMilliseconds(2);
    Write(STMPE811_ADC_CTRL2, 0x01);    // Select the ADC clock speed: 3.25 MHz
    Write(STMPE811_GPIO_AF, 0x00);      // GPIO alternate function - OFF
    // Averaging 4, touch detect delay 1ms, panel driver settling time 1ms
    Write(STMPE811_TSC_CFG, 0xA3);
    Write(STMPE811_FIFO_TH, 0x01);      // Configure the Touch FIFO threshold: single point reading
    Write(STMPE811_FIFO_STA, 0x01);     // Clear FIFO
    Write(STMPE811_FIFO_STA, 0x00);     // Put the FIFO back into operation mode
    Write(STMPE811_TSC_FRACTION_Z, 0x07);   // Z axis data format: 7 fractional, 1 whole
    Write(STMPE811_TSC_I_DRIVE, 0x01);  // max 50mA touchscreen line current
//    Write(STMPE811_TSC_CTRL, 0x01);     // X&Y&Z, no tracking, Enable
    Write(STMPE811_TSC_CTRL, 0x03);     // X&Y, no tracking, Enable
    Write(STMPE811_INT_STA, 0xFF);      // Clear all interrupts
    Write(STMPE811_INT_CTRL, 0x05);     // Active high, Level interrupt, enable interrupts
//    Write(STMPE811_INT_EN, 0x01);       // IRQ: touch_det en
    Write(STMPE811_INT_EN, 0x02);       // IRQ: FIFO th
}

uint8_t STMPE811_t::ReadData() {
    Write(STMPE811_INT_STA, 0xFF);      // Clear all interrupts
//    uint8_t b;
//    Read(STMPE811_FIFO_SIZE, &b);
//    if(b == 0) return EMPTY;
//        Uart.Printf("tch: %X\r", b);
    // Read coords
    int16_t FX = 0, FY = 0;
    uint8_t Data[3];
    if(Read(STMPE811_TSC_DATA_XYZ, Data, 3) != OK) return FAILURE;
    FX = (((uint16_t)Data[0]) << 4) | (((uint16_t)Data[1]) >> 4);
    FY = (((uint16_t)Data[1] & 0x0F) << 8) | ((uint16_t)Data[2]);
    // Clear FIFO
    Write(STMPE811_FIFO_STA, 0x01);     // Clear FIFO
    Write(STMPE811_FIFO_STA, 0x00);     // Put the FIFO back into operation mode
    // Calculate calibrated values
    chSysLock();
    X = lroundf(CLBR_X(FY));
    Y = lroundf(CLBR_Y(FX));
    chSysUnlock();
//        Uart.Printf("tch: %d %d; %d %d\r", FX, FY, X, Y);
//        Uart.Printf("tch: %d %d %d\r", FX, FY, Data[3]);
//        Uart.Printf("tch: %d %d %X\r", FX, FY, Data[0] & 0x80);
    return NEW;
}

void STMPE811_t::DiscardData() {
    Write(STMPE811_INT_STA, 0xFF);      // Clear all interrupts
    Write(STMPE811_FIFO_STA, 0x01);     // Clear FIFO
    Write(STMPE811_FIFO_STA, 0x00);     // Put the FIFO back into operation mode
}

uint8_t STMPE811_t::Write(uint8_t Addr, uint8_t Data) {
    return I2C_TOUCH.WriteWrite(STMPE811_I2C_ADDR, &Addr, 1, &Data, 1);
}
uint8_t STMPE811_t::Read(uint8_t Addr, uint8_t *PData) {
    return I2C_TOUCH.WriteRead(STMPE811_I2C_ADDR, &Addr, 1, PData, 1);
}
uint8_t STMPE811_t::Read(uint8_t Addr, uint8_t *PData, uint32_t Len) {
    return I2C_TOUCH.WriteRead(STMPE811_I2C_ADDR, &Addr, 1, PData, Len);
}
uint8_t STMPE811_t::Read(uint8_t Addr, uint16_t *PData) {
    uint8_t r = I2C_TOUCH.WriteRead(STMPE811_I2C_ADDR, &Addr, 1, (uint8_t*)PData, 2);
    *PData = __REV16(*PData);
    return r;
}

bool STMPE811_t::IsTouched() {
    return PinIsHi(TOUCH_INT_GPIO, TOUCH_INT_PIN);
}
