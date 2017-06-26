#include "acc_mma8452.h"
#include "MsgQ.h"

Acc_t Acc;

// Thread
static THD_WORKING_AREA(waAccThread, 128);
__noreturn
static void AccThread(void *arg) {
    chRegSetThreadName("Acc");
    while(true) Acc.Task();
}

void Acc_t::Task() {
    chThdSleepMilliseconds(108);
//    Printf("t\r");
    if(PinIsHi(ACC_IRQ_GPIO, ACC_IRQ_PIN)) {  // IRQ occured
//        Printf("Irq\r");
        IClearIrq();
        EvtMsg_t Msg(evtIdAcc);
        EvtQMain.SendNowOrExit(Msg);
    }
#ifdef ACC_ACCELERATIONS_NEEDED
    ReadAccelerations();
    Printf("X: %d; Y: %d; Z: %d\r", Accelerations.xMSB, Accelerations.yMSB, Accelerations.zMSB);
#endif
}

void Acc_t::Init() {
    // Init INT pin
    PinSetupInput(ACC_IRQ_GPIO, ACC_IRQ_PIN, pudPullDown);

    // Read WhoAmI
    uint8_t v = 0;
    IReadReg(ACC_REG_WHO_AM_I, &v);
    if(v != 0x2A) {
        Printf("Acc error: %X\r", v);
        return;
    }

    // ==== Setup initial registers ====
    // Put device to StandBy mode
    IWriteReg(ACC_REG_CONTROL1, 0b10100000);    // ODR = 50Hz, Standby
    // Setup High-Pass filter and acceleration scale
    IWriteReg(ACC_REG_XYZ_DATA_CFG, 0x01);      // No filter, scale = 4g
    // Setup Motion Detection
    IWriteReg(ACC_FF_MT_CFG, 0b11111000);       // Latch enable, detect motion, all three axes
    IWriteReg(ACC_FF_MT_THS, ACC_MOTION_TRESHOLD);  // Threshold = acceleration/0.063. "Detected" = (a > threshold)
    IWriteReg(ACC_FF_MT_COUNT, 0);              // Debounce counter: detect when moving longer than value*20ms (depends on ODR, see below)
    // Control registers
    IWriteReg(ACC_REG_CONTROL2, 0x00);          // Normal mode
    IWriteReg(ACC_REG_CONTROL3, 0b00001010);    // Freefall/motion may wake up system; IRQ output = active high, Push-Pull
    IWriteReg(ACC_REG_CONTROL4, 0b00000100);    // Freefall/motion IRQ enabled
    IWriteReg(ACC_REG_CONTROL5, 0b00000100);    // FreeFall/motion IRQ is routed to INT1 pin
    IWriteReg(ACC_REG_CONTROL1, 0b10100001);    // ASleep=10 => 6.75Hz; DR=100 => 50Hz output data rate (ODR); Mode = Active

    // Thread
    chThdCreateStatic(waAccThread, sizeof(waAccThread), NORMALPRIO, (tfunc_t)AccThread, NULL);
}
