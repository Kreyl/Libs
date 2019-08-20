#include "acc_mma8452.h"
#include "MsgQ.h"

Acc_t Acc;

#if !MOTION_BY_IRQ
#define BUF_SZ          20

class a_t {
public:
    int32_t a[3];
    void Set(int32_t AValue) { a[0] = AValue; a[1] = AValue; a[2] = AValue; }
    a_t() { a[0] = 0; a[1] = 0; a[2] = 0; }
    a_t(int32_t AValue) { a[0] = AValue; a[1] = AValue; a[2] = AValue; }
    a_t& operator = (const a_t &Right) {
        a[0] = Right.a[0];
        a[1] = Right.a[1];
        a[2] = Right.a[2];
        return *this;
    }
};

static a_t IArr[BUF_SZ];
static bool IsStable = true;
#endif

// Thread
static THD_WORKING_AREA(waAccThread, 512);
__noreturn
static void AccThread(void *arg) {
    chRegSetThreadName("Acc");
    while(true) Acc.Task();
}

void Acc_t::Task() {
    chThdSleepMilliseconds(108);
//    Printf("t\r");
#if MOTION_BY_IRQ
    if(PinIsHi(ACC_IRQ_GPIO, ACC_IRQ_PIN)) {  // IRQ occured
        Printf("Motion\r");
        IClearIrq();
        EvtQMain.SendNowOrExit(EvtMsg_t(evtIdAcc));
    }
#else
    ReadAccelerations();

    // Shift buffer and find Min and Max
    a_t Min(2000000000), Max(-2000000000);
    for(int i=(BUF_SZ-1); i>=1; i--) {
        IArr[i] = IArr[i-1];
        if(IArr[i].a[0] < Min.a[0]) Min.a[0] = IArr[i].a[0];
        if(IArr[i].a[1] < Min.a[1]) Min.a[1] = IArr[i].a[1];
        if(IArr[i].a[2] < Min.a[2]) Min.a[2] = IArr[i].a[2];
        if(IArr[i].a[0] > Max.a[0]) Max.a[0] = IArr[i].a[0];
        if(IArr[i].a[1] > Max.a[1]) Max.a[1] = IArr[i].a[1];
        if(IArr[i].a[2] > Max.a[2]) Max.a[2] = IArr[i].a[2];
    } // for
    // Add new value
    IArr[0].a[0] = Accelerations.a[0];
    IArr[0].a[1] = Accelerations.a[1];
    IArr[0].a[2] = Accelerations.a[2];

    // Check if is stable
    if(
            (Max.a[0] - Min.a[0]) < ThresholdStable and
            (Max.a[1] - Min.a[1]) < ThresholdStable and
            (Max.a[2] - Min.a[2]) < ThresholdStable) {
        // Stable
//        Printf("%d; %d; %d; Stable\r", (Max.a[0] - Min.a[0]), (Max.a[1] - Min.a[1]), (Max.a[2] - Min.a[2]));
        if(!IsStable) {
            IsStable = true;
            Printf("Stable\r");
            EvtQMain.SendNowOrExit(EvtMsg_t(evtIdStable));
        }
    }
    else { // Not stable
//        Printf("%d; %d; %d\r", (Max.a[0] - Min.a[0]), (Max.a[1] - Min.a[1]), (Max.a[2] - Min.a[2]));
        if(IsStable) {
            IsStable = false;
            Printf("Motion\r");
            EvtQMain.SendNowOrExit(EvtMsg_t(evtIdMotion));
        }
    }
#endif
}

void Acc_t::Init() {
#if MOTION_BY_IRQ
    // Init INT pin
    PinSetupInput(ACC_IRQ_GPIO, ACC_IRQ_PIN, pudPullDown);
#endif

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
#if MOTION_BY_IRQ // Setup Motion Detection
    IWriteReg(ACC_FF_MT_CFG, 0b11111000);       // Latch enable, detect motion, all three axes
    IWriteReg(ACC_FF_MT_THS, ACC_MOTION_TRESHOLD);  // Threshold = acceleration/0.063. "Detected" = (a > threshold)
    IWriteReg(ACC_FF_MT_COUNT, 0);              // Debounce counter: detect when moving longer than value*20ms (depends on ODR, see below)
#endif
    // Control registers
    IWriteReg(ACC_REG_CONTROL2, 0x00);          // Normal mode
#if MOTION_BY_IRQ
    IWriteReg(ACC_REG_CONTROL3, 0b00001010);    // Freefall/motion may wake up system; IRQ output = active high, Push-Pull
    IWriteReg(ACC_REG_CONTROL4, 0b00000100);    // Freefall/motion IRQ enabled
#else
    IWriteReg(ACC_REG_CONTROL3, 0b00000010);    // IRQ output = active high, Push-Pull
    IWriteReg(ACC_REG_CONTROL4, 0b00000000);    // IRQ disabled
#endif
    IWriteReg(ACC_REG_CONTROL5, 0b00000100);    // FreeFall/motion IRQ is routed to INT1 pin
    IWriteReg(ACC_REG_CONTROL1, 0b10100001);    // DR=100 => 50Hz output data rate (ODR); Mode = Active

#if !MOTION_BY_IRQ
    // Init average
    for(int i=0; i<BUF_SZ; i++) IArr[i].Set(1008);
#endif

    // Thread
    chThdCreateStatic(waAccThread, sizeof(waAccThread), NORMALPRIO, (tfunc_t)AccThread, NULL);
}
