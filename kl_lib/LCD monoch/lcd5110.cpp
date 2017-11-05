#include "lcd5110.h"
#include "lcd_font.h"
#include "string.h"
#include <stdarg.h>
#include "uart.h"

#define LCD_DMA_TX_MODE     DMA_PRIORITY_LOW | \
                            STM32_DMA_CR_MSIZE_BYTE | \
                            STM32_DMA_CR_PSIZE_BYTE | \
                            STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                            STM32_DMA_CR_DIR_M2P |    /* Direction is memory to peripheral */ \
                            STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */

Lcd_t Lcd;
static Spi_t ISpi{LCD_SPI};

extern "C"
void DmaLcdTxIrq(void *p, uint32_t flags) { Lcd.IIrqHandler(); }


void Lcd_t::Init(void) {
    BckLt.Init();
    // ==== GPIOs ====
    // Configure LCD_XRES, LCD_XCS, LCD_SCLK & LCD_SDA as Push-Pull output
    PinSetupOut(LCD_RST_PIN, omPushPull);
    PinSetupOut(LCD_CE_PIN,  omPushPull);
    PinSetupOut(LCD_DC_PIN,  omPushPull);
    PinSetupOut(LCD_CLK_PIN, omPushPull);
    PinSetupOut(LCD_DIN_PIN,  omPushPull);
    BckLt.Init();
    // ========================= Init LCD ======================================
    CLK_Lo();
    CE_Hi();
    // Reset display
    RES_Lo();
    chThdSleepMilliseconds(7);
    RES_Hi();
    chThdSleepMilliseconds(7);
    // Initial commands
    WriteCmd(0x21); // extended instruction set control (H=1)
    WriteCmd(0x13); // bias system (1:48)
    WriteCmd(0xC2); // default Vop (3.06 + 66 * 0.06 = 7V)
    WriteCmd(0x20); // extended instruction set control (H=0)
    WriteCmd(0x09); // all display segments on
    chThdSleepMilliseconds(207);
    WriteCmd(0x08); // display blank
    WriteCmd(0x0C); // normal mode (0x0d = inverse mode)
    // Place the cursor at the origin
    WriteCmd(0x80);
    WriteCmd(0x40);

    // ======================== Switch to SPI + DMA ============================
    PinSetupAlterFunc(LCD_CLK_PIN, omPushPull, pudNone, AF0, psHigh);
    PinSetupAlterFunc(LCD_DIN_PIN, omPushPull, pudNone, AF0, psHigh);
    ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, sclkDiv4);
    ISpi.EnableTxDma();
    ISpi.Enable();

    // DMA
    dmaStreamAllocate     (LCD_DMA, IRQ_PRIO_LOW, DmaLcdTxIrq, nullptr);
    dmaStreamSetPeripheral(LCD_DMA, &LCD_SPI->DR);
    dmaStreamSetMemory0   (LCD_DMA, IBuf);
    dmaStreamSetTransactionSize(LCD_DMA, LCD_VIDEOBUF_SIZE);
    dmaStreamSetMode      (LCD_DMA, LCD_DMA_TX_MODE);
    chSemObjectInit(&semLcd, 1);
    Cls();
}

void Lcd_t::Shutdown(void) {
    dmaStreamDisable(LCD_DMA);
    RES_Lo();
    CE_Lo();
    CLK_Lo();
    Din_Lo();
    SetBacklight(0);
    chSemReset(&semLcd, 1);
}

void Lcd_t::Update() {
    msg_t msg = chSemWait(&semLcd);
    if(msg == MSG_OK) {
        dmaStreamDisable(LCD_DMA);
        dmaStreamSetTransactionSize(LCD_DMA, LCD_VIDEOBUF_SIZE);
        dmaStreamSetMode(LCD_DMA, LCD_DMA_TX_MODE);
        dmaStreamSetMemory0(LCD_DMA, IBuf);
        DC_Hi(); // Set "Data" line
        CE_Lo();
        dmaStreamEnable(LCD_DMA);
    }
}

void Lcd_t::IIrqHandler() {
    CE_Hi();
    dmaStreamDisable(LCD_DMA);
    chSysLockFromISR();
    chSemSignalI(&semLcd);
    chSysUnlockFromISR();
//    PrintfI("Irq");
}

void Lcd_t::WriteCmd(uint8_t AByte) {
    CLK_Lo();
    CE_Lo(); // Select chip
    DC_Lo(); // Set "Cmd" line
    // Send byte
    for(uint8_t i=0; i<8; i++) {
        if(AByte & 0x80) Din_Hi();
        else Din_Lo();
        CLK_Hi();
        CLK_Lo();
        AByte <<= 1;
    }
    CE_Hi();
}

// ================================= Printf ====================================
// Prints char at current buf indx
uint8_t Lcd_t::IPutChar(char c) {
    for(uint8_t i=0; i<6; i++) {
        uint8_t b = Font_6x8_Data[(uint32_t)c][i];
        if(IInverted) b = ~b;
        b = __RBIT(b) >> 24;    // Inverse bit order
        IBuf[CurrentPosition++] = b;
        if(CurrentPosition >= LCD_VIDEOBUF_SIZE) CurrentPosition = 0;
    }
    return retvOk;
}

void Lcd_t::Print(const uint8_t x, const uint8_t y, const char *S, ...) {
    IInverted = false;
    msg_t msg = chSemWait(&semLcd);
    if(msg == MSG_OK) {
        GotoCharXY(x, y);
        va_list args;
        va_start(args, S);
        IVsPrintf(S, args);
        va_end(args);
        chSemSignal(&semLcd);
    }
}

void Lcd_t::PrintInverted(const uint8_t x, const uint8_t y, const char *S, ...) {
    IInverted = true;
    msg_t msg = chSemWait(&semLcd);
    if(msg == MSG_OK) {
        GotoCharXY(x, y);
        va_list args;
        va_start(args, S);
        IVsPrintf(S, args);
        va_end(args);
        chSemSignal(&semLcd);
    }
}

// ================================ Graphics ===================================
static const uint8_t
setBit[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 },
clrBit[] = { 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F };

void Lcd_t::DrawPixel(const uint8_t x, const uint8_t y, Invert_t AInvert) {
    uint8_t yStr = y / 8;
    GotoXY(x, yStr);
    uint8_t Offset = y & 7;
    if(AInvert == Inverted) IBuf[CurrentPosition] |= setBit[Offset];
    else IBuf[CurrentPosition] &= clrBit[Offset];
}

void Lcd_t::DrawImage(const uint8_t x, const uint8_t y, const uint8_t* Img) {
    uint8_t *p = (uint8_t*)Img;
    uint8_t b;
    uint8_t Width = *p++, Height = *p++;
    for(uint8_t fy=y; fy < y+Height; fy++) {
        GotoXY(x, fy);
        for(uint8_t fx=x; fx < x+Width; fx++) {
            b = *p++;
            IBuf[CurrentPosition++] = b;
            if(CurrentPosition >= LCD_VIDEOBUF_SIZE) continue;
        } // fx
    } // fy
}

/* Composition of symbols must be terminated with '0'.
 * Example:
 * Lcd.Symbols(0, 4,
        LineHorizDouble, 7,
        LineHorizDoubleDown, 1,
        LineHorizDouble, 8,
        0);
        Lcd.Symbols(7, 6, LineVertDouble, 1, 0);
 */
void Lcd_t::Symbols(const uint8_t x, const uint8_t y, ...) {
    GotoCharXY(x, y);
    va_list Arg;
    va_start(Arg, y);    // Set pointer to last argument
    while(true) {
        uint8_t FCharCode = (uint8_t)va_arg(Arg, int32_t);
        if(FCharCode == 0) break;
        uint8_t RepeatCount = (uint8_t)va_arg(Arg, int32_t);
        for(uint8_t j=0; j<RepeatCount; j++) DrawChar(FCharCode, NotInverted);
    }
    va_end(Arg);
}

void Lcd_t::Cls() {
    memset(IBuf, 0, sizeof(IBuf));
    Update();
}

#if USE_LARGE_FONTS // ================== LargeFonts ======================
#define FNT_CHAR_BUF_SZ     9
struct {
    char Buf[FNT_CHAR_BUF_SZ];
    uint32_t Cnt;
} FntBuf;
static inline void FLcdPutFontChar(char c) { FntBuf.Buf[FntBuf.Cnt++] = c; }

void Lcd_t::PrintfFont(const uint8_t *PFont, uint8_t x, uint8_t y, const char *S, ...) {
    // Print to buf
    FntBuf.Cnt = 0;
    va_list args;
    va_start(args, S);
    kl_vsprintf(FLcdPutFontChar, FNT_CHAR_BUF_SZ, S, args);
    va_end(args);
    // Display what printed
    uint32_t height = PFont[2], MaxWidth = PFont[1];
    uint8_t FirstSymbolCode = PFont[0];
    for(uint32_t i=0; i<FntBuf.Cnt; i++) {
        char c = FntBuf.Buf[i];
        // ==== Draw char ====
        uint8_t *P = (uint8_t*)PFont + 3 + (c - FirstSymbolCode) * (MaxWidth*height + 1);  // Pointer to char
        uint32_t width = 1 + *P++;
        for(uint8_t i=0; i<width; i++) {
            for(uint8_t h=0; h<height; h++) {
                uint32_t Indx = x + i + (y + h) * 96;
                uint32_t dw = *P++;
                dw = __RBIT(dw);
                dw >>= 23;
                uint16_t w = dw | 0x0001;
                IBuf[Indx] = w;
            }
        }
        S++;
        x += width;
    }
}
#endif

