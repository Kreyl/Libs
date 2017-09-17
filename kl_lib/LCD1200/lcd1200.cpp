#include "lcd1200.h"
#include "lcd_font.h"
#include "string.h"
#include <stdarg.h>
#include "uart.h"

#define LCD_DMA_TX_MODE     DMA_PRIORITY_LOW | \
                            STM32_DMA_CR_MSIZE_HWORD | \
                            STM32_DMA_CR_PSIZE_HWORD | \
                            STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                            STM32_DMA_CR_DIR_M2P |    /* Direction is memory to peripheral */ \
                            STM32_DMA_CR_CIRC

Lcd_t Lcd;

void Lcd_t::Init(void) {
    BckLt.Init();
    // ==== GPIOs ====
    // Configure LCD_XRES, LCD_XCS, LCD_SCLK & LCD_SDA as Push-Pull output
    PinSetupOut(LCD_XRES, omPushPull);
    PinSetupOut(LCD_XCS,  omPushPull);
    PinSetupOut(LCD_SCLK, omPushPull);
    PinSetupOut(LCD_SDA,  omPushPull);
    BckLt.Init();
    // ========================= Init LCD ======================================
    SCLK_Lo();
    XCS_Hi();
    // Reset display
    XRES_Lo();
    chThdSleepMilliseconds(7);
    XRES_Hi();
    WriteCmd(0xAF);    // display ON
    // Reset display again
    XRES_Lo();
    chThdSleepMilliseconds(7);
    XRES_Hi();
    chThdSleepMilliseconds(7);
    // Initial commands
    WriteCmd(0xAF);    // display ON
    WriteCmd(0xA4);    // Set normal display mode
    WriteCmd(0x2F);    // Charge pump on
    WriteCmd(0x40);    // Set start row address = 0

#if LCD_MIRROR_Y_AXIS
    WriteCmd(0xC8);    // Mirror Y axis
#endif
#if LCD_MIRROR_X_AXIS
    WriteCmd(0xA1);    // Mirror X axis
#endif
    // Set x=0, y=0
    WriteCmd(0xB0);    // Y axis initialization
    WriteCmd(0x10);    // X axis initialisation1
    WriteCmd(0x00);    // X axis initialisation2
    Cls();             // clear LCD buffer

    // ====================== Switch to USART + DMA ============================
#ifdef STM32F072xB
    PinSetupAlterFunc(LCD_SCLK, omPushPull, pudNone, AF4, psHigh);
    PinSetupAlterFunc(LCD_SDA,  omPushPull, pudNone, AF4, psHigh);
    // ==== USART init ==== clock enabled, idle low, first edge, enable last bit pulse
    rccEnableUSART3(FALSE);
    USART3->BRR = Clk.APBFreqHz / LCD_UART_SPEED;
    USART3->CR2 = USART_CR2_CLKEN | USART_CR2_LBCL; // Enable clock, enable last bit clock
    USART3->CR1 = USART_CR1_M0 | USART_CR1_TE;
    USART3->CR3 = USART_CR3_DMAT;   // Enable DMA at transmitter
    USART3->CR1 |= USART_CR1_UE;     // Enable
#else
    PinSetupAlterFunc(LCD_SCLK, omPushPull, pudNone, AF7, psHigh);
    PinSetupAlterFunc(LCD_SDA,  omPushPull, pudNone, AF7, psHigh);
    // ==== USART init ==== clock enabled, idle low, first edge, enable last bit pulse
    rccEnableUSART3(FALSE);
    USART3->CR1 = USART_CR1_UE;     // Enable
    USART3->BRR = Clk.APBFreqHz / LCD_UART_SPEED;
    USART3->CR2 = USART_CR2_CLKEN | USART_CR2_LBCL; // Enable clock, enable last bit clock
    USART3->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_TE;
    USART3->CR3 = USART_CR3_DMAT;   // Enable DMA at transmitter
#endif
    // DMA
    dmaStreamAllocate     (LCD_DMA, IRQ_PRIO_LOW, nullptr, NULL);
    dmaStreamSetPeripheral(LCD_DMA, &USART3->TDR);
    dmaStreamSetMemory0   (LCD_DMA, IBuf);
    dmaStreamSetTransactionSize(LCD_DMA, LCD_VIDEOBUF_SIZE);
    dmaStreamSetMode      (LCD_DMA, LCD_DMA_TX_MODE);
    // Start transmission
    XCS_Lo();
    dmaStreamEnable(LCD_DMA);
    chSemObjectInit(&semLcd, 1);
}

void Lcd_t::Shutdown(void) {
    dmaStreamDisable(LCD_DMA);
    XRES_Lo();
    XCS_Lo();
    SCLK_Lo();
    SDA_Lo();
    Backlight(0);
    chSemReset(&semLcd, 1);
}

void Lcd_t::WriteCmd(uint8_t AByte) {
    SCLK_Lo();
    XCS_Lo();   // Select chip
    // Send "Cmd" bit
    SDA_Lo();
    SCLK_Hi();
    SCLK_Lo();
    // Send byte
    for(uint8_t i=0; i<8; i++) {
        if(AByte & 0x80) SDA_Hi();
        else SDA_Lo();
        SCLK_Hi();
        SCLK_Lo();
        AByte <<= 1;
    }
    XCS_Hi();
}

// ================================= Printf ====================================
// Prints char at current buf indx
uint8_t Lcd_t::IPutChar(char c) {
    for(uint8_t i=0; i<6; i++) {
        uint8_t b = Font_6x8_Data[(uint32_t)c][i];
        if(Inverted) b = ~b;
        uint16_t w = b;
        w = (w << 1) | 0x0001;
        IBuf[CurrentPosition++] = w;
        if(CurrentPosition >= LCD_VIDEOBUF_SIZE) CurrentPosition = 0;
    }
    return retvOk;
}

void Lcd_t::Printf(const uint8_t x, const uint8_t y, const char *S, ...) {
    Inverted = false;
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

void Lcd_t::PrintfInverted(const uint8_t x, const uint8_t y, const char *S, ...) {
    Inverted = true;
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
void Lcd_t::DrawImage(const uint8_t x, const uint8_t y, const uint8_t* Img) {
    uint8_t *p = (uint8_t*)Img;
    uint16_t w;
    uint8_t Width = *p++, Height = *p++;
    for(uint8_t fy=y; fy < y+Height; fy++) {
        GotoXY(x, fy);
        for(uint8_t fx=x; fx < x+Width; fx++) {
            w = *p++;
            w = (w << 1) | 0x0001;
            IBuf[CurrentPosition++] = w;
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
    for (uint32_t i=0; i < LCD_VIDEOBUF_SIZE; i++) IBuf[i] = 0x0001;
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

