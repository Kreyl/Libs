/*
 * led_rgb.h
 *
 *  Created on: 31 рту. 2014 у.
 *      Author: Kreyl
 */

#pragma once

#include "hal.h"
#include "color.h"
#include "ChunkTypes.h"
#include "uart.h"
#include "kl_lib.h"

#if 1 // ==================== LED on/off, no sequences =========================
class LedOnOff_t {
protected:
    PinOutput_t IChnl;
public:
    LedOnOff_t(GPIO_TypeDef *APGPIO, uint16_t APin, PinOutMode_t AOutputType) :
        IChnl(APGPIO, APin, AOutputType) {}
    void Init() { IChnl.Init(); Off(); }
    void On()  { IChnl.SetHi(); }
    void Off() { IChnl.SetLo(); }
};
#endif

#if 1 // ========================= Simple LED blinker ==========================
class LedBlinker_t : public BaseSequencer_t<BaseChunk_t>, public LedOnOff_t {
protected:
    void ISwitchOff() { Off(); }
    SequencerLoopTask_t ISetup() {
        IChnl.Set(IPCurrentChunk->Value);
        IPCurrentChunk++;   // Always increase
        return sltProceed;  // Always proceed
    }
public:
    LedBlinker_t(GPIO_TypeDef *APGPIO, uint16_t APin, PinOutMode_t AOutputType) :
        BaseSequencer_t(), LedOnOff_t(APGPIO, APin, AOutputType) {}
};
#endif

#if 1 // ======================== Single Led Smooth ============================
class LedSmooth_t : public BaseSequencer_t<LedSmoothChunk_t> {
private:
    const PinOutputPWM_t IChnl;
    uint8_t ICurrentBrightness;
//    void SetupDelay(uint32_t ms) { chVTSetI(&ITmr, MS2ST(ms), LedSmoothTmrCallback, this); }
    void ISwitchOff() { SetBrightness(0); }
    SequencerLoopTask_t ISetup() {
        if(ICurrentBrightness != IPCurrentChunk->Brightness) {
            if(IPCurrentChunk->Value == 0) {     // If smooth time is zero,
                SetBrightness(IPCurrentChunk->Brightness); // set color now,
                ICurrentBrightness = IPCurrentChunk->Brightness;
                IPCurrentChunk++;                // and goto next chunk
            }
            else {
                if     (ICurrentBrightness < IPCurrentChunk->Brightness) ICurrentBrightness++;
                else if(ICurrentBrightness > IPCurrentChunk->Brightness) ICurrentBrightness--;
                SetBrightness(ICurrentBrightness);
                // Check if completed now
                if(ICurrentBrightness == IPCurrentChunk->Brightness) IPCurrentChunk++;
                else { // Not completed
                    // Calculate time to next adjustment
                    uint32_t Delay = ClrCalcDelay(ICurrentBrightness, IPCurrentChunk->Value);
                    SetupDelay(Delay);
                    return sltBreak;
                } // Not completed
            } // if time > 256
        } // if color is different
        else IPCurrentChunk++; // Color is the same, goto next chunk
        return sltProceed;
    }
public:
    LedSmooth_t(const PwmSetup_t APinSetup) :
        BaseSequencer_t(), IChnl(APinSetup), ICurrentBrightness(0) {}
    void Init() {
        IChnl.Init();
        SetBrightness(0);
    }
    void SetBrightness(uint8_t ABrightness) { IChnl.Set(ABrightness); }
};
#endif


#if 0 // ==================== RGB blinker (no smooth switch) ===================
#define LED_RGB_BLINKER
class LedRgbBlinker_t : public BaseSequencer_t<LedRGBChunk_t> {
protected:
    PinOutputPushPull_t R, G, B;
    void ISwitchOff() { SetColor(clBlack); }
    SequencerLoopTask_t ISetup() {
        SetColor(IPCurrentChunk->Color);
        IPCurrentChunk++;   // Always increase
        return sltProceed;  // Always proceed
    }
public:
    LedRgbBlinker_t(const PinOutputPushPull_t ARed, const PinOutputPushPull_t AGreen, const PinOutputPushPull_t ABlue) :
        BaseSequencer_t(), R(ARed), G(AGreen), B(ABlue) {}
    void Init() {
        R.Init();
        G.Init();
        B.Init();
        SetColor(clBlack);
    }
    void SetColor(Color_t AColor) {
        R.Set(AColor.R);
        G.Set(AColor.G);
        B.Set(AColor.B);
    }
};
#endif

#if 1 // =========================== LedRGB Parent =============================
class LedRGBParent_t : public BaseSequencer_t<LedRGBChunk_t> {
protected:
    const PinOutputPWM_t  R, G, B;
    const uint32_t PWMFreq;
    Color_t ICurrColor;
    void ISwitchOff() {
        SetColor(clBlack);
        ICurrColor = clBlack;
    }
    SequencerLoopTask_t ISetup() {
        if(ICurrColor != IPCurrentChunk->Color) {
            if(IPCurrentChunk->Value == 0) {     // If smooth time is zero,
                SetColor(IPCurrentChunk->Color); // set color now,
                ICurrColor = IPCurrentChunk->Color;
                IPCurrentChunk++;                // and goto next chunk
            }
            else {
                ICurrColor.Adjust(IPCurrentChunk->Color);
                SetColor(ICurrColor);
                // Check if completed now
                if(ICurrColor == IPCurrentChunk->Color) IPCurrentChunk++;
                else { // Not completed
                    // Calculate time to next adjustment
                    uint32_t Delay = ICurrColor.DelayToNextAdj(IPCurrentChunk->Color, IPCurrentChunk->Value);
                    SetupDelay(Delay);
                    return sltBreak;
                } // Not completed
            } // if time > 256
        } // if color is different
        else IPCurrentChunk++; // Color is the same, goto next chunk
        return sltProceed;
    }
public:
    LedRGBParent_t(
            const PwmSetup_t ARed,
            const PwmSetup_t AGreen,
            const PwmSetup_t ABlue,
            const uint32_t APWMFreq) :
        BaseSequencer_t(), R(ARed), G(AGreen), B(ABlue), PWMFreq(APWMFreq) {}
    void Init() {
        R.Init();
        R.SetFrequencyHz(PWMFreq);
        G.Init();
        G.SetFrequencyHz(PWMFreq);
        B.Init();
        B.SetFrequencyHz(PWMFreq);
        SetColor(clBlack);
    }
    bool IsOff() { return (ICurrColor == clBlack) and IsIdle(); }
    virtual void SetColor(Color_t AColor) {}
};
#endif

#if 1 // ============================== LedRGB =================================
class LedRGB_t : public LedRGBParent_t {
public:
    LedRGB_t(
            const PwmSetup_t ARed,
            const PwmSetup_t AGreen,
            const PwmSetup_t ABlue,
            const uint32_t AFreq = 0xFFFFFFFF) :
                LedRGBParent_t(ARed, AGreen, ABlue, AFreq) {}

    void SetColor(Color_t AColor) {
        R.Set(AColor.R);
        G.Set(AColor.G);
        B.Set(AColor.B);
    }
};
#endif

#if 1 // =========================== RGB LED with power ========================
class LedRGBwPower_t : public LedRGBParent_t {
private:
    const PinOutput_t PwrPin;
public:
    LedRGBwPower_t(
            const PwmSetup_t ARed,
            const PwmSetup_t AGreen,
            const PwmSetup_t ABlue,
            const PinOutput_t APwrPin,
            const uint32_t AFreq = 0xFFFFFFFF) :
                LedRGBParent_t(ARed, AGreen, ABlue, AFreq), PwrPin(APwrPin) {}
    void Init() {
        PwrPin.Init();
        LedRGBParent_t::Init();
    }
    void SetColor(Color_t AColor) {
        if(AColor == clBlack) PwrPin.SetLo();
        else PwrPin.SetHi();
        R.Set(AColor.R);
        G.Set(AColor.G);
        B.Set(AColor.B);
    }
};
#endif

#if 1 // ====================== LedRGB with Luminocity =========================
class LedRGBLum_t : public LedRGBParent_t {
public:
    LedRGBLum_t(
            const PwmSetup_t ARed,
            const PwmSetup_t AGreen,
            const PwmSetup_t ABlue,
            const uint32_t AFreq = 0xFFFFFFFF) :
                LedRGBParent_t(ARed, AGreen, ABlue, AFreq) {}

    void SetColor(Color_t AColor) {
        R.Set(AColor.R * AColor.Lum);
        G.Set(AColor.G * AColor.Lum);
        B.Set(AColor.B * AColor.Lum);
    }
};
#endif
