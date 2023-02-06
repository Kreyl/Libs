/*
 * color.h
 *
 *  Created on: 05 ���. 2014 �.
 *      Author: Kreyl
 */

#ifndef COLOR_H_
#define COLOR_H_

#include "kl_lib.h"
#include "inttypes.h"
#include <sys/cdefs.h>
#include <stdlib.h> // for random

void Printf(const char *format, ...);

struct ColorHSV_t;

// In Settings, if Color.Brt == RANDOM_CLR_BRT then color should be random.
// The check and transmutation shall be made in upper level.
#define RANDOM_CLR_BRT      255

static const uint8_t gamma8[] = {
    0,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255
};

// Mixing two colors
//#define ClrMix(Fore, Back, Weight)     ((Fore * Weight + Back * (255 - Weight)) / 255)


__attribute__((__always_inline__))
static inline int32_t Abs32(int32_t w) { return (w < 0)? -w : w; }

// Smooth delay
static inline uint32_t ClrCalcDelay(uint16_t AValue, uint32_t Smooth) {
    return (uint32_t)((Smooth / (AValue+4)) + 1);
}

// Calculate Smooth value from desired duration of switching
static inline int32_t CalcSmooth_st_from_ms(int32_t Duration_ms) {
    return (TIME_MS2I(Duration_ms) * 10L) / 36L;
}

class Color_t {
private:
    __always_inline uint8_t  SetSingleBrt(int32_t v, const int32_t Brt, const int32_t BrtMax) { return (v * Brt) / BrtMax; }
    __always_inline uint32_t MixSingleAvg(uint32_t Fore, uint32_t Back) { return (Fore + Back) / 2; }
    __always_inline uint32_t MixSingleAdd(uint32_t Fore, uint32_t Back) {
        uint32_t Sum = Fore + Back;
        return Sum > 255? 255 : Sum;
    }
public:
    union {
        uint32_t DWord32;
        struct {
            uint8_t R, G, B;
            union {
                uint8_t Brt, W;
            };
        } __attribute__((packed));
    };
    bool operator == (const Color_t &AColor) const { return (DWord32 == AColor.DWord32); }
    bool operator != (const Color_t &AColor) const { return (DWord32 != AColor.DWord32); }
    Color_t& operator = (const Color_t &Right) { DWord32 = Right.DWord32; return *this; }

    void ApplyGammaCorrectionRGB() {
        R = gamma8[R];
        G = gamma8[G];
        B = gamma8[B];
    }

    void ApplyGammaCorrectionRGBW() {
        R = gamma8[R];
        G = gamma8[G];
        B = gamma8[B];
        W = gamma8[W];
    }

    void Adjust(const Color_t &PColor) {
        if     (R < PColor.R) R++;
        else if(R > PColor.R) R--;
        if     (G < PColor.G) G++;
        else if(G > PColor.G) G--;
        if     (B < PColor.B) B++;
        else if(B > PColor.B) B--;
        if     (Brt < PColor.Brt) Brt++;
        else if(Brt > PColor.Brt) Brt--;
    }

    void Adjust(const Color_t &PColor, uint32_t Step, const int32_t BrtMax) {
        uint32_t ThrsR = 255 - Step;
        if(R < PColor.R) {
            if(R <= ThrsR) R += Step;
            else R = 255;
        }
        else if(R > PColor.R) {
            if(R >= Step) R -= Step;
            else R = 0;
        }

        if(G < PColor.G) {
            if(G <= ThrsR) G += Step;
            else G = 255;
        }
        else if(G > PColor.G) {
            if(G >= Step) G -= Step;
            else G = 0;
        }

        if(B < PColor.B) {
            if(B <= ThrsR) B += Step;
            else B = 255;
        }
        else if(B > PColor.B) {
            if(B >= Step) B -= Step;
            else B = 0;
        }

        if(Brt < PColor.Brt) {
            Brt += Step;
            if(Brt > BrtMax) Brt = BrtMax;
        }
        else if(Brt > PColor.Brt) {
            if(Brt >= Step) Brt -= Step;
            else Brt = 0;
        }
    }
    void FromRGB(uint8_t Red, uint8_t Green, uint8_t Blue) { R = Red; G = Green; B = Blue; }
    void ToRGB(uint8_t *PR, uint8_t *PG, uint8_t *PB) const { *PR = R; *PG = G; *PB = B; }
    bool IsEqualRGB(uint8_t Red, uint8_t Green, uint8_t Blue) { return (R == Red and G == Green and B == Blue); }

    // H: 0...360, S: 0...100, V: 0...100
    void FromHSV(uint16_t H, uint8_t S, uint8_t V) {
        // Calc chroma: 0...255
        int32_t C = ((int32_t)V * (int32_t)S * 255) / 10000;
        // Tmp values
        int32_t X = 60 - Abs32((H % 120) - 60); // 0...60
        X = (C * X) / 60;
        int32_t m = (((int32_t)V * 255) / 100) - C; // To add the same amount to each component, to match lightness
        // RGB
        if     (H < 60)  { R = C+m; G = X+m; B = m;   } // [0; 60)
        else if(H < 120) { R = X+m; G = C+m; B = m;   }
        else if(H < 180) { R = m;   G = C+m; B = X+m; }
        else if(H < 240) { R = m;   G = X+m; B = C+m; }
        else if(H < 300) { R = X+m; G = m;   B = C+m; }
        else             { R = C+m; G = m;   B = X+m; } // [300; 360]
    }

    bool IsRandom() const { return (R == 0 and G == 0 and B == 0 and Brt == RANDOM_CLR_BRT); }
    void BeRandom() { DWord32 = 0; Brt = RANDOM_CLR_BRT; }
    void GenerateRandomRGB() {
        FromHSV(Random::Generate(0, 360), 100, 100);
    }

    Color_t GetRandomIfIsRandom() {
        if(IsRandom()) {
            Color_t Rslt;
            Rslt.GenerateRandomRGB();
            return Rslt;
        }
        else return *this;
    }

    uint8_t RGBTo565_HiByte() const {
        uint32_t rslt = R & 0b11111000;
        rslt |= G >> 5;
        return (uint8_t)rslt;
    }
    uint8_t RGBTo565_LoByte() const {
        uint32_t rslt = (G << 3) & 0b11100000;
        rslt |= B >> 3;
        return (uint8_t)rslt;
    }
    uint16_t RGBTo565() const {
        uint16_t rslt = ((uint16_t)(R & 0b11111000)) << 8;
        rslt |= ((uint16_t)(G & 0b11111100)) << 3;
        rslt |= ((uint16_t)B) >> 3;
        return rslt;
    }
    // ==== Mixage ====
    // Weight = 0: result is Back; Weight = 255: result is Fore; otherwise result is mix
//    void MixwWeight(const Color_t &Fore, const Color_t &Back, uint32_t Weight) {
//        R = ClrMix(Fore.R, Back.R, Weight);
//        G = ClrMix(Fore.G, Back.G, Weight);
//        B = ClrMix(Fore.B, Back.B, Weight);
//    }
    // Weight = 0: not changed; Weight = 255: result is Fore; otherwise result is mix
//    void MixwWeight(const Color_t &Fore, uint32_t Weight) {
//        R = ClrMix(Fore.R, R, Weight);
//        G = ClrMix(Fore.G, G, Weight);
//        B = ClrMix(Fore.B, B, Weight);
//    }

    // Weight = 0: not changed; Weight = 255: result is Fore; otherwise result is mix
//    void MixwWeightRGBW(const Color_t &Fore, uint32_t Weight) {
//        R = ClrMix(Fore.R, R, Weight);
//        G = ClrMix(Fore.G, G, Weight);
//        B = ClrMix(Fore.B, B, Weight);
//        W = ClrMix(Fore.W, W, Weight);
//    }

    void MixAveragingRGBW(const Color_t &Other) {
        R = MixSingleAvg(Other.R, R);
        G = MixSingleAvg(Other.G, G);
        B = MixSingleAvg(Other.B, B);
        W = MixSingleAvg(Other.W, W);
    }

    void MixAddingRGBW(const Color_t &Other) {
        R = MixSingleAdd(Other.R, R);
        G = MixSingleAdd(Other.G, G);
        B = MixSingleAdd(Other.B, B);
        W = MixSingleAdd(Other.W, W);
    }

    void MixAveragingRGB(const Color_t &Other) {
        R = MixSingleAvg(Other.R, R);
        G = MixSingleAvg(Other.G, G);
        B = MixSingleAvg(Other.B, B);
    }

    void MixAddingRGB(const Color_t &Other) {
        R = MixSingleAdd(Other.R, R);
        G = MixSingleAdd(Other.G, G);
        B = MixSingleAdd(Other.B, B);
    }

//    void MixWith(const Color_t &Clr) {
//        if(Clr.Brt == 0) return;    // Alien is off, no changes with us
//        else if(Brt == 0) DWord32 = Clr.DWord32; // We are off
//        else {
//            uint32_t BrtSum = (uint32_t)Brt + (uint32_t)Clr.Brt;
//            R = (uint8_t)(((uint32_t)R * (uint32_t)Brt + (uint32_t)Clr.R * (uint32_t)Clr.Brt) / BrtSum);
//            G = (uint8_t)(((uint32_t)G * (uint32_t)Brt + (uint32_t)Clr.G * (uint32_t)Clr.Brt) / BrtSum);
//            B = (uint8_t)(((uint32_t)B * (uint32_t)Brt + (uint32_t)Clr.B * (uint32_t)Clr.Brt) / BrtSum);
//            if(Brt < Clr.Brt) Brt = Clr.Brt; // Top brightness wins
//        }
//    }

    // Adjustment
    uint32_t DelayToNextAdj(const Color_t &AClr, uint32_t SmoothValue) {
        uint32_t Delay, Delay2;
        Delay = (R == AClr.R)? 0 : ClrCalcDelay(R, SmoothValue);
        Delay2 = (G == AClr.G)? 0 : ClrCalcDelay(G, SmoothValue);
        if(Delay2 > Delay) Delay = Delay2;
        Delay2 = (B == AClr.B)? 0 : ClrCalcDelay(B, SmoothValue);
        if(Delay2 > Delay) Delay = Delay2;
        Delay2 = (Brt == AClr.Brt)? 0 : ClrCalcDelay(Brt, SmoothValue);
        return (Delay2 > Delay)? Delay2 : Delay;
    }

    void SetRGBWBrightness(Color_t &AClr, int32_t BrtAll, const int32_t BrtMax) {
        R = SetSingleBrt(AClr.R, BrtAll, BrtMax);
        G = SetSingleBrt(AClr.G, BrtAll, BrtMax);
        B = SetSingleBrt(AClr.B, BrtAll, BrtMax);
        W = SetSingleBrt(AClr.W, BrtAll, BrtMax);
    }
    void SetRGBWBrightness(int32_t BrtAll, const int32_t BrtMax) {
        R = SetSingleBrt(R, BrtAll, BrtMax);
        G = SetSingleBrt(G, BrtAll, BrtMax);
        B = SetSingleBrt(B, BrtAll, BrtMax);
        W = SetSingleBrt(W, BrtAll, BrtMax);
    }
    void SetRGBWBrightness(int32_t BrtRGB, int32_t BrtW, const int32_t BrtMax) {
        R = SetSingleBrt(R, BrtRGB, BrtMax);
        G = SetSingleBrt(G, BrtRGB, BrtMax);
        B = SetSingleBrt(B, BrtRGB, BrtMax);
        W = SetSingleBrt(W, BrtW,   BrtMax);
    }

    void SetRGBBrightness(Color_t &AClr, const int32_t ABrt, const int32_t BrtMax) {
        R = SetSingleBrt(AClr.R, ABrt, BrtMax);
        G = SetSingleBrt(AClr.G, ABrt, BrtMax);
        B = SetSingleBrt(AClr.B, ABrt, BrtMax);
    }
    void SetRGBBrightness(const int32_t ABrt, const int32_t BrtMax) {
        R = SetSingleBrt(R, ABrt, BrtMax);
        G = SetSingleBrt(G, ABrt, BrtMax);
        B = SetSingleBrt(B, ABrt, BrtMax);
    }

    void Print() {
        if(IsRandom()) Printf("{random}");
        else Printf("{%u, %u, %u; %u}", R, G, B, Brt);
    }

    Color_t() : R(0), G(0), B(0), Brt(0) {}
    Color_t(uint8_t AR, uint8_t AG, uint8_t AB) : R(AR), G(AG), B(AB), Brt(0) {}
    Color_t(uint8_t AR, uint8_t AG, uint8_t AB, uint8_t ALum) : R(AR), G(AG), B(AB), Brt(ALum) {}
//    Color_t(const Color_t &Fore, const Color_t &Back, uint32_t Brt) {
//        R = ClrMix(Fore.R, Back.R, Brt);
//        G = ClrMix(Fore.G, Back.G, Brt);
//        B = ClrMix(Fore.B, Back.B, Brt);
//    }
} __attribute__((packed));


#if 1 // ============================ Common methods ===========================
#define RED_OF(c)           (((c) & 0xF800)>>8)
#define GREEN_OF(c)         (((c)&0x007E)>>3)
#define BLUE_OF(c)          (((c)&0x001F)<<3)

static inline uint16_t RGBTo565(uint16_t r, uint16_t g, uint16_t b) {
    uint16_t rslt = (r & 0b11111000) << 8;
    rslt |= (g & 0b11111100) << 3;
    rslt |= b >> 3;
    return rslt;
}

// Blend two colors according to the alpha;
// The alpha value (0-255). 0 is all background, 255 is all foreground.
__unused
static uint16_t ColorBlend(Color_t fg, Color_t bg, uint16_t alpha) {
    uint16_t fg_ratio = alpha + 1;
    uint16_t bg_ratio = 256 - alpha;
    uint16_t r, g, b;

    r = fg.R * fg_ratio;
    g = fg.G * fg_ratio;
    b = fg.B * fg_ratio;

    r += bg.R * bg_ratio;
    g += bg.G * bg_ratio;
    b += bg.B * bg_ratio;

    r >>= 8;
    g >>= 8;
    b >>= 8;

    return RGBTo565(r, g, b);
}
#endif

#if 1 // ============================== HSL ====================================
struct ColorHSL_t {
    union {
        uint32_t DWord32;
        struct {
            uint16_t H;     // 0...360
            uint8_t S, L;   // 0...100
        };
    };
    void ToRGB(uint8_t *PR, uint8_t *PG, uint8_t *PB) const {
        // Calc chroma: 0...255
        int32_t S1 = ((int32_t)S * 255) / 100;
        int32_t L1 = ((int32_t)L * 255) / 100;
        int32_t C = 255 - Abs32(L1 * 2 - 255);  // <=> 1 - |2*L - 1|
        C = (C * S1) / 255;                     // <=> (1 - |2*L - 1|) * S
        // Tmp values
        int32_t X = 60 - Abs32((H % 120) - 60); // 0...60
        X = (C * X) / 60;
        int32_t m = L1 - C / 2; // To add the same amount to each component, to match lightness
        // RGB in first glance
        if     (H < 60)  { *PR = C+m; *PG = X+m; *PB = m;   } // [0; 60)
        else if(H < 120) { *PR = X+m; *PG = C+m; *PB = m;   }
        else if(H < 180) { *PR = m;   *PG = C+m; *PB = X+m; }
        else if(H < 240) { *PR = m;   *PG = X+m; *PB = C+m; }
        else if(H < 300) { *PR = X+m; *PG = m;   *PB = C+m; }
        else             { *PR = C+m; *PG = m;   *PB = X+m; } // [300; 360]
    }

    void ToRGB(Color_t &AColor) { ToRGB(&AColor.R, &AColor.G, &AColor.B); }

    Color_t ToRGB() {
        Color_t Clr;
        ToRGB(Clr);
        return Clr;
    }

    ColorHSL_t() : H(0), S(0), L(0) {}
    ColorHSL_t(uint16_t AH, uint8_t AS, uint8_t AL) : H(AH), S(AS), L(AL) {}
} __attribute__((packed));
#endif

#if 1 // ============================== HSV ====================================
#define CLR_HSV_H_MAX   360
#define CLR_HSV_S_MAX   100
#define CLR_HSV_V_MAX   100

struct ColorHSV_t {
    union {
        uint32_t DWord32;
        struct {
            uint16_t H;     // 0...360
            uint8_t S, V;   // 0...100
        };
    };

    void Adjust(const ColorHSV_t &Target) {
        int16_t dH = Target.H - H;
        uint16_t sH = ABS(dH);
        if (sH > 180) sH = 360 - sH;
        // Change H
        if      ((0<dH and dH<=180) or -180>dH) {
            if (H >= 360-1) H = 0;
            else H++;
        }
        else if ((0>dH and dH>=-180) or 180<dH) {
            if (H == 0) H = 360-1;
            else H--;
        }
        // Change S
        if     (S < Target.S and sH <= Target.S-S) S++;
        else if(S > Target.S) S--;
        // Change V
        if     (V < Target.V and sH <= Target.V-V) V++;
        else if(V > Target.V) V--;
//        Printf(" Adjust dH %i (%u %u)\t sH %u\t sS %i (%u %u)\t sV %i (%u %u)\r", dH, H, Target.H, sH, Target.S-S, S, Target.S, Target.V-V, V, Target.V);
    }

    // Weight = 0: result is Back; Weight = 255: result is Fore; otherwise result is mix
//    void MixwWeight(const ColorHSV_t &Fore, const ColorHSV_t &Back, uint32_t Weight) {
//        uint8_t R, G, B;
//        uint8_t ForeR, ForeG, ForeB;
//        uint8_t BackR, BackG, BackB;
//        Fore.ToRGB(&ForeR, &ForeG, &ForeB);
//        Back.ToRGB(&BackR, &BackG, &BackB);
//        R = ClrMix(ForeR, BackR, Weight);
//        G = ClrMix(ForeG, BackG, Weight);
//        B = ClrMix(ForeB, BackB, Weight);
//        FromRGB(R, G, B);
//    }
    // Weight = 0: not changed; Weight = 255: result is Fore; otherwise result is mix
//    void MixwWeight(const ColorHSV_t &Fore, uint32_t Weight) {
//        uint8_t R, G, B;
//        uint8_t ForeR, ForeG, ForeB;
//        ToRGB(&R, &G, &B);
//        Fore.ToRGB(&ForeR, &ForeG, &ForeB);
//        R = ClrMix(ForeR, R, Weight);
//        G = ClrMix(ForeG, G, Weight);
//        B = ClrMix(ForeB, B, Weight);
//        FromRGB(R, G, B);
//    }

    uint32_t DelayToNextAdj(const ColorHSV_t &Target, uint32_t SmoothValue) {
        uint32_t Delay, Delay2;
        Delay = (H == Target.H)? 0 : ClrCalcDelay(H, SmoothValue);
        Delay2 = (S == Target.S)? 0 : ClrCalcDelay(S, SmoothValue);
        if(Delay2 > Delay) Delay = Delay2;
        Delay2 = (V == Target.V)? 0 : ClrCalcDelay(V, SmoothValue);
        return (Delay2 > Delay)? Delay2 : Delay;
    }

    void ToRGB(uint8_t *PR, uint8_t *PG, uint8_t *PB) const {
        // Calc chroma: 0...255
        int32_t C = ((int32_t)V * (int32_t)S * 255) / 10000;
        // Tmp values
        int32_t X = 60 - Abs32((H % 120) - 60); // 0...60
        X = (C * X) / 60;
        int32_t m = (((int32_t)V * 255) / 100) - C; // To add the same amount to each component, to match lightness
        // RGB
        if     (H < 60)  { *PR = C+m; *PG = X+m; *PB = m;   } // [0; 60)
        else if(H < 120) { *PR = X+m; *PG = C+m; *PB = m;   }
        else if(H < 180) { *PR = m;   *PG = C+m; *PB = X+m; }
        else if(H < 240) { *PR = m;   *PG = X+m; *PB = C+m; }
        else if(H < 300) { *PR = X+m; *PG = m;   *PB = C+m; }
        else             { *PR = C+m; *PG = m;   *PB = X+m; } // [300; 360]
    }

    void ToRGB(Color_t &AColor) { ToRGB(&AColor.R, &AColor.G, &AColor.B); }
    Color_t ToRGB() {
        Color_t rgb;
        ToRGB(&rgb.R, &rgb.G, &rgb.B);
        return rgb;
    }

    void FromRGB(int32_t Red, int32_t Green, int32_t Blue) {
        int32_t Max, Min;
        // Find Min & Max
        Max = Red;
        if(Max < Green) Max = Green;
        if(Max < Blue) Max = Blue;
        Min = Red;
        if(Min > Green) Min = Green;
        if(Min > Blue) Min = Blue;
        // H
        if(Max == Min) H = 0;
        else if(Max == Red and Green >= Blue) H = (60 * (Green - Blue) + 254L) / (Max - Min) + 0;
        else if(Max == Red and Green <  Blue) H = (60 * (Green - Blue) + 254L) / (Max - Min) + 360;
        else if(Max == Green)                 H = (60 * (Blue - Red)   + 254L) / (Max - Min) + 120;
        else if(Max == Blue)                  H = (60 * (Red - Green)  + 254L) / (Max - Min) + 240;
        // S
        if(Max == 0) S = 0;
        else S = 100 - (100 * Min) / Max;
        // V
        V = (100 * Max) / 255;
    }

    void FromRGB(Color_t Clr) {
        FromRGB(Clr.R, Clr.G, Clr.B);
    }

    void FromHSV(uint16_t AH, uint8_t AS, uint8_t AV) {
        H = AH; S = AS; V = AV;
    }

    ColorHSV_t& operator = (const ColorHSV_t &Right) { DWord32 = Right.DWord32; return *this; }
    bool operator == (const ColorHSV_t &AColor) const { return (DWord32 == AColor.DWord32); }
    bool operator != (const ColorHSV_t &AColor) const { return (DWord32 != AColor.DWord32); }

    ColorHSV_t() : H(0), S(0), V(0) {}
    ColorHSV_t(uint16_t AH, uint8_t AS, uint8_t AV) : H(AH), S(AS), V(AV) {}
    ColorHSV_t(const ColorHSV_t &AClr) : H(AClr.H), S(AClr.S), V(AClr.V) {}
} __attribute__((packed));

// Colors
#define hsvRed       ((ColorHSV_t){  0, 100, 100})
#define hsvYellow    ((ColorHSV_t){ 60, 100, 100})
#define hsvGreen     ((ColorHSV_t){120, 100, 100})
#define hsvCyan      ((ColorHSV_t){180, 100, 100})
#define hsvBlue      ((ColorHSV_t){240, 100, 100})
#define hsvMagenta   ((ColorHSV_t){300, 100, 100})
#define hsvWhite     ((ColorHSV_t){0,   0,   100})
#define hsvBlack     ((ColorHSV_t){0,   0,   0  })
#endif

#if 1 // ============================= Colors ==================================
#define clBlack     ((Color_t){0,   0,   0})
#define clRed       ((Color_t){255, 0,   0})
#define clGreen     ((Color_t){0,   255, 0})
#define clBlue      ((Color_t){0,   0,   255})
#define clYellow    ((Color_t){255, 255, 0})
#define clMagenta   ((Color_t){255, 0, 255})
#define clCyan      ((Color_t){0, 255, 255})
#define clWhite     ((Color_t){255, 255, 255})

#define clGrey      ((Color_t){126, 126, 126})
#define clLightGrey ((Color_t){180, 180, 180})
#define clDarkGrey  ((Color_t){54, 54, 54})

#define CL_DARK_V       27
#define clDarkRed       ((Color_t){CL_DARK_V, 0,         0})
#define clDarkGreen     ((Color_t){0,         CL_DARK_V, 0})
#define clDarkBlue      ((Color_t){0,         0,         CL_DARK_V})
#define clDarkYellow    ((Color_t){CL_DARK_V, CL_DARK_V, 0})
#define clDarkMagenta   ((Color_t){CL_DARK_V, 0,         CL_DARK_V})
#define clDarkCyan      ((Color_t){0,         CL_DARK_V, CL_DARK_V})
#define clDarkWhite     ((Color_t){CL_DARK_V, CL_DARK_V, CL_DARK_V})

#define clLightBlue ((Color_t){90, 90, 255})

// RGBW
#define clRGBWBlack     ((Color_t){0,   0,   0,   0})
#define clRGBWRed       ((Color_t){255, 0,   0,   0})
#define clRGBWGreen     ((Color_t){0,   255, 0,   0})
#define clRGBWBlue      ((Color_t){0,   0,   255, 0})
#define clRGBWYellow    ((Color_t){255, 255, 0,   0})
#define clRGBWMagenta   ((Color_t){255, 0, 255,   0})
#define clRGBWCyan      ((Color_t){0, 255, 255,   0})
#define clRGBWWhite     ((Color_t){0,   0,   0, 255})
#endif

#endif // COLOR_H_
