/*
 * color.h
 *
 *  Created on: 2014
 *      Author: Kreyl
 */

#ifndef COLOR_H_
#define COLOR_H_

#include "types.h"

void Printf(const char *format, ...);
namespace Random {
    int32_t Generate(int32_t min_inclusive, int32_t max_inclusive);
}

/* For gamma table generation. use next python code:
gamma_name = "2dot8"
gamma = 2.8
max: int = 255
offset = 1  # will be added to all values save 0 one, with restriction to max.

def main():
    print("static const uint8_t gamma_{}[] = {{".format(gamma_name))
    for i in range(0, max + 1):
        g = 0 if i == 0 else offset + round(max * pow((i / max), gamma))
        g = g if g < max else max
        print("{: 4d},".format(g), end="\n" if (i + 1) % 16 == 0 else "")
    print("};")
*/

struct ColorHSV_t;

// In Settings, if Color.Brt == RANDOM_CLR_BRT then color should be random.
// The check and transmutation shall be made in upper level.
#define RANDOM_CLR_BRT      255


__attribute__((__always_inline__))
static inline int32_t Abs32(int32_t w) { return (w < 0)? -w : w; }

// Smooth delay
static inline uint32_t ClrCalcDelay(uint16_t AValue, uint32_t Smooth) {
    return (uint32_t)((Smooth / (AValue+4)) + 1);
}

#pragma region // =================== RGB[W] Color_t ====================
class Color_t {
private:
    __attribute__((__always_inline__)) uint8_t  SetSingleBrt(int32_t v, const int32_t abrt, const int32_t abrt_max) { return (v * abrt) / abrt_max; }
    __attribute__((__always_inline__)) uint32_t MixSingleAvg(uint32_t fore, uint32_t back) { return (fore + back) / 2; }
    __attribute__((__always_inline__)) uint32_t MixSingleAdd(uint32_t fore, uint32_t back) {
        uint32_t sum = fore + back;
        return sum > 255? 255 : sum;
    }
public:
    union {
        uint32_t dword32;
        struct {
            uint8_t R, G, B;
            union {
                uint8_t Brt, W;
            };
        } __attribute__((packed));
    };
    bool operator == (const Color_t &acolor) const { return (dword32 == acolor.dword32); }
    bool operator != (const Color_t &acolor) const { return (dword32 != acolor.dword32); }
    Color_t& operator = (const Color_t &right) { dword32 = right.dword32; return *this; }

    void ApplyGammaCorrectionRGB(uint8_t* gamma_table) {
        R = gamma_table[R];
        G = gamma_table[G];
        B = gamma_table[B];
    }

    void ApplyGammaCorrectionRGBW(uint8_t* gamma_table) {
        R = gamma_table[R];
        G = gamma_table[G];
        B = gamma_table[B];
        W = gamma_table[W];
    }

    void Adjust(const Color_t &acolor) {
        if     (R < acolor.R) R++;
        else if(R > acolor.R) R--;
        if     (G < acolor.G) G++;
        else if(G > acolor.G) G--;
        if     (B < acolor.B) B++;
        else if(B > acolor.B) B--;
        if     (Brt < acolor.Brt) Brt++;
        else if(Brt > acolor.Brt) Brt--;
    }

    void Adjust(const Color_t &acolor, uint32_t step, const int32_t brt_max) {
        uint32_t thrs_r = 255 - step;
        if(R < acolor.R) {
            if(R <= thrs_r) R += step;
            else R = 255;
        }
        else if(R > acolor.R) {
            if(R >= step) R -= step;
            else R = 0;
        }

        if(G < acolor.G) {
            if(G <= thrs_r) G += step;
            else G = 255;
        }
        else if(G > acolor.G) {
            if(G >= step) G -= step;
            else G = 0;
        }

        if(B < acolor.B) {
            if(B <= thrs_r) B += step;
            else B = 255;
        }
        else if(B > acolor.B) {
            if(B >= step) B -= step;
            else B = 0;
        }

        if(Brt < acolor.Brt) {
            Brt += step;
            if(Brt > brt_max) Brt = brt_max;
        }
        else if(Brt > acolor.Brt) {
            if(Brt >= step) Brt -= step;
            else Brt = 0;
        }
    }
    void FromRGB(uint8_t red, uint8_t green, uint8_t blue) { R = red; G = green; B = blue; }
    void ToRGB(uint8_t *pr, uint8_t *pg, uint8_t *pb) const { *pr = R; *pg = G; *pb = B; }
    bool IsEqualRGB(uint8_t red, uint8_t green, uint8_t blue) { return (R == red and G == green and B == blue); }

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
    void BeRandom() { dword32 = 0; Brt = RANDOM_CLR_BRT; }
    void GenerateRandomRGB() { FromHSV(Random::Generate(0, 360), 100, 100); }

    Color_t GetRandomIfIsRandom() {
        if(IsRandom()) {
            Color_t rslt;
            rslt.GenerateRandomRGB();
            return rslt;
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
    // Weight = 0: result is back; Weight = 255: result is fore; otherwise result is mix
//    void MixwWeight(const Color_t &fore, const Color_t &back, uint32_t Weight) {
//        R = ClrMix(fore.R, back.R, Weight);
//        G = ClrMix(fore.G, back.G, Weight);
//        B = ClrMix(fore.B, back.B, Weight);
//    }
    // Weight = 0: not changed; Weight = 255: result is fore; otherwise result is mix
//    void MixwWeight(const Color_t &fore, uint32_t Weight) {
//        R = ClrMix(fore.R, R, Weight);
//        G = ClrMix(fore.G, G, Weight);
//        B = ClrMix(fore.B, B, Weight);
//    }

    // Weight = 0: not changed; Weight = 255: result is fore; otherwise result is mix
//    void MixwWeightRGBW(const Color_t &fore, uint32_t Weight) {
//        R = ClrMix(fore.R, R, Weight);
//        G = ClrMix(fore.G, G, Weight);
//        B = ClrMix(fore.B, B, Weight);
//        W = ClrMix(fore.W, W, Weight);
//    }

    void MixAveragingRGBW(const Color_t &other) {
        R = MixSingleAvg(other.R, R);
        G = MixSingleAvg(other.G, G);
        B = MixSingleAvg(other.B, B);
        W = MixSingleAvg(other.W, W);
    }

    void MixAddingRGBW(const Color_t &other) {
        R = MixSingleAdd(other.R, R);
        G = MixSingleAdd(other.G, G);
        B = MixSingleAdd(other.B, B);
        W = MixSingleAdd(other.W, W);
    }

    void MixAveragingRGB(const Color_t &other) {
        R = MixSingleAvg(other.R, R);
        G = MixSingleAvg(other.G, G);
        B = MixSingleAvg(other.B, B);
    }

    void MixAddingRGB(const Color_t &other) {
        R = MixSingleAdd(other.R, R);
        G = MixSingleAdd(other.G, G);
        B = MixSingleAdd(other.B, B);
    }

//    void MixWith(const Color_t &Clr) {
//        if(Clr.Brt == 0) return;    // Alien is off, no changes with us
//        else if(Brt == 0) dword32 = Clr.dword32; // We are off
//        else {
//            uint32_t BrtSum = (uint32_t)Brt + (uint32_t)Clr.Brt;
//            R = (uint8_t)(((uint32_t)R * (uint32_t)Brt + (uint32_t)Clr.R * (uint32_t)Clr.Brt) / BrtSum);
//            G = (uint8_t)(((uint32_t)G * (uint32_t)Brt + (uint32_t)Clr.G * (uint32_t)Clr.Brt) / BrtSum);
//            B = (uint8_t)(((uint32_t)B * (uint32_t)Brt + (uint32_t)Clr.B * (uint32_t)Clr.Brt) / BrtSum);
//            if(Brt < Clr.Brt) Brt = Clr.Brt; // Top brightness wins
//        }
//    }

    // Adjustment
    uint32_t DelayToNextAdj(const Color_t &aclr, uint32_t smooth_value) {
        uint32_t delay, delay2;
        delay = (R == aclr.R)? 0 : ClrCalcDelay(R, smooth_value);
        delay2 = (G == aclr.G)? 0 : ClrCalcDelay(G, smooth_value);
        if(delay2 > delay) delay = delay2;
        delay2 = (B == aclr.B)? 0 : ClrCalcDelay(B, smooth_value);
        if(delay2 > delay) delay = delay2;
        delay2 = (Brt == aclr.Brt)? 0 : ClrCalcDelay(Brt, smooth_value);
        return (delay2 > delay)? delay2 : delay;
    }

    void SetRGBWBrightness(Color_t &aclr, int32_t brt_all, const int32_t brt_max) {
        R = SetSingleBrt(aclr.R, brt_all, brt_max);
        G = SetSingleBrt(aclr.G, brt_all, brt_max);
        B = SetSingleBrt(aclr.B, brt_all, brt_max);
        W = SetSingleBrt(aclr.W, brt_all, brt_max);
    }
    void SetRGBWBrightness(int32_t brt_all, const int32_t brt_max) {
        R = SetSingleBrt(R, brt_all, brt_max);
        G = SetSingleBrt(G, brt_all, brt_max);
        B = SetSingleBrt(B, brt_all, brt_max);
        W = SetSingleBrt(W, brt_all, brt_max);
    }
    void SetRGBWBrightness(int32_t BrtRGB, int32_t BrtW, const int32_t brt_max) {
        R = SetSingleBrt(R, BrtRGB, brt_max);
        G = SetSingleBrt(G, BrtRGB, brt_max);
        B = SetSingleBrt(B, BrtRGB, brt_max);
        W = SetSingleBrt(W, BrtW,   brt_max);
    }

    void SetRGBBrightness(Color_t &aclr, const int32_t ABrt, const int32_t brt_max) {
        R = SetSingleBrt(aclr.R, ABrt, brt_max);
        G = SetSingleBrt(aclr.G, ABrt, brt_max);
        B = SetSingleBrt(aclr.B, ABrt, brt_max);
    }
    void SetRGBBrightness(const int32_t ABrt, const int32_t brt_max) {
        R = SetSingleBrt(R, ABrt, brt_max);
        G = SetSingleBrt(G, ABrt, brt_max);
        B = SetSingleBrt(B, ABrt, brt_max);
    }

    void Print() {
        if(IsRandom()) Printf("{random}");
        else Printf("{%u, %u, %u; %u}", R, G, B, Brt);
    }

    Color_t() : R(0), G(0), B(0), Brt(0) {}
    Color_t(uint8_t AR, uint8_t AG, uint8_t AB) : R(AR), G(AG), B(AB), Brt(0) {}
    Color_t(uint8_t AR, uint8_t AG, uint8_t AB, uint8_t ALum) : R(AR), G(AG), B(AB), Brt(ALum) {}
//    Color_t(const Color_t &fore, const Color_t &back, uint32_t Brt) {
//        R = ClrMix(fore.R, back.R, Brt);
//        G = ClrMix(fore.G, back.G, Brt);
//        B = ClrMix(fore.B, back.B, Brt);
//    }
} __attribute__((packed));
#pragma endregion

#pragma region // ============================ Common methods ===========================
#define RED_OF(c)           (((c)&0xF800)>>8)
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
__attribute__((unused))
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

__attribute__((unused))
static Color_t Mix3RGBsLimitedAdding(
    const Color_t &c1, const Color_t &c2, const Color_t &c3,
    uint32_t brt1, uint32_t brt2, uint32_t brt3) {
    // Calculate weighted components
    uint32_t r = (c1.R * brt1 + c2.R * brt2 + c3.R * brt3) / 255UL;
    uint32_t g = (c1.G * brt1 + c2.G * brt2 + c3.G * brt3) / 255UL;
    uint32_t b = (c1.B * brt1 + c2.B * brt2 + c3.B * brt3) / 255UL;
    if(r > 255) r = 255;
    if(g > 255) g = 255;
    if(b > 255) b = 255;
    return Color_t(r, g, b);
}

__attribute__((unused))
static Color_t Mix3RGBsScaledAdding(
    const Color_t &c1, const Color_t &c2, const Color_t &c3,
    uint32_t brt1, uint32_t brt2, uint32_t brt3) {
    // Calculate weighted components
    uint32_t r = (c1.R * brt1 + c2.R * brt2 + c3.R * brt3) / 255UL;
    uint32_t g = (c1.G * brt1 + c2.G * brt2 + c3.G * brt3) / 255UL;
    uint32_t b = (c1.B * brt1 + c2.B * brt2 + c3.B * brt3) / 255UL;
    // Find maximum component
    uint32_t max = r;
    if(g > max) max = g;
    if(b > max) max = b;
    // Scale if needed
    if(max > 255) {
        r = (r * 255UL) / max;
        g = (g * 255UL) / max;
        b = (b * 255UL) / max;
    }
    return Color_t(r, g, b);
}

__attribute__((unused))
static Color_t Mix3RGBsAveraging(
    const Color_t &c1, const Color_t &c2, const Color_t &c3,
    uint32_t brt1, uint32_t brt2, uint32_t brt3) {
    uint32_t brt_sum = brt1 + brt2 + brt3;
    if(brt_sum == 0) return Color_t(0, 0, 0);
    // Calculate weighted components
    uint32_t r = (c1.R * brt1 + c2.R * brt2 + c3.R * brt3) / brt_sum;
    uint32_t g = (c1.G * brt1 + c2.G * brt2 + c3.G * brt3) / brt_sum;
    uint32_t b = (c1.B * brt1 + c2.B * brt2 + c3.B * brt3) / brt_sum;
    return Color_t(r, g, b);
}
#pragma endregion

#pragma region // ============================== HSL ====================================
struct ColorHSL_t {
    union {
        uint32_t dword32;
        struct {
            uint16_t H;     // 0...360
            uint8_t S, L;   // 0...100
        };
    };
    void ToRGB(uint8_t *pr, uint8_t *pg, uint8_t *pb) const {
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
        if     (H < 60)  { *pr = C+m; *pg = X+m; *pb = m;   } // [0; 60)
        else if(H < 120) { *pr = X+m; *pg = C+m; *pb = m;   }
        else if(H < 180) { *pr = m;   *pg = C+m; *pb = X+m; }
        else if(H < 240) { *pr = m;   *pg = X+m; *pb = C+m; }
        else if(H < 300) { *pr = X+m; *pg = m;   *pb = C+m; }
        else             { *pr = C+m; *pg = m;   *pb = X+m; } // [300; 360]
    }

    void ToRGB(Color_t &acolor) { ToRGB(&acolor.R, &acolor.G, &acolor.B); }

    Color_t ToRGB() {
        Color_t Clr;
        ToRGB(Clr);
        return Clr;
    }

    ColorHSL_t() : H(0), S(0), L(0) {}
    ColorHSL_t(uint16_t AH, uint8_t AS, uint8_t AL) : H(AH), S(AS), L(AL) {}
} __attribute__((packed));
#pragma endregion

#pragma region // ============================== HSV ====================================
#define CLR_HSV_H_MAX   360
#define CLR_HSV_S_MAX   100
#define CLR_HSV_V_MAX   100

struct ColorHSV_t {
    union {
        uint32_t dword32;
        struct {
            uint16_t H;     // 0...360
            uint8_t S, V;   // 0...100
        };
    };

    void Adjust(const ColorHSV_t &Target) {
        int16_t dH = Target.H - H;
        uint16_t sH = Abs32(dH);
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

    // Weight = 0: result is back; Weight = 255: result is fore; otherwise result is mix
//    void MixwWeight(const ColorHSV_t &fore, const ColorHSV_t &back, uint32_t Weight) {
//        uint8_t R, G, B;
//        uint8_t ForeR, ForeG, ForeB;
//        uint8_t BackR, BackG, BackB;
//        fore.ToRGB(&ForeR, &ForeG, &ForeB);
//        back.ToRGB(&BackR, &BackG, &BackB);
//        R = ClrMix(ForeR, BackR, Weight);
//        G = ClrMix(ForeG, BackG, Weight);
//        B = ClrMix(ForeB, BackB, Weight);
//        FromRGB(R, G, B);
//    }
    // Weight = 0: not changed; Weight = 255: result is fore; otherwise result is mix
//    void MixwWeight(const ColorHSV_t &fore, uint32_t Weight) {
//        uint8_t R, G, B;
//        uint8_t ForeR, ForeG, ForeB;
//        ToRGB(&R, &G, &B);
//        fore.ToRGB(&ForeR, &ForeG, &ForeB);
//        R = ClrMix(ForeR, R, Weight);
//        G = ClrMix(ForeG, G, Weight);
//        B = ClrMix(ForeB, B, Weight);
//        FromRGB(R, G, B);
//    }

    uint32_t DelayToNextAdj(const ColorHSV_t &Target, uint32_t smooth_value) {
        uint32_t delay, delay2;
        delay = (H == Target.H)? 0 : ClrCalcDelay(H, smooth_value);
        delay2 = (S == Target.S)? 0 : ClrCalcDelay(S, smooth_value);
        if(delay2 > delay) delay = delay2;
        delay2 = (V == Target.V)? 0 : ClrCalcDelay(V, smooth_value);
        return (delay2 > delay)? delay2 : delay;
    }

    void ToRGB(uint8_t *pr, uint8_t *pg, uint8_t *pb) const {
        // Calc chroma: 0...255
        int32_t C = ((int32_t)V * (int32_t)S * 255) / 10000;
        // Tmp values
        int32_t X = 60 - Abs32((H % 120) - 60); // 0...60
        X = (C * X) / 60;
        int32_t m = (((int32_t)V * 255) / 100) - C; // To add the same amount to each component, to match lightness
        // RGB
        if     (H < 60)  { *pr = C+m; *pg = X+m; *pb = m;   } // [0; 60)
        else if(H < 120) { *pr = X+m; *pg = C+m; *pb = m;   }
        else if(H < 180) { *pr = m;   *pg = C+m; *pb = X+m; }
        else if(H < 240) { *pr = m;   *pg = X+m; *pb = C+m; }
        else if(H < 300) { *pr = X+m; *pg = m;   *pb = C+m; }
        else             { *pr = C+m; *pg = m;   *pb = X+m; } // [300; 360]
    }

    void ToRGB(Color_t &acolor) { ToRGB(&acolor.R, &acolor.G, &acolor.B); }
    Color_t ToRGB() {
        Color_t rgb;
        ToRGB(&rgb.R, &rgb.G, &rgb.B);
        return rgb;
    }

    void FromRGB(int32_t red, int32_t green, int32_t blue) {
        int32_t Max, Min;
        // Find Min & Max
        Max = red;
        if(Max < green) Max = green;
        if(Max < blue) Max = blue;
        Min = red;
        if(Min > green) Min = green;
        if(Min > blue) Min = blue;
        // H
        if(Max == Min) H = 0;
        else if(Max == red and green >= blue) H = (60 * (green - blue) + 254L) / (Max - Min) + 0;
        else if(Max == red and green <  blue) H = (60 * (green - blue) + 254L) / (Max - Min) + 360;
        else if(Max == green)                 H = (60 * (blue - red)   + 254L) / (Max - Min) + 120;
        else if(Max == blue)                  H = (60 * (red - green)  + 254L) / (Max - Min) + 240;
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

    ColorHSV_t& operator = (const ColorHSV_t &right) { dword32 = right.dword32; return *this; }
    bool operator == (const ColorHSV_t &acolor) const { return (dword32 == acolor.dword32); }
    bool operator != (const ColorHSV_t &acolor) const { return (dword32 != acolor.dword32); }

    ColorHSV_t() : H(0), S(0), V(0) {}
    ColorHSV_t(uint16_t AH, uint8_t AS, uint8_t AV) : H(AH), S(AS), V(AV) {}
    ColorHSV_t(const ColorHSV_t &aclr) : H(aclr.H), S(aclr.S), V(aclr.V) {}
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
#pragma endregion

#pragma region // ============================= Colors ==================================
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
#pragma endregion

#endif // COLOR_H_
