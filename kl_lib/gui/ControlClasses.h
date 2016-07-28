/*
 * ControlClasses.h
 *
 *  Created on: 15 ??? 2016 ?.
 *      Author: Kreyl
 */

#pragma once

#include "color.h"
#include "font.h"

enum Justify_t { jstLeft, jstCenter, jstRight };

#if 1 // ==== Classes ====
enum ControlType_t { ctrlBtn, ctrlTextbox, ctrlChart, ctrlLine };

// Parent class for all controls
class Control_t {
protected:
    void FillRect(Color_t ClrTop, Color_t ClrBottom) const;
public:
    ControlType_t Type;
    uint16_t Left, Top, Width, Height;
    const char* Text;
    PFont_t Font;
    Color_t ClrText;
    virtual void Draw() const;
    bool IsInside(int32_t x, int32_t y) const {
        return (x >= Left) and (x <= Left+Width) and (y >= Top) and (y <= Top+Height);
    }
    Control_t(ControlType_t AType,
            uint16_t ALeft, uint16_t ATop, uint16_t AWidth, uint16_t AHeight,
            const char* AText, PFont_t AFont, Color_t AClrText) :
                Type(AType),
                Left(ALeft), Top(ATop), Width(AWidth), Height(AHeight),
                Text(AText), Font(AFont), ClrText(AClrText)  {}
};

typedef void (*ftEvtCb)(const Control_t *p);

// ==== Lines ====
class LineHoriz_t : public Control_t {
public:
    Color_t Clr;
    void Draw() const;
    LineHoriz_t(uint16_t x0, uint16_t y0, uint16_t Len, uint16_t AWidth, Color_t AClr) :
        Control_t(ctrlLine, x0, y0, Len, AWidth, nullptr, nullptr, clWhite), Clr(AClr) {}
};

class LineVert_t : public LineHoriz_t {
public:
//    Color_t Clr;
//    void Draw() const;
    LineVert_t(uint16_t x0, uint16_t y0, uint16_t Len, uint16_t AWidth, Color_t AClr) :
        LineHoriz_t(x0, y0, AWidth, Len, AClr) {}
};

// ==== Button ====
enum BtnState_t {btnPressed, btnReleased};

class Button_t : public Control_t {
private:
    Color_t ClrReleasedTop, ClrReleasedBottom, ClrPressedTop, ClrPressedBottom;
public:
    void Draw() const { Draw(btnReleased); }
    void Draw(BtnState_t State) const;
    ftEvtCb OnRelease;
    void CheckTouchAndAct(int32_t x, int32_t y) const {
        if(IsInside(x, y)) Draw(btnPressed);
    }
    void CheckDetouchAndAct(int32_t x, int32_t y) const {
        if(IsInside(x, y)) {
            Draw(btnReleased);
            if(OnRelease != nullptr) OnRelease((const Control_t*)this);
        }
    }

    Button_t(uint16_t ALeft, uint16_t ATop, uint16_t AWidth, uint16_t AHeight,
            const char* AText, PFont_t AFont, Color_t AClrText,
            Color_t AClrReleasedTop, Color_t AClrReleasedBottom,
            Color_t AClrPressedTop,  Color_t AClrPressedBottom,
            ftEvtCb AOnRelease) :
                Control_t(ctrlBtn, ALeft, ATop, AWidth, AHeight, AText, AFont, AClrText),
                ClrReleasedTop(AClrReleasedTop), ClrReleasedBottom(AClrReleasedBottom),
                ClrPressedTop(AClrPressedTop),   ClrPressedBottom(AClrPressedBottom),
                OnRelease(AOnRelease) {}
};

// ==== Textbox ====
class Textbox_t : public Control_t {
public:
    void Draw() const;
    Color_t ClrBack;
    Textbox_t(uint16_t ALeft, uint16_t ATop, uint16_t AWidth, uint16_t AHeight,
            const char* AText,
            PFont_t AFont,
            Color_t AClrText, Color_t AClrBack) :
                Control_t(ctrlTextbox, ALeft, ATop, AWidth, AHeight, AText, AFont, AClrText),
                ClrBack(AClrBack) {}

};

// ==== Chart ====
#define X_SCALE         ((float)(CHART_W_MS / CHART_W_PX))

class Chart_t;

struct Point_t {
    float x, y;
};

class Series_t {
private:
    Chart_t *Parent;
public:
    Color_t Color;
    void AddPoint(float x, float y);
    void Clear();
    Series_t(Chart_t *AParent, Color_t AClr) :
        Parent(AParent),
        Color(AClr) {}
};

class Chart_t {
private:
    uint16_t Left, Top, Width, Height;
    float Xmin, Xmax, Ymin, Ymax;
    PFont_t Font;
    Color_t ClrText, ClrBack;
    uint32_t ScaledY(float y);
    uint32_t ScaledX(float x);
public:
    void Clear();
    void AddLineHoriz(float y, Color_t AColor);
    void AddLineVert(float x, Color_t AColor);
    Chart_t(uint16_t ALeft, uint16_t ATop, uint16_t AWidth, uint16_t AHeight,
            float AXmin, float AXmax, float AYmin, float AYmax,
            PFont_t AFont, Color_t AClrText,
            Color_t AClrBack) :
        Left(ALeft), Top(ATop), Width(AWidth), Height(AHeight),
        Xmin(AXmin), Xmax(AXmax), Ymin(AYmin), Ymax(AYmax),
        Font(AFont), ClrText(AClrText),
        ClrBack(AClrBack) {}
    friend class Series_t;
};

// ==== Page ====
class Page_t {
public:
    const Control_t **Controls;
    uint32_t CtrlCnt;
    void Draw() const {
        for(uint32_t i=0; i<CtrlCnt; i++) {
            const Control_t *PCtrl = Controls[i];
            PCtrl->Draw();
        } // for
    }

    void ProcessTouch(int32_t x, int32_t y) const {
        for(uint32_t i=0; i<CtrlCnt; i++) {
            if(Controls[i]->Type == ctrlBtn) {
                ((Button_t*)Controls[i])->CheckTouchAndAct(x, y);
            }
        }
    }

    void ProcessDetouch(int32_t x, int32_t y) const {
        for(uint32_t i=0; i<CtrlCnt; i++) {
            if(Controls[i]->Type == ctrlBtn) {
                ((Button_t*)Controls[i])->CheckDetouchAndAct(x, y);
            }
        }
    }

    Page_t(const Control_t **AControls, uint32_t ACtrlCnt) :
        Controls(AControls), CtrlCnt(ACtrlCnt) {}
};
#endif
