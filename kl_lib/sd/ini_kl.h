/*
 * ini_kl.h
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "color.h"
#include "ff.h"

namespace ini { // =================== ini file operations =====================
/*
 * ini file has the following structure:
 *
 * # This is Comment: comment uses either '#' or ';' symbol
 * ; This is Comment too
 *
 * [Section]    ; This is name of section
 * Count=6      ; This is key with value of int32
 * Volume=-1    ; int32
 * SoundFileName=phrase01.wav   ; string
 *
 * [Section2]
 * Key1=1
 * ...
 */

uint8_t ReadString(const char *AFileName, const char *ASection, const char *AKey, char **PPOutput);

uint8_t ReadStringTo(const char *AFileName, const char *ASection, const char *AKey, char *POutput, uint32_t MaxLen);

template <typename T>
static uint8_t Read(const char *AFileName, const char *ASection, const char *AKey, T *POutput) {
    char *S = nullptr;
    if(ReadString(AFileName, ASection, AKey, &S) == retvOk) {
        int32_t tmp = strtol(S, NULL, 10);
        *POutput = (T)tmp;
        return retvOk;
    }
    else return retvFail;
}

uint8_t ReadInt32(const char *AFileName, const char *ASection, const char *AKey, int32_t *AOutput);

uint8_t ReadColor (const char *AFileName, const char *ASection, const char *AKey, Color_t *AOutput);

void WriteSection(FIL *PFile, const char *ASection);
void WriteString(FIL *PFile, const char *AKey, char *AValue);
void WriteInt32(FIL *PFile, const char *AKey, const int32_t AValue);
void WriteNewline(FIL *PFile);

} // namespace
