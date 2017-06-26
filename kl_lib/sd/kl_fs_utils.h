/*
 * kl_fs_common.h
 *
 *  Created on: 30 џэт. 2016 у.
 *      Author: Kreyl
 */

#pragma once

#include "ff.h"
#include "kl_lib.h"

// Variables
extern FILINFO FileInfo;
extern DIR Dir;

uint8_t TryOpenFileRead(const char *Filename, FIL *PFile);
uint8_t CheckFileNotEmpty(FIL *PFile);
uint8_t TryRead(FIL *PFile, void *Ptr, uint32_t Sz);

//template <typename T>
//uint8_t TryRead(FIL *PFile, T *Ptr);

template <typename T>
uint8_t TryRead(FIL *PFile, T *Ptr) {
    uint32_t ReadSz=0;
    uint8_t r = f_read(PFile, Ptr, sizeof(T), &ReadSz);
    return (r == FR_OK and ReadSz == sizeof(T))? retvOk : retvFail;
}

//bool CurrentDirIsRoot();
//void ResetCurrDir

uint8_t ReadLine(FIL *PFile, char* S, uint32_t MaxLen);

uint8_t CountFilesInDir(const char* DirName, const char* Extension, uint32_t *PCnt);

// =========================== ini file operations =============================
#define SD_STRING_SZ    256 // for operations with strings
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

uint8_t iniReadString(const char *AFileName, const char *ASection, const char *AKey, char **PPOutput);

//template <typename T>
//uint8_t iniRead(const char *AFileName, const char *ASection, const char *AKey, T *POutput);


template <typename T>
static uint8_t iniRead(const char *AFileName, const char *ASection, const char *AKey, T *POutput) {
    char *S = nullptr;
    if(iniReadString(AFileName, ASection, AKey, &S) == retvOk) {
        int32_t tmp = strtol(S, NULL, 10);
        *POutput = (T)tmp;
        return retvOk;
    }
    else return retvFail;
}
