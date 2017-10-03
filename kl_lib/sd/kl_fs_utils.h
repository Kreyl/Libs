/*
 * kl_fs_common.h
 *
 *  Created on: 30 џэт. 2016 у.
 *      Author: Kreyl
 */

#pragma once

#include "ff.h"
#include "kl_lib.h"
#include "shell.h"

// Variables
extern FILINFO FileInfo;
extern DIR Dir;

uint8_t TryOpenFileRead(const char *Filename, FIL *PFile);
uint8_t TryOpenFileRewrite(const char *Filename, FIL *PFile);
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
uint8_t CountDirsStartingWith(const char* Path, const char* DirNameStart, uint32_t *PCnt);

#define SD_STRING_SZ    256 // for operations with strings
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

//template <typename T>
//uint8_t iniRead(const char *AFileName, const char *ASection, const char *AKey, T *POutput);


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

void WriteSection(FIL *PFile, const char *ASection);
void WriteString(FIL *PFile, const char *AKey, char *AValue);
void WriteInt32(FIL *PFile, const char *AKey, const int32_t AValue);
void WriteNewline(FIL *PFile);

} // namespace

namespace csv { // =================== csv file operations =====================
/*
 * csv file has the following structure:
 *
 * # this is comment
 * 14, 0x38, "DirName1"
 * 15, 0, "DirName2"
 * ...
 */

uint8_t OpenFile(const char *AFileName);
void CloseFile();
void RewindFile();
uint8_t ReadNextLine();
void GetNextCellString(char* POutput);
uint8_t GetNextToken(char** POutput);

// Finds first cell with given name and puts pointer to next cell
uint8_t FindFirstCell(const char* Name);

template <typename T>
static uint8_t GetNextCell(T *POutput) {
    char *Token;
    if(GetNextToken(&Token) == retvOk) {
        char *p;
        *POutput = (T)strtoul(Token, &p, 0);
        if(*p == '\0') return retvOk;
        else return retvNotANumber;
    }
    else return retvEmpty;
}

} // namespace
