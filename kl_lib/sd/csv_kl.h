/*
 * csv_kl.h
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"

namespace csv { // =================== csv file operations =====================
/*
 * csv file has the following structure:
 *
 * # this is comment
 * 14, 0x38, "DirName1"
 * Name = "Mr. First"
 * ...
 */

uint8_t OpenFile(const char *AFileName);
void CloseFile();
void RewindFile();
uint8_t ReadNextLine();
uint8_t GetNextCellString(char* POutput);
uint8_t GetNextToken(char** POutput);

// Finds first cell with given name and puts pointer to next cell
uint8_t FindFirstCell(const char* Name);

template <typename T>
static uint8_t GetNextCell(T *POutput) {
    char *Token;
    if(GetNextToken(&Token) == retvOk) {
//        Printf("Token: %S\r", Token);
        char *p;
        *POutput = (T)strtoul(Token, &p, 0);
        if(*p == '\0') return retvOk;
        else return retvNotANumber;
    }
    else return retvEmpty;
}

uint8_t GetNextCell(float *POutput);

template <typename T>
static uint8_t TryLoadParam(char* Token, const char* Name, T *Ptr) {
    if(strcasecmp(Token, Name) == 0) {
        if(csv::GetNextCell<T>(Ptr) == retvOk) return retvOk;
        else Printf("%S load fail\r", Name);
    }
    return retvFail;
}

uint8_t TryLoadParam(char* Token, const char* Name, float *Ptr);

uint8_t TryLoadString(char* Token, const char* Name, char *Dst, uint32_t MaxLen);

} // namespace
