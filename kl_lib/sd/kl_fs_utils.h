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

// Constants
#define MAX_NAME_LEN        128UL

// Variables
extern FILINFO FileInfo;
extern DIR Dir;
extern FIL CommonFile;
//extern

uint8_t TryOpenFileRead(const char *Filename, FIL *PFile);
uint8_t TryOpenFileRewrite(const char *Filename, FIL *PFile);
void CloseFile(FIL *PFile);
uint8_t CheckFileNotEmpty(FIL *PFile);
uint8_t TryRead(FIL *PFile, void *Ptr, uint32_t Sz);
static inline bool FileIsOpen(FIL *PFile) {
    return (PFile->obj.fs != 0);
}

template <typename T>
uint8_t TryRead(FIL *PFile, T *Ptr) {
    uint32_t ReadSz=0;
    uint8_t r = f_read(PFile, Ptr, sizeof(T), &ReadSz);
    return (r == FR_OK and ReadSz == sizeof(T))? retvOk : retvFail;
}

uint8_t ReadLine(FIL *PFile, char* S, uint32_t MaxLen);

bool DirExists(const char* DirName);
bool DirExistsAndContains(const char* DirName, const char* Extension);
uint8_t CountFilesInDir(const char* DirName, const char* Extension, uint32_t *PCnt);
uint8_t CountDirsStartingWith(const char* Path, const char* DirNameStart, uint32_t *PCnt);

#if 1 // ========================= GetRandom from dir ==========================
struct DirRandData_t {
    char Name[MAX_NAME_LEN];
    uint32_t LastN;
    uint32_t FileCnt = 0;
};

#define DIR_CNT   9

class DirList_t {
private:
    DirRandData_t Dirs[DIR_CNT];
    int32_t DirCnt = 0;
    int32_t CurrIndx = 0;
    uint8_t FindDirInList(char* DirName) {
        CurrIndx = 0;
        for(int32_t i=0; i<DirCnt; i++) {
            if(strcmp(DirName, Dirs[i].Name) == 0) {
                CurrIndx = i;
                return retvOk;
            }
        }
        return retvNotFound;
    }
    void AddDir(char* DirName) {
        if(DirCnt >= DIR_CNT) DirCnt = 0;
        CurrIndx = DirCnt;
        DirCnt++;
        strcpy(Dirs[CurrIndx].Name, DirName);
    }
    void CountFiles(const char* Ext) {
        CountFilesInDir(Dirs[CurrIndx].Name, Ext, &Dirs[CurrIndx].FileCnt);
    }
public:
    void Reset() {
        DirCnt = 0;
        CurrIndx = 0;
        Dirs[0].FileCnt = 0;
        Dirs[0].LastN = 0;
    }

    uint8_t GetRandomFnameFromDir(char* DirName, char* AFname) {
        // Check if dir in list
        if(FindDirInList(DirName) != retvOk) { // No such dir
    //        Printf("No Dir %S in list\r" , DirName);
            AddDir(DirName);
            CountFiles("wav");  // Count files in dir
        }
        uint32_t Cnt = Dirs[CurrIndx].FileCnt;
        if(Cnt == 0) return retvFail; // Get out if nothing to play
        // Select number of file
        uint32_t N = 0;
        uint32_t LastN = Dirs[CurrIndx].LastN;
        if(Cnt > 1) { // Get random number if count > 1
            do {
                N = Random::Generate(0, Cnt-1); // [0; Cnt-1]
            } while(N == LastN);   // skip same as previous
        }
        Dirs[CurrIndx].LastN = N;
    //    Printf("Dir %S: N=%u\r", DirName, N);
        // Iterate files in dir until success
        Cnt = 0;
        uint8_t Rslt = f_opendir(&Dir, DirName);
        if(Rslt != FR_OK) return retvFail;
        while(true) {
            Rslt = f_readdir(&Dir, &FileInfo);
            if(Rslt != FR_OK) return retvFail;
            if((FileInfo.fname[0] == 0) and (FileInfo.altname[0] == 0)) return retvFail;  // somehow no files left
            else { // Filename ok, check if not dir
                if(!(FileInfo.fattrib & AM_DIR)) {
                    // Check if wav or mp3
                    char *FName = (FileInfo.fname[0] == 0)? FileInfo.altname : FileInfo.fname;
                    uint32_t Len = strlen(FName);
                    if(Len > 4) {
                        if(strcasecmp(&FName[Len-3], "wav") == 0) {
                            if(N == Cnt) {
                                // Build full filename with path
                                // Check if root dir. Empty string allowed, too
                                int Len = strlen(DirName);
                                if((Len > 1) or (Len == 1 and *DirName != '/' and *DirName != '\\')) {
                                    strcpy(AFname, DirName);
                                    AFname[Len] = '/';
                                }
                                strcpy(&AFname[Len+1], FName);
                                return retvOk;
                            }
                            else Cnt++;
                        }
                    } // if Len>4
                } // if not dir
            } // Filename ok
        } // while true
    }
};
#endif

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
static void TryLoadParam(char* Token, const char* Name, T *Ptr) {
    if(strcasecmp(Token, Name) == 0) {
        if(csv::GetNextCell<T>(Ptr) == retvOk) {
//            if(*Ptr > MaxValue) *Ptr = MaxValue;
//            Printf("  %S: %u\r", Name, *Ptr);
        }
        else Printf("%S load fail\r", Name);
    }
}

} // namespace
