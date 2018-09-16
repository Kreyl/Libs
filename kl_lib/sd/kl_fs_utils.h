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
#include "color.h"

// Constants
#define MAX_NAME_LEN        128UL
#define COMMON_STR_LEN      256UL   // Length of common use string

// Variables
extern FILINFO FileInfo;
extern DIR Dir;
extern FIL CommonFile;

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
uint8_t CountFilesInDir(const char* DirName, const char* Extension, int32_t *PCnt);
uint8_t CountDirsStartingWith(const char* Path, const char* DirNameStart, uint32_t *PCnt);

#if 1 // ========================= GetRandom from dir ==========================
struct DirRandData_t {
    char Name[MAX_NAME_LEN];
    int32_t LastN;
    int32_t FileCnt = 0;
};

#define DIR_CNT   9

class DirList_t {
private:
    DirRandData_t Dirs[DIR_CNT];
    int32_t DirCnt = 0;
    int32_t CurrIndx = 0;
    uint8_t FindDirInList(const char* DirName) {
        CurrIndx = 0;
        for(int32_t i=0; i<DirCnt; i++) {
            if(strcmp(DirName, Dirs[i].Name) == 0) {
                CurrIndx = i;
                return retvOk;
            }
        }
        return retvNotFound;
    }
    void AddDir(const char* DirName) {
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

    uint8_t GetRandomFnameFromDir(const char* DirName, char* AFname) {
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
            if((FileInfo.fname[0] == 0)
#if _USE_LFN
                    and (FileInfo.altname[0] == 0)
#endif
            ) return retvFail;  // somehow no files left
            else { // Filename ok, check if not dir
                if(!(FileInfo.fattrib & AM_DIR)) {
                    // Check if wav or mp3
#if _USE_LFN
                    char *FName = (FileInfo.fname[0] == 0)? FileInfo.altname : FileInfo.fname;
#else
                    char *FName = FileInfo.fname;
#endif
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

