/*
 * Soundlist.cpp
 *
 *  Created on: 18 џэт. 2015 у.
 *      Author: Kreyl
 */

#include "Soundlist.h"
#include "uart.h"
#include "sound.h"

FRESULT SndList_t::CountFilesInDir(const char* DirName, uint32_t *PCnt) {
    FRESULT Rslt = f_opendir(&Dir, DirName);
    if(Rslt != FR_OK) return Rslt;
    *PCnt = 0;
    while(true) {
        Rslt = f_readdir(&Dir, &FileInfo);
        if(Rslt != FR_OK) return Rslt;
        if((FileInfo.fname[0] == 0) and (FileInfo.lfname[0] == 0)) return FR_OK;   // No files left
        else { // Filename ok, check if not dir
            if(FileInfo.fattrib & (AM_HID | AM_DIR)) continue; // Ignore hidden files and dirs
            // Check if wav or mp3
            char *FName = (FileInfo.lfname[0] == 0)? FileInfo.fname : FileInfo.lfname;
//                Uart.Printf("\r%S", FName);
            uint32_t Len = strlen(FName);
            if(Len > 4) {
                if((strcasecmp(&FName[Len-3], "mp3") == 0) or (strcasecmp(&FName[Len-3], "wav") == 0)) (*PCnt)++;
            } // if Len>4
        } // Filename ok
    }
    return FR_OK;
}

void SndList_t::PlayRandomFileFromDir(const char* DirName) {
    int indx = DirIndxInList(DirName);
    if(indx == -1) {
        indx = AddDirToList(DirName);
        FRESULT Rslt = CountFilesInDir(DirName, &DirList[indx].FilesCnt);
        if(Rslt != FR_OK) return;
    }
//    DirList[indx].Print();
    if(DirList[indx].FilesCnt == 0) return; // Get out if nothing to play
    // Select number of file
    int32_t N = 0;
    if(DirList[indx].FilesCnt > 1) {   // Get random number if count > 1
        do {
            N = Random(0, DirList[indx].FilesCnt-1);      // [0; Cnt-1]
        } while(N == DirList[indx].LastN);    // skip same as previous
    }
//    Uart.Printf("Random=%d\r", N);
    DirList[indx].LastN = N;
    // Iterate files in dir until success
    int32_t Counter = 0;
    FRESULT Rslt = f_opendir(&Dir, DirName);
    if(Rslt != FR_OK) return;
    while(true) {
        Rslt = f_readdir(&Dir, &FileInfo);
        if(Rslt != FR_OK) return;
        if((FileInfo.fname[0] == 0) and (FileInfo.lfname[0] == 0)) return;  // somehow no files left
        else { // Filename ok, check if not dir
            if(FileInfo.fattrib & (AM_HID | AM_DIR)) continue; // Ignore hidden files and dirs
            // Check if wav or mp3
            char *FName = (FileInfo.lfname[0] == 0)? FileInfo.fname : FileInfo.lfname;
            uint32_t Len = strlen(FName);
            if(Len > 4) {
                if((strcasecmp(&FName[Len-3], "mp3") == 0) or (strcasecmp(&FName[Len-3], "wav") == 0)) {
                    if(N == Counter) {
                        // Build full filename with path
                        // Check if root dir. Empty string allowed, too
                        int Len = strlen(DirName);
                        if((Len > 1) or (Len == 1 and *DirName != '/' and *DirName != '\\')) {
                            strcpy(Filename, DirName);
                            Filename[Len] = '/';
                        }
                        strcpy(&Filename[Len+1], FName);
                        Uart.Printf("%S  N=%u\r", FName, Counter);
                        Sound.Play(Filename);
                        return;
                    }
                    else Counter++;
                }
            } // if Len>4
        } // Filename o
    } // while true
}

int SndList_t::DirIndxInList(const char* DirName) {
    for(int i=0; i < DIR_CNT_MAX; i++) {
        if(strcasecmp(DirList[i].DirName, DirName) == 0) return i;  // found
    }
    return -1; // No such dir
}

int SndList_t::AddDirToList(const char* DirName) {
    // Iterate all dirs except last one
    int indx=0;
    for(indx=0; indx < (DIR_CNT_MAX-1); indx++) {
        if(DirList[indx].DirName[0] == '\0') break;    // Empty dir
    }
    Uart.Printf("Indx=%d\r", indx);
    strcpy(DirList[indx].DirName, DirName);
    DirList[indx].LastN = -1;
    DirList[indx].FilesCnt = 0;
    return indx;
}
