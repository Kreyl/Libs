/*
 * kl_fs_common.cpp
 *
 *  Created on: 30 џэт. 2016 у.
 *      Author: Kreyl
 */

#include "kl_fs_utils.h"
#include "kl_lib.h"
#include "shell.h"

#if 1 // ============================== Common =================================
uint8_t TryOpenFileRead(const char *Filename, FIL *PFile) {
//    Printf("%S: %S; %X\r", __FUNCTION__, Filename, PFile);
    FRESULT rslt = f_open(PFile, Filename, FA_READ);
    if(rslt == FR_OK) {
        // Check if zero file
        if(f_size(PFile) == 0) {
            f_close(PFile);
            Printf("Empty file %S\r", Filename);
            return retvEmpty;
        }
        return retvOk;
    }
    else {
        if (rslt == FR_NO_FILE or rslt == FR_NO_PATH) Printf("%S: not found\r", Filename);
        else {
            Printf("OpenFile error: %u\r", rslt);
        }
        return retvFail;
    }
}

// Open existing
uint8_t TryOpenFileReadWrite(const char *Filename, FIL *PFile) {
//    Printf("%S: %S; %X\r", __FUNCTION__, Filename, PFile);
    FRESULT rslt = f_open(PFile, Filename, FA_READ+FA_WRITE);
    if(rslt == FR_OK) return retvOk;
    else {
        if(rslt == FR_NO_FILE or rslt == FR_NO_PATH) {
//            Printf("%S: not found\r", Filename);
            return retvNotFound;
        }
        else {
            Printf("OpenFile error: %u\r", rslt);
            return retvFail;
        }
    }
}

uint8_t TryOpenFileRewrite(const char *Filename, FIL *PFile) {
    FRESULT rslt = f_open(PFile, Filename, FA_WRITE+FA_CREATE_ALWAYS);
    if(rslt == FR_OK) return retvOk;
    else {
        Printf("%S open error: %u\r", Filename, rslt);
        return retvFail;
    }
}

void CloseFile(FIL *PFile) {
    f_close(PFile);
}

uint8_t CheckFileNotEmpty(FIL *PFile) {
    if(f_size(PFile) == 0) {
        Printf("Empty file\r");
        return retvEmpty;
    }
    else return retvOk;
}

uint8_t TryRead(FIL *PFile, void *Ptr, uint32_t Sz) {
    uint32_t ReadSz=0;
    uint8_t r = f_read(PFile, Ptr, Sz, &ReadSz);
    return (r == FR_OK and ReadSz == Sz)? retvOk : retvFail;
}

uint8_t ReadLine(FIL *PFile, char* S, uint32_t MaxLen) {
    uint32_t Len = 0, Rcv;
    char c, str[2];
    while(Len < MaxLen-1) {
        if(f_read(PFile, str, 1, &Rcv) != FR_OK) return retvFail;
        if(Rcv != 1) return retvEndOfFile;
        c = str[0];
        if(c == '\r' or c == '\n') {    // End of line
            *S = '\0';
            return retvOk;
        }
        else {
            *S++ = c;
            Len++;
        }
    } // while
    *S = '\0';
    return retvOk;
}

// No subdirs!
uint8_t RemoveDir(const char* DirName) {
    char FFName[72];
    DIR Dir;
    FRESULT Rslt = f_opendir(&Dir, DirName);
    if(Rslt == FR_NO_PATH) return retvNotFound;
    else if(Rslt != FR_OK) return retvFail;
    // Iterate files in dir
    FILINFO FileInfo;
    while(true) {
        *FileInfo.fname = 0;
        Rslt = f_readdir(&Dir, &FileInfo);
        if(Rslt != FR_OK) return retvFail;
        if(FileInfo.fname[0] == 0) break;   // No files left
        else { // Filename ok
            if(FileInfo.fattrib & AM_DIR) return retvFail; // No subdirs!
            // Construct Full filename
            strcpy(FFName, DirName);
            strcat(FFName, "/");
            strcat(FFName, FileInfo.fname);
            // Delete file
            Rslt = f_unlink(FFName);
            if(Rslt != FR_OK) return retvFail;
        }
    } // while
    // Dir is empty now
    f_closedir(&Dir);
    Rslt = f_unlink(DirName);
    if(Rslt == FR_OK) return retvOk;
    else return retvFail;
}

uint8_t RemoveDir1AndRenameDir2AsDir1(const char* DirName1, const char* DirName2) {
    // Delete old dir
    if(RemoveDir(DirName1) != retvOk) return retvFail;
    // rename
    if(f_rename(DirName2, DirName1) == FR_OK) return retvOk;
    else return retvFail;
}

//bool DirExists(const char* DirName) {
//    FRESULT Rslt = f_opendir(&Dir, DirName);
//    return (Rslt == FR_OK);
//}

//bool DirExistsAndContains(const char* DirName, const char* Extension) {
//    if(f_opendir(&Dir, DirName) == FR_OK) {
//        while(true) {
//            // Empty names before reading
//            *FileInfo.fname = 0;
//#if _USE_LFN
//            *FileInfo.altname = 0;
//#endif
//            if(f_readdir(&Dir, &FileInfo) != FR_OK) return false;
//            if((FileInfo.fname[0] == 0)
//#if _USE_LFN
//                    and (FileInfo.altname[0] == 0)
//#endif
//            ) return false;   // No files left
//            else { // Filename ok, check if not dir
//                if(!(FileInfo.fattrib & AM_DIR)) {
//                    // Check the ext
//#if _USE_LFN
//                    char *FName = (FileInfo.fname[0] == 0)? FileInfo.altname : FileInfo.fname;
//#else
//                    char *FName = FileInfo.fname;
//#endif
//                    uint32_t Len = strlen(FName);
//                    if(Len > 4) {
//                        if(strncasecmp(&FName[Len-3], Extension, 3) == 0) return true;
//                    } // if Len>4
//                } // if not dir
//            } // Filename ok
//        } // while
//    }
//    else return false;
//}

//uint8_t CountFilesInDir(const char* DirName, const char* Extension, int32_t *PCnt) {
//    *PCnt = 0;
//    FRESULT Rslt = f_opendir(&Dir, DirName);
////    Printf("f_opendir %S: %u\r", DirName, Rslt);
//    if(Rslt != FR_OK) return retvFail;
//    while(true) {
//        // Empty names before reading
//        *FileInfo.fname = 0;
//        Rslt = f_readdir(&Dir, &FileInfo);
//        if(Rslt != FR_OK) return retvFail;
//        if(FileInfo.fname[0] == 0) return retvOk;   // No files left
//        else { // Filename ok, check if not dir
//            if(!(FileInfo.fattrib & AM_DIR)) {
//                // Check Ext
//                char *FName = FileInfo.fname;
////                Printf("%S\r", FName);
//                uint32_t Len = strlen(FName);
//                if(Len > 4) {
//                    if(strncasecmp(&FName[Len-3], Extension, 3) == 0) (*PCnt)++;
//                } // if Len>4
//            } // if not dir
//        } // Filename ok
//    }
//    return retvOk;
//}

//uint8_t CountDirsStartingWith(const char* Path, const char* DirNameStart, uint32_t *PCnt) {
//    FRESULT Rslt = f_opendir(&Dir, Path);
//    if(Rslt != FR_OK) return retvFail;
//    *PCnt = 0;
//    int Len = strlen(DirNameStart);
//    while(true) {
//        // Empty names before reading
//        *FileInfo.fname = 0;
//#if _USE_LFN
//        *FileInfo.altname = 0;
//#endif
//        Rslt = f_readdir(&Dir, &FileInfo);
//        if(Rslt != FR_OK) return retvFail;
//        if((FileInfo.fname[0] == 0)
//#if _USE_LFN
//                    and (FileInfo.altname[0] == 0)
//#endif
//        ) return retvOk;   // Nothing left
//        else { // Filename ok, check if dir
//            if(FileInfo.fattrib & AM_DIR) {
//                // Check if starts with DirNameStart
//#if _USE_LFN
//                char *FName = (FileInfo.fname[0] == 0)? FileInfo.altname : FileInfo.fname;
//#else
//                char *FName = FileInfo.fname;
//#endif
////                Printf("%S\r", FName);
//                if(strncasecmp(FName, DirNameStart, Len) == 0) (*PCnt)++;
//            } // if dir
//        } // Filename ok
//    }
//    return retvOk;
//}
#endif
