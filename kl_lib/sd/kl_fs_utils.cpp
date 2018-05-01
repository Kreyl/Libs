/*
 * kl_fs_common.cpp
 *
 *  Created on: 30 џэт. 2016 у.
 *      Author: Kreyl
 */

#include "kl_fs_utils.h"
#include "kl_lib.h"
#include "shell.h"

// Variables
FILINFO FileInfo;
DIR Dir;
FIL CommonFile;
char IStr[SD_STRING_SZ];

#if 1 // ============================== Common =================================
uint8_t TryOpenFileRead(const char *Filename, FIL *PFile) {
//    Printf("%S: %S\r", __FUNCTION__, Filename);
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
        if (rslt == FR_NO_FILE) Printf("%S: not found\r", Filename);
        else Printf("OpenFile error: %u\r", rslt);
        return retvFail;
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

bool DirExists(const char* DirName) {
    FRESULT Rslt = f_opendir(&Dir, DirName);
    return (Rslt == FR_OK);
}

bool DirExistsAndContains(const char* DirName, const char* Extension) {
    if(f_opendir(&Dir, DirName) == FR_OK) {
        while(true) {
            *FileInfo.fname = 0;    // }
            *FileInfo.altname = 0;  // } Empty names before reading
            if(f_readdir(&Dir, &FileInfo) != FR_OK) return false;
            if((FileInfo.fname[0] == 0) and (FileInfo.altname[0] == 0)) return false;   // No files left
            else { // Filename ok, check if not dir
                if(!(FileInfo.fattrib & AM_DIR)) {
                    // Check the ext
                    char *FName = (FileInfo.fname[0] == 0)? FileInfo.altname : FileInfo.fname;
                    uint32_t Len = strlen(FName);
                    if(Len > 4) {
                        if(strncasecmp(&FName[Len-3], Extension, 3) == 0) return true;
                    } // if Len>4
                } // if not dir
            } // Filename ok
        } // while
    }
    else return false;
}

uint8_t CountFilesInDir(const char* DirName, const char* Extension, uint32_t *PCnt) {
    *PCnt = 0;
    FRESULT Rslt = f_opendir(&Dir, DirName);
//    Printf("f_opendir %S: %u\r", DirName, Rslt);
    if(Rslt != FR_OK) return retvFail;
    while(true) {
        *FileInfo.fname = 0;    // }
        *FileInfo.altname = 0;  // } Empty names before reading
        Rslt = f_readdir(&Dir, &FileInfo);
        if(Rslt != FR_OK) return retvFail;
        if((FileInfo.fname[0] == 0) and (FileInfo.altname[0] == 0)) return retvOk;   // No files left
        else { // Filename ok, check if not dir
            if(!(FileInfo.fattrib & AM_DIR)) {
                // Check Ext
                char *FName = (FileInfo.fname[0] == 0)? FileInfo.altname : FileInfo.fname;
//                Printf("%S\r", FName);
                uint32_t Len = strlen(FName);
                if(Len > 4) {
                    if(strncasecmp(&FName[Len-3], Extension, 3) == 0) (*PCnt)++;
                } // if Len>4
            } // if not dir
        } // Filename ok
    }
    return retvOk;
}

uint8_t CountDirsStartingWith(const char* Path, const char* DirNameStart, uint32_t *PCnt) {
    FRESULT Rslt = f_opendir(&Dir, Path);
    if(Rslt != FR_OK) return retvFail;
    *PCnt = 0;
    int Len = strlen(DirNameStart);
    while(true) {
        *FileInfo.fname = 0;    // }
        *FileInfo.altname = 0;  // } Empty names before reading
        Rslt = f_readdir(&Dir, &FileInfo);
        if(Rslt != FR_OK) return retvFail;
        if((FileInfo.fname[0] == 0) and (FileInfo.altname[0] == 0)) return retvOk;   // Nothing left
        else { // Filename ok, check if dir
            if(FileInfo.fattrib & AM_DIR) {
                // Check if starts with DirNameStart
                char *FName = (FileInfo.fname[0] == 0)? FileInfo.altname : FileInfo.fname;
//                Printf("%S\r", FName);
                if(strncasecmp(FName, DirNameStart, Len) == 0) (*PCnt)++;
            } // if dir
        } // Filename ok
    }
    return retvOk;
}
#endif

namespace ini { // =================== ini file operations =====================
void WriteSection(FIL *PFile, const char *ASection) {
    f_printf(PFile, "[%S]\r\n", ASection);
}
void WriteString(FIL *PFile, const char *AKey, char *AValue) {
    f_printf(PFile, "%S=%S\r\n", AKey, AValue);
}
void WriteInt32(FIL *PFile, const char *AKey, const int32_t AValue) {
    f_printf(PFile, "%S=%D\r\n", AKey, AValue);
}
void WriteNewline(FIL *PFile) {
    f_putc('\r', PFile);
    f_putc('\n', PFile);
}


static inline char* skipleading(char *S) {
    while (*S != '\0' && *S <= ' ') S++;
    return S;
}
static inline char* skiptrailing(char *S, const char *base) {
    while ((S > base) && (*(S-1) <= ' ')) S--;
    return S;
}
static inline char* striptrailing(char *S) {
    char *ptr = skiptrailing(strchr(S, '\0'), S);
    *ptr='\0';
    return S;
}

uint8_t ReadString(const char *AFileName, const char *ASection, const char *AKey, char **PPOutput) {
    FRESULT rslt;
//    Printf("%S %S %S\r", __FUNCTION__, AFileName, ASection);
    // Open file
    rslt = f_open(&CommonFile, AFileName, FA_READ+FA_OPEN_EXISTING);
    if(rslt != FR_OK) {
        if (rslt == FR_NO_FILE) Printf("%S: not found\r", AFileName);
        else Printf("%S: openFile error: %u\r", AFileName, rslt);
        return retvFail;
    }
    // Check if zero file
    if(f_size(&CommonFile) == 0) {
        f_close(&CommonFile);
        Printf("Empty file\r");
        return retvFail;
    }
    // Move through file one line at a time until a section is matched or EOF.
    char *StartP, *EndP = nullptr;
    int32_t len = strlen(ASection);
    do {
        if(f_gets(IStr, SD_STRING_SZ, &CommonFile) == nullptr) {
            Printf("iniNoSection %S\r", ASection);
            f_close(&CommonFile);
            return retvFail;
        }
        StartP = skipleading(IStr);
        if((*StartP != '[') or (*StartP == ';') or (*StartP == '#')) continue;
        EndP = strchr(StartP, ']');
        if((EndP == NULL) or ((int32_t)(EndP-StartP-1) != len)) continue;
    } while (strncmp(StartP+1, ASection, len) != 0);

    // Section found, find the key
    len = strlen(AKey);
    do {
        if(!f_gets(IStr, SD_STRING_SZ, &CommonFile) or *(StartP = skipleading(IStr)) == '[') {
            Printf("iniNoKey\r");
            f_close(&CommonFile);
            return retvFail;
        }
        StartP = skipleading(IStr);
        if((*StartP == ';') or (*StartP == '#')) continue;
        EndP = strchr(StartP, '=');
        if(EndP == NULL) continue;
    } while(((int32_t)(skiptrailing(EndP, StartP)-StartP) != len or strncmp(StartP, AKey, len) != 0));
    f_close(&CommonFile);

    // Process Key's value
    StartP = skipleading(EndP + 1);
    // Remove a trailing comment
    uint8_t isstring = 0;
    for(EndP = StartP; (*EndP != '\0') and (((*EndP != ';') and (*EndP != '#')) or isstring) and ((uint32_t)(EndP - StartP) < SD_STRING_SZ); EndP++) {
        if (*EndP == '"') {
            if (*(EndP + 1) == '"') EndP++;     // skip "" (both quotes)
            else isstring = !isstring; // single quote, toggle isstring
        }
        else if (*EndP == '\\' && *(EndP + 1) == '"') EndP++; // skip \" (both quotes)
    } // for
    *EndP = '\0';   // Terminate at a comment
    striptrailing(StartP);
    *PPOutput = StartP;
    return retvOk;
}
} // Namespace

namespace csv { // =================== csv file operations =====================
#define CSV_DELIMITERS  " ,;={}\t\r\n"
static char *csvCurToken;

uint8_t OpenFile(const char *AFileName) {
    return TryOpenFileRead(AFileName, &CommonFile);
}
void RewindFile() {
    f_lseek(&CommonFile, 0);
}
void CloseFile() {
    f_close(&CommonFile);
}

/* Skips empty and comment lines (starting with #).
 * Returns retvOk if not-a-comment line read, retvEndOfFile if eof
 */
uint8_t ReadNextLine() {
    // Move through file until comments end
    while(true) {
        if(f_eof(&CommonFile) or f_gets(IStr, SD_STRING_SZ, &CommonFile) == nullptr) {
//            Printf("csvNoMoreData\r");
            return retvEndOfFile;
        }
        csvCurToken = strtok(IStr, CSV_DELIMITERS);
        if(*csvCurToken == '#' or *csvCurToken == 0) continue; // Skip comments and empty lines
        else return retvOk;
    }
}

uint8_t GetNextToken(char** POutput) {
    // First time, return csvCurToken, as it was set in ReadNextLine
    if(csvCurToken != nullptr) {
        *POutput = csvCurToken;
        csvCurToken = nullptr;
    }
    else *POutput = strtok(NULL, CSV_DELIMITERS);
    return (*POutput == '\0')? retvEmpty : retvOk;
}

uint8_t GetNextCellString(char* POutput) {
    char *Token;
    if(GetNextToken(&Token) == retvOk) {
        // Skip leading quotes
        char *StartP = Token;
        while(*StartP == '"' and *StartP != '\0') StartP++;
        char *EndP = Token + strlen(Token) - 1;
        while(*EndP == '"' and EndP > Token) EndP--;
        int32_t Len = EndP - StartP + 1;
        if(Len > 0) strncpy(POutput, StartP, Len);
        else *POutput = '\0';
        return retvOk;
    }
    else return retvFail;
}

uint8_t FindFirstCell(const char* Name) {
    while(true) {
        if(ReadNextLine() != retvOk) break;
//        Printf("Token: %S\r", csvCurToken);
        if(strcasecmp(csvCurToken, Name) == 0) {
            csvCurToken = strtok(NULL, CSV_DELIMITERS);
            return retvOk;
        }
    }
    return retvNotFound;
}

uint8_t GetNextCell(float *POutput) {
    char *Token;
    if(GetNextToken(&Token) == retvOk) {
        char *p;
        *POutput = strtof(Token, &p);
        if(*p == '\0') return retvOk;
        else return retvNotANumber;
    }
    else return retvEmpty;
}

} // namespace
