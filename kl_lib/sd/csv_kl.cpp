/*
 * csv_kl.cpp
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#include "csv_kl.h"

namespace csv { // =================== csv file operations =====================
#define CSV_DELIMITERS  " ,;={}\t\r\n"
__unused static char *csvCurToken;

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

uint8_t TryLoadParam(char* Token, const char* Name, float *Ptr) {
    if(strcasecmp(Token, Name) == 0) {
        if(csv::GetNextCell(Ptr) == retvOk){
//            Printf("  %S: %f\r", Name, *Ptr);
            return retvOk;
        }
        else Printf("%S load fail\r", Name);
    }
    return retvFail;
}

uint8_t TryLoadString(char* Token, const char* Name, char *Dst, uint32_t MaxLen) {
    if(strcasecmp(Token, Name) == 0) {
        *Dst = 0;   // Empty Dst
        char *Cell;
        if(GetNextToken(&Cell) == retvOk) {
            uint32_t Len = strlen(Cell);
            if(Len < MaxLen) strcpy(Dst, Cell);
            else {
                strncpy(Dst, Cell, MaxLen-1);
                Dst[MaxLen-1] = 0;
            }
        }
        return retvOk;
    }
    return retvFail;
}

} // namespace


