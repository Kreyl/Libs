/*
 * ini_kl.cpp
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#include "ini_kl.h"

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

uint8_t ReadStringTo(const char *AFileName, const char *ASection, const char *AKey, char *POutput, uint32_t MaxLen) {
    char *S;
    if(ReadString(AFileName, ASection, AKey, &S) == retvOk) {
        // Copy what was read
        if(strlen(S) > (MaxLen-1)) {
            strncpy(POutput, S, (MaxLen-1));
            POutput[MaxLen-1] = 0;  // terminate string
        }
        else strcpy(POutput, S);
        return retvOk;
    }
    else return retvFail;
}

uint8_t HexToUint(char *S, uint8_t AMaxLength, uint32_t *AOutput) {
    *AOutput = 0;
    char c;
    uint8_t b=0;
    for(uint8_t i=0; i<AMaxLength; i++) {
        c = *S++;
        if (c == 0) return retvOk;    // end of string
        // Shift result
        *AOutput <<= 4;
        // Get next digit
        if     ((c >= '0') && (c <= '9')) b = c-'0';
        else if((c >= 'A') && (c <= 'F')) b = c-'A'+10;
        else if((c >= 'a') && (c <= 'f')) b = c-'a'+10;
        else return retvFail;  // not a hex digit
        *AOutput += b;
    }
    return retvOk;
}

uint8_t ReadColor (const char *AFileName, const char *ASection, const char *AKey, Color_t *AOutput) {
    char *S;
    if(ReadString(AFileName, ASection, AKey, &S) == retvOk) {
        if(strlen(S) != 6) return retvBadValue;
        uint32_t N=0;
        if(HexToUint(&S[0], 2, &N) != retvOk) return retvFail;
        AOutput->R = N;
        if(HexToUint(&S[2], 2, &N) != retvOk) return retvFail;
        AOutput->G = N;
        if(HexToUint(&S[4], 2, &N) != retvOk) return retvFail;
        AOutput->B = N;
        return retvOk;
    }
    else return retvFail;
}

} // Namespace


