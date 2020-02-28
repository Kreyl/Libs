/*
 * json_kl.h
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "color.h"

#define JSON_FILE_EN    FALSE
#if JSON_FILE_EN
#include "ff.h"
#endif

#define ISTRLEN     256

enum TokenParseRslt_t {tprFail, tprBufEnd, tprContainer, tprValue, tprValueEndOfContainer, tprEmptyEndOfContainer};

class JsonObj_t {
private:
#if JSON_FILE_EN
    uint8_t ISaveToFile(FIL *AFile, bool SaveNeighbor);
#endif
public:
    char *Name = nullptr;
    char *Value = nullptr;
    bool IsArray = false;
    bool IsContainer = false;

    // Inner use
    JsonObj_t *Container = nullptr, *Neighbor = nullptr, *Child = nullptr;

    // Methods
    JsonObj_t() {}
    JsonObj_t(const char* AName, const char* AValue);
    ~JsonObj_t();
    void Clear();

    bool IsEmpty() const { return (!Name and !Value and !Child and !Neighbor); }
    JsonObj_t& operator[](const char* AName) const;
    JsonObj_t& operator[](const int32_t Indx) const;
    void AddNeighbor(JsonObj_t *PNode);
    void DeleteChild(const char* AName);
    int32_t ArrayCnt() const;
    // Values
    uint8_t ToInt(int32_t *POut) const;
    uint8_t ToUint(uint32_t *POut) const;
    uint8_t ToByte(uint8_t *POut) const;
    uint8_t ToBool(bool *POut) const;
    bool ToBool() const;
    uint8_t ToFloat(float *POut) const;
    uint8_t SetNewValue(const char* NewValue);
    uint8_t SetNewValue(int32_t NewValue);
    uint8_t SetNewValue(bool NewValue);
    // Strings
    bool NameIsEqualTo(const char* S) const;
    bool ValueIsEqualTo(const char* S) const;
    uint8_t CopyValueIfNotEmpty(char** ptr) const;
    uint8_t CopyValueIfNotEmpty(char *PBuf, uint32_t SzMax) const;
    uint8_t CopyValueOrNullIfEmpty(char** ptr) const;
    // Special case
    uint8_t ToColor(Color_t *PClr) const;
    uint8_t ToTwoInts(int32_t *PInt1, int32_t *PInt2) const;
    uint8_t ToByteArray(uint8_t *PArr, int32_t Len) const;
#if JSON_FILE_EN    // Save to file
    uint8_t SaveToFile(FIL *AFile);
#endif
};

class JsonParser_t {
private:
    void InitContext();
    JsonObj_t *Node = nullptr, *Parent = nullptr, *Container = nullptr;
    // State
    char *PBuf = nullptr;
    char IString[ISTRLEN];
    uint32_t Indx;
    bool IStrIsEmpty() { return Indx == 0; }
    uint8_t CopyStringTo(char **Dst);
    uint32_t BytesLeft = 0;
    void MoveToNextChar() {
        if(BytesLeft > 0) {
            PBuf++;
            BytesLeft--;
        }
    }
    uint8_t SkipCommentsAndWhiteSpaces();
    bool IsReadingName;
    bool IsInsideQuotes;
    // Comments
    bool IsInsideSingleComent = false;
    bool IsInsideLongComment = false;
    bool WasStar = false;
    bool WasSlash = false;
    TokenParseRslt_t IParse();
#if JSON_FILE_EN   // File operations
    FIL *IFile = nullptr;
#endif
    char *IBuf = nullptr;
public:
    JsonObj_t Root;
    // Files
    uint8_t StartReadFromFile(const char* AFName); // Call this to start reading
    uint8_t ReadNewFromFile(); // Read new object from file
    uint8_t SaveToSameFile();
    // Buffers
    uint8_t StartReadFromBuf(char* ABuf, uint32_t ALen); // call this to read anew obj
    uint8_t ContinueToReadBuf(char* ABuf, uint32_t ALen); // call this with new buf if result was retvEmpty

    ~JsonParser_t();
};

#if JSON_FILE_EN
uint8_t JsonGetNodeValue(const char* Filename, const char* NodeName, char** PPValue);
#endif
