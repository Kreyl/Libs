/*
 * json_kl.h
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "color.h"

/*
  if(json::OpenFile("AuxLeds.ini") == retvOk) {
    do {
        JsonObj_t *PRoot = new JsonObj_t();
        Rslt = ReadFromFile(PRoot);
        if(Rslt == retvOk) {
            JsonObj_t &Root = *PRoot;
            AuxLedEffect_t *Eff = AppendAuxEff();
            if(Eff->FillFromJson(Root) != retvOk) RemoveLastAuxEff();
        }
        delete PRoot;
    } while(Rslt == retvOk and AuxEffects.size() < AUX_EFF_CNT_MAX);
    json::CloseFile();
} // if openfile ok
*/

namespace json { // ================== json file parsing =======================
uint8_t OpenFile(const char *AFileName);
void CloseFile() ;

enum TokenParseRslt_t {tprFail, tprContainer, tprValue, tprValueEndOfContainer, tprEmptyEndOfContainer};

class JsonObj_t {
private:
    uint8_t CopyDataToName();
    uint8_t CopyDataToValue();
public:
     // Value
    char *Name = nullptr;
    char *Value = nullptr;

    // Inner use
    JsonObj_t *Container, *Neighbor, *Child;
    TokenParseRslt_t IParse();

    // Methods
    JsonObj_t();
    ~JsonObj_t();

    bool IsEmpty() const { return (!Name and !Value and !Child and !Neighbor); }
    JsonObj_t& operator[](const char* AName) const;
    JsonObj_t& operator[](const int32_t Indx) const;
    int32_t ArrayCnt() const;
    // Values
    uint8_t ToInt(int32_t *POut) const;
    uint8_t ToByte(uint8_t *POut) const;
    uint8_t ToBool(bool *POut) const;
    uint8_t ToFloat(float *POut) const;
    // Strings
    bool ValueIsEqualTo(const char* S) const;
    uint8_t CopyValueIfNotEmpty(char** ptr) const;
    uint8_t CopyValueOrNullIfEmpty(char** ptr) const;
    // Special case
    uint8_t ToColor(Color_t *PClr) const;
    uint8_t ToTwoInts(int32_t *PInt1, int32_t *PInt2) const;
    uint8_t ToByteArray(uint8_t *PArr, int32_t Len) const;
};

uint8_t ReadFromFile(JsonObj_t *Root);

} // namespace
