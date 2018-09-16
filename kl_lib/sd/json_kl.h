/*
 * json_kl.h
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"
#include "color.h"

namespace json { // ================== json file parsing =======================
uint8_t OpenFile(const char *AFileName);
void CloseFile() ;

class JsonObj_t {
private:
    char* GetNextArrayStr(char **SavePtr) const;
    uint8_t GetNextArrayByte(uint8_t *POut, char **SavePtr) const;
public:
    char* Name;
    char* Value;
    JsonObj_t *Neighbor;
    JsonObj_t *Child;
    JsonObj_t *Parent;
    void Reset() {
        Name = nullptr;
        Value = nullptr;
        Neighbor = nullptr;
        Child = nullptr;
        Parent = nullptr;
    }
//    void Print() const {
//        if(Name) Printf("N: %S; ", Name);
//        else Printf("EmptyNode\r");
//        if(Value) Printf("V: %S; ", Value);
//        if(Parent)   if(Parent->Name)   Printf(" P: %S;", Parent->Name);
//        if(Child)    if(Child->Name)    Printf(" C: %S;", Child->Name);
//        if(Neighbor) if(Neighbor->Name) Printf(" N: %S",  Neighbor->Name);
//        PrintfEOL();
//    }
//    void PrintAll() const;
    JsonObj_t() : Name(nullptr), Value(nullptr), Neighbor(nullptr), Child(nullptr), Parent(nullptr) {}

    bool IsEmpty() const { return (!Name and !Value and !Child and !Neighbor); }
    JsonObj_t& operator[](const char* AName) const;
    // Values
    uint8_t ToInt(int32_t *POut) const;
    uint8_t ToColor(Color_t *PClr) const;
    uint8_t ToBool(bool *POut) const;
    uint8_t ToFloat(float *POut) const;
    void ToString(char *S, uint32_t MaxLen) const;
};

JsonObj_t& Read();

void Free(JsonObj_t& Root);

} // namespace
