/*
 * json_kl.cpp
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#include <kl_json.h>
#include "kl_fs_utils.h"

namespace json { // ================== json file parsing =======================
static const JsonObj_t EmptyNode;
static char *IBuf, *PBuf, *IString, *PStr;
static uint32_t BytesLeft;
#define JBUFSZ      4096
#define ISTRLEN     256

void CloseFile() {
    f_close(&CommonFile);
    if(IBuf) {
        free(IBuf);
        free(IString);
    }
}

uint8_t OpenFile(const char *AFileName) {
    if(TryOpenFileRead(AFileName, &CommonFile) == retvOk) {
        IBuf = (char*)malloc(JBUFSZ);
        PBuf = IBuf;
        IString = (char*)malloc(ISTRLEN);
        PStr = IString;
        if(IBuf && IString) {
            if(f_read(&CommonFile, IBuf, JBUFSZ, &BytesLeft) == FR_OK) return retvOk;
        }
        CloseFile();
    }
    return retvFail;
}

JsonObj_t::JsonObj_t() {
    Name = nullptr;
    Value = nullptr;
    Container = nullptr;
    Neighbor = nullptr;
    Child = nullptr;
}

JsonObj_t::~JsonObj_t() {
    if(Name) free(Name);
    if(Value) free(Value);
    if(Neighbor) delete Neighbor;
    if(Child) delete Child;
}

uint8_t JsonObj_t::CopyDataToName() {
    uint32_t StrLen = PStr - IString;
    if(StrLen > 0) {
        *PStr = 0; // End string
        Name = (char*)malloc(StrLen+1); // Reserve extra char for '/0';
        if(Name == nullptr) return retvFail;
        strcpy(Name, IString);
    }
    return retvOk;
}

uint8_t JsonObj_t::CopyDataToValue() {
    uint32_t StrLen = PStr - IString;
    if(StrLen > 0) {
        *PStr = 0; // End string
        Value = (char*)malloc(StrLen+1); // Reserve extra char for '/0'
        if(Value == nullptr) return retvFail;
        strcpy(Value, IString);
    } // strlen
    return retvOk;
}

__always_inline
static inline char GetChar() { return *PBuf; }
static uint8_t MoveToNextChar() {
    if(BytesLeft == 0) { // Buf is empty
        if(f_read(&CommonFile, IBuf, JBUFSZ, &BytesLeft) != FR_OK) return retvFail;
        if(BytesLeft == 0) return retvEndOfFile; // File end
        PBuf = IBuf;
    }
    else {
        PBuf++;
        BytesLeft--;
    }
    return retvOk;
}

uint8_t SkipCommentsAndWhiteSpaces() {
    bool IsInsideSingleComent = false, IsInsideLongComment = false, WasStar = false, WasSlash = false;
    while(true) {
        char c = GetChar();
        if(IsInsideSingleComent) { // Ends with line
            // Check if EOL
            if(c == '\r' or c == '\n') IsInsideSingleComent = false;
        }
        else if(IsInsideLongComment) { // ends with star and slash
            if(WasStar and c == '/') IsInsideLongComment = false;
            else WasStar = (c == '*');
        }

        // Find comments start
        else if(c == '/') {
            if(WasSlash) {
                IsInsideSingleComent = true;
                WasSlash = false;
            }
            else WasSlash = true;
        }
        else if(c == '*' and WasSlash) {
            IsInsideLongComment = true;
        }
        // Not a comment, not a whitespace
        else if(c > ' ') return retvOk;

        uint8_t r = MoveToNextChar();
        if(r != retvOk) return r;
    }
}

__always_inline
static inline bool IStrIsEmpty() { return PStr == IString; }

#define TryReturn(Retv)    return (MoveToNextChar() == retvOk)? (Retv) : tprFail;

TokenParseRslt_t JsonObj_t::IParse() {
    bool IsReadingName = true;
    bool IsInsideQuotes = false;
    PStr = IString;

    while(true) {
        if(!IsInsideQuotes) {
            if(SkipCommentsAndWhiteSpaces() != retvOk) return tprFail;
        }
        char c = GetChar();

        if(IsReadingName) {
            if(c == '\"') IsInsideQuotes = !IsInsideQuotes;
            else if(IsInsideQuotes) { // Add everything
                if(PStr < (IString + ISTRLEN-1)) *PStr++ = c;
                else return tprFail; // too long name
            }
            else if(c == ',') { // End of token
                if(!IStrIsEmpty()) { // This is not a pair, but a single value => node without name
                    // Save string to Value
                    if(CopyDataToValue() != retvOk) return tprFail;
                    TryReturn(tprValue);
                }
            }
            else if(c == ']' or c == '}') {
                if(IStrIsEmpty()) {
                    TryReturn(tprEmptyEndOfContainer);
                }
                else { // This is not a pair, but a single value => node without name
                    // Save string to Value
                    if(CopyDataToValue() != retvOk) return tprFail;
                    TryReturn(tprValueEndOfContainer);
                }
            }
            else if(c == '{' or c == '[') {
                if(CopyDataToName() != retvOk) return tprFail;
                TryReturn(tprContainer);
            }
            else if(c == ':') {
                if(CopyDataToName() != retvOk) return tprFail;
                PStr = IString; // Reset string buffer
                IsReadingName = false;
            }
            // Just char, add it to name
            else {
                if(PStr < (IString + ISTRLEN-1)) *PStr++ = c;
                else return tprFail; // too long name
            }
        } // Reading name

        // Reading value
        else {
            if(c == '\"') IsInsideQuotes = !IsInsideQuotes;
            else if(IsInsideQuotes) { // Add everything
                if(PStr < (IString + ISTRLEN-1)) *PStr++ = c;
                else return tprFail; // too long name
            }
            else if(c == ',') { // End of value
                if(CopyDataToValue() != retvOk) return tprFail;
                TryReturn(tprValue);
            }
            else if(c == ']' or c == '}') { // End of value and container
                if(IStrIsEmpty()) {
                    TryReturn(tprEmptyEndOfContainer);
                }
                else {
                    if(CopyDataToValue() != retvOk) return tprFail;
                    TryReturn(tprValueEndOfContainer);
                }
            }
            else if(c == '{' or c == '[') {
                TryReturn(tprContainer);
            }
            // Just char, add it to value
            else {
                if(PStr < (IString + ISTRLEN-1)) *PStr++ = c;
                else return tprFail; // too long name
            }
        }
        if(MoveToNextChar() != retvOk) return tprFail;
    } // while true
}

uint8_t ReadFromFile(JsonObj_t *Root) {
    JsonObj_t *Node = Root, *Parent = nullptr, *Container = nullptr;

    while(true) {
        if(Node == nullptr) Node = new JsonObj_t();
        TokenParseRslt_t ParseRslt = Node->IParse();

        // Add node to parent if not empty
        if(ParseRslt != tprEmptyEndOfContainer) {
            if(Parent) {
                if(!Parent->Value and !Parent->Child) Parent->Child = Node;
                else Parent->Neighbor = Node;
            }
            Node->Container = Container;
        }

        // Handle tree
        switch(ParseRslt) {
            case tprContainer:
                Container = Node; // go deeper
                Parent = Node;
                Node = nullptr; // Next time, create new node
                break;

            case tprValue:
                Parent = Node;
                Node = nullptr; // Next time, create new node
                break;

            case tprValueEndOfContainer: // min: 18}
                Parent = Container;
                Container = Container->Container;
                if(!Container) return retvOk;
                Node = nullptr; // Next time, create new node
                break;

            case tprEmptyEndOfContainer:
                Parent = Container;
                Container = Container->Container;
                if(!Container) {
                    delete Node; // not used anywhere
                    return retvOk;
                }
                break;

            case tprFail: return retvFail; break;
        } // switch
    } // while
}

JsonObj_t& JsonObj_t::operator[](const char* AName) const {
    JsonObj_t *Node = Child;
    while(Node != nullptr) {
        if(strcasecmp(AName, Node->Name) == 0) return *Node;
        Node = Node->Neighbor;
    }
    return *((JsonObj_t*)&EmptyNode);
}

JsonObj_t& JsonObj_t::operator[](const int32_t Indx) const {
    JsonObj_t *Node = Child;
    int32_t Cnt = 0;
    while(Node != nullptr) {
        if(Cnt == Indx) return *Node;
        else {
            Node = Node->Neighbor;
            Cnt++;
        }
    }
    return *((JsonObj_t*)&EmptyNode);
}


int32_t JsonObj_t::ArrayCnt() const {
    int32_t Rslt = 0;
    JsonObj_t *Node = Child;
    while(Node) {
        Rslt++;
        Node = Node->Neighbor;
    }
    return Rslt;
}

#if 1 // ==== Values ====
//uint8_t JsonObj_t::ToString(char *S) const {

// Try to convert
uint8_t JsonObj_t::ToInt(int32_t *POut) const {
    if(Value) {
        char *p;
        int32_t Rslt = strtol(Value, &p, 0);
        if(*p == 0) {
            *POut = Rslt;
            return retvOk;
        }
        else return retvNotANumber;
    }
    return retvFail;
}

uint8_t JsonObj_t::ToByte(uint8_t *POut) const {
    int32_t tmp;
    if(ToInt(&tmp) == retvOk) {
        *POut = (uint8_t)tmp;
        return retvOk;
    }
    else return retvFail;
}

uint8_t JsonObj_t::ToBool(bool *POut) const {
    if(Value) {
        if(Value[0] == '0' or Value[0] == 'f' or Value[0] == 'F') *POut = false;
        else *POut = true;
        return retvOk;
    }
    else return retvNotANumber;
}

uint8_t JsonObj_t::ToFloat(float *POut) const {
    if(Value) {
        char *p;
        float Rslt = strtof(Value, &p);
        if(*p == 0) {
            *POut = Rslt;
            return retvOk;
        }
    }
    return retvNotANumber;
}

// ==== Strings ====
bool JsonObj_t::ValueIsEqualTo(const char* S) const {
    return (strcasecmp(Value, S) == 0);
}

uint8_t JsonObj_t::CopyValueIfNotEmpty(char **ptr) const {
    if(Value == nullptr) return retvEmpty;
    *ptr = (char*)malloc(strlen(Value) + 1);
    if(*ptr == nullptr) return retvFail;
    strcpy(*ptr, Value);
    return retvOk;
}

uint8_t JsonObj_t::CopyValueOrNullIfEmpty(char **ptr) const {
    if(Value == nullptr) {
        *ptr = nullptr;
        return retvOk;
    }
    else {
        *ptr = (char*)malloc(strlen(Value) + 1);
        if(*ptr == nullptr) return retvFail;
        strcpy(*ptr, Value);
        return retvOk;
    }
}


// ==== Special case ====
uint8_t JsonObj_t::ToColor(Color_t *PClr) const {
    // Check if random
    if(ValueIsEqualTo("random")) PClr->BeRandom();
    // Not random
    else {
        if(Child == nullptr) return retvFail;   // R
        if(Child->Neighbor == nullptr) return retvFail; // G
        if(Child->Neighbor->Neighbor == nullptr) return retvFail; // B

        if(Child->ToByte(&PClr->R) != retvOk) return retvFail;
        if(Child->Neighbor->ToByte(&PClr->G) != retvOk) return retvFail;
        if(Child->Neighbor->Neighbor->ToByte(&PClr->B) != retvOk) return retvFail;
        PClr->Brt = 0; // not random
    }
    return retvOk;
}

uint8_t JsonObj_t::ToTwoInts(int32_t *PInt1, int32_t *PInt2) const {
    if(Child == nullptr) return retvFail;
    if(Child->Neighbor == nullptr) return retvFail;
    if(Child->ToInt(PInt1) != retvOk) return retvFail;
    if(Child->Neighbor->ToInt(PInt2) != retvOk) return retvFail;
    return retvOk;
}

uint8_t JsonObj_t::ToByteArray(uint8_t *PArr, int32_t Len) const {
    int32_t Cnt = 0;
    JsonObj_t *Node = Child;
    while(Node and Cnt < Len) {
        if(Node->ToByte(&PArr[Cnt]) != retvOk) return retvFail;
        Cnt++;
        Node = Node->Neighbor;
    }
    return (Cnt == Len)? retvOk : retvFail;
}

#endif

} // namespace
