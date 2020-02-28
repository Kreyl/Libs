/*
 * json_kl.cpp
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#include "kl_json.h"
#if JSON_FILE_EN
#include "kl_fs_utils.h"
#endif
static const JsonObj_t EmptyNode;
#define JBUFSZ      4096
#define ISTRLEN     256

#if 1 // ==== Parsing ====
uint8_t JsonParser_t::CopyStringTo(char **Dst) {
    if(Indx > 0) {
        IString[Indx] = 0; // End string
        *Dst = (char*)malloc(Indx+1); // Reserve extra char for '/0';
        if(*Dst == nullptr) return retvFail;
        strcpy(*Dst, IString);
        Indx = 0; // Reset string buffer
    }
    return retvOk;
}

uint8_t JsonParser_t::SkipCommentsAndWhiteSpaces() {
    while(true) {
        char c = *PBuf;
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

        MoveToNextChar();
        if(BytesLeft == 0) return retvEmpty;
    }
}

TokenParseRslt_t JsonParser_t::IParse() {
    bool ValueExistsAndIsEmpty = false;
    while(true) {
        if(!IsInsideQuotes) {
            uint8_t r = SkipCommentsAndWhiteSpaces();
            if(r == retvFail) return tprFail;
            else if(r == retvEmpty) return tprBufEnd;
        }
        char c = *PBuf;

        if(IsReadingName) {
            if(c == '\"') IsInsideQuotes = !IsInsideQuotes;
            else if(IsInsideQuotes) { // Add everything
                if(Indx < (ISTRLEN-1)) IString[Indx++] = c;
                else return tprFail; // too long name
            }
            else if(c == ',') { // End of token
                if(!IStrIsEmpty()) { // This is not a pair, but a single value => node without name
                    // Save string to Value
                    if(CopyStringTo(&Node->Value) != retvOk) return tprFail;
                    MoveToNextChar();
                    return tprValue;
                }
            }
            else if(c == ']' or c == '}') {
                if(IStrIsEmpty()) {
                    MoveToNextChar();
                    return tprEmptyEndOfContainer;
                }
                else { // This is not a pair, but a single value => node without name
                    // Save string to Value
                    if(CopyStringTo(&Node->Value) != retvOk) return tprFail;
                    MoveToNextChar();
                    return tprValueEndOfContainer;
                }
            }
            else if(c == '{') {
                if(CopyStringTo(&Node->Name) != retvOk) return tprFail;
                MoveToNextChar();
                return tprContainer;
            }
            else if(c == '[') {
                Node->IsArray = true;
                if(CopyStringTo(&Node->Name) != retvOk) return tprFail;
                MoveToNextChar();
                return tprContainer;
            }
            else if(c == ':') {
                if(CopyStringTo(&Node->Name) != retvOk) return tprFail;
                IsReadingName = false;
            }
            // Just char, add it to name
            else {
                if(Indx < (ISTRLEN-1)) IString[Indx++] = c;
                else return tprFail; // too long name
            }
        } // Reading name

        // Reading value
        else {
            if(c == '\"') {
                if(IsInsideQuotes) { // End of value
                    if(Indx == 0) ValueExistsAndIsEmpty = true;
                }
                IsInsideQuotes = !IsInsideQuotes;
            }
            else if(IsInsideQuotes) { // Add everything
                if(Indx < (ISTRLEN-1)) IString[Indx++] = c;
                else return tprFail; // too long value
            }
            else if(c == ',') { // End of value
                if(CopyStringTo(&Node->Value) != retvOk) return tprFail;
                MoveToNextChar();
                return tprValue;
            }
            else if(c == ']' or c == '}') { // End of value and container
                MoveToNextChar();
                if(!IStrIsEmpty()) {
                    if(CopyStringTo(&Node->Value) != retvOk) return tprFail;
                    return tprValueEndOfContainer;
                }
                else if(ValueExistsAndIsEmpty) return tprValueEndOfContainer;
                else return tprEmptyEndOfContainer;
            }
            else if(c == '{') {
                MoveToNextChar();
                return tprContainer;
            }
            else if(c == '[') {
                Node->IsArray = true;
                MoveToNextChar();
                return tprContainer;
            }
            // Just char, add it to value
            else {
                if(Indx < (ISTRLEN-1)) IString[Indx++] = c;
                else return tprFail; // too long name
            }
        }
        MoveToNextChar();
        if(BytesLeft == 0) return tprBufEnd;
    } // while true
}
#endif

#if 1 // ==== JsonObj methods ====
JsonObj_t::JsonObj_t(const char* AName, const char* AValue) {
    Name = (char*)malloc(strlen(AName) + 1);
    if(Name) strcpy(Name, AName);
    Value = (char*)malloc(strlen(AValue) + 1);
    if(Value) strcpy(Value, AValue);
    Container = nullptr;
    Neighbor = nullptr;
    Child = nullptr;
}

JsonObj_t::~JsonObj_t() {
    if(Name) {
        free(Name);
        Name = nullptr;
    }
    if(Value) {
        free(Value);
        Value = nullptr;
    }
    if(Neighbor) {
        delete Neighbor;
        Neighbor = nullptr;
    }
    if(Child) {
        delete Child;
        Child = nullptr;
    }
}

void JsonObj_t::Clear() {
    if(Name) {
        free(Name);
        Name = nullptr;
    }
    if(Value) {
        free(Value);
        Value = nullptr;
    }
    if(Neighbor) {
        delete Neighbor;
        Neighbor = nullptr;
    }
    if(Child) {
        delete Child;
        Child = nullptr;
    }
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

void JsonObj_t::AddNeighbor(JsonObj_t *PNode) {
    JsonObj_t *OldNeighbor = Neighbor;
    Neighbor = PNode;
    // Move to last neighbor of new node
    while(PNode->Neighbor) PNode = PNode->Neighbor;
    PNode->Neighbor = OldNeighbor;
}

void JsonObj_t::DeleteChild(const char* AName) {
    JsonObj_t *Node = Child;
    JsonObj_t *PrevNode = nullptr;
    while(Node != nullptr) {
        if(strcasecmp(AName, Node->Name) == 0) { // Node found
            if(PrevNode) PrevNode->Neighbor = Node->Neighbor;
            else Child = Node->Neighbor;
            Node->Neighbor = nullptr; // Do not delete it when killing Node
            delete Node;
            return;
        }
        PrevNode = Node;
        Node = Node->Neighbor;
    }
}
#endif

#if 1 // ==== Values ====
uint8_t JsonObj_t::SetNewValue(const char* NewValue) {
    if(Value) {
        free(Value);
        Value = nullptr;
    }
    if(NewValue == nullptr) return retvOk;
    uint32_t Sz = strlen(NewValue);
    if(Sz == 0) return retvOk;
    Value = (char*)malloc(Sz + 1);
    if(Value == nullptr) return retvOutOfMemory;
    strcpy(Value, NewValue);
    return retvOk;
}

uint8_t JsonObj_t::SetNewValue(int32_t NewValue) {
    char S[33];
    return SetNewValue(itoa(NewValue, S, 10));
}

uint8_t JsonObj_t::SetNewValue(bool NewValue) {
    if(NewValue) return SetNewValue("true");
    else return SetNewValue("false");
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

uint8_t JsonObj_t::ToUint(uint32_t *POut) const {
    if(Value) {
        char *p;
        uint32_t Rslt = strtoul(Value, &p, 0);
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
    if(!POut) return retvFail;
    if(Value) {
        if(Value[0] == 0 or Value[0] == '0' or Value[0] == 'f' or Value[0] == 'F') *POut = false;
        else *POut = true;
        return retvOk;
    }
    else return retvNotANumber;
}

// Returns false if false or fail
bool JsonObj_t::ToBool() const {
    if(Value) {
        if(Value[0] == 0 or Value[0] == '0' or Value[0] == 'f' or Value[0] == 'F') return false;
        else return true;
    }
    else return false;
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
bool JsonObj_t::NameIsEqualTo(const char* S) const {
    return (strcasecmp(Name, S) == 0);
}

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
uint8_t JsonObj_t::CopyValueIfNotEmpty(char *PBuf, uint32_t SzMax) const {
    if(Value == nullptr) return retvEmpty;
    if(PBuf == nullptr) return retvFail;
    if((strlen(Value)+1) > SzMax) return retvOverflow;
    strcpy(PBuf, Value);
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

#if JSON_FILE_EN
uint8_t JsonObj_t::SaveToFile(FIL *AFile) {
    return ISaveToFile(AFile, false);
}

uint8_t JsonObj_t::ISaveToFile(FIL *AFile, bool SaveNeighbor) {
    if(Name and *Name != 0) f_printf(AFile, "\"%S\":", Name);
    if(IsArray) {
        f_printf(AFile, "[");
        if(Child) Child->ISaveToFile(AFile, true);
        f_printf(AFile, "]");
    }
    else if(Child) {
        f_printf(AFile, "{");
        Child->ISaveToFile(AFile, true);
        f_printf(AFile, "}");
    }
    else if(Value and *Value != 0) f_printf(AFile, "\"%S\"", Value);
    if(Neighbor and SaveNeighbor) {
        f_printf(AFile, ",");
        Neighbor->ISaveToFile(AFile, true);
    }
    return retvOk;
}
#endif

#if 1 // ============================ Json Parser ==============================
void JsonParser_t::InitContext() {
    if(Node and Node != &Root) {
        delete Node;
        Node = nullptr;
    }
    Root.Clear();
    Node = &Root;
    Parent = nullptr;
    Container = nullptr;
    // Init parsing state
    Indx = 0;
    IsReadingName = true;
}

uint8_t JsonParser_t::StartReadFromBuf(char* ABuf, uint32_t ALen) {
    InitContext();
    IsInsideQuotes = false;
    IsInsideSingleComent = false;
    IsInsideLongComment = false;
    WasStar = false;
    WasSlash = false;
    return ContinueToReadBuf(ABuf, ALen);
}

// Returns retvOk, retvFail, and retvEmpty if buf ended before object completed.
uint8_t JsonParser_t::ContinueToReadBuf(char* ABuf, uint32_t ALen) {
    PBuf = ABuf;
    BytesLeft = ALen;
    while(true) {
        if(BytesLeft == 0) return retvEmpty;
        if(Node == nullptr) {
            Node = new JsonObj_t;
            // Init parsing state
            IsReadingName = true;
            IsInsideQuotes = false;
        }
        TokenParseRslt_t ParseRslt = IParse();

        // Add node to parent if not empty
        if(ParseRslt != tprEmptyEndOfContainer and ParseRslt != tprBufEnd) {
            if(Parent) {
                if(Parent->IsContainer and !Parent->Child) Parent->Child = Node;
                else Parent->Neighbor = Node;
            }
            Node->Container = Container;
        }

        // Handle tree
        switch(ParseRslt) {
            case tprContainer:
                Node->IsContainer = true;
                Container = Node; // go deeper
                Parent = Node;
                Node = nullptr; // Next time, create new node
                break;

            case tprValue:
                Parent = Node;
                Node = nullptr; // Next time, create new node
                break;

            case tprValueEndOfContainer: // example: min: 18}
                Parent = Container;
                Container = Container->Container;
                Node = nullptr; // Next time, create new node
                if(!Container) return retvOk;
                break;

            case tprEmptyEndOfContainer: // example: } }
                if(!Container->Child) Container->IsContainer = false; // clear flag if container is empty
                Parent = Container;
                Container = Container->Container;
                if(!Container) {
                    delete Node; // not used anywhere
                    Node = nullptr; // In destructor, do not try to delete what deleted
                    return retvOk;
                }
                break;

            case tprBufEnd: return retvEmpty; break;

            case tprFail: return retvFail; break;
        } // switch
    } // while
}

#if JSON_FILE_EN
uint8_t JsonParser_t::StartReadFromFile(const char* AFName) {
    // Open file
    if(IFile) free(IFile);
    IFile = (FIL*)malloc(sizeof(FIL));
    if(!IFile) return retvFail;
    // Writing may be required to save modified data
    uint8_t Rslt = TryOpenFileReadWrite(AFName, IFile);
    if(Rslt != retvOk) return Rslt;
    IBuf = (char*)malloc(JBUFSZ);
    if(!IBuf) return retvFail;
    BytesLeft = 0;

    IsInsideQuotes = false;
    IsInsideSingleComent = false;
    IsInsideLongComment = false;
    WasStar = false;
    WasSlash = false;
    return ReadNewFromFile();
}

uint8_t JsonParser_t::ReadNewFromFile() {
    InitContext();
    while(true) {
        if(BytesLeft == 0) {
            if(f_read(IFile, IBuf, JBUFSZ, &BytesLeft) != FR_OK) return retvFail;
            if(BytesLeft == 0) return retvEndOfFile;
            PBuf = IBuf;
        }

        uint8_t Rslt = ContinueToReadBuf(PBuf, BytesLeft);
        if(Rslt == retvOk) return retvOk;
        else if(Rslt != retvEmpty) return Rslt;
    }
}

uint8_t JsonParser_t::SaveToSameFile() {
    if(!IFile) return retvFail;
    if(f_lseek(IFile, 0) != FR_OK) return retvFail;
    if(Root.SaveToFile(IFile) != retvOk) return retvFail;
    if(f_truncate(IFile) != FR_OK) return retvFail;
    if(f_sync(IFile) != FR_OK) return retvFail;
    return retvOk;
}
#endif

JsonParser_t::~JsonParser_t() {
    if(Node and Node != &Root) {
        delete Node;
        Node = nullptr;
    }
#if JSON_FILE_EN
    if(IFile) {
        f_close(IFile);
        free(IFile);
    }
#endif
    if(IBuf) free(IBuf);
}
#endif

#if JSON_FILE_EN
/*
 * Search in first JBUFSZ chars
 * PPValue may be nullptr
 * Retval: retvFail, retvNotFound (when no such file), retvEndOfFile (when no such nodename), retvOk
 */
uint8_t JsonGetNodeValue(const char* Filename, const char* NodeName, char** PPValue) {
    FIL *IFile = new FIL;
    uint8_t Rslt = TryOpenFileRead(Filename, IFile);
    if(Rslt != retvOk) {
        delete IFile;
        return retvNotFound;
    }
    char *IBuf = (char*)malloc(JBUFSZ);
    if(!IBuf) {
        delete IFile;
        return retvFail;
    }
    uint32_t BytesLeft = 0;
    char c, *p;
    char *IStr = (char*)malloc(ISTRLEN);
    if(!IStr) {
        free(IBuf);
        delete IFile;
        return retvFail;
    }
    uint32_t Indx = 0;
    bool IsInsideQuotes = false;
    bool IsInsideSingleComent = false;
    bool IsInsideLongComment = false;
    bool WasStar = false;
    bool WasSlash = false;

    if(f_read(IFile, IBuf, JBUFSZ-1, &BytesLeft) != FR_OK) { Rslt = retvFail; goto gnvEnd; }
    if(BytesLeft == 0) { Rslt = retvEndOfFile; goto gnvEnd; }
    // Buf is read; parse it
    IBuf[BytesLeft] = 0; // Make it string
    p = strstr(IBuf, NodeName);
    if(!p) { Rslt = retvEndOfFile; goto gnvEnd; } // not found
    // Found
    if(!PPValue) { Rslt = retvOk; goto gnvEnd; } // Value is not required
    p += strlen(NodeName);
    // Get value
    // Search :, skipping whitespaces
    while(true) {
        c = *p++;
        if(c == ':') break;
        else if(c == 0 or (c > ' ' and c != '\"')) { Rslt = retvFail; goto gnvEnd; } // end of buf or something not a quote and not a whitespace
    }
    // : found
    while(true) {
        // skip whitespaces
        while(true) {
            c = *p;
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
            // Not a comment, not a whitespace, maybe end of buf
            else if(c > ' ' or c == 0) break;
            p++;
        }

        // Proceed with value
        c = *p;
        if(c == 0) { Rslt = retvFail; goto gnvEnd; } // end of buf
        else if(c == '\"') {
            if(IsInsideQuotes) { // End of value
                if(Indx == 0) { // Empty value
                    *PPValue = nullptr;
                    Rslt = retvOk;
                    goto gnvEnd;
                }
            }
            IsInsideQuotes = !IsInsideQuotes;
        }
        else if(IsInsideQuotes) { // Add everything
            if(Indx < (ISTRLEN-1)) IStr[Indx++] = c;
            else { Rslt = retvFail; goto gnvEnd; } // too long value
        }
        else if(c == ',') { // End of value
            IStr[Indx] = 0;
            *PPValue = (char*)malloc(strlen(IStr));
            if(*PPValue == nullptr) { Rslt = retvFail; goto gnvEnd; }
            strcpy(*PPValue, IStr);
            Rslt = retvOk;
            goto gnvEnd;
        }
        else if(c == ']' or c == '}') { // End of value and container
            if(Indx == 0) *PPValue = nullptr; // Empty value
            else {
                IStr[Indx] = 0;
                *PPValue = (char*)malloc(strlen(IStr));
                if(*PPValue == nullptr) { Rslt = retvFail; goto gnvEnd; }
                strcpy(*PPValue, IStr);
            }
            Rslt = retvOk;
            goto gnvEnd;
        }
        else if(c == '{' or c == '[') { Rslt = retvFail; goto gnvEnd; } // Container
        // Just char, add it to value
        else {
            if(Indx < (ISTRLEN-1)) IStr[Indx++] = c;
            else { Rslt = retvFail; goto gnvEnd; } // too long value
        }
        p++;
    }

    gnvEnd:
    free(IStr);
    free(IBuf);
    delete IFile;
    return Rslt;
}
#endif
