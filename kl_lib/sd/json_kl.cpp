/*
 * json_kl.cpp
 *
 *  Created on: 16 сент. 2018 г.
 *      Author: Kreyl
 */

#include "json_kl.h"
#include "kl_fs_utils.h"

extern char IStr[COMMON_STR_LEN];

namespace json { // ================== json file parsing =======================
static const JsonObj_t EmptyNode;
static char *IBuf, *PChar;
static uint32_t BytesLeft;
#define JBUFSZ      4096

void CloseFile() {
    f_close(&CommonFile);
    if(IBuf) {
        free(IBuf);
        IBuf = nullptr;
    }
}

uint8_t OpenFile(const char *AFileName) {
    if(TryOpenFileRead(AFileName, &CommonFile) == retvOk) {
        IBuf = (char*)malloc(JBUFSZ);
        PChar = IBuf;
        if(IBuf) {
            if(f_read(&CommonFile, IBuf, JBUFSZ, &BytesLeft) == FR_OK) return retvOk;
        }
        CloseFile();
    }
    return retvFail;
}


static JsonObj_t* CreateNewObj() {
    JsonObj_t* ptr = (JsonObj_t*)malloc(sizeof(JsonObj_t));
    if(ptr != nullptr) ptr->Reset();
    return ptr;
}
static void FreeObj(JsonObj_t* Obj) {
    if(Obj->Name  != nullptr) free(Obj->Name);
    if(Obj->Value != nullptr) free(Obj->Value);
    free(Obj);
}

__always_inline
static inline uint8_t GetNextChar(char *c) {
    if(BytesLeft == 0) { // Buf is empty
        if(f_read(&CommonFile, IBuf, JBUFSZ, &BytesLeft) != FR_OK) return retvFail;
        if(BytesLeft == 0) return retvFail; // File end
        PChar = IBuf;
    }
    *c = *PChar++;
    BytesLeft--;
    return retvOk;
}

static uint8_t FinalizeStr(char *PStr, char **PValue) {
    // Remove whitespaces at end
    while(PStr > IStr and *(PStr-1) <= ' ') PStr--;
    uint32_t StrLen = PStr - IStr;
    if(StrLen > 0) {
        *PStr = 0; // End string
        // Copy value
        char *p = (char*)malloc(StrLen+1); // Reserve extra char for '/0'
        if(p == nullptr) return retvFail;
        *PValue = p;
        strcpy(p, IStr);
    }
    return retvOk;
}

enum ExitCondition_t {excFailure, excContainerStart, excContainerEnd, excNoObjContainerEnd, excObjEnd};

static ExitCondition_t ReadNextObj(JsonObj_t* Obj) {
    bool IsInsideSingleComent = false, IsInsideLongComment = false, WasStar = false, WasSlash = false;
    char c;
    bool IsReadingName = true;
    int32_t ArrayDepth = 0;
    char* PStr = IStr;

    while(true) {
        if(GetNextChar(&c) != retvOk) {
            // Cannot read char, finalize object
            if(IsReadingName) {
                if(FinalizeStr(PStr, &Obj->Name) == retvOk) return excObjEnd;
                else return excFailure;
            }
            else {
                if(FinalizeStr(PStr, &Obj->Value) == retvOk) return excObjEnd;
                else return excFailure;
            }
        }

#if 1 // ==== Process whitespaces and comments ====
        if(IsInsideSingleComent) { // Ends with line
            // Check if EOL
            if(c == '\r' or c == '\n') IsInsideSingleComent = false;
            continue;
        }
        else if(IsInsideLongComment) { // ends with star and slash
            if(WasStar and c == '/') IsInsideLongComment = false;
            else WasStar = (c == '*');
            continue;
        }

        // Find comments start
        if(c == '/') {
            if(WasSlash) {
                IsInsideSingleComent = true;
                WasSlash = false;
            }
            else WasSlash = true;
            continue;
        }
        else if(c == '*' and WasSlash) {
            IsInsideLongComment = true;
            continue;
        }
#endif

        // Skip spaces, tabs etc.
        if(c <= ' ') continue;

        if(IsReadingName) {
            // EOL, reset string
            if(c == '\r' or c == '\n' or c == ',') PStr = IStr;
            // Check if end of name: ':'
            else if(c == ':') {
                if(FinalizeStr(PStr, &Obj->Name) != retvOk) return excFailure;
                PStr = IStr;
                IsReadingName = false;
            }
            // Container with missed colon or without name
            else if(c == '{') {
                if(FinalizeStr(PStr, &Obj->Name) == retvOk) return excContainerStart;
                else return excFailure;
            }
            // End of container ",}" condition, no object here
            else if(c == '}') return excNoObjContainerEnd;
            // Just char, add it to name
            else {
                if(PStr < (IStr + COMMON_STR_LEN-1)) *PStr++ = c;
                else return excFailure; // too long name
            } // just char
        } // IsReadingName
        else { // Reading value
            if(c == '{') { // Container start
                Obj->Value = nullptr;
                return excContainerStart;
            }
            // End of container
            else if(c == '}') {
                if(FinalizeStr(PStr, &Obj->Value) == retvOk) return excContainerEnd;
                else return excFailure;
            }
            // Simply skip line feed
            else if(c == '\r' or c == '\n') continue;
            // Array
            else if(c == '[') {
                ArrayDepth++;
                continue;
            }
            else if(c == ']') {
                ArrayDepth--;
                // Check if end of array
                if(ArrayDepth <= 0) {
                    if(FinalizeStr(PStr, &Obj->Value) == retvOk) return excContainerEnd;
                    else return excFailure;
                }
                continue;
            }
            // End of object
            else if(c == ',' and ArrayDepth <= 0) {
                if(FinalizeStr(PStr, &Obj->Value) == retvOk) return excObjEnd;
                else return excFailure;
            }
            // Just char, add it to name
            else {
                if(PStr < (IStr + COMMON_STR_LEN-1)) *PStr++ = c;
                else return excFailure; // too long name
            } // just char
        }
    } // while(true)
}

JsonObj_t& Read() {
    JsonObj_t* Root = CreateNewObj();
    if(Root == nullptr) return *((JsonObj_t*)&EmptyNode); // No more memory
    ExitCondition_t Rslt = ReadNextObj(Root);
    if(Rslt == excFailure) {
        FreeObj(Root);
        return *((JsonObj_t*)&EmptyNode);
    }

    // Proceed with reading
    if(Rslt == excContainerStart) { // Container, read child structs
        JsonObj_t* Parent = Root;
        JsonObj_t* Node, *Container = Root;
        bool CreateNewNode = true;
        int32_t DepthLvl = 1;

        while(DepthLvl > 0) {
            if(CreateNewNode) {
                Node = CreateNewObj();
                if(Node == nullptr) { // No more memory
                    Free(*Root);
                    return *((JsonObj_t*)&EmptyNode);
                }
            }
            else CreateNewNode = true; // Create next time (if will not cancel again)

            // Read Node
            Rslt = ReadNextObj(Node);
//            Node->Print();
            switch(Rslt) {
                case excFailure:
                    Free(*Root);
                    return *((JsonObj_t*)&EmptyNode);
                    break;
                case excContainerStart:
                    if(Parent->Value == nullptr and !Parent->Child) Parent->Child = Node;
                    else Parent->Neighbor = Node;
                    Node->Parent = Parent;
                    Parent = Node;
                    Container = Node;
                    DepthLvl++;
                    break;
                case excContainerEnd:
                    if(Parent->Value == nullptr and !Parent->Child) Parent->Child = Node;
                    else Parent->Neighbor = Node;
                    Node->Parent = Parent;
                    Parent = Container;
                    Container = Container->Parent;
                    DepthLvl--;
                    break;
                case excObjEnd:
                    if(Parent->Value == nullptr and !Parent->Child) Parent->Child = Node;
                    else Parent->Neighbor = Node;
                    Node->Parent = Parent;
                    Parent = Node;
                    break;
                case excNoObjContainerEnd:
                    CreateNewNode = false;
                    Parent = Container;
                    Container = Container->Parent;
                    DepthLvl--;
                    if(DepthLvl <= 0) FreeObj(Node); // Was created, not required
                    break;
            } // switch
        } // while true
    }
    else { // not container start, check if not empty
        if(Root->Name == nullptr and Root->Value == nullptr) {
            // Empty root
            FreeObj(Root);
            return *((JsonObj_t*)&EmptyNode);
        }
    }
    return *Root;
}

void Free(JsonObj_t& Root) {
    if(&Root == &EmptyNode) return;
    JsonObj_t *CurrNode = &Root;
    uint32_t Cnt = 0;
    while(CurrNode != nullptr) {
        if(CurrNode->Child != nullptr) CurrNode = CurrNode->Child;
        else if(CurrNode->Neighbor != nullptr) CurrNode = CurrNode->Neighbor;
        else { // both child and neighbor are absent
//            CurrNode->Print();
            JsonObj_t *Parent = CurrNode->Parent;
            if(Parent != nullptr) {
                // Remove links to curr node from parent
                if(CurrNode == Parent->Child) Parent->Child = nullptr;
                else Parent->Neighbor = nullptr;
            }
            FreeObj(CurrNode);
            Cnt++;
            CurrNode = Parent;
        }
    }
}

//void JsonObj_t::PrintAll() const {
//    Print();
//    chThdSleepMilliseconds(7);
//    if(Child) Child->PrintAll();
//    if(Neighbor) Neighbor->PrintAll();
//}

JsonObj_t& JsonObj_t::operator[](const char* AName) const {
    JsonObj_t *Node = Child;
    while(Node != nullptr) {
        if(strcasecmp(AName, Node->Name) == 0) return *Node;
        Node = Node->Neighbor;
    }
    return *((JsonObj_t*)&EmptyNode);
}

#if 1 // ==== Values ====
//uint8_t JsonObj_t::ToString(char *S) const {

uint8_t JsonObj_t::ToInt(int32_t *POut) const {
    if(Value == nullptr) return retvEmpty;
    char *p;
    *POut = strtol(Value, &p, 0);
    if(*p == '\0') return retvOk;
    else return retvNotANumber;
}

uint8_t JsonObj_t::ToBool(bool *POut) const {
    int32_t w;
    uint8_t r = ToInt(&w);
    if(r == retvOk) {
        *POut = (bool)w;
        return retvOk;
    }
    else return r;
}

uint8_t JsonObj_t::ToFloat(float *POut) const {
    if(Value == nullptr) return retvEmpty;
    char *p;
    *POut = strtof(Value, &p);
    if(*p == '\0') return retvOk;
    else return retvNotANumber;
}

uint8_t JsonObj_t::ToColor(Color_t *PClr) const {
    char *SavePtr = Value;
    if(GetNextArrayByte(&PClr->R, &SavePtr) != retvOk) return retvFail;
    if(GetNextArrayByte(&PClr->G, &SavePtr) != retvOk) return retvFail;
    if(GetNextArrayByte(&PClr->B, &SavePtr) != retvOk) return retvFail;
    return retvOk;
}

#endif

char* JsonObj_t::GetNextArrayStr(char **SavePtr) const {
    return strtok_r(nullptr, "[, ]", SavePtr);
}

uint8_t JsonObj_t::GetNextArrayByte(uint8_t *POut, char **SavePtr) const {
    char *S, *p;
    S =  GetNextArrayStr(SavePtr);
    if(S == nullptr or *S == 0) return retvFail; // Empty string
    *POut = (uint8_t)strtoul(S, &p, 0);
    if(*p == '\0') return retvOk;
    else return retvNotANumber;
}

} // namespace

