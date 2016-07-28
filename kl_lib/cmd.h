/*
 * cmd.h
 *
 *  Created on: 26 сент. 2014 г.
 *      Author: g.kruglov
 */

#ifndef CMD_H_
#define CMD_H_

#include "hal.h"
#include <cstring>

#define DELIMITERS      " ,"

enum ProcessDataResult_t {pdrProceed, pdrNewCmd};

template <uint32_t BufSz>
class Cmd_t {
private:
    char IString[BufSz];
    uint32_t Cnt;
    bool Completed;
public:
    char *Name, *Token;
    ProcessDataResult_t PutChar(char c) {
        // Reset cmd if it was completed, and after that new char arrived
        if(Completed) {
            Completed = false;
            Cnt = 0;
        }
        // Process char
        if(c == '\b') { if(Cnt > 0) Cnt--; }    // do backspace
        else if((c == '\r') or (c == '\n')) {   // end of line, check if cmd completed
            if(Cnt != 0) {  // if cmd is not empty
                IString[Cnt] = 0; // End of string
                Name = strtok(IString, DELIMITERS);
                Completed = true;
                return pdrNewCmd;
            }
        }
        else if(Cnt < BufSz-1) IString[Cnt++] = c;  // Add char if buffer not full
        return pdrProceed;
    }
    uint8_t GetNextTokenString() {
        Token = strtok(NULL, DELIMITERS);
        return (*Token == '\0')? FAILURE : OK;
    }
    uint8_t GetNextNumber(int32_t *POutput) {
        uint8_t r;
        if((r = GetNextTokenString()) == OK) {
            if(*Token == '\0') return EMPTY_STRING;
            char *p;
            *POutput = strtol(Token, &p, 0);
            return (*p == '\0')? OK : NOT_A_NUMBER;
        }
        else return r;
    }
    bool NameIs(const char *SCmd) { return (strcasecmp(Name, SCmd) == 0); }
    Cmd_t() {
        Cnt = 0;
        Completed = false;
        Name = nullptr;
        Token = nullptr;
//        for(uint32_t i=0; i<BufSz; i++) IString[i] = 0;
    }
};

#endif /* CMD_H_ */
