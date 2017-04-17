/*
 * shell.h
 *
 *  Created on: 25 окт. 2015 г.
 *      Author: Kreyl
 */

#pragma once

#include <cstring>
#include "kl_lib.h"

#define CMD_BUF_SZ		99
#define DELIMITERS      " ,"

enum ProcessDataResult_t {pdrProceed, pdrNewCmd};

class Cmd_t {
private:
    char IString[CMD_BUF_SZ];
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
        else if(Cnt < (CMD_BUF_SZ-1)) IString[Cnt++] = c;  // Add char if buffer not full
        return pdrProceed;
    }
    uint8_t GetNextString() {
        Token = strtok(NULL, DELIMITERS);
        return (*Token == '\0')? retvEmpty : retvOk;
    }

    template <typename T>
    uint8_t GetNext(T *POutput) {
        uint8_t r = GetNextString();
        if(r == retvOk) {
            char *p;
            int32_t dw32 = strtol(Token, &p, 0);
            if(*p == '\0') *POutput = (T)dw32;
            else r = retvNotANumber;
        }
        return r;
    }

    template <typename T>
    uint8_t GetArray(T *Ptr, int32_t Len) {
        for(int32_t i=0; i<Len; i++) {
            T Number;
            uint8_t r = GetNext<T>(&Number);
            if(r == retvOk) *Ptr++ = Number;
            else return r;
        }
        return retvOk;
    }

    /*
        int32_t Indx, Value;
        if(PCmd->GetParams<int32_t>(2, &Indx, &Value) == retvOk) {
            ...
        }
        else PShell->Ack(retvCmdError);
     */

    template <typename T>
    uint8_t GetParams(uint8_t Cnt, ...) {
        uint8_t Rslt = retvOk;
        va_list args;
        va_start(args, Cnt);
        while(Cnt--) {
            T* ptr = va_arg(args, T*);
            Rslt = GetNext<T>(ptr);
            if(Rslt != retvOk) break;
        }
        va_end(args);
        return Rslt;
    }

    bool NameIs(const char *SCmd) { return (strcasecmp(Name, SCmd) == 0); }
    Cmd_t() {
        Cnt = 0;
        Completed = false;
        Name = nullptr;
        Token = nullptr;
    }
};

#define EOL     "\r\n"

class Shell_t {
protected:
	thread_t *IPThd;
public:
	Cmd_t Cmd;
	void SignalCmdProcessed() {
	    chSysLock();
	    if(IPThd->p_state == CH_STATE_SUSPENDED) chSchReadyI(IPThd);
	    chSysUnlock();
	}

	virtual void Printf(const char *S, ...);
	void Reply(const char* CmdCode, int32_t Data) { Printf("%S,%d\r\n", CmdCode, Data); }
	void Ack(int32_t Result) { Printf("Ack %d\r\n", Result); }
	void Eol() { Printf(EOL); }
};
