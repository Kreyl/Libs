/*
 * shell.h
 *
 *  Created on: 25 окт. 2015 г.
 *      Author: Kreyl
 */

#pragma once

#include <cstring>
#include <stdarg.h>
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
    uint8_t GetNextString(char **PStr = nullptr) {
        Token = strtok(NULL, DELIMITERS);
        if(PStr != nullptr) *PStr = Token;
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

    /*  int32_t Indx, Value;
        if(PCmd->GetParams<int32_t>(2, &Indx, &Value) == retvOk) {...}
        else PShell->Ack(retvCmdError);    */
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

    bool NameIs(const char *SCmd) { return (kl_strcasecmp(Name, SCmd) == 0); }
    Cmd_t() {
        Cnt = 0;
        Completed = false;
        Name = nullptr;
        Token = nullptr;
    }
};

class Shell_t {
public:
	Cmd_t Cmd;
	virtual void SignalCmdProcessed() = 0;
	virtual void Print(const char *format, ...) = 0;
	void Reply(const char* CmdCode, int32_t Data) { Print("%S,%d\r\n", CmdCode, Data); }
	void Ack(int32_t Result) { Print("Ack %d\r\n", Result); }
};


// Parent class for everything that prints
class PrintfHelper_t {
private:
    uint8_t IPutUint(uint32_t n, uint32_t base, uint32_t width, char filler);
protected:
    virtual uint8_t IPutChar(char c) = 0;
    virtual void IStartTransmissionIfNotYet() = 0;
public:
    void IVsPrintf(const char *format, va_list args);
    void PrintEOL();
};

#if 1 // ========================= Byte protocol ===============================
#define BYTECMD_DATA_SZ     99
class ByteCmd_t {
private:
//    char IString[CMD_BUF_SZ];
    bool Completed;
    uint8_t IBuf[BYTECMD_DATA_SZ];
    bool FirstHalfOfByte = true, WasStarted = false;
    void AddHalfOfByte(uint8_t Half) {
        if(FirstHalfOfByte) {
            IBuf[Cnt] = Half << 4;
            FirstHalfOfByte = false;
        }
        else {
            IBuf[Cnt++] |= Half;
            FirstHalfOfByte = true;
        }
    }
public:
    uint8_t CmdCode, *Data = &IBuf[1];
    uint32_t Cnt;
    ProcessDataResult_t PutChar(char c) {
        // Reset cmd if it was completed, and after that new char arrived
        if(Completed) {
            Completed = false;
            Cnt = 0;
            FirstHalfOfByte = true;
            WasStarted = false;
        }
        // Process char
        if(c == '#') {  // Start of new cmd
            Cnt = 0;
            FirstHalfOfByte = true;
            WasStarted = true;
        }
        else if(WasStarted) {   // Do all the next if was started
            if(c == '\b') { if(Cnt > 0) Cnt--; }    // Do backspace
            else if(c == '#') { Cnt = 0; FirstHalfOfByte = true; }
            else if((c == '\r') or (c == '\n')) {   // End of line, check if cmd completed
                if(Cnt != 0) {  // if not empty
                    CmdCode = IBuf[0];
                    Cnt--;  // Remove CmdCode out of cnt
                    Completed = true;
                    return pdrNewCmd;
                }
            }
            // Some other char, maybe good one
            else if(Cnt < BYTECMD_DATA_SZ) {
                if     (c >= '0' and c <= '9') AddHalfOfByte(c - '0');
                else if(c >= 'A' and c <= 'F') AddHalfOfByte(c - 'A' + 0xA);
                else if(c >= 'a' and c <= 'f') AddHalfOfByte(c - 'a' + 0xA);
            }
        }
        return pdrProceed;
    }

};

class ByteShell_t {
private:
    uint8_t HalfByte2Char(uint8_t hb) {
        hb &= 0x0F;
        if(hb < 0x0A) return ('0' + hb);    // 0...9
        else return ('A' + hb - 0x0A);      // A...F
    }
public:
    ByteCmd_t Cmd;
    bool CmdProcessInProgress;
    void SignalCmdProcessed() { CmdProcessInProgress = false; }
    virtual uint8_t IPutChar(char c) = 0;
    virtual void IStartTransmissionIfNotYet() = 0;
    void Reply(uint8_t CmdCode, uint32_t Len, uint8_t *PData);
//    void SendCmd(uint8_t CmdCode, uint32_t Len, ...);
    //void Reply(const uint8_t CmdCode, int32_t Data) { Printf("%S,%d\r\n", CmdCode, Data); }
    void Ack(uint8_t Result) { Reply(0x90, 1, &Result); }
};

#endif

// Functions
class CmdUart_t;

void Printf(const char *format, ...);
void Printf(CmdUart_t &AUart, const char *format, ...);
void PrintfI(const char *format, ...);
void PrintfEOL();
//void PrintfNow(const char *format, ...);

char* PrintfToBuf(char* PBuf, const char *format, ...);

extern "C" {
void PrintfC(const char *format, ...);
//void PrintfCNow(const char *format, ...);
}
