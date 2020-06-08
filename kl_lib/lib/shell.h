/*
 * shell.h
 *
 *  Created on: 25 ���. 2015 �.
 *      Author: Kreyl
 */

#pragma once

#include <cstring>
#include <stdarg.h>
#include "kl_lib.h"
#include "board.h"

#define DELIMITERS              " ,"
#define PREV_CHAR_TIMEOUT_ms    99UL

enum ProcessDataResult_t {pdrProceed, pdrNewCmd};

class Cmd_t {
private:
    char IString[CMD_BUF_SZ];
    uint32_t Cnt;
    bool Completed;
    systime_t LastCharTimestamp = 0;
    char* Remainer = nullptr;
    bool IsSpace(char c) { return (c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r' || c == ' '); }
    bool IsDigit(char c) { return c >= '0' and c <= '9'; }
public:
    char *Name;
    ProcessDataResult_t PutChar(char c) {
        // Reset cmd: (1) if it was completed and after that new char arrived (2) if new char has come after long pause
        if(Completed or chVTTimeElapsedSinceX(LastCharTimestamp) > TIME_MS2I(PREV_CHAR_TIMEOUT_ms)) {
            Completed = false;
            Cnt = 0;
        }
        LastCharTimestamp = chVTGetSystemTimeX();
        // Process char
        if(c == '\b') { if(Cnt > 0) Cnt--; }    // do backspace
        else if((c == '\r') or (c == '\n')) {   // end of line, check if cmd completed
            if(Cnt != 0) {  // if cmd is not empty
                IString[Cnt] = 0; // End of string
                Name = kl_strtok(IString, DELIMITERS, &Remainer);
                Completed = true;
                return pdrNewCmd;
            }
        }
        else if(Cnt < (CMD_BUF_SZ-1)) IString[Cnt++] = c;  // Add char if buffer not full
        return pdrProceed;
    }

    char* GetNextString() { return kl_strtok(nullptr, DELIMITERS, &Remainer); }

    char* GetRemainder() { return Remainer; }

    template <typename T>
    uint8_t GetNext(T *POutput) {
        char* S = GetNextString();
        if(S) {
            char *p;
            int32_t dw32 = strtol(S, &p, 0);
            if(*p == '\0') {
                *POutput = (T)dw32;
                return retvOk;
            }
            else return retvNotANumber;
        }
        return retvFail;
    }

    /*
     * Codes:
     * ints: %u32 %u16 %u8 %d32 %d16 %d8
     * string: %S %s
     * float: %f
     * '*' means skip next: %*
     * Returns number of successful conversions
     */
    int Get(const char* fmt, ...) {
        int N = 0;
        va_list args;
        va_start(args, fmt);
        while(true) {
            char c = *fmt++;
            if(c == 0) goto End;
            if(c != '%') continue;
            // % found, what next?
            c = *fmt++;
            // Check if skip next token
            if(c == '*') {
                if(GetNextString() == nullptr) goto End;
                else continue;
            }

            // Get next token
            char *tok = kl_strtok(nullptr, DELIMITERS, &Remainer);
            if(tok == nullptr) goto End;

            // Command decoding
            switch(c) {
                case 'u': {
                    // Convert
                    char *ret;
                    uint32_t v = strtoul(tok, &ret, 0); // Conversion failed
                    if(*ret != 0) goto End;
                    // Get sz
                    c = *fmt++;
                    if(c == '8') {
                        uint8_t *p = va_arg(args, uint8_t*);
                        if(p == nullptr) goto End;
                        *p = (uint8_t)v;
                    }
                    else if(c == '1') {
                        uint16_t *p = va_arg(args, uint16_t*);
                        if(p == nullptr) goto End;
                        *p = (uint16_t)v;
                    }
                    else if(c == '3') {
                        uint32_t *p = va_arg(args, uint32_t*);
                        if(p == nullptr) goto End;
                        *p = v;
                    }
                    else goto End;
                    N++;
                }
                break;

                case 'd': {
                    // Convert
                    char *ret;
                    int32_t v = strtol(tok, &ret, 0); // Conversion failed
                    if(*ret != 0) goto End;
                    // Get sz
                    c = *fmt++;
                    if(c == '8') {
                        int8_t *p = va_arg(args, int8_t*);
                        if(p == nullptr) goto End;
                        *p = (int8_t)v;
                    }
                    else if(c == '1') {
                        int16_t *p = va_arg(args, int16_t*);
                        if(p == nullptr) goto End;
                        *p = (int16_t)v;
                    }
                    else if(c == '3') {
                        int32_t *p = va_arg(args, int32_t*);
                        if(p == nullptr) goto End;
                        *p = v;
                    }
                    else goto End;
                    N++;
                }
                break;

                case 's':
                case 'S': {
                    char **p = va_arg(args, char**);
                    if(p == nullptr) goto End;
                    *p = tok;
                    N++;
                }
                break;
#if PRINTF_FLOAT_EN
                case 'f': {
                    float *p = va_arg(args, float*);
                    if(p == nullptr) goto End;
                    char *ret;
                    *p = strtof(tok, &ret);
                    if(*ret != 0) goto End; // Conversion failed
                    N++;
                }
                break;
#endif
                default: break; // including '*'
            } // switch c
        } // while true
        End:
        return N;
    }

#if PRINTF_FLOAT_EN
    uint8_t GetNextFloat(float *POutput) {
        char* S = GetNextString();
        if(!S) return retvFail;
        char *p;
        float f = strtof(S, &p);
        if(*p == '\0') {
            *POutput = f;
            return retvOk;
        }
        else return retvNotANumber;
    }

    uint8_t GetNextDouble(double *POutput) {
        char* S = GetNextString();
        if(!S) return retvFail;
        char *p;
        double f = strtod(S, &p);
        if(*p == '\0') {
            *POutput = f;
            return retvOk;
        }
        else return retvNotANumber;
    }
#endif

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
    }
};

class Shell_t {
public:
	Cmd_t Cmd;
	virtual void Print(const char *format, ...) = 0;
//	void Reply(const char* CmdCode, int32_t Data) { Print("%S,%d\r\n", CmdCode, Data); }
//	void Ack(int32_t Result) { Print("Ack %d\r\n", Result); }
    void Ok()  { Print("Ok\r\n"); }
    void BadParam() { Print("BadParam\r\n"); }
    void CRCError() { Print("CRCError\r\n"); }
    void CmdError() { Print("CmdError\r\n"); }
    void CmdUnknown() { Print("CmdUnknown\r\n"); }
    void Failure() { Print("Failure\r\n"); }
    void Timeout() { Print("Timeout\r\n"); }
    void NoAnswer() { Print("NoAnswer\r\n"); }
    void EOL() { Print("\r\n"); }
	virtual uint8_t ReceiveBinaryToBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) = 0;
	virtual uint8_t TransmitBinaryFromBuf(uint8_t *ptr, uint32_t Len, uint32_t Timeout_ms) = 0;
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
