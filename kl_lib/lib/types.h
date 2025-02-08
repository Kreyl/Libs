/*
 * retval.h
 *
 *  Created on: 29 авг. 2023 г.
 *      Author: laurelindo
 */

#ifndef LIB_TYPES_H_
#define LIB_TYPES_H_

#include <inttypes.h>

struct RGBW32 { int32_t R, G, B, W; };
struct MinMaxI32 { int32_t min, max; };

template <typename T>
class TBuf {
public:
    uint32_t len;
    T *buf;
    TBuf() : len(0), buf(nullptr) {}
    // Iterator
    uint32_t size() const noexcept { return len; }
    T* begin() const { return buf; } // First element
    T* end() const { return begin() + size(); } // One past the last element
};

using TBufBool = TBuf<bool>;
using TBufU8 = TBuf<uint8_t>;

// ==== Return values ====
struct retv {
public:
    enum Enum {
        Ok =            0,
        Fail =          1,
        Timeout =       2,
        Reset =         3,
        Busy =          4,
        InProgress =    5,
        CmdError =      6,
        CmdUnknown =    7,
        BadValue =      8,
        New =           9,
        Same =          10,
        Last =          11,
        Empty =         12,
        Overflow =      13,
        NotANumber =    14,
        WriteProtect =  15,
        WriteError =    16,
        EndOfFile =     17,
        NotFound =      18,
        BadState =      19,
        Disconnected =  20,
        Collision =     21,
        CRCError =      22,
        NACK =          23,
        NoAnswer =      24,
        OutOfMemory =   25,
        NotAuthorised = 26,
        NoChanges =     27,
        Stop =          28,
        Continue =      29,
        Ended =         30,
    } rslt;
    constexpr retv() : rslt(this->Ok) {}
    constexpr retv(Enum avalue) : rslt(avalue) {}
    explicit operator bool() const = delete; // Prevent usage: if(Retv)
    constexpr bool operator== (retv r) const { return rslt == r.rslt; }
    constexpr bool operator!= (retv r) const { return rslt != r.rslt; }
    constexpr bool IsOk()  const { return rslt == Ok; }
    constexpr bool NotOk() const { return rslt != Ok; }
};

// Semaphore states
#define SEM_TAKEN       true
#define SEM_NOT_TAKEN   false

/* ==== Retval with some payload ====
Example:
using CO2Rslt = RetvVal<SnsData>;
CO2Rslt r = Co2Sns.read_measurement();
if(r.IsOk()) Printf("%d %d %d\r", r->CO2, r->Temp, r->RH);
*/

template <typename T>
struct RetvVal : public retv {
    T v;
    RetvVal() : retv() {}
    RetvVal(retv arslt) : retv(arslt) {}
    RetvVal(retv arslt, T av) : retv(arslt), v(av) {}
    constexpr RetvVal& operator = (retv aretv) { this->rslt = aretv.rslt; return *this; }
    constexpr RetvVal& operator = (RetvVal<T> aretvval) {
        this->rslt = aretvval.rslt;
        this->v = aretvval.v;
        return *this;
    }
    T* operator ->() { return &v; }
    T& operator *() { return v; }
};

using RetvValU8  = RetvVal<uint8_t>;
using RetvValU16 = RetvVal<uint16_t>;
using RetvValI16 = RetvVal<int16_t>;
using RetvValU32 = RetvVal<uint32_t>;
using RetvValI32 = RetvVal<int32_t>;
using RetvValFloat = RetvVal<float>;
using RetvValPChar = RetvVal<char*>;
using RetvValU16x2 = RetvVal<uint16_t[2]>;
using RetvValI32x2 = RetvVal<int32_t[2]>;
using RetvValTBufBool = RetvVal<TBufBool>;

union Int32OrPChar {
    int32_t i32;
    char *pchar;
};
using RetvValInt32OrPChar = RetvVal<Int32OrPChar>;

// ==== Functional types ====
typedef void (*ftVoidVoid)(void);
typedef retv (*ftRetvVoid_t)(void);
typedef void (*ftVoidU8)(uint8_t);
typedef void (*ftVoidU32)(uint32_t);
typedef void (*ftVoidPVoid)(void*);
typedef void (*ftVoidPVoidW32)(void*, uint32_t);
typedef void (*ftVoidU8U16)(uint8_t, uint16_t);

#define NAME2VOIDFUNC(name) void name(void)

enum class Inv {NotInverted, Inverted};
enum class BitOrder {MSB, LSB};
enum class RiseFall {None, Rising, Falling, Both};

#ifndef countof
#define countof(A)  (sizeof(A)/sizeof(A[0]))
#endif

template <typename T>
static inline T Proportion(T x_min, T x_max, T y_min, T y_max, T x) {
    return (((x - x_max) * (y_max - y_min)) / (x_max - x_min)) + y_max;
}

// ==== Conversion ====
namespace Convert {

static inline uint16_t BuildU16(uint16_t bLsb, uint16_t bMsb) {
    return (bMsb << 8) | bLsb;
}

static inline uint32_t BuildU132(uint32_t bLsb, uint32_t bL1, uint32_t bM1, uint32_t bMsb) {
    return (bMsb << 24) | (bM1 << 16) | (bL1 << 8) | bLsb;
}

} // namespace

#endif /* LIB_TYPES_H_ */
