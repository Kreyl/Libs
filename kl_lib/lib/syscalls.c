#include "ch.h"

extern void PrintfC(const char *format, ...);

void* malloc(size_t Sz) {
    PrintfC("Malloc %u\r", Sz);
    return chHeapAlloc(NULL, Sz);
}

void Free(void* p) {
    PrintfC("Free %X\r", p);
    chHeapFree(p);
}
