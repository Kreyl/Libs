#include "types.h"

#define RTT_BUFTX_SZ    256
#define RTT_BUFTX_CNT   2
#define RTT_BUFRX_SZ    64
#define RTT_BUFRX_CNT   2

// Operating modes. Define behavior if buffer is full (not enough space for entire message)
#define RTT_MODE_NO_BLOCK_SKIP         (0)     // Skip. Do not block, output nothing. (Default)
#define RTT_MODE_NO_BLOCK_TRIM         (1)     // Trim: Do not block, output as much as fits.
#define RTT_MODE_BLOCK_IF_FIFO_FULL    (2)     // Block: Wait until there is space in the buffer.
#define RTT_MODE_MASK                  (3)

#define RTT_MODE_DEFAULT                RTT_MODE_NO_BLOCK_SKIP // Mode for pre-initialized terminal channel (buffer 0)



class RTT_t {
private:
    void IInit();
public:
    bool echo_on = false; // echo or do not echo characters received from host
    RTT_t();

    uint32_t GetTxBytesCnt(uint32_t BufferIndex);
    uint32_t GetRxBytesCnt(uint32_t BufferIndex);

    retv IPutChar(char c);
};
