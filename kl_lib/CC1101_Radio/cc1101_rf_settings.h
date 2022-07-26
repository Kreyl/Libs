/*
 * File:   cc_rf_settings.h
 * Author: Kreyl Laurelindo
 *
 * Created on 7 Март 2010 г., 12:42
 */

#pragma once

// All this is for 27.0 MHz crystal, and for 868 MHz carrier

// ============================ Common use values ==============================
#define CC_TX_FIFO_SIZE     33
#define CC_RX_FIFO_SIZE     32

// ===================== Frequency 868 MHz: RF Studio ==========================
#define CC_FREQ2_VALUE      0x20        // Frequency control word, high byte.
#define CC_FREQ1_VALUE      0x25        // Frequency control word, middle byte.
#define CC_FREQ0_VALUE      0xED        // Frequency control word, low byte.

// ===================== Channel spacing =======================================
#define CC_CHANNEL_SPACING  421     // 200, 400, 421(top)

#if CC_CHANNEL_SPACING == 200
#define CC_MDMCFG0_VALUE    229     // Channel spacing mantissa. See exponent at MDMCFG1. RF studio.
#define CC_CHANSPC_E        2       // Exponent of Channel Spacing, RF Studio
#elif CC_CHANNEL_SPACING == 421
#define CC_MDMCFG0_VALUE    255     // Channel spacing mantissa. See exponent at MDMCFG1. RF studio.
#define CC_CHANSPC_E        3       // Exponent of Channel Spacing, RF Studio
#else
#error "CC: Wrong Channel Spacing"
#endif

// =================================== Common ==================================
// ==== MDMCFG1 ==== 7 FEC_EN, 6:4 NUM_PREAMBLE, 3:2 not used, 1:0 CHANSPC_E
#define CC_FEC_EN           0x80    // Fec enabled
//#define CC_FEC_EN           0x00    // Fec disabled
#define CC_NUM_PREAMBLE     0x20    // 010 => 4 bytes of preamble
#define CC_MDMCFG1_VALUE    (CC_FEC_EN | CC_NUM_PREAMBLE | CC_CHANSPC_E)

//#define CC_MCSM0_VALUE      0x18        // Calibrate at IDLE->RX,TX
#define CC_MCSM0_VALUE      0x08        // Never calibrate

// ==== MCSM1 ==== bits 7:6 not used, 5:4 ClearChannel mode, 3:2 RxOff mode, 1:0 TxOff mode
//#define CC_CCA_MODE         0x00  // Always clear
//#define CC_CCA_MODE         0x10  // If RSSI below threshold
//#define CC_CCA_MODE         0x20  // Unless currently receiving a packet
#define CC_CCA_MODE         0x30  // If RSSI below threshold unless currently receiving a packet
// RX->IDLE, TX->IDLE
#define CC_MCSM1_VALUE      (CC_CCA_MODE)

#define CC_MCSM2_VALUE      0b00000111  // WOR settings, nothing interesting here

#define CC_FIFOTHR_VALUE    0b00000111  // RX attenuation = 0; RXFIFO and TXFIFO thresholds: TX 33, RX 32
//#define CC_IOCFG2_VALUE     0x0E        // GDO2: Carrier Sense
//#define CC_IOCFG2_VALUE     0x09        // GDO2: Clear Channal Assesment
#define CC_IOCFG2_VALUE     0x3F        // GDO2: CLK_XOSC/192 = 27MHz / 192 = 140 625 Hz
#define CC_IOCFG0_VALUE     0x06        // GDO0 - Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
                                        // In RX, the pin will also deassert when a packet is discarded due to address or maximum length filtering

//#define CC_IOCFG0_VALUE     0x07 // Asserts when a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO

//#define CC_PKTCTRL1_VALUE   0b00001110  // PQT=0, CRC autoflush=1, Append=1, Address check = 10 (check, 0 is broadcast)
#define CC_PKTCTRL1_VALUE   0b00001100  // PQT=0, CRC autoflush=1, Append=1, Address check = 00 (no check)
#define CC_PKTCTRL0_VALUE   0b01000100  // WhiteData=1, PKTFrmt=norm, CRC en, Fixed Length
//#define CC_PKTCTRL0_VALUE   0b00000100  // WhiteData=0, PKTFrmt=norm, CRC en, Fixed Length
//#define CC_PKTCTRL0_VALUE   0b01000101  // WhiteData=1, PKTFrmt=norm, CRC en, Variable Length
//#define CC_PKTCTRL0_VALUE   0b00000000  // WhiteData=0, PKTFrmt=norm, CRC dis, Fixed Length
#define CC_ADDR_VALUE       0x01        // Device address.

struct CCRegValue_t {
    uint8_t Reg, Value;
} __attribute__((packed));

// ========================= Bitrate-specific ==================================
#define CC_BRSETUP_CNT  20

__unused
static const CCRegValue_t CCBitrate2k4[CC_BRSETUP_CNT] = {
        {CC_FSCTRL1,  0x06}, // }
        {CC_FSCTRL0,  0x00}, // } Frequency synthesizer control: RF studio, nothing to do here

        {CC_MDMCFG4,  0xF6}, // }
        {CC_MDMCFG3,  0x75}, // } Modem configuration: RF Studio, nothing to do here
        {CC_MDMCFG2,  0x13}, // Filter, modulation format, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits

        {CC_DEVIATN,  0x14}, // Modem deviation setting - RF studio, nothing to do here
        {CC_FREND1,   0x56}, // Front end RX configuration - RF studio, no docs, nothing to do
        {CC_FREND0,   0x10}, // Front end TX configuration - RF studio, no docs, nothing to do

        {CC_FOCCFG,   0x16}, // Frequency Offset Compensation - RF studio, some unknown reasons for settings
        {CC_BSCFG,    0x6C}, // Bit synchronization Configuration - RF studio, some unknown reasons for settings
        {CC_AGCCTRL2, 0x03}, // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
        {CC_AGCCTRL1, 0x40}, // Generally, nothing interesting
        {CC_AGCCTRL0, 0x91}, // AGC filter settings: RF studio, some unknown reasons for settings
        // Frequency synthesizer calibration: RF studio, nothing to do here
        {CC_FSCAL3,   0xE9},
        {CC_FSCAL2,   0x2A},
        {CC_FSCAL1,   0x00},
        {CC_FSCAL0,   0x1F},
        // Various test settings: RF studio
        {CC_TEST2,    0x81},
        {CC_TEST1,    0x35},
        {CC_TEST0,    0x09},
};

__unused
static const CCRegValue_t CCBitrate10k[CC_BRSETUP_CNT] = {
        {CC_FSCTRL1,  0x06}, // }
        {CC_FSCTRL0,  0x00}, // } Frequency synthesizer control: RF studio, nothing to do here

        {CC_MDMCFG4,  0xC8}, // }
        {CC_MDMCFG3,  0x84}, // } Modem configuration: RF Studio, nothing to do here
        {CC_MDMCFG2,  0x13}, // Filter, modulation format, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits

        {CC_DEVIATN,  0x33}, // Modem deviation setting - RF studio, nothing to do here
        {CC_FREND1,   0x56}, // Front end RX configuration - RF studio, no docs, nothing to do
        {CC_FREND0,   0x10}, // Front end TX configuration - RF studio, no docs, nothing to do

        {CC_FOCCFG,   0x16}, // Frequency Offset Compensation - RF studio, some unknown reasons for settings
        {CC_BSCFG,    0x6C}, // Bit synchronization Configuration - RF studio, some unknown reasons for settings
        {CC_AGCCTRL2, 0x43}, // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
        {CC_AGCCTRL1, 0x40}, // Generally, nothing interesting
        {CC_AGCCTRL0, 0x91}, // AGC filter settings: RF studio, some unknown reasons for settings
        // Frequency synthesizer calibration: RF studio, nothing to do here
        {CC_FSCAL3,   0xE9},
        {CC_FSCAL2,   0x2A},
        {CC_FSCAL1,   0x00},
        {CC_FSCAL0,   0x1F},
        // Various test settings: RF studio
        {CC_TEST2,    0x81},
        {CC_TEST1,    0x35},
        {CC_TEST0,    0x09},
};

__unused
static const CCRegValue_t CCBitrate38k4[CC_BRSETUP_CNT] = {
        {CC_FSCTRL1,  0x06}, // }
        {CC_FSCTRL0,  0x00}, // } Frequency synthesizer control: RF studio, nothing to do here

        {CC_MDMCFG4,  0xCA}, // }
        {CC_MDMCFG3,  0x75}, // } Modem configuration: RF Studio, nothing to do here
        {CC_MDMCFG2,  0x13}, // Filter, modulation format, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits

        {CC_DEVIATN,  0x34}, // Modem deviation setting - RF studio, nothing to do here
        {CC_FREND1,   0x56}, // Front end RX configuration - RF studio, no docs, nothing to do
        {CC_FREND0,   0x10}, // Front end TX configuration - RF studio, no docs, nothing to do

        {CC_FOCCFG,   0x16}, // Frequency Offset Compensation - RF studio, some unknown reasons for settings
        {CC_BSCFG,    0x6C}, // Bit synchronization Configuration - RF studio, some unknown reasons for settings
        {CC_AGCCTRL2, 0x43}, // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
        {CC_AGCCTRL1, 0x40}, // Generally, nothing interesting
        {CC_AGCCTRL0, 0x91}, // AGC filter settings: RF studio, some unknown reasons for settings
        // Frequency synthesizer calibration: RF studio, nothing to do here
        {CC_FSCAL3,   0xE9},
        {CC_FSCAL2,   0x2A},
        {CC_FSCAL1,   0x00},
        {CC_FSCAL0,   0x1F},
        // Various test settings: RF studio
        {CC_TEST2,    0x81},
        {CC_TEST1,    0x35},
        {CC_TEST0,    0x09},
};

__unused
static const CCRegValue_t CCBitrate100k[CC_BRSETUP_CNT] = {
        {CC_FSCTRL1,  0x08}, // }
        {CC_FSCTRL0,  0x00}, // } Frequency synthesizer control: RF studio, nothing to do here

        {CC_MDMCFG4,  0x5B}, // }
        {CC_MDMCFG3,  0xE5}, // } Modem configuration: RF Studio, nothing to do here
        {CC_MDMCFG2,  0x13}, // Filter, modulation format, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits

        {CC_DEVIATN,  0x46}, // Modem deviation setting - RF studio, nothing to do here
        {CC_FREND1,   0xB6}, // Front end RX configuration - RF studio, no docs, nothing to do
        {CC_FREND0,   0x10}, // Front end TX configuration - RF studio, no docs, nothing to do

        {CC_FOCCFG,   0x1D}, // Frequency Offset Compensation - RF studio, some unknown reasons for settings
        {CC_BSCFG,    0x1C}, // Bit synchronization Configuration - RF studio, some unknown reasons for settings
        {CC_AGCCTRL2, 0xC7}, // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
        {CC_AGCCTRL1, 0x00}, // Generally, nothing interesting
        {CC_AGCCTRL0, 0xB2}, // AGC filter settings: RF studio, some unknown reasons for settings
        // Frequency synthesizer calibration: RF studio, nothing to do here
        {CC_FSCAL3,   0xEA},
        {CC_FSCAL2,   0x2A},
        {CC_FSCAL1,   0x00},
        {CC_FSCAL0,   0x1F},
        // Various test settings: RF studio
        {CC_TEST2,    0x81},
        {CC_TEST1,    0x31},
        {CC_TEST0,    0x09},
};

__unused
static const CCRegValue_t CCBitrate250k[CC_BRSETUP_CNT] = {
        {CC_FSCTRL1,  0x0C}, // }
        {CC_FSCTRL0,  0x00}, // } Frequency synthesizer control: RF studio, nothing to do here

        {CC_MDMCFG4,  0x2D}, // }
        {CC_MDMCFG3,  0x2F}, // } Modem configuration: RF Studio, nothing to do here
        {CC_MDMCFG2,  0x13}, // Filter, modulation format, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits

        {CC_DEVIATN,  0x62}, // Modem deviation setting - RF studio, nothing to do here
        {CC_FREND1,   0xB6}, // Front end RX configuration - RF studio, no docs, nothing to do
        {CC_FREND0,   0x10}, // Front end TX configuration - RF studio, no docs, nothing to do

        {CC_FOCCFG,   0x1D}, // Frequency Offset Compensation - RF studio, some unknown reasons for settings
        {CC_BSCFG,    0x1C}, // Bit synchronization Configuration - RF studio, some unknown reasons for settings
        {CC_AGCCTRL2, 0xC7}, // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
        {CC_AGCCTRL1, 0x00}, // Generally, nothing interesting
        {CC_AGCCTRL0, 0xB0}, // AGC filter settings: RF studio, some unknown reasons for settings
        // Frequency synthesizer calibration: RF studio, nothing to do here
        {CC_FSCAL3,   0xEA},
        {CC_FSCAL2,   0x2A},
        {CC_FSCAL1,   0x00},
        {CC_FSCAL0,   0x1F},
        // Various test settings: RF studio
        {CC_TEST2,    0x88},
        {CC_TEST1,    0x31},
        {CC_TEST0,    0x09},
};

__unused
static const CCRegValue_t CCBitrate500k[CC_BRSETUP_CNT] = {
        {CC_FSCTRL1,  0x0E}, // }
        {CC_FSCTRL0,  0x00}, // } Frequency synthesizer control: RF studio, nothing to do here

        {CC_MDMCFG4,  0x0E}, // }
        {CC_MDMCFG3,  0x2F}, // } Modem configuration: RF Studio, nothing to do here
        {CC_MDMCFG2,  0x73}, // Filter on, modulation format MSK, no Manchester, SYNC_MODE=011 => 30/32 sync word bits

        {CC_DEVIATN,  0x00}, // Modem deviation setting - RF studio, nothing to do here
        {CC_FREND1,   0xB6}, // Front end RX configuration - RF studio, no docs, nothing to do
        {CC_FREND0,   0x10}, // Front end TX configuration - RF studio, no docs, nothing to do

        {CC_FOCCFG,   0x1D}, // Frequency Offset Compensation - RF studio, some unknown reasons for settings
        {CC_BSCFG,    0x1C}, // Bit synchronization Configuration - RF studio, some unknown reasons for settings
        {CC_AGCCTRL2, 0xC7}, // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
        {CC_AGCCTRL1, 0x09}, // Carrier Sense is abs, threshold is MAGN_TARGET - 7
        {CC_AGCCTRL0, 0xB0}, // AGC filter settings: RF studio
        // Frequency synthesizer calibration: RF studio, nothing to do here
        {CC_FSCAL3,   0xEA},
        {CC_FSCAL2,   0x2A},
        {CC_FSCAL1,   0x00},
        {CC_FSCAL0,   0x1F},
        // Various test settings: RF studio
        {CC_TEST2,    0x88},
        {CC_TEST1,    0x31},
        {CC_TEST0,    0x09},
};

// Rare use settings
#define CC_SYNC1_VALUE      0xD3
#define CC_SYNC0_VALUE      0x91
