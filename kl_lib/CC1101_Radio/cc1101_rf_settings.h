/*
 * File:   cc_rf_settings.h
 * Author: Kreyl Laurelindo
 *
 * Created on 7 Март 2010 г., 12:42
 */

#pragma once

// All this is for 27.0 MHz crystal, and for 868 MHz carrier

// Bitrate
//#define CC_BITRATE_10K
//#define CC_BITRATE_38K4
//#define  CC_BITRATE_100K
//#define CC_BITRATE_250K
#define CC_BITRATE_500K

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
#define CC_CCA_MODE         0b00000000  // Always clear
//#define CC_CCA_MODE         0b00100000  // Unless currently receiving a packet
#define CC_RXOFF_MODE       0b00000000  // RX->IDLE
//#define CC_RXOFF_MODE       0b00001100  // RX->RX
#define CC_TXOFF_MODE       0b00000000  // TX->IDLE
#define CC_MCSM1_VALUE      (CC_CCA_MODE | CC_RXOFF_MODE | CC_TXOFF_MODE)

#define CC_MCSM2_VALUE      0b00000111  // WOR settings, nothing interesting here

#define CC_FIFOTHR_VALUE    0b00000111  // RX attenuation = 0; RXFIFO and TXFIFO thresholds: TX 33, RX 32
#define CC_IOCFG2_VALUE     0x07        // GDO2 - Asserts when a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO.
#define CC_IOCFG0_VALUE     0x06        // GDO0 - Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
                                        // In RX, the pin will also deassert when a packet is discarded due to address or maximum length filtering

//#define CC_PKTCTRL1_VALUE   0b00001110  // PQT=0, CRC autoflush=1, Append=1, Address check = 10 (check, 0 is broadcast)
#define CC_PKTCTRL1_VALUE   0b00001100  // PQT=0, CRC autoflush=1, Append=1, Address check = 00 (no check)
#define CC_PKTCTRL0_VALUE   0b01000100  // WhiteData=1, PKTFrmt=norm, CRC en, Fixed Length
//#define CC_PKTCTRL0_VALUE   0b00000100  // WhiteData=0, PKTFrmt=norm, CRC en, Fixed Length
//#define CC_PKTCTRL0_VALUE   0b01000101  // WhiteData=1, PKTFrmt=norm, CRC en, Variable Length
//#define CC_PKTCTRL0_VALUE   0b00000000  // WhiteData=0, PKTFrmt=norm, CRC dis, Fixed Length
#define CC_ADDR_VALUE       0x01        // Device address.

// ========================= Bitrate-specific ==================================
#ifdef CC_BITRATE_10K
#define CC_FSCTRL1_VALUE    0x06        // Frequency synthesizer control: IF - RF studio
#define CC_FSCTRL0_VALUE    0x00        // Frequency synthesizer control: freq offset

#define CC_MDMCFG4_VALUE    0xC8        // Modem configuration: channel bandwidth
#define CC_MDMCFG3_VALUE    0x84        // Modem configuration.
#define CC_MDMCFG2_VALUE    0x13        // Filter, modulation format, Manchester coding, SYNC_MODE=011 => 30/32 sync word bits

#define CC_DEVIATN_VALUE    0x34        // Modem deviation setting - RF studio
#define CC_FREND1_VALUE     0x56        // Front end RX configuration - RF studio
#define CC_FREND0_VALUE     0x10        // Front end TX configuration.

#define CC_FOCCFG_VALUE     0x16        // Frequency Offset Compensation Configuration - RF studio
#define CC_BSCFG_VALUE      0x6C        // Bit synchronization Configuration - RF studio
//#define CC_AGCCTRL2_VALUE   0x03        // AGC control: All gain settings can be used, max gain, 33 dB magn target
#define CC_AGCCTRL2_VALUE   0x43        // AGC control: RF studio
#define CC_AGCCTRL1_VALUE   0x40        // AGC control: RF studio
#define CC_AGCCTRL0_VALUE   0x91        // AGC control: RF studio

#define CC_FSCAL3_VALUE     0xE9        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio

#define CC_TEST2_VALUE      0x81        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x35        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio

// ********************
#elif defined CC_BITRATE_38K4
#define CC_FSCTRL1_VALUE    0x06        // }
#define CC_FSCTRL0_VALUE    0x00        // } Frequency synthesizer control: RF studio, nothing to do here

#define CC_MDMCFG4_VALUE    0xCA        // }
#define CC_MDMCFG3_VALUE    0x75        // } Modem configuration: RF Studio, nothing to do here
#define CC_MDMCFG2_VALUE    0x13        // Filter, modulation format, Manchester coding, SYNC_MODE=011 => 30/32 sync word bits

#define CC_DEVIATN_VALUE    0x34        // Modem deviation setting - RF studio, nothing to do here
#define CC_FREND1_VALUE     0x56        // Front end RX configuration - RF studio, no docs, nothing to do
#define CC_FREND0_VALUE     0x10        // Front end TX configuration - RF studio, no docs, nothing to do

#define CC_FOCCFG_VALUE     0x16        // Frequency Offset Compensation - RF studio, some unknown reasons for settings
#define CC_BSCFG_VALUE      0x6C        // Bit synchronization Configuration - RF studio, some unknown reasons for settings
#define CC_AGCCTRL2_VALUE   0x03        // AGC control: 00 - all gain settings, 000 - max possible gain, 011 - target ampl=33dB
#define CC_AGCCTRL1_VALUE   0x40        // Generally, nothing interesting
#define CC_AGCCTRL0_VALUE   0x91        // AGC filter settings: RF studio, some unknown reasons for settings

#define CC_FSCAL3_VALUE     0xE9        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio, nothing to do here

#define CC_TEST2_VALUE      0x81        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x31        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio

// ********************
#elif defined CC_BITRATE_100K
#define CC_FSCTRL1_VALUE    0x08        // }
#define CC_FSCTRL0_VALUE    0x00        // } Frequency synthesizer control: RF studio, nothing to do here

#define CC_MDMCFG4_VALUE    0x5B        // }
#define CC_MDMCFG3_VALUE    0xE5        // } Modem configuration: RF Studio, nothing to do here
//#define CC_MDMCFG2_VALUE    0x11        // Filter, GFSK, no Manchester coding, SYNC_MODE=010 => 16/16 sync word bits
#define CC_MDMCFG2_VALUE    0x13        // Filter, GFSK, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits

#define CC_DEVIATN_VALUE    0x46        // Modem deviation setting: 46 kHz
#define CC_FREND1_VALUE     0xB6        // Front end RX configuration - RF studio, no docs, nothing to do
#define CC_FREND0_VALUE     0x10        // Front end TX configuration - RF studio, no docs, nothing to do

#define CC_FOCCFG_VALUE     0x1D        // Frequency Offset Compensation - RF studio, some unknown reasons for settings
#define CC_BSCFG_VALUE      0x1C        // Bit synchronization Configuration - RF studio, some unknown reasons for settings
#define CC_AGCCTRL2_VALUE   0xC7        // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
#define CC_AGCCTRL1_VALUE   0x00        // Generally, nothing interesting
#define CC_AGCCTRL0_VALUE   0xB2        // AGC filter settings: RF studio, some unknown reasons for settings

#define CC_FSCAL3_VALUE     0xEA        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio, nothing to do here

#define CC_TEST2_VALUE      0x81        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x35        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio

// ********************
#elif defined CC_BITRATE_250K
#define CC_FSCTRL1_VALUE    0x0C        // }
#define CC_FSCTRL0_VALUE    0x00        // } Frequency synthesizer control: RF studio, nothing to do here

#define CC_MDMCFG4_VALUE    0x2D        // }
#define CC_MDMCFG3_VALUE    0x2F        // } Modem configuration: RF Studio, nothing to do here
#define CC_MDMCFG2_VALUE    0x13        // Filter, modulation format, no Manchester coding, SYNC_MODE=011 => 30/32 sync word bits
//#define CC_MDMCFG2_VALUE    0x11        // Filter, modulation format, no Manchester coding, SYNC_MODE=001 => 15/16 sync word bits

#define CC_DEVIATN_VALUE    0x62        // Modem deviation setting - RF studio, nothing to do here
#define CC_FREND1_VALUE     0xB6        // Front end RX configuration - RF studio, no docs, nothing to do
#define CC_FREND0_VALUE     0x10        // Front end TX configuration - RF studio, no docs, nothing to do

#define CC_FOCCFG_VALUE     0x1D        // Frequency Offset Compensation - RF studio, some unknown reasons for settings
#define CC_BSCFG_VALUE      0x1C        // Bit synchronization Configuration - RF studio, some unknown reasons for settings
#define CC_AGCCTRL2_VALUE   0xC7        // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
#define CC_AGCCTRL1_VALUE   0x00        // Generally, nothing interesting
#define CC_AGCCTRL0_VALUE   0xB0        // AGC filter settings: RF studio, some unknown reasons for settings

#define CC_FSCAL3_VALUE     0xEA        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio, nothing to do here

#define CC_TEST2_VALUE      0x88        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x31        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio

#elif defined CC_BITRATE_500K // ================== 500k =======================
#define CC_FSCTRL1_VALUE    0x0E        // }
#define CC_FSCTRL0_VALUE    0x00        // } Frequency synthesizer control: RF studio, nothing to do here

#define CC_MDMCFG4_VALUE    0x0E        // }
#define CC_MDMCFG3_VALUE    0x2F        // } Modem configuration: RF Studio, nothing to do here
#define CC_MDMCFG2_VALUE    0x73        // Filter on, modulation format MSK, no Manchester, SYNC_MODE=011 => 30/32 sync word bits

#define CC_DEVIATN_VALUE    0x00        // Modem deviation setting - RF studio, nothing to do here
#define CC_FREND1_VALUE     0xB6        // Front end RX configuration - RF studio, no docs, nothing to do
#define CC_FREND0_VALUE     0x10        // Front end TX configuration - RF studio, no docs, nothing to do

#define CC_FOCCFG_VALUE     0x1D        // Frequency Offset Compensation - RF studio, some unknown reasons for settings
#define CC_BSCFG_VALUE      0x1C        // Bit synchronization Configuration - RF studio, some unknown reasons for settings
#define CC_AGCCTRL2_VALUE   0xC7        // AGC control: 00 - all gain settings, 000 - max possible gain, 111 - target ampl=42dB
#define CC_AGCCTRL1_VALUE   0x00        // Generally, nothing interesting
#define CC_AGCCTRL0_VALUE   0xB0        // AGC filter settings: RF studio, some unknown reasons for settings

#define CC_FSCAL3_VALUE     0xEA        // }
#define CC_FSCAL2_VALUE     0x2A        // }
#define CC_FSCAL1_VALUE     0x00        // }
#define CC_FSCAL0_VALUE     0x1F        // } Frequency synthesizer calibration: RF studio, nothing to do here

#define CC_TEST2_VALUE      0x88        // Various test settings: RF studio
#define CC_TEST1_VALUE      0x31        // Various test settings: RF studio
#define CC_TEST0_VALUE      0x09        // Various test settings: RF studio
#endif

// Rare use settings
#define CC_SYNC1_VALUE      0xD3
#define CC_SYNC0_VALUE      0x91

#define CC_CHANNR_VALUE     0x00        // Channel number.
