/*
 * sd.h
 *
 *  Created on: 13.02.2013
 *      Author: kreyl
 */

#pragma once

#include "ff.h"
#include "ch.h"
#include "hal.h"
#include "kl_lib.h"
#include "uart.h"

// See SDIO clock divider in halconf.h

class sd_t {
private:
    FATFS SDC_FS;
public:
    bool IsReady;
    void Init();
    void Standby();
    void Resume();
};

extern sd_t SD;
