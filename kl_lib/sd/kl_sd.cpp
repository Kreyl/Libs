/*
 * kl_sd.cpp
 *
 *  Created on: 13.02.2013
 *      Author: kreyl
 */

#include "kl_sd.h"
#include "sdc.h"
#include "sdc_lld.h"
#include <string.h>
#include <stdlib.h>
#include "kl_lib.h"
#include "shell.h"

sd_t SD;
extern semaphore_t semSDRW;

// Using ltm32L476, do not forget to enable SDMMC Clock somewhere else
void sd_t::Init() {
    IsReady = FALSE;
    // Bus pins
    PinSetupAlterFunc(SD_DAT0);
    PinSetupAlterFunc(SD_DAT1);
    PinSetupAlterFunc(SD_DAT2);
    PinSetupAlterFunc(SD_DAT3);
    PinSetupAlterFunc(SD_CLK);
    PinSetupAlterFunc(SD_CMD);
    // Power pin
    PinSetupOut(SD_PWR_PIN, omPushPull);
    PinSetLo(SD_PWR_PIN); // Power on
    chThdSleepMilliseconds(45);    // Let power to stabilize

    FRESULT err;
    sdcInit();
    sdcStart(&SDCD1, NULL);
    if(sdcConnect(&SDCD1)) {
        Printf("SD connect error\r");
        return;
    }
    else {
        Printf("SD capacity: %u\r", SDCD1.capacity);
    }
    err = f_mount(0, &SDC_FS);
    if(err != FR_OK) {
        Printf("SD mount error\r");
        sdcDisconnect(&SDCD1);
        return;
    }
    // Init RW semaphore
    chSemObjectInit(&semSDRW, 1);
    IsReady = true;
}

void sd_t::Standby() {
    PinSetHi(SD_PWR_PIN); // Power on
}

void sd_t::Resume() {
    PinSetLo(SD_PWR_PIN); // Power on
    chThdSleepMilliseconds(45);    // Let power to stabilize
}

// ============================== Hardware =====================================
extern "C" {

bool sdc_lld_is_card_inserted(SDCDriver *sdcp) { return TRUE; }
bool sdc_lld_is_write_protected(SDCDriver *sdcp) { return FALSE; }

}
