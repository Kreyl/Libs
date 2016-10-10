/*
 * pill.cpp
 *
 *  Created on: 22 мая 2014 г.
 *      Author: g.kruglov
 */

#include "pill_mgr.h"

#if 0

#if 1 // ================================ On Connect ===========================
uint8_t App_t::IPillHandlerUmvos() {
    uint8_t rslt = OK;
    switch(Pill.Type) {
        // ==== Cure ====
        case ptCure:
            if(Pill.ChargeCnt > 0) {    // Check charge count, decrease it and write it back
                Pill.ChargeCnt--;
                rslt = PillMgr.Write(PILL_I2C_ADDR, (PILL_START_ADDR + PILL_CHARGECNT_ADDR), &Pill.ChargeCnt, sizeof(Pill.ChargeCnt));
                // Modify dose if not dead
                if((rslt == OK) and (Dose.State != hsDeath)) Dose.Modify(Pill.Value);
            }
            else rslt = FAILURE;
            break;

        // ==== Drug ====
        case ptDrug:
            if(Pill.ChargeCnt > 0) {
                Pill.ChargeCnt--;
                rslt = PillMgr.Write(PILL_I2C_ADDR, (PILL_START_ADDR + PILL_CHARGECNT_ADDR), &Pill.ChargeCnt, sizeof(Pill.ChargeCnt));
                // Apply drug if not dead
                if((rslt == OK) and (Dose.State != hsDeath)) Dose.Drug.Set(Pill.Value, Pill.Time_s);
            }
            else rslt = FAILURE;
            break;

        // ==== Panacea ====
        case ptPanacea: Dose.Reset(); break;

        // ==== Autodoc ====
        case ptAutodoc: rslt = IProlongedPillUmvos(OK); break;

        // ==== Set DoseTop ====
        case ptSetDoseTop:
            Dose.Consts.Setup(Pill.DoseTop);
            Dose.SaveTop();
            break;

        // ==== Diagnostic ====
        case ptDiagnostic:
            PillMgr.Write(PILL_I2C_ADDR, (PILL_START_ADDR + PILL_DOSETOP_ADDR), &Dose.Consts.Top, 4);
            break;

        default:
            Uart.Printf("Unknown Pill\r");
            rslt = FAILURE;
            break;
    } // switch
    SaveDoseToPill();   // Always save DoseAfter
    return rslt;
}

uint8_t App_t::IPillHandlerGrenade() {
    uint8_t rslt = OK;
    switch(Pill.Type) {
        case ptPanacea:
            Grenade.Charge = Grenade.Capacity;
            Grenade.SaveCharge();
            Grenade.State = gsReady;
            break;
        case ptElectrostation:
            Grenade.Capacity = Pill.Capacity;
            Grenade.SaveCapacity();
            IProlongedPillGrenade(OK);
            break;

        default:
            Uart.Printf("Unknown Pill\r");
            rslt = FAILURE;
            break;
    } // switch
    return rslt;
}

uint8_t App_t::IPillHandlerEmpMech() {
    uint8_t rslt = OK;
    switch(Pill.Type) {
        case ptPanacea:
            Mech.SetState(msOperational);
            Mech.SaveState();
            break;
        case ptEmpBreaker:
            Mech.SetState(msBroken);
            Mech.SaveState();
            break;
        case ptEmpRepair:
            Mech.Health = 0;
            IProlongedPillEmpMech(OK);
            break;

        default:
            Uart.Printf("Unknown Pill\r");
            rslt = FAILURE;
            break;
    } // switch
    return rslt;
}


uint8_t App_t::IPillHandlerPillFlasher() {
    uint8_t rslt = FAILURE;
    // Write pill if data exists
    EE.ReadBuf(&Data2Wr, sizeof(Data2Wr), EE_REPDATA_ADDR);
    if(Data2Wr.Sz32 != 0) {
        Uart.Printf("#PillWrite32 0");
        for(uint8_t i=0; i<Data2Wr.Sz32; i++) Uart.Printf(",%d", Data2Wr.Data[i]);
        Uart.Printf("\r\n");
        rslt = PillMgr.Write(PILL_I2C_ADDR, PILL_START_ADDR, Data2Wr.Data, PILL_SZ);
        Uart.Ack(rslt);
    }
    return rslt;
}

// Everybody starts here
void App_t::OnPillConnect() {
    uint8_t rslt = PillMgr.Read(PILL_I2C_ADDR, PILL_START_ADDR, &Pill, sizeof(Pill_t));
    if(rslt == OK) {
        // Print pill
        Uart.Printf("#PillRead32 0 16\r\n");
        Uart.Printf("#PillData32 ");
        int32_t *p = (int32_t*)&Pill;
        for(uint32_t i=0; i<PILL_SZ32; i++) Uart.Printf("%d ", *p++);
        Uart.Printf("\r\n");
        // Everyone
        if((Pill.Type == ptSetType) and (Type != dtPillFlasher)) ISetType(Pill.DeviceType);
        else {
            switch(Type) {
                case dtUmvos:       rslt = IPillHandlerUmvos();       break;
                case dtEmpGrenade:  rslt = IPillHandlerGrenade();     break;
                case dtEmpMech:     rslt = IPillHandlerEmpMech();     break;
                case dtPillFlasher: rslt = IPillHandlerPillFlasher(); break;
                default: rslt = FAILURE; break;
            }
        } // if set type
    } // if read ok
    // Indication
    if(rslt == OK) Indication.PillGood();
    else Indication.PillBad();
}
#endif

#if 1 // ==== Prolonged action Pill ====
// Called first from pill handler, and then periodically by Timer+Event
void App_t::OnProlongedPill() {
    uint8_t rslt = PillMgr.Read(PILL_I2C_ADDR, PILL_START_ADDR, &Pill, sizeof(Pill_t));
    switch(Type) {
        case dtUmvos: IProlongedPillUmvos(rslt); break;
        case dtEmpGrenade: IProlongedPillGrenade(rslt); break;
        case dtEmpMech:    IProlongedPillEmpMech(rslt); break;
        default: break;
    }
}

uint8_t App_t::IProlongedPillUmvos(uint8_t PillState) {
    if((PillState != OK) or (Pill.Type != ptAutodoc)) {
        AutodocActive = false;
        return FAILURE;
    }
    uint8_t rslt = OK;
    if(Pill.ChargeCnt > 0) {
        // Check if timer is armed and do nothing if yes (pill reconnected between tics)
        chSysLock();
        bool IsArmed = chVTIsArmedI(&TmrProlongedPill);
        chSysUnlock();
        if(!IsArmed) {
            Pill.ChargeCnt--;
            rslt = PillMgr.Write(PILL_I2C_ADDR, (PILL_START_ADDR + PILL_CHARGECNT_ADDR), &Pill.ChargeCnt, sizeof(Pill.ChargeCnt));
            // Apply autodoc if not dead
            if((rslt == OK) and (Dose.State != hsDeath)) {
                Dose.Modify(Pill.Value);
                // Check if healing completed
                if(Dose.Value == 0) {
                    AutodocActive = false;
                    Indication.AutodocCompleted();
                }
                else {
                    AutodocActive = true;
                    StartProlongedPillTmr();
                }
            } // if rslt and !dead
        } // if is armed
    } // if chargecnt
    else {
        rslt = FAILURE;
        // Indicate charge exhausted if previously was operational
        if(AutodocActive) Indication.AutodocExhausted();
        AutodocActive = false;
    }
    SaveDoseToPill();
    return rslt;
}

void App_t::IProlongedPillGrenade(uint8_t PillState) {
    if((PillState != OK) or (Pill.Type != ptElectrostation)) {
        if(Grenade.Charge >= Grenade.Capacity) {
            Grenade.State = gsReady;
            Grenade.Charge = Grenade.Capacity;
            Grenade.SaveCharge(); // if charge > Capacity => save charge
        }
        else Grenade.State = gsDischarged;
        return;
    }
    // Check if timer is armed and do nothing if yes (pill reconnected between tics)
    chSysLock();
    bool IsArmed = chVTIsArmedI(&TmrProlongedPill);
    chSysUnlock();
    if(!IsArmed) {
        Grenade.IncreaseCharge();
        // Check if charging completed
        if(Grenade.Charge == Grenade.Capacity) Grenade.State = gsReady;
        else {
            Grenade.State = gsCharging;
            StartProlongedPillTmr();
        }
    }
    Uart.Printf("Cap=%u; Chrg=%u\r", Grenade.Capacity, Grenade.Charge);
}

void App_t::IProlongedPillEmpMech(uint8_t PillState) {
    if(Mech.GetState() == msOperational) return;
    if((PillState != OK) or (Pill.Type != ptEmpRepair)) {
        Mech.SetState(msBroken);
        Mech.SaveState();
        return;
    }
    // Check if timer is armed and do nothing if yes (pill reconnected between tics)
    chSysLock();
    bool IsArmed = chVTIsArmedI(&TmrProlongedPill);
    chSysUnlock();
    if(!IsArmed) {
        if(Mech.Health < Pill.TimeToRepair) Mech.Health++;
        // Check if repair completed
        if(Mech.Health >= Pill.TimeToRepair) {
            Mech.SetState(msOperational);
            Mech.SaveState();
        }
        else {
            Mech.SetState(msRepair);
            StartProlongedPillTmr();
        }
    }
    Uart.Printf("T2Rep=%u; Hlth=%u\r", Pill.TimeToRepair, Mech.Health);
}

#endif

#endif
