/*
 * Controls.h
 *
 *  Created on: 15 ??? 2016 ?.
 *      Author: Kreyl
 */

#pragma once

// Here, in .h file, defined only controls required outside

void ShowHeaterOn();
void ShowHeaterOff();

void ShowTPcb(float t);
void ShowTHtr(float t);
void ShowTime(uint32_t Tms);

void ShowFanStatus(bool IsOn);
void ShowTHtrManual(float t);

// Chart
extern Chart_t Chart;
extern Series_t SeriesTPcb;
extern Series_t SeriesTHtr;
