/*
 * kl_string.h
 *
 *  Created on: 5 дек. 2020 г.
 *      Author: layst
 */

#pragma once

int kl_strcasecmp(const char *s1, const char *s2);
char* kl_strtok(register char* s, register const char* delim, register char**PLast);
int kl_sscanf(const char* s, const char* format, ...);
int kl_strlen(const char* s);

