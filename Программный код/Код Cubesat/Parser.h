// Parser.h
#ifndef PARSER_H
#define PARSER_H

#include <Arduino.h>
#include "Data_Structures.h"

// Объявления функций
bool readLine(char *buf, uint8_t maxLen, uint32_t timeoutMs = 10000);
void cleanChars(char *s);
void normalizeSpaces(char *s);
void toUpperInPlace(char *s);
bool parseIntSafe(const char *s, long &out);
void parseCommand(const char *src, ParsedCommand &pc);
void printParsed(const ParsedCommand &pc, uint32_t num);

#endif