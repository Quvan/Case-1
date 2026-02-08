// PacketHandler.h
#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <Arduino.h>
#include "Data_Structures.h"

// Объявления функций
NRF_BS2CS formPacket(uint16_t mask, MirrorVars &mirror, uint8_t &packetCounter);
void printPacketHex(const NRF_BS2CS &packet);
void printPacketHuman(const NRF_BS2CS &packet);
void printMask(uint16_t mask);

#endif