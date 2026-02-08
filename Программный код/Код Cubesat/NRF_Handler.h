// NRF_Handler.h
#ifndef NRF_HANDLER_H
#define NRF_HANDLER_H

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Data_Structures.h"

// Объявления функций
bool initNRF24(RF24 &radio, const byte address[6]);
bool sendPacket(RF24 &radio, const NRF_BS2CS &packet);

#endif