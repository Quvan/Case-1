#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <RF24.h>
#include "Data_Structures.h"
#include "Actuators.h"
#include "StateMachine.h"

// Глобальная переменная счетчика пакетов телеметрии
extern uint8_t telemetryCounter;

// Прототипы функций
void sendTelemetry(RF24 &radio);
void initTelemetry();

// ==================== РЕАЛИЗАЦИЯ ФУНКЦИЙ ====================

uint8_t telemetryCounter = 0;

void sendTelemetry(RF24 &radio) {
    NRF_CS2BS telemetryPacket;
    memset(&telemetryPacket, 0, sizeof(telemetryPacket));
    
    // Заполняем пакет данными
    telemetryPacket.fields.header = 0x38;
    telemetryPacket.fields.sat_id = 0x25;
    telemetryPacket.fields.packet_num = ++telemetryCounter;
    telemetryPacket.fields.script = (uint8_t)scanManager.currentState;
    telemetryPacket.fields.pos_x = currentAngleX;
    telemetryPacket.fields.pos_y = currentAngleY;
    telemetryPacket.fields.pwm_x = 1500; // Пример, можно вычислять из углов
    telemetryPacket.fields.pwm_y = 1500;
    telemetryPacket.fields.pwr_laser = laserState ? 1 : 0;
    telemetryPacket.fields.pwr_servo = 1; // Всегда включены при работе
    
    // Отправляем
    radio.stopListening();
    bool result = radio.write(&telemetryPacket, sizeof(telemetryPacket));
    radio.startListening();
    
    // Для отладки
    Serial.print(F("[CS->BS] TELEMETRY #"));
    Serial.print(telemetryCounter);
    Serial.print(F(": X="));
    Serial.print(currentAngleX);
    Serial.print(F("° Y="));
    Serial.print(currentAngleY);
    Serial.print(F("° State="));
    Serial.print(scanManager.currentState);
    Serial.print(F(" Laser="));
    Serial.print(laserState ? "ON" : "OFF");
    Serial.print(F(" Sent="));
    Serial.println(result ? "OK" : "FAIL");
}

void initTelemetry() {
    telemetryCounter = 0;
}

#endif