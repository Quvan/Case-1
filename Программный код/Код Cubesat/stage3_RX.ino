

#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

#include "Data_Structures.h"
#include "Actuators.h"
#include "StateMachine.h"

// ══════════════════════════════════════════════════════════════
// КОНФИГУРАЦИЯ NRF24
// ══════════════════════════════════════════════════════════════
#define RF24_CE_PIN  9
#define RF24_CSN_PIN 10

const uint8_t RADIO_ADDRESS_RX[6] = "CUBE1";
const uint8_t RADIO_ADDRESS_TX[6] = "CUBE2";

// ══════════════════════════════════════════════════════════════
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ══════════════════════════════════════════════════════════════
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

NRF_BS2CS rxPacket;
NRF_CS2BS txPacket;

uint32_t packetsReceived = 0;
uint32_t telemetrySent = 0;
uint8_t telemetryCounter = 0;
uint8_t lastPacketNumber = 0;
uint32_t lastTelemetryTime = 0;

bool newPacketAvailable = false;
bool sendTelemetryFlag = false;
bool autoTelemetryEnabled = true;

// ══════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    
    Serial.println(F("\n════════════════════════════════════════"));
    Serial.println(F("  CUBESAT - LASER POINTING v3.0"));
    Serial.println(F("  Status Mask + CRC Implementation"));
    Serial.println(F("════════════════════════════════════════\n"));
    
    actuatorsSetup();
    stateMachineSetup();
    
    Serial.println(F("[Radio] Initializing NRF24L01+..."));
    if (!radio.begin()) {
        Serial.println(F("[Radio] ERROR: Not found!"));
        while (1) delay(100);
    }
    
    radio.openReadingPipe(0, RADIO_ADDRESS_RX);
    radio.openWritingPipe(RADIO_ADDRESS_TX);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(100);
    radio.setPayloadSize(sizeof(NRF_BS2CS));
    radio.setRetries(3, 15);
    radio.startListening();
    
    Serial.println(F("[Radio] Ready ✓\n"));
    
    lastTelemetryTime = millis();
    sendTelemetryFlag = true;
}

// ══════════════════════════════════════════════════════════════
// ПРИЕМ ПАКЕТОВ КОМАНД
// ══════════════════════════════════════════════════════════════
void receivePackets() {
    if (radio.available()) {
        radio.read(&rxPacket, sizeof(rxPacket));
        newPacketAvailable = true;
        packetsReceived++;
        
        Serial.print(F("[Radio] Command #"));
        Serial.print(rxPacket.fields.packet_num);
        Serial.println(F(" received"));
    }
}

// ══════════════════════════════════════════════════════════════
// ПРОВЕРКА CRC И ОБРАБОТКА ПАКЕТА
// ══════════════════════════════════════════════════════════════
void processPacket() {
    if (!newPacketAvailable) return;
    newPacketAvailable = false;
    
    if (rxPacket.fields.header != 0x37) {
        Serial.println(F("[Packet] ERROR: Invalid header!"));
        statusMask &= ~STATUS_CRC_OK;
        return;
    }
    
    if (rxPacket.fields.sat_id != 0x25) {
        Serial.println(F("[Packet] WARNING: Not for this satellite"));
        statusMask &= ~STATUS_CRC_OK;
        return;
    }
    
    // ПРОВЕРКА CRC
    uint16_t received_crc = rxPacket.fields.crc;
    rxPacket.fields.crc = 0;
    uint16_t calculated_crc = calculateCRC16(rxPacket.raw, sizeof(rxPacket.raw));
    rxPacket.fields.crc = received_crc;
    
    if (calculated_crc != received_crc) {
        Serial.print(F("[Packet] CRC ERROR! Got 0x"));
        Serial.print(received_crc, HEX);
        Serial.print(F(", expected 0x"));
        Serial.println(calculated_crc, HEX);
        statusMask &= ~STATUS_CRC_OK;
        return;
    }
    
    statusMask |= STATUS_CRC_OK;
    statusMask |= STATUS_PACKET_LEN_OK;
    
    bool changesMade = false;
    
    // ──── ПОЗИЦИЯ ────
    if (rxPacket.fields.pos_x != 0xFF) {
        int8_t angle_x = nrfToAngle(rxPacket.fields.pos_x);
        updatePositionX(angle_x);
        changesMade = true;
    }
    
    if (rxPacket.fields.pos_y != 0xFF) {
        int8_t angle_y = nrfToAngle(rxPacket.fields.pos_y);
        updatePositionY(angle_y);
        changesMade = true;
    }
    
    // ──── ШИМ (приоритет выше) ────
    if (rxPacket.fields.pwm_x != 0xFFFF) {
        updatePWM_X(rxPacket.fields.pwm_x);
        changesMade = true;
    }
    
    if (rxPacket.fields.pwm_y != 0xFFFF) {
        updatePWM_Y(rxPacket.fields.pwm_y);
        changesMade = true;
    }
    
    // ──── ЛАЗЕР ────
    if (rxPacket.fields.pwr_laser != 0xFF) {
        setLaser(rxPacket.fields.pwr_laser == 1);
        changesMade = true;
    }
    
    // ──── СКРИПТ ────
    if (rxPacket.fields.script != 0xFF) {
        processScriptCommand(rxPacket.fields.script);
        changesMade = true;
    }
    
    lastPacketNumber = rxPacket.fields.packet_num;
    
    if (changesMade) {
        sendTelemetryFlag = true;
        Serial.println(F("[Packet] Processed ✓"));
    }
}

// ══════════════════════════════════════════════════════════════
// ОТПРАВКА ТЕЛЕМЕТРИИ (с CRC)
// ══════════════════════════════════════════════════════════════
void sendTelemetry() {
    if (!sendTelemetryFlag && !autoTelemetryEnabled) return;
    sendTelemetryFlag = false;
    telemetryCounter++;
    
    txPacket.fields.header = 0x38;
    txPacket.fields.sat_id = 0x25;
    txPacket.fields.packet_num = telemetryCounter;
    txPacket.fields.last_cmd_num = lastPacketNumber;
    txPacket.fields.timestamp = millis();
    txPacket.fields.status = statusMask;
    txPacket.fields.mode = stateManager.currentState;
    txPacket.fields.script_step = stateManager.currentStep;
    txPacket.fields.pwm_x = angleToPWM(currentAngleX, SERVO_X_MIN_US, SERVO_X_MAX_US);
    txPacket.fields.pwm_y = angleToPWM(currentAngleY, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
    txPacket.fields.pos_x = currentAngleX;
    txPacket.fields.pos_y = currentAngleY;
    txPacket.fields.pwr_laser = laserState ? 1 : 0;
    txPacket.fields.pwr_servo = (stateManager.currentState != STATE_IDLE) ? 1 : 0;
    
    // ВЫЧИСЛЯЕМ CRC
    txPacket.fields.crc = 0;
    txPacket.fields.crc = calculateCRC16(txPacket.raw, sizeof(txPacket.raw));
    
    // ОТПРАВЛЯЕМ
    radio.stopListening();
    bool success = radio.write(&txPacket, sizeof(txPacket));
    radio.startListening();
    
    if (success) {
        telemetrySent++;
        Serial.print(F("[Telemetry] #"));
        Serial.print(telemetryCounter);
        Serial.print(F(" → Status: 0x"));
        Serial.print(statusMask, HEX);
        Serial.print(F(" | CRC: 0x"));
        Serial.println(txPacket.fields.crc, HEX);
    } else {
        Serial.println(F("[Telemetry] ERROR: Send failed!"));
    }
}

// ══════════════════════════════════════════════════════════════
// ПЕРИОДИЧЕСКАЯ ТЕЛЕМЕТРИЯ
// ══════════════════════════════════════════════════════════════
void periodicTelemetry() {
    uint32_t currentTime = millis();
    if (autoTelemetryEnabled && (currentTime - lastTelemetryTime >= 3000)) {
        sendTelemetryFlag = true;
        lastTelemetryTime = currentTime;
    }
}

// ══════════════════════════════════════════════════════════════
// ГЛАВНЫЙ ЦИКЛ
// ══════════════════════════════════════════════════════════════
void loop() {
    checkEmergencyStop();
    receivePackets();
    processPacket();
    updateStateMachine();
    sendTelemetry();
    periodicTelemetry();
    
    delay(100);
}
