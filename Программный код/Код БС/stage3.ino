// stage3.ino
// БАЗОВАЯ СТАНЦИЯ — ОТПРАВИТЕЛЬ КОМАНД

#include <SPI.h>
#include <RF24.h>

#include "Data_Structures.h"

// ══════════════════════════════════════════════════════════════
// КОНФИГУРАЦИЯ
// ══════════════════════════════════════════════════════════════
#define RF24_CE_PIN  9
#define RF24_CSN_PIN 10

const uint8_t RADIO_ADDRESS_RX[6] = "CUBE2";
const uint8_t RADIO_ADDRESS_TX[6] = "CUBE1";

#define CMD_FULL_SCAN     1
#define CMD_STOP          2
#define CMD_HORIZ_SCAN    3
#define CMD_VERT_SCAN     4
#define CMD_DIAG1_SCAN    5
#define CMD_DIAG2_SCAN    6

// ══════════════════════════════════════════════════════════════
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ══════════════════════════════════════════════════════════════
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

NRF_BS2CS txPacket;
NRF_CS2BS rxPacket;

uint32_t commandsSent = 0;
uint32_t telemetryReceived = 0;
uint8_t commandCounter = 0;

// ТАЙМЕРЫ (100 мс каждый тик)
volatile uint8_t t[6] = {0};
volatile uint16_t t16 = 0;
static uint32_t next_tick_ms = 0;
const uint32_t TICK_MS = 100;

// ══════════════════════════════════════════════════════════════
// ФУНКЦИЯ ОБНОВЛЕНИЯ ТАЙМЕРОВ
// ══════════════════════════════════════════════════════════════
bool updateTimers() {
    uint32_t now = millis();
    
    if (next_tick_ms == 0 || (int32_t)(now - next_tick_ms) >= 0) {
        if (next_tick_ms == 0) next_tick_ms = now + TICK_MS;
        
        bool updated = false;
        while ((int32_t)(now - next_tick_ms) >= 0) {
            for (uint8_t i = 0; i < 6; i++) if (t[i]) t[i]--;
            if (t16) t16--;
            next_tick_ms += TICK_MS;
            updated = true;
        }
        return updated;
    }
    return false;
}

// ══════════════════════════════════════════════════════════════
// ОТПРАВКА КОМАНДЫ
// ══════════════════════════════════════════════════════════════
void sendCommand(uint8_t cmd, uint8_t script = 0xFF, int8_t angle_x = -99, 
                 int8_t angle_y = -99, uint16_t pwm_x = 0xFFFF, uint16_t pwm_y = 0xFFFF) {
    commandCounter++;
    
    txPacket.fields.header = 0x37;
    txPacket.fields.sat_id = 0x25;
    txPacket.fields.packet_num = commandCounter;
    txPacket.fields.script = script;
    txPacket.fields.time_step = 0xFF;
    txPacket.fields.time_telem = 0xFF;
    txPacket.fields.pwr_servo = 0xFF;
    txPacket.fields.pwr_laser = 0xFF;
    txPacket.fields.pwm_x = pwm_x;
    txPacket.fields.pwm_y = pwm_y;
    
    if (angle_x != -99) {
        txPacket.fields.pos_x = angleToNRF(angle_x);
    } else {
        txPacket.fields.pos_x = 0xFF;
    }
    
    if (angle_y != -99) {
        txPacket.fields.pos_y = angleToNRF(angle_y);
    } else {
        txPacket.fields.pos_y = 0xFF;
    }
    
    // ВЫЧИСЛЯЕМ CRC
    txPacket.fields.crc = 0;
    txPacket.fields.crc = calculateCRC16(txPacket.raw, sizeof(txPacket.raw));
    
    // ОТПРАВЛЯЕМ
    radio.stopListening();
    bool success = radio.write(&txPacket, sizeof(txPacket));
    radio.startListening();
    
    if (success) {
        commandsSent++;
        Serial.print(F("[Radio] Command #"));
        Serial.print(commandCounter);
        Serial.print(F(" sent ("));
        Serial.print(cmd);
        Serial.print(F(") | CRC: 0x"));
        Serial.println(txPacket.fields.crc, HEX);
    } else {
        Serial.println(F("[Radio] ERROR: Send failed!"));
    }
}

// ══════════════════════════════════════════════════════════════
// ПРИЕМ ТЕЛЕМЕТРИИ
// ══════════════════════════════════════════════════════════════
void receiveTelemetry() {
    if (radio.available()) {
        radio.read(&rxPacket, sizeof(rxPacket));
        
        uint16_t received_crc = rxPacket.fields.crc;
        rxPacket.fields.crc = 0;
        uint16_t calculated_crc = calculateCRC16(rxPacket.raw, sizeof(rxPacket.raw));
        rxPacket.fields.crc = received_crc;
        
        if (calculated_crc != received_crc) {
            Serial.println(F("[Telemetry] ERROR: CRC mismatch!"));
            return;
        }
        
        telemetryReceived++;
        
        Serial.print(F("[Telemetry] #"));
        Serial.print(rxPacket.fields.packet_num);
        Serial.print(F(" | Status: 0x"));
        Serial.print(rxPacket.fields.status, HEX);
        Serial.print(F(" | X="));
        Serial.print((-1)*rxPacket.fields.pos_x);
        Serial.print(F("° Y="));
        Serial.print((-1)*rxPacket.fields.pos_y);
        Serial.print(F("° | Laser: "));
        Serial.print(rxPacket.fields.pwr_laser ? "ON" : "OFF");
        Serial.print(F(" | Servo: "));
        Serial.println(rxPacket.fields.pwr_servo ? "ON" : "OFF");
    }
}

// ══════════════════════════════════════════════════════════════
// ОБРАБОТКА СЕРИЙНОГО ПОРТА
// ══════════════════════════════════════════════════════════════
// ══════════════════════════════════════════════════════════════
// ОБРАБОТКА СЕРИЙНОГО ПОРТА (новый парсер)
// ══════════════════════════════════════════════════════════════
void processSerialCommand() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();  // Преобразуем в верхний регистр для удобства
    
    Serial.print(F("[Serial] Command: "));
    Serial.println(input);
    
    // ──── КОМАНДА: SCAN ────
    if (input.startsWith("SCAN")) {
        String scanType = input.substring(4);
        scanType.trim();
        
        if (scanType == "1" || scanType == "FULL") {
            sendCommand(CMD_FULL_SCAN, CMD_FULL_SCAN);
            Serial.println(F("→ FULL SCAN (Horiz → Vert → Diag1 → Diag2)"));
        }
        else if (scanType == "3" || scanType == "H" || scanType == "HORIZ") {
            sendCommand(CMD_HORIZ_SCAN, CMD_HORIZ_SCAN);
            Serial.println(F("→ HORIZONTAL SCAN (X=0, Y: -40→+40)"));
        }
        else if (scanType == "4" || scanType == "V" || scanType == "VERT") {
            sendCommand(CMD_VERT_SCAN, CMD_VERT_SCAN);
            Serial.println(F("→ VERTICAL SCAN (Y=0, X: -40→+40)"));
        }
        else if (scanType == "5" || scanType == "D1" || scanType == "DIAG1") {
            sendCommand(CMD_DIAG1_SCAN, CMD_DIAG1_SCAN);
            Serial.println(F("→ DIAGONAL 1 SCAN ((-40,-40)→(+40,+40))"));
        }
        else if (scanType == "6" || scanType == "D2" || scanType == "DIAG2") {
            sendCommand(CMD_DIAG2_SCAN, CMD_DIAG2_SCAN);
            Serial.println(F("→ DIAGONAL 2 SCAN ((-40,+40)→(+40,-40))"));
        }
        else {
            Serial.println(F("? SCAN type unknown. Use: 1/FULL, 3/HORIZ, 4/VERT, 5/DIAG1, 6/DIAG2"));
        }
    }
    
    // ──── КОМАНДА: POS (POSITION) ────
    else if (input.startsWith("POS")) {
        parsePositionCommand(input);
    }
    
    // ──── КОМАНДА: STOP ────
    else if (input == "STOP") {
        sendCommand(CMD_STOP, CMD_STOP);
        Serial.println(F("→ STOP (all systems off)"));
    }
    
    // ──── СПРАВКА ────
    else if (input == "HELP" || input == "?") {
        printCommandHelp();
    }
    
    // ──── ОШИБКА ────
    else {
        Serial.println(F("? Unknown command. Type HELP for assistance"));
    }
}

// ══════════════════════════════════════════════════════════════
// ПАРСЕР КОМАНДЫ POS (POSITION)
// ══════════════════════════════════════════════════════════════
void parsePositionCommand(String input) {
    // Формат: POS X 20  |  POS Y -15  |  POS X 20 Y -15
    
    input = input.substring(3);  // Убираем "POS"
    input.trim();
    
    int8_t angle_x = -99;  // Значение по умолчанию (не установлено)
    int8_t angle_y = -99;
    
    // Парсим строку вроде "X 20 Y -15"
    int xIndex = input.indexOf('X');
    int yIndex = input.indexOf('Y');
    
    // Парсим X
    if (xIndex != -1) {
        String xStr = input.substring(xIndex + 1);
        xStr.trim();
        
        // Если есть Y после X, обрезаем до Y
        int spaceBeforeY = xStr.indexOf('Y');
        if (spaceBeforeY != -1) {
            xStr = xStr.substring(0, spaceBeforeY);
        }
        xStr.trim();
        
        angle_x = xStr.toInt();
        
        // Проверяем диапазон
        if (angle_x < -40 || angle_x > 40) {
            Serial.print(F("? X angle out of range: "));
            Serial.print(angle_x);
            Serial.println(F(" (use -40 to +40)"));
            return;
        }
    }
    
    // Парсим Y
    if (yIndex != -1) {
        String yStr = input.substring(yIndex + 1);
        yStr.trim();
        angle_y = yStr.toInt();
        
        // Проверяем диапазон
        if (angle_y < -40 || angle_y > 40) {
            Serial.print(F("? Y angle out of range: "));
            Serial.print(angle_y);
            Serial.println(F(" (use -40 to +40)"));
            return;
        }
    }
    
    // Если ничего не указано
    if (angle_x == -99 && angle_y == -99) {
        Serial.println(F("? POS syntax: POS X 20  or  POS Y -15  or  POS X 20 Y -15"));
        return;
    }
    
    // Отправляем команду
    sendCommand(CMD_STOP, 0xFF, angle_x, angle_y);
    
    // Выводим результат
    Serial.print(F("→ Position: "));
    if (angle_x != -99) {
        Serial.print(F("X="));
        Serial.print((-1)*angle_x);
        Serial.print(F("° "));
    }
    if (angle_y != -99) {
        Serial.print(F("Y="));
        Serial.print((-1)*angle_y);
        Serial.print(F("° "));
    }
    Serial.println();
}

// ══════════════════════════════════════════════════════════════
// СПРАВКА ПО КОМАНДАМ
// ══════════════════════════════════════════════════════════════
void printCommandHelp() {
    Serial.println(F("\n╔════════════════════════════════════════╗"));
    Serial.println(F("║     CUBESAT COMMAND REFERENCE         ║"));
    Serial.println(F("╚════════════════════════════════════════╝"));
    
    Serial.println(F("\n📡 SCAN COMMANDS:"));
    Serial.println(F("  SCAN 1       (or SCAN FULL)   - Full scan (all 4 patterns)"));
    Serial.println(F("  SCAN 3       (or SCAN HORIZ)  - Horizontal scan"));
    Serial.println(F("  SCAN 4       (or SCAN VERT)   - Vertical scan"));
    Serial.println(F("  SCAN 5       (or SCAN DIAG1)  - Diagonal 1 scan"));
    Serial.println(F("  SCAN 6       (or SCAN DIAG2)  - Diagonal 2 scan"));
    
    Serial.println(F("\n🎯 POSITION COMMANDS:"));
    Serial.println(F("  POS X 20          - Set X angle to 20°"));
    Serial.println(F("  POS Y -15         - Set Y angle to -15°"));
    Serial.println(F("  POS X 20 Y -15    - Set both angles"));
    Serial.println(F("  (Range: -40° to +40°)"));
    
    Serial.println(F("\n⏹️  STOP COMMAND:"));
    Serial.println(F("  STOP              - Stop all systems (laser OFF, servo OFF)"));
    
    Serial.println(F("\nℹ️  HELP:"));
    Serial.println(F("  HELP or ?         - Show this message"));
    Serial.println();
}


// ══════════════════════════════════════════════════════════════
// ИНИЦИАЛИЗАЦИЯ
// ══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    
    Serial.println(F("\n════════════════════════════════════════"));
    Serial.println(F("  BASE STATION - COMMAND & CONTROL"));
    Serial.println(F("════════════════════════════════════════\n"));
    
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
    Serial.println(F("Commands:"));
    Serial.println(F("  1 - Full Scan"));
    Serial.println(F("  2 - STOP"));
    Serial.println(F("  3 - Horizontal"));
    Serial.println(F("  4 - Vertical"));
    Serial.println(F("  5 - Diagonal 1"));
    Serial.println(F("  6 - Diagonal 2"));
    Serial.println(F("  x=-20 - Set X angle"));
    Serial.println(F("  y=+15 - Set Y angle\n"));
}

// ══════════════════════════════════════════════════════════════
// ГЛАВНЫЙ ЦИКЛ
// ══════════════════════════════════════════════════════════════
void loop() {
    updateTimers();
    receiveTelemetry();
    processSerialCommand();
    
    static uint32_t last_poll = 0;
    if (millis() - last_poll > 5000) {
        sendCommand(CMD_STOP, 0xFF);
        last_poll = millis();
    }
    
    delay(50);
}
