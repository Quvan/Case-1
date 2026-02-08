// Data_Structures.h
#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <stdint.h>

// ════════════════════════════════════════════════════════════
// БИТОВЫЕ МАСКИ СОСТОЯНИЯ ТЕЛЕМЕТРИИ
// ════════════════════════════════════════════════════════════
#define STATUS_PWR_SERVO     (1 << 0)  // 0x01 — питание сервопривода
#define STATUS_PWR_LASER     (1 << 1)  // 0x02 — питание лазера
#define STATUS_PWM_X_MODE    (1 << 2)  // 0x04 — X: 1=ШИМ, 0=угол
#define STATUS_PWM_Y_MODE    (1 << 3)  // 0x08 — Y: 1=ШИМ, 0=угол
#define STATUS_PACKET_LEN_OK (1 << 4)  // 0x10 — корректная длина пакета
#define STATUS_CRC_OK        (1 << 5)  // 0x20 — корректная CRC

// ════════════════════════════════════════════════════════════
// ФУНКЦИИ ПРЕОБРАЗОВАНИЯ УГЛОВ
// ════════════════════════════════════════════════════════════
inline uint8_t angleToNRF(int8_t angle) {
    if (angle < -40) angle = -40;
    if (angle > 40) angle = 40;
    return (uint8_t)(angle + 40);
}

inline int8_t nrfToAngle(uint8_t nrf_angle) {
    if (nrf_angle > 80) nrf_angle = 80;
    return (int8_t)(nrf_angle - 40);
}

// ════════════════════════════════════════════════════════════
// ФУНКЦИИ РАСЧЁТА CRC16 (CCITT-FALSE)
// ════════════════════════════════════════════════════════════
inline uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data) {
    crc ^= ((uint16_t)data) << 8;
    for (uint8_t i = 0; i < 8; i++) {
        crc <<= 1;
        if (crc & 0x10000) crc ^= 0x1021;
    }
    return crc;
}

inline uint16_t calculateCRC16(const uint8_t* data, uint8_t length) {
    uint16_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        crc = crc16_ccitt_update(crc, data[i]);
    }
    return crc;
}

// ════════════════════════════════════════════════════════════
// ПАКЕТ КОМАНД БС → КС (24 байта)
// ════════════════════════════════════════════════════════════
union NRF_BS2CS {
    struct {
        uint8_t header;        // 0x37 — заголовок пакета команд
        uint8_t sat_id;        // 0x25 — ID спутника
        uint8_t packet_num;    // циклический номер пакета
        uint8_t script;        // скрипт/режим (0xFF = не менять)
        uint8_t time_step;     // период шага (0xFF = не менять)
        uint8_t time_telem;    // период телеметрии (0xFF = не менять)
        uint8_t pwr_servo;     // управление сервом (0xFF = не менять)
        uint8_t pwr_laser;     // управление лазером (0xFF = не менять)
        uint16_t pwm_x;        // ШИМ X (500–2500 µs, 0xFFFF = не менять)
        uint16_t pwm_y;        // ШИМ Y (500–2500 µs, 0xFFFF = не менять)
        uint8_t pos_x;         // угол X (0...80, где 40 = 0°, 0xFF = не менять)
        uint8_t pos_y;         // угол Y (0...80, где 40 = 0°, 0xFF = не менять)
        uint8_t reserved[6];   // резерв
        uint16_t crc;          // CRC16-CCITT
    } fields;
    uint8_t raw[24];
};

// ════════════════════════════════════════════════════════════
// ПАКЕТ ТЕЛЕМЕТРИИ КС → БС (24 байта)
// ════════════════════════════════════════════════════════════
union NRF_CS2BS {
    struct {
        uint8_t header;        // 0x38 — заголовок пакета телеметрии
        uint8_t sat_id;        // 0x25 — ID спутника
        uint8_t packet_num;    // номер пакета телеметрии
        uint8_t last_cmd_num;  // номер последнего принятого пакета команд
        uint32_t timestamp;    // время в мс (millis())
        uint8_t status;        // БИТОВАЯ МАСКА (STATUS_*)
        uint8_t mode;          // 0=Idle, 1=Horiz, 2=Vert, 3=Diag1, 4=Diag2
        uint8_t script_step;   // текущий шаг скрипта
        uint16_t pwm_x;        // фактический ШИМ X
        uint16_t pwm_y;        // фактический ШИМ Y
        int8_t pos_x;          // фактический угол X
        int8_t pos_y;          // фактический угол Y
        uint8_t pwr_laser;     // состояние лазера
        uint8_t pwr_servo;     // состояние сервопривода
        uint8_t reserved[3];   // резерв
        uint16_t crc;          // CRC16-CCITT
    } fields;
    uint8_t raw[24];
};

#endif
