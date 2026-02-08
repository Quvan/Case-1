// Actuators.cpp
#include <Arduino.h>
#include <Servo.h>
#include "Data_Structures.h"
#include "Actuators.h"


// ══════════════════════════════════════════════════════════════
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ══════════════════════════════════════════════════════════════
Servo servoX;
Servo servoY;
int8_t currentAngleX = 0;
int8_t currentAngleY = 0;
bool laserState = false;
bool servoState = true;
uint8_t statusMask = 0;

volatile bool emergencyPressed = false;

// ══════════════════════════════════════════════════════════════
// ИНИЦИАЛИЗАЦИЯ
// ══════════════════════════════════════════════════════════════
void actuatorsSetup() {
    Serial.println(F("[Actuators] Initializing..."));
    
    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);
    
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    laserState = false;
    
    pinMode(EMERGENCY_BTN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EMERGENCY_BTN), emergencyButtonISR, FALLING);
    
    servoX.writeMicroseconds(SERVO_X_CENTER);
    servoY.writeMicroseconds(SERVO_Y_CENTER);
    currentAngleX = 0;
    currentAngleY = 0;
    
    statusMask = STATUS_PACKET_LEN_OK | STATUS_CRC_OK;
    
    Serial.println(F("[Actuators] Initialized ✓"));
    printCurrentState();
}

// ══════════════════════════════════════════════════════════════
// ПРЕОБРАЗОВАНИЯ
// ══════════════════════════════════════════════════════════════
int16_t angleToPWM(int8_t angle, int16_t min_us, int16_t max_us) {
    if (angle < ANGLE_MIN) angle = ANGLE_MIN;
    if (angle > ANGLE_MAX) angle = ANGLE_MAX;
    return map(angle, ANGLE_MIN, ANGLE_MAX, min_us, max_us);
}

int8_t pwmToAngle(uint16_t pwm, int16_t min_us, int16_t max_us) {
    if (pwm < min_us) pwm = min_us;
    if (pwm > max_us) pwm = max_us;
    return map(pwm, min_us, max_us, ANGLE_MIN, ANGLE_MAX);
}

// ══════════════════════════════════════════════════════════════
// ОБНОВЛЕНИЕ ПОЗИЦИИ ПО УГЛАМ
// ══════════════════════════════════════════════════════════════
void updatePositionX(int8_t angle) {
    currentAngleX = angle;
    uint16_t pwm = angleToPWM(angle, SERVO_X_MIN_US, SERVO_X_MAX_US);
    servoX.writeMicroseconds(pwm);
    statusMask &= ~STATUS_PWM_X_MODE;
    
    Serial.print(F("[Actuators] X → "));
    Serial.print(angle);
    Serial.println(F("°"));
}

void updatePositionY(int8_t angle) {
    currentAngleY = angle;
    uint16_t pwm = angleToPWM(angle, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
    servoY.writeMicroseconds(pwm);
    statusMask &= ~STATUS_PWM_Y_MODE;
    
    Serial.print(F("[Actuators] Y → "));
    Serial.print(angle);
    Serial.println(F("°"));
}

void updatePositionXY(int8_t x, int8_t y) {
    updatePositionX(x);
    updatePositionY(y);
}

// ══════════════════════════════════════════════════════════════
// ОБНОВЛЕНИЕ ШИМ НАПРЯМУЮ
// ══════════════════════════════════════════════════════════════
void updatePWM_X(uint16_t pwm) {
    servoX.writeMicroseconds(pwm);
    currentAngleX = pwmToAngle(pwm, SERVO_X_MIN_US, SERVO_X_MAX_US);
    statusMask |= STATUS_PWM_X_MODE;
    
    Serial.print(F("[Actuators] X PWM → "));
    Serial.print(pwm);
    Serial.print(F(" µs ("));
    Serial.print(currentAngleX);
    Serial.println(F("°)"));
}

void updatePWM_Y(uint16_t pwm) {
    servoY.writeMicroseconds(pwm);
    currentAngleY = pwmToAngle(pwm, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
    statusMask |= STATUS_PWM_Y_MODE;
    
    Serial.print(F("[Actuators] Y PWM → "));
    Serial.print(pwm);
    Serial.print(F(" µs ("));
    Serial.print(currentAngleY);
    Serial.println(F("°)"));
}

void updatePWM_XY(uint16_t pwm_x, uint16_t pwm_y) {
    updatePWM_X(pwm_x);
    updatePWM_Y(pwm_y);
}

// ══════════════════════════════════════════════════════════════
// УПРАВЛЕНИЕ ЛАЗЕРОМ И СЕРВОМ
// ══════════════════════════════════════════════════════════════
void setLaser(bool state) {
    laserState = state;
    digitalWrite(LASER_PIN, state ? HIGH : LOW);
    
    if (state) {
        statusMask |= STATUS_PWR_LASER;
    } else {
        statusMask &= ~STATUS_PWR_LASER;
    }
    
    Serial.print(F("[Actuators] Laser "));
    Serial.println(state ? F("ON") : F("OFF"));
}

void setServo(bool state) {
    servoState = state;
    
    if (state) {
        statusMask |= STATUS_PWR_SERVO;
    } else {
        statusMask &= ~STATUS_PWR_SERVO;
    }
    
    Serial.print(F("[Actuators] Servo "));
    Serial.println(state ? F("ON") : F("OFF"));
}

// ══════════════════════════════════════════════════════════════
// ДИАГНОСТИКА
// ══════════════════════════════════════════════════════════════
void printCurrentState() {
    Serial.println(F("\n[Actuators Status]"));
    Serial.print(F("  X: "));
    Serial.print(currentAngleX);
    Serial.println(F("°"));
    
    Serial.print(F("  Y: "));
    Serial.print(currentAngleY);
    Serial.println(F("°"));
    
    Serial.print(F("  Laser: "));
    Serial.println(laserState ? F("ON") : F("OFF"));
    
    Serial.print(F("  Status: 0x"));
    Serial.println(statusMask, HEX);
    Serial.println();
}

// ══════════════════════════════════════════════════════════════
// АВАРИЙНАЯ ОСТАНОВКА (ISR)
// ══════════════════════════════════════════════════════════════
void emergencyButtonISR() {
    emergencyPressed = true;
}

void checkEmergencyStop() {
    if (emergencyPressed) {
        performEmergencyStop();
        emergencyPressed = false;
    }
}

void performEmergencyStop() {
    digitalWrite(LASER_PIN, LOW);
    laserState = false;
    statusMask &= ~STATUS_PWR_LASER;
    
    servoX.write(90);
    servoY.write(90);
    currentAngleX = 0;
    currentAngleY = 0;
    
    Serial.println(F("\n!!! EMERGENCY STOP ACTIVATED !!!"));
    
    for (int i = 0; i < 10; i++) {
        digitalWrite(13, HIGH);
        delay(100);
        digitalWrite(13, LOW);
        delay(100);
    }
}
