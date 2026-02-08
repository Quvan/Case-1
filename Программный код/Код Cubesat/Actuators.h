// Actuators.h
#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Servo.h>

// ══════════════════════════════════════════════════════════════
// ПИНЫ ПОДКЛЮЧЕНИЯ
// ══════════════════════════════════════════════════════════════
#define SERVO_X_PIN      3     // PWM сервопривод X
#define SERVO_Y_PIN      5     // PWM сервопривод Y
#define LASER_PIN        7     // Лазер (HIGH = включен)
#define EMERGENCY_BTN    2     // Кнопка аварийной остановки (ACTIVE LOW)

// ══════════════════════════════════════════════════════════════
// КАЛИБРОВКА СЕРВОПРИВОВ (µs)
// ══════════════════════════════════════════════════════════════
#define SERVO_X_MIN_US  1000   // -40°
#define SERVO_X_MAX_US  2000   // +40°
#define SERVO_X_CENTER  1500   // 0°

#define SERVO_Y_MIN_US  1100   // -40°
#define SERVO_Y_MAX_US  2100   // +40°
#define SERVO_Y_CENTER  1600   // 0°

#define ANGLE_MIN      -40
#define ANGLE_MAX      40

// ══════════════════════════════════════════════════════════════
// ЭКСТЕРНЫЕ ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ══════════════════════════════════════════════════════════════
extern Servo servoX;
extern Servo servoY;
extern int8_t currentAngleX;
extern int8_t currentAngleY;
extern bool laserState;
extern bool servoState;
extern uint8_t statusMask;

// ══════════════════════════════════════════════════════════════
// ФУНКЦИИ
// ══════════════════════════════════════════════════════════════
void actuatorsSetup();
void updatePositionX(int8_t angle);
void updatePositionY(int8_t angle);
void updatePositionXY(int8_t x, int8_t y);
void updatePWM_X(uint16_t pwm);
void updatePWM_Y(uint16_t pwm);
void updatePWM_XY(uint16_t pwm_x, uint16_t pwm_y);
void setLaser(bool state);
void setServo(bool state);
int16_t angleToPWM(int8_t angle, int16_t min_us, int16_t max_us);
int8_t pwmToAngle(uint16_t pwm, int16_t min_us, int16_t max_us);
void printCurrentState();
void checkEmergencyStop();
void performEmergencyStop();
void emergencyButtonISR();  // ← ЭТА ФУНКЦИЯ ОТСУТСТВУЕТ!

#endif
