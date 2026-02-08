// StateMachine.h
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

// ══════════════════════════════════════════════════════════════
// РЕЖИМЫ СИСТЕМЫ
// ══════════════════════════════════════════════════════════════
enum SystemState {
    STATE_IDLE = 0,
    STATE_SCAN_HORIZONTAL = 1,
    STATE_SCAN_VERTICAL = 2,
    STATE_SCAN_DIAGONAL_1 = 3,
    STATE_SCAN_DIAGONAL_2 = 4,
    STATE_MANUAL = 5
};

// ══════════════════════════════════════════════════════════════
// МЕНЕДЖЕР СОСТОЯНИЯ
// ══════════════════════════════════════════════════════════════
struct StateManager {
    SystemState currentState;
    SystemState nextState;
    uint8_t currentStep;
    int8_t targetAngleX;
    int8_t targetAngleY;
    bool moveComplete;
    uint32_t lastStepTime;
    uint32_t stepInterval;
};

extern StateManager stateManager;

// ══════════════════════════════════════════════════════════════
// ФУНКЦИИ
// ══════════════════════════════════════════════════════════════
void stateMachineSetup();
void updateStateMachine();
void setSystemState(SystemState newState);
void executeScanStep();
void stopAllActions();
void processScriptCommand(uint8_t script);

#endif
