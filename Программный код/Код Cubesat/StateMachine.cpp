// StateMachine.cpp
#include <Arduino.h>
#include "Data_Structures.h"
#include "Actuators.h"
#include "StateMachine.h"


StateManager stateManager;
bool autoScanEnabled = false;

void stateMachineSetup() {
    stateManager.currentState = STATE_IDLE;
    stateManager.nextState = STATE_IDLE;
    stateManager.currentStep = 0;
    stateManager.targetAngleX = 0;
    stateManager.targetAngleY = 0;
    stateManager.moveComplete = true;
    stateManager.lastStepTime = millis();
    stateManager.stepInterval = 300;
    
    Serial.println(F("[StateMachine] Initialized ✓"));
}

void updateStateMachine() {
    if (!autoScanEnabled) return;
    
    uint32_t currentTime = millis();
    if (currentTime - stateManager.lastStepTime >= stateManager.stepInterval) {
        executeScanStep();
        stateManager.lastStepTime = currentTime;
    }
}

void setSystemState(SystemState newState) {
    if (stateManager.currentState == newState) return;
    
    Serial.print(F("[StateMachine] State: "));
    Serial.print(stateManager.currentState);
    Serial.print(F(" → "));
    Serial.println(newState);
    
    stateManager.currentState = newState;
    stateManager.currentStep = 0;
    stateManager.moveComplete = false;
    stateManager.lastStepTime = millis();
    
    switch (newState) {
        case STATE_IDLE:
            autoScanEnabled = false;
            setServo(false);
            Serial.println(F("  → IDLE"));
            break;
            
        case STATE_SCAN_HORIZONTAL:
            autoScanEnabled = true;
            setServo(true);
            stateManager.targetAngleX = 0;
            stateManager.targetAngleY = -40;
            updatePositionXY(0, -40);
            Serial.println(F("  → HORIZONTAL (X=0, Y: -40→+40)"));
            break;
            
        case STATE_SCAN_VERTICAL:
            autoScanEnabled = true;
            setServo(true);
            stateManager.targetAngleX = -40;
            stateManager.targetAngleY = 0;
            updatePositionXY(-40, 0);
            Serial.println(F("  → VERTICAL (Y=0, X: -40→+40)"));
            break;
            
        case STATE_SCAN_DIAGONAL_1:
            autoScanEnabled = true;
            setServo(true);
            stateManager.targetAngleX = -40;
            stateManager.targetAngleY = -40;
            updatePositionXY(-40, -40);
            Serial.println(F("  → DIAGONAL 1 (-40,-40)→(+40,+40)"));
            break;
            
        case STATE_SCAN_DIAGONAL_2:
            autoScanEnabled = true;
            setServo(true);
            stateManager.targetAngleX = -40;
            stateManager.targetAngleY = 40;
            updatePositionXY(-40, 40);
            Serial.println(F("  → DIAGONAL 2 (-40,+40)→(+40,-40)"));
            break;
            
        case STATE_MANUAL:
            autoScanEnabled = false;
            Serial.println(F("  → MANUAL"));
            break;
    }
}

void executeScanStep() {
    if (!autoScanEnabled) return;
    
    stateManager.currentStep++;
    
    switch (stateManager.currentState) {
        case STATE_SCAN_HORIZONTAL:
            stateManager.targetAngleY += 10;
            if (stateManager.targetAngleY > 40) {
                setSystemState(STATE_SCAN_VERTICAL);
                return;
            }
            Serial.print(F("[Step] HORIZ: Y = "));
            Serial.println(stateManager.targetAngleY);
            break;
            
        case STATE_SCAN_VERTICAL:
            stateManager.targetAngleX += 10;
            if (stateManager.targetAngleX > 40) {
                setSystemState(STATE_SCAN_DIAGONAL_1);
                return;
            }
            Serial.print(F("[Step] VERT: X = "));
            Serial.println(stateManager.targetAngleX);
            break;
            
        case STATE_SCAN_DIAGONAL_1:
            stateManager.targetAngleX += 10;
            stateManager.targetAngleY += 10;
            if (stateManager.targetAngleX > 40 || stateManager.targetAngleY > 40) {
                setSystemState(STATE_SCAN_DIAGONAL_2);
                return;
            }
            Serial.print(F("[Step] DIAG1: X="));
            Serial.print(stateManager.targetAngleX);
            Serial.print(F(", Y="));
            Serial.println(stateManager.targetAngleY);
            break;
            
        case STATE_SCAN_DIAGONAL_2:
            stateManager.targetAngleX += 10;
            stateManager.targetAngleY -= 10;
            if (stateManager.targetAngleX > 40 || stateManager.targetAngleY < -40) {
                setSystemState(STATE_IDLE);
                Serial.println(F("[StateMachine] ★ SCAN COMPLETE ✓"));
                return;
            }
            Serial.print(F("[Step] DIAG2: X="));
            Serial.print(stateManager.targetAngleX);
            Serial.print(F(", Y="));
            Serial.println(stateManager.targetAngleY);
            break;
            
        default:
            return;
    }
    
    updatePositionXY(stateManager.targetAngleX, stateManager.targetAngleY);
}

void stopAllActions() {
    setSystemState(STATE_IDLE);
    setLaser(false);
    setServo(false);
    Serial.println(F("[StateMachine] ✓ STOP"));
}

void processScriptCommand(uint8_t script) {
    Serial.print(F("[Script] Command #"));
    Serial.println(script);
    
    switch (script) {
        case 1: setSystemState(STATE_SCAN_HORIZONTAL); setLaser(true); break;
        case 2: stopAllActions(); break;
        case 3: setSystemState(STATE_SCAN_HORIZONTAL); setLaser(true); break;
        case 4: setSystemState(STATE_SCAN_VERTICAL); setLaser(true); break;
        case 5: setSystemState(STATE_SCAN_DIAGONAL_1); setLaser(true); break;
        case 6: setSystemState(STATE_SCAN_DIAGONAL_2); setLaser(true); break;
        default: break;
    }
}
