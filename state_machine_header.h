#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <stdint.h>
#include <stdbool.h>

// Define all the possible states of the state machine
typedef enum {
    STATE_IDLE = 0,
    STATE_SPINNING,
    STATE_RAMPING,
    STATE_DESATURATION_RAMPING,
    STATE_FAULT_HANDLING,
    // Add more states as needed
    STATE_COUNT
} MotorState;

// Typedef for state functions
typedef void (*STATE_FUNC)(void);

// Public function prototypes
void SetState(MotorState newState);
void UpdateState(void);

// State function prototypes
void StateIdle(void);
void StateSpinning(void);
void StateRamping(void);
void StateDesaturationRamping(void);
void StateFaultHandling(void);

#endif // STATEMACHINE_H
