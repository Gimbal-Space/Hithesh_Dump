#include <string.h> // For memset

// Constants for fault handling
#define TEMP_THRESHOLD 100.0f        // Example threshold for over-temperature
#define CURRENT_THRESHOLD 10.0f      // Example threshold for over-current
#define VOLTAGE_MAX_THRESHOLD 50.0f  // Example maximum voltage threshold
#define VOLTAGE_MIN_THRESHOLD 10.0f   // Example minimum voltage threshold

// HealthFlags structure definition
typedef struct {
    uint32_t overtemperature : 1;
    uint32_t overcurrent : 1;
    uint32_t overvoltage : 1;
    uint32_t undervoltage : 1;
    uint32_t hall_sensor_fault : 1;
    uint32_t communication_fault : 1;
} HealthFlags;

volatile HealthFlags health_flags = {0};
volatile uint8_t fault_active = 0;

// Add this to your state enum if not already present
enum {
    // ... existing states ...
    FAULT_HANDLING,
    // ... other states ...
};

// Function to check if any health flags are set
uint8_t checkHealthFlags(void) {
    return (health_flags.overtemperature ||
            health_flags.overcurrent ||
            health_flags.overvoltage ||
            health_flags.undervoltage ||
            health_flags.hall_sensor_fault ||
            health_flags.communication_fault);
}

// Function to put the system in a safe state
void enterSafeState(void) {
    // Disable motor driver outputs
    MotorSetPWM(0, 0, 0, 0);
    
    // Disable any power stage
    HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_RESET);
    
    // Set fault indicator
    HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
    
    // Log the fault occurrence
    // You might want to implement a logging mechanism here
}

// Function to attempt recovering from a fault
uint8_t attemptFaultRecovery(void) {
    // Check if fault conditions are cleared
    if (!checkHealthFlags()) {
        // Reset fault flags
        memset(&health_flags, 0, sizeof(HealthFlags));
        
        // Re-enable power stage
        HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_SET);
        
        // Clear fault indicator
        HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
        
        return 1; // Recovery successful
    }
    return 0; // Fault conditions still present
}

// State function for handling operational faults
STATE_FUNC(StateFaultHandling) {
    switch (subState) {
        case SS_0:
            // Enter safe state
            enterSafeState();
            fault_active = 1;
            subState = SS_1;
            break;
        
        case SS_1:
            // Attempt recovery periodically
            static uint32_t last_recovery_attempt = 0;
            uint32_t current_time = HAL_GetTick();
            
            if (current_time - last_recovery_attempt >= 5000) { // Try every 5 seconds
                if (attemptFaultRecovery()) {
                    fault_active = 0;
                    SetState(StateIdle); // Transition to idle state after recovery
                } else {
                    last_recovery_attempt = current_time; // Update attempt time
                }
            }
            break;
    }
}

// Modify your handleMotorStates function to include fault checking
void handleMotorStates(void) {
    if (checkHealthFlags()) {
        SetState(StateFaultHandling);
    } else if (motor_running_flag == 1) {
        // Existing motor running logic
        if (fabsf(rpm_setpoint - rpm_current) > 10.0f) {
            SetState(StateRamping);  // Large speed difference, start ramping
        }
    } else {
        SetState(StateIdle); // Transition to idle if not running
    }
}

