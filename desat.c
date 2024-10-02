// Constants for desaturation control
#define DESAT_RAMP_RATE 50.0f       // Rate of change for desaturation value
#define DESAT_UPDATE_INTERVAL 5     // Time between desaturation updates (ms)
#define DESAT_THRESHOLD 0.9f        // Threshold to consider motor desaturated

// Global variables for desaturation state
volatile uint32_t last_desat_update = 0;    // Timestamp of last update
volatile float current_desat_value = 0.0f;  // Current value in desaturation process
volatile float desat_setpoint = 0.0f;       // Target value for desaturation

};

float calculateDesaturationMetric(void) {
    // Placeholder: Calculate saturation metric based on voltage and current
    float voltage = get_tele(VBUS_VOLTAGE);
    float current = get_tele(VBUS_CURRENT);
    return voltage / (current + 0.001f);  // Avoid division by zero
}

void updateDesaturationRampingState(void) {
    uint32_t current_time = HAL_GetTick();
    float time_diff = (float)(current_time - last_desat_update) / 1000.0f;
    
    if (current_time - last_desat_update >= DESAT_UPDATE_INTERVAL) {
        // Ramp current_desat_value towards desat_setpoint
        if (current_desat_value < desat_setpoint) {
            current_desat_value += DESAT_RAMP_RATE * time_diff;
            current_desat_value = MIN(current_desat_value, desat_setpoint);
        } else if (current_desat_value > desat_setpoint) {
            current_desat_value -= DESAT_RAMP_RATE * time_diff;
            current_desat_value = MAX(current_desat_value, desat_setpoint);
        }
        
        last_desat_update = current_time;
    }
}

STATE_FUNC(StateDesaturationRamping) {
    switch (subState) {
        case SS_0:
            // Initialize desaturation ramping
            current_desat_value = 0.0f;
            desat_setpoint = 1.0f;
            last_desat_update = HAL_GetTick();
            subState = SS_1;
            break;
        
        case SS_1:
            // Update and apply desaturation control
            updateDesaturationRampingState();
            speed_out = current_desat_value * rpm_max;  // Adjust motor speed
            
            // Check if desaturation is complete
            if (calculateDesaturationMetric() > DESAT_THRESHOLD) {
                subState = SS_2;
            }
            break;
        
        case SS_2:
            // Transition to normal operation
            SetState(StateSpinning);
            break;
    }
}

void handleMotorStates(void) {
    if (motor_running_flag == 1) {
        if (calculateDesaturationMetric() < DESAT_THRESHOLD) {
            SetState(StateDesaturationRamping);  // Motor is saturated, start desaturation
        } else if (fabsf(rpm_setpoint - rpm_current) > 10.0f) {
            SetState(StateRamping);  // Large speed difference, start ramping
        }
    } else {
        SetState(StateIdle);  // Motor not running, go to idle state
    }
}

