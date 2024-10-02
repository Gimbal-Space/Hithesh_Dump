#define RAMP_RATE 100.0f  // RPM per second, adjust as needed
#define RAMP_UPDATE_INTERVAL 10  // milliseconds

static volatile uint32_t last_ramp_update = 0;
static volatile float current_speed = 0.0f;

void RampingState(void) {
    static uint8_t substate = 0;

    switch (substate) {
        case 0: // Initialize ramping state
            current_speed = rpm_current; // Start ramping from current speed
            last_ramp_update = HAL_GetTick(); // Initialize last update time
            substate = 1;
            break;

        case 1: // Main ramping loop
            // Check if enough time has passed for an update
            uint32_t current_time = HAL_GetTick();
            if (current_time - last_ramp_update >= RAMP_UPDATE_INTERVAL) {
                float time_diff = (float)(current_time - last_ramp_update) / 1000.0f; // Convert to seconds

                // Ramp up or down towards setpoint
                if (current_speed < rpm_setpoint) {
                    current_speed += RAMP_RATE * time_diff;
                    if (current_speed > rpm_setpoint) {
                        current_speed = rpm_setpoint; // Cap at setpoint
                    }
                } else if (current_speed > rpm_setpoint) {
                    current_speed -= RAMP_RATE * time_diff;
                    if (current_speed < rpm_setpoint) {
                        current_speed = rpm_setpoint; // Cap at setpoint
                    }
                }

                // Ensure speed is within limits
                if (current_speed > rpm_max) {
                    current_speed = rpm_max;
                } else if (current_speed < rpm_min) {
                    current_speed = rpm_min;
                }

                last_ramp_update = current_time; // Update the last ramp time

                // Apply speed to motor and PID control
                speed_out = PIDController_Update(&pid_speed, current_speed, rpm_current);
                ApplySpeedToMotor(speed_out);
            }

            // Check if ramping is complete
            if (fabsf(current_speed - rpm_setpoint) < 1.0f) {  // Within 1 RPM of setpoint
                substate = 2; // Transition to next state
            }
            break;

        case 2: // Ramping complete, transition to spinning state
            SetState(SpinningState);
            substate = 0; // Reset substate for future ramping
            break;
    }
}

