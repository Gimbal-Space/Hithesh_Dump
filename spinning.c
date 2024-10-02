#include <stdint.h>
#include <stdbool.h>
#define HALL_SENSOR_COUNT 6 // Number of pulses per revolution for 3 Hall sensors
#define TIME_INTERVAL 0.1f 
// Global variables
static uint32_t hall_pulse_count = 0; // Number of hall sensor pulses since last measurement
static float last_time = 0;
void HallSensorInterrupt(void) {
    // Increment the pulse count for each Hall sensor trigger
    hall_pulse_count++;
}
float CalculateSpeedFromHallSensor(void) {
    
    float current_time = GetCurrentTime(); 

    // Calculate the time interval since the last calculation
    float time_interval = current_time - last_time;
    
    // If the time interval is too short, return the last known RPM
    if (time_interval < TIME_INTERVAL) {
        return rpm_current; // Return the last known RPM to avoid calculating too often
    }

    // Calculate RPM
    float rpm = (hall_pulse_count * 60.0f) / (HALL_SENSOR_COUNT * time_interval);
    
    // Reset pulse count and update last time
    hall_pulse_count = 0;
    last_time = current_time;

    return rpm;
}
float GetCurrentTime(void) {
    
    return (float)HAL_GetTick() / 1000.0f; // Convert milliseconds to seconds
}

void SpinningState(void) {
    static uint8_t substate = 0;
    
    switch (substate) {
        case 0: // Initialize spinning state
            motor_running_flag = 1;
            PIDController_Reset(&pid_speed);
            substate = 1;
            break;
        
        case 1: // Main spinning loop
            // Update current speed
            UpdateCurrentSpeed();
            
            // Calculate PID output
            speed_out = PIDController_Update(&pid_speed, rpm_setpoint, rpm_current);
            
            // Apply speed to motor
            ApplySpeedToMotor(speed_out);
            
            // Check for new commands
            if (HAL_UART_Receive(&huart3, rx_buffer, PACKET_SIZE, 10) == HAL_OK) {
                ProcessReceivedPacket();
                if (!motor_running_flag) {
                    substate = 2;
                }
            }
            
            // Update telemetry
            UpdateTelemetry();
            break;
        
        case 2: // Transition out of spinning
            MotorSetPWM(0, 0, 0, 0);
            SetState(IdleState);
            substate = 0;
            break;
    }
}

// Update current motor speed
void UpdateCurrentSpeed(void) {
  
    rpm_current = CalculateSpeedFromHallSensor();
      if (rpm_current > rpm_max) {
        
        rpm_current = rpm_max; 
    }
}

// Apply calculated speed to motor
void ApplySpeedToMotor(float speed) {
    // Convert speed to appropriate PWM values
    float pwm_value = speed / rpm_max;
    
    // Limit PWM value
    if (pwm_value > 1.0f) pwm_value = 1.0f;
    if (pwm_value < 0.0f) pwm_value = 0.0f;
    
    // Apply PWM to motor
    //set based on motor control metghod
    MotorSetPWM(pwm_value, pwm_value, pwm_value, GetHallSensorState());
}
