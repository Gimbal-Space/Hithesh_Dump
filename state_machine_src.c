#include "state_machine_header.h"
#include <string.h>

typedef void STATE_FUNC(void);
static STATE_FUNC *currentState;

// Define the substates enum
typedef enum {
    SS_0 = 0,
    SS_1,
    SS_2,
    SS_3,
    SS_4,
    SS_5,
    SS_6,
    SS_7,
    SS_8,
} SubState;

// Define state types
typedef enum {
    STATE_IDLE,
    STATE_RAMPING,
    STATE_SPINNING,
    STATE_DESATURATION_RAMPING,
    STATE_FAULT_HANDLING,
    STATE_COUNT
} State;

// Constants
#define HALL_SENSOR_COUNT 6
#define TIME_INTERVAL 0.1f
#define RAMP_RATE 100.0f
#define RAMP_UPDATE_INTERVAL 10
#define DESAT_RAMP_RATE 50.0f
#define DESAT_UPDATE_INTERVAL 5
#define DESAT_THRESHOLD 0.9f
#define TEMP_THRESHOLD 100.0f
#define CURRENT_THRESHOLD 10.0f
#define VOLTAGE_MAX_THRESHOLD 50.0f
#define VOLTAGE_MIN_THRESHOLD 10.0f

// Global variables
static SubState subState = SS_0;
static uint32_t hall_pulse_count = 0;
static float last_time = 0;
static volatile uint32_t last_ramp_update = 0;
static volatile float current_speed = 0.0f;
static volatile uint32_t last_desat_update = 0;
static volatile float current_desat_value = 0.0f;
static volatile float desat_setpoint = 0.0f;
static volatile uint8_t motor_running_flag = 0;
static float rpm_current = 0;
static float rpm_setpoint = 0;
static float rpm_max = 0;
static float rpm_min = 0;
static float speed_out = 0;

typedef struct {
    uint32_t overtemperature : 1;
    uint32_t overcurrent : 1;
    uint32_t overvoltage : 1;
    uint32_t undervoltage : 1;
    uint32_t hall_sensor_fault : 1;
    uint32_t communication_fault : 1;
} HealthFlags;

static volatile HealthFlags health_flags = {0};
static volatile uint8_t fault_active = 0;

// Function declarations
static void StateIdle(void);
static void StateRamping(void);
static void StateSpinning(void);
static void StateDesaturationRamping(void);
static void StateFaultHandling(void);
static void SetState(STATE_FUNC *newState);
static uint8_t checkHealthFlags(void);
static void enterSafeState(void);
static uint8_t attemptFaultRecovery(void);
static void UpdateCurrentSpeed(void);
static void ApplySpeedToMotor(float speed);
static float CalculateSpeedFromHallSensor(void);
static float GetCurrentTime(void);
static float calculateDesaturationMetric(void);
static void updateDesaturationRampingState(void);
static void ProcessReceivedPacket(void);
static void UpdateTelemetry(void);
static void MotorSetPWM(float a, float b, float c, uint8_t hall_state);
static float get_tele(uint8_t parameter);
static uint8_t GetHallSensorState(void);

// Array of state functions
static const STATE_FUNC* const state_functions[STATE_COUNT] = {
    StateIdle,
    StateRamping,
    StateSpinning,
    StateDesaturationRamping,
    StateFaultHandling
};

// Function implementations
static void SetState(STATE_FUNC *newState) {
    currentState = newState;
    subState = SS_0;
}

uint8_t checkHealthFlags(void) {
    return (health_flags.overtemperature ||
            health_flags.overcurrent ||
            health_flags.overvoltage ||
            health_flags.undervoltage ||
            health_flags.hall_sensor_fault ||
            health_flags.communication_fault);
}

void enterSafeState(void) {
    MotorSetPWM(0, 0, 0, 0);
    HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
}

uint8_t attemptFaultRecovery(void) {
    if (!checkHealthFlags()) {
        memset(&health_flags, 0, sizeof(HealthFlags));
        HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
        return 1;
    }
    return 0;
}

void StateMachine_Init(void) {
    SetState(StateIdle);
}

void StateMachine_Run(void) {
    if (currentState) {
        currentState();
    }
}

State StateMachine_GetCurrentState(void) {
    for (int i = 0; i < STATE_COUNT; i++) {
        if (currentState == state_functions[i]) {
            return (State)i;
        }
    }
    return STATE_IDLE;
}

static void StateIdle(void) {
    switch (subState) {
        case SS_0:
            RS485_Set_Receive_Mode();
            MotorSetPWM(0, 0, 0, 0);
            motor_running_flag = 0;
            rpm_current = 0;
            speed_out = 0;
            subState = SS_1;
            break;
            
        case SS_1:
            if (HAL_UART_Receive(&huart3, rx_buffer, PACKET_SIZE, 10) == HAL_OK) {
                ProcessReceivedPacket();
                if (motor_running_flag) {
                    subState = SS_2;
                }
            }
            break;
            
        case SS_2:
            PIDController_Reset(&pid_speed);
            SetState(StateRamping);
            break;
            
        default:
            subState = SS_0;
            break;
    }
}

static void StateRamping(void) {
    switch (subState) {
        case SS_0:
            current_speed = rpm_current;
            last_ramp_update = HAL_GetTick();
            subState = SS_1;
            break;

        case SS_1: {
            uint32_t current_time = HAL_GetTick();
            if (current_time - last_ramp_update >= RAMP_UPDATE_INTERVAL) {
                float time_diff = (float)(current_time - last_ramp_update) / 1000.0f;

                if (current_speed < rpm_setpoint) {
                    current_speed += RAMP_RATE * time_diff;
                    if (current_speed > rpm_setpoint) current_speed = rpm_setpoint;
                } else if (current_speed > rpm_setpoint) {
                    current_speed -= RAMP_RATE * time_diff;
                    if (current_speed < rpm_setpoint) current_speed = rpm_setpoint;
                }

                current_speed = (current_speed > rpm_max) ? rpm_max : 
                               (current_speed < rpm_min) ? rpm_min : current_speed;

                last_ramp_update = current_time;
                speed_out = PIDController_Update(&pid_speed, current_speed, rpm_current);
                ApplySpeedToMotor(speed_out);
            }

            if (fabsf(current_speed - rpm_setpoint) < 1.0f) {
                subState = SS_2;
            }
            break;
        }

        case SS_2:
            SetState(StateSpinning);
            break;
            
        default:
            subState = SS_0;
            break;
    }
}

static void StateSpinning(void) {
    switch (subState) {
        case SS_0:
            motor_running_flag = 1;
            PIDController_Reset(&pid_speed);
            subState = SS_1;
            break;
        
        case SS_1:
            UpdateCurrentSpeed();
            speed_out = PIDController_Update(&pid_speed, rpm_setpoint, rpm_current);
            ApplySpeedToMotor(speed_out);
            
            if (HAL_UART_Receive(&huart3, rx_buffer, PACKET_SIZE, 10) == HAL_OK) {
                ProcessReceivedPacket();
                if (!motor_running_flag) {
                    subState = SS_2;
                }
            }
            
            UpdateTelemetry();
            break;
        
        case SS_2:
            SetState(StateDesaturationRamping);
            break;
            
        default:
            subState = SS_0;
            break;
    }
}

static void StateDesaturationRamping(void) {
    switch (subState) {
        case SS_0:
            current_desat_value = 0.0f;
            desat_setpoint = 1.0f;
            last_desat_update = HAL_GetTick();
            subState = SS_1;
            break;
        
        case SS_1:
            updateDesaturationRampingState();
            speed_out = current_desat_value * rpm_max;
            
            if (calculateDesaturationMetric() > DESAT_THRESHOLD) {
                subState = SS_2;
            }
            break;
        
        case SS_2:
            SetState(StateFaultHandling);
            break;
            
        default:
            subState = SS_0;
            break;
    }
}

static void StateFaultHandling(void) {
    static uint32_t last_recovery_attempt = 0;

    switch (subState) {
        case SS_0:
            enterSafeState();
            fault_active = 1;
            subState = SS_1;
            break;
        
        case SS_1: {
            uint32_t current_time = HAL_GetTick();
            
            if (current_time - last_recovery_attempt >= 5000) {
                if (attemptFaultRecovery()) {
                    fault_active = 0;
                    SetState(StateIdle);
                } else {
                    last_recovery_attempt = current_time;
                }
            }
            break;
        }
        
        default:
            subState = SS_0;
            break;
    }
}

static void UpdateCurrentSpeed(void) {
    rpm_current = CalculateSpeedFromHallSensor();
    if (rpm_current > rpm_max) {
        rpm_current = rpm_max; 
    }
}

static void ApplySpeedToMotor(float speed) {
    float pwm_value = speed / rpm_max;
    pwm_value = (pwm_value > 1.0f) ? 1.0f : (pwm_value < 0.0f) ? 0.0f : pwm_value;
    MotorSetPWM(pwm_value, pwm_value, pwm_value, GetHallSensorState());
}

void HallSensorInterrupt(void) {
    hall_pulse_count++;
}

static float CalculateSpeedFromHallSensor(void) {
    float current_time = GetCurrentTime(); 
    float time_interval = current_time - last_time;
    
    if (time_interval < TIME_INTERVAL) {
        return rpm_current;
    }

    float rpm = (hall_pulse_count * 60.0f) / (HALL_SENSOR_COUNT * time_interval);
    hall_pulse_count = 0;
    last_time = current_time;

    return rpm;
}

static float GetCurrentTime(void) {
    return (float)HAL_GetTick() / 1000.0f;
}

static float calculateDesaturationMetric(void) {
    float voltage = get_tele(VBUS_VOLTAGE);
    float current = get_tele(VBUS_CURRENT);
    return voltage / (current + 0.001f);
}

static void updateDesaturationRampingState(void) {
    uint32_t current_time = HAL_GetTick();
    float time_diff = (float)(current_time - last_desat_update) / 1000.0f;
    
    if (current_time - last_desat_update >= DESAT_UPDATE_INTERVAL) {
        if (current_desat_value < desat_setpoint) {
            current_desat_value += DESAT_RAMP_RATE * time_diff;
            current_desat_value = (current_desat_value > desat_setpoint) ? desat_setpoint : current_desat_value;
        } else if (current_desat_value > desat_setpoint) {
            current_desat_value -= DESAT_RAMP_RATE * time_diff;
            current_desat_value = (current_desat_value < desat_setpoint) ? desat_setpoint : current_desat_value;
        }
        
        last_desat_update = current_time;
    }
}

void handleMotorStates(void) {
    if (motor_running_flag == 1) {
        if (calculateDesaturationMetric() < DESAT_THRESHOLD) {
            SetState(StateDesaturationRamping);
        } else if (fabsf(rpm_setpoint - rpm_current) > 10.0f) {
            SetState(StateRamping);
        }
    } else {
        SetState(StateIdle);
    }
}