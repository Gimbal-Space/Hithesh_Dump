void IdleState(void) {
    static uint8_t substate = 0;
    
    switch (substate) {
        case 0: // Initialize idle state
         RS485_Set_Receive_Mode();
            MotorSetPWM(0, 0, 0, 0);
            motor_running_flag = 0;
            rpm_current = 0;
            speed_out = 0;
            substate = 1;
            break;
            
        case 1: // Main idle loop
            if (HAL_UART_Receive(&huart3, rx_buffer, PACKET_SIZE, 10) == HAL_OK) {
                ProcessReceivedPacket();
                if (motor_running_flag) {
                    substate = 2;
                }
            }
            UpdateTelemetry();
            break;
            
        case 2: // Transition out of idle
            PIDController_Reset(&pid_speed);
            SetState(spinning);
            substate = 0;
            break;
    }
}

// Update system telemetry
void UpdateTelemetry(void) {
    float vbus = get_tele(VBUS_VOLTAGE);
    float ibus = get_tele(VBUS_CURRENT);
    // Add code to store or transmit telemetry data
}