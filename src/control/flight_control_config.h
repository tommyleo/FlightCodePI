#ifndef FLIGHTCODEPI_FLIGHT_CONTROL_CONFIG_H
#define FLIGHTCODEPI_FLIGHT_CONTROL_CONFIG_H

#define FLIGHT_PI_F 3.14159265358979323846f

#define PID_INTEGRAL_LIMIT_PERCENT 30.0f
#define PID_ROLL_PITCH_OUTPUT_LIMIT_PERCENT 35.0f
#define PID_YAW_OUTPUT_LIMIT_PERCENT 25.0f

#define GYRO_CALIBRATION_SAMPLES 8000u

// Orientamento previsto: IMU piatta, asse X rivolto verso il muso.
// Cambiare segno se un asse reagisce nella direzione opposta.
#define GYRO_ROLL_SIGN 1.0f
#define GYRO_PITCH_SIGN 1.0f
#define GYRO_YAW_SIGN 1.0f

// Mixer Quad X, ordine Betaflight:
// M1 posteriore destro, M2 anteriore destro,
// M3 posteriore sinistro, M4 anteriore sinistro.
#define MIXER_MOTOR_COUNT 4u

#endif
