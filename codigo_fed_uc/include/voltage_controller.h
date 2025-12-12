#ifndef PID_H
#define PID_H

#define VOLTAGE_GAIN_SCALE (1000U)

#define VOLTAGE_E_GAIN_1 ((int32_t)(10000))
#define VOLTAGE_E_GAIN_2 ((int32_t)(0))
#define VOLTAGE_U_GAIN ((int32_t)(945))

#define REFERENCE_GAIN ((int32_t)(21269))

#define PID_COEF_SCALE (1000U)

#define PID_B0 ((int32_t)10000) /* example: b0 * PID_COEF_SCALE */
#define PID_B1 ((int32_t)0)     /* example: b1 * PID_COEF_SCALE */
#define PID_A1 ((int32_t)945)   /* example: a1 * PID_COEF_SCALE */

#define PWM_TOP ((uint16_t)199U)

#endif // PID_H
