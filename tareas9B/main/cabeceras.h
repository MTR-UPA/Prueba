#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h" //PWM
#include "esp_err.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_2
#define ENCODER_GPIO_A GPIO_NUM_4
#define ENCODER_GPIO_B GPIO_NUM_16
#define PULSOS_POR_VUELTA 270
#define GRADOS_POR_VUELTA 360.0f
#define PULSOS_POR_GRADO (PULSOS_POR_VUELTA / GRADOS_POR_VUELTA)

// TB6612 (canal A)
#define TB6612_AIN1 GPIO_NUM_18
#define TB6612_AIN2 GPIO_NUM_19
#define TB6612_PWMA GPIO_NUM_5 //0-255
#define TB6612_STBY GPIO_NUM_17

#define PWM_FREQ_HZ 20000
#define PWM_RES LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY ((1 << 10) - 1)