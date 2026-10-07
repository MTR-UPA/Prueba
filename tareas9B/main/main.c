#include "cabeceras.h"

static const char *TAG_ENC1 = "ENC1";
static const char *TAG_ENC2 = "ENC2";

static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_chan_a = NULL;
static pcnt_channel_handle_t pcnt_chan_b = NULL;

static volatile float posicion_objetivo_grados = 90.0f; // 1 vuelta

static void moverMotor(int pwm_signed)
{
	if (pwm_signed > PWM_MAX_DUTY) pwm_signed = PWM_MAX_DUTY;
	if (pwm_signed < -PWM_MAX_DUTY) pwm_signed = -PWM_MAX_DUTY;

	if (pwm_signed > 0) {
		gpio_set_level(TB6612_AIN1, 1);
		gpio_set_level(TB6612_AIN2, 0);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_signed);
	} else if (pwm_signed < 0) {
		gpio_set_level(TB6612_AIN1, 0);
		gpio_set_level(TB6612_AIN2, 1);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, -pwm_signed);
	} else {
		gpio_set_level(TB6612_AIN1, 0);
		gpio_set_level(TB6612_AIN2, 0);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
	}
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void configurar_tb6612(void)
{
	gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << TB6612_AIN1) | (1ULL << TB6612_AIN2) | (1ULL << TB6612_STBY),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&io_conf));

	gpio_set_level(TB6612_STBY, 1);
	gpio_set_level(TB6612_AIN1, 0);
	gpio_set_level(TB6612_AIN2, 0);

	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.duty_resolution = PWM_RES,
		.freq_hz = PWM_FREQ_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	ledc_channel_config_t ledc_channel = {
		.gpio_num = TB6612_PWMA,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void configurar_pcnt_encoder(void){
	pcnt_unit_config_t unit_config = {
		.low_limit = -32768,
		.high_limit = 32767,
	};
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

	pcnt_chan_config_t chan_a_config = {
		.edge_gpio_num = ENCODER_GPIO_A,
		.level_gpio_num = ENCODER_GPIO_B,
	};
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
		pcnt_chan_a,
		PCNT_CHANNEL_EDGE_ACTION_INCREASE,
		PCNT_CHANNEL_EDGE_ACTION_DECREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(
		pcnt_chan_a,
		PCNT_CHANNEL_LEVEL_ACTION_KEEP,
		PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

	pcnt_chan_config_t chan_b_config = {
		.edge_gpio_num = ENCODER_GPIO_B,
		.level_gpio_num = ENCODER_GPIO_A,
	};
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
		pcnt_chan_b,
		PCNT_CHANNEL_EDGE_ACTION_DECREASE,
		PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(
		pcnt_chan_b,
		PCNT_CHANNEL_LEVEL_ACTION_KEEP,
		PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

	ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}




static void tarea_lectura_encoder(void *pvParameters)
{
	int count = 0;

	while (1) {
		ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &count));
		float posicion_grados = (float)count / PULSOS_POR_GRADO;
		ESP_LOGI(TAG_ENC1, "Posicion encoder: %.2f° (%d pulsos)", posicion_grados, count);
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

static void tarea_control_posicion(void *pvParameters)
{
	const float Kp = 3.0f;
	const float Ki = 0.15f;
	const float Kd = 0.08f;
	const float banda_muerta = 2.0f; // grados
	const float dt_seg = 0.02f;
	const float integral_max = (float)PWM_MAX_DUTY / Ki;
	int count = 0;
	float integral = 0.0f;
	float error_anterior = 0.0f;

	while (1) {
		ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &count));
		float posicion_grados = (float)count / PULSOS_POR_GRADO;
		float error = posicion_objetivo_grados - posicion_grados;

		if (error < banda_muerta && error > -banda_muerta) {
			integral = 0.0f;
			error_anterior = 0.0f;
			error = 0.0f;
		} else {
			integral += error * dt_seg;
			if (integral > integral_max) integral = integral_max;
			if (integral < -integral_max) integral = -integral_max;
		}

		float derivada = (error - error_anterior) / dt_seg;
		float u = Kp * error + Ki * integral + Kd * derivada;
		float u_saturada = u;

		if (u_saturada > PWM_MAX_DUTY) u_saturada = (float)PWM_MAX_DUTY;
		if (u_saturada < -PWM_MAX_DUTY) u_saturada = -(float)PWM_MAX_DUTY;

		if ((u != u_saturada) && (error * u > 0.0f)) {
			integral -= error * dt_seg;
		}

		moverMotor((int)u_saturada);
		error_anterior = error;

		ESP_LOGI(TAG_ENC2, "Ref: %.1f°, Pos: %.2f° (%d pulsos), Err: %.2f°, Int: %.2f, Der: %.2f, PWM: %.1f",
			 posicion_objetivo_grados, posicion_grados, count, error, integral, derivada, u_saturada);
		vTaskDelay(pdMS_TO_TICKS((uint32_t)(dt_seg * 1000.0f)));
	}
}



void app_main(void)
{
	configurar_pcnt_encoder();
	configurar_tb6612();
	xTaskCreate(tarea_lectura_encoder, "tarea_lectura_encoder", 3072, NULL, 1, NULL);
	//xTaskCreate(tarea_control_posicion, "tarea_control_posicion", 3072, NULL, 2, NULL);
}
