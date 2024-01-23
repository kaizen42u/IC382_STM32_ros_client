
#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"
#include <stdbool.h>

typedef struct
{
	GPIO_TypeDef *in1port;
	GPIO_TypeDef *in2port;
	GPIO_TypeDef *enport;
	uint16_t in1pin;
	uint16_t in2pin;
	uint16_t enpin;
	
	TIM_HandleTypeDef *htim_encoder;
	
	TIM_HandleTypeDef *htim_pwm;
	uint32_t channel;
	bool reversed;
	
} motor_config_t;

typedef struct
{
	motor_config_t *left;
	motor_config_t *right;
} motor_group_config_t;

typedef enum
{
	MOTOR_DIRECTION_STOP,
	MOTOR_DIRECTION_TURN_LEFT,
	MOTOR_DIRECTION_TURN_RIGHT,
	MOTOR_DIRECTION_FORWARD,
	MOTOR_DIRECTION_BACKWARD,
	MOTOR_DIRECTION_ERROR,
} motor_direction_t;

uint32_t motor_get_counter(motor_config_t *motor);
void motor_pwm(motor_config_t *motor, int32_t value);
void motor_set_direction(motor_config_t *motor, motor_direction_t direction);

#endif // MOTOR_H_
