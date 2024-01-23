
#include "main.h"
#include "motor.h"

void motor_pwm(motor_config_t *motor, int32_t value)
{
	__HAL_TIM_SetCompare(motor->htim_pwm, motor->channel, value);
}

void motor_forward(motor_config_t *motor)
{
	HAL_GPIO_WritePin(motor->in1port, motor->in1pin, motor->reversed ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->in2port, motor->in2pin, motor->reversed ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_backward(motor_config_t *motor)
{
	HAL_GPIO_WritePin(motor->in1port, motor->in1pin, motor->reversed ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->in2port, motor->in2pin, motor->reversed ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void motor_stop(motor_config_t *motor)
{
	HAL_GPIO_WritePin(motor->in1port, motor->in1pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->in2port, motor->in2pin, GPIO_PIN_RESET);
}

uint32_t motor_get_counter(motor_config_t *motor)
{
	return __HAL_TIM_GET_COUNTER(motor->htim_encoder);
}

void motor_set_direction(motor_config_t *motor, motor_direction_t direction)
{
	switch(direction)
	{
		case MOTOR_DIRECTION_STOP:
			motor_stop(motor);
			break;
		case MOTOR_DIRECTION_FORWARD:
			motor_forward(motor);
			break;
		case MOTOR_DIRECTION_BACKWARD:
			motor_backward(motor);
			break;
		default:
			motor_stop(motor);
			// ERROR state
			break;
	}
}
