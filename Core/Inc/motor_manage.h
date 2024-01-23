/***
@Author: Vincent Chan
@About: Motor Control libraries
***/
#include "main.h"

extern "C" {
	#include "motor.h"
}

/*** Enabled peripherals ***/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

extern motor_group_config_t motor_config;

/*** User defined variables ***/
const int max_register_value = 3600;
enum MOTOR_INDEX{MOTOR1,MOTOR2,MOTOR3,MOTOR4};
enum MOTOR_DIRECTION{CLOCKWISE,COUNTERCLOCKWISE,BREAK};

/*** Function Prototypes ***/
void motor_peripherals_init(MOTOR_INDEX motor);
void motor_controller(MOTOR_INDEX motor,MOTOR_DIRECTION turn,int duty_cycle);
int calculate_MOTOR_timer_register(int percentage,double counter_max);
void motor_selftest(void);

/*** Function Definition ***/
// 1. Enable motor peripherals based on MOTOR_INDEX
void motor_peripherals_init(MOTOR_INDEX motor)
{
	int x = 0;
	if(motor == MOTOR1)
	{
		// MOTOR1 with ENCODER 3
    /* USER CODE BEGIN */
		// Start PWM and Encoder
	  /* USER CODE END */
	}else if(motor == MOTOR2)
	{
		// MOTOR2 with ENCODER 4 [Align with BAT IN in the motor drive]
    /* USER CODE BEGIN */
		// Start PWM and Encoder
	  /* USER CODE END */
	}else if (motor == MOTOR3)
	{
		// TO-DO: Configure motor 3
		x = 0;
	}else{
		// TO-DO: Configure motor 4
		x = 0;
	}
	x++;
}

// 2. Motor Controller
void motor_controller(MOTOR_INDEX motor,MOTOR_DIRECTION turn,int duty_cycle,int max_counter)
{
	int x = 0;
	if(motor == MOTOR1)
	{
		if(turn == CLOCKWISE)
		{
    /* USER CODE BEGIN */
			motor_set_direction(motor_config.left, MOTOR_DIRECTION_FORWARD);
		// Set the Motor CW
	  /* USER CODE END */
		}else if (turn == COUNTERCLOCKWISE)
		{
    /* USER CODE BEGIN */
		// Set the Motor CCW
			motor_set_direction(motor_config.left, MOTOR_DIRECTION_BACKWARD);
	  /* USER CODE END */
		}else
		{
    /* USER CODE BEGIN */
		// Stop the Motor
			motor_set_direction(motor_config.left, MOTOR_DIRECTION_STOP);
	  /* USER CODE END */
		}
		/* USER CODE BEGIN */
	  // Use __HAL_TIM_SET_COMPARE() & calculate_MOTOR_timer_register functions to set the PWM
		motor_pwm(motor_config.left, duty_cycle * max_counter / 100);
		/* USER CODE END */
	}else if(motor == MOTOR2)
	{
		if(turn == CLOCKWISE)
		{
    /* USER CODE BEGIN */
		// Set the Motor CW
			motor_set_direction(motor_config.right, MOTOR_DIRECTION_FORWARD);
	  /* USER CODE END */
		}else if (turn == COUNTERCLOCKWISE)
		{
    /* USER CODE BEGIN */
		// Set the Motor CCW
			motor_set_direction(motor_config.right, MOTOR_DIRECTION_BACKWARD);
	  /* USER CODE END */
		}else
		{
    /* USER CODE BEGIN */
		// Stop the Motor
			motor_set_direction(motor_config.right, MOTOR_DIRECTION_STOP);
	  /* USER CODE END */
		}
		/* USER CODE BEGIN */
	  // Use __HAL_TIM_SET_COMPARE() & calculate_MOTOR_timer_register functions to set the PWN
		motor_pwm(motor_config.right, duty_cycle * max_counter / 100);
		/* USER CODE END */
	}else if (motor == MOTOR3)
	{
		// TO-DO: Configure motor 3
		x = 0;
	}else{
		// TO-DO: Configure motor 4
		x = 0;
	}
	x++;
}	

// 3. Convert duty cycle to counter register value
int calculate_MOTOR_timer_register(int percentage,double counter_max)
{
	return (int)((percentage*counter_max)/100);
}

// 4. Motor Testing
void motor_selftest(void)
{
	// No debouncing because it is just for testing
	// A.When SW1 is pressed, the motors should turn in clockwise
	// B.When SW2 is pressed, the motors should turn in counterclockwise
	// C.When SW3 is pressed, the motors should stop
	if(HAL_GPIO_ReadPin(USER_SWITCH_1_GPIO_Port,USER_SWITCH_1_Pin) == GPIO_PIN_RESET)
	{
		motor_controller(MOTOR1,CLOCKWISE,90,max_register_value);
		motor_controller(MOTOR2,CLOCKWISE,90,max_register_value);
	}else if(HAL_GPIO_ReadPin(USER_SWITCH_2_GPIO_Port,USER_SWITCH_2_Pin) == GPIO_PIN_RESET)
	{
		motor_controller(MOTOR1,COUNTERCLOCKWISE,90,max_register_value);
		motor_controller(MOTOR2,COUNTERCLOCKWISE,90,max_register_value);		
	}else if(HAL_GPIO_ReadPin(USER_SWITCH_3_GPIO_Port,USER_SWITCH_3_Pin) == GPIO_PIN_RESET)
	{
		motor_controller(MOTOR1,BREAK	,0,max_register_value);
		motor_controller(MOTOR2,BREAK,0,max_register_value);			
	}else{
		int p = 0;
		p ++;
	}	
}