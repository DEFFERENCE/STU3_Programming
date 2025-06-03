/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "Encoder.h"
#include "Trajectory.h"
#include "Anti_backlash.h"
//#include "ModBusRTU.h"
#include "Based_System_Communication.h"
#include "Kalman_Filter.h"
#include "Prismatic.h"
#include "Revolute.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Encoder encoder1;
Encoder encoder2;
uint32_t QEIReadRaw3;
uint32_t QEIReadRaw4;
float pos;
float vel;
TrajectorySegment segments[MAX_SEGMENTS];
int current_segment = 0;
float t_global = 0;
float acc;
float p1 = 0;
float v1 = 0;
float a1 = 0;
float p2 = 0;
float v2 = 0;
float a2 = 0;
int check = 0;
int State = 0;
uint16_t adc_1 = 0;
uint16_t adc_2 = 0;

ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];
float Goal_r_position = 999;
float Goal_theta_position = 999;

uint16_t status;
int DIR_18V ;
int DIR_24V;
float Gain_disturbance_pris;
float Gain_disturbance_rev;
float voltage_dis_pris;
float voltage_dis_rev;
float pwm;
float voltage;
int count_Tim2;

// position control prismatic
arm_pid_instance_f32 Pris_posi_PID = {0};
float position_pris = 0;
float setposition_pris = 0;
float V_pris_posi_PID = 0;
float V_absoulte_pris = 0;
float pwm_pris_posi;
float error_posi_pris[2];
float delta_posi_pris;

// velocity control prismatic
arm_pid_instance_f32 Pris_velo_PID = {0};
float velocity_pris = 0;
float setvelocity_pris = 0;
float V_pris_velo_PID = 0;
float pwm_pris_velo;
float error_velo_pris[2];
float delta_velo_pris;

// position control revolute
arm_pid_instance_f32 Rev_posi_PID = {0};
float position_rev = 0;
float setposition_rev = 0;
float V_rev_posi_PID = 0;
float pwm_rev_posi;
float error_posi_rev[2];
float delta_posi_rev;

// velocity control revolute
arm_pid_instance_f32 Rev_velo_PID = {0};
float velocity_rev = 0;
float setvelocity_rev = 0;
float V_rev_velo_PID = 0;
float pwm_rev_velo;
float error_velo_rev[2];
float delta_velo_rev;

KalmanFilter kf_pris;
KalmanFilter kf_rev;
PrismaticMotor Pris_motor;
RevoluteMotor Rev_motor;
float Measurement_Pris[4] = {0};
float Measurement_Rev[4]= {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float Prismatic_position_control(float delta_posi);
float Prismatic_velocity_control(float delta_velo);
float Revolute_position_control(float delta_posi);
float Revolute_velocity_control(float delta_velo);
float voltage_to_pwm(float voltage);
float Prismatic_dis();
float Revolute_dis();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM20_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim20);
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_3);
	Encoder_Init(&encoder1, &htim4);
	Encoder_Init(&encoder2, &htim3);
	HAL_ADC_Start(&hadc1);
	HAL_TIM_Base_Start_IT(&htim2);

	int lastTick = 0;
	int pre_tick = 0;

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	hmodbus.huart = &huart2;
	hmodbus.htim = &htim16;
	hmodbus.slaveAddress = 0x15;
	hmodbus.RegisterSize = 200;
	Modbus_init(&hmodbus, registerFrame);

	Kalman_Init(&kf_pris);

	kf_pris.A_data[0] = 1;
	kf_pris.A_data[1] = 0.0009998;
	kf_pris.A_data[2] = -2.659e-06;
	kf_pris.A_data[3] = 8.108e-08;
	kf_pris.A_data[4] = 0;
	kf_pris.A_data[5] = 0.9996;
	kf_pris.A_data[6] = -0.005318;
	kf_pris.A_data[7] = 0.0001622;
	kf_pris.A_data[8] = 0;
	kf_pris.A_data[9] = 0;
	kf_pris.A_data[10] = 1;
	kf_pris.A_data[11] = 0;
	kf_pris.A_data[12] = 0;
	kf_pris.A_data[13] = -2.746;
	kf_pris.A_data[14] = 0.007303;
	kf_pris.A_data[15] = 0.1354;

	kf_pris.B_data[0] = 1.203e-07;
	kf_pris.B_data[1] = 0.0002406;
	kf_pris.B_data[2] = 0;
	kf_pris.B_data[3] = 1.685;

	// Identity H
	for (int i = 0; i < KALMAN_MEAS_DIM; i++) {
	    for (int j = 0; j < KALMAN_STATE_DIM; j++) {
	    	kf_pris.H_data[i * KALMAN_STATE_DIM + j] = (i == j) ? 1.0f : 0.0f;
	    }
	}

	// Prismatic
	kf_pris.x_data[0] = 0;
	kf_pris.x_data[1] = 0;
	kf_pris.x_data[2] = 0;
	kf_pris.x_data[3] = 0;

	Kalman_SetMeasurementNoise(&kf_pris, 0.01f);
	Kalman_SetProcessNoise(&kf_pris, 0.1f);

	Pris_motor = create_prismatic_motor(2.29e-04, 4.82e-04, 8.75e-01, 1.77e-01, 1.77e-01, 3.8719, 0.0016);
	Rev_motor =  create_motor(1.88E-01,6.91E-03,7.36E-01,1.63E+00,1.63E+00*7.36E-01,5.13E-01,3.37E-04);

	// Prismatic Position
	Pris_posi_PID.Kp = 1;
	Pris_posi_PID.Ki = 0.05;
	Pris_posi_PID.Kd = 0.1;
	arm_pid_init_f32(&Pris_posi_PID, 0);

	// Prismatic Velocity
	Pris_velo_PID.Kp = 1;
	Pris_velo_PID.Ki = 0.1;
	Pris_velo_PID.Kd = 0;
	arm_pid_init_f32(&Pris_velo_PID, 0);

	// Revolute Position
	Rev_posi_PID.Kp = 1;
	Rev_posi_PID.Ki = 0.00001;
	Rev_posi_PID.Kd = 0.1;
	arm_pid_init_f32(&Rev_posi_PID, 0);

	// Revolute Velocity
	Rev_velo_PID.Kp = 1;
	Rev_velo_PID.Ki = 0.00001;
	Rev_velo_PID.Kd = 0;
	arm_pid_init_f32(&Rev_velo_PID, 0);

	InitTrajectorySegment(&segments[0], 0.0f, 500.0f, 500.0f, 250.0f, 0.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		//__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_1,500);
		//__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_3,status);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_Start(&hadc2);
		adc_1 = HAL_ADC_GetValue(&hadc1);
		adc_2 = HAL_ADC_GetValue(&hadc2);
		QEIReadRaw3 = __HAL_TIM_GET_COUNTER(&htim3);
		QEIReadRaw4 = __HAL_TIM_GET_COUNTER(&htim4);
		Modbus_Protocal_Worker();
		//modbus_r_position(&hmodbus,7);
//		hmodbus.RegisterAddress[0x00].U16 = 22881;
		//hmodbus.RegisterAddress[0x10].U16 = 4;
//		modbus_r_position(&hmodbus,5);
//		modbus_theta_position(&hmodbus,5);
//		modbus_r_velocity(&hmodbus,5);
//		modbus_theta_velocity(&hmodbus,5);
//		modbus_r_acceleration(&hmodbus,5);
//		modbus_theta_acceleration(&hmodbus,5);
//		modbus_Update_All(&hmodbus, r_pos, theta_pos, r_Velo, theta_Velo,
//				r_accel, theta_accel);
//		for (int i = 0; i < 10; i++) {
//			set_Target_Position_ten_points(&hmodbus, i, i + 10, i);
//		}
//		Goal_r_position = modbus_set_goal_r_position(&hmodbus);
//		Goal_theta_position = modbus_set_goal_theta_position(&hmodbus);
		//float a = a + 1;
		//hmodbus.RegisterAddress[0x15].U16 = 100;
		//registerFrame[0x15].U16 = 100;
//		for (int i = 0;i<31;i++)
//		{
//		registerFrame[0x00].U16 = 1;
//		registerFrame[0x01].U16 = 2;
//		registerFrame[0x02].U16 = 3;
//		registerFrame[0x03].U16 = 4;
//		registerFrame[0x04].U16 = 5;
//		registerFrame[0x05].U16 = 6;
//		registerFrame[0x06].U16 = 7;
//		registerFrame[0x07].U16 = 8;
//		registerFrame[0x08].U16 = 9;
		//registerFrame[0x15].U16 = 10;
		//}
		uint32_t currentTick = HAL_GetTick();
		float dt = (currentTick - lastTick) / 1000.0f;
//		QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim4);
		if (dt >= 0.01f) {
			Encoder_Update(&encoder1, dt);
			Encoder_Update(&encoder2, dt);
			lastTick = currentTick;

			p1 = Encoder_GetPosition(&encoder1);
			v1 = Encoder_GetVelocity(&encoder1);
			a1 = Encoder_GetAcceleration(&encoder1);

			p2 = Encoder_GetPosition(&encoder2);
			v2 = Encoder_GetVelocity(&encoder2);
			a2 = Encoder_GetAcceleration(&encoder2);

// Now use p1,v1,a1 and p2,v2,a2 as needed
		}
//
		t_global = HAL_GetTick() / 1000.0f;
		pos = GetTrajectoryPosition(&segments[0], t_global);
		vel = GetTrajectoryVelocity(&segments[0], t_global);


		if (V_pris_velo_PID < 0) {
			DIR_24V = 0;
			V_absoulte_pris  = fabsf(V_pris_velo_PID);
		} else if (V_pris_velo_PID > 0) {
			DIR_24V = 1;
			V_absoulte_pris  = V_pris_velo_PID;
		}
		pwm_pris_velo = voltage_to_pwm(V_absoulte_pris );
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, DIR_24V);
		__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_1,pwm_pris_velo);

		Measurement_Pris[0] = Encoder_GetPosition_mm(&encoder1);
		Measurement_Pris[1] = Encoder_GetVelocity_mm(&encoder1);
		Measurement_Pris[2] = 0;
		Measurement_Pris[3] = 0;
		Kalman_SetInput(&kf_pris,V_absoulte_pris);
		Kalman_Predict(&kf_pris);
		Kalman_Update(&kf_pris,Measurement_Pris);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_9) {
		State = 9;
	} else if (GPIO_Pin == GPIO_PIN_10) {
		State = 10;
	} else if (GPIO_Pin == GPIO_PIN_11) {
		State = 11;
	} else if (GPIO_Pin == GPIO_PIN_12) {
		State = 12;
	} else if (GPIO_Pin == GPIO_PIN_13) {
		State = 13;
	} else if (GPIO_Pin == GPIO_PIN_14) {
		State = 14;
	} else if (GPIO_Pin == GPIO_PIN_15) {
		State = 15;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		count_Tim2 += 1;
		// Velocity Control
		velocity_pris = Encoder_GetVelocity_mm(&encoder1);
		setvelocity_pris = GetTrajectoryVelocity(&segments[0], t_global) + V_pris_posi_PID;
		delta_velo_pris = setvelocity_pris - velocity_pris;
		V_pris_velo_PID = Prismatic_velocity_control(delta_velo_pris);
		if (count_Tim2 >= 10) {
			// Position Control
			position_pris = Encoder_GetPosition_mm(&encoder1);
			setposition_pris = GetTrajectoryPosition(&segments[0], t_global);
			delta_posi_pris = setposition_pris - position_pris;
			V_pris_posi_PID = Prismatic_position_control(delta_posi_pris);
			count_Tim2 = 0;
		}
	}
}

float Prismatic_position_control(float delta_posi) {
	int anti_windup;
	error_posi_pris[0] = delta_posi;
//	Pris_posi_PID.Kp = 1;
//	Pris_posi_PID.Kd = 1;

	if (error_posi_pris[0] < 0 && error_posi_pris[1] > 0) {
		anti_windup = 0;
	} else if (error_posi_pris[0] > 0 && error_posi_pris[1] < 0) {
		anti_windup = 0;
	} else {
		anti_windup = 1;
	}

//	if (V_pris_posi_PID >= 24 && anti_windup == 0) {
//		Pris_posi_PID.Ki = 0;
//	} else {
//		Pris_posi_PID.Ki = 1;
//	}

	V_pris_posi_PID = arm_pid_f32(&Pris_posi_PID, delta_posi);

//	if (V_pris_posi_PID > 24) {
//		V_pris_posi_PID = 24;
//	}

	error_posi_pris[1] = error_posi_pris[0];
	return V_pris_posi_PID;
}

float Prismatic_velocity_control(float delta_velo) {
	int anti_windup;
	error_velo_pris[0] =  delta_velo;
//	Pris_velo_PID.Kp = 0.01;

	if (error_velo_pris[0] < 0 && error_velo_pris[1] > 0) {
		anti_windup = 0;
	} else if (error_velo_pris[0] > 0 && error_velo_pris[1] < 0) {
		anti_windup = 0;
	} else {
		anti_windup = 1;
	}

	if (V_pris_velo_PID >= 24 && anti_windup == 0) {
		Pris_velo_PID.Ki = 0;
	} else {
		Pris_velo_PID.Ki = 0.001;
	}

	V_pris_velo_PID = arm_pid_f32(&Pris_velo_PID, delta_velo);

	if (V_pris_velo_PID > 24) {
		V_pris_velo_PID = 24;
	}

	error_velo_pris[1] = error_velo_pris[0];
	return V_pris_velo_PID;
}

float Revolute_position_control(float delta_posi) {
	int anti_windup;
	error_posi_rev[0] = delta_posi;
	Rev_posi_PID.Kp = 1;
	Rev_posi_PID.Kd = 1;

	if (error_posi_rev[0] < 0 && error_posi_rev[1] > 0) {
		anti_windup = 0;
	} else if (error_posi_rev[0] > 0 && error_posi_rev[1] < 0) {
		anti_windup = 0;
	} else {
		anti_windup = 1;
	}

	if (V_rev_posi_PID >= 24 && anti_windup == 0) {
		Rev_posi_PID.Ki = 0;
	} else {
		Rev_posi_PID.Ki = 1;
	}

	V_rev_posi_PID = arm_pid_f32(&Rev_posi_PID, delta_posi);

	if (V_rev_posi_PID > 24) {
		V_rev_posi_PID = 24;
	}

	error_posi_rev[1] = error_posi_rev[0];
	return V_rev_posi_PID;
}

float Revolute_velocity_control(float delta_velo) {
	int anti_windup;
	error_velo_rev[0] =  delta_velo;
	Rev_velo_PID.Kp = 0.01;

	if (error_velo_rev[0] < 0 && error_velo_rev[1] > 0) {
		anti_windup = 0;
	} else if (error_velo_rev[0] > 0 && error_velo_rev[1] < 0) {
		anti_windup = 0;
	} else {
		anti_windup = 1;
	}

	if (V_rev_velo_PID >= 24 && anti_windup == 0) {
		Rev_velo_PID.Ki = 0;
	} else {
		Rev_velo_PID.Ki = 0.001;
	}

	V_rev_velo_PID = arm_pid_f32(&Rev_velo_PID, delta_velo);
	error_velo_rev[1] = error_velo_rev[0];
	return V_rev_velo_PID;
}

float voltage_to_pwm(float voltage) {
	float pwm = (voltage * 65535) / 24;
	return pwm;
}

float Prismatic_dis() {
	float load = 0.01 / (2.0 * (22.0/7.0) * 4.0 * Pris_motor.Kt_Pri);
	voltage_dis_pris = (disturbance_feedforward_pri(&Pris_motor, load)) * (0.3*9.81) * Gain_disturbance_pris;; // อย่าลืมคูณ sin(theta)
	return voltage_dis_pris;
}

float Revolute_dis() {
	float load = 0.01 / (2.0 * (22.0/7.0) * 4.0 * Pris_motor.Kt_Pri);
	voltage_dis_rev = (disturbance_feedforward(&Rev_motor, load)) * (0.3*9.81) * Gain_disturbance_rev;; // อย่าลืมคูณ sin(theta)
	return voltage_dis_rev;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
