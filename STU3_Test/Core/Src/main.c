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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "Encoder.h"
#include "Trajectory.h"
//#include "ModBusRTU.h"
#include "Based_System_Communication.h"
#include "Kalman_Filter.h"
#include "Prismatic.h"
#include "Revolute.h"
#include "Joystick.h"
#include "FIBO_path.h"
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

//Trajectory
float pos_pris;
float vel_pris;
float pos_rev;
float vel_rev;
int current_segment = 0;
TrajectorySegment Prismatic[10];
#define v_max_pris 500.0f
#define a_max_pris 250.0f
TrajectorySegment Revolute[10];
#define v_max_rev 0.3f
#define a_max_rev 0.05f
float t_global = 0;
float start_pris;
float end_pris;
float start_rev;
float end_rev;
TrajectorySegment currentPrismatic;
TrajectorySegment currentRevolute;
int current_index = 0;
float next_start_time = 0.0f;

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
float gain_disturbance_rev;
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
float V_absolute_rev;
float V_plant;

KalmanFilter kf_pris;
KalmanFilter kf_rev;
PrismaticMotor Pris_motor;
RevoluteMotor Rev_motor;
float Measurement_Pris[4] = {0};
float Measurement_Rev[4]= {0};

float load;
float sine;
float encoder;

int Circle;
int Square;
int Triangle;
int Cross;
int R1;
int R2;
int Select;
int Start;
int L2;
//float PrismaticTenPoints[11] = {0.0f, 0.52f, -0.9f, 0.9f, 0.0f, 0.9f, -0.78f, 0.0f, 0.9f, 0.78f, 0.0f};
//float PrismaticTenpoints_actual[11] = {0.0f, 0.62f, -0.8f, 1.0f, -0.1f, 1.0f, -0.68f, 0.1f, 1.0f, 0.88f, -0.1f};
//float RevoluteTenPoints[11] = {0.0f, 0.62f, -0.8f, 1.0f, -0.1f, 1.0f, -0.68f, 0.1f, 1.0f, 0.88f, -0.1f};
int count = 0;
int test = 0;
int start_trajectory;
float delay_pris[10];
float delay_rev[10];
int state_joy;
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
void updateTrajectoryIfNeeded(float t_global);
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
  MX_SPI1_Init();
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

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	hmodbus.huart = &huart2;
	hmodbus.htim = &htim16;
	hmodbus.slaveAddress = 0x15;
	hmodbus.RegisterSize = 200;
	Modbus_init(&hmodbus, registerFrame);

	Kalman_Init(&kf_pris);

	kf_pris.A_data[0] = 1;
	kf_pris.A_data[1] = 0.0008395;
	kf_pris.A_data[2] = -4.198e-07;
	kf_pris.A_data[3] = 1.282e-05;
	kf_pris.A_data[4] = 0;
	kf_pris.A_data[5] = 0.6791;
	kf_pris.A_data[6] = -0.0008395;
	kf_pris.A_data[7] = 0.02564;
	kf_pris.A_data[8] = 0;
	kf_pris.A_data[9] = 0;
	kf_pris.A_data[10] = 1;
	kf_pris.A_data[11] = 0;
	kf_pris.A_data[12] = 0;
	kf_pris.A_data[13] = -0.04203;
	kf_pris.A_data[14] = 2.101e-05;
	kf_pris.A_data[15] = -0.09565;

	kf_pris.B_data[0] = 4.006e-06;
	kf_pris.B_data[1] = 0.008011;
	kf_pris.B_data[2] = 0;
	kf_pris.B_data[3] = 0.2826;

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
	Kalman_SetProcessNoise(&kf_pris, 0.9f);

	Kalman_Init(&kf_rev);

	kf_rev.A_data[0] = 1;
	kf_rev.A_data[1] = 0.0009998;
	kf_rev.A_data[2] = -2.659e-06;
	kf_rev.A_data[3] = 8.108e-08;
	kf_rev.A_data[4] = 0;
	kf_rev.A_data[5] = 0.9996;
	kf_rev.A_data[6] = -0.005318;
	kf_rev.A_data[7] = 0.0001622;
	kf_rev.A_data[8] = 0;
	kf_rev.A_data[9] = 0;
	kf_rev.A_data[10] = 1;
	kf_rev.A_data[11] = 0;
	kf_rev.A_data[12] = 0;
	kf_rev.A_data[13] = -2.746;
	kf_rev.A_data[14] = 0.007303;
	kf_rev.A_data[15] = 0.1354;

	kf_rev.B_data[0] = 1.203e-07;
	kf_rev.B_data[1] = 0.0002406;
	kf_rev.B_data[2] = 0;
	kf_rev.B_data[3] = 1.685;

	// Identity H
	for (int i = 0; i < 2; i++) {
	    for (int j = 0; j < 4; j++) {
	        if (i == j) {
	            kf_rev.H_data[i * 4 + j] = 1.0f;
	        } else {
	            kf_rev.H_data[i * 4 + j] = 0.0f;
	        }
	    }
	}

	// Revolute
	kf_rev.x_data[0] = 0;
	kf_rev.x_data[1] = 0;
	kf_rev.x_data[2] = 0;
	kf_rev.x_data[3] = 0;

	Kalman_SetMeasurementNoise(&kf_rev, 0.08f);
	Kalman_SetProcessNoise(&kf_rev, 0.12f);

	Pris_motor = create_prismatic_motor(2.29e-04, 4.82e-04, 8.75e-01, 1.77e-01, 1.77e-01, 3.8719, 0.0016);
	Rev_motor = create_motor(1.88E-01, 6.91E-03, 7.36E-01, 1.63E+00, 1.63E+00 * 7.36E-01, 5.13E-01, 3.37E-04);

	// Prismatic Position
	Pris_posi_PID.Kp = 0.3;
	Pris_posi_PID.Ki = 0.01;
	Pris_posi_PID.Kd = 0.3;
	arm_pid_init_f32(&Pris_posi_PID, 0);

	// Prismatic Velocity
	Pris_velo_PID.Kp = 0.08;
	Pris_velo_PID.Ki = 0.01;
	Pris_velo_PID.Kd = 0;
	arm_pid_init_f32(&Pris_velo_PID, 0);

	// Revolute Position
	Rev_posi_PID.Kp = 2.5;
	Rev_posi_PID.Ki = 0.2;
	Rev_posi_PID.Kd = 1.0;
	arm_pid_init_f32(&Rev_posi_PID, 0);

	// Revolute Velocity
	Rev_velo_PID.Kp = 2.0;
	Rev_velo_PID.Ki = 0.1;
	Rev_velo_PID.Kd = 0;
	arm_pid_init_f32(&Rev_velo_PID, 0);

//	InitTrajectorySegment(&segments[0], 0.0f, 200.0f, 500.0f, 250.0f, 0.0f);
//	InitTrajectorySegment(&segments[0], 0.0f,  0.785f, 1.0f, 0.4f, 0.0f);
//	InitTrajectorySegment(&segments[1], 100.0f, 50.0f, 40.0f, 80.0f, segments[0].t_start + segments[0].t_total);
//	InitTrajectorySegment(&segments[2], 50.0f, 200.0f, 60.0f, 120.0f, segments[1].t_start + segments[1].t_total);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_ADC_Start(&hadc1);
		HAL_ADC_Start(&hadc2);
		adc_1 = HAL_ADC_GetValue(&hadc1);
		adc_2 = HAL_ADC_GetValue(&hadc2);
		QEIReadRaw3 = __HAL_TIM_GET_COUNTER(&htim3);
		QEIReadRaw4 = __HAL_TIM_GET_COUNTER(&htim4);

		Modbus_Protocal_Worker();

		uint32_t currentTick = HAL_GetTick();
		float dt = (currentTick - lastTick) / 1000.0f;
		if (dt >= 0.001f) {
			Encoder_Update(&encoder1, dt);
			Encoder_Update(&encoder2, dt);
			lastTick = currentTick;

			p1 = Encoder_GetPosition_mm(&encoder1);
			v1 = Encoder_GetVelocity(&encoder1);
			a1 = Encoder_GetAcceleration(&encoder1);

			p2 = Encoder_GetPosition(&encoder2);
			v2 = Encoder_GetVelocity(&encoder2);
			a2 = Encoder_GetAcceleration(&encoder2);

			Measurement_Pris[0] = Encoder_GetPosition_mm(&encoder1);
			Measurement_Pris[1] = Encoder_GetVelocity_mm(&encoder1);
			Measurement_Pris[2] = 0;
			Measurement_Pris[3] = 0;
			Kalman_SetInput(&kf_pris, V_pris_velo_PID);
			Kalman_Predict(&kf_pris);
			Kalman_Update(&kf_pris, Measurement_Pris);

			Measurement_Rev[0] = Encoder_GetPosition(&encoder2) / (100/30);
			Measurement_Rev[1] = Encoder_GetVelocity(&encoder2) / (100/30);
			Measurement_Rev[2] = 0;
			Measurement_Rev[3] = 0;
			Kalman_SetInput(&kf_rev, V_rev_velo_PID);
			Kalman_Predict(&kf_rev);
			Kalman_Update(&kf_rev, Measurement_Rev);

			Revolute_dis();
			count_Tim2 += 1;
			// Velocity Control Prismatic
			velocity_pris = Encoder_GetVelocity_mm(&encoder1);
//			setvelocity_pris = GetTrajectoryVelocity(&Prismatic[current_segment], t_global) + V_pris_posi_PID;
			setvelocity_pris = vel_pris + V_pris_posi_PID;
			delta_velo_pris = setvelocity_pris - velocity_pris;
//			delta_velo_pris = setvelocity_pris - kf_pris.x_data[1];
			V_pris_velo_PID = Prismatic_velocity_control(delta_velo_pris);

			// Velocity Control revolute
			velocity_rev = Encoder_GetVelocity(&encoder2) / (100.0 / 30.0);
//			setvelocity_rev = GetTrajectoryVelocity(&Revolute[current_segment], t_global) + V_rev_posi_PID;
			setvelocity_rev = vel_rev + V_rev_posi_PID;
//			delta_velo_rev = setvelocity_rev - velocity_rev;
			delta_velo_rev = setvelocity_rev - kf_rev.x_data[1];
			V_rev_velo_PID = Revolute_velocity_control(delta_velo_rev);
			if (count_Tim2 >= 10) {
				// Position Control Prismatic
				position_pris = Encoder_GetPosition_mm(&encoder1);
//				setposition_pris = GetTrajectoryPosition(&Prismatic[current_segment], t_global);
				setposition_pris = pos_pris;
				delta_posi_pris = setposition_pris - position_pris;
				if (delta_posi_pris <= 0.1 && delta_posi_pris >= -0.1) {
					V_pris_posi_PID = 0;
					V_pris_velo_PID = 0;
				} else {
					V_pris_posi_PID = Prismatic_position_control(delta_posi_pris);
				}
//				V_pris_posi_PID = Prismatic_position_control(delta_posi_pris);

				// Position Control Revolute
				position_rev = Encoder_GetPosition(&encoder2) / (100.0 / 30.0);
//				setposition_rev = GetTrajectoryPosition(&Revolute[current_segment], t_global) + Rev_backlash.backlash_offset;
				setposition_rev = pos_rev;
//				Backlash_Update(&Rev_backlash, pos_rev, p2, v2);
				delta_posi_rev = setposition_rev - position_rev;
				if (delta_posi_rev <= 0.1 && delta_posi_rev >= -0.1) {
					V_rev_posi_PID = 0;
					V_rev_velo_PID = 0;
				} else {
					V_rev_posi_PID = Revolute_position_control(delta_posi_rev);
				}

				count_Tim2 = 0;
			}
		}

		t_global = HAL_GetTick() / 1000.0f;
		updateTrajectoryIfNeeded(t_global);

		pos_pris = GetTrajectoryPosition(&currentPrismatic, t_global);
		vel_pris = GetTrajectoryVelocity(&currentPrismatic, t_global);
		pos_rev = GetTrajectoryPosition(&currentRevolute, t_global);
		vel_rev = GetTrajectoryVelocity(&currentRevolute, t_global);

		if (current_index >= 2064 - 1) {
		    pos_pris = currentPrismatic.end_pos;
		    vel_pris = 0;
		    pos_rev = currentRevolute.end_pos;
		    vel_rev = 0;
		}

		if (V_pris_velo_PID < 0) {
			DIR_24V = 0;
			V_absoulte_pris = fabsf(V_pris_velo_PID);
		} else if (V_pris_velo_PID > 0) {
			DIR_24V = 1;
			V_absoulte_pris = V_pris_velo_PID;
		}
		pwm_pris_velo = voltage_to_pwm(V_absoulte_pris);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, DIR_24V);
		__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_1,pwm_pris_velo);

		if (V_rev_velo_PID < 0) {
			DIR_18V = 0;
			V_absolute_rev = fabsf(V_rev_velo_PID);
		} else if (V_rev_velo_PID > 0) {
			DIR_18V = 1;
			V_absolute_rev = V_rev_velo_PID;
		}
		V_plant = V_absolute_rev + voltage_dis_rev;
		if (V_plant > 18) {
			V_plant = 18;
		}
		pwm_rev_velo = (V_plant / 18) * 65535;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, DIR_18V);
		__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_3,pwm_rev_velo);
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
//	if (htim == &htim2) {
//		PS2_ReadData();
//	}
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
	} else if (V_pris_velo_PID < -24) {
		V_pris_velo_PID = -24;
	}

	error_velo_pris[1] = error_velo_pris[0];
	return V_pris_velo_PID;
}

float Revolute_position_control(float delta_posi) {
	int anti_windup;
	error_posi_rev[0] = delta_posi;
//	Rev_posi_PID.Kp = 1;
//	Rev_posi_PID.Kd = 1;

	if (error_posi_rev[0] < 0 && error_posi_rev[1] > 0) {
		anti_windup = 0;
	} else if (error_posi_rev[0] > 0 && error_posi_rev[1] < 0) {
		anti_windup = 0;
	} else {
		anti_windup = 1;
	}

//	if (V_rev_posi_PID >= 24 && anti_windup == 0) {
//		Rev_posi_PID.Ki = 0;
//	} else {
//		Rev_posi_PID.Ki = 1;
//	}

	V_rev_posi_PID = arm_pid_f32(&Rev_posi_PID, delta_posi);

//	if (V_rev_posi_PID > 24) {
//		V_rev_posi_PID = 24;
//	}

	error_posi_rev[1] = error_posi_rev[0];
	return V_rev_posi_PID;
}

float Revolute_velocity_control(float delta_velo) {
	int anti_windup;
	error_velo_rev[0] =  delta_velo;
//	Rev_velo_PID.Kp = 0.01;

	if (error_velo_rev[0] < 0 && error_velo_rev[1] > 0) {
		anti_windup = 0;
	} else if (error_velo_rev[0] > 0 && error_velo_rev[1] < 0) {
		anti_windup = 0;
	} else {
		anti_windup = 1;
	}

	if (V_rev_velo_PID >= 18 && anti_windup == 0) {
		Rev_velo_PID.Ki = 0;
	} else {
		Rev_velo_PID.Ki = 0.001;
	}

	if (V_rev_velo_PID > 18) {
		V_rev_velo_PID = 18;
	}

	V_rev_velo_PID = arm_pid_f32(&Rev_velo_PID, delta_velo);
	error_velo_rev[1] = error_velo_rev[0];
	return V_rev_velo_PID;
}

float voltage_to_pwm(float voltage) {
	float pwm = (voltage * 65535) / 24;
	return pwm;
}

//float Prismatic_dis() {
//	float load = 0.01 / (2.0 * (22.0/7.0) * 4.0 * motor.Kt_Pri);
//	voltage_dis = (disturbance_feedforward_pri(&motor, load)) * (0.3*9.81) * gain_disturbance; // อย่าลืมคูณ sin(theta)
//	return voltage_dis;
//}

float Revolute_dis() {
	load = (8.2 * 9.81 * 0.45 * sinf(Encoder_GetPosition(&encoder2) / (100.0/30.0))) +
			(0.3 * 9.81 * sinf(Encoder_GetPosition(&encoder2) / (100.0/30.0)) * 0.4);
	sine = sinf(Encoder_GetPosition(&encoder2) / (100.0/30.0));
	encoder = Encoder_GetPosition(&encoder2) / (100.0/30.0);
//	load = (8.2 * 9.81 * 0.45 * cosf(1.57)) + (0.3 * 9.81 * cosf(1.57) * 0.4);
//	voltage_dis_rev = (disturbance_feedforward(&Rev_motor, load)) * gain_disturbance_rev;
	voltage_dis_rev = (Rev_motor.R_Rev / Rev_motor.Ke_Rev) * kf_rev.x_data[2] * 1.0 / 3.3;
	return voltage_dis_rev;
}

void updateTrajectoryIfNeeded(float t_global) {
    if (current_index >= 460 - 1) return;

    if (t_global >= next_start_time) {
        float start_pris = smooth_path_polar[current_index][1];
        float end_pris = smooth_path_polar[current_index + 1][1];

        float start_rev = smooth_path_polar[current_index][0];
        float end_rev = smooth_path_polar[current_index + 1][0];

        InitTrajectorySegment(&currentPrismatic, start_pris, end_pris, v_max_pris, a_max_pris, t_global);
        InitTrajectorySegment(&currentRevolute,  start_rev,  end_rev,  v_max_rev,  a_max_rev,  t_global);
        next_start_time = t_global + fmaxf(currentPrismatic.t_total, currentRevolute.t_total);

        current_index++;
    }
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
