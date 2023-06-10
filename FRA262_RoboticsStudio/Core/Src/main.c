/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ModBusRTU.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define endEffector_ADDR 0x15
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// > BASE SYSTEM -------------------------------------------------------------------------------------------------------
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[70];
// < BASE SYSTEM -------------------------------------------------------------------------------------------------------

// > Joy Stick -------------------------------------------------------------------------------------------------------
struct PortPin {
	GPIO_TypeDef *PORT;
	uint16_t PIN;
};

struct PortPin joyPin[4] = { { GPIOC, GPIO_PIN_2 }, // Button M
		{ GPIOA, GPIO_PIN_4 }, // Button R / P
		{ GPIOB, GPIO_PIN_0 }, // Button A / O
		{ GPIOB, GPIO_PIN_7 }, // Button L / D
		};

struct _GPIOState {
	GPIO_PinState Current;
	GPIO_PinState Last;
};

struct _GPIOState Button1[4];
typedef union {
	struct {
		uint16_t xAxis;
		uint16_t yAxis;
	} subdata;
} DMA_ADC_BufferType;
DMA_ADC_BufferType buffer[10];

float refXPos = 2000;
float refYPos = 2000;

uint8_t countBottomB = 0;
uint8_t countTopB = 0;
int8_t countRightB = 0;
uint8_t countLeftB = 0;

uint8_t switchAxis = 1;
uint8_t joyLogic = 0;
uint8_t joyLogicLED = 1;
// < Joy Stick -------------------------------------------------------------------------------------------------------

// > PILOT LAMP ------------------------------------------------------------------------------------------------------
struct PortPin pilotLampPin[3] = { { GPIOA, GPIO_PIN_10 },
		{ GPIOA, GPIO_PIN_11 }, { GPIOA, GPIO_PIN_12 }, };
// < PILOT LAMP ------------------------------------------------------------------------------------------------------

// > MOTOR -----------------------------------------------------------------------------------------------------------
float duty = 0;
uint8_t dirAxisY = 1;
uint8_t dirAxisX = 1;

// < MOTOR -----------------------------------------------------------------------------------------------------------

// > ENCODER ---------------------------------------------------------------------------------------------------------
int32_t QEIReadRaw;
int32_t QEIReadModified;
int32_t QEIHome;

// < ENCODER ---------------------------------------------------------------------------------------------------------

// > PID -------------------------------------------------------------------------------------------------------------
struct pidVariables {
	float pTerm;
	float iTerm;
	float dTerm;
	float eIntegral;
};

struct pidVariables positionPID = { 168, 1.84, 0, 0 }; // 60 / 80  ::: 220 / 122  // 10 / 25 ::: 168 / 1.84
struct pidVariables velocityPID = { 0, 0, 0, 0 };

float mmActPos = 0;
float mmTargetPos = 0;
float mmError = 0;
float mmActVel = 0;
float mmActAcc = 0;
float prePos = 0;
float preVel = 0;
float pidVel = 0;

float testVar = 0;
// < PID -------------------------------------------------------------------------------------------------------------

// > Calibrate -------------------------------------------------------------------------------------------------------
uint8_t calibrateTrayInput = 0;
// < Calibrate -------------------------------------------------------------------------------------------------------

// > SENSOR ----------------------------------------------------------------------------------------------------------
uint8_t photoSig[3];
// < SENSOR ----------------------------------------------------------------------------------------------------------

// > TRAJECTORY ------------------------------------------------------------------------------------------------------

typedef struct {
	float x;
	float y;
} Point;

Point objPickPos[9];
Point objPlacePos[9];

typedef struct {
	int16_t pos1;
	int16_t pos2;
	int16_t pos3;
	int16_t pos4;
	int16_t pos5;
	int16_t pos6;
} trayPos;

trayPos trayPickX = { .pos1 = 0, .pos2 = 0, .pos3 = 0 };
trayPos trayPickY = { .pos1 = 0, .pos2 = 0, .pos3 = 0 };

trayPos trayPlaceX = { .pos1 = 0, .pos2 = 0, .pos3 = 0 };
trayPos trayPlaceY = { .pos1 = 0, .pos2 = 0, .pos3 = 0 };

typedef struct {
	float posTraj;
	float velTraj;
	float accTraj;
	uint8_t reachTraj;
} calculationTraj;

float qddm = 2000; // 210*11.205 -> 2117.745 float qddm = 2000; // 210*11.205 -> 2117.745
float qdm = 2100; // 189*11.205 -> 2353.05 float qdm = 2100; // 189*11.205 -> 2353.05

float actualTime = 0.001;

float checkPos = 0;
float checkVel = 0;
float checkAcc = 0;
// < TRAJECTORY ------------------------------------------------------------------------------------------------------

// > HOME STATE ------------------------------------------------------------------------------------------------------
uint8_t myHomeState = 0;
uint8_t startSetHome = 1;
// < HOME STATE ------------------------------------------------------------------------------------------------------

// > TEST START M2 ---------------------------------------------------------------------------------------------------
uint8_t startTraj = 0; // TRACK 18 path (Only Position)
uint8_t startKalman = 0; // Lab Kalman
uint8_t startOnlyPosControl = 0; // Feedback From
uint8_t startCascadeControl = 0; // Feedback From Sensor
uint8_t startJoyStick = 0; // Start JoyStick
// > TEST START M2 ---------------------------------------------------------------------------------------------------

// > POINT MODE ------------------------------------------------------------------------------------------------------
uint8_t startPointModeY = 0;
int32_t initPosY;
// < POINT MODE ------------------------------------------------------------------------------------------------------

// > END EFFECTOR ----------------------------------------------------------------------------------------------------
uint8_t endEffectorWriteFlag = 0;
uint8_t endEffectorReadFlag = 0;
uint8_t endEffectorReadBack[1];
uint8_t selectMode;
uint8_t selectStatus;
uint8_t readStatus[1];

struct endE {
	uint8_t testMode;
	uint8_t emergencyMode;
	uint8_t gripperWork;
	uint8_t gripperPickAndPlace;
	uint8_t reset;
	uint8_t status;
};
struct endE endEffector = { 0, 1, 2, 3, 4, 5 };
uint8_t endEffectorDataScan[2] = { 0, 0 };
uint8_t robotArmStateDataScan[2] = { 0, 0 };
uint8_t endEffectorState = 0;
// < END EFFECTOR ----------------------------------------------------------------------------------------------------

// WRITING ...

uint8_t startRunTray = 0;
uint8_t pathComplete = 0;
uint8_t pickComplete = 0;
uint8_t placeComplete = 0;

uint8_t runTrayModeCase = 0;
uint8_t endEffectorPicking = 0;
uint8_t endEffectorPlacing = 0;

uint8_t testFlag = 0;
uint8_t testMode = 0;
uint8_t testStatus = 0;
uint8_t joyStart = 0;
uint8_t nextPath = 0;

uint8_t passInit = 0;

uint8_t checkGoPick = 0;
uint8_t PIDCase = 0;

uint8_t x[4] = { 0, 0, 0, 0 };
uint8_t x2 = 0;

float finalPIDChecky = 0;
uint8_t myJoyState = 30;

uint8_t goPick = 1;
uint8_t goPlace = 1;

uint8_t endEffectorStatusStep = 0;
// Kalman Filter ----------
//double K = 0;
//double x = 0;
//double P = 0;
//double P_pre = 0;
//
//double R = 708.5903334;
//double C = 1;
//double Q = 10000;
//
//float kalmanVel = 0;
//float kalmanPos = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void readEncoder();
void setMotor();
void cascadePIDControl();
float positionLoop(float targetPos);
void velocityLoop(float targetVel, float velFromPID);

void buttonInput();
void buttonLogic(uint16_t state);
void photoDetect();
calculationTraj trapezoidalTraj(float initPos, float targetPos);
float kalmanFilter(float y);
void calibrateTrayTest();

void setHome();
void onlyPositionControl(float initPos, float targetPos);
void robotArmState(uint16_t state);

void calibrateTray(trayPos trayX, trayPos trayY, Point *objPos);
// Point rotatePoint(float p1, float p2, float centerX, float centerY, float angle);
Point rotatePoint(int16_t p1, int16_t p2, int16_t centerX, int16_t centerY,
		int16_t radians);
void jogAxisY();
void jogAxisX();

void joyDisplayLED();

void kalmanLap();

void handleEmergency();
void pilotLamp(uint8_t id, uint8_t status);

void endEffectorStatusControl(uint16_t regisFrame);
void endEffectorControl(uint8_t mode, uint8_t status);

void runTrayMode();

void endEffectorPick();
void endEffectorPlace();

void handleJogAxisY(uint8_t jogStateY);

void onlyPositionControlPointMode(float initPos, float targetPos);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM5_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM11_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	HAL_TIM_Base_Start_IT(&htim2);

	HAL_ADC_Start_DMA(&hadc1, (uint16_t*) buffer, 20);

	hmodbus.huart = &huart2;
	hmodbus.htim = &htim11;
	hmodbus.slaveAddress = 0x15;
	hmodbus.RegisterSize = 70; // 70
	Modbus_init(&hmodbus, registerFrame);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		Modbus_Protocal_Worker();
		static uint32_t timestamp = 0;

		handleEmergency();
		if (HAL_GetTick() >= timestamp) {
			timestamp = HAL_GetTick() + 100;

			int16_t sentPos = mmActPos * 10;
			int16_t sentVel = mmActVel * 10;
			int16_t sentAcc = mmActAcc * 10;
			registerFrame[0].U16 = 22881; // WRITE : Heartbeat Protocol
			registerFrame[17].U16 = sentPos; // WRITE : y-axis Actual Position
			registerFrame[18].U16 = sentVel; // WRITE : y-axis Actual Speed
			registerFrame[19].U16 = sentAcc; // WRITE : y-axis Actual Acceleration

			mmActPos = QEIReadModified * (2 * 3.14159 * 11.205 / 8192); // NO THIS FUNCTION WHEN TEST WITH ONLY Y-AXIS

//			if (x[0] && x2) {
//				endEffectorControl(endEffector.status, 0);
//
//				endEffectorPick();
//
//			} else if (x[1] && x2) {
//				endEffectorControl(endEffector.status, 0);
//
//				endEffectorPlace();
//			}
//			if (x[0] && x2) {
//				switch (endEffectorStatusStep) {
//				case 0:
//					endEffectorControl(endEffector.status, 0);
//					endEffectorStatusStep = 1;
//					break;
//				case 1:
//					endEffectorPick();
//					if (runTrayModeCase == 4) {
//						endEffectorStatusStep = 0;
//					} else {
//						endEffectorStatusStep = 2;
//					}
//					break;
//				case 2:
//					endEffectorControl(endEffector.status, 0);
//					endEffectorStatusStep = 1;
//					break;
//
//				}
//			} else if (x[1] && x2) {
//				switch (endEffectorStatusStep) {
//				case 0:
//					endEffectorControl(endEffector.status, 0);
//					endEffectorStatusStep = 1;
//					break;
//				case 1:
//					endEffectorPlace();
//					if (runTrayModeCase == 6) {
//						endEffectorStatusStep = 0;
//					} else {
//						endEffectorStatusStep = 2;
//					}
//					break;
//				case 2:
//					endEffectorControl(endEffector.status, 0);
//					endEffectorStatusStep = 1;
//					break;
//				}
//			}

			if (x[0] && x2) {
				//hi2cState = hi2c2.State;
				if (hi2c2.State == HAL_I2C_STATE_READY) {
					switch (endEffectorStatusStep) {
					case 0:
						endEffectorControl(endEffector.status, 0);
						endEffectorStatusStep = 1;
						break;
					case 1:
						endEffectorPick();
						if (runTrayModeCase == 4) {
							endEffectorStatusStep = 0;
						} else {
							endEffectorStatusStep = 2;
						}
						break;
					case 2:
						endEffectorControl(endEffector.status, 0);
						endEffectorStatusStep = 1;
						break;
					}
				}

			} else if (x[1] && x2) {
				if (hi2c2.State == HAL_I2C_STATE_READY) {
					switch (endEffectorStatusStep) {
					case 0:
						endEffectorControl(endEffector.status, 0);
						endEffectorStatusStep = 1;
						break;
					case 1:
						endEffectorPlace();
						if (runTrayModeCase == 6) {
							endEffectorStatusStep = 0;
						} else {
							endEffectorStatusStep = 2;
						}
						break;
					case 2:
						endEffectorControl(endEffector.status, 0);
						endEffectorStatusStep = 1;
						break;
					}
				}
			}
			endEffectorDataScan[1] = registerFrame[2].U16;
			if (endEffectorDataScan[1] != endEffectorDataScan[0]) {
				endEffectorStatusControl(registerFrame[2].U16);
				endEffectorDataScan[0] = endEffectorDataScan[1];

			}

			joyDisplayLED();

			if (joyStart) {

				buttonInput(); // DETECT : Button Input
				buttonLogic(joyLogic);
			}

		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 99;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 9999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 9;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 99;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 2005;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim11, TIM_OPMODE_SINGLE) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
	sConfigOC.Pulse = 1433;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 19200;
	huart2.Init.WordLength = UART_WORDLENGTH_9B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_EVEN;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
	GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA10 PA11 PA12 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB14 PB5 PB7
	 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_14 | GPIO_PIN_5 | GPIO_PIN_7
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PC6 PC8 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC10 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2 && x2 == 0) //
			{
		photoDetect();
		robotArmStateDataScan[1] = registerFrame[1].U16;
		if (robotArmStateDataScan[1] != robotArmStateDataScan[0]) {
			robotArmState(registerFrame[1].U16); // READ : Base System Status
			robotArmStateDataScan[0] = robotArmStateDataScan[1];
		}

		readEncoder();

		if (photoSig[0] || photoSig[2]) // SOFTWARE LIMIT : Motor/Encoder Photo Sensor
				{
			duty = 0;
			setMotor();
		}

		if (startSetHome) {
			setHome();
		}

		if (startPointModeY) {
			int16_t targetPosPointMode = registerFrame[49].U16; // READ : Goal Point y
			onlyPositionControlPointMode(initPosY, targetPosPointMode / 10);
		}

		if (startRunTray) {
			runTrayMode();
		}

	}
}

void runTrayMode() {

	switch (runTrayModeCase) {
	// BEFORE START SET HOME FIRST
	case 0:
		setHome();
		break;
	case 1: // GO PICK

		if (goPick) {
			// X-Axis
			PIDCase = 0;
			registerFrame[65].U16 =
					(int16_t) ((objPickPos[pathComplete].x) * 10); // SET : x-axis Target Position
			registerFrame[66].U16 = 3000; // SET : x-axis Target Speed
			registerFrame[67].U16 = 1; // SET : x-axis Target Speed

			registerFrame[64].U16 = 0b0000000000000010; // RUN : x-axis Moving Status

			// Y-Axis
			passInit = 0;

			initPosY = QEIReadModified * (2 * 3.14159 * 11.205 / 8192);
		}

		if (registerFrame[64].U16 == 0) {
			registerFrame[64].U16 = 2;
			runTrayModeCase = 2;
		}

		// runTrayModeCase = 2; //// only Test Y-Axis

		goPick = 0;
		break;
	case 2:
		goPick = 1;
		onlyPositionControl(initPosY, objPickPos[pathComplete].y);
		break;
	case 3:
		// END EFFECTOR CONTROL
		checkGoPick = 5;
		//endEffectorControl(endEffector.status, 0);
		//HAL_Delay(10);
		//endEffectorPick();
		// endEffectorPicking = 1;
		x2 = 1;
		x[0] = 1;

		break;
	case 4:  // GO PLACE

		if (goPlace) {
			// X-Axis
			PIDCase = 0;
			registerFrame[65].U16 = (int16_t) ((objPlacePos[pathComplete].x)
					* 10);  // SET : x-axis Target Position
			registerFrame[66].U16 = 3000; // SET : x-axis Target Speed
			registerFrame[67].U16 = 1; // SET : x-axis Target Speed

			registerFrame[64].U16 = 0b0000000000000010; // RUN : x-axis Moving Status

			// Y-Axis
			passInit = 0;

			initPosY = QEIReadModified * (2 * 3.14159 * 11.205 / 8192);
		}

		if (registerFrame[64].U16 == 0) {
			registerFrame[64].U16 = 2;
			runTrayModeCase = 5;
		}

		// runTrayModeCase = 5; // only test Y-Axis
		goPlace = 0;
		break;
	case 5:
		goPlace = 1;
		onlyPositionControl(initPosY, objPlacePos[pathComplete].y);
		break;
	case 6:
		//	checkGoPick = 0;
		// END EFFECTOR CONTROL
		//endEffectorControl(endEffector.status, 0);
		//HAL_Delay(10);
		//endEffectorPlace();
		// endEffectorPlacing = 1;
		x2 = 1;
		x[1] = 1;

		break;
	}

}

void endEffectorPick() {

	switch (endEffectorState) {
	case 0:
		endEffectorControl(endEffector.gripperWork, 1); // GRIPPER ON
		if (readStatus[0] == 0b100) {
			endEffectorState = 1;
		}
		break;
	case 1:
		endEffectorControl(endEffector.gripperPickAndPlace, 1); // PICK UP
//		if (readStatus[0] == 0b0111) {
//			endEffectorState = 2;
//		}
		if (readStatus[0] == 0b0111) {
			endEffectorState = 2;
		} else if (readStatus[0] != 0b0101) {
			endEffectorControl(endEffector.gripperPickAndPlace, 1); // PICK UP
		}
		break;

		break;
	case 2:
		endEffectorControl(endEffector.gripperWork, 0); // GRIPPER OFF
		if (readStatus[0] == 0b011) {
			endEffectorState = 3;
		}
		break;
	case 3:
		runTrayModeCase = 4;
		endEffectorState = 0;
		endEffectorPicking = 0;
		x[0] = 0;
		x2 = 0;
		break;
	}
}

void endEffectorPlace() {
	switch (endEffectorState) {
	case 0:
		endEffectorControl(endEffector.gripperWork, 1); // GRIPPER ON
		if (readStatus[0] == 0b111) {
			endEffectorState = 1;
		}
		break;
	case 1:
//		endEffectorControl(endEffector.gripperPickAndPlace, 0); // PICK DOWN
//		if (readStatus[0] == 0b0100) {
//			endEffectorState = 2;
//		}
		if (readStatus[0] == 0b0100) {
			endEffectorState = 2;
		} else if (readStatus[0] != 0b110) {
			endEffectorControl(endEffector.gripperPickAndPlace, 0); // PICK DOWN
		}
		break;
	case 2:
		endEffectorControl(endEffector.gripperWork, 0); // GRIPPER OFF
		if (readStatus[0] == 0b0000) {
			endEffectorState = 3;
		}
		break;
	case 3:
		runTrayModeCase = 1;
		endEffectorState = 0;
		endEffectorPicking = 0;
		pathComplete += 1;
		x[1] = 0;
		x2 = 0;
		if (pathComplete == 9) { // PUT IN SWITCH CASE
			// setHome();
			pathComplete = 0;
			runTrayModeCase = 0;
		}
		break;
	}

}
void setMotor() {
	if (dirAxisY) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

void readEncoder() {
	QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim5);
	QEIReadModified = QEIReadRaw - QEIHome;
}

void setHome() {

	switch (myHomeState) {
	case 0:
		if (photoSig[0]) // Motor Photo Sensor
		{
			myHomeState = 1;
		} else // ANY Position
		{
			dirAxisY = 0;
			duty = 360;
			setMotor();
			myHomeState = 1;
		}
		break;
	case 1:
		if (photoSig[0]) // Motor Photo Sensor
		{
			dirAxisY = 1;
			duty = 360;
			setMotor();
		} else if (photoSig[1]) // Center Photo Sensor
		{
			myHomeState = 2;
		}
		break;
	case 2:
		duty = 0;
		setMotor();
		QEIHome = QEIReadRaw;
		startSetHome = 0;
		myHomeState = 0;
		registerFrame[16].U16 = 0; // RESET : y-axis Moving Status
		mmActPos = 0;
		if (startRunTray) {
			runTrayModeCase = 1;
		}
		break;

	}

}

void cascadePIDControl() {
// calculationTraj result = trapezoidalTraj();
// velocityLoop(result.velTraj, positionLoop(result.posTraj));
}

float positionLoop(float targetPos) {

	mmActPos = QEIReadRaw * (2 * 3.14159 * 11.205 / 8192);
	mmActVel = (mmActPos - prePos) / 0.01;

	mmError = targetPos - mmActPos;
	positionPID.eIntegral = positionPID.eIntegral + (mmError * 0.01);
	pidVel = (positionPID.pTerm * mmError)
			+ (positionPID.iTerm * positionPID.eIntegral);
	return pidVel;

}

void velocityLoop(float targetVel, float velFromPID) {
	float velError = targetVel + velFromPID - mmActVel;
	duty = (velocityPID.pTerm * velError)
			+ (velocityPID.iTerm * velocityPID.eIntegral);
	if (duty < 0) {
		dirAxisY = 0;
		duty = (-1) * duty;
	} else {
		dirAxisY = 1;
	}
	if (duty > 1000) {
		duty = 1000;
	} else if (duty <= 120) {
		duty = 0;
	}
	prePos = mmActPos;
	setMotor();
}

void onlyPositionControl(float initPos, float targetPos) {
	calculationTraj result = trapezoidalTraj(initPos, targetPos);

	switch (PIDCase) {
	case 0:

		mmActPos = QEIReadModified * (2 * 3.14159 * 11.205 / 8192);
		mmActVel = (mmActPos - prePos) / 0.001;
		mmActAcc = (mmActVel - preVel) / 0.001;

		mmError = result.posTraj - mmActPos;
		positionPID.eIntegral = positionPID.eIntegral + (mmError * 0.001);
		duty = (positionPID.pTerm * mmError)
				+ (positionPID.iTerm * positionPID.eIntegral);
		if (duty < 0) {
			dirAxisY = 0;
			duty = (-1) * duty;
		} else {
			dirAxisY = 1;
		}
		if (duty > 1000) {
			duty = 1000;
		} else if (duty <= 300) {
			duty = 0;
		}

		setMotor();

		prePos = mmActPos;
		preVel = mmActVel;
		finalPIDChecky = result.velTraj;

		if (fabs(mmError) <= 2.5 && result.velTraj == 0.0 && passInit) {
			PIDCase = 1;
		}
		passInit = 1;
		break;
	case 1:
		if (runTrayModeCase == 2) {
			runTrayModeCase = 3;
			passInit = 0;
			PIDCase = 0;
			duty = 0;
			setMotor();

		} else if (runTrayModeCase == 5) {
			runTrayModeCase = 6;
			passInit = 0;
			PIDCase = 0;
			duty = 0;
			setMotor();

		}
		break;
	}

}

void onlyPositionControlPointMode(float initPos, float targetPos) {
	calculationTraj result = trapezoidalTraj(initPos, targetPos);

	mmActPos = QEIReadModified * (2 * 3.14159 * 11.205 / 8192);
	mmActVel = (mmActPos - prePos) / 0.001;
	mmActAcc = (mmActVel - preVel) / 0.001;

	mmError = result.posTraj - mmActPos;
	positionPID.eIntegral = positionPID.eIntegral + (mmError * 0.001);
	duty = (positionPID.pTerm * mmError)
			+ (positionPID.iTerm * positionPID.eIntegral);
	if (duty < 0) {
		dirAxisY = 0;
		duty = (-1) * duty;
	} else {
		dirAxisY = 1;
	}
	if (duty > 1000) {
		duty = 1000;
	} else if (duty <= 300) {
		duty = 0;
	}

	setMotor();

	prePos = mmActPos;
	preVel = mmActVel;
	finalPIDChecky = result.posTraj;

}
void jogAxisY() {
	refYPos = buffer[0].subdata.yAxis;
	if (refYPos > 2500) {
		dirAxisY = 1;
	} else if (refYPos < 1500) {
		dirAxisY = 0;
	}
	if (refYPos > 3600 || refYPos < 100) {
		duty = 290;
	} else if (refYPos > 2500 && refYPos <= 3600) {
		duty = 270;
	}

	else if (refYPos > 100 && refYPos <= 1500) {
		duty = 270;
	} else {
		duty = 0;
	}
	setMotor();

}
//
//void handleJogAxisY(uint8_t jogStateY)
//{
//	switch (jogStateY % 3)
//	{
//	case 0:
//		jogAxisY1();
//		break;
//	case 1:
//		jogAxisY2();
//		break;
//	case 2:
//		jogAxisY3();
//		break;
//	}
//}
//

//void jogAxisY1() {
//	refYPos = buffer[0].subdata.yAxis;
//	if (refYPos > 2500) {
//		dirAxisY = 1;
//	} else if (refYPos < 1500) {
//		dirAxisY = 0;
//	}
//	if (refYPos > 3600 || refYPos < 100) {
//		duty = 500;
//	} else if (refYPos > 2500 && refYPos <= 3600) {
//		duty = 400;
//	}
//
//	else if (refYPos > 100 && refYPos <= 1500) {
//		duty = 200;
//	} else {
//		duty = 0;
//	}
//	setMotor();
//
//}
//void jogAxisY2() {
//	refYPos = buffer[0].subdata.yAxis;
//	if (refYPos > 2500) {
//		dirAxisY = 1;
//	} else if (refYPos < 1500) {
//		dirAxisY = 0;
//	}
//	if (refYPos > 3600 || refYPos < 100) {
//		duty = 900;
//	} else if (refYPos > 2500 && refYPos <= 3600) {
//		duty = 700;
//	}
//
//	else if (refYPos > 100 && refYPos <= 1500) {
//		duty = 500;
//	} else {
//		duty = 0;
//	}
//	setMotor();
//
//}
//void jogAxisY3() {
//	refYPos = buffer[0].subdata.yAxis;
//	if (refYPos > 2500) {
//		dirAxisY = 1;
//	} else if (refYPos < 1500) {
//		dirAxisY = 0;
//	}
//	if (refYPos > 3600 || refYPos < 100) {
//		duty = 380;
//	} else if (refYPos > 2500 && refYPos <= 3600) {
//		duty = 140;
//	}
//
//	else if (refYPos > 100 && refYPos <= 1500) {
//		duty = 200;
//	} else {
//		duty = 0;
//	}
//	setMotor();
//
//}

void photoDetect() {
	photoSig[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);  // MOTOR Photo Sensor
	photoSig[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14); // CENTER Photo Sensor
	photoSig[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);  // ENCODER Photo Sensor
}

calculationTraj trapezoidalTraj(float qi, float qf) {
	calculationTraj result;

	float diffPos = abs(qf - qi);
	int8_t handleMinus = (qf - qi) / diffPos;
	float timeTrapSeg1 = qdm / qddm;
	float timeTriSeg1 = pow((diffPos / qddm), 0.5);

	if (timeTriSeg1 < timeTrapSeg1) // triangle shape
			{
		float qTriSeg1 = 0.5 * qddm * timeTriSeg1 * timeTriSeg1;
		float qdTriSeg1 = qddm * timeTriSeg1;
		if (actualTime <= timeTriSeg1) {
			result.posTraj = qi
					+ (0.5 * qddm * actualTime * actualTime) * handleMinus;
			result.velTraj = qddm * actualTime;
			result.accTraj = qddm;
		}

		else if (actualTime > timeTriSeg1 && actualTime <= (timeTriSeg1 * 2)) {
			float actualTimeSeg2 = actualTime - timeTriSeg1;
			result.posTraj = qi
					+ (qTriSeg1 + (qdTriSeg1 * actualTimeSeg2)
							- (0.5 * qddm * actualTimeSeg2 * actualTimeSeg2))
							* handleMinus;
			result.velTraj = qdTriSeg1 - (qddm * actualTimeSeg2);
			result.accTraj = -qddm;
		}

		else {
			result.posTraj = qf;
			result.velTraj = 0;
			result.accTraj = 0;
		}

	}

	else // trapezoidal shape
	{
		float timeTrapSeg2 = (diffPos
				- (2 * 0.5 * qddm * (timeTrapSeg1) * (timeTrapSeg1))) / qdm;
		float timeTrapSeg3 = qdm / qddm;
		float qTrapSeg1 = 0.5 * qddm * (timeTrapSeg1) * (timeTrapSeg1);
		float qTrapSeg2 = qTrapSeg1 + (qdm * timeTrapSeg2);

		if (actualTime <= timeTrapSeg1) {
			result.posTraj = qi
					+ (0.5 * qddm * actualTime * actualTime) * handleMinus;
			result.velTraj = qddm * actualTime;
			result.accTraj = qddm;
		} else if (actualTime > timeTrapSeg1
				&& actualTime <= timeTrapSeg2 + timeTrapSeg1) {
			float t2 = actualTime - timeTrapSeg1;
			result.posTraj = qi + (qTrapSeg1 + qdm * (t2)) * handleMinus;
			result.velTraj = qdm;
			result.accTraj = 0;
		} else if (actualTime > timeTrapSeg2 + timeTrapSeg1
				&& actualTime <= timeTrapSeg3 + timeTrapSeg2 + timeTrapSeg1) {
			float t3 = actualTime - timeTrapSeg2 - timeTrapSeg1;
			result.posTraj = qi
					+ (qTrapSeg2 + (qdm * t3) - 0.5 * qddm * t3 * t3)
							* handleMinus;
			result.velTraj = -qddm * t3 + qdm;
			result.accTraj = -qddm;
		} else {
			result.posTraj = qf;
			result.velTraj = 0;
			result.accTraj = 0;

		}

	}

	checkPos = result.posTraj;
	checkVel = result.velTraj;
	checkAcc = result.accTraj;

	actualTime += 0.001;

	// CHECK STATUS

	if (startPointModeY) 		// POINT MODE

	{

		if (result.posTraj == qf) {
			result.reachTraj = 1;
			actualTime = 0.001;
			startPointModeY = 0;
			initPosY = mmActPos;
			registerFrame[16].U16 = 0; // RESET : y-axis Moving Status
		} else {
			result.reachTraj = 0;
		}
	}

	if (startRunTray) {
		// RUN TRAY MODE

		if (result.posTraj == qf) {
			result.reachTraj = 1;
			actualTime = 0.001;
			initPosY = mmActPos;
			registerFrame[16].U16 = 0; // RESET : y-axis Moving Status // ---------
		}

		else {
			result.reachTraj = 0;
		}
	}

	return result;
}

void calibrateTray(trayPos trayX, trayPos trayY, Point *objPos) {
	float length1 = pow(
			pow(trayX.pos1 - trayX.pos2, 2) + pow(trayY.pos1 - trayY.pos2, 2),
			0.5);
	float length2 = pow(
			pow(trayX.pos2 - trayX.pos3, 2) + pow(trayY.pos2 - trayY.pos3, 2),
			0.5);
	uint8_t k = 50;
	if (length1 > length2) {
		k = 60;
	}

	float length3 = trayY.pos1 - trayY.pos2;
	float radians = acos(length3 / k);

	float TrayOriginX = trayX.pos2;
	float TrayOriginY = trayY.pos2;
	int16_t writeDeg = 36000 - (radians * (180 / M_PI) * 100);
	if (k == 60) {

		TrayOriginX = trayX.pos3;
		TrayOriginY = trayY.pos3;
		writeDeg = 27000 - (radians * (180 / M_PI) * 100);
		radians -= (1.5 * M_PI);
	}

	float a[3] = { 10.0f, 30.0f, 50.0f };
	float b[3] = { 40.0f, 25.0f, 10.0f };

	for (int i = 0; i < 9; i++) {
		uint8_t index = i % 3;
		objPos[i].x = TrayOriginX + a[index];

		uint8_t row = i / 3;
		objPos[i].y = TrayOriginY + b[row];
		objPos[i] = rotatePoint(objPos[i].x, objPos[i].y, TrayOriginX,
				TrayOriginY, radians);
	}

	int16_t writeTrayOriginX = TrayOriginX * 10; // CHANGE DATA TYPE
	int16_t writeTrayOriginY = TrayOriginY * 10; // CHANGE DATA TYPE
	if (calibrateTrayInput == 1) {
		registerFrame[32].U16 = writeTrayOriginX; // WRTTE : Pick Tray Origin x
		registerFrame[33].U16 = writeTrayOriginY; // WRTTE : Pick Tray Origin y
		registerFrame[34].U16 = writeDeg; // WRTTE : Pick Tray Orientation
	} else if (calibrateTrayInput == 2) {
		registerFrame[35].U16 = writeTrayOriginX; //  WRTTE : Place Tray Origin x
		registerFrame[36].U16 = writeTrayOriginY; // WRTTE : Place Tray Origin y
		registerFrame[37].U16 = writeDeg; // WRTTE : Place Tray Orientation
	}

}

Point rotatePoint(int16_t p1, int16_t p2, int16_t centerX, int16_t centerY,
		int16_t radians) {
// ROTATION MATRIX
	int16_t cosTheta = cosf(radians);
	int16_t sinTheta = sinf(radians);

	int16_t translatedX = p1 - centerX;
	int16_t translatedY = p2 - centerY;

	Point rotatedPoint;
	rotatedPoint.x = (translatedX * cosTheta) - (translatedY * sinTheta)
			+ centerX;
	rotatedPoint.y = (translatedX * sinTheta) + (translatedY * cosTheta)
			+ centerY;

	return rotatedPoint;
}

void buttonInput() {
	register int i;
	for (i = 0; i < 4; i++) {
		Button1[i].Current = HAL_GPIO_ReadPin(joyPin[i].PORT, joyPin[i].PIN);
		if (Button1[i].Last == 0 && Button1[i].Current == 1) {
			if (i == 0) {
				countTopB += 1;
			}
			if (i == 1) {
				countRightB += 1;
			}
			if (i == 2) {
				countBottomB += 1;
			}
			if (i == 3) {
				countLeftB += 1;

			}
			Button1[i].Last = Button1[i].Current;
			joyLogic = i;
		}
		Button1[i].Last = Button1[i].Current;
	}
}

void buttonLogic(uint16_t state) {
	if (countTopB % 2 == 1) {
		switch (state) {
		case 0: // ENTER JOG MODE
			joyLogicLED = 1;
			if (switchAxis) {
				jogAxisY();
			} else {
				jogAxisX();
			}

			// handleJogAxisY(myJoyState);
			break;
		case 1: // RIGHT
			myJoyState += 1;

			joyLogic = 0;
			break;
		case 2: // CHANGE AXIS X/Y
			if (switchAxis) {
				switchAxis = 0;
			} else {
				switchAxis = 1;
			}
			joyLogic = 0;
			break;
		case 3: // LEFT
			joyLogic = 0;
			myJoyState -= 1;
			break;
		}
	}
	if (countTopB % 2 == 0) {
		switch (state) {
		case 0: // ENTER CALIBRATE MODE
			if (countRightB == 4) {
				joyLogicLED = 3;
			} else if (countRightB == 7) {
				joyLogicLED = 4;
			} else {
				joyLogicLED = 2;
			}

			break;
		case 1: // MARK POSITION
			if (countRightB == 2) {
				trayPickX.pos1 = ((int16_t) registerFrame[68].U16 / 10.0); // READ : x-axis Actual Position
				trayPickY.pos1 = mmActPos;
			} else if (countRightB == 3) {
				trayPickX.pos2 = ((int16_t) registerFrame[68].U16 / 10.0); // READ : x-axis Actual Position
				trayPickY.pos2 = mmActPos;
			} else if (countRightB == 4) {
				registerFrame[16].U16 = 0;
				trayPickX.pos3 = ((int16_t) registerFrame[68].U16 / 10.0); // READ : x-axis Actual Position
				trayPickY.pos3 = mmActPos;
				calibrateTrayInput = 1;
				calibrateTray(trayPickX, trayPickY, objPickPos);
			} else if (countRightB == 5) {
				trayPlaceX.pos1 = ((int16_t) registerFrame[68].U16 / 10.0); // READ : x-axis Actual Position
				trayPlaceY.pos1 = mmActPos;
			} else if (countRightB == 6) {
				trayPlaceX.pos2 = ((int16_t) registerFrame[68].U16 / 10.0); // READ : x-axis Actual Position
				trayPlaceY.pos2 = mmActPos;
			} else if (countRightB == 7) {
				registerFrame[16].U16 = 0;
				trayPlaceX.pos3 = ((int16_t) registerFrame[68].U16 / 10.0); // READ : x-axis Actual Position
				trayPlaceY.pos3 = mmActPos;
				calibrateTrayInput = 2;
				calibrateTray(trayPlaceX, trayPlaceY, objPlacePos);
			}
			joyLogic = 0;
			break;
		case 2: // OPEN LASER
			joyLogic = 0;

			break;
		case 3: //  DELETE
			countRightB = 0;
			joyLogic = 0;
			break;
		}
	}

}

void joyDisplayLED() {
	if (joyLogicLED == 1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	} else if (joyLogicLED == 2) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	} else if (joyLogicLED == 3) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	} else if (joyLogicLED == 4) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
	}
}

void robotArmState(uint16_t state) {

	switch (state) {
	case 0b0000000000000001: // SET PICK TRAY
		registerFrame[16].U16 = 0b0000000000000001; // Jog Pick : y-axis Moving Status

		joyStart = 1;
		registerFrame[66].U16 = 300; // SET : x-axis Target Speed
		registerFrame[67].U16 = 3; // SET : x-axis Target Speed
		pilotLamp(0, 0); // OFF : PILOT LAMP LEFT
		pilotLamp(1, 1); // ON : PILOT LAMP CENTER
		pilotLamp(2, 0); // OFF : PILOT LAMP RIGHT

		break;
	case 0b0000000000000010: // SET PLACE TRAY
		registerFrame[16].U16 = 0b0000000000000010; // Jog Place : y-axis Moving Status

		joyStart = 1;
		registerFrame[66].U16 = 300; // SET : x-axis Target Speed
		registerFrame[67].U16 = 3; // SET : x-axis Target Speed
		pilotLamp(0, 0); // OFF : PILOT LAMP LEFT
		pilotLamp(1, 1); // ON : PILOT LAMP CENTER
		pilotLamp(2, 0);  // OFF : PILOT LAMP RIGHT

		break;
	case 0b0000000000000100: // HOME
		joyStart = 0;
		registerFrame[16].U16 = 0b0000000000000100; // HOME : y-axis Moving Status
		registerFrame[64].U16 = 0b0000000000000001; // HOME : x-axis Moving Status
		registerFrame[1].U16 = 0; // RESET : Base System Status

		startSetHome = 1; // START HOME -> Function

		pilotLamp(0, 1); // ON : PILOT LAMP LEFT
		pilotLamp(1, 0); // OFF : PILOT LAMP CENTER
		pilotLamp(2, 0); // OFF : PILOT LAMP RIGHT

		break;
	case 0b0000000000001000: // RUN TRAY MODE 18 PATH
		joyStart = 0;

		startRunTray = 1; // START RUN TRAY -> Function

		pilotLamp(0, 0); // OFF : PILOT LAMP LEFT
		pilotLamp(1, 0); // OFF : PILOT LAMP CENTER
		pilotLamp(2, 1); // ON : PILOT LAMP RIGHT
		break;
	case 0b0000000000010000: // RUN POINT MODE
		joyStart = 0;

		pilotLamp(0, 0); // OFF : PILOT LAMP LEFT
		pilotLamp(1, 0); // OFF : PILOT LAMP CENTER
		pilotLamp(2, 1); // ON : PILOT LAMP RIGHT

		// X-Axis
		registerFrame[64].U16 = 0b0000000000000010; // RUN : x-axis Moving Status
		registerFrame[65].U16 = registerFrame[48].U16; // SET : x-axis Target Position = Read : Goal Point x
		registerFrame[66].U16 = 3000; // SET : x-axis Target Speed
		registerFrame[67].U16 = 1; // SET : x-axis Target Speed

		// Y-Axis
		startPointModeY = 1; // START POINT MODE -> OnlyPositionControl Function
		initPosY = QEIReadModified * (2 * 3.14159 * 11.205 / 8192);

		registerFrame[16].U16 = 0b0000000000100000; // Go Point : y-axis Moving Status

		registerFrame[1].U16 = 0; // RESET: Base System Status

		break;
	}
}

void jogAxisX() {
	refXPos = buffer[0].subdata.xAxis;
	if (refXPos > 2500) {
		dirAxisX = 1;
		registerFrame[64].U16 = 0b0000000000000100; // JOG RIGHT : x-axis Moving Status
	} else if (refXPos < 1500) {
		dirAxisX = 0;
		registerFrame[64].U16 = 0b0000000000001000; // JOG LEFT : x-axis Moving Status
	} else {
		registerFrame[64].U16 = 0; // RESET : x-axis Moving Status

	}

}

void HAL_ADC_ConvCallback(ADC_HandleTypeDef *hadc) {

}

void handleEmergency() {
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
	}
}
void pilotLamp(uint8_t id, uint8_t status) {
	HAL_GPIO_WritePin(pilotLampPin[id].PORT, pilotLampPin[id].PIN, status);

}

void endEffectorControl(uint8_t mode, uint8_t status) {
	switch (mode) {
	case 0: // LED ON-Off
		if (hi2c2.State == HAL_I2C_STATE_READY) {
			x[2] += 1;
			static uint8_t data[2][2] = { { 0x00 }, { 0x01 } };
			HAL_I2C_Mem_Write(&hi2c2, endEffector_ADDR << 1, 0x01,
			I2C_MEMADD_SIZE_8BIT, data[status], 1, 100);
//			HAL_I2C_Master_Transmit(&hi2c2, endEffector_ADDR << 1, data[status],
//					2, 100);
		}
		break;

	case 1: //  Emergency Mode
		if (hi2c2.State == HAL_I2C_STATE_READY) {
			if (status == 1) {
				static uint8_t data[3] = { 0x7A, 0xFF, 0x81 };
				HAL_I2C_Mem_Write(&hi2c2, endEffector_ADDR << 1, 0xE5,
				I2C_MEMADD_SIZE_8BIT, data, 3, 100);
			} else {
				static uint8_t data[1] = { 0xF0 };
				HAL_I2C_Mem_Write(&hi2c2, endEffector_ADDR << 1, 0xF0,
				I2C_MEMADD_SIZE_8BIT, data, 0, 100);
			}
		}
		break;

	case 2: // Gripper Working and Gripper Stops Working
		if (hi2c2.State == HAL_I2C_STATE_READY) {
			static uint8_t data[2][1] = { { 0x8C }, { 0x13 } };
			HAL_I2C_Mem_Write(&hi2c2, endEffector_ADDR << 1, 0x10,
			I2C_MEMADD_SIZE_8BIT, data[status], 1, 100);
		}
		break;

	case 3: // Gripper Pick Up and Gripper Place down
		if (hi2c2.State == HAL_I2C_STATE_READY) {
			static uint8_t data[2][1] = { { 0x69 }, { 0x5A } };
			HAL_I2C_Mem_Write(&hi2c2, endEffector_ADDR << 1, 0x10,
			I2C_MEMADD_SIZE_8BIT, data[status], 1, 100);
		}
		break;

	case 4: // Soft reset
		if (hi2c2.State == HAL_I2C_STATE_READY) {
			x[2] += 1;
			static uint8_t data[3] = { 0xFF, 0x55, 0xAA };
			HAL_I2C_Mem_Write(&hi2c2, endEffector_ADDR << 1, 0x00,
			I2C_MEMADD_SIZE_8BIT, data, 3, 100);
		}
		break;

	case 5: // Current status
		if (hi2c2.State == HAL_I2C_STATE_READY) {
			HAL_I2C_Master_Receive(&hi2c2, endEffector_ADDR << 1, readStatus, 1,
					100);
		}
		break;
	}
}

void endEffectorStatusControl(uint16_t regisFrame) // PUT REGISTOR
{

	switch (regisFrame) {
	case 0b0000000000000000: // LASER OFF
		//x[2]=0;
		//endEffectorControl(endEffector.gripperWork, 0);
		//HAL_Delay(10);
		endEffectorControl(endEffector.testMode, 0);
		break;
	case 0b0000000000000001: // LASER ON
		//x[2]+=1;
		//endEffectorControl(endEffector.gripperWork, 0);
		//HAL_Delay(10);
		endEffectorControl(endEffector.testMode, 1);
		break;
	case 0b0000000000000010: // GRIPPER POWER

		endEffectorControl(endEffector.testMode, 0);
		//HAL_Delay(10);
		endEffectorControl(endEffector.gripperWork, 1);
		break;
	case 0b0000000000000110: // GRIPPER PICK

		endEffectorControl(endEffector.gripperPickAndPlace, 1); // 1 -> PICK
		break;
	case 0b0000000000001010: // GRIPPER PLACE
		endEffectorControl(endEffector.gripperPickAndPlace, 0); // 1 -> PLACE
		break;
	}

}

// --------------------------------------------------
//float kalmanFilter(float y) {
//	P_pre = P + Q;
//	K = (P_pre * C) / ((C * P_pre * C) + R);
//	x = x + K * (y - C * x);
//	P = (1 - (K * C) * P_pre);
//	return x;
//}
//void kalmanLap() {
//	mmActPos = QEIReadRaw * (2 * 3.14159 * 11.205 / 8192);
//	mmActVel = (mmActPos - prePos) / 0.01;
//	setMotor();
//	prePos = mmActPos;
//}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
