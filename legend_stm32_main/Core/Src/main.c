/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FlashPROM.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define bool uint16_t
#define false 0
#define true 1
#define PWM_8BIT 64  // 8-bit PWM 
#define I2C_SLAVE_ADDRESS 0x67
#define I2C_TIMEOUT 10
#define I2C_BUFFER 14 
	
//status byte 0
#define STATUS_BIT0_L_ON          0
#define STATUS_BIT0_R_ON          1
#define STATUS_BIT0_L_BUT         2
#define STATUS_BIT0_R_BUT         3
#define STATUS_BIT0_WATERMODE     4		 // Water Mode
#define STATUS_BIT0_SQUEEZE       5  
#define STATUS_BIT0_OVERTEMP      6
#define STATUS_BIT0_RESERVE       7

//status byte 1

#define STATUS_BIT1_L_TRIP_STAGE1 0   // L TRIP current protection
#define STATUS_BIT1_R_TRIP_STAGE1 1   // R TRIP current protection
#define STATUS_BIT1_L_TRIP_STAGE2 2   // L TRIP over current protection short circuit
#define STATUS_BIT1_R_TRIP_STAGE2 3   // R TRIP over current protection short circuit
#define STATUS_BIT1_DO_SHIFT      4
#define STATUS_BIT1_DO_ERROR      5
#define STATUS_BIT1_DI_MODE1      6
#define STATUS_BIT1_DI_MODE2      7

//status byte 2
#define STATUS_BIT2_GEAR_P        0
#define STATUS_BIT2_FORCE         1
#define STATUS_BIT2_GEAR_N        2
#define STATUS_BIT2_GEAR_D        3
#define STATUS_BIT2_CURR1_L       4
#define STATUS_BIT2_CURR2_L       5
#define STATUS_BIT2_CURR1_R       6
#define STATUS_BIT2_CURR2_R       7

#define PARAMETERS 30     //See also BUFFSIZE 30 in FlashPROM.h
#define PAGES_COUNT 6
#define STR_LENGTH 15*2 // 2 bytes per symbol


struct AnalogFilter{
	uint64_t in_value;
	uint16_t out_value;
	uint16_t time_rate;
	uint16_t analog_limit;
	uint16_t time_count;
	bool trip;
};


struct SwitchFilter{
	bool outstate;
	bool lock;
	bool trigger;
	uint16_t scanrate;
	GPIO_TypeDef * port;
	uint16_t pin;
	uint16_t curr_scan;
};

struct Timer_v_in{
	bool timer_v_in_trigger;
  uint16_t timer_v_in_on;
  uint16_t timer_v_in_off;
	uint16_t timer_v_off_pause;
};

struct pwm{
		uint8_t dc_fill;
		bool enable;
	  uint8_t duty_cycle;
		GPIO_TypeDef * port;
	  uint16_t pin;
	  uint8_t curr_tick;
};	

struct double_pwm{
		uint8_t dc_fill;
		bool enable;
	  bool trigger;
	  uint16_t duty_cycle;
		GPIO_TypeDef * port_hi;
	  uint16_t pin_hi;
		GPIO_TypeDef * port_lo;
	  uint16_t pin_lo;
	  uint8_t curr_tick;
};	

struct i2c_data_{
  uint8_t statusbyte0;
  uint8_t statusbyte1;
  uint8_t statusbyte2;
  uint8_t GP_Speed;
  uint8_t GP_temp;
  uint8_t current_page;
  uint8_t paramfromdisp;
  uint8_t paramfromdisp_addr;
};

struct _ProcessChannel{
  uint8_t gear_mode;
	bool button_pressed;
	struct double_pwm * pwm; 
	uint16_t  duty_cycle; 
	bool squeeze;
	bool force;
	uint8_t * led;
	uint8_t * strength;
	uint8_t T_release;
	uint8_t T_on;
	bool fault;
	uint16_t timer0;
	uint16_t timer1;
};


struct MenuRow{
  uint8_t id;
	char desc_cust[STR_LENGTH];
	char desc_en[STR_LENGTH];
  uint8_t def;
  uint8_t min;
  uint8_t max;
  uint8_t writable; // 1- writable, 0 - readonly
  int8_t dp; //decimal point position
  uint8_t symb; // 0 - null, 1 - c, 2 - %, 3- A, 4 - s, 5 - mc, 6 - ms, 7 - Celsius, 8 - km/h
};

struct MenuRow menus[] = {
  {1,"01","01 I L LOW",30,0,100,1,0,2},
  {2,"02","02 I L MID",60,0,100,1,0,2},
  {3,"03","03 I L HIGH",100,0,100,1,0,2},
  {4,"04","04 I R LOW",30,0,100,1,0,2},
  {5,"05","05 I R MID",60,0,100,1,0,2},
  {6,"06","06 I R HIGH",100,0,100,1,0,2},
  {7,"07","07 Trel. norm.",5,0,100,1,1,1},
  {8,"08","08 Trel. water",20,0,100,1,1,1},
  {9,"09","09 Act curr. L",0,0,0,0,1,3},
  {10,"10","10 Act curr. R",0,0,0,0,1,3},
  {11,"11","11 Tr_del. L",0,0,50,1,1,1},
  {12,"12","12 Tr_del. R",0,0,50,1,1,1},
  {13,"13","13 I parking P",15,0,100,1,0,2},
  {14,"14","14 Temp fail",80,20,140,1,0,7},
  {15,"15","15 Off on temp",1,0,1,1,0,0},
  {16,"16","16 Iovercur. L",15,0,30,1,0,3},
  {17,"17","17 Iovercur. R",15,0,30,1,0,3},
  {18,"18","18 CAN speed",4,0,6,1,0,0},
  {19,"19","19 CAN address",0x11,0x01,0xFE,1,0,0}, 
  {20,"20","20 DI mode 1",0,0,1,0,0,0}, 
  {21,"21","21 DI mode 2",0,0,1,0,0,0}, 
	{22,"22","22 I Calibr L",40,10,200,1,0,0}, 
	{23,"23","23 I Calibr R",40,10,200,1,0,0}, 
  {24,"24","24 Speed cal.",29,1,250,1,0,0},  
  {25,"25","25 Firmware", 10,1,100,0,1,0},
	{26,"26","26 Speed 1 st",  3,1,100,1,0,8},
	{27,"27","27 Speed 2 st", 15,1,100,1,0,8},
	{28,"28","28 DI SQ/FORCE", 1,0,1,1,0,0},
	{29,"29","29 DI WTR/FUNC", 1,0,1,1,0,0},
  {30,"30","30 Language",1,0,1,1,0,0},
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

struct AnalogFilter pa[7]; // analog pin filter pins PA0-PA6
struct SwitchFilter pa7,pb0,pb1,pb2,pb15,pa8,pa12; //DI filter PA7,PB0,PB1,PB2,PB15,PA8,PA12
//struct pwm on_R_H, on_R_L, on_L_H, on_L_L;
struct double_pwm pwm_R, pwm_L;

uint16_t params_buf[PARAMETERS+1]; // zero byte - FLASH not erased info
uint32_t res_addr = 0;
uint32_t tick = 0;
uint16_t speed_tick = 0;
uint16_t adc_curr_L, adc_curr_R = 0; 

struct _ProcessChannel Channel_L, Channel_R;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

ADC_ChannelConfTypeDef sConfig = {0};
ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

struct MainDataSet{
  int16_t GP_Temp;
  uint16_t GP_Speed;
  bool Water_mode;
  bool Squeeze_mode;
	bool Force_mode;
  bool Left_button;
  bool Right_button;
  uint8_t Right_Strength;       // Empty, LOW, MID, HIGH
  uint8_t Right_Strength_level; //Level in persantage
  uint8_t Left_Strength;        // Empty, LOW, MID, HIGH
  uint8_t Left_Strength_level;  //Level in persantage
  uint8_t selector_mode;        // 1 - P, 2 - R, 3 - N, 4 - D
  uint8_t Left_fault_stage1;
	uint8_t Left_fault_stage2;
  uint8_t Right_fault_stage1;
	uint8_t Right_fault_stage2;
	uint8_t Left_On;
	uint8_t Right_On;
	uint8_t DO_shift;
	uint8_t DO_error;
	uint8_t DI_mode1;
	uint8_t DI_mode2;
	bool OverTemp;
	uint8_t gear;	
};

struct MainDataSet main_data;
struct i2c_data_ i2c_data;
uint8_t tick100Hz = 0;
uint16_t  temp0, temp2 = 0;
long temp1 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t i2c_buf[I2C_BUFFER];
uint8_t i2c_receive[I2C_BUFFER];
uint8_t regAddress = 0;


uint8_t Calc_PWM(uint8_t duty_cycle)
{
	
	 if (duty_cycle == 0) return 0;
	
	 uint16_t result = PWM_8BIT*duty_cycle/100;
	 return (uint8_t) result;
}
	
bool AnalogPinTrip(struct AnalogFilter * ana)
{
  if (ana->analog_limit == 0) return false;
 	
	if (ana->in_value > ana->analog_limit)
	{
		if (ana->time_count >= ana->time_rate)
		{
			ana->trip=true;
		} else
		  ana->time_count++;
	} else
  {	
	  ana->time_count = 0;
		ana->trip=false;
	}
	return ana->trip;
}

void AnalogAverage(struct AnalogFilter * ana)
{
	if (ana->time_count == 0) return;
	
  ana->out_value = (uint16_t) sqrt(ana->in_value/ana->time_count); // RMS average
			
	ana->time_count = 0;
	ana->in_value = 0;
}

void AnalogAddValue(struct AnalogFilter * ana, uint16_t value)
{
	ana->in_value += value*value; // sum of squares
	ana->time_count++;
}

void Init_Channels()
{
	Channel_L.timer0 = 0;
	Channel_L.timer1 = 0;
	Channel_L.fault = false;
	Channel_L.force = false;
	Channel_L.pwm = &pwm_L;
	Channel_L.strength = &main_data.Left_Strength;
	
	Channel_R.timer0 = 0;
	Channel_R.timer1 = 0;	
	Channel_R.fault = false;
	Channel_R.force = false;
	Channel_R.pwm = &pwm_R;
	Channel_R.strength = &main_data.Right_Strength;
}

void Init_i2c_data()
{
  i2c_data.statusbyte0 = 0;
  i2c_data.statusbyte1 = 0;
  i2c_data.statusbyte2 = 0;
  i2c_data.paramfromdisp = 0;
  i2c_data.paramfromdisp_addr = 0;
  i2c_data.current_page = 0;
  i2c_data.GP_Speed = 0;
  i2c_data.GP_temp = 0;
}

void InitPWM()
{
	pwm_R.dc_fill = 0;
	pwm_R.duty_cycle = 0;
	pwm_R.enable = true;
	pwm_R.trigger = false;
	pwm_R.port_hi = GPIOB;
	pwm_R.pin_hi = GPIO_PIN_13;
  pwm_R.port_lo = GPIOB;
	pwm_R.pin_lo = GPIO_PIN_14;

	pwm_L.dc_fill = 0;
	pwm_L.duty_cycle = 0;
	pwm_L.enable = true;
	pwm_L.trigger = false;
	pwm_L.port_hi = GPIOA;
	pwm_L.pin_hi = GPIO_PIN_11;
  pwm_L.port_lo = GPIOB;
	pwm_L.pin_lo = GPIO_PIN_12;
}

void InitMainScreenSet()
{
  main_data.GP_Speed = 0xFFFF;
  main_data.GP_Temp = 0xFFFF;

  main_data.Left_button = false;
  main_data.Left_Strength_level = 0;
  main_data.Left_Strength = 0;

  main_data.Right_button = false;
  main_data.Right_Strength_level = 0;
  main_data.Right_Strength = 0;

  main_data.Squeeze_mode = false;
	main_data.Force_mode = false;
  main_data.Water_mode = false;
  main_data.selector_mode = 0;
	main_data.Left_On = 0;
	main_data.Right_On = 0;
	main_data.Left_fault_stage1 = 0;
	main_data.Left_fault_stage2 = 0;
	main_data.Right_fault_stage1 = 0;
	main_data.Right_fault_stage2 = 0;
	main_data.DI_mode1 = 0;
	main_data.DI_mode2 = 0;
	main_data.DO_error = 0;
	main_data.DO_shift = 0;
	main_data.OverTemp = false;
	main_data.gear = 0;	
}

void InitAnalogPins(void)
{
	
	//CS_L
	pa[0].trip = false;
	pa[0].in_value = 0;
	pa[0].time_rate = 3;
	pa[0].time_count = 0;
	pa[0].out_value = 0;
	pa[0].analog_limit = 0; 

	//CS_R
  pa[1].trip = false;
	pa[1].in_value = 0;
	pa[1].time_rate = 3;
	pa[1].time_count = 0;
	pa[1].out_value = 0;
	pa[1].analog_limit = 0; 
	
	//SEARCH COIL L
  pa[2].trip = false;
	pa[2].in_value = 0;
	pa[2].time_rate = 3; 
	pa[2].time_count = 0;
	pa[2].out_value = 0;
	pa[2].analog_limit = 0; 
	
	//SEARCH COIL R
  pa[3].trip = false;
	pa[3].in_value = 0;
	pa[3].time_rate = 3;  
	pa[3].time_count = 0;
	pa[3].out_value = 0;
	pa[3].analog_limit = 0;	
	
	//TEMP 0
  pa[4].trip = false;
	pa[4].in_value = 0;
	pa[4].time_rate = 3;  
	pa[4].time_count = 0;
	pa[4].out_value = 0;
	pa[4].analog_limit = 0;

	//ANALOG 1
  pa[5].trip = false;
	pa[5].in_value = 0;
	pa[5].time_rate = 3;  
	pa[5].time_count = 0;
	pa[5].out_value = 0;
	pa[5].analog_limit = 0;	
	
	//ANALOG 2
  pa[6].trip = false;
	pa[6].in_value = 0;
	pa[6].time_rate = 3;  
	pa[6].time_count = 0;
	pa[6].out_value = 0;
	pa[6].analog_limit = 0;	
	
}

bool _switch_filter(struct SwitchFilter * source)
{	
	if (!(HAL_GPIO_ReadPin(source->port,source->pin)))
	{
		if (source->curr_scan >= source->scanrate){	
		    
			
			source->outstate = true;
			source->trigger = true;
		} else
	
		source->curr_scan++;
		
		
		if  (source->scanrate == 0xFFFF) source->curr_scan = 0;
		
	} else
	{ 
	    source->curr_scan = 0;	
			source->outstate = false;
		
		  if (source->trigger) 
			{ 
				source->lock = false;
				source->trigger = false;
			}
	}

	
	return source->outstate;
}

void InitSwitches(void)
{
	 //DI filter PA7,PB0,PB1,PB2,PB15,PA8,PA9,PA12

		pa7.outstate = false;
	  pa7.lock = false;
	  pa7.trigger = false;
	  pa7.port = GPIOA;
	  pa7.pin = GPIO_PIN_7;
	  pa7.scanrate = 4;
	
		pb0.outstate = false;
	  pb0.lock = false;
	  pb0.trigger = false;
	  pb0.port = GPIOB;
	  pb0.pin = GPIO_PIN_0;
	  pb0.scanrate = 4;

		pb1.outstate = false;
	  pb1.lock = false;
	  pb1.trigger = false;
	  pb1.port = GPIOB;
	  pb1.pin = GPIO_PIN_1;
	  pb1.scanrate = 4;

		pb2.outstate = false;
		pb2.lock = false;
		pb2.trigger = false;
	  pb2.port = GPIOB;
	  pb2.pin = GPIO_PIN_2;
	  pb2.scanrate = 4;
		
		pa8.outstate = false;
		pa8.lock = false;
		pa8.trigger = false;
	  pa8.port = GPIOA;
	  pa8.pin = GPIO_PIN_8;
	  pa8.scanrate = 4;		
		
    pa12.outstate = false;
		pa12.lock = false;
		pa12.trigger = false;
	  pa12.port = GPIOA;
	  pa12.pin = GPIO_PIN_12;
	  pa12.scanrate = 4;		

}

void ProcessPWM(struct pwm * channel) //one transistor PWM
{
	
	 if (channel->dc_fill == 0){
		 HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_RESET);
		 return;
	 }
	 
	 channel->curr_tick++;
	
	if (channel->curr_tick >= PWM_8BIT)
	{
		 channel->curr_tick = 0;
	}
  
	if (channel->enable && (channel->dc_fill >= channel->curr_tick))
	{
		HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_SET);
	} else
	  HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_RESET);
}

void DoublePWM(struct double_pwm * pwm) //HIGH and LOW transistors used to PWM
{
	 if (pwm->dc_fill == 0 || (!pwm->enable)){
		 HAL_GPIO_WritePin(pwm->port_hi, pwm->pin_hi, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(pwm->port_lo, pwm->pin_lo, GPIO_PIN_RESET);
		 return;
	 }
	 
	 pwm->curr_tick++;
	 
	 if (pwm->curr_tick >= PWM_8BIT)
	 {
		  pwm->curr_tick = 0;
		  pwm->trigger = !pwm->trigger; 
	 }
	 
	 if (pwm->trigger)
	 { 
	 	 HAL_GPIO_WritePin(pwm->port_lo, pwm->pin_lo, GPIO_PIN_SET);
		 
	 	 if (pwm->dc_fill >= pwm->curr_tick)
	    {
	      HAL_GPIO_WritePin(pwm->port_hi, pwm->pin_hi, GPIO_PIN_SET);
	    } else
	      HAL_GPIO_WritePin(pwm->port_hi, pwm->pin_hi, GPIO_PIN_RESET);
	  
	 } else
	 {
		 HAL_GPIO_WritePin(pwm->port_hi, pwm->pin_hi, GPIO_PIN_SET);
		 
	 	 if (pwm->dc_fill >= pwm->curr_tick)
	    {
	      HAL_GPIO_WritePin(pwm->port_lo, pwm->pin_lo, GPIO_PIN_SET);
	    } else
	      HAL_GPIO_WritePin(pwm->port_lo, pwm->pin_lo, GPIO_PIN_RESET);
		 
	 }
}


void ProcessChannel(struct _ProcessChannel * channel)
{
	
	if ((channel->gear_mode == 0) || (channel->squeeze == 1))  //Neutral
	{
		channel->pwm->duty_cycle = 0;
		*channel->led = 0;
		return;
	}
	
	if (channel->gear_mode == 1) //Parking
	{
		if (params_buf[13] >= 97) params_buf[13] = 97; //Safety condition to charge BOOTSTRAP capacitor. 100% DC are not applicable
		
		channel->pwm->duty_cycle = params_buf[13];
		
		if (!channel->fault)
		*channel->led = 1; else *channel->led = 0;
	}
	
	if (channel->duty_cycle >= 97) channel->duty_cycle = 97; //Safety condition to charge BOOTSTRAP capacitor. 100% DC are not applicable
	
	
	if (channel->gear_mode == 2) //Drive
	{

	
  	if (channel->button_pressed) 
	  {	
			
	    if (channel->T_release > 0)
			{
				channel->timer1 = 0;
				
	      if (channel->timer0 < channel->T_release*10)
	      {
		       channel->timer0++;
					 channel->pwm->duty_cycle = (channel->T_release*10 - channel->timer0)*channel->duty_cycle/channel->T_release/10;  //formula DC_actual=(T_rel-T)*DC/Trel
			 
    	  } else
				{		  		
    				channel->pwm->duty_cycle = 0;
        }						
		    
			
			} 
			else 
			channel->pwm->duty_cycle = 0;
		  
			if (channel->pwm->duty_cycle == 0) *channel->led = 0;
	  
		}
		else 
		{
			channel->timer0 = 0;
			
			if (channel->T_on> 0)
			{
				if (channel->timer1 < channel->T_on*10)
				{
						channel->timer1++;
					  channel->pwm->duty_cycle = (channel->timer1)*channel->duty_cycle/channel->T_on/10;  //formula DC_actual=T*DC/Ton
					  
				} else
				{		  		
    				channel->pwm->duty_cycle = channel->duty_cycle;
        }					
			
			} 
			else 
						channel->pwm->duty_cycle = channel->duty_cycle;
			
			if (!channel->fault)
			*channel->led = 1; else *channel->led = 0;
		}
	}
	
	
	if (channel->pwm->duty_cycle == 0) *channel->strength = 0; 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) // 100Hz Timer
{ 
	if(htim->Instance == TIM3)
  {
		
		//Calculate PWM (persentage to 8-bit value)
		pwm_L.dc_fill = Calc_PWM(pwm_L.duty_cycle);
		pwm_R.dc_fill = Calc_PWM(pwm_R.duty_cycle);
		
		// Analog values pa[0]...pa[7]
	
			for (uint8_t i=0;i<7;i++)
				AnalogPinTrip(&pa[i]);
	
		tick100Hz++;
		
		
		//DI filter PA7,PB0,PB1,PB2,PA8,PA9,PA12
		_switch_filter(&pa7);  //Mode 1
		_switch_filter(&pb0);  //Button L
		_switch_filter(&pb1);  //Button R
		_switch_filter(&pb2);  //Squeeze
		_switch_filter(&pa8);  //Mode 2
		_switch_filter(&pa12); //Water
		
		ProcessChannel(&Channel_L); //LEFT channel processing
	  ProcessChannel(&Channel_R); //RIGHT channel processing


	}
	
	if (htim->Instance == TIM4)
	{
		temp1++;
		DoublePWM(&pwm_L);
		DoublePWM(&pwm_R);
	}
}

uint32_t GetADC1_Value(uint32_t channel)
{

		uint32_t g_ADCValue = 0;
		sConfig.Channel = channel;

		sConfig.Rank = 1;

		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5; //or any other value available.

		//add to channel select

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
				return -1;
		}
    HAL_ADC_Start(&hadc1);
		

		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
			
		g_ADCValue = HAL_ADC_GetValue(&hadc1);

		//remove from channel select

		HAL_ADC_Stop(&hadc1);
		
		sConfig.Rank = 0; //ADC_RANK_NONE;

		
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			return -1;
		}

		return (g_ADCValue);
}


void WriteParamsToFlash()
{
	params_buf[0] = 0;
	
	for (int i = 1; i <= PARAMETERS;i++)
	{
		if ((params_buf[i] < (uint16_t) menus[i-1].min) || (params_buf[i] > (uint16_t) menus[i-1].max)) params_buf[i] = menus[i-1].def;
	}
	
	write_to_flash(&params_buf[0]);
}

void UpdateStatus()
{
	bool write_params = false;
	
	//Status byte 0  
	i2c_data.statusbyte0 = ((bool)(main_data.Left_On) << STATUS_BIT0_L_ON) | 
		           ((bool)(main_data.Right_On) << STATUS_BIT0_R_ON) |
						   ((bool)(main_data.Left_button) << STATUS_BIT0_L_BUT) |
							 ((bool)(main_data.Right_button) << STATUS_BIT0_R_BUT) |
						   ((bool)(main_data.Water_mode) << STATUS_BIT0_WATERMODE) |
							 ((bool)(main_data.Squeeze_mode) << STATUS_BIT0_SQUEEZE) | 
							 ((bool)(main_data.OverTemp) << STATUS_BIT0_OVERTEMP) | 
							 (0 << STATUS_BIT0_RESERVE);
							 
	//Status byte 1
  i2c_data.statusbyte1 = ((bool)(main_data.Left_fault_stage1) << STATUS_BIT1_L_TRIP_STAGE1) | 
		           ((bool)(main_data.Right_fault_stage1) << STATUS_BIT1_R_TRIP_STAGE1) |
						   ((bool)(main_data.Left_fault_stage2) << STATUS_BIT1_L_TRIP_STAGE2) |
							 ((bool)(main_data.Right_fault_stage2) << STATUS_BIT1_R_TRIP_STAGE2) |
						   ((bool)(main_data.DO_shift) << STATUS_BIT1_DO_SHIFT) |
							 ((bool)(main_data.DO_error) << STATUS_BIT1_DO_ERROR) | 
							 ((bool)(main_data.DI_mode1) << STATUS_BIT1_DI_MODE1) | 
							 ((bool)(main_data.DI_mode2) << STATUS_BIT1_DI_MODE2);
	
  //Status byte 2
	
							  
							 
  i2c_data.statusbyte2 = (main_data.gear) |
						   (main_data.Left_Strength << STATUS_BIT2_CURR1_L) |
							 (main_data.Right_Strength << STATUS_BIT2_CURR1_R);		
	//Temperature
	i2c_data.GP_temp = main_data.GP_Temp;
							 
	
	//Speed
	i2c_data.GP_Speed = main_data.GP_Speed;
	
	
	i2c_buf[1] = i2c_data.statusbyte0;
	i2c_buf[2] = i2c_data.statusbyte1;
	i2c_buf[3] = i2c_data.statusbyte2;
	i2c_buf[4] = i2c_data.GP_temp;
	i2c_buf[5] = i2c_data.GP_Speed;
	i2c_buf[6] = i2c_data.current_page;
	
	
	// current parameters at page						 
	for (int i=0; i<5; i++)
	{
		if ((i2c_data.current_page > 0) && (i2c_data.current_page <= 6))
		{   
		  int index = i+1+(i2c_data.current_page-1)*5;
			
			if (index <= PARAMETERS)
			    i2c_buf[7+i] = params_buf[index];
			else
			    i2c_buf[7+i] = 0;
		}
		else
			 i2c_buf[7+i] = 0;
	}
	
	if ((i2c_data.paramfromdisp_addr > 0) && (i2c_data.paramfromdisp_addr <= PARAMETERS))
	{
		if (i2c_data.paramfromdisp >= menus[i2c_data.paramfromdisp_addr-1].min)
		if (i2c_data.paramfromdisp <= menus[i2c_data.paramfromdisp_addr-1].max)
		{
			write_params = (params_buf[i2c_data.paramfromdisp_addr] != (uint16_t) i2c_data.paramfromdisp);
				
			params_buf[i2c_data.paramfromdisp_addr] = (uint16_t) i2c_data.paramfromdisp;
			
			if (write_params)	
			{
				WriteParamsToFlash();     
			}
			
		}
	}


}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
 //
}



void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // I2C data ready!
}



bool CheckParamsLimits(uint16_t * params)
{
	bool result = true;
	
	for (int i=1; i<= PARAMETERS; i++)
	{
	 if ((uint16_t) params[i] < (uint16_t) menus[i-1].min) result = false;
	 if ((uint16_t) params[i] > (uint16_t) menus[i-1].max) result = false;
	}
	
	return result;
}


void WriteDefsToFlash()
{
	params_buf[0] = 0;
	for (int i = 1; i <= PARAMETERS; i++){
		  params_buf[i] = menus[i-1].def;
	}
	
	write_to_flash(&params_buf[0]);

}

uint8_t SelectCurrentFromSpeed(uint8_t low_dc, uint8_t mid_dc, uint8_t high_dc, uint8_t * strength)
{
	uint8_t result = 0;
	
	if (((main_data.GP_Speed >= 0) && (main_data.GP_Speed < params_buf[26]) || main_data.Force_mode)){
  		result = high_dc;  
		  * strength = 3;
	}
	else
	if ((main_data.GP_Speed >= params_buf[26]) && (main_data.GP_Speed < params_buf[27])){
	  	result = mid_dc;
      * strength = 2;		
	}
	else
	if  (main_data.GP_Speed >= params_buf[27]) 
	{
			result = low_dc;
			* strength = 1;
  }		
	
	return result;
}

void UpdateChannelsData()
{
  Channel_L.button_pressed = pb0.outstate;
	Channel_L.gear_mode = main_data.selector_mode;
	Channel_L.duty_cycle = SelectCurrentFromSpeed(params_buf[1],params_buf[2],params_buf[3], &main_data.Left_Strength);
	

	Channel_L.squeeze = main_data.Squeeze_mode;
	Channel_L.led = &main_data.Left_On;
	Channel_L.T_release = params_buf[11];

  Channel_R.button_pressed = pb1.outstate;
	Channel_R.gear_mode = main_data.selector_mode;
	Channel_R.duty_cycle = SelectCurrentFromSpeed(params_buf[4],params_buf[5],params_buf[6], &main_data.Right_Strength);
	
	Channel_R.squeeze = main_data.Squeeze_mode;
	Channel_R.led = &main_data.Right_On;	
	Channel_R.T_release = params_buf[12];
	
	if (main_data.Water_mode) //Watermode current up
	{
	    Channel_L.T_on = params_buf[8];
	    Channel_R.T_on = params_buf[8];
	}
  else //Normal current up
	{
		  Channel_L.T_on = params_buf[7];
	    Channel_R.T_on = params_buf[7];
	}
}


void ProcessDIevents()
{

	if (pb0.outstate) //Button L
	{
		main_data.Left_button = 1;
		
	} else
	{
		main_data.Left_button = 0;
    
	}

	if (pb1.outstate) //Button R
	{
		main_data.Right_button = 1;
	} else
	{
		main_data.Right_button = 0;
    
	}	
	

	
	if (pa7.outstate) //DI Mode 1
	{
    main_data.DI_mode1 = 1;  
	}
	else
	{
		main_data.DI_mode1 = 0;
	}
		
	params_buf[20] = main_data.DI_mode1;
	
	
	if (pb2.outstate) //Squeeze/Force
	{
		if (params_buf[28] == 1) main_data.Force_mode = 1;
		else
														 main_data.Squeeze_mode = 1;
	} else
	{
		main_data.Force_mode = 0;
		main_data.Squeeze_mode  = 0;
	}
	
	if (pa8.outstate) //DI Mode 2
	{
		main_data.DI_mode2 = 1;
	} else
	{
		main_data.DI_mode2  = 0;
		
	}
	
	params_buf[21] = main_data.DI_mode2;
	
	if (pa12.outstate) //Water
	{
		main_data.Water_mode = 1;
	} else
	{
		main_data.Water_mode  = 0;
	}
	
  main_data.gear = (1 << STATUS_BIT2_GEAR_N);
	main_data.selector_mode = 0;
	
	if (main_data.DI_mode1) {
		  main_data.gear = (1 << STATUS_BIT2_GEAR_P);
		  main_data.selector_mode = 1;
	}
	
  if (main_data.DI_mode2) {
		  main_data.gear = (1 << STATUS_BIT2_GEAR_D);
		  main_data.selector_mode = 2;
	}	
	 
	if (main_data.Force_mode) {
		  main_data.gear = (1 << STATUS_BIT2_FORCE);
	}	
	
}

int16_t ADC_to_Celsius(int16_t adc)
{
	float result = 0;
	
	if ((adc <= 0x004) || (adc >= 3280)) // short circuit on sensor and open circuit
		return -127;
	
	if (adc >= 640)	
	  result = -0.00000714*adc*adc-0.0104*adc+76.5; //cold temperature curve approximation 
	else
	  result = 0.0014*adc*adc-1.5857*adc+513.13; //high temperature curve approximation
	
	return (int16_t) roundf(result); 
}

void CANsendmsg()
{
	CAN_TxHeaderTypeDef msgHeader;
  uint8_t msgData[8];

	if (params_buf[18] == 0) return;
	
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
  {

    msgHeader.StdId = params_buf[19]; //CAN bus node address
    msgHeader.DLC = 8;
    msgHeader.TransmitGlobalTime = DISABLE;
    msgHeader.RTR = CAN_RTR_DATA;
    msgHeader.IDE = CAN_ID_STD;
    
    uint32_t mailBoxNum = 0;
		
		msgData[0] = i2c_data.statusbyte0;
		msgData[1] = i2c_data.statusbyte1;
		msgData[2] = i2c_data.statusbyte2;
		msgData[3] = (int8_t)  i2c_data.GP_temp;
	  msgData[4] = (uint8_t) i2c_data.GP_Speed;
		
		msgData[5] = params_buf[9];  //current L
		msgData[6] = params_buf[10]; //current R
    
    HAL_CAN_AddTxMessage(&hcan, &msgHeader, &msgData[0], &mailBoxNum);
  }
}

void ScanSpeedSensor()
{
	bool value;
	static bool old_value;
  
	uint16_t result = 0;
		
	value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	
	if (value && !old_value) //RISING EDGE
	{
    speed_tick++;
	}
	
	old_value = value;
}

void DOError()
{
		if (main_data.DO_error) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); else
		                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t timers[8];
	

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	res_addr = flash_search_adress(STARTADDR, PARAMETERS * DATAWIDTH);
	read_last_data_in_flash(&params_buf[0]); 
	 
	if (params_buf[0] == 0xFFFF)
	{
		erase_flash();
		HAL_Delay(5);
		WriteDefsToFlash();
	}
	if (!CheckParamsLimits(&params_buf[0]))
	{
		WriteDefsToFlash();	
	}	

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
	
	
	for (int j=0; j<8; j++)
	     timers[j] = 0;	
	
	InitMainScreenSet();
	InitPWM();
	Init_i2c_data();
	Init_Channels();
	UpdateChannelsData();
	
	HAL_TIM_Base_Start_IT(&htim3);		
	HAL_TIM_Base_Start_IT(&htim4);	
	
	InitAnalogPins();
	InitSwitches();	
	

	
	CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
  
	
	if (params_buf[18] > 0)
	{
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
	

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_IWDG_Refresh(&hiwdg);
		
		if (!pb0.lock) {
			  pwm_L.enable = true;
			  main_data.Left_fault_stage2 = false;
			  Channel_L.fault = false;
			  
		}
		if (!pb1.lock) {
			  pwm_R.enable = true;
			  main_data.Right_fault_stage2 = false;
   			Channel_R.fault = false;
		}
		
		if (!pb0.lock && !pb1.lock) main_data.DO_error = false;
		
		
		AnalogAddValue(&pa[0], GetADC1_Value(ADC_CHANNEL_0));  //CS_L
		AnalogAddValue(&pa[1], GetADC1_Value(ADC_CHANNEL_1));  //CS_R
		
		AnalogAddValue(&pa[4], GetADC1_Value(ADC_CHANNEL_4)); //TEMP0
		
		
		//pa[2].in_value = GetADC1_Value(ADC_CHANNEL_2);  //SEARCH_L
		//pa[3].in_value = GetADC1_Value(ADC_CHANNEL_3);  //SEARCH_R

		//pa[5].in_value = GetADC1_Value(ADC_CHANNEL_5);  //ANALOG 1
		//pa[6].in_value = GetADC1_Value(ADC_CHANNEL_6);  //ANALOG 2		

		ScanSpeedSensor();
		DOError();
		
		
		if ((HAL_GetTick() - timers[0]) > 100) //0.1 sec
		{ 
			UpdateStatus();
			UpdateChannelsData();
			ProcessDIevents();
			
			timers[0] = HAL_GetTick();	
		}
		

		
		if ((HAL_GetTick() - timers[1]) > 150) //0.15 sec
		{	
			HAL_I2C_Master_Transmit(&hi2c2, (I2C_SLAVE_ADDRESS << 1), &i2c_buf[0], I2C_BUFFER,  I2C_TIMEOUT);
			
			CANsendmsg();
			timers[1] = HAL_GetTick();	
		}
		
		
		if ((HAL_GetTick() - timers[2]) > 5) //every 5 ms
		{			
			timers[2] = HAL_GetTick();
		}

		
		if ((HAL_GetTick() - timers[3]) > 300) //0.3 sec
		{
			AnalogAverage(&pa[0]);
			AnalogAverage(&pa[1]);
			AnalogAverage(&pa[4]);
			
			

			params_buf[9] = pa[0].out_value/params_buf[22]; //Actual Current L
			
			params_buf[10] = pa[1].out_value/params_buf[23]; //Actual Current R
			
			if (((uint16_t) params_buf[9] >=  (uint16_t) params_buf[16]*10) && (params_buf[16] > 0)) // Overload LEFT
			{
			    pwm_L.enable = false;
			    main_data.DO_error = true;
			    main_data.Left_fault_stage2 = true;
			    pb0.lock = true;
				  Channel_L.fault = true;
			  	
		  } 	
			
			if (((uint16_t) params_buf[10] >= (uint16_t) params_buf[17]*10) && (params_buf[17] > 0)) // Overload RIGHT
			{
			    pwm_R.enable = false;
			    main_data.DO_error = true;
			    main_data.Right_fault_stage2 = true;
			    pb1.lock = true;
				  Channel_R.fault = true;
				  
		  } 	
			
			
			main_data.GP_Temp = ADC_to_Celsius(pa[4].out_value);   //Temperature of GP
			
			if (params_buf[15] == 1)
			{
					bool OverTemp_flag = main_data.OverTemp;
			    main_data.OverTemp = (main_data.GP_Temp >= params_buf[14]);
			
			    if (OverTemp_flag && !main_data.OverTemp) {
						  
						  main_data.DO_error = false;
						  pwm_R.enable = true;
					  	pwm_L.enable = true;
						  Channel_L.fault = false;
						  Channel_R.fault = false;
						
					}
			
					if (main_data.OverTemp) //Overheat
					{
				    main_data.DO_error = true;
						pwm_R.enable = false;
						pwm_L.enable = false;
						main_data.Left_On = false;
						main_data.Right_On = false;
						Channel_L.fault = true;
						Channel_R.fault = true;
					} 
			
				}
			 
			timers[3] = HAL_GetTick();
		}
		
		
		if ((HAL_GetTick() - timers[4]) > 500) //0.50 sec
		{
					
			HAL_I2C_Master_Receive_IT(&hi2c2, (I2C_SLAVE_ADDRESS << 1), &i2c_receive[0], I2C_BUFFER);
			i2c_data.current_page 			= i2c_receive[1];
			i2c_data.paramfromdisp 			= i2c_receive[2];
			i2c_data.paramfromdisp_addr = i2c_receive[3];
			
			if (params_buf[24] > 0)
    			main_data.GP_Speed = speed_tick*20/params_buf[24];
		
			speed_tick = 0;
			
			timers[4] = HAL_GetTick();
			
		}
		
		if ((HAL_GetTick() - timers[5]) > 1000) //1.0 sec
		{
			
			timers[5] = HAL_GetTick();
		}

		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
	
	if (params_buf[18] > 0)
	  hcan.Init.Prescaler = params_buf[18]*2; //params_buf[18] = 4 (125kbps), 3 (166kbps), 2 (250kbps), 1 (500kbps)
	else 
		hcan.Init.Prescaler = 8;

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  //hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	


  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 325;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 160;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	


	 

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 50;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA7 PA8 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


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
  while (1)
  {
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
