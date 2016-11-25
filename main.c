/** PROGRAMA PRINCIPAL PRE-CALENTADOR DE PCB
 *  Integrantes: Marano Garcia
 *
 *LO IMPLEMENTADO:
 *-DISPLAY 16X2
 *-LM35 (1)
 *-PID
 *-TECLADO
 *
 *Falta implementar:
 *-Ajuste PID
 *-MENU
 *-MEDIDOR DE POTENCIA
 *
 *  Adelante y suerte
 */

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "cmsis_lib/include/stm32f4xx_tim.h"   //lib timers
#include "PHinclude.h"       //lib propia de funciones
#include "adc.h"             // lib de ADC
#include "LCD/include/stm32_ub_lcd_2x16.h"  // lib LCD 16X2
#include "cmsis_lib/include/PID_FUNCIONES.h"  // lib adaptacion de arm_math... solo funciones PID
#include "stdio.h"
#include "teclado.h"


#define PID_PARAM_KP		100			/* Proporcional */  //PARAMETROS PID  //1
#define PID_PARAM_KI		0.025		/* Integral */                        //0.05
#define PID_PARAM_KD		20			/* Derivative */                      //0.25

void Delay(__IO uint32_t nTime);  //funcion Delay que usa SysTick
int32_t devolver_temperatura_en_grados(); // funcion PHinclude
void color_segun_temperatura();
void iniciarPWM(int duty);
void TIM_Config(void);
void arm_pid_init_f32();

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //variable para el timer y PWM
TIM_OCInitTypeDef  TIM_OCInitStructure;   //variable para el timer y PWM


uint16_t PrescalerValue = 0;

#define MAX_ADC	4095.0    // resolucion de ADC 12bit

int32_t adc_valor_obtenido;  // lectura del ADC
uint32_t time_ms = 0;

static __IO uint32_t TimingDelay;

int main(void)
	{

	uint16_t duty; //   DUTY !!! (ciclo de trabajo)
	float pid_error=0;

	int32_t temperatura_Actual=0,temperatura_Deseada=700;

	arm_pid_instance_f32 PID;

	/* Set PID parameters */
		/* Set this for your needs */
	PID.Kp = PID_PARAM_KP;		/* Proporcional */
	PID.Ki = PID_PARAM_KI;		/* Integral */
	PID.Kd = PID_PARAM_KD;		/* Derivative */

	/* Initialize PID system, float32_t format */
	PID.A0 = PID_PARAM_KP + PID_PARAM_KI + PID_PARAM_KD; //The derived gain, A0 = Kp + Ki + Kd . */
	PID.A1 = (-PID_PARAM_KP - (2*PID_PARAM_KD) );  //The derived gain, A1 = -Kp - 2Kd. */
    PID.A2 = PID_PARAM_KD; // The derived gain, A2 = Kd . */

	PID.state[0]=0;
	PID.state[1]=0;
	PID.state[2]=0;

	SystemInit(); // inicializa el sistema

	TIM_Config(); // config timers para PWM y demas

	UB_LCD_2x16_Init(); // inicializa el display 16x2

    declarar_leds();     // GPIO leds pin 11 12 13 14

	//declarar_boton();    // GPIO boton

	adc_inicializar();   // Inicializa ADC polling

	SysTick_Config(SystemCoreClock / 1000);

	// Ejemplo:
	// HCLK= 168MHz
	// Requerimiento= 1 mseg
	// 		1seg --- 168 Mticks
	// 		1ms ---- x
	// 		x = 168.000 ticks
	// 		para lograr este valor divido 168 M ticks / 1000 = 168.000 ticks

	char stringtemperatura[4]; // String donde se guarda la temperatura



	while (1)
    	{

		temperatura_Actual=devolver_temperatura_en_grados();

		/*
    	sprintf(stringtemperatura,"%d",devolver_temperatura_en_grados());   // pasa de un entero a un String para imprimir

    	UB_LCD_2x16_Clear();                    //usa una funcion ya definida para limpiar las string
    	UB_LCD_2x16_String(0,0,"Temp actual:");
    	UB_LCD_2x16_String(0,1,stringtemperatura);    // usa una funcion ya definida para imprimir un string
    	UB_LCD_2x16_String(3,1,"\176");
    	Delay(250);
		*/
    	//color_segun_temperatura();
		Delay(1); // DELAY NECESARIO PARA QUE CONVERSIONES Y QUE EL PWM SE ACTIVE BIEN

    	/* Calculate error */
    	pid_error =  temperatura_Deseada-temperatura_Actual;

    					/* Calculate PID here, argument is error */
    					/* Output data will be returned, we will use it as duty cycle parameter */
    	duty = arm_pid_f32(&PID, pid_error);

    		/* Check overflow, duty cycle in percent */
    					if (duty > 100) {
    						duty = 100;
    					} else if (duty < 0) {
    						duty = 0;
    					}

    	duty= ((498/100)*(duty));   // como el duty va de 498 a 0 se lo adapta

    	iniciarPWM(duty);

    	//TIM_OCInitStructure.TIM_Pulse=duty;

    	}
	}


void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
}

int32_t devolver_temperatura_en_grados()
{
	int32_t temperatura=0;

	temperatura=adc_leer_cuentas();   // Lee el ADC de la funcion adc.h y PHinclude

	temperatura=((temperatura*3000)/4095);  // de Tension de ADC a Grados centigrados

	return temperatura;
}


void color_segun_temperatura()
{
				//GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15); //restart todos los led

				int32_t temperaturaengrados=devolver_temperatura_en_grados();

	    	    if(temperaturaengrados<270)   //menor de 27 grados enciende el led VERDE
	    	    {
	    	    	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	    	    	GPIO_ResetBits(GPIOD,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	    	    }

	    	    if(temperaturaengrados>=270) //Mayor o igual a  27 grados enciende el led ROJO
	    	       {
	    	    	GPIO_SetBits(GPIOD,GPIO_Pin_13);
	    	       	GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15);
	    	       }
}

void iniciarPWM(int duty)
{
	 /* Compute the prescaler value */
		  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1; // 1MHZ

	//PWM PWM PWM PWM PWM PWM PWM PWM PMW
		/* Time base configuration */
		  TIM_TimeBaseStructure.TIM_Period = 499; //499
		  TIM_TimeBaseStructure.TIM_Prescaler =PrescalerValue ;
		  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

		  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		  /* PWM1 Mode configuration: Channel1 */
		  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		  TIM_OCInitStructure.TIM_Pulse = duty;                         // DUTY !!!
		  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

		  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

		  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

		  TIM_ARRPreloadConfig(TIM3, ENABLE);

		  /* TIM3 enable counter */
		  TIM_Cmd(TIM3, ENABLE);
		//PWM PWM PWM PWM PWM PWM PWM PWM PMW
}

/*void control_PID(arm_pid_instance_f32 *PID,int temperatura_Deseada, uint16_t duty)
{
	int32_t temperatura_Actual=devolver_temperatura_en_grados();

	float pid_error =  temperatura_Deseada-temperatura_Actual;

	duty = arm_pid_f32(PID, pid_error);

    		//Check overflow, duty cycle in percent
    					if (duty > 100) {
    						duty = 100;
    					} else if (duty < 0) {
    						duty = 0;
    					}

    	duty= ((490/100)*(duty));

    	iniciarPWM(duty);
}*/