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
 *
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


#define PID_PARAM_KP		2		/* Proporcional */  //PARAMETROS PID  //1    //6.06
#define PID_PARAM_KI		0.432		/* Integral */                        //0.05 //0.43
#define PID_PARAM_KD		21.21			/* Derivative */                      //0.25 //21.21

void Delay(__IO uint32_t nTime);  //funcion Delay que usa SysTick
int32_t devolver_temperatura_en_grados(); // funcion PHinclude
void color_segun_temperatura();
void iniciarPWM(int32_t duty);
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

	SystemInit(); // inicializa el sistema

	TIM_Config(); // config timers para PWM y demas

	UB_LCD_2x16_Init(); // inicializa el display 16x2

    declarar_leds();     // GPIO leds pin 11 12 13 14

	Init_Teclado();   // inicializa el teclado de la lib teclado.c

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

	//teclado

	char stringtemperaturadeseada[4];
	char bufferteclado[4]={0,0,0,0};
	int temperaturaporteclado[4]={0,0,0,0}, tempSet=0;
	int i=0,flag=0,ingreso=0;

	UB_LCD_2x16_Clear();
	UB_LCD_2x16_String(0,0,"PreHeat 1.0");
	Delay(2000);
	UB_LCD_2x16_Clear();
	UB_LCD_2x16_String(0,0,"Ingrese temp:");

	while(flag==0)  // flag que se hace 1 cuando se tomaron 3 valores
	{
	for(i=0;i<4;i++) // for de 4 valores
	{
		ingreso=0;
		while(ingreso==0) // ingreso se hace 1 cuando se ingreso un valor del teclado osea distinto de 16
		{

		temperaturaporteclado[i]=Leer_Teclado();  // usa la funcion leer para leer de teclado.c

			/* anula los botones no usados */
			if((temperaturaporteclado[i]!=16)&&(temperaturaporteclado[i]!=12)&&(temperaturaporteclado[i]!=13)&&(temperaturaporteclado[i]!=14)&&(temperaturaporteclado[i]!=15)&&(temperaturaporteclado[i]!=10)&&(temperaturaporteclado[i]!=11))
			{
				sprintf(bufferteclado,"%d",temperaturaporteclado[i]);
				UB_LCD_2x16_String(i,1,bufferteclado);

				Delay(500); // delay para que no tome mas de un valor cuando de presiona
				break;  // sale porque entro un numero distinto de 16 y demas letras y simbolos

			}

		}
		if(i==3){flag=1;}  // hasta que se llene el buffer
	}
	}
	UB_LCD_2x16_Clear();

	tempSet=armarEntero(temperaturaporteclado,4); // funcion que transforma el buffer en un INT

	sprintf(stringtemperaturadeseada,"%d",tempSet);    // muestra la temperatura por pantalla
	UB_LCD_2x16_String(0,1,stringtemperaturadeseada);

	Delay(1000);

	//teclado

	int32_t duty; //   DUTY !!! (ciclo de trabajo)
	int32_t pid_error=0;
	int32_t temperatura_Actual=0,temperatura_Deseada=500;
	//double errSum=0,lastErr=0;
	int32_t kp=PID_PARAM_KP;      //,ki=PID_PARAM_KI,kd=PID_PARAM_KD;
	//double dErr=0;

	temperatura_Deseada=tempSet;   // fijo la temperatura deseada obtenida del teclado

	while (1)
    	{

		for(i=0;i<100;i++)
		{

			temperatura_Actual+=devolver_temperatura_en_grados();

		}

		temperatura_Actual=temperatura_Actual/100;


    	sprintf(stringtemperatura,"%d",devolver_temperatura_en_grados());   // pasa de un entero a un String para imprimir

    	UB_LCD_2x16_Clear();                    //usa una funcion ya definida para limpiar las string
    	UB_LCD_2x16_String(0,0,"Temp actual:");
    	UB_LCD_2x16_String(0,1,stringtemperatura);    // usa una funcion ya definida para imprimir un string
    	UB_LCD_2x16_String(3,1,"\176");
    	UB_LCD_2x16_String(5,1,stringtemperaturadeseada);
    	UB_LCD_2x16_String(9,1,"       ");
    	Delay(1000);  // 50Hz, 50 ondas por seg si uso medio segundo (0.5seg) tengo control sobre la onda... ideal 1 seg

    	//color_segun_temperatura();

		//Delay(250); // DELAY NECESARIO PARA QUE CONVERSIONES Y QUE EL PWM SE ACTIVE BIEN MANEJANDO ONDA 50Hz

    	/* Calcular error*/
    	pid_error = temperatura_Deseada-temperatura_Actual;

//    	errSum+=(pid_error*0.001);
//    	dErr=(pid_error-lastErr)/0.001;

    	duty   =   pid_error;   //+   ki*errSum   +  kd*dErr;

    	duty=  kp  * duty;

//    	lastErr=pid_error;


    	 // Anti sobrecarga del duty
    	if (duty > 100) {duty = 100;}
    	if (duty < 0)   {duty = 0;}


    	duty   = (duty*499);   // como el duty va de 498 a 0 se lo adapta
    	duty   = (duty/100);

    	iniciarPWM(duty);  //TIM_OCInitStructure.TIM_Pulse=duty;

    	//printf(" %d  %d \n",temperatura_Actual,duty);  // !!!!!!! si no lo tenes en DEBUG!!! FRENA EL PROGRAMA!!!!

    	 if(duty==0)   //duty = 0
    		    	    {
    		    	    	GPIO_SetBits(GPIOD,GPIO_Pin_15);
    		    	    	GPIO_ResetBits(GPIOD,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_12);
    		    	    }
    	 if(duty>0)
    	 	 	 	 	{
    	    		    	GPIO_SetBits(GPIOD,GPIO_Pin_14);
    	    		    	GPIO_ResetBits(GPIOD,GPIO_Pin_13|GPIO_Pin_15|GPIO_Pin_12);
    	 	 	 	 	}

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

	    	    if(temperaturaengrados<500)   //menor de 500 grados enciende el led VERDE
	    	    {
	    	    	GPIO_ToggleBits(GPIOD,GPIO_Pin_12);
	    	    	GPIO_ResetBits(GPIOD,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	    	    }

	    	    if(temperaturaengrados>=500) //Mayor o igual a  500 grados enciende el led ROJO
	    	       {
	    	    	GPIO_SetBits(GPIOD,GPIO_Pin_13);
	    	       	GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15);
	    	       }
}

void iniciarPWM(int32_t duty)
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
