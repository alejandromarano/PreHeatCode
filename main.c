/** PROGRAMA PRINCIPAL PRE-CALENTADOR DE PCB
 *  Integrantes: Marano Garcia
 *
 *LO IMPLEMENTADO:
 *-DISPLAY 16X2
 *-LM35 (2)
 *-PID (PROPORCIONAL)
 *-TECLADO
 *-MENU
 *
 *Falta implementar:
 *
 *
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


#define PID_PARAM_KP		100		/* Proporcional */  //PARAMETROS PID  //1    //6.06
#define PID_PARAM_KI		0.432		/* Integral */                        //0.05 //0.43
#define PID_PARAM_KD		21.21			/* Derivative */                      //0.25 //21.21

void Delay(__IO uint32_t nTime);  //funcion Delay que usa SysTick
int32_t devolver_temperatura_en_grados(); // funcion PHinclude
void iniciarPWM();
void TIM_Config(void);

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //variable para el timer y PWM
TIM_OCInitTypeDef  TIM_OCInitStructure;   //variable para el timer y PWM

uint16_t PrescalerValue = 0;

int sensor_numero=0;

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

	adc_inicializar();   // Inicializa ADC polling

	SysTick_Config(SystemCoreClock / 1000);

	char stringtemperatura[4],stringzona[1]; // String donde se guarda la temperatura
	char stringtemperaturadeseada[4];       // String de temperatura deseada
	char bufferteclado[4]={0,0,0,0};        // String Buffer de char para guardar los numeros tomados por teclado
	char stringduty[4]={0,0,0,0};           // String de ciclo de trabajo
	int temperaturaporteclado[4]={0,0,0,0}, temperatura_deseada=0;
	int i=0,flag=0,ingreso=0,zona_seleccionada=0;

	// INICIO DEL MENU //

	UB_LCD_2x16_Clear();
	UB_LCD_2x16_String(0,0,"PreHeat 1.0");
	Delay(2000);
	UB_LCD_2x16_Clear();
	UB_LCD_2x16_String(0,0,"Temp? [0 a 1000]");

	// INGRESO DE TEMPERATURA //

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

	temperatura_deseada=armarEntero(temperaturaporteclado,4); // funcion que transforma el buffer en un INT

	UB_LCD_2x16_String(0,0,"Usted selecciono:");
	sprintf(stringtemperaturadeseada,"%d",temperatura_deseada);    // muestra la temperatura por pantalla
	UB_LCD_2x16_String(0,1,stringtemperaturadeseada);

	Delay(2000);

	if(temperatura_deseada>1000) // control de sobrecarga MAX 100 GRADOS
		{
			UB_LCD_2x16_Clear();
			UB_LCD_2x16_String(0,0,"Sobrecarga");
			UB_LCD_2x16_String(0,1,"temp = 1000");
			temperatura_deseada=1000;
			sprintf(stringtemperaturadeseada,"%d",temperatura_deseada);
			Delay(2000);
		}

	// INGRESO DE ZONA //

	flag=0;	// reset de flag
	i=0;

	UB_LCD_2x16_Clear();
	UB_LCD_2x16_String(0,0,"Zona? [1 a 3]");

	while(flag==0)  // flag que se hace 1 cuando se tomaron 3 valores
	{

		zona_seleccionada=Leer_Teclado();  // usa la funcion leer para leer de teclado.c

			/* anula los botones no usados solo se usa 1 2 3 */
			if((zona_seleccionada!=16)&&(zona_seleccionada!=12)&&(zona_seleccionada!=13)&&(zona_seleccionada!=14)&&(zona_seleccionada!=15)&&(zona_seleccionada!=10)&&(zona_seleccionada!=11)&&(zona_seleccionada!=4)&&(zona_seleccionada!=5)&&(zona_seleccionada!=6)&&(zona_seleccionada!=7)&&(zona_seleccionada!=8)&&(zona_seleccionada!=9)&&(zona_seleccionada!=0))
			{
				sprintf(bufferteclado,"%d",zona_seleccionada);
				UB_LCD_2x16_String(i,1,bufferteclado);
				flag=1;
				Delay(500); // delay para que no tome mas de un valor cuando de presiona
				break;  // sale porque entro un numero distinto de 16 y demas letras y simbolos

			}
	}

	UB_LCD_2x16_Clear();
	UB_LCD_2x16_String(0,0,"Usted selecciono:");
	UB_LCD_2x16_String(0,1,"Zona:");
	sprintf(stringzona,"%d",zona_seleccionada);    // muestra la temperatura por pantalla
	UB_LCD_2x16_String(5,1,stringzona);

	Delay(2000);

	// INGRESO DE NUMERO DE SENSOR //

	flag=0;	// reset de flag
	i=0;

	UB_LCD_2x16_Clear();
	UB_LCD_2x16_String(0,0,"Sensor? [1 a 2]");

	while(flag==0)  // flag que se hace 1 cuando se tomaron 2 valores
	{

		sensor_numero=Leer_Teclado();  // usa la funcion leer para leer de teclado.c

			/* anula los botones no usados solo se usa 1 2 */
			if((sensor_numero!=16)&&(sensor_numero!=12)&&(sensor_numero!=13)&&(sensor_numero!=3)&&(sensor_numero!=14)&&(sensor_numero!=15)&&(sensor_numero!=10)&&(sensor_numero!=11)&&(sensor_numero!=4)&&(sensor_numero!=5)&&(sensor_numero!=6)&&(sensor_numero!=7)&&(sensor_numero!=8)&&(sensor_numero!=9)&&(sensor_numero!=0))
			{
				sprintf(bufferteclado,"%d",sensor_numero);
				UB_LCD_2x16_String(i,1,bufferteclado);
				flag=1;
				Delay(500); // delay para que no tome mas de un valor cuando de presiona
				break;  // sale porque entro un numero distinto de 16 y demas letras y simbolos

			}
	}

	UB_LCD_2x16_Clear();
	UB_LCD_2x16_String(0,0,"Usted selecciono:");
	UB_LCD_2x16_String(0,1,"Sensor:");
	sprintf(stringzona,"%d",sensor_numero);    // muestra la temperatura por pantalla
	UB_LCD_2x16_String(7,1,stringzona);


	Delay(2000);

	iniciarPWM(); // declaracion del PWM

	uint32_t duty; //   DUTY !!! (ciclo de trabajo)
	int32_t pid_error=0;
	uint32_t temperatura_actual=0;
	//double errSum=0,lastErr=0;
	int32_t kp=PID_PARAM_KP;      //,ki=PID_PARAM_KI,kd=PID_PARAM_KD;
	//double dErr=0;

	while (1)
    	{


		/* promedio de 100 muestras para mantener el valor estable*/
		for(i=0;i<100;i++)
		{

			temperatura_actual+=devolver_temperatura_en_grados();

		}

		temperatura_actual=temperatura_actual/100;

		/*-------------------------------------------------------*/

    	sprintf(stringtemperatura,"%d",temperatura_actual);   // pasa de un entero a un String para imprimir

    	UB_LCD_2x16_Clear();                    //usa una funcion ya definida para limpiar las string
    	UB_LCD_2x16_String(0,0,"Temp actual:");
    	UB_LCD_2x16_String(0,1,stringtemperatura);    // usa una funcion ya definida para imprimir un string
    	UB_LCD_2x16_String(3,1,"\176");
    	UB_LCD_2x16_String(5,1,stringtemperaturadeseada);
    	UB_LCD_2x16_String(9,1,"       ");
    	Delay(750);

    	/* Calcular error*/
    	pid_error = temperatura_deseada-temperatura_actual;


//    	errSum+=(pid_error*0.001);
//    	dErr=(pid_error-lastErr)/0.001;

    	if(pid_error<0)
    	{

    		pid_error=0;   // porque duty no tiene signo (uint32_t)

    	}


    	duty   =   pid_error;   //+   ki*errSum   +  kd*dErr;  // en esta version no hay parametros I y D


    	duty   =  kp  * duty;

//    	lastErr=pid_error;


    	 // Anti sobrecarga del duty
    	if (duty > 100) {duty = 100;}
    	if (duty < 0)   {duty = 0;}


    	duty   = (duty*1000);   // DUTY !!! //pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
    	duty   = (duty/100);

    	TIM_OCInitStructure.TIM_Pulse = duty; // DUTY !!! //pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1

    	// SEGUN LA ZONA SE ENCIENDE 1 2 o 3 PWM

    	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  // ZONA 1

    	if(zona_seleccionada>=2)  // ZONA 2 = ZONA 2 + ZONA 1
    	{
    	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    	}

    	if(zona_seleccionada==3) // ZONA 3 = ZONA 3 + ZONA 2 + ZONA 1
    	{
    	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    	}

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


    	 /*

    	 sprintf(stringduty,"%d",duty);   // pasa de un entero a un String para imprimir


    	 UB_LCD_2x16_Clear();                    //usa una funcion ya definida para limpiar las string
    	 UB_LCD_2x16_String(0,0,"Ciclo de trabajo:");
    	 UB_LCD_2x16_String(0,1,stringduty);
    	 Delay(500); */

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

	if(sensor_numero==1)
	{
	temperatura=adc_leer_cuentas_PC1();   // Lee el ADC1 (PC1) de la funcion adc.h y PHinclude
	}
	if(sensor_numero==2)
	{
	temperatura=adc_leer_cuentas_PC2();   // Lee el ADC2 (PC2) de la funcion adc.h y PHinclude
	}

	temperatura=((temperatura*3000)/4095);  // de Tension de ADC a Grados centigrados

	return temperatura;
}

void iniciarPWM()
{
         /*TIM_TimeBaseStructure.TIM_Prescaler = 1680 - 1;  // 84MHz/1680 = 50kHz
         TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 50KHz/1000 = 50 Hz */

	    //PWM PWM PWM PWM PWM PWM PWM PWM PMW
		/* Time base configuration */
		  TIM_TimeBaseStructure.TIM_Period = 1000 - 1;    // HAY QUE TRABAJAR SOBRE ONDA DE 50Hz
		  TIM_TimeBaseStructure.TIM_Prescaler =1680 - 1 ;
		  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

		  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		  /* PWM1 Mode configuration: Channel1    ---------------------------------------------------------- */
		  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		  TIM_OCInitStructure.TIM_Pulse = 0; // DUTY !!! //pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
		  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

		  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

		  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);


		  /* PWM1 Mode configuration: Channel2    ---------------------------------------------------------- */
		  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		  TIM_OCInitStructure.TIM_Pulse = 0;

		  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

		  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);


		  /* PWM1 Mode configuration: Channel3    -----------------------------------------------------------*/
		  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		  TIM_OCInitStructure.TIM_Pulse = 0;

		  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

		  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

   		  TIM_ARRPreloadConfig(TIM3, ENABLE);

		  /* TIM3 enable counter */
		  TIM_Cmd(TIM3, ENABLE);

		  //PWM PWM PWM PWM PWM PWM PWM PWM PMW
}
