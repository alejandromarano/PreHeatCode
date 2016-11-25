#include "stm32f4xx_gpio.h"
#include "teclado.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"


int retardoFila=0;

void Init_Teclado()
{
	GPIO_InitTypeDef GPIO_InitStructure; //Estructura de configuracion

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	//Habilitacion de la senal de reloj para el periferico GPIOD
		//Enmascaramos los pines que usaremos como FILA

	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_2 |	GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;		//Los pines seleccionados como salida
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;		//Tipo de salida como push pull
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_25MHz;		//Velocidad del clock para el GPIO
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;		//Sin pull ups
	GPIO_Init(GPIOE, &GPIO_InitStructure); //Se aplica la configuracion definidas anteriormente


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//Enmascaramos los pines que usaremos como COLUMNAS
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0 | GPIO_Pin_1 |	GPIO_Pin_3 | GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN;		//Los pines seleccionados como entrada
		GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;		//Tipo de entrada push pull
		GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_25MHz;		//Velocidad del clock para el GPIO
		GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_DOWN;		//con pull down
		GPIO_Init(GPIOD, &GPIO_InitStructure); //Se aplica la configuracion definidas anteriormente

}

int Leer_Teclado(void)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_0);

	/* Set filas */
	if (retardoFila==0){
	GPIO_SetBits(GPIOE, GPIO_Pin_4);
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)==1){retardoFila=1; return 1;}//Tecla 1
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)==1){retardoFila=1; return 2;}//Tecla 2
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1)==1){retardoFila=1; return 3;}//Tecla 3
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0)==1){retardoFila=1; return 12;}//Tecla A
	retardoFila=1;
	GPIO_ResetBits(GPIOE, GPIO_Pin_4);
	return 16;
	}

	if (retardoFila==1){
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)==1){retardoFila=2; return 4;}//Tecla 4
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)==1){retardoFila=2; return 5;}//Tecla 5
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1)==1){retardoFila=2; return 6;}//Tecla 6
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0)==1){retardoFila=2; return 13;}//Tecla B
	retardoFila=2;
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
	return 16;
	}

	if (retardoFila==2){
	GPIO_SetBits(GPIOE, GPIO_Pin_2);
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)==1){retardoFila=3; return 7;}//Tecla 7
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)==1){retardoFila=3; return 8;}//Tecla 8
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1)==1){retardoFila=3; return 9;}//Tecla 9
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0)==1){retardoFila=3; return 14;}//Tecla C
	retardoFila=3;
	GPIO_ResetBits(GPIOE, GPIO_Pin_2);
	return 16;
	}

	if (retardoFila==3){
	GPIO_SetBits(GPIOE, GPIO_Pin_0);
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)==1){retardoFila=0; return 10;}//Tecla Asterisco
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)==1){retardoFila=0; return 0;}//Tecla 0
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1)==1){retardoFila=0; return 11;}//Tecla Numeral
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0)==1){retardoFila=0; return 15;}//Tecla D
	retardoFila=0;
	GPIO_ResetBits(GPIOE, GPIO_Pin_0);
	return 16;
	}

	return 16;
}


int armarEntero( int ingresadas[], int cantidad )
{
	int entero=0, mult=1000, i=0;
	if (cantidad==4)
	for (i=0; i<cantidad; i++)
	{
		entero+=ingresadas[i]*mult;
		mult=mult/10;
	}
	else
	for(i=2; i<cantidad;i++)
	{
		entero+=ingresadas[i]*mult;
		mult=mult/10;
	}
	return entero;
}
