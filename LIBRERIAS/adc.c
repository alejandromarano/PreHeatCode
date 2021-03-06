/**
    @file
    @brief Entrada en el pin PC1 Y PC2

    @author Equipo armcortexm.blogs.upv.es
    @date 2012/11/24
*/

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include "adc.h"
#include <stm32f4xx_adc.h>
#include <stm32f4xx_rcc.h>



/*****************************************************************************/
/**
    @brief Prepara el sistema ADC para leer
    @returns Nada
*/
void adc_inicializar(void) {

    GPIO_InitTypeDef        GPIO_InitStructure;
    ADC_InitTypeDef         ADC_InitStructure;
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
   
    /* Puerto C -------------------------------------------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* PC1 Y PC2 para entrada analógica */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_1|GPIO_Pin_2;  // PC1 PC2
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 

    /* Activar ADC1 ----------------------------------------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 , ENABLE);  // CLOCK PARA ADC1 Y ADC2

    /* ADC Common Init -------------------------------------------------------*/
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode                = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler           = ADC_Prescaler_Div4; // max 36 MHz segun datasheet
    ADC_CommonInitStructure.ADC_DMAAccessMode       = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay    = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC Init ---------------------------------------------------------------*/
    ADC_StructInit (&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution             = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode           = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode     = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge   = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign              = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion        = 1;
    ADC_Init(ADC1, &ADC_InitStructure);  // Aplicar cambios ADC1
    ADC_Init(ADC2, &ADC_InitStructure);   // Aplicar cambios ADC2

    /* Establecer la configuración de conversión ADC1 PC1-------------*/
    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_SetInjectedOffset(ADC1, ADC_InjectedChannel_1, 0);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);

    /* Establecer la configuración de conversión ADC2 PC2-------------*/
    ADC_InjectedSequencerLengthConfig(ADC2, 2);
    ADC_SetInjectedOffset(ADC2, ADC_InjectedChannel_2, 0);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_480Cycles);



    /* Poner en marcha ADC ----------------------------------------------------*/
    ADC_Cmd(ADC1, ENABLE);   
    ADC_Cmd(ADC2, ENABLE);
}

/*****************************************************************************/
/**
    @brief Leer tension
    @returns
*/
int32_t adc_leer_cuentas_PC1(void) {
   
    uint32_t valor_adc;

    ADC_ClearFlag(ADC1,ADC_FLAG_JEOC);      // borrar flag de fin conversion
    
    ADC_SoftwareStartInjectedConv(ADC1);    // inicial conversion

    while (ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC) == RESET); // esperar conversion

    valor_adc = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);

    return valor_adc;

}

int32_t adc_leer_cuentas_PC2(void) {

    uint32_t valor_adc;

    ADC_ClearFlag(ADC2,ADC_FLAG_JEOC);      // borrar flag de fin conversion

    ADC_SoftwareStartInjectedConv(ADC2);    // inicial conversion

    while (ADC_GetFlagStatus(ADC2,ADC_FLAG_JEOC) == RESET); // esperar conversion

    valor_adc = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);

    return valor_adc;

}
/*****************************************************************************/
