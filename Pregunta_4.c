#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// PUNTEROS DEL ADC
#define SENS_SAR_ATTEN1_REG ((volatile uint32_t)0x3FF48834)
#define SENS_SAR_READ_CTRL_REG ((volatile uint32_t)0x3FF48800)
#define SENS_SAR_START_FORCE_REG ((volatile uint32_t)0x3FF4882C)
#define SENS_SAR_MEAS_START1_REG ((volatile uint32_t)0x3FF48854)

// PUERTOS
#define BTN_INC 5
#define BTN_DEC 4
#define LEDR 27
#define LEDG 26
#define LEDB 25

uint32_t valores[10];
uint16_t index_valor = 0;
uint16_t valor_actual = 2;
uint32_t valor_adc = 0;
uint8_t flag = 0;
uint8_t bandera =0;
uint32_t valor_auxiliar = 0;
uint32_t promedio_arit = 0;

void adc_config(void){

  SENS_SAR_ATTEN1_REG |= (0x3 << 12);

  SENS_SAR_READ_CTRL_REG |= (1 << 27);
  SENS_SAR_READ_CTRL_REG |= (0x3 << 16);

  SENS_SAR_START_FORCE_REG |= (0x3 << 0);
}

uint32_t adc_leer (){

  SENS_SAR_MEAS_START1_REG |= (1 << 31) | (1 << 25) | (1 << 18) | (1 << 17);
  
  uint32_t timeout = 100000000;
  while((SENS_SAR_MEAS_START1_REG & (1 << 16)) == 0)
  {
    timeout--;
    if(timeout == 0)
    {
      return -1;
    }
  }

  return (uint32_t)(SENS_SAR_MEAS_START1_REG & 0xFFFF);
}

void ejecutar_secuencia() {

  if(valor_actual <= 10)
  {
    gpio_set_level(LEDR, 1);
  }
  else if(valor_actual <= 16)
  {
    gpio_set_level(LEDB, 1);
  }
  else
  {
    gpio_set_level(LEDG, 1);
  }

  const TickType_t xDelay = (1000*valor_actual) / portTICK_PERIOD_MS;

  for(int i = 0; i < 10; i++)
  {
    valor_adc = adc_leer();
    if(valor_adc < 0)
    {
      printf("ERROR EN EL ADC1 \n");
    }
    else
    {
      valores[i] = valor_adc;
    }
    vTaskDelay(xDelay);
  }

  for(int i = 0; i < 10; i++)
  {
    valor_auxiliar = valor_auxiliar + valores[i];
  }

  promedio_arit = valor_auxiliar / 10;

  gpio_set_level(LEDR, 0);
  gpio_set_level(LEDG, 0);
  gpio_set_level(LEDB, 0);
}

void app_main() {

  adc_config();

  gpio_config_t output_led = {
    .pin_bit_mask = (1 << LEDR) | (1 << LEDG) | (1 << LEDB),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };

  gpio_config(&output_led);

  gpio_config_t input_btn = {
    .pin_bit_mask = (1 << BTN_INC) | (1 << BTN_DEC),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };

  gpio_config(&input_btn);

  gpio_set_level(LEDR, 0);
  gpio_set_level(LEDG, 0);
  gpio_set_level(LEDB, 0);

  while(1) {
    if(gpio_get_level(BTN_INC) == 0)
    {
      vTaskDelay(50/portTICK_PERIOD_MS);
      while(gpio_get_level(BTN_INC) == 0){
        vTaskDelay(50/portTICK_PERIOD_MS);
      }
      if(valor_actual < 20)
      {
        valor_actual = valor_actual + 2;
      }
      else
      {
        valor_actual = 20;
      }
      bandera = 0;
      vTaskDelay(50/portTICK_PERIOD_MS);
    }

    if(gpio_get_level(BTN_DEC) == 0)
    {
      vTaskDelay(50/portTICK_PERIOD_MS);
      while(gpio_get_level(BTN_DEC) == 0){
        vTaskDelay(50/portTICK_PERIOD_MS);
      }
      if(valor_actual > 2)
      {
        valor_actual = valor_actual - 2;
      }
      else
      {
        valor_actual = 2;
      }
      bandera = 0;
      vTaskDelay(50/portTICK_PERIOD_MS);
    }
    
    while(1)
    {
      if(gpio_get_level(BTN_DEC) == 0 || gpio_get_level(BTN_INC) == 0)
      {
        break;
      }
      if(bandera == 100)
      {
        flag = 1;
        break;
      }
      vTaskDelay(50/portTICK_PERIOD_MS);
      bandera++;
    }

    if(flag == 1)
    {
      ejecutar_secuencia();
      flag = 0;
      bandera = 0;
      printf("Promedio Obtenido: %"PRIu32"\n", promedio_arit);
      printf("Tiempo de muestro: %"PRIu16"\n", valor_actual);

    }
  }
}
