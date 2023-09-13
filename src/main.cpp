#include "stm32f4xx.h"

int main(){
  uint32_t timer_count = 2000000;

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enable clock for GPIO port D

  GPIOD->MODER |= GPIO_MODER_MODER12_0; // Set PD12 as output
  GPIOD->MODER |= GPIO_MODER_MODER13_0; // Set PD13 as output
  GPIOD->MODER |= GPIO_MODER_MODER14_0; // Set PD14 as output
  GPIOD->MODER |= GPIO_MODER_MODER15_0; // Set PD15 as output

  while(1){
    //Turn on LEDs
    GPIOD->BSRR = 1<<12; //Set the BSRR bit 12 to 1 to turn on the LED
    for(uint32_t i=0;i<timer_count;i++){};
    GPIOD->BSRR = 1<<13; //Set the BSRR bit 13 to 1 to turn on the LED
    for(uint32_t i=0;i<timer_count;i++){};
    GPIOD->BSRR = 1<<14; //Set the BSRR bit 14 to 1 to turn on the LED
    for(uint32_t i=0;i<timer_count;i++){};
    GPIOD->BSRR = 1<<15; //Set the BSRR bit 15 to 1 to turn on the LED
    for(uint32_t i=0;i<timer_count;i++){};

    //Turn off LEDs
    GPIOD->BSRR = 1<<(12+16); //Set the BSRR bit 12 to 1 to turn off the LED
    for(uint32_t i=0;i<timer_count;i++){};
    GPIOD->BSRR = 1<<(13+16); //Set the BSRR bit 13 to 1 to turn off the LED
    for(uint32_t i=0;i<timer_count;i++){};
    GPIOD->BSRR = 1<<(14+16); //Set the BSRR bit 14 to 1 to turn off the LED
    for(uint32_t i=0;i<timer_count;i++){};
    GPIOD->BSRR = 1<<(15+16); //Set the BSRR bit 15 to 1 to turn off the LED
    for(uint32_t i=0;i<timer_count;i++){};
  }
}