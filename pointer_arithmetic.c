/*
	pointer_arithmetic.c - Demonstrates Pointer Arithmetic
	Author - Diptopal
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"

uint8_t data_buffer_in[16] = {
  0xAA,
  0xBB,
  0xCC,
  0xDD,
  0xEE,
  0xFF,
  0x11,
  0x22,
  0x33,
  0x44,
  0x55,
  0x66,
  0x77,
  0x88,
  0x99,
  0x10
};



typedef struct
{
	uint8_t  first_element;
	//One byte padded here
	uint16_t second_element;
	uint32_t third_element;
	uint16_t fourth_element;
} memory_map_unaligned_not_packed;


typedef struct
{
	uint8_t  first_element;
	uint16_t second_element;
	uint32_t third_element;
	uint16_t fourth_element;
} __attribute__ ((packed))  memory_map_unaligned_packed;


typedef struct
{
	uint16_t  first_element;
	uint16_t second_element;
	uint32_t third_element;
	uint32_t fourth_element;
} memory_map_aligned;

typedef struct
{
  volatile uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  volatile uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  volatile uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  volatile const  uint32_t CALIB;           /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_baseaddress;



int main(){
    PLL_Init_84MHZ();
    uint8_t word = 67;
    uint16_t * pointer = (uint16_t *)&word;
    pointer += 4;
    pointer--;
    return 0;
}



