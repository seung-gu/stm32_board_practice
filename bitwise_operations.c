/*Contains code for bitwise operations
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
    #define SYSTICK_RW 0xE000E010
    //SysTick_baseaddress * SYSTICK_BASE	=	(SysTick_baseaddress *) SYSTICK_RW;
    #define SYSTICK_BASE ((SysTick_baseaddress *) SYSTICK_RW)
    uint32_t value1 = SYSTICK_BASE->LOAD;
    SYSTICK_BASE->LOAD = 0xFFFFFF;
    uint32_t CLEARBITS = 0b11111111 << 16 ;
    SYSTICK_BASE->LOAD &= ~CLEARBITS;
    uint32_t BITPATTERN = 0b01010101 << 16;
    SYSTICK_BASE->LOAD |= BITPATTERN;
    value1 = SYSTICK_BASE->LOAD;
    uint32_t BIT23CHECK = 0x1 << 23;
    while(!(SYSTICK_BASE->LOAD & BIT23CHECK)){};
    return 0;
}



