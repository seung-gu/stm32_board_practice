/*
	Memory_Access_Methods_1_and_2.c - Various Memory Access Methods
	Author - Diptopal
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"


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
	uint16_t first_element;
	uint16_t second_element;
	uint32_t third_element;
	uint32_t fourth_element;
} memory_map_aligned;


typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile const uint32_t CALIB;
}My_SysTick_Type;


int main(){
    PLL_Init_84MHZ();

    /*
    //////////// Method 1 /////////////////////////////////
    #define SCS_BASE        0xE000E000UL   
    #define STK_CTRL        (SCS_BASE + 0x0010UL)  
    #define STK_LOAD        (SCS_BASE + 0x0014UL)
    #define STK_VAL         (SCS_BASE + 0x0018UL)
    #define STK_CALIB       (SCS_BASE + 0x001CUL)
    #define ADDRW(x)        (*((volatile unsigned int *)(x))) 
    #define ADDRH(x)        (*((volatile unsigned short *)(x)))
    #define ADDRB(x)        (*((volatile unsigned char *)(x)))
    #define CTRL(x)         0x0000
    
    uint32_t value1 = ADDRH(STK_CTRL);
    uint32_t value2 = ADDRW(STK_LOAD);
    ADDRH(STK_CTRL) = 5;
    ADDRW(STK_LOAD) = 0x000000FF;
    value1 = ADDRH(STK_CTRL);
    value2 = ADDRW(STK_LOAD);
	*/
   
    ///////////// Method 2 ////////////////////////////////
    //memory_map_aligned * EXTRACT_DATA;
    //EXTRACT_DATA = (memory_map_aligned *) &data_buffer_in;
    #define SysTick_BASE 0xE000E010UL
    #define SysTick ((My_SysTick_Type *) SysTick_BASE)

    uint32_t value1 = SysTick->CTRL;
    SysTick->CTRL = 5;
    uint32_t value2 = SysTick->LOAD;
    SysTick->LOAD = 0x000000FF;

    return 0;
}

