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



int main(){
    PLL_Init_84MHZ();
    /*Uncomment whichever section you wish to use 
	  Uncomment any one Method at a time
	*/
    //////////// Method 1 /////////////////////////////////
    #define SCS_BASE        0xE000E000UL   
    #define STK_CTRL        (SCS_BASE +  0x0010UL)  
    #define STK_LOAD        (SCS_BASE +  0x0014UL)
    #define STK_VAL         (SCS_BASE +  0x0018UL)
    #define STK_CALIB       (SCS_BASE +  0x001CUL)

    #define ADDRW(x)        (*((volatile unsigned int *)(x))) 
    #define ADDRH(x)        (*((volatile unsigned short *)(x)))
    #define ADDRB(x)        (*((volatile unsigned char *)(x)))
    #define CTRL(x)         0x0000

    uint32_t value1 = ADDRH(STK_CTRL);
    ADDRH(STK_CTRL) = 5;
    value1 = ADDRH(STK_CTRL);
	
    /*
    ///////////// Method 2 ////////////////////////////////
    //memory_map_aligned * EXTRACT_DATA;
    //EXTRACT_DATA = (memory_map_aligned *) &data_buffer_in;
    #define EXTRACT_DATA       ((memory_map_unaligned_packed *) &data_buffer_in)
    uint32_t array_size = sizeof(memory_map_unaligned_packed);
    uint32_t first = EXTRACT_DATA->first_element;
    uint32_t second = EXTRACT_DATA->second_element;
    uint32_t third = EXTRACT_DATA->third_element;
    uint32_t fourth = EXTRACT_DATA->fourth_element;
    */
    return 0;
}

