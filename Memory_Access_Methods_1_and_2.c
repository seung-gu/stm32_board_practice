/*
	Memory_Access_Methods_1_and_2.c - Various Memory Access Methods
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



int main(){
    PLL_Init_84MHZ();
    /*Uncomment whichever section you wish to use 
	  Uncomment any one Method at a time
	*/
    //////////// Method 1 /////////////////////////////////
	
    #define ARRAY_BASE ((uint8_t *) &data_buffer_in)
	#define ADDRW(x) (*((volatile unsigned int *)(x)))
	#define ADDRH(x) (*((volatile unsigned short *)(x)))
	#define ADDRB(x) (*((volatile unsigned char *)(x)))
    #define zeroth_offset 0
    #define second_offset 2
    #define third_offset 3
    #define fourth_offset 4
    #define eighth_offset 8
    uint32_t first = ADDRW(ARRAY_BASE);
	uint32_t second = ADDRH(ARRAY_BASE+fourth_offset);
	uint32_t third = ADDRB(ARRAY_BASE+eighth_offset);
	uint32_t fourth = ADDRH(ARRAY_BASE+zeroth_offset);
	
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

