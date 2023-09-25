/*
  struct_padding.c - Demonstrates memory access through structures of various types
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

  #define STRUCT memory_map_unaligned_packed
// type in debug console : "set output-radix 16" to see the values in hex
  
  STRUCT *EXTRACT_DATA, struct_with_space;
  STRUCT *SPACE_DATA = &struct_with_space;
  SPACE_DATA->first_element = 0xAA;
  SPACE_DATA->second_element = 0xAABB;
  SPACE_DATA->third_element = 0xAABBCCDD;
  SPACE_DATA->fourth_element = 0xAABB;

  EXTRACT_DATA = (STRUCT *) &data_buffer_in;
  uint32_t array_size = sizeof(STRUCT);
  uint32_t first = EXTRACT_DATA->first_element;
  uint32_t second = EXTRACT_DATA->second_element;
  uint32_t third = EXTRACT_DATA->third_element;
  uint32_t fourth = EXTRACT_DATA->fourth_element;
  uint32_t LEDPINSCLR = ~(0xFF << 24);
  return 0;
}

