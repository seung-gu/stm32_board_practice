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
   
    ///////////// Method 2 CMSIS ////////////////////////////
    #define SysTick_BASE 0xE000E010UL
    #define SysTick ((My_SysTick_Type *) SysTick_BASE)

    SysTick->LOAD = 0x0;
    uint32_t set4thbit = 0x1 << 3;
    SysTick->LOAD |= set4thbit;
    
    // save values in registers
    SysTick->LOAD = 0xFFFFFF; // = 0x00FFFFFF, 24~31 bits are reserved (read only)
    uint32_t clear_bits = 0xFF << 16;
    SysTick->LOAD &= ~clear_bits;
    uint32_t bit_pattern = 0b01010101 << 16;
    SysTick->LOAD |= bit_pattern;

    // read values from registers
    uint32_t bit23check = 0x1 << 22;
    while(!(SysTick->LOAD & bit23check)){};

    // pointer casting test examples
    uint32_t words[4] = {0x12345678, 0x87654321, 0xABCDEF12, 0x12345678};
    uint32_t *ptr = words;
    *++ptr = 0x0; // 0x87654321 -> 0x00000000
    // 16 bit 로 캐스팅 시, +1 할때, 0x12345678 -> 0x87654321로 가는게 아닌
    // 0x1234 -> 0x5678로 가는 것을 확인할 수 있다. 16 bit 마다 접근하고 값 읽음
    uint16_t *ptr16 = (uint16_t *) words; 
    *++ptr16 = 0x0; // 0x12345678 -> 0x00005678

    uint16_t words16[4] = {0x1234, 0x5678, 0xABCD, 0xEF12};
    uint32_t *ptr32 = (uint32_t *) words16;
    *++ptr32 = 0x0; // 0xABCD, 0xEF12 -> 0x0000 0000
    return 0;
}

