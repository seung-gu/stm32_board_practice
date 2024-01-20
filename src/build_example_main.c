/*
	build_example_main.c - Demonstrate Interrupt handling
	Author - Diptopal
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"

uint8_t data_buffer_in[10000];
uint8_t data_buffer_in_EP2[EP2_MAX_PACKET_SIZE];

uint32_t EP2_polling_rate = 16;
//uint8_t test0 = 12;
const uint8_t test1 = 100;
const int32_t test2 = 999999;
uint32_t test3, test4;
uint32_t test5 = 0;


//The State Machine Instance created here, this will be used right through
EP0_Ex_Status EP0_State_Machine; //This state machine will be used globally
EP2_Ex_Status EP2_State_Machine; //State machine for reports

int main(){
	test3 = 4;
	test5++;
	double_word_padding_disable();
    PLL_Init_84MHZ();
    initialise_FPU();
    enable_LEDs();
    enable_systick();
    configure_user_button();
    configure_EXTI0_PortA0();
	int32_t some_random_variable = 100;
	some_random_variable++;
    reset_EP0_state_machine();//This is used for both initialisation and reset
    reset_EP2_state_machine(); //This is used for both initialisation and reset
	uint32_t first_report_flag = 0;
    float test_float = 23.77689;
    //asm("push {r0}");
    uint16_t force_padding5 = 56;
    test_float *= 2; // To test lazy stacking
    //asm("VLDR.F32 S2, =0x0"); // To test lazy stacking
    //Causes Hard-Fault
    //uint32_t hard_fault = 10;
    //uint32_t div_by_zero = 0;
    //hard_fault /= div_by_zero;
    //Causes Bus Fault
    //uint32_t *BUSFAULT = ((volatile uint32_t *) 0xB0000000);
    //*BUSFAULT = 0xAAFF;
	//demo_relocatable();
    while(1){
        //printm("Reached the while loop \n");
    	/*
    	uint32_t ALLLEDS = 0b1111 << 12; //Orange LED position in ODR

    	uint32_t PA0BUTTON = 0x1 << 0;
    	if (!(GPIOA->IDR & PA0BUTTON)){
    		GPIOD->ODR |= ALLLEDS;
    	}else{
    		GPIOD->ODR &= ~ALLLEDS;
    	}
    	*/
    }
    return 0;
}

void demo_relocatable(){
    uint32_t counter = 100;
	for(; counter >= 0; counter--){
		;	
	} 		
}


