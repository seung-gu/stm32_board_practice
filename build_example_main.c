#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"

uint8_t data_buffer_in[10000];
uint8_t data_buffer_in_EP2[EP2_MAX_PACKET_SIZE];

uint32_t EP2_polling_rate = 16;
uint8_t test0 = 12;
const uint8_t test1 = 100;
const int32_t test2 = 999999;
uint32_t test3, test4;
uint32_t test5 = 0;


//The State Machine Instance created here, this will be used right through
EP0_Ex_Status EP0_State_Machine; //This state machine will be used globally
EP2_Ex_Status EP2_State_Machine; //State machine for reports

int main(){
	test3 = 4;
	//test0 = 6;
	//static uint8_t test_static = 6;
	test5++;
    PLL_Init_84MHZ();
    enable_LEDs();
    enable_systick();
    configure_user_button();
	configure_EXTI0_PortA0();
	int32_t some_random_variable = 100;
	some_random_variable++;
    reset_EP0_state_machine();//This is used for both initialisation and reset
    reset_EP2_state_machine(); //This is used for both initialisation and reset
	uint8_t first_report_flag = 0;
//	demo_relocatable();
    while(1){
        //printm("Reached the while loop \n");
    }
    test3 = 1;
    return 0;
}

void demo_relocatable(){
    uint32_t counter = 100;
	for(; counter >= 0; counter--){
		;	
	} 		
}
	
