/*
  core_initialisation_STM32F411xe.c - Core initialisation for STM32F411xe
  Author - Diptopal
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"
      
uint32_t just_a_random_variable;   


void PLL_Init_84MHZ(){// Initialising PLL for 80Mhz operation, please refer section 6.3 of stm32f411xc
	                  //reference manual external crystal is 8Mhz connected to HSE
	uint32_t latency = 0x2 << 0; 
	FLASH->ACR |= latency;      // Setting Flash memory wait state to 2WS, the Vdd is 3V 
	                            // discovery board user manual
	uint32_t HSEON = 0x1 << 16;          // 17th bit in the CR register
    RCC->CR |= HSEON;           // We are using HSE as the PLL clock source, hence let's enable HSE
	                            // first, in the next step we will change the PLL clock source to HSE
	uint32_t HSEREADY = 0x1 << 17;       // 17th bit in the CR resister
    while(!(RCC->CR & HSEREADY)){};// Waiting for HSE clock to stabilise
    uint32_t PLLP = 0x1 << 16;   //This is targeting the VCO of 240Mhz -> 0x1 means 4 here 
	uint32_t PLLQ = 0x7 << 24;   //This is targeting 48Mhz for USB clock
	uint32_t PLLM = 0x2 << 0;    //This is assuming PLL input from HSE to be 8Mhz
	uint32_t PLLN = 0x54 << 6;  // This is assuming PLL input from HSE to be 8Mhz
    RCC->PLLCFGR &= ~0xF437FFF; // Clearing the register as it has a non zero reset value
	                             // and we will need to set all the values afresh
                                 //f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
                                 //f(PLL general clock output) = f(VCO clock) / PLLP
                                 //f(USB OTG FS, SDIO) = f(VCO clock) / PLLQ
	RCC->PLLCFGR |= PLLP | PLLQ | PLLM | PLLN ; // Writing the values in the PLLCFGR register
	uint32_t PLLSRC = 0x1 << 22;           //This is where we set up the bit for the PLL to choose the 
	                                       //HSE clock
	RCC->PLLCFGR |= PLLSRC;                //We select the HSE clock
	uint32_t PPRE2 = 0b000 << 13;            //Value for APB2 prescalar, we are not touching this and 
	                                         //letting the PLL output clock frequency be the APB2 frequency
	uint32_t PPRE1 = 0b100 << 10;          //letting the APB1 frequency be half of the clock output frequency
	uint32_t HPRE  = 0b0000 << 4;           // Letting the AHB frequency be the same as the PLL frequency
	RCC->CFGR |= PPRE2 | PPRE1 | HPRE; //Setting the prescalars here 
	uint32_t PLLON = 0x1 << 24;             //Bit to switch on the PLL in CR
	RCC->CR |= PLLON;              //Switch on the PLL
	uint32_t PLLRDY = 0x1 << 25;            //Bit to check if PLL is ready
	while(!(RCC->CR & PLLRDY)){};  //Wait until PLL is locked
	uint32_t SWPLL = 0b10 << 0;             //Bit to Switch the MCU clock source to PLL
	RCC->CFGR |= SWPLL;            //Switch MCU clock source to PLL
	uint32_t SWST = 0b10 << 2;              //Bit to check if the MCU clock source is indeed switched to PLL
	while(!(RCC->CFGR & SWST)){};  //Checking if the MCU clock has switched to PLL
	uint32_t HSION = 0x1 << 0 ;             //Bit to control HSI
	RCC->CR &= ~HSION;             //Turn off HSI. Done in the end because so far we were running off the 
	                               // HSI. Only after we became sure of the MCU using PLL as clock, we could
	                               // switch off HSI
}

void delay(uint32_t value) {
        while (value--){;}
  }

void enable_systick(){
	//blink_LED(10);
    uint32_t CLK_SRC = 0x0 << 2;
    uint32_t INTEN = 0x1 << 1;
    uint32_t ENABLE = 0x1 << 0;
    uint32_t RELOAD = 0x280DE7 ;
    //0x280DE7
    //uint32_t RELOAD = 0x7A1200 ; // For 10Hz tick frequency with 80 Mhz clock and 2Hz for a 16Mhz clock
    //uint32_t RELOAD = 0x4C4B400 ; // The counter is a 24 bit field, this value overflows
    uint32_t SYSTICK_PRI = 0x4 << 4; // SysTick Priority left shift by 4 as upper 4 bits are relevant
    asm volatile("cpsid i"); //Disabling all interrupts with configurable priority
    /*Enabling SysTick interrupt and setting a priority of 2
    The SYSTICK interrupt registers are different from the conventional interrupt config registers
    Enabling the interrupts here instead of a separate function since the interrupt registers are common with systick*/
    SCB->SHP[11] = SYSTICK_PRI; //This is a byte access
    SysTick->LOAD = RELOAD ; 
    SysTick->CTRL &= ~(0b111);
    SysTick->CTRL |= ENABLE | INTEN | CLK_SRC;
    asm volatile("cpsie i"); //Enabling all interrupts with configurable priority
 }

void configure_EXTI0_PortA0(){
	asm volatile("cpsid i"); //Disabling all interrupts with configurable priority
	uint32_t EXTICR1A0 = 0xF << 0; //PA0 is configured to accept EXTI0 interrupt
	SYSCFG->EXTICR[0] &= ~EXTICR1A0; // Clearing the INTI0 field of EXTICR1
	                                 // Nothing more to be done here PA0 is now
	                                 // linked to INTI0
	uint32_t ENINTL0 = 0x1 << 0;     // Configure interrupt line enable 0 bit
	EXTI->IMR |= ENINTL0;            // Enable interrupt line 0
    uint32_t RISTRIGINTI0 = 0x1 << 0; // Configure rising trigger variable for INTI0
                                      // the button is active high
    EXTI->RTSR |= RISTRIGINTI0;       // Enable interrupt on rising trigger
    uint32_t NVICINTI0EN = 0x1 << 6;  // Configure enable bit variable in NVIC for EXTII0
    NVIC->ISER[0] |= NVICINTI0EN;     // Enabling ISER0 in NVIC


    uint8_t INTI0PRIO = 0x3 << 4;     // Setting the INTI0 priority variable
                                      // left shift by 4 bytes as upper 4 bits are used
    NVIC->IP[6] &= (uint8_t)~0xFF;    // Clearing the NVIC INTI0 priority field
    NVIC->IP[6] |=  INTI0PRIO;        // Setting the interrupt priority here
    asm volatile("cpsie i");          //Enabling all interrupts with configurable priority
}

void configure_user_button(){
	uint32_t GPIOACLOCK = 0x1 << 0; //GPIO A position
	RCC->AHB1ENR |= GPIOACLOCK; /* enable GPIOA clock */
	uint32_t BUTTONPINSCLR = ~(0b11 << 0); //These bits clear the mode for the port A0 pin
	GPIOA->MODER &= BUTTONPINSCLR; /* clear pin mode */

	uint32_t BUTTONPINSMODE = 0b00 << 0; //Configiration to set Port A0 as input port
	                                     //Actually this is set by default but just keeping it here
	GPIOA->MODER |= BUTTONPINSMODE;      // Set Port A0 as input port
	uint32_t ACTIVELOW = 0x2;
	GPIOA->PUPDR |= ACTIVELOW << 0; // Setting pin to active low here
}

void enable_LEDs(){
   uint32_t GPIODCLOCK = 0x1 << 3 ; //GPIO D position	
   RCC->AHB1ENR |= GPIODCLOCK; /* enable GPIOD clock */
   uint32_t LEDPINSCLR = ~(0xFF << 24); //These bits clear the mode of all 4 LEDs 	
   GPIOD->MODER &= LEDPINSCLR; /* clear pin mode */
   uint32_t LEDPINSMODE = 0b01000000 << 24; //Set blue LED port as output port PD15
   GPIOD->MODER |= LEDPINSMODE; /* set pins to output mode */    		
}

void blink_LED(uint32_t duration){
    
}

 void toggle_LED(){
    //Toggles all LEDs
	uint32_t ALLLEDS = 0b1111 << 12; //Blue LED position in ODR
    GPIOD->ODR ^= ALLLEDS; // Toggle the LED   
 }


 void SysTick_Handler(){
	toggle_LED();
	//float test_float_isr = 23.77689;
	//test_float_isr *= 3; // To test lazy stacking
    //delay(0xFFFFFF);
    //NVIC->ISPR[0] |= 0x1 << 6; //Set pending status of EXTII0 through software
    //NVIC->STIR = 0x6; //Set pending status of EXTI0 through software using the STIR
    asm("isb"); // Memory barrier for pipeline flush, so that no other instruction
                  // is executed in the pipeline before the interrupt begins executing
 }

 void EXTI0_Handler(){
	static uint32_t position = 0;
	position = position % 4;
	uint32_t LEDPINSCLR = ~(0xFF << 24); //These bits clear the mode of all 4 LEDs
	GPIOD->MODER &= LEDPINSCLR;           /* clear pin mode */
	uint32_t LEDPINSMODE = 0x1 << (24 + (position * 2)); //Set orange LED port as output port PD13
	                                        // So now the systick will glow the orange LED
	position++;
	GPIOD->MODER |= LEDPINSMODE;            /* set pins to output mode */
	uint32_t CLEARLINEPEND = 0x1 << 0;    //Clear pending on the external interrupt line
                                          // This is unique to the external interrupts that
	                                      // there is a pending bit even outside NVIC
	EXTI->PR |= CLEARLINEPEND;
 }

 void initialise_FPU(){
	 uint32_t ENABLECP10CP11 = 0xF << 20;
	 SCB->CPACR |= ENABLECP10CP11;
	 asm("DSB");
	 asm("ISB");

   }


void double_word_padding_disable(){
	//Clearing the Stack Align Bit to Disable Double Word Padding
	uint32_t STACKALIGNMASK = 0x1 << 9;
	SCB->CCR &= ~STACKALIGNMASK;
}




/*
Resets the EP0 state machine. Sets the execution states to idle
and sets all remaining values to zero.
*/
void reset_EP0_state_machine(){
    uint8_t row, column;
    EP0_State_Machine.EP0_execution_state = STATE_IDLE;
    for(row = 0 ; row < 2 ; row++){
        for(column = 0 ; column < 7 ; column++){
            EP0_State_Machine.EP_halt_status[row][column] = 0;
        }
    }
    EP0_State_Machine.data_to_transfer = 0;
    EP0_State_Machine.buffer_position = 0;
    EP0_State_Machine.remaining_data = 0;
    EP0_State_Machine.set_address_pending = 0;
    EP0_State_Machine.enumeration_status = 0;

}

/*
Resets the EP2 state machine. Sets the execution states to idle
and sets all remaining values to zero.
*/
void reset_EP2_state_machine(){
    EP2_State_Machine.EP2_execution_state = STATE_IDLE;
    EP2_State_Machine.set_idle_time = 0;//In milliseconds
    EP2_State_Machine.idle_counter = 0;
    EP2_State_Machine.running_counter = 0;
    EP2_State_Machine.data_to_transfer = 0;
    EP2_State_Machine.buffer_position = 0; //Not sure where to point it, yet to find out
    EP2_State_Machine.remaining_data = 0;
}


