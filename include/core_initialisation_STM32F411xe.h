


#ifndef __CORE_INITIALISATION_STM32F411XE_H__ //Include guard for this header
#define __CORE_INITIALISATION_STM32F411XE_H__


typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef int int32_t;
typedef short int16_t;
typedef char int8_t;        

/*
//*********************************************************************************
// These 3 macros will convert base and offset values to address.
// This is not done with structures as structures need to be accurately filled and padded
//*********************************************************************************
#define ADDRW(x) (*((volatile unsigned int *)(x)))
#define ADDRH(x) (*((volatile unsigned short *)(x)))
#define ADDRB(x) (*((volatile unsigned char *)(x)))
//**********************************************************************************
*/

#define EP0_MAX_PACKET_SIZE 64
#define EP2_MAX_PACKET_SIZE 64
#define STATE_IDLE 0

/*Declations of Functions found in core_drivers.c*/
void PLL_Init_80MHZ();
void enable_systick();
static void enable_usb_interrupt();
void toggle_LED();
void SysTick_Handler();
void set_systick_timer(uint16_t);
void EXTI0_Handler();
void initialise_FPU();
void initialise_portG2();
void initialise_usb0();
void delay(uint32_t);
void blink_LED();
int printm(const char *, ...);  
void UART0_init();
void double_word_padding_disable();
void PLL_Init_84MHZ();
void enable_LEDs();
void configure_user_button();
void configure_EXTI0_PortA0();
int main();


/*
This structure is a FSM holding the status of execution of a transmit mainly
in between IN transactions. As between IN transactions, the status of
the Tx/Rx buffer, the handler executing it, the status of the entire execution
needs to be saved, this structure is created. This can be used anywhere.
Create a separate instance for each endpoint for convenience.
CHANGE TO uint format below
*/
typedef struct{
    uint32_t EP0_execution_state;
    uint8_t EP_halt_status[2][7]; //2 Rows for Direction
                               //7 columns for EP number
    uint8_t set_address_pending;
    uint8_t device_address;
    uint8_t *data_to_transfer;
    uint8_t *buffer_position;
    uint32_t remaining_data;
    uint8_t  enumeration_status;
}EP0_Ex_Status;

extern EP0_Ex_Status EP0_State_Machine;


/*
EP2 state machine. Will look different from the EP0 state machine
*/
typedef struct{
    uint32_t EP2_execution_state;
    uint32_t set_idle_time;//In milliseconds
    uint32_t idle_counter;
    uint32_t running_counter;
    char *data_to_transfer;
    char *buffer_position;
    uint32_t remaining_data;
}EP2_Ex_Status;

extern EP2_Ex_Status EP2_State_Machine;

void reset_EP0_state_machine();
void reset_EP2_state_machine();

//adding a comment here    


#endif
