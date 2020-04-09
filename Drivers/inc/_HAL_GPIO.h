#ifndef _HAL_GPIO
#define _HAL_GPIO

#include "stm32f10x.h"

#define LOW  0
#define HIGH 1

//PORT names
#define PORTA				GPIOA
#define PORTB				GPIOB
#define PORTC				GPIOC
#define PORTD				GPIOD
#define PORTE				GPIOE
#define PORTF				GPIOF
#define PORTG				GPIOG

//PIN MODE
#define OUTPUT_MODE 										((uint32_t) 0x01)
#define INPUT_MODE 											((uint32_t) 0x02)

//INPUT MODES TYPE
#define INPUT_ANALOG 										((uint32_t) 0x00)
#define INPUT_FLOATING 									((uint32_t) 0x01)   //default at reset
#define INPUT_PU_PO 										((uint32_t) 0x02)		//input with pull up or pull down

//PULL UP OR PULL DOWN
#define NOPULL													((uint32_t) 0x00)
#define PULLUP													((uint32_t) 0x01)
#define PULLDOWN												((uint32_t) 0x02)

//OUTPUT MODES TYPE
#define OUTPUT_GEN_PURPOSE  						((uint32_t) 0x00)		//General purpose output
#define OUTPUT_OD  											((uint32_t) 0x01)		//output Open-drain
#define OUTPUT_ALT_FUNCTION  						((uint32_t) 0x02)		//Push-pull
#define OUTPUT_ALT_FUNCTION_OD  				((uint32_t) 0x03)		//OPEN drain

//PIN Speeds/Slew Rate
#define SPEED_2MHZ											((uint32_t) 0x02)
#define SPEED_10MHZ											((uint32_t) 0x01)
#define SPEED_50MHZ											((uint32_t) 0x03)


//clock enabling
#define GPIO_CLOCK_ENABLE_ALT_FUNC			(RCC->APB2ENR |= (1<<0))
#define GPIO_CLOCK_ENABLE_PORTA					(RCC->APB2ENR |= (1<<2))
#define GPIO_CLOCK_ENABLE_PORTB					(RCC->APB2ENR |= (1<<3))
#define GPIO_CLOCK_ENABLE_PORTC					(RCC->APB2ENR |= (1<<4))
#define GPIO_CLOCK_ENABLE_PORTD					(RCC->APB2ENR |= (1<<5))
//#define GPIO_CLOCK_ENABLE_PORTE					(RCC->APB2ENR |= (1<<6))

//HIGH BIT POSITIONS FOR CRH REGISTER CNFYG AND MODE
#define CNF_POS_BIT1			(PINPOS[pinNumber] + 2)
#define CNF_POS_BIT2			(PINPOS[pinNumber] + 3)


//Configuation Structon
typedef struct
	{
		GPIO_TypeDef *port;
			
		uint32_t pin;
			
		uint32_t mode;
			
		uint32_t mode_type;
			
		uint32_t pull;
			
		uint32_t speed;
			
		uint32_t alt_func;



	}GPIO_TYPE ;


typedef enum
{
	RISING_EDGE,
	FALLING_EDGE,
	RISING_FALLING_EDGE
}edge_select;

//Function Prototypes
//***********************************************************************
//											GPIO CONFIGURATION
void gpio_init( GPIO_TYPE gpio_type);
void gpio_pull(GPIO_TypeDef *port , uint32_t pinNumber , uint8_t pull);
static void config_pin (GPIO_TypeDef *port , uint32_t pinNumber , uint32_t mode_type);
static void config_pin_speed (GPIO_TypeDef *port , uint32_t pinNumber , uint32_t pinSpeed , uint32_t mode);

//***********************************************************************
//										GPIO USER PIN FUNCTIONS 
int gpio_digital_read(GPIO_TypeDef *port , uint32_t pinNumber );
void gpio_write(GPIO_TypeDef *port , uint32_t pinNumber , uint8_t state);
void gpio_toggle(GPIO_TypeDef *port , uint32_t pinNumber );
//***********************************************************************
//										INTERPUT FUNCTIONS 
void configure_gpio_interrupt(GPIO_TypeDef *port , uint32_t pinNumber , edge_select edge , uint32_t pull);
void enable_gpio_interrupt(uint32_t pinNumber , IRQn_Type irqNumber);	
int check_gpio_pin_interrupt(uint32_t pinNumber);
void clear_gpio_interrupt(uint32_t pinNumber);


#endif
