#ifndef _HAL_UART
#define _HAL_UART

#include "stm32f10x.h"


//**********************************************************************
//                     USART clock enable
#define UART1_CLOCK_ENABLE					(RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#define UART2_CLOCK_ENABLE					(RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define UART3_CLOCK_ENABLE					(RCC->APB1ENR |= RCC_APB1ENR_USART3EN)

//**********************************************************************
//                     USART BAUDRATE
#define UART1_BAUDRATE_9600         (USART1->BRR = 0x1D4C) //for 72MHZ on APB2 bus
#define UART1_BAUDRATE_115200       (USART1->BRR = 0x271) //for 72MHZ on APB2 bus

#define UART2_BAUDRATE_9600         (USART2->BRR = 0xEA6) //for 36MHZ on APB1 bus
#define UART2_BAUDRATE_115200       (USART2->BRR = 0x138) //for 36MHZ on APB1 bus

#define UART3_BAUDRATE_9600         (USART3->BRR = 0xEA6) //for 36MHZ on APB1 bus
#define UART3_BAUDRATE_115200       (USART3->BRR = 0x138) //for 36MHZ on APB1 bus

//**********************************************************************

#define WORD_LENGTH_8BIT            USART_CR1_M
#define STOPBIT											
//#define  

//**********************************************************************
typedef struct
	{
		USART_TypeDef *USART;
			
		uint32_t BaudRate;

	}UART_TYPE ;


//**********************************************************************	
	
//Function Prototypes
//***********************************************************************
//											USART CONFIGURATION
	
static void uart1_init(void);
static void uart2_init(void);
static void uart3_init(void);

//***********************************************************************
//											USART ENABLE

void	uart_init( UART_TYPE uart_type);




#endif

