#include "_HAL_UART.h"
#include <stdint.h>

//**********************************************************************
//                        UART_INIT
void uart_init( UART_TYPE uart_type)
{
	if(uart_type.USART == USART1)
	{
		//   UART_INIT _1
		uart1_init();
		if(uart_type.BaudRate == 9600)
			UART1_BAUDRATE_9600;
		else if (uart_type.BaudRate == 115200)
			UART1_BAUDRATE_115200;
		
		
	}
	
	if(uart_type.USART == USART2)
	{
		//   UART_INIT _2
		uart2_init();
		if(uart_type.BaudRate == 9600)
			UART2_BAUDRATE_9600;
		else if (uart_type.BaudRate == 115200)
			UART2_BAUDRATE_115200;
		
		
	}
	
	if(uart_type.USART == USART3)
		{
				//   UART_INIT _3
		uart3_init();
		if(uart_type.BaudRate == 9600)
			UART3_BAUDRATE_9600;
		else if (uart_type.BaudRate == 115200)
			UART3_BAUDRATE_115200;
		
	
	}
	
		

}

//**********************************************************************
//                              USART1
static void uart1_init(void) //PA9- TX  PA10- RX
{
	
	
	// UART1 clock enable
	UART1_CLOCK_ENABLE;
	
	// GPIOA clock enable
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	
	//pin configurations: PA9- TX is set to ALternate-push-pull and 50MHz
	GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;
	GPIOA->CRH &= ~(GPIO_CRH_CNF9_0);
	
 //PIN PA10 is the RX pin and it has to be set to input and FLOATING
	GPIOA->CRH &= ~( GPIO_CRH_MODE10 | GPIO_CRH_CNF10_0 );
	GPIOA->CRH |= (GPIO_CRH_CNF10_0);
	
	
	
	
	//enable RXNE and TXE interrupts on USART SIDE
//	USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_TXEIE;
	USART1->CR1 |= USART_CR1_RXNEIE ; 
 //----------|  RX enable        TX enable     UART enable
 USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

 //ENABLE interrupt for USART1 on NVIC side
 NVIC_EnableIRQ(USART1_IRQn);
	
}

//**********************************************************************
//                     USART2
static void uart2_init(void) //PA2- TX  PA3- RX
{
	// GPIOA clock enable
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	//pin configurations: PA2- TX is set to ALternate-push-pull and 50MHz
	GPIOA->CRL |= GPIO_CRL_MODE2 | GPIO_CRL_CNF2_1;
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0);
	
 //PIN PA3 is the RX pin and it has to be set to input and FLOATING
	GPIOA->CRL &= ~( GPIO_CRL_MODE3 | GPIO_CRL_CNF3_0 );
	GPIOA->CRL |= (GPIO_CRL_CNF3_0);
	
	
	// UART2 clock enable
	UART2_CLOCK_ENABLE;
	
	
	//enable RXNE and TXE interrupts on USART SIDE
 //USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_TXEIE;

 //----------|  RX enable        TX enable     UART enable
 USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

 //ENABLE interrupt for USART1 on NVIC side
// NVIC_EnableIRQ(USART2_IRQn);
	
}


//**********************************************************************
//                        USART3
static void uart3_init(void) //PB10- TX  PB11- RX
{
	// GPIOB clock enable
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	
	//pin configurations: PB10- TX is set to ALternate-push-pull and 50MHz
	GPIOB->CRL |= GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1;
	GPIOB->CRL &= ~(GPIO_CRH_CNF10_0);
	
 //PIN PB11 is the RX pin and it has to be set to input and FLOATING
	GPIOB->CRH &= ~( GPIO_CRH_MODE11 | GPIO_CRH_CNF11_0 );
	GPIOB->CRH |= (GPIO_CRH_CNF11_0);
	
	
	// UART3 clock enable
	UART3_CLOCK_ENABLE ;
	
	
	//enable RXNE and TXE interrupts on USART SIDE
 USART3->CR1 |= USART_CR1_RXNEIE | USART_CR1_TXEIE;

 //----------|  RX enable        TX enable     UART enable
 USART3->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

 //ENABLE interrupt for USART1 on NVIC side
 NVIC_EnableIRQ(USART3_IRQn);
	
}


//**********************************************************************
/*
void USART1_IRQHandler(void) {

 //check if we are here because of RXNE interrupt
 if (USART1->SR & USART_SR_RXNE) //if RX is not empty
 {
  char temp = USART1->DR; //fetch the data received
  USART1->DR = temp;  //send it back out
  while (!(USART1->SR & USART_SR_TC))
   ;

 }

 //check if we are here because of TXEIE interrupt
 if (USART1->SR & USART_SR_TXE) //if RX is not empty
 {
  //handle transmit completion here

 }

}*/

