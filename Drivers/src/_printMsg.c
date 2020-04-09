

#include <stdint.h>
#include "string.h "
#include "stdlib.h "
#include "stdarg.h "
#include "_printMsg.h"





void printMsg_init(void )
{
	 //-----------------------| UART CODE |------------------------------------------
        //USART1 / GPIOA clock enable
 RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN ; 

 
	// PA9 : USART1_TX 
	//PA10 : USART1_RX
 
	//pin configurations: PA9- TX is set to ALternate-push-pull and 50MHz
	GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;
	GPIOA->CRH &= ~(GPIO_CRH_CNF9_0);
	  
	
	//Boad_rate
 //USART DIV value
	//USART1->BRR = 0x1D4C; //for 72MHZ on APB2 bus
	USART1->BRR = 0x271;
	
	
	
	// use Tx mode Only to save power we need to write data on serial not read from it
 //----------   TX enable     UART enable
 USART1->CR1 |= USART_CR1_TE|  USART_CR1_UE;

}


void printMsg(char *msg ,... )
{
	char buff [80];
	int test;
	va_list args ;  //male a list of arguments
	va_start (args, msg);
	
	vsprintf(buff , msg,args); 
	
	for (int i =0 ; i< strlen(buff) ; i++)
	{
		USART1->DR = buff[i];
		while (!(USART1->SR & USART_SR_TXE )     );
	
	}
	va_end(args);

}

