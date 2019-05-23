#include"stm8s.h"

void delay_ms(u32 nms)
{
while(nms--);
}
static void CLK_Config(void)
{
    /* Initialization of the clock */
    /* Clock divider to HSI/1 */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

  void UART1_Config(void)
 {
   /* EVAL COM (UART) configuration -----------------------------------------*/
   /* USART configured as follow:
         - BaudRate = 115200 baud  
         - Word Length = 8 Bits
         - One Stop Bit
         - Odd parity
         - Receive and transmit enabled
         - UART Clock disabled
   */
   UART1_DeInit();
   
   
   UART1_Init((uint32_t)9600, UART1_WORDLENGTH_8D,UART1_STOPBITS_1, UART1_PARITY_NO,
                    UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
 
   /* Enable the UART Receive interrupt: this interrupt is generated when the UART
     receive data register is not empty */
   UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
   
   /* Enable the UART Transmit complete interrupt: this interrupt is generated 
      when the UART transmit Shift Register is empty */
   UART1_ITConfig(UART1_IT_TXE, ENABLE);
 
   /* Enable UART */
   UART1_Cmd(ENABLE);
   
     /* Enable general interrupts */
   enableInterrupts();
 }

void main(void)
{

    GPIO_Init(GPIOD,GPIO_PIN_0,GPIO_MODE_OUT_PP_LOW_FAST);

    CLK_Config();
    UART1_Config();

    while(1)
    {

      if(UART1_ReceiveData8()  != 0)
        GPIO_WriteReverse(GPIOD,GPIO_PIN_0);
     delay_ms(60000);
    }
  
}

void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
