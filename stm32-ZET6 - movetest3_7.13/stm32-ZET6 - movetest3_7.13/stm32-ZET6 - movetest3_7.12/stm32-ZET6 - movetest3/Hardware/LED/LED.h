#ifndef __LED_H
#define __LED_H

#include "HeaderFiles.h"

void LED_Init(void);
void LED1_Turn(void);
void LED2_Turn(void);
void LED3_Turn(void);

#define LED1_GPIO_PORT                  GPIOF
#define LED1_GPIO_PIN                   GPIO_Pin_8
#define LED1_GPIO_CLK                   RCU_GPIOF

#define LED2_GPIO_PORT                  GPIOF
#define LED2_GPIO_PIN                   GPIO_Pin_9
#define LED2_GPIO_CLK                   RCU_GPIOF

#define LED3_GPIO_PORT                  GPIOF
#define LED3_GPIO_PIN                   GPIO_Pin_10
#define LED3_GPIO_CLK                   RCU_GPIOF

/* LED¶Ë¿Ú¶¨Òå */
#define LED1(x)   do{ x ? \
                      GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN): \
                      GPIO_SetBits  (LED1_GPIO_PORT, LED1_GPIO_PIN); \
                  }while(0)      

#define LED2(x)   do{ x ? \
                      GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN): \
                      GPIO_SetBits  (LED2_GPIO_PORT, LED2_GPIO_PIN); \
                  }while(0)     

#define LED3(x)   do{ x ? \
                      GPIO_ResetBits(LED3_GPIO_PORT, LED3_GPIO_PIN): \
                      GPIO_SetBits  (LED3_GPIO_PORT, LED3_GPIO_PIN); \
                  }while(0)     

#endif
