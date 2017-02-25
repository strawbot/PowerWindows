#ifndef USART_H
#define USART_H
#include <stdint.h>

typedef struct {
    uint32_t CR1, CR2, CR3, SR, DR;
}usart;

typedef struct {
    usart * Instance;
} UART_HandleTypeDef;

typedef struct {

} GPIO_TypeDef;

#define USART_CR2_LINEN 1
#define USART_CR1_RXNEIE 2
#define USART_CR2_LBDIE 3
#define USART_SR_LBD 4
#define USART_SR_RXNE 5

#define READ_REG(reg) (reg)
#define SET_BIT(reg, bit) reg |= bit

#define HAL_UART_Transmit(p, a, l, t) { (void)(p); (void)(a); (void)(l); (void)(t); }
#define HAL_LIN_SendBreak(port) (void)port

#define PORT1_SENSE_GPIO_Port (GPIO_TypeDef *)nullptr
#define PORT1_SENSE_Pin 0
#define PORT1_12V_GPIO_Port (GPIO_TypeDef *)nullptr
#define PORT1_12V_Pin 1
#define PORT2_SENSE_GPIO_Port (GPIO_TypeDef *)nullptr
#define PORT2_SENSE_Pin 2
#define PORT2_12V_GPIO_Port (GPIO_TypeDef *)nullptr
#define PORT2_12V_Pin 3

#endif // USART_H
