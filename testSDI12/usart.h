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
#define USART_SR_FE 6
#define USART_SR_PE 7
#define USART_SR_NE 8
#define USART_SR_ORE 9

#define READ_REG(reg) (reg)
#define SET_BIT(reg, bit) reg |= bit

#define HAL_UART_Transmit(p, a, l, t) { (void)(p); (void)(a); (void)(l); (void)(t); }
#define HAL_LIN_SendBreak(port) (void)port

#define DetectOut1_GPIO_Port (GPIO_TypeDef *)nullptr
#define DetectOut1_Pin 0
#define DetectOut2_GPIO_Port (GPIO_TypeDef *)nullptr
#define DetectOut2_Pin 1
#define DetectIn1_GPIO_Port (GPIO_TypeDef *)nullptr
#define DetectIn1_Pin 0
#define DetectIn2_GPIO_Port (GPIO_TypeDef *)nullptr
#define DetectIn2_Pin 1
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIO_PinState bool

#define HAL_GPIO_ReadPin(p, n) (p+n)
#define HAL_GPIO_WritePin(p, n, q) (void)q

#endif // USART_H
