// SDI-12 Protocol  Robert Chapman III  Feb 19, 2017

// port 1 will be the data logger and port 1 will be the sensor
// should use C++ and make base sensor; then initiate with port, address, profile
// allow for multiple sensors per port
#include "timbre.h"
#include "botkernl.h"
#include "kernel.h"
#include "usart.h"

void safe_emit(Byte c);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define PORT1 huart1
#define PORT2 huart2

#define CR 13
#define LF 10

BQUEUE(75, rxq);
BQUEUE(75, rx2q);
BQUEUE(1, breaks);
BQUEUE(1, break2);
Timeout portTo, port2To;

enum {OFF, BREAK, MARK, ADDRESS, COMMAND, RESPONSE} sensor;
static Byte address = '1';

static Byte command[2] = {0, '!'};

void sendCommand(void)
{
	static enum { SEND_BREAK, SEND_COMMAND } state = SEND_BREAK;
	static Timeout markTo;
	
	switch(state) {
	case SEND_BREAK:
		HAL_LIN_SendBreak(&PORT2);
		setTimeout(22 TO_MSECS, &markTo);
		state = SEND_COMMAND;
		break;
	case SEND_COMMAND:
		if (checkTimeout(&markTo)) {
			HAL_UART_Transmit(&PORT2, command, 2, 100);
			state = SEND_BREAK;
			return;
		}
		break;
	}
	activate(sendCommand);
}

void ackCmd(void)
{
	command[0] = address;
	sendCommand();
}	

void ackRsp(void)
{
	Byte command[3] = {address, CR, LF};
	
	HAL_UART_Transmit(&PORT1, command, 3, 100);
}

void pushRx(Byte rx)
{
	pushbq(rx, rxq);
}

void pushRx2(Byte rx)
{
	pushbq(rx, rx2q);
}

bool portPower(void)
{
	return true;
}

bool portBreak(void)
{
	return qbq(breaks) != 0;
}

void setPortBreak(void)
{
	pushbq(0, breaks);
}

bool port2Break(void)
{
	return qbq(break2) != 0;
}

void setPort2Break(void)
{
	pushbq(0, break2);
}

void clearPortBreak(void)
{
	zerobq(breaks);
}

// interrupt calls
void port1Interrupt(void)
{
	uint32_t flags = READ_REG(huart1.Instance->SR);

	if (flags & USART_SR_RXNE)
		pushRx(huart1.Instance->DR & 0x7F);
	if (flags & USART_SR_LBD) {
  		huart1.Instance->SR = flags & ~USART_SR_LBD;
		if (!portBreak())
			setPortBreak();
	}
}

void port2Interrupt(void)
{
	uint32_t flags = READ_REG(huart2.Instance->SR);

	if (flags & USART_SR_RXNE)
		pushRx2(huart2.Instance->DR & 0x7F);
	if (flags & USART_SR_LBD) {
  		huart2.Instance->SR = flags & ~USART_SR_LBD;
		if (!port2Break())
			setPort2Break();
	}
}

// command and response
static Byte response[3] = {0, CR, LF};

void parseCommand(void)
{
	if (qbq(rxq) == 0)
		return;

	switch(pullbq(rxq)) {
	case '!':
		sensor = RESPONSE;
		response[0] = address;
		setTimeout(9 TO_MSECS, &portTo);
		break;
	default:
		sensor = OFF;
		break;
	}
}

// machines
void sensorMachine(void)
{
	if (portPower() == 0)
		sensor = OFF;

	switch (sensor) {
	case OFF:
		if (portPower()) {
			clearPortBreak();
			sensor = BREAK;
		}
		break;
	case BREAK:
		if (portBreak()) {
			clearPortBreak();
			zerobq(rxq);
			setTimeout(8 TO_MSECS, &portTo);
			sensor = MARK;
		}
		break;
	case MARK:
		if (checkTimeout(&portTo)) {
			setTimeout(100 TO_MSECS, &portTo);
			sensor = ADDRESS;
		}
		else if (portBreak())
			sensor = BREAK;
		else if (qbq(rxq))
				{ ; } // TODO: deal with error condition
		break;
	case ADDRESS:
		if (qbq(rxq)) {
			Byte a = pullbq(rxq);
			if (a == address || a == '?')
				sensor = COMMAND;
			else
				sensor = BREAK;
		} else if (checkTimeout(&portTo))
			sensor = OFF;
		break;
	case COMMAND:
		if (checkTimeout(&portTo))
			sensor = OFF;
		else
			parseCommand();
		break;
	case RESPONSE:
		if (checkTimeout(&portTo)) {
			HAL_UART_Transmit(&PORT1, response, 3, 100);
			sensor = OFF;
		}
		break;
	}
	activate(sensorMachine);
}

void rx2Machine(void)
{
	while (qbq(rx2q))
		safe_emit(pullbq(rx2q));

	activate(rx2Machine);
}

void initSdi12(void)
{
	sensor = OFF;
	activateOnce(sensorMachine);
	activateOnce(rx2Machine);
	
	SET_BIT(huart1.Instance->CR2, USART_CR2_LINEN);
	SET_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE);
	SET_BIT(huart1.Instance->CR2, USART_CR2_LBDIE);
	
	SET_BIT(huart2.Instance->CR2, USART_CR2_LINEN);
	SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
	SET_BIT(huart2.Instance->CR2, USART_CR2_LBDIE);
}