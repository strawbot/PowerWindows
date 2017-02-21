// SDI-12 Protocol  Robert Chapman III  Feb 19, 2017

// port 1 will be the data logger and port 1 will be the sensor
// should use C++ and make base sensor; then initiate with port, address, profile
// allow for multiple sensors per port
extern "C" {
#include "timbre.h"
#include "botkernl.h"
#include "kernel.h"
#include "usart.h"
}

void safe_emit(Byte c);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define CR 13
#define LF 10
#define CHAR_TIME 9 // > 8.333ms per character

// Port Class: bind port object to a UART and GPIO for sense and power
class Port {
public:
    UART_HandleTypeDef * uart;
	
    BQUEUE(75, rxq);
	BQUEUE(1, breaks);
	Timeout portTo;
	
    Port (UART_HandleTypeDef * u) : uart(u) {}

    bool portPower () { return true; }
	void pushRx (Byte rx) { pushbq(rx, rxq); }

	bool portBreak () { return qbq(breaks) != 0; }
	void setPortBreak () { pushbq(0, breaks); }
	void clearPortBreak () { zerobq(breaks); }

public:
	void portInterrupt ();
};

void Port::portInterrupt () {
	uint32_t flags = READ_REG(uart->Instance->SR);

	if (flags & USART_SR_RXNE)
		pushRx(uart->Instance->DR & 0x7F);

	if (flags & USART_SR_LBD) {
  		uart->Instance->SR = flags & ~USART_SR_LBD;
		if (!portBreak())
			setPortBreak();
	}
}

// Declare ports
Port port1 = {&huart1};
Port port2 = {&huart2};

// External interrupt calls
extern "C" void port1Interrupt()
{
	port1.portInterrupt();
}

extern "C" void port2Interrupt()
{
	port2.portInterrupt();
}

// SDI-12 Class: used to implement a sensor or device
//class Sdi12 {
//public:
//	Port * port;
//};

//class Device : Sdi12 {
//	Byte command[2] = {0, '!'};
//	enum { SEND_BREAK, SEND_COMMAND } state = SEND_BREAK;
//};

// Sensor Class: sensors are given an address and bound to a specific port
//class Sensor : public Sdi12 {
class Sensor {
    Port * port;
	Byte address;
	enum {OFF, BREAK, MARK, ADDRESS, COMMAND, RESPONSE} state;
    Byte response[4];
    Byte length;

public:
    Sensor (Port * p, Byte a) : address(a) { port = p; state = OFF; }
	void parseCommand();

public:
	void stateMachine();
};

void Sensor::parseCommand() {
    if (qbq(port->rxq) == 0)
		return;

    switch(pullbq(port->rxq)) {
	case '!':
		state = RESPONSE;
		response[0] = address;
		response[1] = CR;
		response[2] = LF;
		length = 3;
        setTimeout(9 TO_MSECS, &port->portTo);
		break;
	default:
		state = OFF;
		break;
	}
}

void Sensor::stateMachine(void)
{
	if (port->portPower() == 0)
		state = OFF;

	switch (state) {
	case OFF:
		if (port->portPower()) {
			port->clearPortBreak();
			state = BREAK;
		}
		break;
	case BREAK:
		if (port->portBreak()) {
			port->clearPortBreak();
            zerobq(port->rxq);
            setTimeout(8 TO_MSECS, &port->portTo);
			state = MARK;
		}
		break;
	case MARK:
        if (checkTimeout(&port->portTo)) {
            setTimeout(100 TO_MSECS, &port->portTo);
			state = ADDRESS;
		}
		else if (port->portBreak())
			state = BREAK;
        else if (qbq(port->rxq))
				{ ; } // TODO: deal with error condition
		break;
	case ADDRESS:
        if (qbq(port->rxq)) {
            Byte a = pullbq(port->rxq);
			if (a == address || a == '?')
				state = COMMAND;
			else
				state = BREAK;
        } else if (checkTimeout(&port->portTo))
			state = OFF;
		break;
	case COMMAND:
        if (checkTimeout(&port->portTo))
			state = OFF;
		else
			parseCommand();
		break;
	case RESPONSE:
        if (checkTimeout(&port->portTo)) {			
            HAL_UART_Transmit(port->uart, response, length, length * 2 * CHAR_TIME);
			state = OFF;
		}
		break;
	}
}

// declare sensors
Sensor sensor1 = {&port1, '1'};
Sensor sensor2 = {&port2, '2'};

// machines
void sensor1Machine(void)
{
	sensor1.stateMachine();
	activate(sensor1Machine);
}

void sensor2Machine(void)
{
	sensor2.stateMachine();
	activate(sensor2Machine);
}

extern "C" void initSdi12(void)
{
	activateOnce(sensor1Machine);
//	activateOnce(sensor2Machine);
	
	SET_BIT(huart1.Instance->CR2, USART_CR2_LINEN);
	SET_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE);
	SET_BIT(huart1.Instance->CR2, USART_CR2_LBDIE);
	
	SET_BIT(huart2.Instance->CR2, USART_CR2_LINEN);
	SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
	SET_BIT(huart2.Instance->CR2, USART_CR2_LBDIE);
}

// Device
static Byte address = '1';
static Byte command[2] = {0, '!'};

void sendCommand(void)
{
    static enum { SEND_BREAK, SEND_COMMAND } state = SEND_BREAK;
    static Timeout markTo;

    switch(state) {
    case SEND_BREAK:
        HAL_LIN_SendBreak(&huart2);
        setTimeout(22 TO_MSECS, &markTo);
        state = SEND_COMMAND;
        break;
    case SEND_COMMAND:
        if (checkTimeout(&markTo)) {
            HAL_UART_Transmit(&huart2, command, 2, 100);
            state = SEND_BREAK;
            return;
        }
        break;
    }
    activate(sendCommand);
}

extern "C" void ackCmd(void)
{
    command[0] = address;
    sendCommand();
}

extern "C" void ackRsp(void)
{
    Byte response[3] = {address, CR, LF};

    HAL_UART_Transmit(&huart1, response, 3, 100);
}

