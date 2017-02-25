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
extern UART_HandleTypeDef huart4;

#define CR 13
#define LF 10
#define CHAR_TIME 9 // > 8.333ms per character

// Port Class: bind port object to a UART and GPIO for sense and power
class Port {
public:
    UART_HandleTypeDef * uart;
	
    BQUEUE(75, rxq);
	BQUEUE(1, breaks);
	
    Port (UART_HandleTypeDef * u) : uart(u) { }

    bool portPower () { return true; } // maybe these two should be part of sensor since they will also be different for device
    void portSense(bool present);
	void pushRx (Byte rx) { pushbq(rx, rxq); }

    void sendBreak () { HAL_LIN_SendBreak(uart); }
	bool portBreak () { return qbq(breaks) != 0; }
	void setPortBreak () { pushbq(0, breaks); }
	void clearPortBreak () { zerobq(breaks); }

    void sendData (Byte * data, Byte length);
	void init() {
		SET_BIT(uart->Instance->CR1, USART_CR1_RXNEIE);
        SET_BIT(uart->Instance->CR2, USART_CR2_LINEN);
        SET_BIT(uart->Instance->CR2, USART_CR2_LBDIE);
    }


public:
    void portInterrupt ();
};

// Port members
void Port::sendData (Byte * data, Byte length)
{
    HAL_UART_Transmit(uart, data, length, length * 2 * CHAR_TIME);
}

void Port::portInterrupt () {
    uint32_t flags = READ_REG(uart->Instance->SR);

    if (flags & USART_SR_RXNE) {
		Byte rx = uart->Instance->DR & 0x7F;
		if (0 == (flags & (USART_SR_FE | USART_SR_PE | USART_SR_NE | USART_SR_ORE)))
			pushRx(rx);
	}
    if (flags & USART_SR_LBD) {
        uart->Instance->SR = flags & ~USART_SR_LBD;
        if (!portBreak())
            setPortBreak();
    }
}

// Sensor class: sensors are given an address and bound to a specific port
// need to consider the case of multiple sensors on the same port. Address will determine who talks. If ?! is sent, then they all talk
// but their values will all have to be combined first. So we will need a port driver that can do that. So no longer do we call HAL directly
// but instead through the port.
class Sensor {
    Port * port;
    Byte address;
    enum {OFF, BREAK, MARK, ADDRESS, COMMAND, RESPONSE} state;
    Byte response[45];
    Timeout sto;

public:
    Sensor (Port * p, Byte a) : port(p), address(a) { state = OFF; }
    void parseCommand();
    void stateMachine();
    void sendResponse();
};

// Sensor members
void Sensor::sendResponse() {
    response[0] = address;
    response[1] = CR;
    response[2] = LF;
    port->sendData(response, 3);
}

void Sensor::parseCommand() {
    if (qbq(port->rxq) == 0)
		return;

    switch(pullbq(port->rxq)) {
	case '!':
        setTimeout(9 TO_MSECS, &sto);
        state = RESPONSE;
        break;
	default:
		state = OFF;
		break;
	}
}

void Sensor::stateMachine()
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
            setTimeout(8 TO_MSECS, &sto);
			state = MARK;
		}
		break;
	case MARK:
        if (checkTimeout(&sto)) {
            setTimeout(100 TO_MSECS, &sto);
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
        } else if (checkTimeout(&sto))
			state = OFF;
		break;
	case COMMAND:
        if (checkTimeout(&sto))
			state = OFF;
		else
			parseCommand();
		break;
	case RESPONSE:
        if (checkTimeout(&sto)) {
            sendResponse();
			state = OFF;
		}
		break;
	}
}

class Device {
    Port * port;
    Byte command[38];
    Timeout markTo;
    enum { SEND_BREAK, WAIT_MARK, SEND_COMMAND } state;

public:
    Device (Port * p, Byte a) : port(p) { command[0] = a; state = SEND_BREAK; }
    bool sendCommand();
    void setCommand(const char * sequence) { strcpy((char *)&command[1], sequence); }
};

// members
bool Device:: sendCommand()
{
    switch(state) {
    case SEND_BREAK:
        port->sendBreak();
        setTimeout(22 TO_MSECS, &markTo);
        state = WAIT_MARK;
        break;;
    case WAIT_MARK:
        if (checkTimeout(&markTo))
            state = SEND_COMMAND;
        break;
    case SEND_COMMAND:
        port->sendData(command, 2);
        state = SEND_BREAK;
        return true;
    }
    return false;
}


// Declare ports
Port port1 = {&huart1};
Port port2 = {&huart4};
Port port3 = {&huart2};

extern "C" void uart1Interrupt()
{
    port1.portInterrupt();
}

extern "C" void uart2Interrupt()
{
    port3.portInterrupt();
}

extern "C" void uart4Interrupt()
{
    port2.portInterrupt();
}

// declare sensors
Sensor sensor1 = {&port1, '1'};
Sensor sensor2 = {&port2, '1'};

void sensor1Machine()
{
    sensor1.stateMachine();
    activate(sensor1Machine);
}

// declare devices
Device device1 = {&port3, '1'};

void device1SendCommand() {
    if (!device1.sendCommand())
        activate(device1SendCommand);
}

extern "C" void ackCmd() {
    device1.setCommand("!");
    device1SendCommand();
}

extern "C" void ackRsp() {
    sensor1.sendResponse();
}

// init
extern "C" void initSdi12()
{
	port1.init();
	port2.init();
	port3.init();
    activateOnce(sensor1Machine);
}

