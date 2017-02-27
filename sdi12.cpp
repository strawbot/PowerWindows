// SDI-12 Protocol  Robert Chapman III  Feb 19, 2017

// port 1 will be the data logger and port 1 will be the sensor
// should use C++ and make base sensor; then initiate with port, address, profile
// allow for multiple sensors per port
extern "C" {
#include "timbre.h"
#include "botkernl.h"
#include "kernel.h"
#include "usart.h"
#include "printers.h"

#include <stdio.h>
}

void safe_emit(Byte c);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

#define CR 13
#define LF 10
#define CHAR_TIME 9 // > 8.333ms per character

bool monitor = false;

void printxx(Byte who, Byte i, Byte x) {
    if (monitor)
        printChar(who), printnDec(0, i), print(":<"), printnHex(2, x), print("> ");;
}

// Port Class: bind port object to a UART and GPIO for sense and power
class Port {
    BQUEUE(75, rxq);
    BQUEUE(1, breaks);
    UART_HandleTypeDef * uart;
    Byte number;

public:
	
    Port (UART_HandleTypeDef * u) : uart(u) { }
    Port (UART_HandleTypeDef * u, Byte n) : uart(u), number(n) { }

    bool portPower () { return true; } // maybe these two should be part of sensor since they will also be different for device
    void portSense(bool present);

	void pushRx (Byte rx) { pushbq(rx, rxq); }
    bool gotRx () { return qbq(rxq) != 0;}
    Byte getRx () { Byte r = pullbq(rxq); printxx('R', number, r); return r; }
    void emptyRx () { zerobq(rxq); }

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
    Byte * d = data, l = length;
    while (l--) printxx('T', number, *d++);

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
    enum {OFF, BREAK, MARK, ADDRESS, COMMAND, RESPONSE} state;
    enum {NONE, CONTINUOUS } command;
    Timeout sto;

public:
    float measurement[10];
    Byte response[45];
    Byte address;
    const char * id;
    Sensor (Port * p, Byte a) : port(p), address(a) { response[0] = a; state = OFF; command = NONE; }
    void setId(const char * i) { id = i; }
    void parseCommand();
    void stateMachine();
    void setResponse(const char * sequence) { strcpy((char *)&response[1], sequence); }
    void sendResponse();
};

// Sensor members
void Sensor::sendResponse() {
    Byte ending[2] = {CR, LF};

    port->sendData(response, strlen((char *)response));
    port->sendData(ending, 2);
}

void Sensor::parseCommand() {
    if (port->gotRx() == false)
		return;

    Byte c = port->getRx();

    switch(command) {
    case NONE:
        switch(c) {
        case 'I':
            setResponse(id);
            break;
        case 'R':
            command = CONTINUOUS;
            break;
        case '!':
            setTimeout(9 TO_MSECS, &sto);
            state = RESPONSE;
            break;
        default:
            state = OFF;
            break;
        }
        break;
    case CONTINUOUS:
        c = c - '0';
        if (c < 10)
            sprintf((char *)&response[1], "%+04.2f", measurement[c]);
        else
            state = OFF;
        command = NONE;
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
            port->emptyRx();
            response[1] = 0;
            state = BREAK;
		}
        break;
	case BREAK:
		if (port->portBreak()) {
			port->clearPortBreak();
            setTimeout(8 TO_MSECS, &sto);
			state = MARK;
        } else if (port->gotRx())
            port->getRx();
        break;
	case MARK:
        if (checkTimeout(&sto)) {
            setTimeout(100 TO_MSECS, &sto);
			state = ADDRESS;
            command = NONE;
		}
		else if (port->portBreak())
			state = BREAK;
        else if (port->gotRx())
				{ ; } // TODO: deal with error condition
		break;
	case ADDRESS:
        if (port->gotRx()) {
            Byte a = port->getRx();
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
    Timeout markTo;
    enum { SEND_BREAK, WAIT_MARK, SEND_COMMAND } state;

public:
    Byte command[38];
    Byte address;
    Device (Port * p, Byte a) : port(p), address(a) { command[0] = a; state = SEND_BREAK; }
    void setCommand(const char * sequence) { strcpy((char *)&command[1], sequence); }
    bool sendCommand();
    void getResponse();
};

// members
bool Device:: sendCommand() {
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
    case SEND_COMMAND: {
        Byte ending[1] = {'!'};

        port->sendData(command, strlen((char *)command));
        port->sendData(ending, 1);
        command[1] = 0;
        state = SEND_BREAK;
        return true;
    }
    }
    return false;
}

void Device::getResponse() {
    if (port->gotRx()) {
        Byte c = port->getRx();
        if (c < ' ')
            print("<"), printnHex(2, c), print(">");
        else
            printChar(c);
    }
}

// Declare ports
Port port1 = {&huart1, 1};
Port port2 = {&huart2, 2};
Port port4 = {&huart4, 4};

// declare sensors
Sensor sensor1 = {&port1, '1'};
Sensor sensor2 = {&port2, '2'};

// declare devices
Device device1 = {&port4, '1'};

// C API
extern "C" {

// Interrupts
void uart1Interrupt() {
    port1.portInterrupt();
}

void uart2Interrupt() {
    port2.portInterrupt();
}

void uart4Interrupt() {
    port4.portInterrupt();
}

// CLI
void sensor1Machine()
{
    sensor1.stateMachine();
    activate(sensor1Machine);
}

void sensor2Machine()
{
    sensor2.stateMachine();
    activate(sensor2Machine);
}

void device1SendCommand() {
    if (!device1.sendCommand())
        activate(device1SendCommand);
}

void deviceResponse() {
    device1.getResponse();
    activate(deviceResponse);
}

void ackCmd() {
    device1.setCommand("!");
    device1SendCommand();
}

void idCmd() {
    device1.setCommand("I!");
    device1SendCommand();
}

void getMeasurement() { // ( n )
    char r[3] = {'R', (char)('0'+ret), 0};

    device1.setCommand(r);
    device1SendCommand(); // make this part of the machine; check for command and send it if its there and then zero it.
}

void ackRsp() {
    sensor1.sendResponse();
}

void sensorAddress() { // ( a s )
    Byte s = (Byte)ret;
    Byte a = (Byte)(ret+'0');

    switch(s) {
    case 1: sensor1.response[0] = sensor1.address = a; break;
    case 2: sensor2.response[0] = sensor2.address = a; break;
    }
}

void deviceAddress() { // ( a )
    device1.address = device1.command[0] = (Byte)(ret+'0');
}

void sendPort1() { // ( c )
    Byte data = (Byte)ret;

    port1.sendData(&data, 1);
}

void sendPort2() { // ( c )
    Byte data = (Byte)ret;

    port2.sendData(&data, 1);
}

void sendPort4() { // ( c )
    Byte data = (Byte)ret;

    port4.sendData(&data, 1);
}

void setMonitor() { // ( b )
    monitor = (bool)ret;
}

void makeMeasurement() { // ( f n s )
    Byte s = (Byte)ret;
    Byte n = (Byte)ret;
    float f = *(float *)sp++;

    switch(s) {
    case 1: sensor1.measurement[n] = f; break;
    case 2: sensor2.measurement[n] = f; break;
    }
}

void printPin(GPIO_TypeDef* port, Short pin) {
    if (HAL_GPIO_ReadPin(port, pin)) print("open"); else print("closed");
}

void listSensors() {
    Sensor *sensors[] = {&sensor1, &sensor2};

    for (Byte i=0; i<elementsof(sensors); i++) {
        print("\nSensor"), printDec0(i+1);
        print("\n address: "), printChar(sensors[i]->address);
        print("\n id: "), print(sensors[i]->id);
        print("\n measurements:");
        for (Byte j=0; j<10; j++)
            print("\n  "), printDec(j), print("= "), printFloat(sensors[i]->measurement[j],2);
    }
    print("\nPort1 detect: "), printPin(DetectOut1_GPIO_Port, DetectOut1_Pin);
    print("\nPort2 detect: "), printPin(DetectOut2_GPIO_Port, DetectOut2_Pin);
    print("\nDevice detect 1: "), printPin(DetectIn1_GPIO_Port, DetectIn1_Pin);
    print("\nDevice detect 2: "), printPin(DetectIn2_GPIO_Port, DetectIn2_Pin);
}

void senseDetect1() { // ( b )
    GPIO_PinState state = ret ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(DetectOut1_GPIO_Port, DetectOut1_Pin, state);
}

void senseDetect2() { // ( b )
    GPIO_PinState state = ret ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(DetectOut2_GPIO_Port, DetectOut2_Pin, state);
}

// init
void initSdi12()
{
    port1.init();
    port2.init();
    port4.init();
    sensor1.setId("13CAMPBELLCS225 1.0 SN:01012");
    sensor2.setId("13CAMPBELLCS230 1.0 SN:01000");
    activateOnce(sensor1Machine);
    activateOnce(sensor2Machine);
    activateOnce(deviceResponse);
}

}
