// SDI-12 Protocol  Robert Chapman III  Feb 19, 2017

// port 1 and 2 will be the Sensors and port 4 will be the Device
// allow for multiple sensors per port
extern "C" {
#include "timbre.h"
#include "botkernl.h"
#include "kernel.h"
#include "usart.h"
#include "printers.h"

#include <stdio.h>
}

// external
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

// funny numbers
#define CR 13
#define LF 10
#define CHAR_TIME 9 // > 8.333ms per character

// debug
bool monitor = false;

void printxx(Byte who, Byte i, Byte x) {
    if (monitor)
        printChar(who), printnDec(0, i), print(":<"), printnHex(2, x), print("> ");;
}

// Class hierarchy: Port->Sdi12->Sensor/Device

// Port Class: bind port object to a UART and GPIO for sense and power
class Port {
    BQUEUE(75, rxq);
    BQUEUE(1, breaks);
    UART_HandleTypeDef * uart;

public:
    Byte number;

    Port (UART_HandleTypeDef * u, Byte n=0) : uart(u), number(n) { }

    void emptyRx () { zerobq(rxq); }
    void pushRx (Byte rx) { pushbq(rx, rxq); }
    bool gotRx () { return qbq(rxq) != 0;}
    Byte getRx () { Byte r = pullbq(rxq); printxx('R', number, r); return r; }

    void sendBreak () { HAL_LIN_SendBreak(uart); }
    bool portBreak () { return qbq(breaks) != 0; }
    void setPortBreak () { pushbq(0, breaks); }
    void clearPortBreak () { zerobq(breaks); }

    void sendData (Byte * data, Byte length)
    { HAL_UART_Transmit(uart, data, length, length * 2 * CHAR_TIME); }

    void portInterrupt ();

    void init() {
        SET_BIT(uart->Instance->CR1, USART_CR1_RXNEIE);
        SET_BIT(uart->Instance->CR2, USART_CR2_LINEN);
        SET_BIT(uart->Instance->CR2, USART_CR2_LBDIE);
    }
};

// SDI-12 common protoocl functions
class Sdi12 : public Port {
protected:
    Timeout to;
    enum {OFF, BREAK, MARK, ADDRESS, COMMAND, RESPONSE} state;
    Byte ending[2];
    Byte * buffer;

public:
    Sdi12(UART_HandleTypeDef * u, Byte n, Byte a)
    : Port (u, n) { setAddress(a); }

    void sendData (Byte * data, Byte length);

    void setAddress (Byte a) { buffer[0] = a; }
    Byte getAddress () { return buffer[0]; }
    void setBuffer (const char * s) { strcpy((char *)&buffer[1], s); }
    void sendBuffer () { sendData(buffer, strlen((char *)buffer)); }
    Byte * getBuffer() { return &buffer[1]; }
};

// Sensor class: sensors are given an address and bound to a specific port
// need to consider the case of multiple sensors on the same port. Address will determine who talks.
// If ?! is sent, then they all talk but their values will all have to be combined first. So we will
// need a port driver that can do that. So no longer do we call HAL directly but instead through the port.
class Sensor : public Sdi12 {
    Byte response[45];
    enum {NONE, CONTINUOUS } command;

public:
    float measurement[10];
    const char * id;

    Sensor (UART_HandleTypeDef * u, Byte n, Byte a)
    : Sdi12 (u, n, a) { state = OFF; command = NONE; ending[0] = CR; ending[1] = LF; buffer = response; }

    void protocol ();
    void setId (const char * i) { id = i; }
    void parseCommand ();
};

// device class - similar to sensors but other side; could factor up a super class with: address, timeout, cmd/rsp buffer, ending
class Device : public Sdi12 {
    Byte command[38];

public:
    Device (UART_HandleTypeDef * u, Byte n, Byte a)
    : Sdi12(u, n, a) { state = BREAK;  ending[0] = '!'; buffer = command; }

    bool sendCommand();
    void getResponse();
};


// Port members
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

// Sdi12 members
void Sdi12::sendData (Byte * data, Byte length) {
    Byte * d = data, l = length;

    while (l--) printxx('T', number, *d++);

    Port::sendData(data, length);
}

// Sensor members
void Sensor::parseCommand() {
    if (gotRx() == false)
		return;

    Byte c = getRx();

    switch(command) {
    case NONE:
        switch(c) {
        case 'I':
            setBuffer(id);
            break;
        case 'R':
            command = CONTINUOUS;
            break;
        case '!':
            setTimeout(9 TO_MSECS, &to);
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
            sprintf((char *)getBuffer(), "%+04.2f", measurement[c]);
        else
            state = OFF;
        command = NONE;
        break;
    }
}

void Sensor::protocol() {
	switch (state) {
	case OFF:
        clearPortBreak();
        emptyRx();
        setBuffer("");
		state = BREAK;
        break;
	case BREAK:
        if (portBreak()) {
            clearPortBreak();
            setTimeout(8 TO_MSECS, &to);
			state = MARK;
        } else if (gotRx())
            getRx();
        break;
	case MARK:
        if (checkTimeout(&to)) {
            setTimeout(100 TO_MSECS, &to);
			state = ADDRESS;
            command = NONE;
		}
        else if (portBreak())
			state = BREAK;
        else if (gotRx())
				{ ; } // TODO: deal with error condition
		break;
	case ADDRESS:
        if (gotRx()) {
            Byte a = getRx();

            if (a == getAddress() || a == '?')
				state = COMMAND;
			else
				state = BREAK;
        } else if (checkTimeout(&to))
			state = OFF;
		break;
	case COMMAND:
        if (checkTimeout(&to))
			state = OFF;
		else
			parseCommand();
		break;
	case RESPONSE:
        if (checkTimeout(&to)) {
            sendBuffer();
            sendData(ending, 2);
            state = OFF;
		}
		break;
	}
}

// device members
bool Device:: sendCommand() {
    switch(state) {
    case BREAK:
        sendBreak();
        setTimeout(22 TO_MSECS, &to);
        state = MARK;
        break;
    case MARK:
        if (checkTimeout(&to))
            state = COMMAND;
        break;
    case COMMAND:
        sendBuffer();
        sendData(ending, 1);
        state = BREAK;
        return true;
    default:
        state = BREAK;
        break;
    }
    return false;
}

void Device::getResponse() {
    if (gotRx()) {
        Byte c = getRx();
        if (c < ' ')
            print("<"), printnHex(2, c), print(">");
        else
            printChar(c);
    }
}

// declare sensors
Sensor sensor1 = {&huart1, 1, '1'};
Sensor sensor2 = {&huart2, 2, '2'};

// declare devices
Device device1 = {&huart4, 4, '1'};

// C API
extern "C" {

// Interrupts
void uart1Interrupt() {
    sensor1.portInterrupt();
}

void uart2Interrupt() {
    sensor2.portInterrupt();
}

void uart4Interrupt() {
    device1.portInterrupt();
}

// CLI
void sensor1Machine() {
    sensor1.protocol();
    activate(sensor1Machine);
}

void sensor2Machine() {
    sensor2.protocol();
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
    device1.setBuffer("");
    device1SendCommand();
}

void idCmd() {
    device1.setBuffer("I");
    device1SendCommand();
}

void getMeasurement() { // ( n )
    char r[3] = {'R', (char)('0'+ret), 0};

    device1.setBuffer(r);
    device1SendCommand(); // make this part of the machine; check for command and send it if its there and then zero it.
}

void ackRsp() {
    sensor1.sendBuffer();
}

Sensor * pickSensor() { // ( s )
    Byte s = (Byte)ret;
    switch(s) {
    case 1: return &sensor1;
    case 2: return &sensor2;
    }
    return (Sensor *)NULL;
}

void sensorAddress() { // ( a s )
    Sensor * s = pickSensor();

    s->setAddress(ret+'0');
}

void deviceAddress() { // ( a )
    device1.setAddress(ret+'0');
}

void sendPort1() { // ( c )
    Byte data = (Byte)ret;

    sensor1.sendData(&data, 1);
}

void sendPort2() { // ( c )
    Byte data = (Byte)ret;

    sensor2.sendData(&data, 1);
}

void sendPort4() { // ( c )
    Byte data = (Byte)ret;

    device1.sendData(&data, 1);
}

void setMonitor() { // ( b )
    monitor = (bool)ret;
}

void makeMeasurement() { // ( f n s )
    Sensor * s = pickSensor();
    Byte n = (Byte)ret;
    float f = *(float *)sp++;

    s->measurement[n] = f;
}

void printPin(GPIO_TypeDef* port, Short pin) {
    if (HAL_GPIO_ReadPin(port, pin)) print("open"); else print("closed");
}

void listSensors() {
    Sensor *sensors[] = {&sensor1, &sensor2};

    for (Byte i=0; i<elementsof(sensors); i++) {
        print("\nSensor"), printDec0(i+1);
        print("\n address: "), printChar(sensors[i]->getAddress());
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

void setId() { // ( i n )
	Sensor * s = pickSensor();
	char * i = (char *)ret;
	
	s->setId(i);
}
	
// init
void initSdi12() {
    sensor1.init();
    sensor2.init();
    device1.init();
    sensor1.setId("13CAMPBELLCS225 1.0 SN:01012");
    sensor2.setId("13CAMPBELLCS230 1.0 SN:01000");
    activateOnce(sensor1Machine);
    activateOnce(sensor2Machine);
    activateOnce(deviceResponse);
}

}
