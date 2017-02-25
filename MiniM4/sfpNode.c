// SFP links for MiniM4  Robert Chapman III  Dec 5, 2016

#include "node.h"
#include "talkhandler.h"
#include "framepool.h"
#include "machines.h"
#include "stats.h"
#include "services.h"

void initSfpUart(void);
void initSfpUsb(void);
void initVersion(void);
void serviceSfpUart(void);

sfpNode_t myNode;
extern sfpLink_t uartLink;
extern sfpLink_t usbLink;

/*
Factor uart.c and network.c into one file but pull out the differences for the serial and sfp builds and put in separate files under board subfolders
Add another board folder for the ethernet/ usb board
Add usb in for the mini-m4 board
Could add nucleo board with SPI and I2C and build up a network of boards for testing
*/

// Run the SFP protocol layer
void sfpMachine(void)
{
	serviceSfpUart();
	processFrames();
	sendeqSfp();
	activate(sfpMachine);
}

void initTalkPort(void)
{
	// initialize the node
    initNode(&myNode);
    setNode(&myNode);
    setWhoami(MAIN_CPU);
    setWhatami(0);
 	initServices();

	// initialize pool of frame buffers
    initFramePool();

 	// initialize links
 	sfpLink_t * link = &uartLink;

//	initSfpUsb();
	initSfpUart();
	initVersion();
	initTalkHandler();

	// initialize services and stats
    initSfpStats();
    
    activateOnce(sfpMachine);

	addLink(0, link); // attached links
	setRouteTo(DIRECT, link);
    setRouteTo(MAIN_HOST, link); // routes for other nodes
}