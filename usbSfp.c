// USB link  Robert Chapman III  Feb 24, 2017

#include "stats.h"
#include "sfpRxSm.h"
#include "sfpTxSm.h"
#include "devices.h"
#include "machines.h"

sfpLink_t usbLink;
static Byte * input;
static Cell length;

// rx
void usbRx(Byte * p, Long l) // interrupt call
{
	input = p;
	length = l;
	while (sfpRxSm(&usbLink))
		;
}

static bool checkusb(sfpLink_t *link)
{
	(void)link;
	return length != 0;
}

static Byte getusb(sfpLink_t * link)
{
	length--;
	return *input++;
}

// tx
Byte CDC_Transmit_FS(Byte * Buf, Short Len);

void usbMachine(void)
{
	sfpLink_t *link	= &usbLink;
	
	sfpTxSm(link);

	if (link->sfpBytesToTx)
		if (0 == CDC_Transmit_FS(link->sfpTxPtr, link->sfpBytesToTx))
			link->sfpBytesToTx = 0;

	activate(usbMachine);
}

// init
QUEUE(MAX_FRAMES, usbFrameq);
QUEUE(MAX_FRAMES, usbNpsq);
QUEUE(MAX_FRAMES, usbSpsq);

void initSfpUsb(void)
{
	sfpLink_t *link  = &usbLink;
	
	// initialize usb link
	initLink(link, "usb Link");
	link->sfpRx = checkusb;
	link->sfpGet = getusb;

	// initialize state machines
	initSfpRxSM(link, usbFrameq);
	initSfpTxSM(link, usbNpsq, usbSpsq);
	
	activateOnce(usbMachine);
}
