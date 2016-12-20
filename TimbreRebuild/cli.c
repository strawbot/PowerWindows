// Command Line Interpreter  Robert Chapman III  Dec 6, 2016
// or call it TimbreTalk?

#include "machines.h"
#include "byteq.h"

// parameters
#define DCELLS 30  /* number of data stack cells */
#define RCELLS 30  /* number of return stack cells */
#define LINE_LENGTH 80 /* number of characters allowed in tib */

// structures
static QUEUE(DCELLS, dataStack);
static QUEUE(RCELLS, returnStack);

// data stack
Cell ret(void)  /* m - */
{
	return popq(dataStack);
}

void lit(Cell n)  /* - n */
{
	pushq(n, dataStack);
}

void spStore(void) /* ... - */
{
	zeroq(dataStack);
}

void swap(void)  /* m n - n m */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);
	
	pushq(top, dataStack);
	pushq(next, dataStack);
}

void dup(void)  /* m - m m */
{
	pushq(p(dataStack), dataStack);
}

void over(void) /* m n - m n m */
{
	Cell top = popq(dataStack);
	Cell next = p(dataStack);
	
	pushq(top, dataStack);
	pushq(next, dataStack);
}

void questionDup(void)  /* n - [n] n */
{
	if (p(dataStack) != 0)
		dup();
}

// return stack
void rpStore(void)
{
	zeroq(returnStack);
}

void tor(void)  /* m - */
{
	pushq(popq(dataStack), returnStack);
}

void rat(void)  /* - m */
{
	pushq(p(returnStack), dataStack);
}

void rfrom(void)  /* - m */
{
	pushq(popq(returnStack), dataStack);
}

// logic
#define binary(op) 	\
	Cell top = popq(dataStack); \
	Cell next = popq(dataStack); \
	\
	pushq(next op top, dataStack)
#define binaryInts(op) 	\
	Cell top = popq(dataStack); \
	Cell next = popq(dataStack); \
	\
	pushq((Cell)((Integer)next op (Integer)top), dataStack)

#define unary(op) pushq( op popq(dataStack), dataStack)

void andOp(void)  /* m n - p */
{
	binary(&);
}

void orOp(void)  /* m n - p */
{
	binary(|);
}

void xorOp(void)  /* m n - p */
{
	binary(^);
}

void notOp(void)  /* m - n */
{
	unary(~);
}

void shiftOp(void) /* n m - p */
{
	Cell top = popq(dataStack); \
	Cell next = popq(dataStack); \

	if ((signed)top < 0)
		next >>= abs((Integer)top);
	else
		next <<= top;
	pushq(next, dataStack);
}

// math
void plusOp(void)  /* m \ n -- p */
{
	binary(+);
}

void minusOp(void)  /* m \ n -- p */
{
	binary(-);
}

void negateOp(void)  /* m -- n */
{
	unary(-);
}

void slashModOp(void)  /* n \ m -- remainder \ quotient */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);

	pushq(next % top, dataStack);
	pushq(next / top, dataStack);
}

void slashOp(void)  /* n \ m -- quotient */
{
	binary(/);
}

void modOp(void)  /* n \ m -- remainder */
{
	binary(%);
}

void starOp(void)  /* n \ m -- p */
{
	binary(*);
}

void absOp(void)  /* n -- n */
{
	if((Integer)p(dataStack) < 0)
		pushq((Cell)(-(Integer)popq(dataStack)), dataStack);
}

void maxOp(void)  /* n \ m -- p */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);

	if ((Integer)top > (Integer)next)
		pushq(top, dataStack);
	else
		pushq(next, dataStack);
}

void minOp(void)  /* n \ m -- p */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);

	if ((Integer)top < (Integer)next)
		pushq(top, dataStack);
	else
		pushq(next, dataStack);
}

// compare
void equals(void)  /* n \ m -- flag */
{
	binary(==);
}

void lessThan(void)  /* n \ m -- flag */
{
	binaryInts(<);
}

void greaterThan(void)  /* n \ m -- flag */
{
	binaryInts(>);
}

// memory
static Byte * hp;

void here(void)  /* -- addr */
{
	lit((Cell)hp);
}

void allot(void)  /* n -- */
{
	hp += popq(dataStack);
}

void cComma(void)  /* n -- */
{
	*hp++ = (Byte)popq(dataStack);
}

Cell align(Cell p)  /* a -- a' */
{
	struct{void*x; char y; void*z;}a;
	Cell z = (Cell)&a.z - (Cell)&a.y - sizeof(a.y); /* 1 or 3 */
	
	return (p + z) & ~z;
}

void comma(void)  /* n -- */
{
	Cell top = popq(dataStack);
	Cell * p = (Cell *)hp;

	hp = (Byte *)align((Cell)hp);
	*p = top;
	hp += sizeof(Cell);
}

void fetch(void)  /* a -- n */
{
	pushq(*(Cell *)popq(dataStack), dataStack);
}

void store(void)  /* n \ a -- */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);
	
	*(Cell *)top = next;
}

void longFetch(void) // a - lo \ hi
{
	Long l = *(Long *)popq(dataStack);
	
	pushq((Cell)(l & 0xFFFF), dataStack);
	pushq((Cell)(l>>16), dataStack);
}

void longStore(void) // lo \ hi \ a -
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);
	Cell third = popq(dataStack);

	*(Long *)top = ((Long)next<<16) + third;
}
	
void shortFetch(void) // a - n
{
	Short s = *(Short *)popq(dataStack);
	
	pushq((Cell)s, dataStack);
}

void shortStore(void) // n \ a -
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);

	*(Short *)top = (Short)next;
}

void byteFetch(void)  /* a -- c */
{
	Byte c = *(Byte *)popq(dataStack);
	
	pushq((Cell)c, dataStack);
}

void byteStore(void)  /* c \ a -- */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);

	*(Byte *)top = (Byte)next;
}

void plusBits(void)  /* bits \ addr -- */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);

	*(Byte *)top |= next;
}

void minusBits(void)  /* bits \ addr -- */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);

	*(Byte *)top = (Byte)(*(Byte *)top & ~next);
}

void byteMove(void)  /* src \ dest \ count -- */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);
	Cell third = popq(dataStack);

	memcpy((void *)next, (void *)third, top);
}

void byteFill(void)  /* addr \ count \ char -- */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);
	Cell third = popq(dataStack);

	memset((void *)third, (Byte)top, next);
}

void byteErase(void)  /* addr \ count -- */
{
	lit(0);
	byteFill();
}

// output stream
static BQUEUE(160, emitq);

static Cell outp=0;

void safeEmit(Byte c)
{
	static bool alreadyHere = false; // prevent overwrites

	if (fullbq(emitq)) {
		if (alreadyHere) // support blocking on first writer but dump after that
			return;
		alreadyHere = true;
		while (fullbq(emitq))
			runMachines();  // sit here until sent
		alreadyHere = false;
	}
	pushbq(c, emitq);
}

#define BEEP 7
#define BSPACE 8
#define LFEED 10
#define CRETURN 13
#define DELETE 127

void emitByte(Byte c)
{
	if ((c == BSPACE) || (c == DELETE)) {
		if (outp != 0)
			outp -= 1, c = BSPACE;
		else
			c = BEEP;
	}
	else if (c == CRETURN)  /* cursor return? */
		outp = 0;
	else if ((c != LFEED) && (c != BEEP))
		outp += 1;         /* not line feed, pacing character or bell? */
	safeEmit(c);
}

void emitOp(void)  /* char -- */
{
	emitByte((Byte)popq(dataStack));
}

void cursorReturn(void)  /* -- */
{
	emitByte(10);
	emitByte(13);
}

void maybeCr(void)  /* -- */
{
	if(outp != 0)
		cursorReturn();
}

void count(void)  /* addr -- addr' \ count */
{
	Byte * a = (Byte *)popq(dataStack);
	
	pushq((Cell)&a[1], dataStack);
	pushq((Cell)*a, dataStack);
}

void type(void)  /* addr \ count -- */
{
	Byte n = (Byte)popq(dataStack);
	Byte * a = (Byte *)popq(dataStack);
	
	while (n--)
		emitByte(*a++);
}

void spaces(Cell n)
{
	while (n--)
		emitByte(' ');
}

static Byte base = 10; // command line number radix

void bin(void)  /* -- */
{
	base = 2;
}

void oct(void)  /* -- */
{
	base = 8;
}

void decimal(void)  /* -- */
{
	base = 10;
}

void hex(void)  /* -- */
{
	base = 16;
}

BQUEUE(20, padq); // safe place to format numbers

void hold(void)  /* char -- */
{
	pushbq((Byte)popq(dataStack), padq);
}

void startNumberConversion(void)  /* -- */
{
	zerobq(padq);
}

void convertDigit(void)  /* n -- n */
{
	Cell n = popq(dataStack);
	Byte c = (Byte)(n % base);

	if (c > 9)
		c += 7;
	c += 48;
	pushbq(c, padq);

	n /= base;
	pushq(n, dataStack);
}

void convertNumber(void)  /* n -- n */
{
	while(p(dataStack) != 0)
		convertDigit();
}

void sign(void)  /* m \ n -- n */
{
	Cell top = popq(dataStack);
	Cell next = popq(dataStack);

	if((Integer)next < 0)
		pushbq('-', padq);
	pushq(top, dataStack);
}

void endNumberConversion(void)  /* n -- addr \ count */
{
	Cell n = qbq(padq);
	Byte * a = &hp[LINE_LENGTH];
	
	popq(dataStack);
	pushq((Cell)a, dataStack);
	pushq(n, dataStack);

	while (qbq(padq))
		*a++ = popbq(padq);
}

void dotr(void)  /* n \ m -- */
{
	tor();

	startNumberConversion();
	if (base == 10) {
		dup();
		absOp();
		convertNumber();
		sign();
	} else
		convertNumber();
	endNumberConversion();

	rfrom();
	over();
	minusOp();
	lit(0);
	maxOp();
	spaces(popq(dataStack));
	type();
}

void dot(void)  /* n -- */
{
	lit(0);
	dotr();
	spaces(1);
}

// compiler
#define Headless(function) \
extern void function(void); \
struct { vector tick; }_##function = {function};

Cell compiling = 0;
thread *ip, tick;

void righBracket(void)
{
	compiling = 0x80;
}

void leftBracket(void)
{
	compiling = 0;
}

void compileIt(thread t)
{
	lit((Cell)t);
	comma();
}

void litii(void)  /* -- n */
{
	lit((Cell)*ip++);
}

Headless(litii)

void literal(Cell n)
{
	if (compiling)
		compileIt(&_litii.tick);
	lit(n);
	comma();
}

void executeIt(thread t)
{
	tick = *t;
	(*t)();
}

void execute(void) /* a - */
{
	executeIt((thread)ret());
}

// prompt
#ifndef PROMPTSTRING  // replace by defining
#define PROMPTSTRING "\010timbre: "
#endif

Byte prompt[10]={PROMPTSTRING};

void setPrompt(char *string)
{
	prompt[0] = (Byte)strlen(string);
	strcpy((char*)&prompt[1], string);
}

void dotPrompt(void)
{
	maybeCr();
	compiling ?	lit((Cell)"\002] ") : lit((Cell)prompt);
	count();
	type();
}

// parsing
struct {
	Cell in; // index into buffer
	Byte buffer[LINE_LENGTH + 1]; // extra room for zero at end
} tib;

void zeroTib(void)
{
	tib.in = 0;
}

void skip(Byte c) // skip c in input
{
	while (tib.buffer[tib.in] == c)
		tib.in++;
}

void parse(Byte c) // parse string till char or 0 from input to here count prefixed
{
	Byte * input = &tib.buffer[tib.in];
	Byte * output = hp;

	while (*input != 0) {
		Byte b = *input++;

		if (b != c)
			*++output = b;
	}
	hp[0] = output - hp;	// count prefixed
	output[1] = 0;			// null terminate for usage as C string
	tib.in = input - tib.buffer;
}

Byte * parseWord(Byte c) // return C-string from input
{
	skip(c);
	parse(c);
	return &hp[1];
}

// dictionary words
/* CLI built words use this header structure:
 * [ link | name | II | list of other ticks ]

 * Prebuilt headers use:
 * [ name1 | name2 | ... | nameN ]
 * [ func1 | func2 | ... | funcN ]
 *
 * Address of II or funci is called the tick. ticks are executed or compiled.
 */
#define NAME_BITS 0x80
#define IMMEDIATE_BITS (NAME_BITS | 0x40)
#define SMUDGE_BITS 0x20
#define HEADER_BITS (IMMEDIATE_BITS | SMUDGE_BITS)

typedef struct header {
	struct header * list;
	Byte name[];
} header;

static header * wordlist = NULL; // list of words created from CLI

// search CLI list
header * searchWordlist(Byte * string)
{
	header * list = wordlist;

	while (list) {
		Byte * name = list->name;

		if ((name[0] & ~IMMEDIATE_BITS) == string[0]) // smudged bit prevents matching bad headers
			if (0 == memcmp(&name[1], &string[1], string[0]))
				break;
		list = list->list;
	}
	return list;
}

// search prebuilt word lists
// Non HARVAARD architectures
#define PROGMEM const
#define PGM_P const char *
#define pgm_read_byte *
#define strcmp_P(a,b) strcmp((char*)a,(char*)b)
#define strlen_P(m) strlen(m)

extern vector wordbodies[];
extern void (*constantbodies[])();
extern void (*immediatebodies[])();
extern PROGMEM char wordnames[];
extern PROGMEM char constantnames[];
extern PROGMEM char immediatenames[];

Short searchNames(Byte * name, PGM_P dictionary) // return name number or 0 if not found
{
	Short index = 1;

	while(pgm_read_byte(dictionary)) {
		if (strcmp_P(name, dictionary) == 0)
			return index;
		index++;
		dictionary += strlen_P(dictionary) + 1;
	}
	return 0;
}

Byte searchDictionaries(Byte * name, thread * t) // look through dictionaries for word
{ // s -- a \ f
	Short index;

	index = searchNames(name, wordnames);
	if (index != 0) {
		*t = &wordbodies[index-1];
		return NAME_BITS;
	}

	index = searchNames(name, constantnames);
	if (index != 0) {
		*t = &constantbodies[2*(index-1)]; // array of two pointers
		return NAME_BITS;
	}

	index = searchNames(name, immediatenames);
	if (index != 0) {
		*t = &immediatebodies[index-1];
		return IMMEDIATE_BITS;
	}

	return 0;
}

thread link2tick(header * link)
{
	Byte length = link->name[0] & ~HEADER_BITS;
	Cell t = align((Cell)link->name[1 + length]);

	return (thread)t;
}

Byte lookup(Byte * string, thread * t)
{
	header * result;

	result = searchWordlist(string);
	if (result != 0) {
		*t = link2tick(result);
		return result->name[0] & HEADER_BITS;
	}

	return searchDictionaries(string, t);
}

// Error recovery
Byte interpretError = 0;

void msg(const char * m) // message in program space
{
	while (*m)
		emitByte(*m++);
}

void error(void)  /* -- */
{
	here();
	count();
	type();
	msg("<- eh?");
	interpretError = 1;
}

// Number conversion
Byte checkBase(Byte * string) // check for prefixes: 0X, 0x, 0C, 0c, 0B or 0b
{
	if (string[0] != 0 && string[1] != 0 && string[2] != 0) // count is longer than 2
		if (*string == '0') {  // and first digit is 0
			switch(string[1]) {
			case 'b': case 'B':	bin(); break;
			case 'c': case 'C':	oct(); break;
			case 'x': case 'X':	hex(); break;
			default: return 1; // skip leading zero
			}
			return 2; // skip leading base change
		}
	return 0; // skip nothing
}

bool toDigit(Byte *n) // convert character to number according to base
{// covers all alphanumerics and bases
	Byte c = (Byte)(*n - '0');

	if (c > 9) {
		c -= 7;
		if (c > 35)
			c -= ' ';
		if (c < 10)
			return false;
	}
	if (c >= base)
		return false;
	*n = c;
	return true;
}

Cell signDigits(Byte * string, bool sign) // convert string to number according to base
{
	Cell n = 0;

	if (*string == 0) {
		error();
		return 0;
	}

	while (*string) {
		Byte c = *string++;

		if (!toDigit(&c)) {
#ifdef FLOAT_SUPPORT
			if (c == '.') { // decimal point encountered - try for mantissa
				float f = 0;

				while (*string) // start at end of string to work back to decimal
					string++;

				while (*--string != '.') {
					c = *string;
					if (!toDigit(&c)) {
						error();
						return;
					}
					f = (f+c)/base;
				}
				f = f + n;
				if (sign)
					f = -f;
				return (Cell)f;
			}
#endif
			error();
			return n;
		}
		n = n*base + c;
	}
	if (sign)
		n = -n;
	return n;
}

Cell stringNumber(Byte * string)
{
	Cell n;
	Byte b = base;
	bool sign = (*string == '-');

	if (sign)
		string++;
	string += checkBase(string);
	n = signDigits(string, sign);
	base = b;
	return n;
}

// interpreter
void quit(void)  /* -- */
{
	interpretError = 0;
	spStore();
	rpStore();
	zeroTib();
	leftBracket();
	cursorReturn();
	dotPrompt();
}

void interpret(void)
{
	while (tib.buffer[tib.in] != 0) {
		thread t;
		Byte headbits;
		Byte * string;

		string = parseWord(' ');
		headbits = lookup(string, &t);
		if (headbits != 0)
			headbits == compiling ? compileIt(t) : executeIt(t);
		else {
			Cell n = stringNumber(string);

			if (interpretError) {
				quit();
				break;
			}
			literal(n);
		}
	}
}

void comment(void)  /* char -- */ // scan input for end comment or 0
{
	Byte * input = &tib.buffer[tib.in];

	while (*input)
		if (*input++ == ')')
			break;
	tib.in = input - tib.buffer;
}

// input stream
BQUEUE(80,keyq);
Byte keyEcho = 0;
Byte autoecho = 0; // can be turned off to silently process a line

void autoEchoOn(void) // echo keys back
{
	keyEcho = autoecho = 1;
}

void autoEchoOff(void) // don't echo keys back
{
	keyEcho = autoecho = 0;
}

void collectKeys(void)
{
	if (qbq(keyq) == 0)
		return;
	
	Byte echo = keyEcho;
	Byte key = pullbq(keyq);

	switch (key) {
	case 10: // ignore line feeds
		return;
	case 8:
	case 127: // backspace or delete
		if (tib.in != 0)
			tib.in -= 1;
		else
			key = 7;
		break;
	case 13:
	case 0:  // a cursor return
		keyEcho = autoecho;
		key = 0;
		tib.buffer[tib.in] = key;
		outp = 0;
		if (echo)
			spaces(1);
		zeroTib();
		interpret();
		zeroTib();
		tib.buffer[tib.in] = 0;
		dotPrompt();
		return;
	default:
		if ( key < 27 )
			key = 7;
		else if ( tib.in < LINE_LENGTH ) { // check in not out!
			tib.buffer[tib.in] = key;
			tib.in++;
		}
		else
			key = 7;
	}

	if (keyEcho)
		emitByte(key);
}