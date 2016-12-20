// Header file for CLI

#include "ttypes.h"

#ifndef _CLI_H_
#define _CLI_H_

#endif

void absOp(void);
Cell align(Cell p);
void allot(void);
void andOp(void);
void autoEchoOff(void);
void autoEchoOn(void);
void byteErase(void);
void byteFetch(void);
void byteFill(void);
void byteMove(void);
void byteStore(void);
void cComma(void);
void collectKeys(void);
void comma(void);
void comment(void);
void compileIt(Cell tick);
void convertDigit(void);
void convertNumber(void);
void count(void);
void cursorReturn(void);
void decimal(void);
void dot(void);
void dotPrompt(void);
void dotr(void);
void dup(void);
void emitByte(Byte c);
void emitOp(void);
void endNumberConversion(void);
void equals(void);
void execute(void);;
void fetch(void);
void greaterThan(void);
void here(void);
void hex(void);
void hold(void);
void interpret(void);
void leftBracket(void);
void lessThan(void);
void lit(Cell n);
void longFetch(void);
void longStore(void);
void maxOp(void);
void maybeCr(void);
void minOp(void);
void minusBits(void);
void minusOp(void);
void modOp(void);
void negateOp(void);
void notOp(void);
void orOp(void);
void over(void);
void plusBits(void);
void plusOp(void);
void questionDup(void);
void rat(void);
Cell ret(void);
void rfrom(void);
void righBracket(void);
void safeEmit(Byte c);
void * searchWordlist(char * string);
void setPrompt(char *string);
void shiftOp(void);
void shortFetch(void);
void shortStore(void);
void sign(void);
void slashModOp(void);
void slashOp(void);
void spaces(Cell n);
void spStore(void);
void starOp(void);
void startNumberConversion(void);
void store(void);
void swap(void);
void tor(void);
void type(void);
void word(void);
void xorOp(void);