#include <QtTest>

#include "support.h"
#include "testoutput.h"

testoutput::testoutput(QObject *parent) : QObject(parent)
{

}

void testoutput::init()
{
    zeroEmits();
    hpStore();
    rpStore();
    spStore();
    decimal();
}

// output stream
void testoutput::TestSafeEmit()
{
    QVERIFY(numEmits() == 0);
    safeEmit('a');
    QVERIFY(numEmits() == 1);
    QVERIFY(getEmit() == 'a');
    QVERIFY(numEmits() == 0);
    QVERIFY(emitOverflow == false);
    QVERIFY(outs() == 0);
}

void testoutput::TestEmitByte()
{
    emitByte('a');
    QVERIFY(numEmits() == 1);
    QVERIFY(getEmit() == 'a');
    QVERIFY(outs() == 1);

    emitByte(8);
    QVERIFY(getEmit() == 8);
    QVERIFY(outs() == 0);

    emitByte(127);
    QVERIFY(getEmit() == 7);
    QVERIFY(outs() == 0);

    emitByte('a');
    emitByte(13);
    QVERIFY(getEmit() == 'a');
    QVERIFY(getEmit() == 13);
    QVERIFY(outs() == 0);

    emitByte(10);
    emitByte(7);
    QVERIFY(getEmit() == 10);
    QVERIFY(getEmit() == 7);
    QVERIFY(outs() == 0);
}

void testoutput::TestEmitOp()
{
    lit(45);
    emitOp();
    QVERIFY(getEmit() == 45);
    QVERIFY(depth() == 0);
}

void testoutput::TestCursorReturn()
{
    cursorReturn();
    QVERIFY(getEmit() == 10);
    QVERIFY(getEmit() == 13);
}

void testoutput::TestMaybeCr()
{
    emitByte('a');
    maybeCr();
    maybeCr();
    QCOMPARE(numEmits(), (Cell)3);
}

void testoutput::TestCount()
{
    Byte array[] = {6};

    lit((Cell)array);
    count();
    QCOMPARE(ret(), (Cell)6);
    QCOMPARE(ret(), (Cell)array+1);
    QVERIFY(depth() == 0);
}

void testoutput::TestType()
{
    Byte array[] = {2,'a','b'};

    lit((Cell)array);
    count();
    type();
    QCOMPARE(numEmits(), (Cell)2);
    QVERIFY(getEmit() == 'a');
    QVERIFY(getEmit() == 'b');
    QVERIFY(depth() == 0);
}

void testoutput::TestSpaces()
{
    spaces(2);
    QCOMPARE(numEmits(), (Cell)2);
    QVERIFY(getEmit() == ' ');
    QVERIFY(getEmit() == ' ');
}

void testoutput::TestBin()
{
    QCOMPARE(getBase(), (Byte)10);
    bin();
    QCOMPARE(getBase(), (Byte)2);
}

void testoutput::TestOct()
{
    QCOMPARE(getBase(), (Byte)10);
    oct();
    QCOMPARE(getBase(), (Byte)8);
}

void testoutput::TestDecimal()
{
    bin();
    decimal();
    QCOMPARE(getBase(), (Byte)10);
}

void testoutput::TestHex()
{
    QCOMPARE(getBase(), (Byte)10);
    hex();
    QCOMPARE(getBase(), (Byte)16);
}

void testoutput::TestHold()
{
    lit(10);
    hold();
    QVERIFY(qbq(getPadq()) == 1);
    QVERIFY(pullbq(getPadq()) == 10);
    QVERIFY(depth() == 0);
}

void testoutput::TestStartNumberConversion()
{
    lit(10);
    hold();
    startNumberConversion();
    QVERIFY(qbq(getPadq()) == 0);
}

void testoutput::TestConvertDigit()
{
    lit(9);
    convertDigit();
    QVERIFY(pullbq(getPadq()) == '9');
    QVERIFY(ret() == 0);

    lit(10);
    convertDigit();
    QVERIFY(pullbq(getPadq()) == '0');
    QVERIFY(ret() == 1);

    hex();
    lit(9);
    convertDigit();
    QVERIFY(pullbq(getPadq()) == '9');
    QVERIFY(ret() == 0);

    lit(10);
    convertDigit();
    QVERIFY(pullbq(getPadq()) == 'A');
    QVERIFY(ret() == 0);

    QVERIFY(qbq(getPadq()) == 0);
    QVERIFY(depth() == 0);
}

void testoutput::TestConvertNumber()
{
    lit(12345);
    convertNumber();
    QVERIFY(popbq(getPadq()) == '1');
    QVERIFY(popbq(getPadq()) == '2');
    QVERIFY(popbq(getPadq()) == '3');
    QVERIFY(popbq(getPadq()) == '4');
    QVERIFY(popbq(getPadq()) == '5');

    QVERIFY(ret() == 0);
    QVERIFY(qbq(getPadq()) == 0);
    QVERIFY(depth() == 0);
}

void testoutput::TestSign()
{
    lit(-1);
    lit(0);
    sign();
    QVERIFY(popbq(getPadq()) == '-');
    QVERIFY(ret() == 0);
    QVERIFY(qbq(getPadq()) == 0);

    lit(0);
    lit(-1);
    sign();
    QVERIFY((Integer)ret() == -1);
    QVERIFY(qbq(getPadq()) == 0);
    QVERIFY(depth() == 0);
}

void testoutput::TestEndNumberConversion()
{
    lit(123);
    endNumberConversion();
    QVERIFY(ret() == 0);
    QVERIFY(ret() == padBuffer());
    QVERIFY(qbq(getPadq()) == 0);
    QVERIFY(depth() == 0);
}

void testoutput::TestDotr()
{
    lit(-1);
    lit(3);
    dotr();
    QVERIFY(getEmit() == ' ');
    QVERIFY(getEmit() == '-');
    QVERIFY(getEmit() == '1');
    QVERIFY(qbq(getPadq()) == 0);
    QVERIFY(depth() == 0);
}

void testoutput::TestDot()
{
    lit(-1);
    dot();
    QVERIFY(getEmit() == '-');
    QVERIFY(getEmit() == '1');
    QVERIFY(getEmit() == ' ');
    QVERIFY(qbq(getPadq()) == 0);
    QVERIFY(depth() == 0);

    hex();
    lit(-1);
    dot();
    while(qbq(getPadq()))
        QVERIFY(getEmit() == 'F');
}

void testoutput::TestSetPrompt()
{
    setPrompt("");
    QVERIFY(*getPrompt() == 0);
    setPrompt("very long prompt");
    QVERIFY(*getPrompt() < sizeof("very long prompt"));
    setPrompt("x");
    QVERIFY(getPrompt()[1] == 'x');
}

void testoutput::TestdotPrompt()
{
    setPrompt("x");
    dotPrompt();
    QVERIFY(getEmit() == 10);
    QVERIFY(getEmit() == 13);
    QVERIFY(getEmit() == 'x');
    QVERIFY(outs() == 1);
    righBracket();
    dotPrompt();
    QVERIFY(getEmit() == 10);
    QVERIFY(getEmit() == 13);
    QVERIFY(getEmit() == ']');
    QVERIFY(getEmit() == ' ');
}
