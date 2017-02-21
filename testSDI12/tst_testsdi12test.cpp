#include <QString>
#include <QtTest>



class TestSDI12Test : public QObject
{
    Q_OBJECT

public:
    TestSDI12Test();

private Q_SLOTS:
    void testCase1();
};

TestSDI12Test::TestSDI12Test()
{
}

void TestSDI12Test::testCase1()
{
    QVERIFY2(true, "Failure");
}

QTEST_APPLESS_MAIN(TestSDI12Test)

#include "tst_testsdi12test.moc"
