#include <IRremote.h> // Now using PIN 9

IRsend irsend;

void setup()
{
}

void loop()
{
    irsend.sendNECRaw(0xFF00FF, 32); // The code 'FF00FF' is the received diffuser
    delay(2000);
}
