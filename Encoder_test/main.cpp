#include "mbed.h"

volatile uint8_t last; //0=Arise, 1=Afall, 2=Brise, 3=Bfall

void encoderAfall()
{

}

void encoderArise()
{

}

void encoderBfall()
{

}

void encoderBrise()
{

}

// main() runs in its own thread in the OS
int main()
{
    // SPI device(PB_5, PB_4, PB_3);
    // DigitalOut encoder_cs(PA_4);

    InterruptIn  A(PA_3);
    InterruptIn  B(PA_4);
    A.fall(&encoderAfall);
    A.rise(&encoderArise);

    B.fall(&encoderBfall);
    B.rise(&encoderBrise);
    while (true) {
    }
}

