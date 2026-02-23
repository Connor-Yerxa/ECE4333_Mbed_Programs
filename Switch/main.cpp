#include "mbed.h"

// main() runs in its own thread in the OS
int main()
{
    DigitalIn IPF2(PF_2);
    while (true) {
        if(IPF2)
        {
            printf("AHHHHHHH.\r\n");
            while(IPF2){}
        }
    }
}

