#include "mbed.h"

// main() runs in its own thread in the OS
int main()
{
    bool z, y;
    char x;
    static UnbufferedSerial pc(USBTX,USBRX);
    DigitalIn IPF2(PF_2); // alias for PF_2 at at CN9
    DigitalOut OPG1(PG_1); // alias for PG_0 at CN9
    z=1;
    do {
        OPG1=z; // Write to output port
        z=!z;
        y=IPF2; // Read input port
        printf("\r\n z=%x, IPF2=%x.", z, y);
        printf("\r\nPress space bar to continue - any other key to quit:");
        if(pc.read(&x,1)) pc.write(&x,1); // Echo keypress at USB consol
    }
    while (x==0x20);

}

