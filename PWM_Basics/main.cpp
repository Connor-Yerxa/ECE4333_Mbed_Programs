#include "mbed.h"

// main() runs in its own thread in the OS
int main()
{
    char x=0;
    int y=0;
    static UnbufferedSerial pc(USBTX,USBRX);

    PwmOut pwm0(PE_9); // PE_9 at CN10 configured as PWM out with alias pwm0
    pwm0.period_us(5000); // This sets the PWM period to 5000 us.

    // pwm0.pulsewidth_us(x); // Sets the PWM pulsewidth to abs(u) us.



    while(true)
    {  
        // y=((int) x)*1000;
        pwm0.pulsewidth_us(y++); // Sets the PWM pulsewidth to abs(u) us.
        // if(pc.read(&x,1)) pc.write(&x,1); // Echo keypress at USB consol
        if (y>5000) y=0;
        osDelay(10);
    }
}
