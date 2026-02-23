#include "mbed.h"

// main() runs in its own thread in the OS
int main()
{
    bool dir=0, brake=0;
    static UnbufferedSerial pc(USBTX,USBRX);
    char bb = ' ';

    int period=500;
    int pwm = 0;

    DigitalOut MDIR(PF_2);
    DigitalOut MBRAKE(PG_1);

    PwmOut pwm0(PE_9); // PE_9 at CN10 configured as PWM out with alias pwm0
    pwm0.period_us(period); // This sets the PWM period to 5000 us.

    pwm0.pulsewidth_us(pwm);
    MDIR = dir;
    MBRAKE = brake;


    while (true) {
        // HAL_Delay(100);
        pc.read(&bb, 1);

        if(bb == 'd')
        {
            dir = !dir;
        }
        if(bb == ' ')
        {
            pwm = pwm + 0.005*period;
        }
        
        MDIR = dir;
        MBRAKE = brake;

        if(pwm > 0.4*period)
        {
            pwm = 0;
        }
        pwm0.pulsewidth_us(pwm);
    }
}

