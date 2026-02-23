#include "mbed.h"
#include <cstdio>

// IO Port Configuration
DigitalOut MDIR(PF_2);
DigitalOut MBRAKE(PG_1);
PwmOut pwm0(PE_9); // PE_9 at CN10 configured as PWM out with alias pwm0


Ticker PeriodicInt; // Dclare a periodic timer for interrupt generation


// Function prototypes
void ExtInterruptISR(void);
void ExtInterruptThread(void const *n);
// void WatchdogISR(void const *n);
// void WatchdogThread(void const *argument);
void PeriodicInterruptISR(void);
void PeriodicInterruptThread(void const *argument);


// Processes and threads
osThreadId PeriodicInterruptId;
osThreadDef(PeriodicInterruptThread, osPriorityRealtime, 1024); // Declare PeriodicInterruptThread as a thread/process


//Declare global variables
bool dir=0, brake=0;
static UnbufferedSerial pc(USBTX,USBRX);
int u=0;

int period=500;
int maxDuty = 0.4*period;
int duty = 0;

// main() runs in its own thread in the OS
int main()
{
    // Start execution of: PeriodicInterruptThread with ID, PeriodicInterruptId:
    PeriodicInterruptId = osThreadCreate(osThread(PeriodicInterruptThread), NULL);
    // Start periodic interrupt generaion, specifying the period, and address of the isr.
    PeriodicInt.attach(&PeriodicInterruptISR, 250ms);
    pwm0.period_us(period); // This sets the PWM period to 5000 us.

    pwm0.pulsewidth_us(duty);
    MBRAKE = brake;

    while (true) {
        int newU=0;
        scanf("\n%d", &newU);
        printf(">>>>%d<<<\n", newU);

        if(abs(newU) >= abs(u))
        {
            u = newU;
        }
        wait_us(500*1000);
    }
}

// ******** Periodic Timer Interrupt Thread ********
void PeriodicInterruptThread(void const *argument) {
    while (true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalPi, is received.

        // if(bb == 'd')
        // {
        //     dir = !dir;
        // }
        // if(bb == ' ')
        // {
        //     pwm = pwm + 0.005*period;
        // }

        if(u<0)dir = 0;
        else dir = 1;

        MDIR = dir;

        float dutyf = (float)abs(u) / 100;
        duty = dutyf*maxDuty;
        printf("duty: %d\n", duty);
        pwm0.pulsewidth_us(duty);
    } 
}

// ******** Period Timer Interrupt Handler ********
void PeriodicInterruptISR(void) {
    osSignalSet(PeriodicInterruptId,0x1); // Send signal to the thread with ID, PeriodicInterruptId, i.e., PeriodicInterruptThread.
}

