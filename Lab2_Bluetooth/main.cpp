#include "mbed.h"
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>


Ticker PeriodicInt; // Dclare a periodic timer for interrupt generation

// void WatchdogThread(void const *argument);
void PeriodicInterruptISR(void);
void PeriodicInterruptThread(void const *argument);


// Processes and threads
osThreadId PeriodicInterruptId;
osThreadDef(PeriodicInterruptThread, osPriorityRealtime, 1024); // Declare PeriodicInterruptThread as a thread/process

// IO Port Configuration
DigitalOut MDIR(PF_2);
DigitalOut MBRAKE(PG_1);
PwmOut pwm0(PE_9); // PE_9 at CN10 configured as PWM out with alias pwm0

SPI FPGA(PB_5, PB_4, PB_3);
DigitalOut IoReset(PG_2);
DigitalOut SpiReset(PG_3);

//Declare global variables
bool dir=0, brake=0;
static UnbufferedSerial bluetooth(PB_13,PB_12);
static UnbufferedSerial pc(USBTX,USBRX);
int u=0;

uint8_t ID;

int period=500;
int maxDuty = 0.4*period;
int duty = 0;

int Pos0=0;
int N=64;

int dP0, dT0, dP1, dT1;
float PosInRads, Vel0=0;
float posDeg=0;
uint8_t Dummy=0;

// float idealVel = 300;
float idealAngle = 0; //degrees 1=-180, 2=-90, 3=0, 4=90, 5=180
int stepsPerRotation = 1216;

//PI controller vars
float kp = 0.07, ki = 0.1;

void ResetFPGA()
{
    IoReset = 0; 
    wait_us(5); 
    IoReset = 1;
    wait_us(5);
    IoReset = 0;
} 

void ResetFPGA_SPI()
{
    SpiReset = 0;
    SpiReset = 1;
    wait_us(5);
    SpiReset = 0;
}

void init()
{
    // Start execution of: PeriodicInterruptThread with ID, PeriodicInterruptId:
    PeriodicInterruptId = osThreadCreate(osThread(PeriodicInterruptThread), NULL);
    // Start periodic interrupt generaion, specifying the period, and address of the isr.
    PeriodicInt.attach(&PeriodicInterruptISR, 100ms);

    FPGA.format(16,1); // SPI format: 16-bit words, mode 1
    FPGA.frequency(500000);

    bluetooth.baud(9600);

    ResetFPGA();
    ResetFPGA_SPI();
    
    ID = FPGA.write(0x8004); // ID of SPI slave is returned as 0x0017

    pwm0.period_us(period); // This sets the PWM period to 500 us.

    pwm0.pulsewidth_us(duty);
    MDIR = dir;
    MBRAKE = brake;
}

// main() runs in its own thread in the OS
int main()
{
    init();
    while (true) {
        char n = '\n';
        // printf("%d\n", dP0);
        // in units of counts OR 
        // PosInRads = 2 * 3.1415 * Pos0/(4*N); // in units of rads

        // printf("Pr: %d \t", (int)posDeg);
        // printf("Pi: %d \t", (int)idealAngle);
        
        // printf("p: %d \t", (int)(kp*100));
        // printf("i: %d \t", (int)(ki*100));
        // // printf("")
        // printf("D: %d\n", duty);

        char d='\n';
        bluetooth.write(&d, 1);

        char c;
        float scaler=10;
        if(bluetooth.readable())
        {
            if(bluetooth.read(&c,1))
            {bluetooth.write(&c, 1);
            printf("Got data!\n");
                switch(c)
                {
                    case '1': idealAngle = -180;
                    break;
                    case '2': idealAngle = -90;
                    break;
                    case '3': idealAngle = 0;
                    break;
                    case '4': idealAngle = 90;
                    break;
                    case '5': idealAngle = 180;
                    break;
                }
            }
        }

        wait_us(100000);
    }
}


// ******** Periodic Timer Interrupt Thread ********
void PeriodicInterruptThread(void const *argument) {
    int side=0, newSide=0;
    float integration=0;
    while (true) {
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal, SignalPi, is received.

        // May be executed in a loop or periodically
        dP0 = FPGA.write(Dummy); // Read QEI-0 position register 
        dT0 = FPGA.write(Dummy); // Read QEI-0 time interval
        dP1 = FPGA.write(Dummy); // Read QEI-1 position register 
        dT1 = FPGA.write(Dummy); // Read QEI-1 time interval
        // Sign extened 16-bit word to 32-bit int
        if (dP0 & 0x00008000) {
        dP0 = dP0 | 0xFFFF0000;
        // Units of dP0 are in ‘counts’ (assume 4× counting)
        // 1 count represents 2π/(4*N) rad
        }
        // Accumulated position 
        Pos0 += dP0;

        // Velocity estimate 
        Vel0 = 10000 * ((float)dP0) / ((float)dT0); // in units of counts/10.24 μs OR
        // Vel0 = 100 * 2 * 3.1415 * (float)dP0/((float)dT0*4*(float)N*10.24); // in rad/s

        //position estimate
        posDeg = Pos0 * 360.0f / (float)stepsPerRotation;
        newSide = idealAngle - posDeg >= 0;

        // float deriv = kd*Vel0/abs(idealAngle-posDeg);
        // printf("\n%d\n", (int)deriv);

        if(newSide == side) 
        integration = duty + ki * (idealAngle - posDeg)/abs(idealAngle - posDeg);
        else integration = 0;

        
        duty = integration + kp * (idealAngle - posDeg);

        if(abs(duty) > 0.4*period)
        {
            duty = 0.4*period * duty/abs(duty);
        }

        dir = duty <= 0;
        MDIR = dir;
        pwm0.pulsewidth_us(abs(duty));
        side = newSide;
    } 
}

// ******** Period Timer Interrupt Handler ********
void PeriodicInterruptISR(void) {
    osSignalSet(PeriodicInterruptId,0x1); // Send signal to the thread with ID, PeriodicInterruptId, i.e., PeriodicInterruptThread.
}
