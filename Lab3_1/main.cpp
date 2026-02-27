#include "mbed.h"
#include <cstdint>
#include <cstdio>

// Function prototypes
void PiControlThread(void const *argument);
void PeriodicInterruptISR(void);
void ExtCollisionThread(void const *argument);
void ExtInterruptISR(void);
void WatchdogThread(void const *argument);
void WatchdogISR(void const *n);


// Processes and threads
osThreadId WatchdogId, ExtCollisionId, PiControlId;     // Thread ID's
osThreadDef(PiControlThread, osPriorityRealtime, 1024); // Declare PiControlThread as a thread
osThreadDef(ExtCollisionThread, osPriorityHigh, 1024);  // Declare ExtCollisionThread as a thread
osThreadDef(WatchdogThread, osPriorityRealtime, 1024);  // Declare WatchdogThread as a thread
osTimerDef(Wdtimer, WatchdogISR);   

// IO Port Configuration
DigitalOut MDIR(PF_2);
DigitalOut MBRAKE(PG_1);
PwmOut pwm0(PE_9); // PE_9 at CN10 configured as PWM out with alias pwm0

DigitalOut IoReset(PG_2);
DigitalOut SpiReset(PG_3);                    // Declare a watchdog timer

Mutex mPg;

//Variables
bool dir=0, brake=0;

uint8_t ID;

int period=500;
int maxDuty = 0.4*period;
int duty = 0;


Ticker PeriodicInt;

//PI controller vars
float kp = 5, ki = 1;
float currentVel=0;
float idealVel = 2 * 3.1415;
float e;
int stepsPerRotation = 1216;
int dP0, dT0, dP1, dT1;

//COMs
UnbufferedSerial pc(USBTX, USBRX);
SPI FPGA(PB_5, PB_4, PB_3);


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
    PiControlId = osThreadCreate(osThread(PiControlThread), NULL);
    // Start periodic interrupt generaion, specifying the period, and address of the isr.
    PeriodicInt.attach(&PeriodicInterruptISR, 50ms);

    FPGA.format(16,1); // SPI format: 16-bit words, mode 1
    FPGA.frequency(500000);

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
        float cvel, idvel, err;
        mPg.lock();
        cvel = currentVel;
        idvel = idealVel;
        err = e;
        mPg.unlock();
        // printf("C: %d I: %d\n", (int)(cvel * 100), (int)(idvel * 100));
        printf("D: %d\n", dP0);
        printf("e: %d\n", (int)(err*100));
        wait_us(500000);
    }
}

// ******** Periodic Timer Interrupt Thread ********
void PiControlThread(void const *argument) {
    // int side=0, newSide=0;
    float integration=0, propotion=0;

    // float posDeg=0;
    // int currentPosition=0;
    uint8_t Dummy=0;

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
        // currentPosition += dP0;

        // Velocity estimate 
        // currentVel = 10000 * ((float)dP0) / ((float)dT0); // in units of counts/10.24 μs OR
        mPg.lock();
        currentVel = 1000000 * 2 * 3.1415 * (float)dP0/((float)dT0*(float)stepsPerRotation*10.24); // in rad/s

        e = idealVel - currentVel;
        mPg.unlock();

        //position estimate
        // posDeg = currentPosition * 360.0f / (float)stepsPerRotation;

        // float deriv = kd*Vel0/abs(idealAngle-posDeg);
        // printf("\n%d\n", (int)deriv);

        // if(newSide == side) 
        integration = ki*(integration + e);
        propotion = kp * e;

        if(abs(integration) > maxDuty)
        {
            integration = maxDuty * integration/abs(integration);
        }
        // else integration = 0;

        
        duty = integration + propotion;

        if(abs(duty) > maxDuty)
        {
            duty = maxDuty * duty/abs(duty);
        }

        dir = duty <= 0;
        MDIR = dir;
        pwm0.pulsewidth_us(abs(duty));
        // side = newSide;
    } 
}

// ******** Period Timer Interrupt Handler ********
void PeriodicInterruptISR(void) {
    osSignalSet(PiControlId,0x1); // Activate the signal, PiControl, with each periodic timer interrupt.
}


